#define GLFW_INCLUDE_NONE
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "Object.h"
#include "PBD.h"

struct SimulationUI {
    bool runSimulation = false;

    float timeStep = 1.0f / 120.0f;
    int solverIterations = 10;

    float distanceStiffness = 0.7f;
    float volumeStiffness = 0.8f;
    float shapeMatchingStiffness = 0.5f;

    float gravityX = 0;
    float gravityY = -9.8f;
};

GLuint compileShader(const char* src, GLenum type) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        std::cerr << "Shader error: " << log << std::endl;
    }
    return shader;
}

GLuint createProgram(const char* vs, const char* fs) {
    GLuint v = compileShader(vs, GL_VERTEX_SHADER);
    GLuint f = compileShader(fs, GL_FRAGMENT_SHADER);

    GLuint program = glCreateProgram();
    glAttachShader(program, v);
    glAttachShader(program, f);
    glLinkProgram(program);

    glDeleteShader(v);
    glDeleteShader(f);

    return program;
}

void framebufferCallback(GLFWwindow*, int w, int h) {
    glViewport(0, 0, w, h);
}

int main(int argc, char* argv[]) {

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window =
        glfwCreateWindow(800, 600, "Soft body animation", nullptr, nullptr);

    if (!window) {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebufferCallback);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "GLAD initialization failed\n";
        return -1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
    SimulationUI ui;

    // ---------------------------
    // Load OBJ from argument
    // ---------------------------
    if (argc < 2) {
        std::cerr << "Usage: ./program model.obj\n";
        return -1;
    }

    Object obj;
    if (!obj.loadObject(argv[1])) {
        std::cerr << "Failed to load OBJ\n";
        return -1;
    }

    // ---------------------------
    // Convert mesh to flat vertex array
    // ---------------------------
    std::vector<Eigen::Vector3f> vertexBuffer;
    vertexBuffer.reserve(obj.getFaces().size() * 3);

    for (auto& f : obj.getFaces()) {
        auto vertices = obj.getVertices();
        vertexBuffer.emplace_back(vertices[f.x()].cast<float>());
        vertexBuffer.emplace_back(vertices[f.y()].cast<float>());
        vertexBuffer.emplace_back(vertices[f.z()].cast<float>());
    }

    size_t vertexCount = vertexBuffer.size();

    float ground = -1.0f;
    std::vector<Eigen::Vector3f> groundGridVertices;

    const float gridSize = 5.0f;
    const int gridLines = 20;     // number of lines per axis
    const float step = (2.0f * gridSize) / gridLines;

    for (int i = 0; i <= gridLines; ++i) {
        float x = -gridSize + i * step;
        float z = -gridSize + i * step;

        // Lines parallel to Z (vary X)
        groundGridVertices.emplace_back(x, ground, -gridSize);
        groundGridVertices.emplace_back(x, ground,  gridSize);

        // Lines parallel to X (vary Z)
        groundGridVertices.emplace_back(-gridSize, ground, z);
        groundGridVertices.emplace_back( gridSize, ground, z);
    }

    // ---------------------------
    // Shader setup
    // ---------------------------
    const char* vsSrc = R"(
        #version 460 core
        layout (location = 0) in vec3 aPos;
        uniform mat4 uMVP;
        void main() {
            gl_Position = uMVP * vec4(aPos, 1.0);
        }
    )";

    const char* fsSrc = R"(
        #version 460 core
        out vec4 FragColor;
        void main() {
            FragColor = vec4(0.7, 0.8, 1.0, 1.0);
        }
    )";

    GLuint shader = createProgram(vsSrc, fsSrc);

    // ---------------------------
    // GPU geometry upload
    // ---------------------------
    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER,
                 vertexCount * sizeof(Eigen::Vector3f),
                 vertexBuffer.data(),
                 GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Eigen::Vector3f),
        (void*)0
    );

    glEnable(GL_DEPTH_TEST);

    GLuint gridVAO, gridVBO;
    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);

    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);

    glBufferData(
        GL_ARRAY_BUFFER,
        groundGridVertices.size() * sizeof(Eigen::Vector3f),
        groundGridVertices.data(),
        GL_STATIC_DRAW
    );

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Eigen::Vector3f),
        (void*)0
    );

    glBindVertexArray(0);

    // ---------------------------
    // Projection matrix
    // ---------------------------
    float aspect = 800.0f / 600.0f;
    float fov = 60.0f * M_PI / 180.0f;
    float near = 0.1f, far = 100.0f;

    float f = 1.0f / tan(fov * 0.5f);

    Eigen::Matrix4f proj;
    proj << f/aspect, 0,  0,                          0,
            0,        f,  0,                          0,
            0,        0,  (far+near)/(near-far),     (2*far*near)/(near-far),
            0,        0, -1,                          0;

    float dt = 1.0f / 120.0f;

    PBD pbd(obj, ground, dt);

    auto updateVertexBufferFromParticles =
    [&](PBD& pbd)
    {
        const auto& particles = pbd.getParticles();

        size_t idx = 0;
        for (const auto& f : obj.getFaces()) {
            vertexBuffer[idx++] = particles[f.x()]->x.cast<float>();
            vertexBuffer[idx++] = particles[f.y()]->x.cast<float>();
            vertexBuffer[idx++] = particles[f.z()]->x.cast<float>();
        }

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(
            GL_ARRAY_BUFFER,
            0,
            vertexBuffer.size() * sizeof(Eigen::Vector3f),
            vertexBuffer.data()
        );
    };



    // ---------------------------
    // Main loop
    // ---------------------------
    while (!glfwWindowShouldClose(window)) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowSize(ImVec2(360, 350), ImGuiCond_Once);
        ImGui::Begin("Simulation Controls");

        ImGui::Checkbox("Run Simulation", &ui.runSimulation);

        ImGui::Separator();
        ImGui::Text("Time Integration");

        ImGui::SliderFloat("Time Step", &ui.timeStep, 0.0005f, 0.02f, "%.5f");

        ImGui::SliderInt("Solver Iterations", &ui.solverIterations, 1, 50);

        ImGui::Separator();
        ImGui::Text("Soft Body Parameters");

        ImGui::SliderFloat("Distance Stiffness", &ui.distanceStiffness, 0.0f, 1.0f);

        ImGui::SliderFloat("Volume Stiffness", &ui.volumeStiffness, 0.0f, 1.0f);

        ImGui::SliderFloat("Shape Matching Stiffness", &ui.shapeMatchingStiffness, 0.0f, 1.0f);

        ImGui::Separator();
        ImGui::Text("Gravity");

        ImGui::SliderFloat("Gravity X", &ui.gravityX, -50.0f, 20.0f);

        ImGui::SliderFloat("Gravity Y", &ui.gravityY, -50.0f, 20.0f);

        if (ImGui::Button("Reset Gravity")) {
            ui.gravityX = 0.0f;
            ui.gravityY = -9.8f;
        }

        ImGui::Separator();

        if (ImGui::Button("Reset Simulation")) {
            pbd.resetSimulation(obj, ground);
        }

        ImGui::End();

        Eigen::Vector3d gravity = Eigen::Vector3d(ui.gravityX, ui.gravityY, 0);

        pbd.setTimeStep(ui.timeStep);
        pbd.setSolverIterations(ui.solverIterations);
        pbd.setDistanceStiffness(ui.distanceStiffness);
        pbd.setVolumeStiffness(ui.volumeStiffness);
        pbd.setShapeMatchingStiffness(ui.shapeMatchingStiffness);
        pbd.setOutsideForces(gravity);

        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // --- Run physics ---
        if (ui.runSimulation) {
            pbd.step();
        }

        // --- Upload updated particle positions ---
        updateVertexBufferFromParticles(pbd);

        // --- Camera only ---
        Eigen::Affine3f view = Eigen::Affine3f::Identity();
        view.translate(Eigen::Vector3f(0, 0, -4));
        Eigen::Matrix4f mvp = proj * view.matrix();

        glUseProgram(shader);

        // MVP already computed
        GLint mvpLoc = glGetUniformLocation(shader, "uMVP");
        glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.data());

        // Grid color (subtle)
        GLint colorLoc = glGetUniformLocation(shader, "uColor");
        glUniform3f(colorLoc, 1.0f, 0.0f, 0.0f);

        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(groundGridVertices.size()));

        glUseProgram(shader);
        GLint loc = glGetUniformLocation(shader, "uMVP");
        glUniformMatrix4fv(loc, 1, GL_FALSE, mvp.data());

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertexCount));

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}

