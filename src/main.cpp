#define GLFW_INCLUDE_NONE
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "Object.h"
#include "Solver.h"
#include "Renderer.h"
#include "Scene.h"

struct SimulationUI {
    bool runSimulation = false;
    bool resetSimulation = false;

    float timeStep = 1.0f / 120.0f;
    int solverIterations = 10;

    AlgorithmType algorithmType = AlgorithmType::PBD;

    // PBD attributes
    float distanceStiffness = 0.7f;
    float volumeStiffness = 0.2f;

    // XPBD atributes
    float distanceCompliance = 5e-5f;
    float volumeCompliance = 5e-6f;

    float gravityX = 0;
    float gravityY = -9.8f;
} ui;

void drawUI() {
    ImGui::SetNextWindowSize(ImVec2(360, 350), ImGuiCond_Once);
    ImGui::Begin("Simulation Controls");

    ImGui::PushItemWidth(160.0f);

    ImGui::Separator();
    ImGui::Text("Time Integration");

    ImGui::SliderFloat("Time Step", &ui.timeStep, 0.0005f, 0.02f, "%.5f");

    ImGui::SliderInt("Solver Iterations", &ui.solverIterations, 1, 40);

    int mode = (ui.algorithmType == AlgorithmType::PBD) ? 0 : 1;

    ImGui::Text("Algorithm");

    ImGui::RadioButton("PBD", &mode, 0);
    ImGui::SameLine();
    ImGui::RadioButton("XPBD", &mode, 1);

    ui.algorithmType = (mode == 0) ? AlgorithmType::PBD : AlgorithmType::XPBD;

    ImGui::Separator();
    ImGui::Text("Soft Body Parameters");

    if (mode == 0) { // PBD
        ImGui::SliderFloat("Distance Stiffness", &ui.distanceStiffness, 0.0f, 1.0f);
        ImGui::SliderFloat("Volume Stiffness", &ui.volumeStiffness, 0.0f, 1.0f);
    } else {
        ImGui::SliderFloat("Distance Compliance", &ui.distanceCompliance, 0.0f, 1e-4f, "%.6f");
        ImGui::SliderFloat("Volume Compliance", &ui.volumeCompliance, 0.0f, 1e-5f, "%.7f");
    }

    ImGui::Separator();
    ImGui::Text("Gravity");

    ImGui::SliderFloat("Gravity X", &ui.gravityX, -50.0f, 20.0f);
    ImGui::SliderFloat("Gravity Y", &ui.gravityY, -50.0f, 20.0f);

    if (ImGui::Button("Reset Gravity")) {
        ui.gravityX = 0.0f;
        ui.gravityY = -9.8f;
    }

    ImGui::Separator();

    if (ui.runSimulation) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.2f, 0.2f, 1.0f)); // red
        if (ImGui::Button("Stop Simulation")) {
            ui.runSimulation = false;
        }
        ImGui::PopStyleColor();
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f)); // green
        if (ImGui::Button("Run Simulation")) {
            ui.runSimulation = true;
        }
        ImGui::PopStyleColor();
    }

    ImGui::SameLine();

    if (ImGui::Button("Reset Simulation")) {
        ui.resetSimulation = true;
    }

    ImGui::End();
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

    auto framebufferCallback = [](GLFWwindow*, int w, int h) {
        glViewport(0, 0, w, h);
    };

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

    if (argc < 2) {
        std::cerr << "Usage: ./program model.obj\n";
        return -1;
    }

    Object obj;
    if (!obj.loadObject(argv[1])) {
        std::cerr << "Failed to load OBJ\n";
        return -1;
    }
    float ground = -1.0f;

    Solver solver(ui.solverIterations, ui.timeStep);
    solver.addSoftBody(obj, ground);
    Renderer renderer = Renderer();
    renderer.initGrid(ground);
    renderer.initMesh(solver.getParticles(0), obj.getFaces());

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);

    float aspect = 800.0f / 600.0f;
    float fov = 60.0f * M_PI / 180.0f;
    float near = 0.1f, far = 100.0f;

    float f = 1.0f / tan(fov * 0.5f);

    Eigen::Matrix4f proj;
    proj << f/aspect, 0,  0,                          0,
            0,        f,  0,                          0,
            0,        0,  (far+near)/(near-far),     (2*far*near)/(near-far),
            0,        0, -1,                          0;

    while (!glfwWindowShouldClose(window)) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawUI();

        Eigen::Vector3f gravity = Eigen::Vector3f(ui.gravityX, ui.gravityY, 0);

        solver.setTimeStep(ui.timeStep);
        solver.setSolverIterations(ui.solverIterations);
        solver.setAlgorithmType(ui.algorithmType, 0);

        solver.setDistanceStiffness(ui.distanceStiffness, 0);
        solver.setVolumeStiffness(ui.volumeStiffness, 0);

        solver.setDistanceCompliance(ui.distanceCompliance, 0);
        solver.setVolumeCompliance(ui.volumeCompliance, 0);

        solver.setOutsideForces(gravity, 0);

        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (ui.runSimulation) {
            solver.step();
        }

        if (ui.resetSimulation) {
            solver.resetSimulation();
            ui.resetSimulation = false;
        }

        renderer.updateMeshFromParticles(solver.getParticles(0), obj.getFaces());

        Eigen::Affine3f view = Eigen::Affine3f::Identity();
        view.translate(Eigen::Vector3f(0, 0, -4));
        Eigen::Matrix4f mvp = proj * view.matrix();


        renderer.draw(mvp);

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

