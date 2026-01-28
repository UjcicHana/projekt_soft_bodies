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
#include "Renderer.h"

struct SimulationUI {
    bool runSimulation = false;
    bool resetSimulation = false;

    float timeStep = 1.0f / 120.0f;
    int solverIterations = 10;

    float distanceStiffness = 0.7f;
    float volumeStiffness = 0.8f;
    float shapeMatchingStiffness = 0.5f;

    float gravityX = 0;
    float gravityY = -9.8f;
} ui;

void drawUI(PBD& pbd) {
    ImGui::SetNextWindowSize(ImVec2(360, 310), ImGuiCond_Once);
    ImGui::Begin("Simulation Controls");

    ImGui::PushItemWidth(160.0f);

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

    float dt = 1.0f / 120.0f;

    PBD pbd(obj, ground, dt);
    Renderer renderer = Renderer();
    renderer.initGrid(ground);
    renderer.initMesh(pbd.getParticles(), obj.getFaces());

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

        drawUI(pbd);

        Eigen::Vector3d gravity = Eigen::Vector3d(ui.gravityX, ui.gravityY, 0);

        pbd.setTimeStep(ui.timeStep);
        pbd.setSolverIterations(ui.solverIterations);
        pbd.setDistanceStiffness(ui.distanceStiffness);
        pbd.setVolumeStiffness(ui.volumeStiffness);
        pbd.setShapeMatchingStiffness(ui.shapeMatchingStiffness);
        pbd.setOutsideForces(gravity);

        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (ui.runSimulation) {
            pbd.step();
        }

        if (ui.resetSimulation) {
            pbd.resetSimulation(obj, ground);
            ui.resetSimulation = false;
        }

        renderer.updateMeshFromParticles(pbd.getParticles(), obj.getFaces());

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

