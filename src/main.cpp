#define GLFW_INCLUDE_NONE
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

#include "Solver.h"
#include "Renderer.h"
#include "Scene.h"

struct SimulationUI {
    bool runSimulation = false;
    bool resetSimulation = false;

    int solverIterations = 10;

    int selectedScene = 0;
    bool sceneChanged = false;
} ui;

void drawUI(Scene& scene)
{
    ImGui::SetNextWindowSize(ImVec2(390, 420), ImGuiCond_Once);
    ImGui::Begin("Simulation Controls");

    ImGui::PushItemWidth(180.0f);

    const char* sceneNames[] = {
        "Basic Collision",
        "Cloth Comparison",
        "Constraint Comparison"
    };

    int previousScene = ui.selectedScene;

    ImGui::Combo(
        "Scene",
        &ui.selectedScene,
        sceneNames,
        IM_ARRAYSIZE(sceneNames)
    );

    if (ui.selectedScene != previousScene) {
        ui.sceneChanged = true;
    }

    ImGui::Text("Scene: %s", scene.name().c_str());

    ImGui::Separator();

    ImGui::Text("Solver");

    ImGui::SliderInt(
        "Solver Iterations",
        &ui.solverIterations,
        1,
        40
    );

    scene.drawUI();

    ImGui::Separator();

    if (ui.runSimulation)
    {
        ImGui::PushStyleColor(
            ImGuiCol_Button,
            ImVec4(0.8f, 0.2f, 0.2f, 1.0f)
        );

        if (ImGui::Button("Stop Simulation"))
        {
            ui.runSimulation = false;
        }

        ImGui::PopStyleColor();
    }
    else
    {
        ImGui::PushStyleColor(
            ImGuiCol_Button,
            ImVec4(0.2f, 0.7f, 0.2f, 1.0f)
        );

        if (ImGui::Button("Run Simulation"))
        {
            ui.runSimulation = true;
        }

        ImGui::PopStyleColor();
    }

    ImGui::SameLine();

    if (ImGui::Button("Reset Simulation"))
    {
        ui.resetSimulation = true;
    }

    ImGui::PopItemWidth();
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


    std::vector<std::unique_ptr<Scene>> scenes;

    scenes.push_back(std::make_unique<BasicCollisionScene>());
    scenes.push_back(std::make_unique<ClothComparisonScene>());
    scenes.push_back(std::make_unique<ConstraintComparisonScene>());

    int currentSceneIndex = ui.selectedScene;
    Scene* currentScene = scenes[currentSceneIndex].get();

    Solver solver(ui.solverIterations, 1.0f / 120.0f);

    Renderer renderer;

    auto rebuildScene = [&]() {
        solver.clearObjects();

        currentScene = scenes[currentSceneIndex].get();
        currentScene->setup(solver);

        renderer.initGrid(currentScene->ground);
        renderer.initMesh(
            solver.getParticles(0),
            solver.getFaces(0)
        );
    };

    rebuildScene();


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

    int noSteps = 0;
    std::chrono::high_resolution_clock::time_point timeStart;
    bool timingStarted = false;

    while (!glfwWindowShouldClose(window)) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawUI(*currentScene);

        if (ui.sceneChanged)
        {
            currentSceneIndex = ui.selectedScene;
            rebuildScene();

            ui.runSimulation = false;
            ui.sceneChanged = false;
        }

        solver.setSolverIterations(ui.solverIterations);

        if (auto object = currentScene->getObject())
        {
            solver.setTimeStep(object->simulation.timeStep);
        }

        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (ui.runSimulation)
        {
            if (!timingStarted)
            {
                timeStart = std::chrono::high_resolution_clock::now();
                timingStarted = true;
            }

            solver.step();
            /*noSteps++;

            if (noSteps >= 120)
            {
                auto timeEnd = std::chrono::high_resolution_clock::now();

                float elapsedMs =
                    std::chrono::duration<float, std::milli>(timeEnd - timeStart).count();

                std::cout << "Elapsed time for 120 steps: " << elapsedMs << " ms\n";
                std::cout << "Average time per step: " << elapsedMs / 120.0f << " ms\n";
                std::cout << solver.getParticles(0).size() << " particles\n";

                ui.runSimulation = false;
                timingStarted = false;
            }*/
        }

        if (ui.resetSimulation)
        {
            rebuildScene();

            ui.resetSimulation = false;
            noSteps = 0;
            timingStarted = false;
        }

        renderer.updateMeshFromParticles(solver.getParticles(0), solver.getFaces(0));

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

