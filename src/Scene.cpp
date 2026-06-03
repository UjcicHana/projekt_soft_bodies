//
// Created by hana on 6/3/26.
//

#include "Scene.h"

std::string BasicCollisionScene::name() const {
    return "Object ground collision";
}

void BasicCollisionScene::setup(Solver &solver) {
    Object obj;
    if (!obj.loadObject("../models/cube_detailed.obj")) {
        std::cerr << "Failed to load OBJ\n";
        return;
    }

    float ground = -1.0f;
    solver.addSoftBody(obj, ground);
}

void BasicCollisionScene::drawUI() {
    ImGui::SetNextWindowSize(ImVec2(360, 350), ImGuiCond_Once);
    ImGui::Begin("Simulation Controls");

    ImGui::PushItemWidth(160.0f);

    ImGui::Separator();
    ImGui::Text("Time Integration");

    ImGui::SliderFloat("Time Step", &ui.timeStep, 0.0005f, 0.02f, "%.5f");

    ImGui::SliderInt("Solver Iterations", &ui.solverIterations, 1, 50);

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

/*
 * struct SimulationUI {
    bool runSimulation = false;
    bool resetSimulation = false;

    float timeStep = 1.0f / 120.0f;
    int solverIterations = 10;

    AlgorithmType algorithmType = AlgorithmType::XPBD;

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

    ImGui::SliderInt("Solver Iterations", &ui.solverIterations, 1, 50);

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

 */
