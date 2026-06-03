//
// Created by hana on 6/3/26.
//

#ifndef PROJEKT_SOFT_BODIES_SCENE_H
#define PROJEKT_SOFT_BODIES_SCENE_H

#include "Solver.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"


class Scene {
public:
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

    virtual ~Scene() = default;

    [[nodiscard]] virtual std::string name() const = 0;
    virtual void setup(Solver& solver) = 0;
    virtual void drawUI() = 0;
    virtual void reset(Solver& solver) = 0;
};

class BasicCollisionScene : public Scene {
public:
    [[nodiscard]] std::string name() const override;
    void setup(Solver& solver) override;
    void drawUI() override;
    void reset(Solver& solver) override;

};


#endif //PROJEKT_SOFT_BODIES_SCENE_H
