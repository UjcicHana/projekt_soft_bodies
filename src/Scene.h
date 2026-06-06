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
    virtual ~Scene() = default;

    [[nodiscard]] virtual std::string name() const = 0;
    virtual void setup(Solver& solver) = 0;
    virtual void drawUI() = 0;
    virtual void reset(Solver& solver) = 0;

    [[nodiscard]] virtual std::shared_ptr<SimulatedObject> getObject() const = 0;

    float ground = -1.0f;
    float mass = 1.0f;

    Eigen::Vector3f initialTranslation =
        Eigen::Vector3f(1.0f, 1.5f, 0.0f);

    Eigen::Vector3f initialVelocity =
        Eigen::Vector3f::Zero();
};

class BasicCollisionScene final : public Scene {
public:
    BasicCollisionScene() {
        object.loadObject(objFile);
    }

    [[nodiscard]] std::string name() const override {
        return "Basic Collision";
    }

    void setup(Solver& solver) override;
    void reset(Solver& solver) override;
    void drawUI() override;

    [[nodiscard]] std::shared_ptr<SimulatedObject> getObject() const override {
        return simulatedObject;
    }

    AlgorithmType algorithmType =
        AlgorithmType::PBD;

    float timeStep = 1.0f / 120.0f;

    float distanceStiffness = 0.7f;
    float volumeStiffness = 0.2f;

    float distanceCompliance = 5e-5f;
    float volumeCompliance = 5e-6f;

    Eigen::Vector3f externalAcceleration =
        Eigen::Vector3f(0.0f, -9.8f, 0.0f);

private:
    Object object;
    std::shared_ptr<SimulatedObject> simulatedObject;
    std::string objFile = "../models/cube_detailed.obj";
};

class ClothComparisonScene final : public Scene {
public:
    explicit ClothComparisonScene() {
        object.loadObject(objFile);
    }

    [[nodiscard]] std::string name() const override {
        return "Cloth PBD vs XPBD";
    }

    void setup(Solver& solver) override;
    void reset(Solver& solver) override;
    void drawUI() override;

    [[nodiscard]] std::shared_ptr<SimulatedObject> getObject() const override {
        return simulatedObject;
    }

    Eigen::Vector3f externalAcceleration =
        Eigen::Vector3f(0.0f, -9.8f, 0.0f);

    AlgorithmType algorithmType =
        AlgorithmType::PBD;

    float timeStep = 1.0f / 120.0f;

    bool useBending = true;
    bool useGroundCollision = false;

    float distanceStiffness = 0.8f;
    float bendingStiffness = 0.005f;

    float distanceCompliance = 5e-5f;
    float bendingCompliance = 1e-3f;

private:
    Object object;
    std::shared_ptr<SimulatedObject> simulatedObject;
    std::string objFile = "../models/cloth_10x10.obj";
};

#endif //PROJEKT_SOFT_BODIES_SCENE_H
