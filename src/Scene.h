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

    float distanceStiffness = 0.1f;
    float volumeStiffness = 0.2f;

    float distanceCompliance = 5e-5f;
    float volumeCompliance = 5e-6f;

    Eigen::Vector3f externalAcceleration =
        Eigen::Vector3f(0.0f, -9.8f, 0.0f);

private:
    Object object;
    std::shared_ptr<SimulatedObject> simulatedObject;
    std::string objFile = "../models/cube_10.obj";
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

    float distanceStiffness = 0.1f;
    float bendingStiffness = 0.005f;

    float distanceCompliance = 5e-5f;
    float bendingCompliance = 1e-3f;

private:
    Object object;
    std::shared_ptr<SimulatedObject> simulatedObject;
    std::string objFile = "../models/cloth_10x10.obj";
};

class ConstraintComparisonScene final : public Scene {
public:
    ConstraintComparisonScene() = default;

    std::string name() const override {
        return "Constraint Ablation";
    }

    void setup(Solver& solver) override;
    void reset(Solver& solver) override;
    void drawUI() override;

    std::shared_ptr<SimulatedObject> getObject() const override {
        return simulatedObject;
    }

private:
    Object object;
    std::shared_ptr<SimulatedObject> simulatedObject;

    std::string objectPath = "../models/cube_10.obj";

public:
    float mass = 1.0f;

    Eigen::Vector3f initialTranslation =
        Eigen::Vector3f(1.0f, 1.5f, 0.0f);

    Eigen::Vector3f initialVelocity =
        Eigen::Vector3f::Zero();

    Eigen::Vector3f externalAcceleration =
        Eigen::Vector3f(0.0f, -9.8f, 0.0f);

    AlgorithmType algorithmType =
        AlgorithmType::PBD;

    float timeStep = 1.0f / 120.0f;

    bool useDistance = true;
    bool useVolume = true;
    bool useBending = false;
    bool useContinuumTriangle = false;
    bool useGroundCollision = true;

    float distanceStiffness = 0.7f;
    float volumeStiffness = 0.2f;
    float bendingStiffness = 0.005f;
    float continuumStiffness = 0.01f;

    float distanceCompliance = 5e-5f;
    float volumeCompliance = 5e-6f;
    float bendingCompliance = 1e-3f;
    float continuumCompliance = 1e-4f;

    float youngsModulus = 100.0f;
    float poissonRatio = 0.3f;
};

#endif //PROJEKT_SOFT_BODIES_SCENE_H
