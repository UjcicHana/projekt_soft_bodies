//
// Created by hana on 1/19/26.
//

#include "SimulatedObject.h"

#include <iostream>
#include <set>

Eigen::Matrix3f rotationX(float angleRadians)
{
    float c = std::cos(angleRadians);
    float s = std::sin(angleRadians);

    Eigen::Matrix3f R;
    R << 1,  0, 0,
         0,  c, -s,
         0,  s,  c;
    return R;
}

void SimulatedObject::initializeFromObject(
    const Object& obj,
    float ground,
    float mass,
    const Eigen::Vector3f& initialVelocity,
    const Eigen::Vector3f& initialTranslation,
    AlgorithmType algorithmType) {
    simulation.algorithmType = algorithmType;

    clearState();

    createParticlesFromObject(
        obj,
        mass,
        initialVelocity,
        initialTranslation
    );

    faces = obj.getFaces();

    buildConstraints(ground);

    calculateForces(simulation.externalAcceleration);
}

void SimulatedObject::generateDistanceConstraints(float stiffness, float compliance, float dt) {

    std::set<std::pair<int,int>> edges;

    for (auto& f : faces) {
        int ids[3] = { f.x(), f.y(), f.z() };

        for (int i = 0; i < 3; ++i) {
            int a = ids[i];
            int b = ids[(i + 1) % 3];
            if (a > b) std::swap(a, b);
            edges.insert({a, b});
        }
    }

    for (auto& e : edges) {
        distanceConstraints.push_back(
            std::make_shared<DistanceConstraint>(
                particles[e.first], particles[e.second],
                stiffness, compliance, dt));
    }
}

void SimulatedObject::generateBendingConstraints(
    float stiffness,
    float compliance,
    float dt)
{
    isometricBendingConstraints.clear();

    struct EdgeInfo {
        int face;
        int opposite;
    };

    std::map<std::pair<int, int>, std::vector<EdgeInfo>> edgeMap;

    for (int fi = 0; fi < faces.size(); ++fi)
    {
        const auto& f = faces[fi];

        int ids[3] = { f[0], f[1], f[2] };

        for (int i = 0; i < 3; ++i)
        {
            int a = ids[i];
            int b = ids[(i + 1) % 3];
            int opp = ids[(i + 2) % 3];

            std::pair<int, int> key =
                std::minmax(a, b);

            edgeMap[key].push_back({ fi, opp });
        }
    }

    for (const auto& [edge, adjacent] : edgeMap)
    {
        if (adjacent.size() != 2)
            continue;

        int p0 = edge.first;
        int p1 = edge.second;
        int p2 = adjacent[0].opposite;
        int p3 = adjacent[1].opposite;

        if (p2 == p3 || p2 == p0 || p2 == p1 || p3 == p0 || p3 == p1)
            continue;

        isometricBendingConstraints.push_back(
            std::make_shared<IsometricBendingConstraint>(
                particles[p0],
                particles[p1],
                particles[p2],
                particles[p3],
                stiffness,
                compliance,
                dt
            )
        );
    }
}


void SimulatedObject::generateCollisionConstraints(float ground) {
    collisionConstraints.clear();
    Eigen::Vector3f normal(0, 1, 0);

    for (const auto& p : particles) {
        collisionConstraints.push_back(
            std::make_shared<EnvironmentalCollisionConstraint>(p, normal, ground));
    }
}

void SimulatedObject::generateClothFixedPointConstraints()
{
    if (particles.empty())
        return;

    int upperLeft  = 0;
    int upperRight = 0;

    for (int i = 1; i < particles.size(); ++i)
    {
        const auto& p = particles[i]->p;

        const auto& left  = particles[upperLeft]->p;
        const auto& right = particles[upperRight]->p;

        // Upper-left:
        // prioritize larger Y
        // tie-break smaller X
        if (
            p.y() > left.y() ||
            (std::abs(p.y() - left.y()) < 1e-6f &&
             p.x() < left.x())
        )
        {
            upperLeft = i;
        }

        // Upper-right:
        // prioritize larger Y
        // tie-break larger X
        if (
            p.y() > right.y() ||
            (std::abs(p.y() - right.y()) < 1e-6f &&
             p.x() > right.x())
        )
        {
            upperRight = i;
        }
    }

    fixedPointConstraints.push_back(std::make_shared<FixedPointConstraint>
        (particles[upperLeft], initialPositions[upperLeft]));

    fixedPointConstraints.push_back(std::make_shared<FixedPointConstraint>
        (particles[upperRight], initialPositions[upperRight]));

    std::cout << "Upper Left  = " << upperLeft << std::endl;
    std::cout << "Upper Right = " << upperRight << std::endl;


}

void SimulatedObject::generateContinuumTriangleConstraints(
    float stiffness,
    float compliance,
    float dt,
    float youngsModulus,
    float poissonRatio)
{
    continuumTriangleConstraints.clear();

    for (const auto& f : faces)
    {
        int i0 = f[0];
        int i1 = f[1];
        int i2 = f[2];

        continuumTriangleConstraints.push_back(
            std::make_shared<ContinuumTriangleConstraint>(
                particles[i0],
                particles[i1],
                particles[i2],
                stiffness,
                compliance,
                dt,
                youngsModulus,
                poissonRatio
            )
        );
    }
}

void SimulatedObject::calculateForces(const Eigen::Vector3f& externalAcceleration) const
{
    for (const auto& particle : particles)
    {
        particle->F =
            particle->m * externalAcceleration;
    }
}

void SimulatedObject::reset() {
    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i]->x = initialPositions[i];
        particles[i]->p = particles[i]->x;

        particles[i]->v.setZero();
        particles[i]->F.setZero();
    }

    calculateForces(simulation.externalAcceleration);
    resetLambdaConstraints();
}

void SimulatedObject::resetConstraints()
{
    fixedPointConstraints.clear();
    distanceConstraints.clear();
    continuumTriangleConstraints.clear();
    isometricBendingConstraints.clear();
    volumeConstraints.clear();
    collisionConstraints.clear();
}

void SimulatedObject::resetLambdaConstraints() {
    for (const auto& c : fixedPointConstraints) {
        c->lambda = 0.0f;
    }
    for (const auto& c : distanceConstraints) {
        c->lambda = 0.0f;
    }
    for (const auto& c : continuumTriangleConstraints) {
        c->lambda = 0.0f;
    }
    for (const auto& c : isometricBendingConstraints) {
        c->lambda = 0.0f;
    }
    for (const auto& c : volumeConstraints) {
        c->lambda = 0.0f;
    }
    for (const auto& c : collisionConstraints) {
        c->lambda = 0.0f;
    }
}

void SimulatedObject::projectConstraints() {
    for (const auto& c : fixedPointConstraints) {
        c->solve(simulation.algorithmType);
    }
    for (const auto& c : distanceConstraints) {
        c->solve(simulation.algorithmType);
    }
    for (const auto& c : continuumTriangleConstraints) {
        c->solve(simulation.algorithmType);
    }
    for (const auto& c : isometricBendingConstraints) {
        c->solve(simulation.algorithmType);
    }
    for (const auto& c : volumeConstraints) {
        c->solve(simulation.algorithmType);
    }
    for (const auto& c : collisionConstraints) {
        c->solve(simulation.algorithmType);
    }
}

void SimulatedObject::clearState()
{
    particles.clear();
    initialPositions.clear();
    faces.clear();

    resetConstraints();
}

void SimulatedObject::createParticlesFromObject(
    const Object& obj,
    float mass,
    const Eigen::Vector3f& initialVelocity,
    const Eigen::Vector3f& initialTranslation)
{
    float angleDeg = 0.0f;
    float angleRad = static_cast<float>(angleDeg * M_PI / 180.0f);

    Eigen::Matrix3f R = rotationX(angleRad);

    Eigen::Vector3f center = Eigen::Vector3f::Zero();

    for (const auto& v : obj.getVertices()) {
        center += v;
    }

    center /= static_cast<float>(obj.getVertices().size());

    for (const auto& vertex : obj.getVertices())
    {
        auto particle = std::make_shared<Particle>();

        Eigen::Vector3f rotated =
            R * (vertex - center) + center;

        particle->x = rotated + initialTranslation;
        particle->p = particle->x;

        particle->v = initialVelocity;
        particle->F.setZero();

        particle->m = mass;
        particle->w = particle->m > 0.0f
            ? 1.0f / particle->m
            : 0.0f;

        particles.push_back(particle);
        initialPositions.push_back(particle->p);
    }
}

void SimulatedObject::buildConstraints(float ground)
{
    resetConstraints();

    if (config.useDistance)
        generateDistanceConstraints(
            material.distanceStiffness,
            material.distanceCompliance,
            simulation.timeStep);

    if (config.useContinuumTriangle)
        generateContinuumTriangleConstraints(
            material.continuumStiffness,
            material.continuumCompliance,
            simulation.timeStep,
            material.youngsModulus,
            material.poissonRatio);

    if (config.useBending)
        generateBendingConstraints(
            material.bendingStiffness,
            material.bendingCompliance,
            simulation.timeStep);

    if (config.useVolume)
        volumeConstraints.push_back(
            std::make_shared<VolumeConstraint>(
                particles,
                faces,
                material.volumeStiffness,
                material.volumeCompliance,
                simulation.timeStep));

    if (config.useFixedPoints)
        generateClothFixedPointConstraints();

    if (config.useGroundCollision)
        generateCollisionConstraints(ground);

    // Optional single fixed point test
    /*
    fixedPointConstraints.push_back(
        std::make_shared<FixedPointConstraint>(
            particles[0],
            initialPositions[0]
        )
    );
    */
}
