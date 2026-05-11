//
// Created by hana on 1/28/26.
//

#ifndef RENDERER_H
#define RENDERER_H

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Particle.h"

class Renderer {
public:
    Renderer();
    ~Renderer();
    void initShader();
    void initMesh(const std::vector<std::shared_ptr<Particle>>& particles,
    const std::vector<Eigen::Vector3i>& faces);
    void initGrid(float ground);
    void updateMeshFromParticles(const std::vector<std::shared_ptr<Particle>>& particles,
        const std::vector<Eigen::Vector3i>& faces);

    void draw(const Eigen::Matrix4f& mvp);
    void drawMesh(const Eigen::Matrix4f& mvp);
    void drawGrid(const Eigen::Matrix4f& mvp);

private:
    struct Vertex {
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
    };

    GLuint meshVAO, meshVBO;
    GLuint gridVAO, gridVBO;
    GLuint shader;

    size_t meshVertexCount;
    std::vector<Vertex> buffer;
    std::vector<Eigen::Vector3f> groundGridVertices;

};



#endif //RENDERER_H
