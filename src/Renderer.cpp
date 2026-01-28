//
// Created by hana on 1/28/26.
//

#include "Renderer.h"
#include <iostream>

Renderer::Renderer()
    : meshVAO(0), meshVBO(0), gridVAO(0), gridVBO(0), shader(0), meshVertexCount(0)
{
    initShader();
}

Renderer::~Renderer() {
    if (meshVBO) glDeleteBuffers(1, &meshVBO);
    if (meshVAO) glDeleteVertexArrays(1, &meshVAO);

    if (gridVBO) glDeleteBuffers(1, &gridVBO);
    if (gridVAO) glDeleteVertexArrays(1, &gridVAO);

    if (shader) glDeleteProgram(shader);
}


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

void computeVertexNormals(
    const std::vector<std::shared_ptr<Particle>>& particles,
    const std::vector<Eigen::Vector3i>& faces,
    std::vector<Eigen::Vector3f>& normals
) {
    normals.assign(particles.size(), Eigen::Vector3f::Zero());

    for (const auto& f : faces) {
        Eigen::Vector3f p0 = particles[f.x()]->x.cast<float>();
        Eigen::Vector3f p1 = particles[f.y()]->x.cast<float>();
        Eigen::Vector3f p2 = particles[f.z()]->x.cast<float>();

        Eigen::Vector3f n = (p1 - p0).cross(p2 - p0);

        normals[f.x()] += n;
        normals[f.y()] += n;
        normals[f.z()] += n;
    }

    for (auto& n : normals) {
        if (n.norm() > 1e-6f)
            n.normalize();
    }
}

void Renderer::initShader() {
    const char* vs = R"(
        #version 460 core

        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;

        uniform mat4 uMVP;
        uniform mat4 uModel;

        out vec3 vNormal;
        out vec3 vWorldPos;

        void main() {
            vWorldPos = (uModel * vec4(aPos, 1.0)).xyz;
            vNormal = normalize(mat3(uModel) * aNormal);
            gl_Position = uMVP * vec4(aPos, 1.0);
        }
    )";

    const char* fs = R"(
        #version 460 core

        in vec3 vNormal;
        in vec3 vWorldPos;

        out vec4 FragColor;

        uniform vec3 uLightDir;  // e.g. (0, -1, 0)
        uniform vec3 uColor;

        void main() {
            vec3 N = normalize(vNormal);
            vec3 L = normalize(-uLightDir);

            float diff = max(dot(N, L), 0.0);

            vec3 ambient = 0.2 * uColor;
            vec3 diffuse = diff * uColor;

            FragColor = vec4(ambient + diffuse, 1.0);
        }
    )";


    GLuint v = compileShader(vs, GL_VERTEX_SHADER);
    GLuint f = compileShader(fs, GL_FRAGMENT_SHADER);

    shader = glCreateProgram();
    glAttachShader(shader, v);
    glAttachShader(shader, f);
    glLinkProgram(shader);

    glDeleteShader(v);
    glDeleteShader(f);
}

void Renderer::initMesh(const std::vector<std::shared_ptr<Particle>>& particles,
    const std::vector<Eigen::Vector3i>& faces) {
    buffer.clear();
    buffer.reserve(faces.size() * 3);

    std::vector<Eigen::Vector3f> normals;
    computeVertexNormals(particles, faces, normals);

    buffer.clear();
    buffer.reserve(faces.size() * 3);

    for (const auto& f : faces) {
        buffer.push_back({ particles[f.x()]->x.cast<float>(), normals[f.x()] });
        buffer.push_back({ particles[f.y()]->x.cast<float>(), normals[f.y()] });
        buffer.push_back({ particles[f.z()]->x.cast<float>(), normals[f.z()] });
    }

    // for (size_t i = 0; i < vertexBuffer.size(); ++i) {
    //     std::cout << "Vertex " << i << " position: " << vertexBuffer[i].position.x() << " " << vertexBuffer[i].position.y() << " " << vertexBuffer[i].position.z() << std::endl;
    //     std::cout << "Vertex " << i << " normal: " << vertexBuffer[i].normal.x() << " " << vertexBuffer[i].normal.y() << " " << vertexBuffer[i].normal.z() << std::endl;
    // }

    meshVertexCount = buffer.size();

    glGenVertexArrays(1, &meshVAO);
    glGenBuffers(1, &meshVBO);

    glBindVertexArray(meshVAO);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);

    glBufferData(GL_ARRAY_BUFFER,
                 meshVertexCount * sizeof(Vertex),
                 buffer.data(),
                 GL_STATIC_DRAW);

    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE,
        sizeof(Vertex),
        (void*)offsetof(Vertex, position)
    );

    glEnableVertexAttribArray(1); // normal
    glVertexAttribPointer(
        1, 3, GL_FLOAT, GL_FALSE,
        sizeof(Vertex),
        (void*)offsetof(Vertex, normal)
    );

}
void Renderer::initGrid(float ground) {

    const float gridSize = 8.0f;
    const int gridLines = 24;
    const float step = (2.0f * gridSize) / gridLines;

    for (int i = 0; i <= gridLines; ++i) {
        float x = -gridSize + i * step;
        float z = -gridSize + i * step;

        groundGridVertices.emplace_back(x, ground, -gridSize);
        groundGridVertices.emplace_back(x, ground,  gridSize);

        groundGridVertices.emplace_back(-gridSize, ground, z);
        groundGridVertices.emplace_back( gridSize, ground, z);
    }

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
}
void Renderer::updateMeshFromParticles(const std::vector<std::shared_ptr<Particle>>& particles,
    const std::vector<Eigen::Vector3i>& faces) {
    buffer.clear();
    std::vector<Eigen::Vector3f> normals;

    computeVertexNormals(particles, faces, normals);

    for (const auto& f : faces) {
        buffer.push_back({ particles[f.x()]->x.cast<float>(), normals[f.x()] });
        buffer.push_back({ particles[f.y()]->x.cast<float>(), normals[f.y()] });
        buffer.push_back({ particles[f.z()]->x.cast<float>(), normals[f.z()] });
    }

    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glBufferSubData(
        GL_ARRAY_BUFFER,
        0,
        buffer.size() * sizeof(Vertex),
        buffer.data()
    );

}

void Renderer::draw(const Eigen::Matrix4f& mvp) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    glUseProgram(shader);

    glUniformMatrix4fv(
        glGetUniformLocation(shader, "uMVP"),
        1, GL_FALSE, mvp.data()
    );

    glUniformMatrix4fv(
        glGetUniformLocation(shader, "uModel"),
        1, GL_FALSE, model.data()
    );

    glUniform3f(
        glGetUniformLocation(shader, "uLightDir"),
        0.0f, -1.0f, -2.0f
    );

    drawGrid(mvp);
    drawMesh(mvp);
}

void Renderer::drawGrid(const Eigen::Matrix4f& mvp) {
    glUniform3f(glGetUniformLocation(shader, "uColor"), 1.0f, 1.0f, 1.0f);

    glBindVertexArray(gridVAO);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(groundGridVertices.size()));
}

void Renderer::drawMesh(const Eigen::Matrix4f &mvp) {
    glUniform3f(glGetUniformLocation(shader, "uColor"), 0.7f, 0.7f, 0.7f);

    glBindVertexArray(meshVAO);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(meshVertexCount));
}

