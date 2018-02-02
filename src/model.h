// vim: et ts=4 sts=4 sw=4
#pragma once
#include <vector>
#include <glm/glm.hpp>


class Model {
public:
    bool load(const char* name);
    struct Vertex {
        glm::vec3 p;
        glm::vec3 n;
    };
    // private:
    std::vector<Vertex> m_vertices;
    std::vector<int>    m_indices;
};
