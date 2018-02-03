// vim: et ts=4 sts=4 sw=4
#pragma once
#include <vector>
#include <glm/glm.hpp>


struct Mesh {

    bool load(const char* name);

    struct Vertex {
        glm::vec3 p;
        glm::vec3 n;
    };

    std::vector<Vertex> vertices;
    std::vector<int>    indices;
};
