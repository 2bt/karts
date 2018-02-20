// vim: et ts=4 sts=4 sw=4
#pragma once
#include "rmw.h"
#include "mesh.h"
#include <LinearMath/btVector3.h>


inline glm::vec3 to_glm(const btVector3& v) { return glm::vec3(v.x(), v.y(), v.z()); }
inline btVector3 to_bt(const glm::vec3& v) { return btVector3(v.x, v.y, v.z); }


struct Model {
    rmw::VertexBuffer::Ptr vb;
    rmw::IndexBuffer::Ptr  ib;
    rmw::VertexArray::Ptr  va;
    glm::mat4              transform;
    glm::vec3              color;

    void init(const Mesh& mesh) {
        vb = rmw::context.create_vertex_buffer(rmw::BufferHint::StreamDraw);
        ib = rmw::context.create_index_buffer(rmw::BufferHint::StreamDraw);
        va = rmw::context.create_vertex_array();
        va->set_primitive_type(rmw::PrimitiveType::Triangles);
        va->set_attribute(0, vb, rmw::ComponentType::Float, 3, false, 0, 24);
        va->set_attribute(1, vb, rmw::ComponentType::Float, 3, false, 12, 24);
        va->set_index_buffer(ib);
        vb->init_data(mesh.vertices);
        ib->init_data(mesh.indices);
        va->set_count(mesh.indices.size());
    }
};


class GameObject {
public:
    const Model& get_model() const { return m_model; }
    virtual void pick(const glm::vec3& pos, const glm::vec3& normal) {}

protected:
    Model m_model;
};


