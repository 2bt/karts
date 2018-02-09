#pragma once
#include "rmw.h"
#include "mesh.h"
#include <btBulletDynamicsCommon.h>


struct Model {
    rmw::VertexBuffer::Ptr vb;
    rmw::IndexBuffer::Ptr  ib;
    rmw::VertexArray::Ptr  va;
    glm::mat4              transform;
    glm::vec3              color;
};


class Map {
public:
    void init();

    const Model& get_model() const { return m_model; }

    btRigidBody* get_rigid_body() const { return m_rigid_body.get(); }

private:
    Mesh                                        m_mesh;
    Model                                       m_model;
    std::unique_ptr<btBvhTriangleMeshShape>     m_shape;
    std::unique_ptr<btTriangleIndexVertexArray> m_interface;
    std::unique_ptr<btRigidBody>                m_rigid_body;
};


class Kart {
public:
    void init();

    void tick();

    const Model& get_model() {
        // XXX
        btTransform t;
        m_motion_state->getWorldTransform(t);
        t.getOpenGLMatrix(reinterpret_cast<float*>(&m_model.transform));
        return m_model;
    }

    btRigidBody* get_rigid_body() const { return m_rigid_body.get(); }

private:
    Model                                 m_model;
    glm::vec3                             m_size;
    std::unique_ptr<btBoxShape>           m_shape;
    std::unique_ptr<btDefaultMotionState> m_motion_state;
    std::unique_ptr<btRigidBody>          m_rigid_body;
};


