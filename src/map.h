// vim: et ts=4 sts=4 sw=4
#pragma once
#include "rmw.h"
#include "mesh.h"
#include <btBulletDynamicsCommon.h>


inline glm::vec3 to_glm(const btVector3& v) { return glm::vec3(v.x(), v.y(), v.z()); }
inline btVector3 to_bt(const glm::vec3& v) { return btVector3(v.x, v.y, v.z); }


struct Model {
    rmw::VertexBuffer::Ptr vb;
    rmw::IndexBuffer::Ptr  ib;
    rmw::VertexArray::Ptr  va;
    glm::mat4              transform;
    glm::vec3              color;
};


class GameObject {
public:
    const Model& get_model() const { return m_model; }
    virtual void pick(const glm::vec3& pos, const glm::vec3& normal) {}

protected:
    Model m_model;
};


class Map : public GameObject {
public:
    void init(btDynamicsWorld* world);

private:
    Mesh                                        m_mesh;
    std::unique_ptr<btBvhTriangleMeshShape>     m_shape;
    std::unique_ptr<btTriangleIndexVertexArray> m_interface;
    std::unique_ptr<btRigidBody>                m_rigid_body;
};


class Kart : public GameObject {
public:
    void init(btDynamicsWorld* world);
    void update();
    void tick();
    void debug_draw();
    void pick(const glm::vec3& pos, const glm::vec3& normal) override;

private:
    glm::vec3                             m_size;
    std::unique_ptr<btBoxShape>           m_shape;
    std::unique_ptr<btDefaultMotionState> m_motion_state;
    std::unique_ptr<btRigidBody>          m_rigid_body;
    btDynamicsWorld*                      m_world;
};
