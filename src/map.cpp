// vim: et ts=4 sts=4 sw=4
#include "map.h"


void Map::init(btDynamicsWorld* world) {
    // XXX: the mesh must be kept alive because the shape needs it
//    m_mesh.load("assets/hill.obj");
    m_mesh.load("assets/playground.obj");
    for (auto& v : m_mesh.vertices) v.p *= 3.0f;

    m_model.init(m_mesh);
    m_model.color = { 0.4, 0.6, 0.3 };

    m_interface = std::make_unique<btTriangleIndexVertexArray>(
            m_mesh.indices.size() / 3, m_mesh.indices.data(), sizeof(int) * 3,
            m_mesh.vertices.size(), (float*) m_mesh.vertices.data(), sizeof(Mesh::Vertex));
    m_shape = std::make_unique<btBvhTriangleMeshShape>(m_interface.get(), true);
    btRigidBody::btRigidBodyConstructionInfo info(0, nullptr, m_shape.get());
    info.m_startWorldTransform.setIdentity();
    m_rigid_body = std::make_unique<btRigidBody>(info);
    world->addRigidBody(m_rigid_body.get());
}
