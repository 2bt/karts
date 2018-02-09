#include "map.h"
#include "mesh.h"
#include "log.h"


void init_model(Model& model, const Mesh& mesh) {
    model.vb = rmw::context.create_vertex_buffer(rmw::BufferHint::StreamDraw);
    model.ib = rmw::context.create_index_buffer(rmw::BufferHint::StreamDraw);
    model.va = rmw::context.create_vertex_array();
    model.va->set_primitive_type(rmw::PrimitiveType::Triangles);
    model.va->set_attribute(0, model.vb, rmw::ComponentType::Float, 3, false, 0, 24);
    model.va->set_attribute(1, model.vb, rmw::ComponentType::Float, 3, false, 12, 24);
    model.va->set_index_buffer(model.ib);
    model.vb->init_data(mesh.vertices);
    model.ib->init_data(mesh.indices);
    model.va->set_count(mesh.indices.size());
}



void Map::init() {
    // XXX: the mesh must be kept alive because the shape needs it
    m_mesh.load("assets/hill.obj");
    init_model(m_model, m_mesh);
    m_model.color = { 0.4, 0.6, 0.3 };

    m_interface = std::make_unique<btTriangleIndexVertexArray>(
            m_mesh.indices.size() / 3, m_mesh.indices.data(), sizeof(int) * 3,
            m_mesh.vertices.size(), (float*) m_mesh.vertices.data(), sizeof(Mesh::Vertex));
    m_shape = std::make_unique<btBvhTriangleMeshShape>(m_interface.get(), true);
    btRigidBody::btRigidBodyConstructionInfo info(0, nullptr, m_shape.get());
    info.m_startWorldTransform.setIdentity();
    m_rigid_body = std::make_unique<btRigidBody>(info);
}


void Kart::init() {
    // model
    Mesh mesh("assets/box.obj");
    init_model(m_model, mesh);
    m_model.color = { 0.7, 0.8, 1.0 };
    // physics

    for (auto& v : mesh.vertices) m_size = glm::max(m_size, v.p);

    m_shape = std::make_unique<btBoxShape>(btVector3(m_size.x, m_size.y, m_size.z));
    float mass = 100;
    btVector3 inertia;
    m_shape->calculateLocalInertia(mass, inertia);

    m_motion_state = std::make_unique<btDefaultMotionState>(
            btTransform(btQuaternion(1, 3, 0, 1), btVector3(0, 3, 0)));

    btRigidBody::btRigidBodyConstructionInfo info(mass,
                                                  m_motion_state.get(),
                                                  m_shape.get(),
                                                  inertia);

    m_rigid_body = std::make_unique<btRigidBody>(info);
    m_rigid_body->setActivationState(DISABLE_DEACTIVATION);
}

void Kart::tick() {
    // random impulse
    const Uint8* ks = SDL_GetKeyboardState(nullptr);
    bool q = !!ks[SDL_SCANCODE_RETURN];
    static bool old_q;
    if (q && !old_q) {
        LOG("impulse");

        auto randf = []() { return rand() / (float) RAND_MAX; };
        glm::vec3 s = m_size * glm::vec3(randf(), randf(), randf()) * 2.0f - glm::vec3(1);
        LOG("%f %f %f", s.x, s.y, s.z);
        m_rigid_body->applyImpulse(btVector3(0, 500, 0), btVector3(s.x, s.y, s.z));

    }
    old_q = q;
}

