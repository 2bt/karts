#include "map.h"
#include "mesh.h"
#include "log.h"
#include "renderer3d.h"


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



void Map::init(btDynamicsWorld* world) {
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
    world->addRigidBody(m_rigid_body.get());
}


void Kart::init(btDynamicsWorld* world) {
    m_world = world;

    // model
    Mesh mesh("assets/box.obj");
    init_model(m_model, mesh);
    m_model.color = { 1.0, 0.7, 0.6 };

    // physics

    for (auto& v : mesh.vertices) m_size = glm::max(m_size, v.p);
    m_shape = std::make_unique<btBoxShape>(btVector3(m_size.x, m_size.y, m_size.z));

    float mass = 100;
    btVector3 inertia;
    m_shape->calculateLocalInertia(mass, inertia);

    m_motion_state = std::make_unique<btDefaultMotionState>(
            btTransform(btQuaternion(0, 0, 0), btVector3(0, 3, 0)));

    btRigidBody::btRigidBodyConstructionInfo info(mass,
                                                  m_motion_state.get(),
                                                  m_shape.get(),
                                                  inertia);

    m_rigid_body = std::make_unique<btRigidBody>(info);
    m_world->addRigidBody(m_rigid_body.get());

    m_rigid_body->setActivationState(DISABLE_DEACTIVATION);
    m_rigid_body->setDamping(0.1, 0.1);
    m_rigid_body->setUserPointer(this);
}


struct Sensor {
    glm::vec3 o;
    glm::vec3 p;
};
std::array<Sensor, 4> m_sensors;



void Kart::update() {

    btTransform t;
    m_motion_state->getWorldTransform(t);
    t.getOpenGLMatrix(reinterpret_cast<float*>(&m_model.transform));



    // suspension
    for (int i = 0; i < 4; ++i) {
        Sensor& s = m_sensors[i];

        const glm::vec2 vs[] = {
            glm::vec2( 1,  1),
            glm::vec2(-1,  1),
            glm::vec2( 1, -1),
            glm::vec2(-1, -1),
        };
        const glm::vec3 v = glm::vec3(vs[i].x * m_size.x, 0, vs[i].y * m_size.z) * 1.05f;
        s.o = glm::vec3(m_model.transform * glm::vec4(v, 1));

        const float sensor_length = 1.0;
        const glm::vec3 n = glm::vec3(m_model.transform * glm::vec4(0, 1, 0, 0));
        s.p = s.o - n * sensor_length;

        btVector3 o = to_bt(s.o);
        btVector3 p = to_bt(s.p);
        btCollisionWorld::ClosestRayResultCallback cb(o, p);
        m_world->rayTest(o, p, cb);

        if (cb.hasHit()) {
            s.p = to_glm(cb.m_hitPointWorld);

//            glm::vec3 vel = to_glm(m_rigid_body->getVelocityInLocalPoint(o));
//            float v = glm::dot(vel, n);

            float f = 1 - cb.m_closestHitFraction;
            m_rigid_body->applyForce(to_bt(n) * 150 * f, o);
        }
    }


    // random impulse
    const Uint8* ks = SDL_GetKeyboardState(nullptr);
    bool q = !!ks[SDL_SCANCODE_RETURN];
    static bool old_q;
    if (q && !old_q) {
        LOG("impulse");
        auto randf = []() { return rand() / (float) RAND_MAX; };
        glm::vec3 s = m_size * (glm::vec3(randf(), randf(), randf()) * 2.0f - glm::vec3(1));
        m_rigid_body->applyImpulse(btVector3(0, 200, 0), btVector3(s.x, s.y, s.z));

    }

    if (ks[SDL_SCANCODE_X]) {
        btTransform& t = m_rigid_body->getWorldTransform();
        t.setOrigin(btVector3(0, 3, 0));
        t.setRotation(btQuaternion(0, 0, 0));
        m_motion_state->setWorldTransform(t);
    }

    m_rigid_body->applyTorque(btVector3(0, (!!ks[SDL_SCANCODE_C] - ks[SDL_SCANCODE_V]) * 100, 0));

    old_q = q;


}


void Kart::tick() {

    // update model transform
    btTransform t;
    m_motion_state->getWorldTransform(t);
    t.getOpenGLMatrix(reinterpret_cast<float*>(&m_model.transform));
}


void Kart::debug_draw() {

    for (const Sensor& s : m_sensors) {
        renderer3D.set_color(255, 200, 0);
        renderer3D.line(s.o, s.p);
    }
    renderer3D.set_point_size(3);
    for (const Sensor& s : m_sensors) {
        renderer3D.set_color(255, 0, 0);
        renderer3D.point(s.p);
    }
}



