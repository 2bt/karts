// vim: et ts=4 sts=4 sw=4
#include "map.h"
#include "mesh.h"
#include "log.h"
#include "renderer3d.h"
#include "gui.h"


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
    for (auto& v : m_mesh.vertices) v.p *= 3.0f;

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


    float mass = 1000;
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
    //m_rigid_body->setDamping(0.2, 0.2);
    m_rigid_body->setUserPointer(this);
}


glm::vec3 m_pick_pos;
glm::vec3 m_pick_normal;


void Kart::pick(const glm::vec3& pos, const glm::vec3& normal) {
    glm::mat4 inv = glm::inverse(m_model.transform);
    m_pick_pos    = glm::vec3(inv * glm::vec4(pos, 1));
    m_pick_normal = glm::vec3(inv * glm::vec4(normal, 0));
}


struct Spring {
    // for drawing
    glm::vec3 start_point;
    glm::vec3 end_point;


    bool      touches_ground;
    float     compression_ratio;
    float     old_compression_ratio;
    glm::vec3 ground_normal;
    float     force;
};

std::array<Spring, 4> m_springs;
float                 spring_rest_length = 0.8;
float                 spring_constant    = 20000;
float                 spring_damping     = 0.1;


void Kart::update() {

    // random impulse
    const Uint8* ks = SDL_GetKeyboardState(nullptr);
    static bool old_q;
    bool q = ks[SDL_SCANCODE_RETURN];
    if (gui::button("random impulse") | (q && !old_q)) {
        auto randf = []() { return rand() / (float) RAND_MAX; };
        glm::vec3 s = glm::vec3(randf(), randf(), randf()) * 2.0f - glm::vec3(1);
        s *= m_size;
        m_rigid_body->applyImpulse(btVector3(0, 2000, 0), btVector3(s.x, s.y, s.z));
    }
    old_q = q;

    // reset transform
    gui::same_line();
    if (gui::button("reset transform") || ks[SDL_SCANCODE_X]) {
        btTransform& t = m_rigid_body->getWorldTransform();
        t.setOrigin(btVector3(0, 3, 0));
        t.setRotation(btQuaternion(0, 0, 0));
        m_motion_state->setWorldTransform(t);
    }



    // update model transform
    btTransform trans;
    m_motion_state->getWorldTransform(trans);
    trans.getOpenGLMatrix(reinterpret_cast<float*>(&m_model.transform));

    static bool spring_physics = true;
    gui::separator();
    gui::checkbox("enable spring physics", spring_physics);
    if (spring_physics) {

        gui::drag_float("spring length", spring_rest_length, 0, 0.1, 3);
        gui::drag_float("spring constant", spring_constant, 0, 1000, 60000);
        gui::drag_float("spring damping", spring_damping, 0, 0, 0.5);


        // suspension
        const glm::vec3 kart_normal = glm::vec3(m_model.transform * glm::vec4(0, 1, 0, 0));
        for (int i = 0; i < 4; ++i) {
            Spring& spring = m_springs[i];

            const glm::vec2 vs[] = {
                glm::vec2( 1,  1),
                glm::vec2(-1,  1),
                glm::vec2( 1, -1),
                glm::vec2(-1, -1),
            };
            const glm::vec3 v        = glm::vec3(vs[i].x * m_size.x, 0, vs[i].y * m_size.z) * 1.05f;

            // reset spring
            spring.start_point       = glm::vec3(m_model.transform * glm::vec4(v, 1));
            spring.end_point         = spring.start_point - kart_normal * spring_rest_length;
            spring.touches_ground    = false;
            spring.compression_ratio = 0;
            spring.force             = 0;

            btVector3 sp = to_bt(spring.start_point);
            btVector3 ep = to_bt(spring.end_point);
            btCollisionWorld::ClosestRayResultCallback cb(sp, ep);
            m_world->rayTest(sp, ep, cb);
            if (cb.hasHit()) {
                spring.end_point         = to_glm(cb.m_hitPointWorld);
                spring.touches_ground    = true;
                spring.ground_normal     = to_glm(cb.m_hitNormalWorld);
                spring.compression_ratio = 1 - cb.m_closestHitFraction;
                spring.force             = spring_constant * spring.compression_ratio;

                // damping
                float vel = (spring.old_compression_ratio - spring.compression_ratio) * 60;
                float damping_force = vel * spring_constant * spring_damping;
                if (damping_force > 0) damping_force *= 0.9;
                spring.force -= damping_force;

                // clamping
                spring.force = glm::clamp(spring.force, 0.0f, 10000.0f);

                m_rigid_body->applyForce(to_bt(kart_normal) * spring.force, sp - trans.getOrigin());
            }
            spring.old_compression_ratio = spring.compression_ratio;

            //if (i == 0) LOG("%*s", int(30 + spring_force  * 0.01), "#");

            gui::text("compression: %9.3f\n"
                      "force:       %9.3f", spring.compression_ratio, spring.force);
        }
    }


    // movement
    {
        int steering = !!ks[SDL_SCANCODE_J] - !!ks[SDL_SCANCODE_L];
        m_rigid_body->applyTorque(btVector3(0, steering * 4000, 0));

        if (m_springs[2].touches_ground || m_springs[3].touches_ground) {
            float engine = !!ks[SDL_SCANCODE_I] - !!ks[SDL_SCANCODE_K] * 0.5;
            m_rigid_body->applyForce(
                trans.getBasis() * btVector3(0, 0, 10000 * engine ),
                trans.getBasis() * btVector3(0, -0.1, -1));
        }


    }

    // pull up
    {
        int d = ks[SDL_SCANCODE_PERIOD] - ks[SDL_SCANCODE_COMMA];
        glm::vec3 p = glm::vec3(m_model.transform * glm::vec4(m_pick_pos, 1));
        m_rigid_body->applyForce(btVector3(0, d * 15000, 0),
                                 to_bt(p) - trans.getOrigin());
    }
}


void Kart::tick() {

    // update model transform
    btTransform t;
    m_motion_state->getWorldTransform(t);
    t.getOpenGLMatrix(reinterpret_cast<float*>(&m_model.transform));
}


void Kart::debug_draw() {

    for (const Spring& spring : m_springs) {
        if (spring.touches_ground) renderer3D.set_color(255, 0, 0);
        else                       renderer3D.set_color(0, 255, 0);
        renderer3D.line(spring.start_point, spring.end_point);
    }
    renderer3D.set_point_size(3);
    for (const Spring& spring : m_springs) {
        if (spring.touches_ground) renderer3D.set_color(255, 0, 0);
        else                       renderer3D.set_color(0, 255, 0);
        renderer3D.point(spring.start_point);
        renderer3D.point(spring.end_point);
    }


    glm::vec3 p1 = glm::vec3(m_model.transform * glm::vec4(m_pick_pos, 1));
    glm::vec3 p2 = glm::vec3(m_model.transform *
            glm::vec4(m_pick_pos + m_pick_normal * 0.1f, 1));

    renderer3D.set_color(255, 200, 0);
    renderer3D.line(p1, p2);
    renderer3D.set_color(0, 255, 0);
    renderer3D.point(p1);
}
