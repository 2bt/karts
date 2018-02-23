// vim: et ts=4 sts=4 sw=4
#include "kart.h"
#include "log.h"
#include "renderer3d.h"
#include "gui.h"


void Kart::init(btDynamicsWorld* world) {
    m_world = world;

    // model
    Mesh mesh("assets/box.obj");
    m_model.init(mesh);
    m_model.color = { 1.0, 0.7, 0.6 };

    // physics
    // use a simple box
    // TODO: use btConvexHullShape
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

    // init wheels
    m_wheels[0].local_spring_origin = glm::vec3( m_size.x - 0.1, 0.2,  m_size.z - 0.2);
    m_wheels[1].local_spring_origin = glm::vec3(-m_size.x + 0.1, 0.2,  m_size.z - 0.2);
    m_wheels[2].local_spring_origin = glm::vec3( m_size.x - 0.1, 0.2, -m_size.z + 0.2);
    m_wheels[3].local_spring_origin = glm::vec3(-m_size.x + 0.1, 0.2, -m_size.z + 0.2);
    m_wheels[0].is_front_wheel = true;
    m_wheels[1].is_front_wheel = true;
    m_wheels[2].is_front_wheel = false;
    m_wheels[3].is_front_wheel = false;
}


void Kart::pick(const glm::vec3& pos, const glm::vec3& normal) {
    glm::mat4 inv = glm::inverse(m_model.transform);
    m_pick_pos    = glm::vec3(inv * glm::vec4(pos, 1));
    m_pick_normal = glm::vec3(inv * glm::vec4(normal, 0));
}


float                 spring_rest_length = 1;
float                 spring_constant    = 20000;
float                 spring_damping     = 0.1;


struct WheelRayResultCallback : public btCollisionWorld::RayResultCallback
{
    btCollisionObject* m_kart;
    btVector3          m_hitNormalWorld;

    btScalar addSingleResult(btCollisionWorld::LocalRayResult& res, bool normalInWorldSpace) override {

        if (res.m_collisionObject == m_kart) return 1;

        m_closestHitFraction = res.m_hitFraction;
        m_collisionObject    = res.m_collisionObject;
        if (normalInWorldSpace) {
            m_hitNormalWorld = res.m_hitNormalLocal;
        }
        else {
            m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis() * res.m_hitNormalLocal;
        }
//        if (res.m_localShapeInfo) {
//            LOG("%d: %d", res.m_localShapeInfo->m_shapePart,
//                          res.m_localShapeInfo->m_triangleIndex);
//        }
        return res.m_hitFraction;
    }
};


void Kart::update_wheel(Wheel& wheel, const glm::vec3 kart_normal) {

    // reset spring
    wheel.start_point    = glm::vec3(m_model.transform * glm::vec4(wheel.local_spring_origin, 1));
    wheel.end_point      = wheel.start_point - kart_normal * spring_rest_length;
    wheel.touches_ground = false;
    wheel.compression    = 0;
    wheel.force          = 0;

    WheelRayResultCallback callback;
    callback.m_kart = m_rigid_body.get();
    m_world->rayTest(to_bt(wheel.start_point), to_bt(wheel.end_point), callback);
    if (callback.hasHit()) {

        wheel.end_point      = glm::mix(wheel.start_point,
                                        wheel.end_point,
                                        callback.m_closestHitFraction);
        wheel.touches_ground = true;
        wheel.ground_normal  = to_glm(callback.m_hitNormalWorld);
        wheel.compression    = 1 - callback.m_closestHitFraction;
        wheel.force          = spring_constant * wheel.compression;

        // damping
        float vel = (wheel.old_compression - wheel.compression) * 60;
        float damping_force = vel * spring_constant * spring_damping;
        if (damping_force > 0) damping_force *= 0.9;
        wheel.force -= damping_force;

        // clamping
        wheel.force = glm::clamp(wheel.force, 0.0f, 10000.0f);


        glm::vec3 com = glm::vec3(m_model.transform[3]);
        m_rigid_body->applyForce(to_bt(kart_normal) * wheel.force,
                                 to_bt(wheel.start_point - com));


        // engine
        const Uint8* ks = SDL_GetKeyboardState(nullptr);
        float engine = !!ks[SDL_SCANCODE_I] - !!ks[SDL_SCANCODE_K];
        m_rigid_body->applyForce(
            to_bt(glm::vec3(m_model.transform[2] * 5000.0f * engine)),
            to_bt(wheel.end_point + kart_normal * 0.3f - com));

    }
    wheel.old_compression = wheel.compression;

    //if (i == 0) LOG("%*s", int(30 + spring_force  * 0.01), "#");

    gui::text("compression: %9.3f\n"
              "force:       %9.3f", wheel.compression, wheel.force);

}


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

        const glm::vec3 kart_normal = glm::vec3(m_model.transform * glm::vec4(0, 1, 0, 0));
        for (Wheel& wheel : m_wheels) update_wheel(wheel, kart_normal);
    }


    // movement
    {
        int steering = !!ks[SDL_SCANCODE_J] - !!ks[SDL_SCANCODE_L];
        m_rigid_body->applyTorque(btVector3(0, steering * 4000, 0));
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

    for (const Wheel& wheel : m_wheels) {
        if (wheel.touches_ground) renderer3D.set_color(255, 0, 0);
        else                      renderer3D.set_color(0, 255, 0);
        renderer3D.line(wheel.start_point, wheel.end_point);
    }
    renderer3D.set_point_size(3);
    for (const Wheel& wheel : m_wheels) {
        if (wheel.touches_ground) renderer3D.set_color(255, 0, 0);
        else                      renderer3D.set_color(0, 255, 0);
        renderer3D.point(wheel.start_point);
        renderer3D.point(wheel.end_point);
    }


    glm::vec3 p1 = glm::vec3(m_model.transform * glm::vec4(m_pick_pos, 1));
    glm::vec3 p2 = glm::vec3(m_model.transform *
            glm::vec4(m_pick_pos + m_pick_normal * 0.1f, 1));

    renderer3D.set_color(255, 200, 0);
    renderer3D.line(p1, p2);
    renderer3D.set_color(0, 255, 0);
    renderer3D.point(p1);
}

