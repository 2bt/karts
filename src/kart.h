// vim: et ts=4 sts=4 sw=4
#pragma once
#include "gameobject.h"
#include <btBulletDynamicsCommon.h>


class Kart : public GameObject {
public:
    void init(btDynamicsWorld* world);
    void update();
    void tick();
    void debug_draw();
    void pick(const glm::vec3& pos, const glm::vec3& normal) override;

    glm::vec3 get_pos() const {
        return glm::vec3(m_model.transform[3]);
    }

private:
    glm::vec3                             m_size;
    std::unique_ptr<btCollisionShape>     m_shape;
    std::unique_ptr<btDefaultMotionState> m_motion_state;
    std::unique_ptr<btRigidBody>          m_rigid_body;
    btDynamicsWorld*                      m_world;

    glm::vec3                             m_pick_pos;
    glm::vec3                             m_pick_normal;

    struct Wheel {
        bool      is_front_wheel;
        glm::vec3 local_spring_origin;

        glm::vec3 start_point;
        glm::vec3 end_point;

        bool      touches_ground;
        glm::vec3 ground_normal;

        float     force;
        float     compression;
        float     old_compression;
    };


    std::array<Wheel, 4>                  m_wheels;
    void update_wheel(Wheel& wheel, const glm::vec3 kart_normal);


};

