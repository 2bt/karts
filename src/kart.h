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
};

