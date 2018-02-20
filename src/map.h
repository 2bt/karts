// vim: et ts=4 sts=4 sw=4
#pragma once
#include "gameobject.h"
#include <btBulletDynamicsCommon.h>


class Map : public GameObject {
public:
    void init(btDynamicsWorld* world);

private:
    Mesh                                        m_mesh;
    std::unique_ptr<btBvhTriangleMeshShape>     m_shape;
    std::unique_ptr<btTriangleIndexVertexArray> m_interface;
    std::unique_ptr<btRigidBody>                m_rigid_body;
};

