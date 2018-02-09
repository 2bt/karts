#pragma once
#include "rmw.h"
#include "map.h"



struct Camera {
    glm::vec3 pos;
    float     ang_x;
    float     ang_y;
    float     fov;

    glm::mat4 vp_mat;
};


struct Light {
    glm::vec3             pos;
    glm::vec3             dir;
    glm::mat4             vp_mat;

    rmw::Framebuffer::Ptr framebuffer;
    rmw::Texture2D::Ptr   shadow_map;
};


class World {
public:

    void init();
    void update();
    void draw();

private:

    void update_camera();
    void render_shadow_map();
    void render_models();

    Camera               m_camera;
    Light                m_light;

    // render stuff
    rmw::Shader::Ptr     m_light_shader;
    rmw::Shader::Ptr     m_model_shader;


    // physics stuff
    static void update(btDynamicsWorld* world, float dt) {
        static_cast<World*>(world->getWorldUserInfo())->tick();
    }
    void tick();


    Map                                                  m_map;
    Kart                                                 m_kart;

    std::unique_ptr<btSequentialImpulseConstraintSolver> m_solver;
    std::unique_ptr<btDbvtBroadphase>                    m_broadphase;
    std::unique_ptr<btCollisionDispatcher>               m_dispatcher;
    std::unique_ptr<btDefaultCollisionConfiguration>     m_config;
    std::unique_ptr<btDiscreteDynamicsWorld>             m_world;
};
