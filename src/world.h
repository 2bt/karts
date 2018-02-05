#pragma once
#include "rmw.h"


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



struct Model {
    rmw::VertexBuffer::Ptr vb;
    rmw::IndexBuffer::Ptr  ib;
    rmw::VertexArray::Ptr  va;
    glm::mat4              transform;
    glm::vec3              color;
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

    std::vector<Model>   m_models;
    Camera               m_camera;
    Light                m_light;


    rmw::Shader::Ptr     m_light_shader;
    rmw::Shader::Ptr     m_model_shader;
};
