// vim: et ts=4 sts=4 sw=4
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#include "log.h"
#include "renderer2d.h"
#include "renderer3d.h"
#include "rmw.h"
#include "eye.h"
#include "model.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <cstdlib>
#include <cstdio>
#include <cmath>


//Renderer2D renderer2D;
//Renderer3D renderer3D;


struct Thingo {
    rmw::VertexBuffer::Ptr vb;
    rmw::IndexBuffer::Ptr  ib;
    rmw::VertexArray::Ptr  va;
    glm::mat4              transform;
};


struct Light {
};


struct Scene {
    std::vector<Thingo> thingos;
    Light               light;
};


class App {
public:
    App() {
        rmw::context.init(800, 600, "scene");
        // renderer2D.init();
        // renderer3D.init();

        m_shader = rmw::context.create_shader(R"(#version 100
        attribute vec3 a_pos;
        attribute vec3 a_norm;
        uniform mat4 mvp;
        varying float v_depth;
        varying vec3 v_norm;
        void main() {
            gl_Position = mvp * vec4(a_pos, 1.0);
            v_depth = gl_Position.z;
            v_norm = a_norm;
        })",
        R"(#version 100
        precision mediump float;
        varying float v_depth;
        varying vec3 v_norm;
        void main() {
            vec3 col = normalize(v_norm) * 0.5 + vec3(0.5, 0.5, 0.5);
            gl_FragColor = vec4(col * pow(0.8, v_depth), 1.0);
        })");

        m_vb = rmw::context.create_vertex_buffer(rmw::BufferHint::StreamDraw);
        m_ib = rmw::context.create_index_buffer(rmw::BufferHint::StreamDraw);

        m_va = rmw::context.create_vertex_array();
        m_va->set_primitive_type(rmw::PrimitiveType::Triangles);
        m_va->set_attribute(0, m_vb, rmw::ComponentType::Float, 3, false, 0, 24);
        m_va->set_attribute(1, m_vb, rmw::ComponentType::Float, 3, false, 12, 24);
        m_va->set_index_buffer(m_ib);

        Model model;
        model.load("media/cat.obj");
        m_vb->init_data(model.m_vertices);
        m_ib->init_data(model.m_indices);
        m_va->set_count(model.m_indices.size());

        rmw::Framebuffer::Ptr framebuffer;
        rmw::Texture2D::Ptr depth_tex;

        framebuffer = rmw::context.create_framebuffer();
        depth_tex = rmw::context.create_texture_2D(rmw::TextureFormat::Depth, 512, 512);

        framebuffer->attach_color(nullptr);
        framebuffer->attach_depth(depth_tex);
     }

    bool loop() {
        SDL_Event e;
        while (rmw::context.poll_event(e)) {
            switch (e.type) {
            case SDL_QUIT: return false;
            case SDL_KEYDOWN:
                if (e.key.keysym.scancode == SDL_SCANCODE_ESCAPE) return false;
                break;
            case SDL_WINDOWEVENT:
                if (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {}
                break;
            default: break;
            }
        }


        // update
        m_eye.update();

        // render
        rmw::context.clear(rmw::ClearState { { 0, 0, 0, 1 } });

        rmw::RenderState rs;
        rs.depth_test_enabled = true;

        glm::mat4 mat_perspective = glm::perspective(glm::radians(60.0f),
                rmw::context.get_aspect_ratio(), 0.1f, 500.0f);
        m_shader->set_uniform("mvp", mat_perspective * m_eye.get_view_mtx());

        rmw::context.draw(rs, m_shader, m_va);

        rmw::context.flip_buffers();
        return true;
    }
private:
    rmw::Shader::Ptr       m_shader;

    rmw::VertexBuffer::Ptr m_vb;
    rmw::IndexBuffer::Ptr  m_ib;
    rmw::VertexArray::Ptr  m_va;

    Eye                    m_eye;
};


void loop(void* arg) { static_cast<App*>(arg)->loop(); }

int main(int argc, char** argv) {
    App app;
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg(loop, &app, -1, true);
#else
    while (app.loop()) {}
#endif
}
