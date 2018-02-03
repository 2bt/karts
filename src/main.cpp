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
        rmw::context.init(800, 600, "karts");
        // renderer2D.init();
        // renderer3D.init();

        m_depth_shader = rmw::context.create_shader(R"(#version 100
        attribute vec3 a_pos;
        uniform mat4 depth_mvp;
        void main() {
            gl_Position = depth_mvp * vec4(a_pos, 1.0);
        })",
        R"(#version 100
        void main() {
        })");

        m_shader = rmw::context.create_shader(R"(#version 100
        attribute vec3 a_pos;
        attribute vec3 a_norm;
        uniform mat4 mvp;
        uniform mat4 depth_mvp;
        varying vec4 v_shadow_coord;
        varying vec3 v_norm;
        varying float v_depth;
        void main() {
            gl_Position = mvp * vec4(a_pos, 1.0);
            v_shadow_coord = depth_mvp * vec4(a_pos, 1.0);
            v_norm = a_norm;
            v_depth = gl_Position.z;
        })",
        R"(#version 100
        precision mediump float;
        uniform sampler2D depth_map;
        varying vec4 v_shadow_coord;
        varying vec3 v_norm;
        varying float v_depth;
        void main() {
            float v = 1.0;
            if (texture2D(depth_map, v_shadow_coord.xy).r < v_shadow_coord.z + 0.003) {
                v = 0.5;
            }
            vec3 col = normalize(v_norm) * 0.5 + vec3(0.5, 0.5, 0.5);
            gl_FragColor = vec4(col * pow(0.85, v_depth), 1.0) * v;
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
        model.load("media/hill.obj");
        m_vb->init_data(model.m_vertices);
        m_ib->init_data(model.m_indices);
        m_va->set_count(model.m_indices.size());


        m_depth_map = rmw::context.create_texture_2D(rmw::TextureFormat::Depth, 2 * 1024, 2 * 1024);
        m_framebuffer = rmw::context.create_framebuffer();

#ifdef __EMSCRIPTEN__
        // the frame buffer is unhappy without color attachment :(
        static auto foobar = rmw::context.create_texture_2D(rmw::TextureFormat::RGB,
                                                            m_depth_map->get_width(),
                                                            m_depth_map->get_height());
        m_framebuffer->attach_color(foobar);
#endif

        m_framebuffer->attach_depth(m_depth_map);
        if (!m_framebuffer->is_complete()) LOG("framebuffer incomplete");
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
        rmw::RenderState rs;
        rs.depth_test_enabled = true;

        // render depth map
        glm::mat4 depth_mvp;
        {
            rmw::context.clear(rmw::ClearState { { 0, 0, 0, 1 } }, m_framebuffer);

            glm::mat4 projection = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 0.0f, 20.0f);
            glm::mat4 view = glm::lookAt(glm::vec3(-1, 10, 8),
                                         glm::vec3(0, 0, 0),
                                         glm::vec3(0, 1, 0));

            depth_mvp = projection * view; // * model
            m_depth_shader->set_uniform("depth_mvp", depth_mvp);


            rs.cull_face = rmw::CullFace::Front;
            rmw::context.draw(rs, m_depth_shader, m_va, m_framebuffer);
            rs.cull_face = rmw::CullFace::Back;
        }

        // render scene with shadow
        {
            rmw::context.clear(rmw::ClearState { { 0.1, 0.15, 0.25, 1 } });

            glm::mat4 projection = glm::perspective(glm::radians(60.0f),
                    rmw::context.get_aspect_ratio(), 0.1f, 100.0f);
            m_shader->set_uniform("mvp", projection * m_eye.get_view_mtx());

            glm::mat4 bias(0.5, 0.0, 0.0, 0.0,
                           0.0, 0.5, 0.0, 0.0,
                           0.0, 0.0, 0.5, 0.0,
                           0.5, 0.5, 0.5, 1.0);
            m_shader->set_uniform("depth_mvp", bias * depth_mvp);
            m_shader->set_uniform("depth_map", m_depth_map);


            rmw::context.draw(rs, m_shader, m_va);
        }


        // debug render depth map
        if (0)
        {
            auto vb = rmw::context.create_vertex_buffer(rmw::BufferHint::StreamDraw);
            std::vector<int8_t> data = { 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, };
            vb->init_data(data);
            auto va = rmw::context.create_vertex_array();
            va->set_primitive_type(rmw::PrimitiveType::Triangles);
            va->set_count(6);
            va->set_attribute(0, vb, rmw::ComponentType::Int8, 2, false, 0, 2);
            auto shader = rmw::context.create_shader(R"(#version 100
            attribute vec2 a_pos;
            varying vec2 v_uv;
            void main() {
                gl_Position = vec4((a_pos - vec2(0.5)) * 1.0, 0, 1.0);
                v_uv = a_pos;
            })",
            R"(#version 100
            precision mediump float;
            varying vec2 v_uv;
            uniform sampler2D tex;
            void main() {
                gl_FragColor = vec4(texture2D(tex, v_uv).rrr, 1.0);
            })");
            rs.depth_test_enabled = false;
            shader->set_uniform("tex", m_depth_map);
            rmw::context.draw(rs, shader, va);
        }

        rmw::context.flip_buffers();
        return true;
    }
private:
    rmw::Shader::Ptr       m_depth_shader;
    rmw::Shader::Ptr       m_shader;

    rmw::VertexBuffer::Ptr m_vb;
    rmw::IndexBuffer::Ptr  m_ib;
    rmw::VertexArray::Ptr  m_va;

    rmw::Framebuffer::Ptr  m_framebuffer;
    rmw::Texture2D::Ptr    m_depth_map;

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
