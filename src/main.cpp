// vim: et ts=4 sts=4 sw=4
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#include "log.h"
#include "renderer3d.h"
#include "rmw.h"
#include "eye.h"
#include "mesh.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <cstdlib>
#include <cstdio>
#include <cmath>


Renderer3D renderer3D;


struct Model {
    rmw::VertexBuffer::Ptr vb;
    rmw::IndexBuffer::Ptr  ib;
    rmw::VertexArray::Ptr  va;
    glm::mat4              transform;
    // material
    glm::vec3              color;
};


//struct Light {
//    glm::mat4             mvp;
//    glm::vec4             pos;
//    glm::vec3             color;
//    rmw::Framebuffer::Ptr framebuffer;
//    rmw::Texture2D::Ptr   depth_map;
//    // only for spot lights
//    glm::vec3             dir;
//    float                 cosphi;
//    float                 attenuation;
//};



class App {
public:
    App() {
        rmw::context.init(800, 600, "karts");
        renderer3D.init();

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
        uniform mat3 normal_mtx;
        varying vec4 v_shadow_coord;
        varying vec3 v_norm;
        varying float v_depth;
        void main() {
            gl_Position = mvp * vec4(a_pos, 1.0);
            v_shadow_coord = depth_mvp * vec4(a_pos, 1.0);
            v_norm = normalize(normal_mtx * a_norm);
            v_depth = gl_Position.z;
        })",
        R"(#version 100
        precision mediump float;
        uniform sampler2D depth_map;
        uniform vec3 light_dir;
        uniform vec3 color;
        varying vec4 v_shadow_coord;
        varying vec3 v_norm;
        varying float v_depth;
        void main() {
            float v = 1.0;

            for (int x = -1; x <= 1; ++x)
            for (int y = -1; y <= 1; ++y) {
                v -= (1.0 / 9.0) * step(texture2D(depth_map, v_shadow_coord.xy + vec2(x, y) / 1024.0).r,
                                        v_shadow_coord.z + 0.001);
            }

            // vec3 col = normalize(v_norm) * 0.5 + vec3(0.5, 0.5, 0.5);

            vec3 ambient = 0.1 * color;
            vec3 diffuse = 0.9 * color * min(v, max(0.0, dot(v_norm, -light_dir)));

            gl_FragColor = vec4(ambient + diffuse * pow(0.85, v_depth), 1.0);
        })");



        // init models
        for (const char* name : { "media/cat.obj", "media/hill.obj" }) {
            Model model;
            model.vb = rmw::context.create_vertex_buffer(rmw::BufferHint::StreamDraw);
            model.ib = rmw::context.create_index_buffer(rmw::BufferHint::StreamDraw);
            model.va = rmw::context.create_vertex_array();
            model.va->set_primitive_type(rmw::PrimitiveType::Triangles);
            model.va->set_attribute(0, model.vb, rmw::ComponentType::Float, 3, false, 0, 24);
            model.va->set_attribute(1, model.vb, rmw::ComponentType::Float, 3, false, 12, 24);
            model.va->set_index_buffer(model.ib);

            Mesh mesh;
            mesh.load(name);
            model.vb->init_data(mesh.vertices);
            model.ib->init_data(mesh.indices);
            model.va->set_count(mesh.indices.size());

            m_models.emplace_back(std::move(model));
        }
        m_models[0].color = { 1, 0.8, 0.7 };
        m_models[1].color = { 0.4, 1, 0.1 };


        // init framebuffer
        m_depth_map = rmw::context.create_texture_2D(rmw::TextureFormat::Depth, 2 * 1024, 2 * 1024);
        m_framebuffer = rmw::context.create_framebuffer();
#ifdef __EMSCRIPTEN__
        // XXX: the webgl framebuffer is unhappy without color attachment :(
        // is there a trick to get around this?
        static auto foobar = rmw::context.create_texture_2D(
                rmw::TextureFormat::RGB, m_depth_map->get_width(), m_depth_map->get_height());
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

        // rotate cat
        static double t = 0;
        t += 0.01;
        m_models[0].transform = glm::translate(glm::vec3(0, 1, 0)) *
                                glm::rotate<float>(t, glm::vec3(1, 0, 0)) *
                                glm::translate(glm::vec3(0, -1, 0));

        glm::mat4 depth_mvp;
        glm::vec3 light_lookat = { 0, 0, 0 };
        glm::vec3 light_pos = glm::vec3(3, 10, cosf(t * 0.5) * 8);

        // render depth map
        {
            rmw::context.clear({ 0, 0, 0, 1 }, m_framebuffer);


            glm::mat4 view = glm::lookAt(light_pos, light_lookat, glm::vec3(0, 1, 0));
            glm::mat4 projection = glm::ortho(-7.0f, 7.0f, -7.0f, 7.0f, 0.0f, 20.0f);
            depth_mvp = projection * view;


            rs.cull_face = rmw::CullFace::Front;
            for (const Model& model : m_models) {
                m_depth_shader->set_uniform("depth_mvp", depth_mvp * model.transform);
                rmw::context.draw(rs, m_depth_shader, model.va, m_framebuffer);
            }
            rs.cull_face = rmw::CullFace::Back;
        }

        // render scene with shadow
        glm::mat4 mvp;
        {
            rmw::context.clear({ 0.1, 0.15, 0.25, 1 });

            m_shader->set_uniform("depth_map", m_depth_map);

            glm::mat4 projection = glm::perspective(glm::radians(60.0f),
                    rmw::context.get_aspect_ratio(), 0.1f, 100.0f);

            mvp = projection * m_eye.get_view_mtx();

            static const glm::mat4 bias = { 0.5, 0.0, 0.0, 0.0,
                                            0.0, 0.5, 0.0, 0.0,
                                            0.0, 0.0, 0.5, 0.0,
                                            0.5, 0.5, 0.5, 1.0 };
            glm::mat4 biased_depth_mvp = bias * depth_mvp;

            for (const Model& model : m_models) {
                m_shader->set_uniform("light_dir", glm::normalize(light_lookat - light_pos));
                m_shader->set_uniform("normal_mtx", glm::transpose(glm::inverse(glm::mat3(model.transform))));
                m_shader->set_uniform("depth_mvp", biased_depth_mvp * model.transform);
                m_shader->set_uniform("mvp", mvp * model.transform);
                m_shader->set_uniform("color", model.color);
                rmw::context.draw(rs, m_shader, model.va);
            }
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



        // debug render light frustum
        {
            renderer3D.set_transformation(mvp);

            renderer3D.set_color(255, 0, 0);
            renderer3D.set_point_size(5);
            renderer3D.point(light_pos);

            glm::mat4 inv_depth = glm::inverse(depth_mvp);
            glm::vec3 corners[8];
            int i = 0;
            for (int x : {-1, 1})
            for (int y : {-1, 1})
            for (int z : {-1, 1}) {
                glm::vec4 p = inv_depth * glm::vec4(x, y, z, 1);
                corners[i++] = glm::vec3(p) / p.w;
            }

            renderer3D.line(corners[0], corners[1]);
            renderer3D.line(corners[0], corners[2]);
            renderer3D.line(corners[1], corners[3]);
            renderer3D.line(corners[2], corners[3]);

            renderer3D.line(corners[4], corners[5]);
            renderer3D.line(corners[4], corners[6]);
            renderer3D.line(corners[5], corners[7]);
            renderer3D.line(corners[6], corners[7]);

            renderer3D.line(corners[0], corners[4]);
            renderer3D.line(corners[1], corners[5]);
            renderer3D.line(corners[2], corners[6]);
            renderer3D.line(corners[3], corners[7]);

            renderer3D.line(corners[0], corners[6]);
            renderer3D.line(corners[2], corners[4]);

            renderer3D.flush();
        }

        rmw::context.flip_buffers();
        return true;
    }
private:
    rmw::Shader::Ptr       m_depth_shader;
    rmw::Shader::Ptr       m_shader;

    rmw::Framebuffer::Ptr  m_framebuffer;
    rmw::Texture2D::Ptr    m_depth_map;

    Eye                    m_eye;
    std::vector<Model>     m_models;
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
