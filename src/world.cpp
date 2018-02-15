// vim: et ts=4 sts=4 sw=4
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#include "world.h"
#include "mesh.h"
#include "log.h"
#include "renderer3d.h"
#include <glm/gtx/transform.hpp>


void init_light_map(Light& l) {
    int light_map_size = 1024;
    l.shadow_map = rmw::context.create_texture_2D(rmw::TextureFormat::Depth,
            light_map_size, light_map_size);
    l.framebuffer = rmw::context.create_framebuffer();

    #ifdef __EMSCRIPTEN__
    // XXX: the webgl framebuffer is unhappy without color attachment :(
    // is there a trick to get around this?
    static auto foobar = rmw::context.create_texture_2D(rmw::TextureFormat::RGB,
                l.shadow_map->get_width(), l.shadow_map->get_height());
    l.framebuffer->attach_color(foobar);
    #endif

    l.framebuffer->attach_depth(l.shadow_map);
    if (!l.framebuffer->is_complete()) LOG("framebuffer incomplete");
}


void World::init() {

    m_light_shader = rmw::context.create_shader(R"(#version 100
    attribute vec3 a_pos;
    uniform mat4 light_mvp_mat;
    void main() {
        gl_Position = light_mvp_mat * vec4(a_pos, 1.0);
    })",
    R"(#version 100
    void main() {
    })");

    m_model_shader = rmw::context.create_shader(R"(#version 100
    attribute vec3 a_pos;
    attribute vec3 a_norm;
    uniform mat4 mvp_mat;
    uniform mat4 model_mat;
    uniform mat4 biased_shadow_mvp_mat;
    uniform mat3 normal_mat;
    varying vec3 v_frag_pos;
    varying vec4 v_shadow_coord;
    varying vec3 v_norm;
    varying float v_depth;
    void main() {
        v_frag_pos = vec3(model_mat * vec4(a_pos, 1.0));
        gl_Position = mvp_mat * vec4(a_pos, 1.0);
        v_shadow_coord = biased_shadow_mvp_mat * vec4(a_pos, 1.0);
        v_norm = normalize(normal_mat * a_norm);
        v_depth = gl_Position.z;
    })",
    R"(#version 100
    precision mediump float;
    uniform sampler2D shadow_map;
    uniform float light_map_size;
    uniform vec3 light_dir;
    uniform vec3 view_pos;
    uniform vec3 color;
    varying vec3 v_frag_pos;
    varying vec4 v_shadow_coord;
    varying vec3 v_norm;
    varying float v_depth;
    float shadow_sample(vec2 uv, float compare) {
        return step(compare, texture2D(shadow_map, uv).r);
    }
    float smooth_shadow_sample(vec2 uv, float compare) {
        vec2 f = fract(uv + vec2(0.5));
        vec2 c = floor(uv + vec2(0.5)) / light_map_size;
        float lb = shadow_sample(c + vec2(0.0, 0.0) / light_map_size, compare);
        float lt = shadow_sample(c + vec2(0.0, 1.0) / light_map_size, compare);
        float rb = shadow_sample(c + vec2(1.0, 0.0) / light_map_size, compare);
        float rt = shadow_sample(c + vec2(1.0, 1.0) / light_map_size, compare);
        return mix(mix(lb, lt, f.y), mix(rb, rt, f.y), f.x);
    }
    void main() {
        float v = 0.0;
        for (int x = -1; x <= 1; x += 1)
        for (int y = -1; y <= 1; y += 1) {
            vec2 uv = v_shadow_coord.xy * light_map_size + vec2(x, y);
            v += smooth_shadow_sample(uv, v_shadow_coord.z + 0.001) / 9.0;
        }
        vec3 ambient = 0.25 * color;
        vec3 diffuse = 0.75 * color * min(v, max(0.0, dot(v_norm, -light_dir)));
        float shininess = 200.0;
        vec3 halfway = normalize(-light_dir + normalize(view_pos - v_frag_pos));
        vec3 specular = vec3(0.3) * pow(max(dot(v_norm, halfway), 0.0), shininess) * v;
        vec3 color = ambient + diffuse + specular;
        vec3 fog = vec3(0.1, 0.15, 0.25);
        gl_FragColor = vec4(mix(fog, color, pow(0.95, v_depth)), 1.0);
    })");



    m_camera.ang_x = 0.75;
    m_camera.ang_y = -0.69;
    m_camera.pos = { 4.665983, 6.837914, 5.553909 };
    m_camera.fov = 60;


    init_light_map(m_light);
    m_light.dir = glm::normalize(glm::vec3(3, -10, 6));

    // physics
    m_config     = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_config.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver     = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world      = std::make_unique<btDiscreteDynamicsWorld>(m_dispatcher.get(),
                                                             m_broadphase.get(),
                                                             m_solver.get(),
                                                             m_config.get());
    m_world->setGravity(btVector3(0, -2, 0));
    m_world->setInternalTickCallback(World::tick, static_cast<void*>(this), false);



    // init objects
    m_map.init(m_world.get());
    m_kart.init(m_world.get());
}


void World::update_camera() {
    // update camera
    const Uint8* ks = SDL_GetKeyboardState(nullptr);

    m_camera.ang_y += (ks[SDL_SCANCODE_RIGHT] - ks[SDL_SCANCODE_LEFT]) * 0.03;
    m_camera.ang_x += (ks[SDL_SCANCODE_DOWN ] - ks[SDL_SCANCODE_UP  ]) * 0.03;
    m_camera.ang_x = glm::clamp(m_camera.ang_x, (float) -M_PI * 0.5f, (float) M_PI * 0.5f);

    float x = (ks[SDL_SCANCODE_D    ] - ks[SDL_SCANCODE_A     ]) * 0.1;
    float z = (ks[SDL_SCANCODE_S    ] - ks[SDL_SCANCODE_W     ]) * 0.1;
    float y = (ks[SDL_SCANCODE_SPACE] - ks[SDL_SCANCODE_LSHIFT]) * 0.1;

    float cy = cosf(m_camera.ang_y);
    float sy = sinf(m_camera.ang_y);
    float cx = cosf(m_camera.ang_x);
    float sx = sinf(m_camera.ang_x);

    m_camera.pos += glm::vec3(x * cy - z * sy * cx + sy * sx * y,
                              y * cx + z * sx,
                              x * sy + z * cy * cx - cy * sx * y);

    m_camera.vp_mat = glm::perspective(glm::radians(m_camera.fov),
                                       rmw::context.get_aspect_ratio(), 0.1f, 100.0f) *
                      glm::rotate<float>(m_camera.ang_x, glm::vec3(1, 0, 0)) *
                      glm::rotate<float>(m_camera.ang_y, glm::vec3(0, 1, 0)) *
                      glm::translate(-m_camera.pos);

    renderer3D.set_transformation(m_camera.vp_mat);

//    LOG("camera: %f %f %f %f %f", m_camera.ang_x, m_camera.ang_y,
//        m_camera.pos.x, m_camera.pos.y, m_camera.pos.z);

}


void World::update_light() {

    float cy = cosf(m_camera.ang_y);
    float sy = sinf(m_camera.ang_y);
    m_light.pos = glm::vec3(m_camera.pos.x, 0, m_camera.pos.z) +
                  glm::vec3(sy, 0, -cy) * 8.0f;

    glm::mat4 view = glm::lookAt(m_light.pos - m_light.dir * 20.0f,
                                 m_light.pos, glm::vec3(0, 1, 0));
    float s = 10;
    glm::mat4 proj = glm::ortho(-s, s, -s, s, 0.0f, s * 3);

    m_light.vp_mat = proj * view;
    m_model_shader->set_uniform("light_map_size", (float) m_light.shadow_map->get_width());
}


void World::check_picking() {
    int x, y;
    Uint32 buttons = SDL_GetMouseState(&x, &y);
    if (buttons & SDL_BUTTON(1)) {

        glm::vec4 v = glm::inverse(m_camera.vp_mat) *
                      glm::vec4(x / (float) rmw::context.get_width() * 2 - 1,
                                y / (float) rmw::context.get_height() * -2 + 1,
                                1, 1);

        glm::vec3 trg = glm::vec3(v) / v.w;
        btVector3 o = to_bt(m_camera.pos);
        btVector3 p = to_bt(trg);
        btCollisionWorld::ClosestRayResultCallback cb(o, p);
        m_world->rayTest(o, p, cb);
        if (cb.hasHit()) {
            void* ptr = cb.m_collisionObject->getUserPointer();
            if (ptr) {
                GameObject* obj = static_cast<GameObject*>(ptr);
                obj->pick(to_glm(cb.m_hitPointWorld),
                          to_glm(cb.m_hitNormalWorld));
            }
        }
    }
}


void World::update() {

    update_camera();
    update_light();
    check_picking();

    m_kart.update();
    m_world->stepSimulation(1 / 60.0);
}


void World::tick() {

    m_kart.tick();
}


void World::render_shadow_map() {
    rmw::RenderState rs;
    rs.depth_test_enabled = true;


    rmw::context.clear({}, m_light.framebuffer);
    rs.cull_face = rmw::CullFace::Front;

    auto draw = [&](const Model& model) {
        m_light_shader->set_uniform("light_mvp_mat", m_light.vp_mat * model.transform);
        rmw::context.draw(rs, m_light_shader, model.va, m_light.framebuffer);
    };

    draw(m_map.get_model());
    draw(m_kart.get_model());

    rs.cull_face = rmw::CullFace::Back;
}


void World::render_models() {
    rmw::RenderState rs;
    rs.depth_test_enabled = true;

    static const glm::mat4 bias = { 0.5, 0.0, 0.0, 0.0,
                                    0.0, 0.5, 0.0, 0.0,
                                    0.0, 0.0, 0.5, 0.0,
                                    0.5, 0.5, 0.5, 1.0 };
    glm::mat4 biased_shadow_vp_mat = bias * m_light.vp_mat;

    rmw::context.clear({ 0.1, 0.15, 0.25, 1 });

    m_model_shader->set_uniform("shadow_map", m_light.shadow_map);
    m_model_shader->set_uniform("light_dir", m_light.dir);
    m_model_shader->set_uniform("view_pos", m_camera.pos);


    auto draw = [&](const Model& model) {
        m_model_shader->set_uniform("normal_mat", glm::transpose(glm::inverse(glm::mat3(model.transform))));
        m_model_shader->set_uniform("biased_shadow_mvp_mat", biased_shadow_vp_mat * model.transform);
        m_model_shader->set_uniform("model_mat", model.transform);
        m_model_shader->set_uniform("mvp_mat", m_camera.vp_mat * model.transform);
        m_model_shader->set_uniform("color", model.color);
        rmw::context.draw(rs, m_model_shader, model.va);
    };

    draw(m_map.get_model());
    draw(m_kart.get_model());


    m_kart.debug_draw();
}


class Gui {
    public:
        using Vec = glm::i16vec2;
        using Col = glm::u8vec4;

        void init() {
            m_texture = rmw::context.create_texture_2D("assets/gui.png", rmw::FilterMode::Nearest);
            m_shader = rmw::context.create_shader(
            R"(#version 100
            attribute vec2 a_pos;
            attribute vec2 a_uv;
            attribute vec4 a_col;
            varying vec2 v_uv;
            varying vec4 v_col;
            uniform vec2 scale;
            uniform vec2 texture_scale;
            void main() {
                v_uv = a_uv * texture_scale;
                v_col = a_col;
                gl_Position = vec4(vec2(2.0, -2.0) * scale * a_pos + vec2(-1.0, 1.0), 0.0, 1.0);
            })",
            R"(#version 100
            precision mediump float;
            uniform sampler2D texture;
            varying vec2 v_uv;
            varying vec4 v_col;
            void main() {
                gl_FragColor = v_col * texture2D(texture, v_uv);
            })");
            m_shader->set_uniform("texture", m_texture);
            m_shader->set_uniform("texture_scale", glm::vec2(1.0f / m_texture->get_width(),
                        1.0f / m_texture->get_height()));
            m_vb = rmw::context.create_vertex_buffer(rmw::BufferHint::StreamDraw);
            m_va = rmw::context.create_vertex_array();
            m_va->set_primitive_type(rmw::PrimitiveType::Triangles);
            m_va->set_attribute(0, m_vb, rmw::ComponentType::Int16, 2, false, 0, 12);
            m_va->set_attribute(1, m_vb, rmw::ComponentType::Int16, 2, false, 4, 12);
            m_va->set_attribute(2, m_vb, rmw::ComponentType::Uint8, 4, true,  8, 12);
        }

        void begin_win(const char* name) {
            Window* w = find_win(name);
            if (!w) w = create_win(name);
            m_win_stack.emplace_back(w);

            w->cursor_pos = w->pos;

            draw_rect(w->pos, w->size, style.window_color);
        }
        void end_win() {
            Window* w = m_win_stack.back();
            w->size.y = w->cursor_pos.y - w->pos.y;
            m_win_stack.pop_back();
        }
        bool button(const char* label) {
            Window* w = m_win_stack.back();
            draw_rect(w->cursor_pos + Vec(2, 2), {96, 16}, style.button_color);
            w->cursor_pos += Vec(0, 20);
            return false;
        }
        void text(const char* label) {
            Window* w = m_win_stack.back();

            w->cursor_pos += Vec(0, 20);
        }

        void new_frame() {
            m_vertices.clear();
        }
        void render() {
            m_vb->init_data(m_vertices);
            m_va->set_count(m_vertices.size());
            m_shader->set_uniform("scale", glm::vec2(1.0f / rmw::context.get_width(),
                        1.0f / rmw::context.get_height()));
            rmw::RenderState rs;
            rs.blend_enabled = true;
            rs.blend_func_src_rgb = rmw::BlendFunc::SrcAlpha;
            rs.blend_func_dst_rgb = rmw::BlendFunc::OneMinusSrcAlpha;
            rmw::context.draw(rs, m_shader, m_va);
        }

        ~Gui() {
            Window* w = m_win_head;
            while (w) {
                Window* o = w;
                w = w->next;
                delete o;
            }
        }


    private:

        struct {
            Col window_color = {100, 100, 100, 100};
            Col button_color = {100, 130, 170, 255};
        } const style;

        void draw_rect(const Vec& pos, const Vec& size, const Col& color) {
            Vertex vs[] = {
                { pos, {0, 0}, color },
                { pos + Vec(0, size.y), {0, 1}, color },
                { pos + Vec(size.x, 0), {1, 0}, color },
                { pos + size, {1, 1}, color },
            };
            m_vertices.emplace_back(vs[0]);
            m_vertices.emplace_back(vs[1]);
            m_vertices.emplace_back(vs[2]);
            m_vertices.emplace_back(vs[2]);
            m_vertices.emplace_back(vs[1]);
            m_vertices.emplace_back(vs[3]);
        }
        void draw_rect(const Vec& pos, const Vec& size, const Vec& uv, const Col& color) {
            Vertex vs[] = {
                { pos, uv, color },
                { pos + Vec(0, size.y), uv + Vec(0, size.y), color },
                { pos + Vec(size.x, 0), uv + Vec(size.x, 0), color },
                { pos + size, uv + size, color },
            };
            m_vertices.emplace_back(vs[0]);
            m_vertices.emplace_back(vs[1]);
            m_vertices.emplace_back(vs[2]);
            m_vertices.emplace_back(vs[2]);
            m_vertices.emplace_back(vs[1]);
            m_vertices.emplace_back(vs[3]);
        }


        struct Window {
            Window*       next;
            const char*   name;
            Vec           pos;
            Vec           size;

            Vec           cursor_pos;
        };


        Window* find_win(const char* name) {
            Window* w = m_win_head;
            while (w) {
                if (strcmp(w->name, name) == 0) break;
                w = w->next;
            }
            return w;
        }
        Window* create_win(const char* name) {
            Window* w = new Window;
            w->next = m_win_head;
            m_win_head = w;
            w->name = name;
            w->pos = { 50, 50 };
            w->size = { 100, 100 };
            return w;
        }


        Window*                m_win_head = nullptr;
        std::vector<Window*>   m_win_stack;

        struct Vertex {
            Vec pos;
            Vec uv;
            Col col;
        };

        std::vector<Vertex>    m_vertices;

        rmw::Texture2D::Ptr    m_texture;
        rmw::Shader::Ptr       m_shader;
        rmw::VertexArray::Ptr  m_va;
        rmw::VertexBuffer::Ptr m_vb;

};


Gui gui;


void World::draw() {

    render_shadow_map();

    render_models();


    static int c = 0; if (!c++) gui.init();
    gui.new_frame();

    gui.begin_win("my test window");
    if (gui.button("click me!")) puts("button was clicked");
    gui.text("hallo");
    gui.button("another button");
    gui.end_win();

    gui.render();

    // debug render shadow map
//    {
//        auto vb = rmw::context.create_vertex_buffer(rmw::BufferHint::StreamDraw);
//        std::vector<int8_t> data = { 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, };
//        vb->init_data(data);
//        auto va = rmw::context.create_vertex_array();
//        va->set_primitive_type(rmw::PrimitiveType::Triangles);
//        va->set_count(6);
//        va->set_attribute(0, vb, rmw::ComponentType::Int8, 2, false, 0, 2);
//        auto shader = rmw::context.create_shader(R"(#version 100
//        attribute vec2 a_pos;
//        varying vec2 v_uv;
//        void main() {
//            gl_Position = vec4((a_pos - vec2(0.5)) * 1.0, 0, 1.0);
//            v_uv = a_pos;
//        })",
//        R"(#version 100
//        precision mediump float;
//        varying vec2 v_uv;
//        uniform sampler2D tex;
//        void main() {
//            gl_FragColor = vec4(texture2D(tex, v_uv).rrr, 1.0);
//        })");
//        rmw::RenderState rs;
//        shader->set_uniform("tex", m_light.shadow_map);
//        rmw::context.draw(rs, shader, va);
//    }



    // debug render light frustum
//    {
//        renderer3D.set_color(255, 0, 0);
//        glm::mat4 inv_light = glm::inverse(m_light.vp_mat);
//        glm::vec3 corners[8];
//        int i = 0;
//        for (int x : {-1, 1})
//        for (int y : {-1, 1})
//        for (int z : {-1, 1}) {
//            glm::vec4 p = inv_light * glm::vec4(x, y, z, 1);
//            corners[i++] = glm::vec3(p) / p.w;
//        }
//        renderer3D.line(corners[0], corners[1]);
//        renderer3D.line(corners[0], corners[2]);
//        renderer3D.line(corners[1], corners[3]);
//        renderer3D.line(corners[2], corners[3]);
//        renderer3D.line(corners[4], corners[5]);
//        renderer3D.line(corners[4], corners[6]);
//        renderer3D.line(corners[5], corners[7]);
//        renderer3D.line(corners[6], corners[7]);
//        renderer3D.line(corners[0], corners[4]);
//        renderer3D.line(corners[1], corners[5]);
//        renderer3D.line(corners[2], corners[6]);
//        renderer3D.line(corners[3], corners[7]);
//        renderer3D.line(corners[0], corners[6]);
//        renderer3D.line(corners[2], corners[4]);
//    }

    renderer3D.flush();
}
