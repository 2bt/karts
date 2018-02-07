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


Model load_model(const char* name) {
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
    return model;
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
        vec3 ambient = 0.15 * color;
        vec3 diffuse = 0.85 * color * min(v, max(0.0, dot(v_norm, -light_dir)));
        float shininess = 200.0;
        vec3 halfway = normalize(-light_dir + normalize(view_pos - v_frag_pos));
        vec3 specular = vec3(0.3) * pow(max(dot(v_norm, halfway), 0.0), shininess) * v;
        vec3 color = ambient + diffuse + specular;
        vec3 fog = vec3(0.1, 0.15, 0.25);
        gl_FragColor = vec4(mix(fog, color, pow(0.95, v_depth)), 1.0);
    })");



    m_camera.ang_x = 0.75;
    m_camera.ang_y = -0.69;
    m_camera.pos = { 2.896143, 4.247687, 3.40952 };
    m_camera.fov = 60;

    // init models
    m_models.emplace_back(load_model("media/cat.obj"));
    m_models.emplace_back(load_model("media/hill.obj"));
    m_models[0].color = { 1, 0.8, 0.7 };
    m_models[1].color = { 0.4, 0.6, 0.3 };


    init_light_map(m_light);
    m_light.pos = glm::vec3(-3, 10, -8);
    m_light.dir = glm::normalize(glm::vec3(0, 0, 0) - m_light.pos);
    glm::mat4 view = glm::lookAt(m_light.pos, m_light.pos + m_light.dir, glm::vec3(0, 1, 0));
    glm::mat4 proj = glm::ortho(-7.0f, 7.0f, -7.0f, 7.0f, 0.0f, 20.0f);
    m_light.vp_mat = proj * view;
    m_model_shader->set_uniform("light_map_size", (float) m_light.shadow_map->get_width());


    // physics
    m_config     = std::make_unique<btDefaultCollisionConfiguration>();
    m_dispatcher = std::make_unique<btCollisionDispatcher>(m_config.get());
    m_broadphase = std::make_unique<btDbvtBroadphase>();
    m_solver     = std::make_unique<btSequentialImpulseConstraintSolver>();
    m_world      = std::make_unique<btDiscreteDynamicsWorld>(m_dispatcher.get(),
                                                             m_broadphase.get(),
                                                             m_solver.get(),
                                                             m_config.get());
    m_world->setGravity(btVector3(0, -30, 0));
    m_world->setInternalTickCallback(World::update, static_cast<void*>(this), false);
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

    glm::vec3 mov = {
        x * cy - z * sy * cx + sy * sx * y,
        y * cx + z * sx,
        x * sy + z * cy * cx - cy * sx * y,
    };

    m_camera.pos += mov;
//    LOG("camera: %f %f %f %f %f", m_camera.ang_x, m_camera.ang_y, m_camera.pos.x, m_camera.pos.y, m_camera.pos.z);

    m_camera.vp_mat = glm::perspective(glm::radians(m_camera.fov), rmw::context.get_aspect_ratio(), 0.1f, 100.0f) *
                      glm::rotate<float>(m_camera.ang_x, glm::vec3(1, 0, 0)) *
                      glm::rotate<float>(m_camera.ang_y, glm::vec3(0, 1, 0)) *
                      glm::translate(-m_camera.pos);
}


void World::update() {

    update_camera();

    // rotate cat
    static double t = 0;
    t += 0.01;
    m_models[0].transform = glm::translate(glm::vec3(0, 1, 0)) *
                            glm::rotate<float>(t, glm::vec3(1, 0, 0)) *
                            glm::translate(glm::vec3(0, -1, 0));

    // physics
    m_world->stepSimulation(1 / 60.0);
}


void World::tick() {
}


void World::render_shadow_map() {
    rmw::RenderState rs;
    rs.depth_test_enabled = true;


    rmw::context.clear({}, m_light.framebuffer);
    rs.cull_face = rmw::CullFace::Front;
    for (const Model& model : m_models) {
        m_light_shader->set_uniform("light_mvp_mat", m_light.vp_mat * model.transform);
        rmw::context.draw(rs, m_light_shader, model.va, m_light.framebuffer);
    }
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
    for (const Model& model : m_models) {
        m_model_shader->set_uniform("normal_mat", glm::transpose(glm::inverse(glm::mat3(model.transform))));
        m_model_shader->set_uniform("biased_shadow_mvp_mat", biased_shadow_vp_mat * model.transform);
        m_model_shader->set_uniform("model_mat", model.transform);
        m_model_shader->set_uniform("mvp_mat", m_camera.vp_mat * model.transform);
        m_model_shader->set_uniform("color", model.color);
        rmw::context.draw(rs, m_model_shader, model.va);
    }
}


void World::draw() {

    render_shadow_map();

    render_models();



    // debug render shadow map
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
        rmw::RenderState rs;
        shader->set_uniform("tex", m_light.shadow_map);
        rmw::context.draw(rs, shader, va);
    }



    // debug render light frustum
    {
        renderer3D.set_transformation(m_camera.vp_mat);

        renderer3D.set_color(255, 0, 0);
        renderer3D.set_point_size(5);
        renderer3D.point(m_light.pos);

        glm::mat4 inv_light = glm::inverse(m_light.vp_mat);
        glm::vec3 corners[8];
        int i = 0;
        for (int x : {-1, 1})
        for (int y : {-1, 1})
        for (int z : {-1, 1}) {
            glm::vec4 p = inv_light * glm::vec4(x, y, z, 1);
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
}
