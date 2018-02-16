// vim: et ts=4 sts=4 sw=4
#include "gui.h"
#include "rmw.h"
#include "log.h"


namespace gui {
namespace {


    using Vec = glm::i16vec2;
    using Col = glm::u8vec4;


    enum {
        FONT_WIDTH  = 7,
        FONT_HEIGHT = 12,
    };


    struct Vertex {
        Vec pos;
        Vec uv;
        Col col;
    };


    struct Rect {
        Rect() {}
        Rect(const Vec& min, const Vec& max) : min(min), max(max) {}
        Vec tl() const { return min; }
        Vec tr() const { return Vec(max.x, min.y); }
        Vec bl() const { return Vec(min.x, max.y); }
        Vec br() const { return max; }
        Vec size() const { return max - min; }
        Rect expand(short d) const { return { min - Vec(d), max + Vec(d) }; }
        bool contains(const Vec& p) {
            return p.x >= min.x && p.y >= min.y &&
                   p.x <  max.x && p.y <  max.y;
        }

        Vec min;
        Vec max;
    };


    struct Window {
        Window*             next;
        const char*         name;

        Rect                rect;

        Vec                 cursor_pos;
        Rect                content_rect;

        // drawing
        std::vector<Vertex> vertices;
    };


    Vec                    m_mouse_pos;
    Vec                    m_mouse_mov;
    std::array<bool, 3>    m_mouse_buttons;
    std::array<bool, 3>    m_mouse_buttons_clicked;

    const char*            m_item_active;
    const char*            m_item_hovered;

    Vec                    m_window_spawn_pos = { 10, 10 };
    Window*                m_window_head = nullptr;
    std::vector<Window*>   m_window_stack;
    Window*                m_window_active;
    Window*                m_window_hovered;

    std::vector<Vertex>    m_vertices;
    rmw::Texture2D::Ptr    m_texture;
    rmw::Shader::Ptr       m_shader;
    rmw::VertexArray::Ptr  m_va;
    rmw::VertexBuffer::Ptr m_vb;




    Window* find_window(const char* name) {
        Window* w = m_window_head;
        while (w) {
            if (strcmp(w->name, name) == 0) break;
            w = w->next;
        }
        return w;
    }


    Window* create_window(const char* name) {
        Window* w = new Window;
        w->next = m_window_head;
        m_window_head = w;
        w->name = name;
        w->rect.min = w->rect.max = m_window_spawn_pos;
        m_window_spawn_pos += Vec(200, 0);
        return w;
    }


    void draw_quad(const Vertex& v0,
                   const Vertex& v1,
                   const Vertex& v2,
                   const Vertex& v3)
    {
        m_vertices.emplace_back(v0);
        m_vertices.emplace_back(v1);
        m_vertices.emplace_back(v2);
        m_vertices.emplace_back(v2);
        m_vertices.emplace_back(v1);
        m_vertices.emplace_back(v3);
    }


    void draw_rect(const Rect& rect, const Col& color) {
        Vertex vs[] = {
            { rect.tl(), {0, 0}, color },
            { rect.bl(), {0, 1}, color },
            { rect.tr(), {1, 0}, color },
            { rect.br(), {1, 1}, color },
        };
        draw_quad(vs[0], vs[1], vs[2], vs[3]);
    }


    void draw_rect(const Rect& rect, const Col& color, const Vec& uv) {
        Vec s = rect.size();
        Vertex vs[] = {
            { rect.tl(), uv, color },
            { rect.bl(), uv + Vec(0, s.y), color },
            { rect.tr(), uv + Vec(s.x, 0), color },
            { rect.br(), uv + s, color },
        };
        draw_quad(vs[0], vs[1], vs[2], vs[3]);
    }


    enum RectStyle {
        RECT_FILL,
        RECT_FILL_ROUND_1,
        RECT_FILL_ROUND_2,
        RECT_FILL_ROUND_3,
        RECT_STROKE,
        RECT_STROKE_ROUND_1,
        RECT_STROKE_ROUND_2,
        RECT_STROKE_ROUND_3,
    };


    void draw_rect(const Rect& rect, const Col& color, RectStyle style) {
        if (style == 0) {
            draw_rect(rect, color);
            return;
        }
        Vec o = { 16 * style, 0 };
        Vec u = { 7, 0 };
        Vec v = { 0, 7 };
        Vertex vs[] = {
            { rect.tl(),     o,     color },
            { rect.tl() + v, o + v, color },
            { rect.bl() - v, o + v, color },
            { rect.bl(),     o,     color },
            { rect.tl() + u,     o + u,     color },
            { rect.tl() + u + v, o + u + v, color },
            { rect.bl() + u - v, o + u + v, color },
            { rect.bl() + u,     o + u,     color },
            { rect.tr() - u,     o + u,     color },
            { rect.tr() - u + v, o + u + v, color },
            { rect.br() - u - v, o + u + v, color },
            { rect.br() - u,     o + u,     color },
            { rect.tr(),     o,     color },
            { rect.tr() + v, o + v, color },
            { rect.br() - v, o + v, color },
            { rect.br(),     o,     color },
        };
        draw_quad(vs[0], vs[1], vs[4], vs[5]);
        draw_quad(vs[1], vs[2], vs[5], vs[6]);
        draw_quad(vs[2], vs[3], vs[6], vs[7]);
        draw_quad(vs[4], vs[5], vs[8], vs[9]);
        if (style < RECT_STROKE) draw_quad(vs[5], vs[6], vs[9], vs[10]);
        draw_quad(vs[6], vs[7], vs[10], vs[11]);
        draw_quad(vs[8], vs[9], vs[12], vs[13]);
        draw_quad(vs[9], vs[10], vs[13], vs[14]);
        draw_quad(vs[10], vs[11], vs[14], vs[15]);
    }


    Vec get_text_size(const char* text) {
        Vec s = { 0, FONT_HEIGHT };
        short x = 0;
        while (char c = *text++) {
            if (c == '\n') {
                s.y += FONT_HEIGHT;
                x = 0;
            }
            else {
                x += FONT_WIDTH;
                s.x = std::max(s.x, x);
            }
        }
        return s;
    }


    void draw_text(const Vec& pos, const char* text) {
        Vec p = pos;
        while (char c = *text++) {
            if (c == 10) {
                p.y += FONT_HEIGHT;
                p.x = pos.x;
                continue;
            }
            if (c > 32 || c < 128) {
                Vec uv = { c % 16 * FONT_WIDTH, c / 16 * FONT_HEIGHT };
                Rect rect = { p, p + Vec(FONT_WIDTH, FONT_HEIGHT) };
                draw_rect(rect, { 255, 255, 255, 255 }, uv);
            }
            p.x += FONT_WIDTH;
        }
    }



    Col make_color(uint32_t c, uint8_t a = 255) {
        return { (c >> 16) & 255, (c >>  8) & 255, c & 255, a};
    }


    struct {
        Col window         = make_color(0x111111, 200);
        Col button         = make_color(0x225577);
        Col button_hovered = make_color(0x336688);
        Col button_active  = make_color(0x337799);
    } const m_colors;


} // namespace


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
        gl_Position = vec4(vec2(2.0, -2.0) * scale * a_pos +
                           vec2(-1.0, 1.0), 0.0, 1.0);
    })",
    R"(#version 100
    precision mediump float;
    uniform sampler2D texture;
    varying vec2 v_uv;
    varying vec4 v_col;
    void main() {
        gl_FragColor = v_col * vec4(1.0, 1.0, 1.0, texture2D(texture, v_uv).r);
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


void kill() {
    Window* w = m_window_head;
    while (w) {
        Window* o = w;
        w = w->next;
        delete o;
    }
}


void new_frame() {
    // mouse
    int x, y;
    Uint32 b = SDL_GetMouseState(&x, &y);
    bool bs[] = {
        b & SDL_BUTTON(SDL_BUTTON_LEFT),
        b & SDL_BUTTON(SDL_BUTTON_MIDDLE),
        b & SDL_BUTTON(SDL_BUTTON_RIGHT),
    };
    for (int i = 0; i < 3; ++i) {
        m_mouse_buttons_clicked[i] = !m_mouse_buttons[i] && bs[i];
        m_mouse_buttons[i] = bs[i];
    }
    if (!m_mouse_buttons[0]) m_item_active = nullptr;
    Vec p = { x, y };
    m_mouse_mov = p - m_mouse_pos;
    m_mouse_pos = p;

    m_vertices.clear();
    m_window_hovered = nullptr;
}


void render() {
    m_vb->init_data(m_vertices);
    m_va->set_count(m_vertices.size());
    glm::vec2 scale = { 1.0f / rmw::context.get_width(),
                        1.0f / rmw::context.get_height() };
    m_shader->set_uniform("scale", scale);
    rmw::RenderState rs;
    rs.blend_enabled = true;
    rs.blend_func_src_rgb = rmw::BlendFunc::SrcAlpha;
    rs.blend_func_dst_rgb = rmw::BlendFunc::OneMinusSrcAlpha;
    rmw::context.draw(rs, m_shader, m_va);
}


void begin_window(const char* name) {
    Window* w = find_window(name);
    if (!w) w = create_window(name);
    m_window_stack.emplace_back(w);

//    bool hovered = w->rect.contains(m_mouse_pos);
//    bool clicked = hovered && m_mouse_buttons_clicked[0] &&
//    if (!m_item_active && m_window_hovered == name &&
//    if (m_window_hovered

    draw_rect(w->rect, m_colors.window, RECT_FILL_ROUND_1);
    w->cursor_pos = w->rect.min + Vec(4);
    w->content_rect.min = w->cursor_pos;
    w->content_rect.max = w->cursor_pos;
}


void end_window() {
    Window* w = m_window_stack.back();
    w->rect.max = w->content_rect.max + Vec(4);
    m_window_stack.pop_back();
//    if (m_mouse_buttons[0] && w->rect.contains(m_mouse_pos)) {
//        w->rect.min += m_mouse_mov;
//        w->rect.max += m_mouse_mov;
//    }
}


bool button(const char* label) {
    Window* w = m_window_stack.back();
    Vec s = get_text_size(label);
    Rect rect = { w->cursor_pos, w->cursor_pos + s + Vec(12) };

    Rect bb = rect.expand(-2);
    bool hovered = bb.contains(m_mouse_pos);
    bool clicked = hovered && m_mouse_buttons_clicked[0];
    if (clicked) m_item_active = label;
    bool active = m_item_active == label;

    Col color = active  ? m_colors.button_active :
                hovered ? m_colors.button_hovered :
                          m_colors.button;

    draw_rect(bb, color, RECT_FILL_ROUND_1);
    draw_text(bb.min + Vec(4), label);

    w->cursor_pos.y = rect.max.y;
    w->content_rect.max = max(w->content_rect.max, rect.max);

    return clicked;
}


void text(const char* label) {
    Window* w = m_window_stack.back();
    Vec s = get_text_size(label);
    Rect rect = { w->cursor_pos, w->cursor_pos + s + Vec(4) };

    draw_text(rect.min + Vec(2), label);

    w->cursor_pos.y = rect.max.y;
    w->content_rect.max = max(w->content_rect.max, rect.max);
}


} // namespace
