// vim: et ts=4 sts=4 sw=4
#pragma once
#include <glm/glm.hpp>


namespace gui {

    using Vec = glm::i16vec2;
    using Col = glm::u8vec4;

    void init();
    void kill();

    void new_frame();
    void render();

    void set_next_window_pos(const Vec& pos);
    void begin_window(const char* name);
    void end_window();

    bool button(const char* label);
    void text(const char* fmt, ...);
    bool drag_float(const char* label, float& v, float speed = 1, float min = 0, float max = 0, const char* fmt = "%.3f");
}
