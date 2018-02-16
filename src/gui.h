// vim: et ts=4 sts=4 sw=4
#pragma once
#include <glm/glm.hpp>


namespace gui {

    void init();
    void kill();

    void new_frame();
    void render();
    void begin_window(const char* name);
    void end_window();
    bool button(const char* label);
    void text(const char* label);

}
