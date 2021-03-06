// vim: et ts=4 sts=4 sw=4
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <html5.h>
#endif
#include "log.h"
#include "world.h"
#include "gui.h"


class App {
public:
    App() {
        rmw::context.init(800, 600, "karts");
        gui::init();
        m_world.init();
    }

    bool loop() {
        SDL_Event e;
        while (rmw::context.poll_event(e)) {
            if (gui::process_event(e)) continue;
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

        gui::new_frame();

        m_world.update();
        m_world.draw();

        gui::render();

        rmw::context.flip_buffers();

        return true;
    }
    static void loop(void* arg) { static_cast<App*>(arg)->loop(); }
private:
    World m_world;
};



int main(int argc, char** argv) {
    App app;
#ifdef __EMSCRIPTEN__
    EmscriptenFullscreenStrategy strategy = {
        EMSCRIPTEN_FULLSCREEN_SCALE_STRETCH,
        EMSCRIPTEN_FULLSCREEN_CANVAS_SCALE_NONE,
        EMSCRIPTEN_FULLSCREEN_FILTERING_DEFAULT,
        nullptr,
        nullptr
    };
    emscripten_enter_soft_fullscreen("canvas", &strategy);
    emscripten_set_main_loop_arg(App::loop, &app, -1, true);
#else
    while (app.loop()) {}
#endif
}
