// vim: et ts=4 sts=4 sw=4
#include "eye.h"
#include "log.h"
#include <algorithm>
#include <glm/gtx/transform.hpp>
#include <SDL2/SDL.h>



Eye::Eye() {
    m_ang_x = 0.75;
    m_ang_y = -0.69;
    m_pos = { 2.896143, 4.247687, 3.40952 };
}


void Eye::update() {

    const Uint8* ks = SDL_GetKeyboardState(nullptr);

    m_ang_y += (ks[SDL_SCANCODE_RIGHT] - ks[SDL_SCANCODE_LEFT]) * 0.03;
    m_ang_x += (ks[SDL_SCANCODE_DOWN ] - ks[SDL_SCANCODE_UP  ]) * 0.03;
    m_ang_x = glm::clamp(m_ang_x, (float) -M_PI * 0.5f, (float) M_PI * 0.5f);

    float x = (ks[SDL_SCANCODE_D    ] - ks[SDL_SCANCODE_A     ]) * 0.1;
    float z = (ks[SDL_SCANCODE_S    ] - ks[SDL_SCANCODE_W     ]) * 0.1;
    float y = (ks[SDL_SCANCODE_SPACE] - ks[SDL_SCANCODE_LSHIFT]) * 0.1;

    float cy = cosf(m_ang_y);
    float sy = sinf(m_ang_y);
    float cx = cosf(m_ang_x);
    float sx = sinf(m_ang_x);

    glm::vec3 mov = {
        x * cy - z * sy * cx + sy * sx * y,
        y * cx + z * sx,
        x * sy + z * cy * cx - cy * sx * y,
    };

    m_pos += mov;
//    LOG("eye: %f %f %f %f %f", m_ang_x, m_ang_y, m_pos.x, m_pos.y, m_pos.z);
}


glm::mat4x4 Eye::get_view_mat() const {
    return glm::rotate<float>(m_ang_x, glm::vec3(1, 0, 0)) *
           glm::rotate<float>(m_ang_y, glm::vec3(0, 1, 0)) *
           glm::translate(-m_pos);
}
