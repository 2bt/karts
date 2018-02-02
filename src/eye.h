// vim: et ts=4 sts=4 sw=4
#pragma once
#include <glm/glm.hpp>


class Eye {
public:
    Eye();
    void        update();
    glm::mat4x4 get_view_mtx() const;
private:
    float       m_ang_x;
    float       m_ang_y;
    glm::vec3   m_pos;
};


extern Eye eye;
