// vim: et ts=4 sts=4 sw=4
#include "model.h"
#include "log.h"
#include <cstdio>
#include <cstring>
#include <tuple>
#include <map>


bool Model::load(const char* name) {
    FILE* f = fopen(name, "r");
    if (!f) return false;

    std::map<std::tuple<int, int, int>, int> index_table;
    std::vector<glm::vec3> pos;
    std::vector<glm::vec3> norm;

    char line[256];
    while (char* p = fgets(line, sizeof(line), f)) {
        auto next = [&p]() {
            char* q = p + strspn(p, " \t");
            p = q + strcspn(q, " \t");
            if (*p != '\0') *p++ = '\0';
            return q;
        };
        char* cmd = next();
        if (cmd[0] == '#') continue;
        else if (strcmp(cmd, "v") == 0) {
            pos.push_back({ atof(next()), atof(next()), atof(next()) });
        }
        else if (strcmp(cmd, "vn") == 0) {
            norm.push_back({ atof(next()), atof(next()), atof(next()) });
        }
        else if (strcmp(cmd, "f") == 0) {
            for (int i = 0; i < 3; ++i) {
                char* q = next();
                auto t = std::make_tuple(0, 0, 0);
                std::get<0>(t) = atoi(strsep(&q, "/")) - 1;
                if (q) std::get<1>(t) = atoi(strsep(&q, "/")) - 1;
                if (q) std::get<2>(t) = atoi(q) - 1;
                auto it = index_table.find(t);
                if (it == index_table.end()) {
                    index_table[t] = m_vertices.size();
                    m_vertices.push_back(Vertex{ pos[std::get<0>(t)],
                                                 norm[std::get<2>(t)]}); }
                m_indices.emplace_back(index_table[t]);
            }
            if (*next() != '\0') LOG("model '%s' is not triangulated", name);
        }
        else LOG("skipping command \"%s\"", cmd);
    }
    fclose(f);

    return true;
}
