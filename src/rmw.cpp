// vim: et ts=4 sts=4 sw=4
#include "rmw.h"
#include <glm/glm.hpp>
#include <GL/glew.h>


namespace rmw {


namespace {

// opengl functions with caching
class {
public:
    void bind_vertex_array(uint32_t handle) {
        if (m_vertex_array != handle) {
            m_vertex_array = handle;
            glBindVertexArray(handle);
        }
    }
    void bind_framebuffer(uint32_t handle) {
        if (m_framebuffer != handle) {
            m_framebuffer = handle;
            glBindFramebuffer(GL_FRAMEBUFFER, handle);
        }
    }
    void bind_texture(int unit, uint32_t target, uint32_t handle) {
        if (m_active_texture != unit) {
            m_active_texture = unit;
            glActiveTexture(GL_TEXTURE0 + unit);
        }
        if (m_textures[unit] != handle) {
            m_textures[unit] = handle;
            glBindTexture(target, handle);
        }
    }
private:
    uint32_t                    m_vertex_array;
    uint32_t                    m_framebuffer;
    std::array<uint32_t, 80>    m_textures;
    int                         m_active_texture = 0;
} gl;

} // namespace


constexpr uint32_t map_to_gl(ComponentType t) {
    const uint32_t lut[] = {
        GL_BYTE, GL_UNSIGNED_BYTE, GL_SHORT, GL_UNSIGNED_SHORT,
        GL_INT, GL_UNSIGNED_INT, GL_FLOAT, GL_HALF_FLOAT,
    };
    return lut[static_cast<int>(t)];
}
constexpr uint32_t map_to_gl(BufferHint h) {
    const uint32_t lut[] = { GL_STREAM_DRAW, GL_DYNAMIC_DRAW, GL_STATIC_DRAW };
    return lut[static_cast<int>(h)];
}
constexpr uint32_t map_to_gl(DepthTestFunc dtf) {
    const uint32_t lut[] = { GL_NEVER, GL_LESS, GL_EQUAL, GL_LEQUAL, };
    return lut[static_cast<int>(dtf)];
}
constexpr uint32_t map_to_gl(PrimitiveType pt) {
    const uint32_t lut[] = {
        GL_POINTS, GL_LINE_STRIP, GL_LINE_LOOP, GL_LINES,
        GL_TRIANGLE_STRIP, GL_TRIANGLE_FAN, GL_TRIANGLES
    };
    return lut[static_cast<int>(pt)];
}
constexpr uint32_t map_to_gl(BlendFunc bf) {
    const uint32_t lut[] = {
        GL_ZERO, GL_ONE, GL_SRC_COLOR, GL_ONE_MINUS_SRC_COLOR, GL_DST_COLOR, GL_ONE_MINUS_DST_COLOR,
        GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_DST_ALPHA, GL_ONE_MINUS_DST_ALPHA,
        GL_CONSTANT_COLOR, GL_ONE_MINUS_CONSTANT_COLOR,
        GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA,
        GL_SRC_ALPHA_SATURATE,
    };
    return lut[static_cast<int>(bf)];
}
constexpr uint32_t map_to_gl(BlendEquation be) {
    const uint32_t lut[] = { GL_FUNC_ADD, GL_FUNC_SUBTRACT, GL_FUNC_REVERSE_SUBTRACT };
    return lut[static_cast<int>(be)];
}
constexpr uint32_t map_to_gl(CullFace cf) {
    const uint32_t lut[] = { GL_FRONT, GL_BACK, GL_FRONT_AND_BACK };
    return lut[static_cast<int>(cf)];
}
constexpr uint32_t map_to_gl(TextureFormat tf) {
    const uint32_t lut[] = { GL_RED, GL_RGB, GL_RGBA, GL_DEPTH_COMPONENT, GL_STENCIL_INDEX, GL_DEPTH_STENCIL };
    return lut[static_cast<int>(tf)];
}


VertexArray::VertexArray() {
    m_first = 0;
    m_count = 0;
    m_indexed = false;
    m_primitive_type = PrimitiveType::Triangles;
    glGenVertexArrays(1, &m_handle);
}
VertexArray::~VertexArray() {
    glDeleteVertexArrays(1, &m_handle);
}


void VertexArray::set_attribute(int i, const VertexBuffer::Ptr& vb, ComponentType component_type,
                                int component_count, bool normalized, int offset, int stride) {
    gl.bind_vertex_array(m_handle);
    vb->bind();
    glEnableVertexAttribArray(i);
    glVertexAttribPointer(i, component_count, map_to_gl(component_type), normalized, stride, reinterpret_cast<void*>(offset));
}
void VertexArray::set_attribute(int i, float f) {
    gl.bind_vertex_array(m_handle);
    glDisableVertexAttribArray(i);
    glVertexAttrib1f(i, f);
}
void VertexArray::set_attribute(int i, const glm::vec2& v) {
    gl.bind_vertex_array(m_handle);
    glDisableVertexAttribArray(i);
    glVertexAttrib2fv(i, &v.x);
}
void VertexArray::set_attribute(int i, const glm::vec3& v) {
    gl.bind_vertex_array(m_handle);
    glDisableVertexAttribArray(i);
    glVertexAttrib3fv(i, &v.x);
}
void VertexArray::set_attribute(int i, const glm::vec4& v) {
    gl.bind_vertex_array(m_handle);
    glDisableVertexAttribArray(i);
    glVertexAttrib4fv(i, &v.x);
}

void VertexArray::set_index_buffer(const IndexBuffer::Ptr& ib) {
    if (ib) {
        m_indexed = true;
        gl.bind_vertex_array(m_handle);
        ib->bind();
    }
    else m_indexed = false;
}


GpuBuffer::GpuBuffer(uint32_t target, BufferHint hint) : m_target(target), m_hint(hint), m_size(0) {
    glGenBuffers(1, &m_handle);
}
GpuBuffer::~GpuBuffer() {
    glDeleteBuffers(1, &m_handle);
}
void GpuBuffer::bind() const {
    glBindBuffer(m_target, m_handle);
}
void GpuBuffer::init_data(const void* data, int size) {
    m_size = size;
    gl.bind_vertex_array(0);
    bind();
    glBufferData(m_target, m_size, data, map_to_gl(m_hint));
}

VertexBuffer::VertexBuffer(BufferHint hint) : GpuBuffer(GL_ARRAY_BUFFER, hint) {}
IndexBuffer::IndexBuffer(BufferHint hint) : GpuBuffer(GL_ELEMENT_ARRAY_BUFFER, hint) {}


// shader

static GLuint compile_shader(uint32_t type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint e = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &e);
    if (e) return s;
    int len = 0;
    glGetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
    char log[len];
    glGetShaderInfoLog(s, len, &len, log);
    fprintf(stderr, "Error: can't compile shader\n%s\n%s\n", src, log);
    glDeleteShader(s);
    return 0;
}


bool Shader::init(const char* vs, const char* fs) {
    GLint v = compile_shader(GL_VERTEX_SHADER, vs);
    if (v == 0) return false;
    GLint f = compile_shader(GL_FRAGMENT_SHADER, fs);
    if (f == 0) {
        glDeleteShader(v);
        return false;
    }
    m_program = glCreateProgram();

    glAttachShader(m_program, v);
    glAttachShader(m_program, f);
    glDeleteShader(v);
    glDeleteShader(f);
    glLinkProgram(m_program);


    // attributes
    int count;
    glGetProgramiv(m_program, GL_ACTIVE_ATTRIBUTES, &count);
    for (int i = 0; i < count; ++i) {
        char name[128];
        int size;
        uint32_t type;
        glGetActiveAttrib(m_program, i, sizeof(name), nullptr, &size, &type, name);
        m_attributes.push_back({ name, type, glGetAttribLocation(m_program, name) });
    }

    // uniforms
    glGetProgramiv(m_program, GL_ACTIVE_UNIFORMS, &count);
    for (int i = 0; i < count; ++i) {
        char name[128];
        int size; // > 1 for arrays
        uint32_t type;
        glGetActiveUniform(m_program, i, sizeof(name), nullptr, &size, &type, name);
        int location = glGetUniformLocation(m_program, name);
        m_uniforms.emplace_back(name, type, location);
        Uniform& u = m_uniforms.back();
        switch (type) {
        case GL_INT:        u.extent = Uniform::Extent<int>(); break;
        case GL_FLOAT:      u.extent = Uniform::Extent<float>(); break;
        case GL_FLOAT_VEC2: u.extent = Uniform::Extent<glm::vec2>(); break;
        case GL_FLOAT_VEC3: u.extent = Uniform::Extent<glm::vec3>(); break;
        case GL_FLOAT_VEC4: u.extent = Uniform::Extent<glm::vec4>(); break;
        case GL_FLOAT_MAT3: u.extent = Uniform::Extent<glm::mat3>(); break;
        case GL_FLOAT_MAT4: u.extent = Uniform::Extent<glm::mat4>(); break;
        case GL_SAMPLER_2D: u.extent = Uniform::ExtentTexture2D(); break;
        default:
            fprintf(stderr, "Error: uniform '%s' has unknown type (%d)\n", name, type);
            assert(false);
        }
    }

    return true;
}
Shader::~Shader() {
    glDeleteProgram(m_program);
}


void gl_uniform(int l, int v) { glUniform1i(l, v); }
void gl_uniform(int l, float v) { glUniform1f(l, v); }
void gl_uniform(int l, const glm::vec2& v) { glUniform2fv(l, 1, &v.x); }
void gl_uniform(int l, const glm::vec3& v) { glUniform3fv(l, 1, &v.x); }
void gl_uniform(int l, const glm::vec4& v) { glUniform4fv(l, 1, &v.x); }
void gl_uniform(int l, const glm::mat3& v) { glUniformMatrix3fv(l, 1, false, &v[0].x); }
void gl_uniform(int l, const glm::mat4& v) { glUniformMatrix4fv(l, 1, false, &v[0].x); }


void Shader::Uniform::update() const {
    std::visit([this](auto& e) {
        using T = std::decay_t<decltype(e)>;
        if constexpr (std::is_same_v<T, ExtentTexture2D>) {
            int unit = location;
            gl.bind_texture(unit, GL_TEXTURE_2D, e.handle);
            glUniform1i(location, unit);
        }
        else {
            if (!e.dirty) return;
            e.dirty = false;
            gl_uniform(location, e.value);
        }
    }, extent);
}


// texture

Texture2D::Texture2D() {
    glGenTextures(1, &m_handle);
}
Texture2D::~Texture2D() {
    glDeleteTextures(1, &m_handle);
}
bool Texture2D::init(const char* filename, FilterMode filter) {
    SDL_Surface* s = IMG_Load(filename);
    if (!s) return false;
    init(s, filter);
    SDL_FreeSurface(s);
    return true;
}
bool Texture2D::init(SDL_Surface* s, FilterMode filter) {
    TextureFormat f = s->format->BytesPerPixel == 1 ? TextureFormat::Red
                    : s->format->BytesPerPixel == 3 ? TextureFormat::RGB
                                                    : TextureFormat::RGBA;
    return init(f, s->w, s->h, s->pixels, filter);
}
bool Texture2D::init(TextureFormat format, int w, int h, void* data, FilterMode filter) {
    //printf("error %d\n", glGetError());
    m_width  = w;
    m_height = h;
    m_format = format;

    gl.bind_texture(0, GL_TEXTURE_2D, m_handle);

    if (filter == FilterMode::Nearest) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                filter == FilterMode::Trilinear ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    }


    // XXX: the browser is very finicky. this is the result of trial and error.
    if (m_format == TextureFormat::Depth) {
        float c[] = { 1.0f, 1.0f, 1.0f, 1.0f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, c);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16,
                     m_width, m_height, 0, map_to_gl(m_format),
                     GL_UNSIGNED_INT, data);
    }
    else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexImage2D(GL_TEXTURE_2D, 0, map_to_gl(m_format),
                     m_width, m_height, 0, map_to_gl(m_format),
                     GL_UNSIGNED_BYTE, data);
    }

    if (filter == FilterMode::Trilinear) {
        glGenerateMipmap(GL_TEXTURE_2D);
    }

    return true;
}



// framebuffer

Framebuffer::Framebuffer(bool gen) {
    if (gen) glGenFramebuffers(1, &m_handle);
    else m_handle = 0;
}
Framebuffer::~Framebuffer() {
    if (m_handle) glDeleteFramebuffers(1, &m_handle);
}
void Framebuffer::attach_color(const Texture2D::Ptr& t) {
    if (t) {
        m_width  = t->m_width;
        m_height = t->m_height;
    }
    gl.bind_framebuffer(m_handle);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_2D, t ? t->m_handle : 0, 0);
    // TODO: is this correct?
    //glDrawBuffer(t ? GL_COLOR_ATTACHMENT0 : GL_NONE);
}
void Framebuffer::attach_depth(const Texture2D::Ptr& t) {
    if (t) {
        m_width  = t->m_width;
        m_height = t->m_height;
    }
    gl.bind_framebuffer(m_handle);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                           GL_TEXTURE_2D, t ? t->m_handle : 0, 0);
}
bool Framebuffer::is_complete() const {
    gl.bind_framebuffer(m_handle);
    return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
}

// context

bool Context::init(int width, int height, const char* title) {
    m_default_framebuffer = Framebuffer::Ptr(new Framebuffer(false));
    m_default_framebuffer->m_width = width;
    m_default_framebuffer->m_height = height;

    SDL_Init(SDL_INIT_VIDEO);
    IMG_Init(IMG_INIT_PNG);

//    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
//    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
//    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    m_window = SDL_CreateWindow(title,
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            width, height,
            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);

    m_gl_context = SDL_GL_CreateContext(m_window);
    if (!m_gl_context) return false;

    SDL_GL_SetSwapInterval(1); // v-sync

    glewExperimental = true;
    glewInit();

    // initialize the reder state according to opengl's initial state
    m_render_state.cull_face_enabled = false;
    m_render_state.depth_test_enabled = false;
    m_render_state.depth_test_func = DepthTestFunc::Less;

    glEnable(GL_PROGRAM_POINT_SIZE);

    return true;
}

Context::~Context() {
    SDL_GL_DeleteContext(m_gl_context);
    SDL_DestroyWindow(m_window);
    SDL_Quit();
    IMG_Quit();
}


bool Context::poll_event(SDL_Event& e) {
    if (!SDL_PollEvent(&e)) return false;
    if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
        m_default_framebuffer->m_width  = e.window.data1;
        m_default_framebuffer->m_height = e.window.data2;
    }
    return true;
}


void Context::clear(const glm::vec4& color, const Framebuffer::Ptr& fb) {
    if (m_clear_color != color) {
        m_clear_color = color;
        glClearColor(m_clear_color.x, m_clear_color.y, m_clear_color.z, m_clear_color.w);
    }
    gl.bind_framebuffer(fb->m_handle);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


void Context::sync_render_state(const RenderState& rs) {
    // depth
    if (m_render_state.depth_test_enabled != rs.depth_test_enabled) {
        m_render_state.depth_test_enabled = rs.depth_test_enabled;
        if (m_render_state.depth_test_enabled) glEnable(GL_DEPTH_TEST);
        else glDisable(GL_DEPTH_TEST);
    }
    if (m_render_state.depth_test_enabled) {
        if (m_render_state.depth_test_func != rs.depth_test_func) {
            m_render_state.depth_test_func = rs.depth_test_func;
            glDepthFunc(map_to_gl(m_render_state.depth_test_func));
        }
    }

    // cull face
    if (m_render_state.cull_face_enabled != rs.cull_face_enabled) {
        m_render_state.cull_face_enabled = rs.cull_face_enabled;
        if (m_render_state.cull_face_enabled) glEnable(GL_CULL_FACE);
        else glDisable(GL_CULL_FACE);
    }
    if (m_render_state.cull_face_enabled) {
        if (m_render_state.cull_face != rs.cull_face) {
            m_render_state.cull_face = rs.cull_face;
            glCullFace(map_to_gl(m_render_state.cull_face));
        }
    }

    // depth
    if (m_render_state.scissor_test_enabled != rs.scissor_test_enabled) {
        m_render_state.scissor_test_enabled = rs.scissor_test_enabled;
        if (m_render_state.scissor_test_enabled) glEnable(GL_SCISSOR_TEST);
        else glDisable(GL_SCISSOR_TEST);
    }
    if (m_render_state.scissor_test_enabled) {
        if (m_render_state.scissor_box.x != rs.scissor_box.x ||
            m_render_state.scissor_box.y != rs.scissor_box.y ||
            m_render_state.scissor_box.w != rs.scissor_box.w ||
            m_render_state.scissor_box.h != rs.scissor_box.h)
        {
            m_render_state.scissor_box = rs.scissor_box;
            glScissor(m_render_state.scissor_box.x,
                      m_render_state.scissor_box.y,
                      m_render_state.scissor_box.w,
                      m_render_state.scissor_box.h);
        }
    }


    // blend
    if (m_render_state.blend_enabled != rs.blend_enabled) {
        m_render_state.blend_enabled = rs.blend_enabled;
        if (m_render_state.blend_enabled) glEnable(GL_BLEND);
        else glDisable(GL_BLEND);
    }
    if (m_render_state.blend_enabled) {
        if (m_render_state.blend_func_src_rgb != rs.blend_func_src_rgb ||
            m_render_state.blend_func_src_alpha != rs.blend_func_src_alpha ||
            m_render_state.blend_func_dst_rgb != rs.blend_func_dst_rgb ||
            m_render_state.blend_func_dst_alpha != rs.blend_func_dst_alpha)
        {
            m_render_state.blend_func_src_rgb   = rs.blend_func_src_rgb;
            m_render_state.blend_func_src_alpha = rs.blend_func_src_alpha;
            m_render_state.blend_func_dst_rgb   = rs.blend_func_dst_rgb;
            m_render_state.blend_func_dst_alpha = rs.blend_func_dst_alpha;
            glBlendFuncSeparate(map_to_gl(m_render_state.blend_func_src_rgb),
                                map_to_gl(m_render_state.blend_func_dst_rgb),
                                map_to_gl(m_render_state.blend_func_src_alpha),
                                map_to_gl(m_render_state.blend_func_dst_alpha));
        }
        if (m_render_state.blend_equation_rgb != rs.blend_equation_rgb ||
            m_render_state.blend_equation_alpha != rs.blend_equation_alpha)
        {
            m_render_state.blend_equation_rgb = rs.blend_equation_rgb;
            m_render_state.blend_equation_alpha = rs.blend_equation_alpha;
            glBlendEquationSeparate(map_to_gl(m_render_state.blend_equation_rgb),
                                    map_to_gl(m_render_state.blend_equation_alpha));
        }
        if (m_render_state.blend_color != rs.blend_color) {
            m_render_state.blend_color = rs.blend_color;
            glBlendColor(m_render_state.blend_color.r,
                         m_render_state.blend_color.g,
                         m_render_state.blend_color.b,
                         m_render_state.blend_color.a);
        }
    }
}


void Context::draw(const RenderState& rs, const Shader::Ptr& shader, const VertexArray::Ptr& va, const Framebuffer::Ptr& fb) {
    if (va->m_count == 0) return;

    sync_render_state(rs);

    // only consider RenderState::viewport if it's valid
    Rect vp = {0, 0, 0, 0};
    if (rs.viewport.w != 0) vp = rs.viewport;
    else {
        vp.w = fb->m_width;
        vp.h = fb->m_height;
    }
    if (memcmp(&m_render_state.viewport, &vp, sizeof(Rect)) != 0) {
        m_render_state.viewport = vp;
        glViewport(m_render_state.viewport.x,
                   m_render_state.viewport.y,
                   m_render_state.viewport.w,
                   m_render_state.viewport.h);
    }
    if (m_render_state.line_width != rs.line_width) {
        m_render_state.line_width = rs.line_width;
        glLineWidth(m_render_state.line_width);
    }


    // sync shader
    if (m_shader != shader.get()) {
        m_shader = shader.get();
        glUseProgram(m_shader->m_program);
    }
    m_shader->update_uniforms();

    gl.bind_vertex_array(va->m_handle);

    gl.bind_framebuffer(fb->m_handle);

    if (va->m_indexed) {
        glDrawElements(map_to_gl(va->m_primitive_type), va->m_count, GL_UNSIGNED_INT,
                       reinterpret_cast<void*>(va->m_first));
    }
    else {
        glDrawArrays(map_to_gl(va->m_primitive_type), va->m_first, va->m_count);
    }

}


void Context::flip_buffers() const {
    SDL_GL_SwapWindow(m_window);
}



Context context;


} // namespace
