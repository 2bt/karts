// vim: et ts=4 sts=4 sw=4
#pragma once
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <glm/glm.hpp>
#include <memory>
#include <array>
#include <vector>
#include <string>
#include <variant>


namespace rmw {


// render state

struct Viewport {
    int x = 0;
    int y = 0;
    int w = 0;
    int h = 0;
};

enum class DepthTestFunc { Never, Less, Equal, LEqual };

enum class CullFace { Front, Back, BackAndFront };

enum class BlendFunc {
    Zero, One, SrcColor, OneMinusSrcColor, DstColor, OneMinusDstColor,
    SrcAlpha, OneMinusSrcAlpha, DstAlpha, OneMinusDstAlpha,
    ConstantColor, OneMinusConstantColor,
    ConstantAlpha, OneMinusConstantAlph,
    SrcAlphaSaturate,
};
enum class BlendEquation { Add, Subtract, ReverseSubtract };


struct RenderState {
    Viewport      viewport;

    // depth
    bool          depth_test_enabled      = false;
    DepthTestFunc depth_test_func         = DepthTestFunc::LEqual;

    // cull face
    bool          cull_face_enabled       = true;
    CullFace      cull_face               = CullFace::Back;

    // blend
    bool          blend_enabled           = false;
    BlendFunc     blend_func_src_rgb      = BlendFunc::One;
    BlendFunc     blend_func_src_alpha    = BlendFunc::Zero;
    BlendFunc     blend_func_dst_rgb      = BlendFunc::One;
    BlendFunc     blend_func_dst_alpha    = BlendFunc::Zero;
    BlendEquation blend_equation_rgb      = BlendEquation::Add;
    BlendEquation blend_equation_alpha    = BlendEquation::Add;
    glm::vec4     blend_color;

    float         line_width              = 1;
};


// buffers

enum class BufferHint { StaticDraw, StreamDraw, DynamicDraw };


class GpuBuffer {
public:
    virtual ~GpuBuffer();

    void init_data(const void* data, int size);

    int size() const { return m_size; }

protected:
    GpuBuffer(const GpuBuffer&) = delete;
    GpuBuffer& operator=(const GpuBuffer&) = delete;
    GpuBuffer(uint32_t target, BufferHint hint);

    void bind() const;

    uint32_t   m_target;
    BufferHint m_hint;
    int        m_size;
    uint32_t   m_handle;
};


class VertexBuffer : public GpuBuffer {
    friend class Context;
    friend class VertexArray;
public:
    typedef std::unique_ptr<VertexBuffer> Ptr;

    using GpuBuffer::init_data;

    template<class T>
    void init_data(const std::vector<T>& data) {
        init_data(static_cast<const void*>(data.data()), data.size() * sizeof(T));
    }

private:

    VertexBuffer(BufferHint hint);
};


class IndexBuffer : public GpuBuffer {
    friend class Context;
    friend class VertexArray;
public:
    typedef std::unique_ptr<IndexBuffer> Ptr;

    using GpuBuffer::init_data;

    void init_data(const std::vector<int>& data) {
        init_data(static_cast<const void*>(data.data()), data.size() * sizeof(int));
    }

private:
    IndexBuffer(BufferHint hint);
};


enum class PrimitiveType { Points, LineStrip, LineLoop, Lines, TriangleStrip, TriangleFan, Triangles };
enum class ComponentType { Int8, Uint8, Int16, Uint16, Int32, Uint32, Float, HalfFloat };


class VertexArray {
    friend class Context;
public:
    typedef std::unique_ptr<VertexArray> Ptr;

    ~VertexArray();

    void set_first(int i) { m_first = i; };
    void set_count(int i) { m_count = i; };
    void set_primitive_type(PrimitiveType t) { m_primitive_type = t; };
    PrimitiveType get_primitive_type() const { return m_primitive_type; };

    void set_attribute(int i, const VertexBuffer::Ptr& vb, ComponentType component_type,
                       int component_count, bool normalized, int offset, int stride);
    void set_attribute(int i, float f);
    void set_attribute(int i, const glm::vec2& v);
    void set_attribute(int i, const glm::vec3& v);
    void set_attribute(int i, const glm::vec4& v);
    // ...

    void set_index_buffer(const IndexBuffer::Ptr& ib);

    enum { MAX_NUM_ATTRIBUTES = 5 };

private:
    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;
    VertexArray();

    int           m_first;
    int           m_count;
    bool          m_indexed;
    PrimitiveType m_primitive_type;
    uint32_t      m_handle;
};


// texture

//enum class WrapMode { Clamp, Repeat, ClampZero, MirrowedRepeat };
enum class FilterMode { Nearest, Linear, Trilinear };
enum class TextureFormat { RGB, RGBA, Depth, Stencil, DepthStencil };


class Texture2D {
    friend class Context;
    friend class Shader;
    friend class Framebuffer;
public:
    typedef std::unique_ptr<Texture2D> Ptr;
    ~Texture2D();

    bool init(SDL_Surface* s, FilterMode filter = FilterMode::Trilinear);
    bool init(const char* filename, FilterMode filter = FilterMode::Trilinear);
    bool init(TextureFormat format, int w, int h, void* data = nullptr, FilterMode filter = FilterMode::Nearest);

    // TODO: sampler stuff
//    void set_wrap(WrapMode horiz, WrapMode vert);
//    void set_filter(FilterMode min, FilterMode mag);

    int get_width() const    { return m_width; }
    int get_height() const    { return m_height; }

private:
    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;
    Texture2D();


    int           m_width;
    int           m_height;
    TextureFormat m_format;
    uint32_t      m_handle;
};


// frame buffer
class Framebuffer {
    friend class Context;
public:
    typedef std::unique_ptr<Framebuffer> Ptr;

    ~Framebuffer();

    void attach_color(const Texture2D::Ptr& t);
    void attach_depth(const Texture2D::Ptr& t);
    bool is_complete() const;

private:
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;
    Framebuffer(bool gen = true);

    uint32_t m_handle;
    int      m_width;
    int      m_height;
};


// shader

class Shader {
    friend class Context;
public:
    typedef std::unique_ptr<Shader> Ptr;

    template<class T>
    void set_uniform(const std::string& name, const T& value) {
        for (auto& u : m_uniforms) {
            if (u.name == name) {
                u.set(value);
                return;
            }
        }
        assert(false);
    }

    ~Shader();
private:
    Shader() {}
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    bool init(const char* vs, const char* fs);

    struct Attribute {
        std::string name;
        uint32_t    type;
        int         location;
    };

    struct Uniform {
        Uniform(const std::string& name, uint32_t type, int location)
            : name(name), type(type), location(location) {}

        void update() const;

        template<class T>
        struct Extent {
            Extent() : dirty(true) {}
            mutable bool dirty;
            T            value;
        };
        struct ExtentTexture2D {
            uint32_t handle;
        };

        template<class T>
        void set(const T& value) {
            if constexpr (std::is_same<T, Texture2D::Ptr>::value) {
                ExtentTexture2D& e = std::get<ExtentTexture2D>(extent);
                e.handle = value->m_handle;
            }
            else {
                Extent<T>& e = std::get<Extent<T>>(extent);
                if (e.value != value) {
                    e.value = value;
                    e.dirty = true;
                }
            }
        }

        std::string name;
        uint32_t    type;
        int         location;
        std::variant<
            Extent<float>,
            Extent<glm::vec2>,
            Extent<glm::vec3>,
            Extent<glm::vec4>,
            Extent<glm::mat3>,
            Extent<glm::mat4>,
            ExtentTexture2D
        > extent;
    };

    void update_uniforms() const {
        for (auto& u : m_uniforms) u.update();
    }

    uint32_t               m_program = 0;
    std::vector<Attribute> m_attributes;
    std::vector<Uniform>   m_uniforms;
};



class Context {
public:

    bool init(int width, int height, const char* title);
    ~Context();

    bool poll_event(SDL_Event& e);

    int get_width()  const { return m_default_framebuffer->m_width; }
    int get_height() const { return m_default_framebuffer->m_height; }
    float get_aspect_ratio() const { return get_width() / (float) get_height(); }


    void clear(const glm::vec4& color, const Framebuffer::Ptr& fb);
    void clear(const glm::vec4& color) {
        clear(color, m_default_framebuffer);
    }

    void draw(const RenderState& rs,
              const Shader::Ptr& shader,
              const VertexArray::Ptr& va,
              const Framebuffer::Ptr& fb);

    void draw(const RenderState& rs,
              const Shader::Ptr& shader,
              const VertexArray::Ptr& va)
    {
        draw(rs, shader, va, m_default_framebuffer);
    }

    void flip_buffers() const;


    Shader::Ptr create_shader(const char* vs, const char* fs) const {
        Shader::Ptr s(new Shader());
        if (!s->init(vs, fs)) return nullptr;
        return s;
    }

    VertexBuffer::Ptr create_vertex_buffer(BufferHint hint) const {
        return VertexBuffer::Ptr(new VertexBuffer(hint));
    }

    IndexBuffer::Ptr create_index_buffer(BufferHint hint) const {
        return IndexBuffer::Ptr(new IndexBuffer(hint));
    }

    VertexArray::Ptr create_vertex_array() const {
        return VertexArray::Ptr(new VertexArray());
    }

    Framebuffer::Ptr create_framebuffer() const {
        return Framebuffer::Ptr(new Framebuffer());
    }

    template<typename... Args>
    Texture2D::Ptr create_texture_2D(Args&&... args) const {
        Texture2D::Ptr t(new Texture2D());
        if (!t->init(std::forward<Args>(args)...)) return nullptr;
        return t;
    }

    const Framebuffer::Ptr& get_default_framebuffer() {
        return m_default_framebuffer;
    }

private:
    void sync_render_state(const RenderState& rs);

    SDL_Window*      m_window;
    SDL_GLContext    m_gl_context;

    RenderState      m_render_state;
    glm::vec4        m_clear_color;
    const Shader*    m_shader;

    Framebuffer::Ptr m_default_framebuffer;
};


extern rmw::Context context;


}
