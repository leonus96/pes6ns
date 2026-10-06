#pragma once

// Minimal OpenGL 3.3 core loader for the GL GE backend. Every entry point is
// fetched through SDL_GL_GetProcAddress, so the same code runs on macOS
// (OpenGL.framework, 4.1 core) and on Switch (devkitPro Mesa over EGL)
// without a system GL header or an extra loader library. Only what the
// backend uses is declared; the enum values are the standard GL ones.

#include <cstddef>
#include <cstdint>
#include <string>

namespace pes6::gl {

using GLenum = unsigned int;
using GLuint = unsigned int;
using GLint = int;
using GLsizei = int;
using GLfloat = float;
using GLdouble = double;
using GLboolean = unsigned char;
using GLbitfield = unsigned int;
using GLchar = char;
using GLubyte = unsigned char;
using GLsizeiptr = std::ptrdiff_t;
using GLintptr = std::ptrdiff_t;

// Prefixed where newlib/libnx define a macro of the plain name (MIN, MAX...).
inline constexpr GLboolean BOOL_FALSE = 0;
inline constexpr GLboolean BOOL_TRUE = 1;
inline constexpr GLbitfield DEPTH_BUFFER_BIT = 0x0100;
inline constexpr GLbitfield COLOR_BUFFER_BIT = 0x4000;
inline constexpr GLenum TRIANGLES = 0x0004;
inline constexpr GLenum NEVER = 0x0200;
inline constexpr GLenum ALWAYS = 0x0207;
inline constexpr GLenum ZERO = 0;
inline constexpr GLenum ONE = 1;
inline constexpr GLenum SRC_COLOR = 0x0300;
inline constexpr GLenum ONE_MINUS_SRC_COLOR = 0x0301;
inline constexpr GLenum SRC_ALPHA = 0x0302;
inline constexpr GLenum ONE_MINUS_SRC_ALPHA = 0x0303;
inline constexpr GLenum DST_ALPHA = 0x0304;
inline constexpr GLenum ONE_MINUS_DST_ALPHA = 0x0305;
inline constexpr GLenum DST_COLOR = 0x0306;
inline constexpr GLenum ONE_MINUS_DST_COLOR = 0x0307;
inline constexpr GLenum CONSTANT_COLOR = 0x8001;
inline constexpr GLenum FUNC_ADD = 0x8006;
inline constexpr GLenum BLEND_MIN = 0x8007;
inline constexpr GLenum BLEND_MAX = 0x8008;
inline constexpr GLenum FUNC_SUBTRACT = 0x800A;
inline constexpr GLenum FUNC_REVERSE_SUBTRACT = 0x800B;
inline constexpr GLenum BLEND = 0x0BE2;
inline constexpr GLenum DEPTH_TEST = 0x0B71;
inline constexpr GLenum SCISSOR_TEST = 0x0C11;
inline constexpr GLenum CULL_FACE = 0x0B44;
inline constexpr GLenum DITHER = 0x0BD0;
inline constexpr GLenum DEPTH_CLAMP = 0x864F;
inline constexpr GLenum TEXTURE_2D = 0x0DE1;
inline constexpr GLenum TEXTURE0 = 0x84C0;
inline constexpr GLenum TEXTURE_MAG_FILTER = 0x2800;
inline constexpr GLenum TEXTURE_MIN_FILTER = 0x2801;
inline constexpr GLenum TEXTURE_WRAP_S = 0x2802;
inline constexpr GLenum TEXTURE_WRAP_T = 0x2803;
inline constexpr GLenum TEXTURE_BASE_LEVEL = 0x813C;
inline constexpr GLenum TEXTURE_MAX_LEVEL = 0x813D;
inline constexpr GLenum NEAREST = 0x2600;
inline constexpr GLenum LINEAR = 0x2601;
inline constexpr GLenum NEAREST_MIPMAP_NEAREST = 0x2700;
inline constexpr GLenum LINEAR_MIPMAP_NEAREST = 0x2701;
inline constexpr GLenum NEAREST_MIPMAP_LINEAR = 0x2702;
inline constexpr GLenum LINEAR_MIPMAP_LINEAR = 0x2703;
inline constexpr GLenum REPEAT = 0x2901;
inline constexpr GLenum CLAMP_TO_EDGE = 0x812F;
inline constexpr GLenum RGBA = 0x1908;
inline constexpr GLenum RGBA8 = 0x8058;
inline constexpr GLenum UNSIGNED_BYTE = 0x1401;
inline constexpr GLenum FLOAT = 0x1406;
inline constexpr GLenum DEPTH_COMPONENT24 = 0x81A6;
inline constexpr GLenum UNPACK_ALIGNMENT = 0x0CF5;
inline constexpr GLenum PACK_ALIGNMENT = 0x0D05;
inline constexpr GLenum FRAMEBUFFER = 0x8D40;
inline constexpr GLenum READ_FRAMEBUFFER = 0x8CA8;
inline constexpr GLenum DRAW_FRAMEBUFFER = 0x8CA9;
inline constexpr GLenum RENDERBUFFER = 0x8D41;
inline constexpr GLenum COLOR_ATTACHMENT0 = 0x8CE0;
inline constexpr GLenum DEPTH_ATTACHMENT = 0x8D00;
inline constexpr GLenum FRAMEBUFFER_COMPLETE = 0x8CD5;
inline constexpr GLenum ARRAY_BUFFER = 0x8892;
inline constexpr GLenum STREAM_DRAW = 0x88E0;
inline constexpr GLenum VERTEX_SHADER = 0x8B31;
inline constexpr GLenum FRAGMENT_SHADER = 0x8B30;
inline constexpr GLenum COMPILE_STATUS = 0x8B81;
inline constexpr GLenum LINK_STATUS = 0x8B82;
inline constexpr GLenum INFO_LOG_LENGTH = 0x8B84;
inline constexpr GLenum VENDOR = 0x1F00;
inline constexpr GLenum RENDERER = 0x1F01;
inline constexpr GLenum VERSION = 0x1F02;

struct Api {
    const GLubyte *(*GetString)(GLenum){};
    GLenum (*GetError)(){};
    void (*Enable)(GLenum){};
    void (*Disable)(GLenum){};
    void (*Viewport)(GLint, GLint, GLsizei, GLsizei){};
    void (*Scissor)(GLint, GLint, GLsizei, GLsizei){};
    void (*ClearColor)(GLfloat, GLfloat, GLfloat, GLfloat){};
    void (*ClearDepth)(GLdouble){};
    void (*Clear)(GLbitfield){};
    void (*ColorMask)(GLboolean, GLboolean, GLboolean, GLboolean){};
    void (*DepthMask)(GLboolean){};
    void (*DepthFunc)(GLenum){};
    void (*BlendEquation)(GLenum){};
    void (*BlendFunc)(GLenum, GLenum){};
    void (*BlendColor)(GLfloat, GLfloat, GLfloat, GLfloat){};
    void (*GenTextures)(GLsizei, GLuint *){};
    void (*DeleteTextures)(GLsizei, const GLuint *){};
    void (*BindTexture)(GLenum, GLuint){};
    void (*ActiveTexture)(GLenum){};
    void (*TexImage2D)(GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void *){};
    void (*TexSubImage2D)(GLenum, GLint, GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, const void *){};
    void (*TexParameteri)(GLenum, GLenum, GLint){};
    void (*PixelStorei)(GLenum, GLint){};
    void (*ReadPixels)(GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, void *){};
    void (*GenFramebuffers)(GLsizei, GLuint *){};
    void (*DeleteFramebuffers)(GLsizei, const GLuint *){};
    void (*BindFramebuffer)(GLenum, GLuint){};
    void (*FramebufferTexture2D)(GLenum, GLenum, GLenum, GLuint, GLint){};
    void (*FramebufferRenderbuffer)(GLenum, GLenum, GLenum, GLuint){};
    GLenum (*CheckFramebufferStatus)(GLenum){};
    void (*GenRenderbuffers)(GLsizei, GLuint *){};
    void (*DeleteRenderbuffers)(GLsizei, const GLuint *){};
    void (*BindRenderbuffer)(GLenum, GLuint){};
    void (*RenderbufferStorage)(GLenum, GLenum, GLsizei, GLsizei){};
    void (*BlitFramebuffer)(GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum){};
    void (*GenBuffers)(GLsizei, GLuint *){};
    void (*DeleteBuffers)(GLsizei, const GLuint *){};
    void (*BindBuffer)(GLenum, GLuint){};
    void (*BufferData)(GLenum, GLsizeiptr, const void *, GLenum){};
    void (*GenVertexArrays)(GLsizei, GLuint *){};
    void (*DeleteVertexArrays)(GLsizei, const GLuint *){};
    void (*BindVertexArray)(GLuint){};
    void (*EnableVertexAttribArray)(GLuint){};
    void (*VertexAttribPointer)(GLuint, GLint, GLenum, GLboolean, GLsizei, const void *){};
    GLuint (*CreateShader)(GLenum){};
    void (*ShaderSource)(GLuint, GLsizei, const GLchar *const *, const GLint *){};
    void (*CompileShader)(GLuint){};
    void (*GetShaderiv)(GLuint, GLenum, GLint *){};
    void (*GetShaderInfoLog)(GLuint, GLsizei, GLsizei *, GLchar *){};
    void (*DeleteShader)(GLuint){};
    GLuint (*CreateProgram)(){};
    void (*AttachShader)(GLuint, GLuint){};
    void (*LinkProgram)(GLuint){};
    void (*GetProgramiv)(GLuint, GLenum, GLint *){};
    void (*GetProgramInfoLog)(GLuint, GLsizei, GLsizei *, GLchar *){};
    void (*DeleteProgram)(GLuint){};
    void (*UseProgram)(GLuint){};
    GLint (*GetUniformLocation)(GLuint, const GLchar *){};
    void (*Uniform1i)(GLint, GLint){};
    void (*Uniform2f)(GLint, GLfloat, GLfloat){};
    void (*Uniform3f)(GLint, GLfloat, GLfloat, GLfloat){};
    void (*Uniform4f)(GLint, GLfloat, GLfloat, GLfloat, GLfloat){};
    void (*Uniform4i)(GLint, GLint, GLint, GLint, GLint){};
    void (*DrawArrays)(GLenum, GLint, GLsizei){};
    void (*Finish)(){};
};

// Valid after a successful load(); the current context must stay current on
// the calling thread for every use.
extern Api api;

// Loads every entry point with get_proc (SDL_GL_GetProcAddress). Returns false
// and names the first missing function in `error`.
[[nodiscard]] bool load(void *(*get_proc)(const char *), std::string &error);

} // namespace pes6::gl
