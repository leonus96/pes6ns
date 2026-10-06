#include "gl_api.hpp"

#include <type_traits>

namespace pes6::gl {

Api api;

bool load(void *(*get_proc)(const char *), std::string &error) {
    Api loaded{};
    bool ok = true;
    const auto get = [&](auto &slot, const char *name) {
        void *symbol = get_proc(name);
        if (symbol == nullptr && ok) {
            error = std::string("missing OpenGL function ") + name;
            ok = false;
        }
        slot = reinterpret_cast<std::remove_reference_t<decltype(slot)>>(symbol);
    };
#define PES6_GL_LOAD(name) get(loaded.name, "gl" #name)
    PES6_GL_LOAD(GetString);
    PES6_GL_LOAD(GetError);
    PES6_GL_LOAD(Enable);
    PES6_GL_LOAD(Disable);
    PES6_GL_LOAD(Viewport);
    PES6_GL_LOAD(Scissor);
    PES6_GL_LOAD(ClearColor);
    PES6_GL_LOAD(ClearDepth);
    PES6_GL_LOAD(Clear);
    PES6_GL_LOAD(ColorMask);
    PES6_GL_LOAD(DepthMask);
    PES6_GL_LOAD(DepthFunc);
    PES6_GL_LOAD(BlendEquation);
    PES6_GL_LOAD(BlendFunc);
    PES6_GL_LOAD(BlendColor);
    PES6_GL_LOAD(GenTextures);
    PES6_GL_LOAD(DeleteTextures);
    PES6_GL_LOAD(BindTexture);
    PES6_GL_LOAD(ActiveTexture);
    PES6_GL_LOAD(TexImage2D);
    PES6_GL_LOAD(TexSubImage2D);
    PES6_GL_LOAD(TexParameteri);
    PES6_GL_LOAD(PixelStorei);
    PES6_GL_LOAD(ReadPixels);
    PES6_GL_LOAD(GenFramebuffers);
    PES6_GL_LOAD(DeleteFramebuffers);
    PES6_GL_LOAD(BindFramebuffer);
    PES6_GL_LOAD(FramebufferTexture2D);
    PES6_GL_LOAD(FramebufferRenderbuffer);
    PES6_GL_LOAD(CheckFramebufferStatus);
    PES6_GL_LOAD(GenRenderbuffers);
    PES6_GL_LOAD(DeleteRenderbuffers);
    PES6_GL_LOAD(BindRenderbuffer);
    PES6_GL_LOAD(RenderbufferStorage);
    PES6_GL_LOAD(BlitFramebuffer);
    PES6_GL_LOAD(GenBuffers);
    PES6_GL_LOAD(DeleteBuffers);
    PES6_GL_LOAD(BindBuffer);
    PES6_GL_LOAD(BufferData);
    PES6_GL_LOAD(BufferSubData);
    PES6_GL_LOAD(GenVertexArrays);
    PES6_GL_LOAD(DeleteVertexArrays);
    PES6_GL_LOAD(BindVertexArray);
    PES6_GL_LOAD(EnableVertexAttribArray);
    PES6_GL_LOAD(VertexAttribPointer);
    PES6_GL_LOAD(VertexAttribIPointer);
    PES6_GL_LOAD(TexBuffer);
    PES6_GL_LOAD(GetIntegerv);
    PES6_GL_LOAD(CullFace);
    PES6_GL_LOAD(FrontFace);
    PES6_GL_LOAD(CreateShader);
    PES6_GL_LOAD(ShaderSource);
    PES6_GL_LOAD(CompileShader);
    PES6_GL_LOAD(GetShaderiv);
    PES6_GL_LOAD(GetShaderInfoLog);
    PES6_GL_LOAD(DeleteShader);
    PES6_GL_LOAD(CreateProgram);
    PES6_GL_LOAD(AttachShader);
    PES6_GL_LOAD(LinkProgram);
    PES6_GL_LOAD(GetProgramiv);
    PES6_GL_LOAD(GetProgramInfoLog);
    PES6_GL_LOAD(DeleteProgram);
    PES6_GL_LOAD(UseProgram);
    PES6_GL_LOAD(GetUniformLocation);
    PES6_GL_LOAD(Uniform1i);
    PES6_GL_LOAD(Uniform2f);
    PES6_GL_LOAD(Uniform3f);
    PES6_GL_LOAD(Uniform4f);
    PES6_GL_LOAD(Uniform4i);
    PES6_GL_LOAD(DrawArrays);
    PES6_GL_LOAD(DrawElements);
    PES6_GL_LOAD(Finish);
#undef PES6_GL_LOAD
    if (ok) api = loaded;
    return ok;
}

} // namespace pes6::gl
