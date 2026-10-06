// OpenGL 3.3 core GE backend (desktop and Switch).
//
// The renderer (ge_renderer.cpp) keeps decoding GE lists, vertices, transforms,
// clipping and textures on the CPU; this backend replaces only the per-pixel
// work, which was ~95 % of a frame. Its inputs are the hooks ge_gpu_backend.hpp
// already defines for VCS's DX12 backend:
//   - record_draw: one per PRIM, names the render target;
//   - texture_needed / upload_decoded_texture_chain_packed / texture_available:
//     RGBA8 texture cache keyed by the full PSP texture state;
//   - accumulate_color_triangles: clipped screen-space triangles (PSP pixels,
//     Z in 0..65535, clip W for perspective-correct interpolation).
// Draws are batched per state and submitted when the frame is finished
// (finish_color_frame at vblank), before a frame dump, or before a texture
// that pending draws sample is replaced.
//
// PES6 renders into just two framebuffers and never samples one (measured with
// PSPRECOMP_GE_TARGET_CENSUS over a whole match), so each is a GL texture +
// depth renderbuffer owned by the GPU; guest VRAM only matters when the CPU
// writes pixels there (the intro movie: invalidate_framebuffer).
//
// Image orientation: PSP row 0 (top) is the highest GL row, so the default
// framebuffer is filled with a straight blit; CPU uploads and readbacks flip.
//
// Not emulated yet (approximations noted in place): 5551 colour quantization
// and the 1-bit destination alpha, 2x alpha blend factors, ABS blend, partial
// colour write masks, stencil, lines/points (never sent by the renderer).
#include "ge_gpu_backend.hpp"

#include "framebuffer_capture.hpp"
#include "gl_api.hpp"
#include "pes6_runtime_log.hpp"
#include "psprecomp/guest_memory.hpp"

#include <SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace pes6 {
namespace {

using namespace gl;

constexpr std::uint32_t kTargetWidth = 480u;
constexpr std::uint32_t kTargetHeight = 272u;

std::uint32_t target_key(std::uint32_t address) noexcept { return address & 0x001FFFF0u; }

struct GlVertex {
    float x, y, z, w;
    std::uint32_t rgba;
    float u, v, q, fog;
};
static_assert(sizeof(GlVertex) == 36u);

// Everything that differs between batches. All 32-bit fields: compared and
// hashed as raw bytes, so there must be no padding.
struct DrawState {
    std::uint32_t target{};
    std::uint32_t texture{};  // GL name, 0 = untextured
    std::uint32_t texture_function{};
    std::uint32_t texture_use_alpha{};
    std::uint32_t texture_double{};
    std::uint32_t texture_env{};
    float texture_width{1.0f};
    float texture_height{1.0f};
    std::uint32_t alpha_test{};
    std::uint32_t alpha_function{};
    std::uint32_t alpha_reference{};
    std::uint32_t alpha_mask{};
    std::uint32_t fog{};
    std::uint32_t fog_color{};
    std::uint32_t blend{};
    std::uint32_t blend_equation{};
    std::uint32_t blend_source{};
    std::uint32_t blend_dest{};
    std::uint32_t blend_color{};
    std::uint32_t source_premultiply{};   // 1: the shader multiplies RGB by premultiply_color
    std::uint32_t premultiply_color{};
    std::uint32_t depth_test{};
    std::uint32_t depth_function{};
    std::uint32_t depth_write{};
    std::uint32_t color_mask{};  // bit per channel R,G,B,A
    std::int32_t scissor_x0{}, scissor_y0{}, scissor_x1{}, scissor_y1{};
};
static_assert(sizeof(DrawState) == 29u * 4u);

bool same_state(const DrawState &a, const DrawState &b) noexcept {
    return std::memcmp(&a, &b, sizeof(DrawState)) == 0;
}

struct Batch {
    DrawState state;
    std::uint32_t first{};
    std::uint32_t count{};
};

struct Target {
    std::uint32_t address{};
    GLuint color{};
    GLuint depth{};
    GLuint fbo{};
    std::uint32_t format{1u};
    std::uint32_t stride{512u};
    // The GPU image is authoritative (drawn into since the last CPU write).
    bool gpu_valid{};
    // The CPU wrote into VRAM: upload it before the next GPU draw.
    bool needs_upload{true};
};

struct Texture {
    GLuint name{};
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint64_t signature{};
    std::uint64_t signature_epoch{~0ull};
    std::uint64_t last_used_epoch{};
    std::uint64_t pending_flush{~0ull};  // flush serial whose batches sample it
    std::size_t bytes{};
};

// Scratch colour surface for flipped uploads/readbacks and CPU presents.
struct Scratch {
    GLuint texture{};
    GLuint fbo{};
    std::uint32_t width{};
    std::uint32_t height{};
};

struct Uniforms {
    GLint target_size{-1};
    GLint texture_sampler{-1};
    GLint texture_enabled{-1};
    GLint texture_control{-1};
    GLint texture_env{-1};
    GLint texture_size{-1};
    GLint alpha_control{-1};
    GLint fog_enabled{-1};
    GLint fog_color{-1};
    GLint premultiply{-1};
    GLint premultiply_color{-1};
};

struct GlBackend {
    bool active{};
    std::uint32_t scale{1u};
    const psprecomp::GuestMemory *memory{};
    GeGpuBackendReport report{};
    std::uint32_t display_framebuffer{};

    GLuint program{};
    Uniforms uniforms{};
    GLuint vao{};
    GLuint vbo{};
    // Bound for untextured draws: texture 0 is incomplete, and some drivers
    // (macOS) warn about any incomplete texture on a sampled unit.
    GLuint white_texture{};

    std::unordered_map<std::uint32_t, Target> targets;
    std::unordered_map<std::uint64_t, Texture> textures;
    std::size_t texture_bytes{};
    std::size_t texture_budget{256u << 20u};
    std::uint64_t frame_epoch{1u};
    std::uint64_t flush_serial{1u};

    std::vector<GlVertex> vertices;
    std::vector<Batch> batches;

    Scratch upload;
    Scratch readback;
    Scratch present;

    // Last state applied to GL, to skip redundant calls.
    DrawState applied{};
    bool applied_valid{};
    std::uint32_t bound_target{~0u};
};

GlBackend &backend() {
    static GlBackend b;
    return b;
}

bool environment_flag(const char *name) {
    const char *text = std::getenv(name);
    return text != nullptr && *text != '\0' && std::strcmp(text, "0") != 0;
}

std::uint64_t hash_mix(std::uint64_t hash, std::uint64_t value) noexcept {
    hash ^= value + 0x9E3779B97F4A7C15ull + (hash << 6u) + (hash >> 2u);
    return hash;
}

// Same identity as the DX12 backend's: every field that changes the decoded
// image or how it is sampled. The content signature is checked separately so
// a texture streamed into the same address refreshes in place.
std::uint64_t texture_key(const GeGpuDrawDescriptor &draw) noexcept {
    if (draw.texture_cache_key_hint != 0u) return draw.texture_cache_key_hint;
    std::uint64_t key = 0xCBF29CE484222325ull;
    const std::uint32_t levels = draw.texture_mipmap_enabled
        ? std::min<std::uint32_t>(8u, draw.texture_max_level + 1u) : 1u;
    key = hash_mix(key, levels);
    for (std::uint32_t level = 0u; level < levels; ++level) {
        key = hash_mix(key, draw.texture_level_addresses[level]);
        key = hash_mix(key, draw.texture_level_buffer_widths[level]);
        key = hash_mix(key, draw.texture_level_widths[level]);
        key = hash_mix(key, draw.texture_level_heights[level]);
    }
    key = hash_mix(key, draw.texture_format);
    key = hash_mix(key, draw.clut_address);
    key = hash_mix(key, draw.clut_format);
    key = hash_mix(key, draw.clut_shift);
    key = hash_mix(key, draw.clut_mask);
    key = hash_mix(key, draw.clut_start);
    key = hash_mix(key, draw.clut_checksum);
    key = hash_mix(key, static_cast<std::uint64_t>(draw.texture_swizzled));
    key = hash_mix(key, static_cast<std::uint64_t>(draw.texture_min_linear));
    key = hash_mix(key, static_cast<std::uint64_t>(draw.texture_mag_linear));
    key = hash_mix(key, static_cast<std::uint64_t>(draw.texture_mipmap_enabled));
    key = hash_mix(key, static_cast<std::uint64_t>(draw.texture_mipmap_linear));
    key = hash_mix(key, draw.texture_max_level);
    key = hash_mix(key, static_cast<std::uint64_t>(draw.texture_clamp_u));
    key = hash_mix(key, static_cast<std::uint64_t>(draw.texture_clamp_v));
    return key;
}

Texture *find_texture(GlBackend &b, std::uint64_t key) {
    const auto found = b.textures.find(key);
    return found == b.textures.end() ? nullptr : &found->second;
}

// ---- shaders ----------------------------------------------------------------

constexpr const char *kVertexShader = R"(#version 330 core
layout(location = 0) in vec4 a_position;   // PSP pixels x, y; Z 0..65535; clip W
layout(location = 1) in vec4 a_color;
layout(location = 2) in vec4 a_uvqf;       // u, v (texels), q, fog factor
uniform vec2 u_target_size;
out vec4 v_color;
out vec3 v_uvq;
out float v_fog;
void main() {
    float w = a_position.w > 0.0 ? a_position.w : 1.0;
    vec2 ndc = vec2(a_position.x / u_target_size.x * 2.0 - 1.0,
                    1.0 - a_position.y / u_target_size.y * 2.0);
    float z = a_position.z / 65535.0 * 2.0 - 1.0;
    gl_Position = vec4(ndc * w, z * w, w);
    v_color = a_color;
    v_uvq = a_uvqf.xyz;
    v_fog = a_uvqf.w;
}
)";

// Integer-domain helpers follow the software rasterizer's rounding
// (mul8 = (a*b + 127) / 255) so 8-bit results match it.
constexpr const char *kFragmentShader = R"(#version 330 core
in vec4 v_color;
in vec3 v_uvq;
in float v_fog;
uniform sampler2D u_texture;
uniform int u_texture_enabled;
uniform ivec4 u_texture_control;   // function, use alpha, double colour, -
uniform vec3 u_texture_env;
uniform vec2 u_texture_size;
uniform ivec4 u_alpha_control;     // enable, function, reference, mask
uniform int u_fog_enabled;
uniform vec3 u_fog_color;
uniform int u_premultiply;
uniform vec3 u_premultiply_color;
layout(location = 0) out vec4 o_color;

vec4 to8(vec4 c) { return floor(clamp(c, 0.0, 1.0) * 255.0 + 0.5); }
float mul8(float a, float b) { return floor((a * b + 127.0) / 255.0); }
vec3 mul8(vec3 a, vec3 b) { return floor((a * b + 127.0) / 255.0); }

bool compare(int function, int left, int right) {
    if (function == 0) return false;
    if (function == 1) return true;
    if (function == 2) return left == right;
    if (function == 3) return left != right;
    if (function == 4) return left < right;
    if (function == 5) return left <= right;
    if (function == 6) return left > right;
    return left >= right;
}

void main() {
    vec4 vc = to8(v_color);
    vec4 c = vc;
    if (u_texture_enabled != 0) {
        float q = abs(v_uvq.z) < 1.0e-20 ? 1.0 : v_uvq.z;
        vec4 t = to8(texture(u_texture, (v_uvq.xy / q) / u_texture_size));
        int function = u_texture_control.x;
        bool use_alpha = u_texture_control.y != 0;
        if (function == 0) {          // MODULATE
            c = vec4(mul8(vc.rgb, t.rgb), use_alpha ? mul8(vc.a, t.a) : vc.a);
        } else if (function == 1) {   // DECAL
            float a = use_alpha ? t.a : 255.0;
            c = vec4(floor((t.rgb * a + vc.rgb * (255.0 - a) + 127.0) / 255.0), vc.a);
        } else if (function == 2) {   // BLEND
            vec3 env = u_texture_env * 255.0;
            c = vec4(floor((vc.rgb * (255.0 - t.rgb) + env * t.rgb + 127.0) / 255.0),
                     use_alpha ? mul8(vc.a, t.a) : vc.a);
        } else if (function == 3) {   // REPLACE
            c = vec4(t.rgb, use_alpha ? t.a : vc.a);
        } else if (function == 4) {   // ADD
            c = vec4(min(vc.rgb + t.rgb, vec3(255.0)), use_alpha ? mul8(vc.a, t.a) : vc.a);
        } else {
            c = t;
        }
        if (u_texture_control.z != 0) c.rgb = min(c.rgb * 2.0, vec3(255.0));
    }
    if (u_fog_enabled != 0 && v_fog < 1.0) {
        float f = max(v_fog, 0.0);
        vec3 fog = u_fog_color * 255.0;
        c.rgb = floor(fog + (c.rgb - fog) * f + 0.5);
    }
    if (u_alpha_control.x != 0) {
        int a = int(c.a);
        if (!compare(u_alpha_control.y, a & u_alpha_control.w,
                     u_alpha_control.z & u_alpha_control.w)) discard;
    }
    vec4 result = c / 255.0;
    if (u_premultiply != 0) result.rgb *= u_premultiply_color;
    o_color = result;
}
)";

GLuint compile_shader(GLenum type, const char *source, std::string &error) {
    const GLuint shader = api.CreateShader(type);
    api.ShaderSource(shader, 1, &source, nullptr);
    api.CompileShader(shader);
    GLint ok = 0;
    api.GetShaderiv(shader, COMPILE_STATUS, &ok);
    if (ok == 0) {
        GLint length = 0;
        api.GetShaderiv(shader, INFO_LOG_LENGTH, &length);
        std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
        api.GetShaderInfoLog(shader, length, nullptr, log.data());
        error = std::string(type == VERTEX_SHADER ? "vertex" : "fragment") + " shader: " + log;
        api.DeleteShader(shader);
        return 0u;
    }
    return shader;
}

bool build_program(GlBackend &b, std::string &error) {
    const GLuint vs = compile_shader(VERTEX_SHADER, kVertexShader, error);
    if (vs == 0u) return false;
    const GLuint fs = compile_shader(FRAGMENT_SHADER, kFragmentShader, error);
    if (fs == 0u) { api.DeleteShader(vs); return false; }
    b.program = api.CreateProgram();
    api.AttachShader(b.program, vs);
    api.AttachShader(b.program, fs);
    api.LinkProgram(b.program);
    api.DeleteShader(vs);
    api.DeleteShader(fs);
    GLint ok = 0;
    api.GetProgramiv(b.program, LINK_STATUS, &ok);
    if (ok == 0) {
        GLint length = 0;
        api.GetProgramiv(b.program, INFO_LOG_LENGTH, &length);
        std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
        api.GetProgramInfoLog(b.program, length, nullptr, log.data());
        error = "program link: " + log;
        return false;
    }
    Uniforms &u = b.uniforms;
    u.target_size = api.GetUniformLocation(b.program, "u_target_size");
    u.texture_sampler = api.GetUniformLocation(b.program, "u_texture");
    u.texture_enabled = api.GetUniformLocation(b.program, "u_texture_enabled");
    u.texture_control = api.GetUniformLocation(b.program, "u_texture_control");
    u.texture_env = api.GetUniformLocation(b.program, "u_texture_env");
    u.texture_size = api.GetUniformLocation(b.program, "u_texture_size");
    u.alpha_control = api.GetUniformLocation(b.program, "u_alpha_control");
    u.fog_enabled = api.GetUniformLocation(b.program, "u_fog_enabled");
    u.fog_color = api.GetUniformLocation(b.program, "u_fog_color");
    u.premultiply = api.GetUniformLocation(b.program, "u_premultiply");
    u.premultiply_color = api.GetUniformLocation(b.program, "u_premultiply_color");
    return true;
}

// ---- surfaces ----------------------------------------------------------------

void ensure_scratch(Scratch &scratch, std::uint32_t width, std::uint32_t height) {
    if (scratch.texture != 0u && scratch.width == width && scratch.height == height) return;
    if (scratch.texture == 0u) {
        api.GenTextures(1, &scratch.texture);
        api.GenFramebuffers(1, &scratch.fbo);
    }
    api.BindTexture(TEXTURE_2D, scratch.texture);
    api.TexImage2D(TEXTURE_2D, 0, static_cast<GLint>(RGBA8), static_cast<GLsizei>(width),
                   static_cast<GLsizei>(height), 0, RGBA, UNSIGNED_BYTE, nullptr);
    api.TexParameteri(TEXTURE_2D, TEXTURE_MIN_FILTER, static_cast<GLint>(LINEAR));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAG_FILTER, static_cast<GLint>(LINEAR));
    api.TexParameteri(TEXTURE_2D, TEXTURE_WRAP_S, static_cast<GLint>(CLAMP_TO_EDGE));
    api.TexParameteri(TEXTURE_2D, TEXTURE_WRAP_T, static_cast<GLint>(CLAMP_TO_EDGE));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAX_LEVEL, 0);
    api.BindFramebuffer(FRAMEBUFFER, scratch.fbo);
    api.FramebufferTexture2D(FRAMEBUFFER, COLOR_ATTACHMENT0, TEXTURE_2D, scratch.texture, 0);
    scratch.width = width;
    scratch.height = height;
}

void invalidate_bindings(GlBackend &b) {
    b.bound_target = ~0u;
    b.applied_valid = false;
}

// Copies a top-row-first RGBA8 image into `fbo` (which is `width` x `height`
// device pixels), flipping it into GL orientation and scaling.
void upload_rgba_flipped(GlBackend &b, std::span<const std::byte> rgba, std::uint32_t width,
                         std::uint32_t height, GLuint fbo, std::uint32_t fbo_width,
                         std::uint32_t fbo_height, bool linear) {
    ensure_scratch(b.upload, width, height);
    api.BindTexture(TEXTURE_2D, b.upload.texture);
    api.PixelStorei(UNPACK_ALIGNMENT, 1);
    api.TexSubImage2D(TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height),
                      RGBA, UNSIGNED_BYTE, rgba.data());
    api.Disable(SCISSOR_TEST);
    api.BindFramebuffer(READ_FRAMEBUFFER, b.upload.fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, fbo);
    // Source rows are top-first: read them bottom-up to flip.
    api.BlitFramebuffer(0, static_cast<GLint>(height), static_cast<GLint>(width), 0,
                        0, 0, static_cast<GLint>(fbo_width), static_cast<GLint>(fbo_height),
                        COLOR_BUFFER_BIT, linear ? LINEAR : NEAREST);
    invalidate_bindings(b);
}

void upload_target_from_vram(GlBackend &b, Target &target) {
    target.needs_upload = false;
    if (b.memory == nullptr) return;
    try {
        const FramebufferDescription description{0x04000000u | target.address, kTargetWidth, kTargetHeight,
                                                 target.stride, target.format};
        const std::vector<std::byte> rgba = decode_framebuffer_rgba(*b.memory, description);
        upload_rgba_flipped(b, rgba, kTargetWidth, kTargetHeight, target.fbo,
                            kTargetWidth * b.scale, kTargetHeight * b.scale, false);
    } catch (const std::exception &error) {
        runtime_log_error("gl target upload", error.what());
    }
}

Target *find_target(GlBackend &b, std::uint32_t address) {
    const auto found = b.targets.find(target_key(address));
    return found == b.targets.end() ? nullptr : &found->second;
}

Target &ensure_target(GlBackend &b, std::uint32_t address, std::uint32_t stride, std::uint32_t format) {
    const std::uint32_t key = target_key(address);
    auto [it, inserted] = b.targets.try_emplace(key);
    Target &target = it->second;
    if (stride != 0u) target.stride = stride;
    target.format = format;
    if (!inserted) return target;
    target.address = key;
    const auto width = static_cast<GLsizei>(kTargetWidth * b.scale);
    const auto height = static_cast<GLsizei>(kTargetHeight * b.scale);
    api.GenTextures(1, &target.color);
    api.BindTexture(TEXTURE_2D, target.color);
    api.TexImage2D(TEXTURE_2D, 0, static_cast<GLint>(RGBA8), width, height, 0, RGBA, UNSIGNED_BYTE, nullptr);
    api.TexParameteri(TEXTURE_2D, TEXTURE_MIN_FILTER, static_cast<GLint>(LINEAR));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAG_FILTER, static_cast<GLint>(LINEAR));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAX_LEVEL, 0);
    api.GenRenderbuffers(1, &target.depth);
    api.BindRenderbuffer(RENDERBUFFER, target.depth);
    api.RenderbufferStorage(RENDERBUFFER, DEPTH_COMPONENT24, width, height);
    api.GenFramebuffers(1, &target.fbo);
    api.BindFramebuffer(FRAMEBUFFER, target.fbo);
    api.FramebufferTexture2D(FRAMEBUFFER, COLOR_ATTACHMENT0, TEXTURE_2D, target.color, 0);
    api.FramebufferRenderbuffer(FRAMEBUFFER, DEPTH_ATTACHMENT, RENDERBUFFER, target.depth);
    if (api.CheckFramebufferStatus(FRAMEBUFFER) != FRAMEBUFFER_COMPLETE)
        runtime_log_error("gl", "incomplete framebuffer for target " + std::to_string(key));
    api.Disable(SCISSOR_TEST);
    api.DepthMask(BOOL_TRUE);
    api.ClearDepth(0.0);
    api.Clear(DEPTH_BUFFER_BIT);
    invalidate_bindings(b);
    ++b.report.framebuffer_targets_observed;
    std::cerr << "[gl] render target 0x" << std::hex << key << std::dec << " " << width << "x" << height
              << " (scale " << b.scale << ")\n";
    return target;
}

// ---- state translation --------------------------------------------------------

GLenum blend_factor(std::uint32_t factor, bool source) noexcept {
    switch (factor) {
    case 0u: return source ? DST_COLOR : SRC_COLOR;
    case 1u: return source ? ONE_MINUS_DST_COLOR : ONE_MINUS_SRC_COLOR;
    case 2u: return SRC_ALPHA;
    case 3u: return ONE_MINUS_SRC_ALPHA;
    case 4u: return DST_ALPHA;
    case 5u: return ONE_MINUS_DST_ALPHA;
    // 2x factors have no GL equivalent; the 1x factor is the closest.
    case 6u: return SRC_ALPHA;
    case 7u: return ONE_MINUS_SRC_ALPHA;
    case 8u: return DST_ALPHA;
    case 9u: return ONE_MINUS_DST_ALPHA;
    default: return ONE;
    }
}

GLenum blend_equation(std::uint32_t equation) noexcept {
    switch (equation) {
    case 1u: return FUNC_SUBTRACT;
    case 2u: return FUNC_REVERSE_SUBTRACT;
    case 3u: return BLEND_MIN;
    case 4u: return BLEND_MAX;
    case 5u: return BLEND_MAX;  // ABS(src - dst): no GL equation
    default: return FUNC_ADD;
    }
}

DrawState make_state(GlBackend &b, const GeGpuDrawDescriptor &draw) {
    DrawState state{};
    state.target = target_key(draw.framebuffer_address);
    if (draw.texture_enabled) {
        if (Texture *texture = find_texture(b, texture_key(draw)); texture != nullptr && texture->name != 0u) {
            state.texture = texture->name;
            state.texture_width = static_cast<float>(std::max(1u, draw.texture_width));
            state.texture_height = static_cast<float>(std::max(1u, draw.texture_height));
            texture->pending_flush = b.flush_serial;
            texture->last_used_epoch = b.frame_epoch;
        }
        state.texture_function = draw.texture_function;
        state.texture_use_alpha = draw.texture_use_alpha ? 1u : 0u;
        state.texture_double = draw.texture_double_color ? 1u : 0u;
        state.texture_env = draw.texture_env & 0x00FFFFFFu;
    }
    if (draw.alpha_test_enabled) {
        state.alpha_test = 1u;
        state.alpha_function = draw.alpha_function & 7u;
        state.alpha_reference = draw.alpha_reference & 0xFFu;
        state.alpha_mask = draw.alpha_mask & 0xFFu;
    }
    if (draw.fog_enabled) {
        state.fog = 1u;
        state.fog_color = draw.fog_color & 0x00FFFFFFu;
    }
    if (draw.blend_enabled) {
        state.blend = 1u;
        state.blend_equation = blend_equation(draw.blend_equation & 7u);
        const std::uint32_t source = draw.blend_source_factor & 0xFu;
        const std::uint32_t dest = draw.blend_dest_factor & 0xFu;
        // Fixed factors: white/black become ONE/ZERO; a fixed destination
        // colour uses the GL blend constant, and a fixed source colour is
        // multiplied into the fragment so the two never compete for it.
        state.blend_source = blend_factor(source, true);
        state.blend_dest = blend_factor(dest, false);
        if (source >= 10u) {
            const std::uint32_t fix = draw.blend_fix_source & 0x00FFFFFFu;
            state.blend_source = ONE;
            if (fix != 0x00FFFFFFu) {
                state.source_premultiply = 1u;
                state.premultiply_color = fix;
            }
        }
        if (dest >= 10u) {
            const std::uint32_t fix = draw.blend_fix_dest & 0x00FFFFFFu;
            if (fix == 0x00FFFFFFu) state.blend_dest = ONE;
            else if (fix == 0u) state.blend_dest = ZERO;
            else { state.blend_dest = CONSTANT_COLOR; state.blend_color = fix; }
        }
    }
    // The GE writes Z only while the depth test is on (as the software
    // rasterizer does), except for clear-mode rectangles, which write it with
    // the test off; GL needs the test enabled (ALWAYS) to write at all.
    if (draw.clear_mode) {
        if (draw.depth_write_enabled) {
            state.depth_test = 1u;
            state.depth_function = 1u;
            state.depth_write = 1u;
        }
    } else if (draw.depth_test_enabled) {
        state.depth_test = 1u;
        state.depth_function = draw.depth_function & 7u;
        state.depth_write = draw.depth_write_enabled ? 1u : 0u;
    }
    // PSP mask bytes: 0x00 writable, 0xFF masked. Partial masks keep the
    // channel writable.
    const std::uint32_t mask = draw.color_write_mask;
    state.color_mask = ((mask & 0xFFu) != 0xFFu ? 1u : 0u) |
                       (((mask >> 8u) & 0xFFu) != 0xFFu ? 2u : 0u) |
                       (((mask >> 16u) & 0xFFu) != 0xFFu ? 4u : 0u) |
                       (((mask >> 24u) & 0xFFu) != 0xFFu ? 8u : 0u);
    state.scissor_x0 = std::clamp(draw.scissor_x0, 0, static_cast<int>(kTargetWidth));
    state.scissor_y0 = std::clamp(draw.scissor_y0, 0, static_cast<int>(kTargetHeight));
    state.scissor_x1 = std::clamp(draw.scissor_x1, -1, static_cast<int>(kTargetWidth) - 1);
    state.scissor_y1 = std::clamp(draw.scissor_y1, -1, static_cast<int>(kTargetHeight) - 1);
    return state;
}

void set_color(GLint location, std::uint32_t rgb) {
    api.Uniform3f(location, static_cast<float>(rgb & 0xFFu) / 255.0f,
                  static_cast<float>((rgb >> 8u) & 0xFFu) / 255.0f,
                  static_cast<float>((rgb >> 16u) & 0xFFu) / 255.0f);
}

constexpr std::array<GLenum, 8> kDepthFunctions{0x0200u, 0x0207u, 0x0202u, 0x0205u,
                                                0x0201u, 0x0203u, 0x0204u, 0x0206u};

void apply_state(GlBackend &b, const DrawState &state) {
    const DrawState &old = b.applied;
    const bool all = !b.applied_valid;
    if (all || b.bound_target != state.target) {
        Target *target = find_target(b, state.target);
        api.BindFramebuffer(FRAMEBUFFER, target != nullptr ? target->fbo : 0u);
        api.Viewport(0, 0, static_cast<GLsizei>(kTargetWidth * b.scale),
                     static_cast<GLsizei>(kTargetHeight * b.scale));
        b.bound_target = state.target;
    }
    if (all) {
        api.UseProgram(b.program);
        api.BindVertexArray(b.vao);
        api.Uniform2f(b.uniforms.target_size, static_cast<float>(kTargetWidth),
                      static_cast<float>(kTargetHeight));
        api.Uniform1i(b.uniforms.texture_sampler, 0);
        api.ActiveTexture(TEXTURE0);
        api.Enable(SCISSOR_TEST);
        api.Enable(DEPTH_CLAMP);
        api.Disable(CULL_FACE);
        api.Disable(DITHER);
    }
    if (all || old.texture != state.texture) {
        api.BindTexture(TEXTURE_2D, state.texture != 0u ? state.texture : b.white_texture);
        api.Uniform1i(b.uniforms.texture_enabled, state.texture != 0u ? 1 : 0);
    }
    if (all || old.texture_function != state.texture_function || old.texture_use_alpha != state.texture_use_alpha ||
        old.texture_double != state.texture_double)
        api.Uniform4i(b.uniforms.texture_control, static_cast<GLint>(state.texture_function),
                      static_cast<GLint>(state.texture_use_alpha), static_cast<GLint>(state.texture_double), 0);
    if (all || old.texture_env != state.texture_env) set_color(b.uniforms.texture_env, state.texture_env);
    if (all || old.texture_width != state.texture_width || old.texture_height != state.texture_height)
        api.Uniform2f(b.uniforms.texture_size, state.texture_width, state.texture_height);
    if (all || old.alpha_test != state.alpha_test || old.alpha_function != state.alpha_function ||
        old.alpha_reference != state.alpha_reference || old.alpha_mask != state.alpha_mask)
        api.Uniform4i(b.uniforms.alpha_control, static_cast<GLint>(state.alpha_test),
                      static_cast<GLint>(state.alpha_function), static_cast<GLint>(state.alpha_reference),
                      static_cast<GLint>(state.alpha_mask));
    if (all || old.fog != state.fog) api.Uniform1i(b.uniforms.fog_enabled, static_cast<GLint>(state.fog));
    if (all || old.fog_color != state.fog_color) set_color(b.uniforms.fog_color, state.fog_color);
    if (all || old.source_premultiply != state.source_premultiply)
        api.Uniform1i(b.uniforms.premultiply, static_cast<GLint>(state.source_premultiply));
    if (all || old.premultiply_color != state.premultiply_color)
        set_color(b.uniforms.premultiply_color, state.premultiply_color);
    if (all || old.blend != state.blend) (state.blend != 0u ? api.Enable : api.Disable)(BLEND);
    if (state.blend != 0u) {
        if (all || old.blend_equation != state.blend_equation) api.BlendEquation(state.blend_equation);
        if (all || old.blend_source != state.blend_source || old.blend_dest != state.blend_dest)
            api.BlendFunc(state.blend_source, state.blend_dest);
        if (all || old.blend_color != state.blend_color)
            api.BlendColor(static_cast<float>(state.blend_color & 0xFFu) / 255.0f,
                           static_cast<float>((state.blend_color >> 8u) & 0xFFu) / 255.0f,
                           static_cast<float>((state.blend_color >> 16u) & 0xFFu) / 255.0f, 1.0f);
    }
    if (all || old.depth_test != state.depth_test) (state.depth_test != 0u ? api.Enable : api.Disable)(DEPTH_TEST);
    if (all || old.depth_function != state.depth_function) api.DepthFunc(kDepthFunctions[state.depth_function & 7u]);
    if (all || old.depth_write != state.depth_write) api.DepthMask(state.depth_write != 0u ? BOOL_TRUE : BOOL_FALSE);
    if (all || old.color_mask != state.color_mask)
        api.ColorMask((state.color_mask & 1u) != 0u, (state.color_mask & 2u) != 0u,
                      (state.color_mask & 4u) != 0u, (state.color_mask & 8u) != 0u);
    if (all || old.scissor_x0 != state.scissor_x0 || old.scissor_y0 != state.scissor_y0 ||
        old.scissor_x1 != state.scissor_x1 || old.scissor_y1 != state.scissor_y1) {
        const std::int32_t width = std::max(0, state.scissor_x1 - state.scissor_x0 + 1);
        const std::int32_t height = std::max(0, state.scissor_y1 - state.scissor_y0 + 1);
        const auto s = static_cast<std::int32_t>(b.scale);
        // GL scissor rows count from the bottom.
        api.Scissor(state.scissor_x0 * s,
                    (static_cast<std::int32_t>(kTargetHeight) - 1 - state.scissor_y1) * s,
                    width * s, height * s);
    }
    b.applied = state;
    b.applied_valid = true;
}

void flush(GlBackend &b) {
    ++b.flush_serial;
    if (b.batches.empty()) return;
    api.BindBuffer(ARRAY_BUFFER, b.vbo);
    api.BufferData(ARRAY_BUFFER, static_cast<GLsizeiptr>(b.vertices.size() * sizeof(GlVertex)),
                   b.vertices.data(), STREAM_DRAW);
    invalidate_bindings(b);
    for (const Batch &batch : b.batches) {
        apply_state(b, batch.state);
        api.DrawArrays(TRIANGLES, static_cast<GLint>(batch.first), static_cast<GLsizei>(batch.count));
    }
    b.report.game_draw_calls += b.batches.size();
    b.vertices.clear();
    b.batches.clear();
}

void evict_textures(GlBackend &b) {
    while (b.texture_bytes > b.texture_budget) {
        auto oldest = b.textures.end();
        for (auto it = b.textures.begin(); it != b.textures.end(); ++it) {
            if (it->second.last_used_epoch == b.frame_epoch) continue;
            if (oldest == b.textures.end() || it->second.last_used_epoch < oldest->second.last_used_epoch)
                oldest = it;
        }
        if (oldest == b.textures.end()) return;
        api.DeleteTextures(1, &oldest->second.name);
        b.texture_bytes -= oldest->second.bytes;
        b.textures.erase(oldest);
        ++b.report.evicted_textures;
    }
}

// Letterboxed destination rectangle for a 480:272 image in the drawable.
void letterbox(int drawable_width, int drawable_height, int &x, int &y, int &width, int &height) {
    const double aspect = static_cast<double>(kTargetWidth) / kTargetHeight;
    width = drawable_width;
    height = static_cast<int>(std::lround(drawable_width / aspect));
    if (height > drawable_height) {
        height = drawable_height;
        width = static_cast<int>(std::lround(drawable_height * aspect));
    }
    x = (drawable_width - width) / 2;
    y = (drawable_height - height) / 2;
}

void clear_default_framebuffer(int drawable_width, int drawable_height) {
    api.BindFramebuffer(FRAMEBUFFER, 0u);
    api.Disable(SCISSOR_TEST);
    api.ColorMask(BOOL_TRUE, BOOL_TRUE, BOOL_TRUE, BOOL_TRUE);
    api.Viewport(0, 0, drawable_width, drawable_height);
    api.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    api.Clear(COLOR_BUFFER_BIT);
}

} // namespace

// ---- lifecycle ------------------------------------------------------------------

bool initialize_ge_gpu_backend(std::string &error) {
    GlBackend &b = backend();
    if (b.active) return true;
    if (!gl::load(&SDL_GL_GetProcAddress, error)) return false;
    if (const char *text = std::getenv("PES6_RENDER_SCALE"); text != nullptr && *text != '\0')
        b.scale = static_cast<std::uint32_t>(std::clamp(std::atoi(text), 1, 8));
    if (!build_program(b, error)) return false;
    api.GenVertexArrays(1, &b.vao);
    api.GenBuffers(1, &b.vbo);
    api.BindVertexArray(b.vao);
    api.BindBuffer(ARRAY_BUFFER, b.vbo);
    api.EnableVertexAttribArray(0);
    api.VertexAttribPointer(0, 4, FLOAT, BOOL_FALSE, sizeof(GlVertex), reinterpret_cast<const void *>(0));
    api.EnableVertexAttribArray(1);
    api.VertexAttribPointer(1, 4, UNSIGNED_BYTE, BOOL_TRUE, sizeof(GlVertex),
                            reinterpret_cast<const void *>(offsetof(GlVertex, rgba)));
    api.EnableVertexAttribArray(2);
    api.VertexAttribPointer(2, 4, FLOAT, BOOL_FALSE, sizeof(GlVertex),
                            reinterpret_cast<const void *>(offsetof(GlVertex, u)));
    const std::uint32_t white = 0xFFFFFFFFu;
    api.GenTextures(1, &b.white_texture);
    api.BindTexture(TEXTURE_2D, b.white_texture);
    api.TexImage2D(TEXTURE_2D, 0, static_cast<GLint>(RGBA8), 1, 1, 0, RGBA, UNSIGNED_BYTE, &white);
    api.TexParameteri(TEXTURE_2D, TEXTURE_MIN_FILTER, static_cast<GLint>(NEAREST));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAG_FILTER, static_cast<GLint>(NEAREST));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAX_LEVEL, 0);
    b.report.requested = GeGpuBackendKind::OpenGL;
    b.report.active = GeGpuBackendKind::OpenGL;
    b.report.message = std::string("OpenGL ") + reinterpret_cast<const char *>(api.GetString(VERSION)) +
                       " on " + reinterpret_cast<const char *>(api.GetString(RENDERER));
    b.active = true;
    std::cerr << "[gl] " << b.report.message << ", internal resolution " << kTargetWidth * b.scale << "x"
              << kTargetHeight * b.scale << "\n";
    return true;
}

void shutdown_ge_gpu_backend() noexcept {
    GlBackend &b = backend();
    if (!b.active) return;
    std::cerr << "[gl] draws=" << b.report.draw_calls << " batches=" << b.report.game_draw_calls
              << " textures=" << b.textures.size() << " (" << (b.texture_bytes >> 20u) << " MiB)"
              << " uploads=" << b.report.texture_image_uploads << " evicted=" << b.report.evicted_textures
              << " untextured_fallbacks=" << b.report.missing_texture_draw_calls << "\n";
    b.active = false;
}

bool ge_gpu_backend_active() noexcept { return backend().active; }
bool ge_gpu_backend_transfer_ready() noexcept { return false; }
bool ge_gpu_backend_graphics_ready() noexcept { return backend().active; }
void ge_gpu_backend_attach_memory(const psprecomp::GuestMemory *memory) noexcept { backend().memory = memory; }

void ge_gpu_backend_record_draw(const GeGpuDrawDescriptor &draw) noexcept {
    GlBackend &b = backend();
    if (!b.active) return;
    ++b.report.draw_calls;
    b.report.vertices += draw.vertex_count;
    if (draw.texture_enabled) ++b.report.textured_draw_calls;
    ensure_target(b, draw.framebuffer_address, draw.framebuffer_stride, draw.framebuffer_format);
}

// ---- textures -------------------------------------------------------------------

bool ge_gpu_backend_texture_needed(const GeGpuDrawDescriptor &draw) noexcept {
    GlBackend &b = backend();
    if (!b.active || !draw.texture_enabled || draw.texture_format > 10u || draw.texture_width == 0u ||
        draw.texture_height == 0u) return false;
    ++b.report.texture_decode_requests;
    Texture *found = find_texture(b, texture_key(draw));
    if (found == nullptr) return true;
    found->signature_epoch = b.frame_epoch;
    found->last_used_epoch = b.frame_epoch;
    if (draw.texture_content_signature != 0u && found->signature != draw.texture_content_signature) return true;
    ++b.report.texture_cache_hits;
    return false;
}

void ge_gpu_backend_prepare_texture_keys(GeGpuDrawDescriptor &draw) noexcept {
    draw.texture_cache_key_hint = 0u;
    draw.texture_image_key_hint = 0u;
    if (!draw.texture_enabled) return;
    draw.texture_cache_key_hint = texture_key(draw);
    draw.texture_image_key_hint = draw.texture_cache_key_hint;
}

bool ge_gpu_backend_texture_signature_needed(const GeGpuDrawDescriptor &draw) noexcept {
    GlBackend &b = backend();
    if (!b.active || !draw.texture_enabled || draw.texture_width == 0u || draw.texture_height == 0u)
        return false;
    // Once per texture per frame: the guest can stream new texels into the
    // same address (the T8 textures PES6 keeps in VRAM).
    Texture *found = find_texture(b, texture_key(draw));
    return found == nullptr || found->signature_epoch != b.frame_epoch;
}

bool ge_gpu_backend_is_framebuffer_feedback_texture(const GeGpuDrawDescriptor &) noexcept { return false; }
void ge_gpu_backend_note_through_extent(const GeGpuDrawDescriptor &, float, float) noexcept {}
bool ge_gpu_backend_adopt_shared_texture(const GeGpuDrawDescriptor &) noexcept { return false; }

bool ge_gpu_backend_texture_available(const GeGpuDrawDescriptor &draw) noexcept {
    GlBackend &b = backend();
    if (!b.active || !draw.texture_enabled) return false;
    const Texture *found = find_texture(b, texture_key(draw));
    return found != nullptr && found->name != 0u;
}

bool ge_gpu_backend_upload_decoded_texture_chain_packed(const GeGpuDrawDescriptor &draw, std::uint32_t width,
                                                        std::uint32_t height, std::uint32_t mip_levels,
                                                        std::vector<std::byte> rgba8) noexcept {
    GlBackend &b = backend();
    if (!b.active || width == 0u || height == 0u || mip_levels == 0u || rgba8.empty()) return false;
    const std::uint64_t key = texture_key(draw);
    Texture &texture = b.textures[key];
    // Pending draws sample the old texels: submit them first.
    if (texture.name != 0u && texture.pending_flush == b.flush_serial) flush(b);
    if (texture.name == 0u) api.GenTextures(1, &texture.name);
    api.BindTexture(TEXTURE_2D, texture.name);
    api.PixelStorei(UNPACK_ALIGNMENT, 1);
    std::size_t offset = 0u;
    std::uint32_t level_width = width;
    std::uint32_t level_height = height;
    std::uint32_t uploaded = 0u;
    for (; uploaded < mip_levels; ++uploaded) {
        const std::size_t bytes = static_cast<std::size_t>(level_width) * level_height * 4u;
        if (offset + bytes > rgba8.size()) break;
        api.TexImage2D(TEXTURE_2D, static_cast<GLint>(uploaded), static_cast<GLint>(RGBA8),
                       static_cast<GLsizei>(level_width), static_cast<GLsizei>(level_height), 0, RGBA,
                       UNSIGNED_BYTE, rgba8.data() + offset);
        offset += bytes;
        level_width = std::max(1u, level_width >> 1u);
        level_height = std::max(1u, level_height >> 1u);
    }
    const bool mipmapped = uploaded > 1u && draw.texture_mipmap_enabled;
    const GLenum min_filter = mipmapped
        ? (draw.texture_min_linear ? (draw.texture_mipmap_linear ? LINEAR_MIPMAP_LINEAR : LINEAR_MIPMAP_NEAREST)
                                   : (draw.texture_mipmap_linear ? NEAREST_MIPMAP_LINEAR : NEAREST_MIPMAP_NEAREST))
        : (draw.texture_min_linear ? LINEAR : NEAREST);
    api.TexParameteri(TEXTURE_2D, TEXTURE_MIN_FILTER, static_cast<GLint>(min_filter));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAG_FILTER,
                      static_cast<GLint>(draw.texture_mag_linear ? LINEAR : NEAREST));
    api.TexParameteri(TEXTURE_2D, TEXTURE_WRAP_S, static_cast<GLint>(draw.texture_clamp_u ? CLAMP_TO_EDGE : REPEAT));
    api.TexParameteri(TEXTURE_2D, TEXTURE_WRAP_T, static_cast<GLint>(draw.texture_clamp_v ? CLAMP_TO_EDGE : REPEAT));
    api.TexParameteri(TEXTURE_2D, TEXTURE_BASE_LEVEL, 0);
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAX_LEVEL, static_cast<GLint>(mipmapped ? uploaded - 1u : 0u));
    b.texture_bytes -= texture.bytes;
    texture.bytes = offset;
    b.texture_bytes += texture.bytes;
    texture.width = width;
    texture.height = height;
    texture.signature = draw.texture_content_signature;
    texture.signature_epoch = b.frame_epoch;
    texture.last_used_epoch = b.frame_epoch;
    b.applied_valid = false;  // the texture binding changed
    ++b.report.texture_image_uploads;
    b.report.texture_image_upload_bytes += offset;
    return true;
}

bool ge_gpu_backend_upload_decoded_texture(const GeGpuDrawDescriptor &draw, std::uint32_t width,
                                           std::uint32_t height, std::span<const std::byte> rgba8) noexcept {
    if (rgba8.empty()) return false;
    return ge_gpu_backend_upload_decoded_texture_chain_packed(
        draw, width, height, 1u, std::vector<std::byte>(rgba8.begin(), rgba8.end()));
}

bool ge_gpu_backend_upload_decoded_texture_chain(const GeGpuDrawDescriptor &draw,
                                                 std::span<const GeGpuDecodedMipLevel> levels) noexcept {
    if (levels.empty()) return false;
    std::vector<std::byte> packed;
    for (const auto &level : levels) packed.insert(packed.end(), level.rgba8.begin(), level.rgba8.end());
    return ge_gpu_backend_upload_decoded_texture_chain_packed(draw, levels.front().width, levels.front().height,
                                                              static_cast<std::uint32_t>(levels.size()),
                                                              std::move(packed));
}

bool ge_gpu_backend_copy_last_texture_rgba(std::span<std::byte>) noexcept { return false; }

// ---- geometry -------------------------------------------------------------------

bool ge_gpu_backend_stage_vertices(const GeGpuDrawDescriptor &, std::span<const GeGpuVertex>) noexcept {
    return false;
}

void ge_gpu_backend_accumulate_color_triangles(const GeGpuDrawDescriptor &draw,
                                               std::span<const GeGpuVertex> triangle_vertices) noexcept {
    GlBackend &b = backend();
    if (!b.active || triangle_vertices.size() < 3u) return;
    Target &target = ensure_target(b, draw.framebuffer_address, draw.framebuffer_stride, draw.framebuffer_format);
    if (target.needs_upload) {
        // Draw on top of what the CPU left in VRAM, in order with any batch
        // already queued for this target.
        flush(b);
        upload_target_from_vram(b, target);
    }
    target.gpu_valid = true;
    const DrawState state = make_state(b, draw);
    if (draw.texture_enabled && state.texture == 0u) ++b.report.missing_texture_draw_calls;
    const std::size_t count = triangle_vertices.size() - triangle_vertices.size() % 3u;
    const auto first = static_cast<std::uint32_t>(b.vertices.size());
    b.vertices.reserve(b.vertices.size() + count);
    for (std::size_t i = 0u; i < count; ++i) {
        const GeGpuVertex &v = triangle_vertices[i];
        b.vertices.push_back({v.x, v.y, v.z, v.w, v.rgba, v.u, v.v, v.q, v.fog_factor});
    }
    if (!b.batches.empty() && same_state(b.batches.back().state, state) &&
        b.batches.back().first + b.batches.back().count == first) {
        b.batches.back().count += static_cast<std::uint32_t>(count);
    } else {
        b.batches.push_back({state, first, static_cast<std::uint32_t>(count)});
    }
    b.report.game_triangles += count / 3u;
    b.report.game_vertices += count;
}

void ge_gpu_backend_accumulate_hardware_triangles(const GeGpuDrawDescriptor &, const GeGpuHardwareTransform &,
                                                  std::span<const GeGpuVertex>,
                                                  std::span<const std::uint32_t>) noexcept {}
bool ge_gpu_backend_accumulate_hardware_packed_0115(const GeGpuDrawDescriptor &, const GeGpuHardwareTransform &,
                                                    std::span<const std::byte>, std::uint32_t,
                                                    std::span<const std::uint32_t>) noexcept {
    return false;
}

// ---- framebuffers ---------------------------------------------------------------

void ge_gpu_backend_set_native_window(void *) noexcept {}

void ge_gpu_backend_set_display_framebuffer(std::uint32_t address) noexcept {
    backend().display_framebuffer = target_key(address);
}

bool ge_gpu_backend_finish_color_frame(std::uint64_t vblank) noexcept {
    GlBackend &b = backend();
    if (!b.active) return false;
    flush(b);
    evict_textures(b);
    ++b.frame_epoch;
    ++b.report.game_frames;
    b.report.game_frame_vblank = vblank;
    return true;
}

bool ge_gpu_backend_owns_framebuffer(std::uint32_t address) noexcept {
    GlBackend &b = backend();
    return b.active && find_target(b, address) != nullptr;
}

void ge_gpu_backend_invalidate_framebuffer(std::uint32_t address, std::uint32_t size) noexcept {
    GlBackend &b = backend();
    if (!b.active || size == 0u || (address & 0x0F000000u) != 0x04000000u) return;  // VRAM only
    const std::uint32_t begin = target_key(address);
    const std::uint32_t end = begin + size;
    for (auto &[key, target] : b.targets) {
        const std::uint32_t bytes = target.stride * kTargetHeight * (target.format == 3u ? 4u : 2u);
        if (begin < key + bytes && key < end) {
            target.gpu_valid = false;
            target.needs_upload = true;
        }
    }
}

bool ge_gpu_backend_read_framebuffer_rgba(std::uint32_t address, std::uint32_t width, std::uint32_t height,
                                          std::vector<std::byte> &rgba) noexcept {
    GlBackend &b = backend();
    Target *target = b.active ? find_target(b, address) : nullptr;
    if (target == nullptr || !target->gpu_valid || width == 0u || height == 0u) return false;
    flush(b);
    ensure_scratch(b.readback, width, height);
    api.Disable(SCISSOR_TEST);
    api.BindFramebuffer(READ_FRAMEBUFFER, target->fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, b.readback.fbo);
    // Flip so that the first row read back is the PSP's top row.
    const auto source_width = static_cast<GLint>(std::min(width, kTargetWidth) * b.scale);
    const auto source_height = static_cast<GLint>(std::min(height, kTargetHeight) * b.scale);
    api.BlitFramebuffer(0, static_cast<GLint>(kTargetHeight * b.scale) - source_height, source_width,
                        static_cast<GLint>(kTargetHeight * b.scale), 0, static_cast<GLint>(height),
                        static_cast<GLint>(width), 0, COLOR_BUFFER_BIT, b.scale == 1u ? NEAREST : LINEAR);
    api.BindFramebuffer(READ_FRAMEBUFFER, b.readback.fbo);
    api.PixelStorei(PACK_ALIGNMENT, 1);
    rgba.resize(static_cast<std::size_t>(width) * height * 4u);
    api.ReadPixels(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height), RGBA, UNSIGNED_BYTE,
                   rgba.data());
    invalidate_bindings(b);
    b.report.game_frame_readback_bytes += rgba.size();
    return true;
}

bool ge_gpu_backend_present_framebuffer(std::uint32_t address, int drawable_width, int drawable_height) noexcept {
    GlBackend &b = backend();
    Target *target = b.active ? find_target(b, address) : nullptr;
    if (target == nullptr || !target->gpu_valid) return false;
    flush(b);
    clear_default_framebuffer(drawable_width, drawable_height);
    int x = 0, y = 0, width = 0, height = 0;
    letterbox(drawable_width, drawable_height, x, y, width, height);
    api.BindFramebuffer(READ_FRAMEBUFFER, target->fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, 0u);
    api.BlitFramebuffer(0, 0, static_cast<GLint>(kTargetWidth * b.scale), static_cast<GLint>(kTargetHeight * b.scale),
                        x, y, x + width, y + height, COLOR_BUFFER_BIT, LINEAR);
    invalidate_bindings(b);
    b.report.gpu_frame_presented_to_window = true;
    return true;
}

void ge_gpu_backend_present_rgba(std::span<const std::byte> rgba, std::uint32_t width, std::uint32_t height,
                                 int drawable_width, int drawable_height) noexcept {
    GlBackend &b = backend();
    if (!b.active || width == 0u || height == 0u ||
        rgba.size() < static_cast<std::size_t>(width) * height * 4u) return;
    flush(b);
    ensure_scratch(b.present, width, height);
    api.BindTexture(TEXTURE_2D, b.present.texture);
    api.PixelStorei(UNPACK_ALIGNMENT, 1);
    api.TexSubImage2D(TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height), RGBA,
                      UNSIGNED_BYTE, rgba.data());
    clear_default_framebuffer(drawable_width, drawable_height);
    int x = 0, y = 0, w = 0, h = 0;
    letterbox(drawable_width, drawable_height, x, y, w, h);
    api.BindFramebuffer(READ_FRAMEBUFFER, b.present.fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, 0u);
    // Top-first rows: read bottom-up to flip.
    api.BlitFramebuffer(0, static_cast<GLint>(height), static_cast<GLint>(width), 0, x, y, x + w, y + h,
                        COLOR_BUFFER_BIT, LINEAR);
    invalidate_bindings(b);
}

bool ge_gpu_backend_copy_game_frame_rgba(std::span<std::byte>) noexcept { return false; }
bool ge_gpu_backend_presents_directly() noexcept { return false; }
std::uint32_t ge_gpu_backend_owned_framebuffer() noexcept { return 0u; }
std::uint32_t ge_gpu_backend_display_framebuffer() noexcept { return backend().display_framebuffer; }
std::span<const std::byte> ge_gpu_backend_game_frame_rgba() noexcept { return {}; }
bool ge_gpu_backend_copy_offscreen_rgba(std::span<std::byte>) noexcept { return false; }
void ge_gpu_backend_mark_window_presented() noexcept {}
GeGpuBackendReport ge_gpu_backend_report() { return backend().report; }

const char *ge_gpu_backend_name(GeGpuBackendKind kind) noexcept {
    switch (kind) {
    case GeGpuBackendKind::Software: return "software";
    case GeGpuBackendKind::DirectX12: return "directx12";
    case GeGpuBackendKind::OpenGL: return "opengl";
    }
    return "unknown";
}

} // namespace pes6
