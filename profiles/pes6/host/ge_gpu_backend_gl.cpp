// OpenGL 3.3 core GE backend (desktop and Switch).
//
// The renderer (ge_renderer.cpp) keeps decoding GE lists, vertex formats,
// skinning and textures on the CPU; this backend does the per-pixel work (~95 %
// of a frame on the CPU rasterizer) and, for projected triangles, the vertex
// transform, clipping, culling, fog and directional lighting. Its inputs are
// the hooks ge_gpu_backend.hpp already defines for VCS's DX12 backend:
//   - record_draw: one per PRIM, names the render target;
//   - texture_needed / upload_decoded_texture_chain_packed / texture_available:
//     RGBA8 texture cache keyed by the full PSP texture state;
//   - accumulate_color_triangles: clipped screen-space triangles (PSP pixels,
//     Z in 0..65535, clip W for perspective-correct interpolation);
//   - accumulate_hardware_triangles: model/world-space vertices plus the draw's
//     transform (PSPRECOMP_GE_GPU_HW_TRANSFORM). The vertex shader applies
//     model->clip, the viewport and fog, and clips against the projection
//     frustum with gl_ClipDistance exactly where the CPU clipper would; the
//     per-draw records live in a texture buffer indexed by a per-vertex draw id
//     so batching by state is unchanged.
//
// Record and replay. Those hooks run on whichever thread executes the GE --
// the emulation thread, or the GE worker (PSPRECOMP_GE_ASYNC) -- while the GL
// context lives on the emulation thread. So the hooks never call GL: the
// Recorder keeps the texture and render-target caches as metadata, batches
// draws per state and queues everything that needs GL (target creation and
// CPU uploads, decoded texture uploads, evictions, batched geometry) as
// commands, in GE order. ge_gpu_backend_seal_recording(), called by the HLE
// while the GE is idle, closes the frame and hands its commands to the Player,
// which runs them on the GL thread at the next finish_color_frame (vblank),
// present or readback. Texture names are the Recorder's ids; the Player maps
// them to GL names.
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
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>
#include <variant>
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
    // 0: screen-space vertex. n > 0: model/world-space vertex whose draw
    // record starts at texel n - 1 of the frame's transform buffer.
    std::uint32_t draw;
    float nx, ny, nz;  // model-space normal, GPU-lit draws only
};
static_assert(sizeof(GlVertex) == 52u);

// One hardware-transform draw in the texture buffer, as RGBA32F texels:
// model->clip (4 columns), viewport scale (xyz, depth clip flag), viewport
// centre minus offset (xyz), model->view Z row, fog (end, slope, lit flag),
// UV scale and offset. A GPU-lit draw appends the world 3x3 (columns, the
// first W = reverse normals), the base colour and four lights (direction with
// W = enabled, diffuse). Must match kVertexShader.
constexpr std::uint32_t kTransformTexels = 9u;
constexpr std::uint32_t kLightingTexels = 12u;

// Everything that differs between batches. All 32-bit fields: compared and
// hashed as raw bytes, so there must be no padding.
struct DrawState {
    std::uint32_t target{};
    std::uint32_t texture{};  // Recorder texture id, 0 = untextured
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
    // Face culling, hardware-transform draws only (the CPU culls the
    // screen-space ones): 0 off, 1 keep GL-counter-clockwise, 2 keep GL-clockwise.
    std::uint32_t cull{};
};
static_assert(sizeof(DrawState) == 30u * 4u);

bool same_state(const DrawState &a, const DrawState &b) noexcept {
    return std::memcmp(&a, &b, sizeof(DrawState)) == 0;
}

struct Batch {
    DrawState state;
    std::uint32_t first{};  // in the index stream
    std::uint32_t count{};
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
    GLint transforms{-1};
};

// Vertex, index and draw-record buffers for one flush. Flushes rotate through
// kStreamSets of them, and each set is only grown, never re-specified: the
// first console builds called BufferData on the same three buffers at every
// flush, and on Mesa/nouveau the orphaned storage piled up between GPU syncs
// -- flush and present time grew steadily through any stretch without a
// texture upload (a replay: +40 % and +80 % in 70 s) and fell back after one.
struct StreamSet {
    GLuint vao{};
    GLuint vbo{};
    GLuint ebo{};
    GLuint transform_buffer{};
    GLuint transform_texture{};
    std::size_t vbo_capacity{};
    std::size_t ebo_capacity{};
    std::size_t transform_capacity{};
};
constexpr std::size_t kStreamSets = 4u;

// ---- recording side (no GL) ---------------------------------------------------

struct RecordedTexture {
    std::uint32_t id{};  // Player handle, assigned at the first upload
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint64_t signature{};
    std::uint64_t signature_epoch{~0ull};
    std::uint64_t last_used_epoch{};
    std::uint64_t pending_flush{~0ull};  // segment serial whose batches sample it
    std::size_t bytes{};
};

struct RecordedTarget {
    std::uint32_t address{};
    std::uint32_t format{1u};
    std::uint32_t stride{512u};
    // The CPU wrote into VRAM: upload it before the next GPU draw.
    bool needs_upload{true};
};

// Batched geometry of one flush. Segments are recycled between the two sides
// so their vectors keep their capacity.
struct DrawSegment {
    std::vector<GlVertex> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<float> transforms;  // RGBA32F texels, one record per draw
    std::vector<Batch> batches;
};

struct CreateTarget { std::uint32_t key{}; };
struct UploadTarget { std::uint32_t key{}; std::vector<std::byte> rgba; };
struct InvalidateTarget { std::uint32_t key{}; };
struct UploadTexture {
    std::uint32_t id{};
    std::uint32_t width{};
    std::uint32_t height{};
    std::uint32_t levels{};
    GLenum min_filter{};
    GLenum mag_filter{};
    GLenum wrap_s{};
    GLenum wrap_t{};
    std::uint32_t max_level{};
    std::vector<std::byte> rgba;  // levels back to back
};
struct DeleteTexture { std::uint32_t id{}; };
using Command = std::variant<DrawSegment, CreateTarget, UploadTarget, InvalidateTarget, UploadTexture, DeleteTexture>;

struct Recorder {
    std::unordered_map<std::uint32_t, RecordedTarget> targets;
    std::unordered_map<std::uint64_t, RecordedTexture> textures;
    // One-entry lookup caches: consecutive draws nearly always share the
    // texture and the target, and each draw looked both up several times.
    // Map nodes are stable; eviction resets the texture entry.
    std::uint64_t last_texture_key{};
    RecordedTexture *last_texture{};
    std::uint32_t last_target_key{~0u};
    RecordedTarget *last_target{};
    std::uint32_t next_texture_id{1u};
    std::size_t texture_bytes{};
    std::size_t texture_budget{256u << 20u};
    std::uint64_t frame_epoch{1u};
    std::uint64_t flush_serial{1u};
    DrawSegment segment;
    std::vector<Command> commands;
};

// ---- replay side (GL thread) -----------------------------------------------------

struct PlayerTarget {
    GLuint color{};
    GLuint depth{};
    GLuint fbo{};
    // The GPU image is authoritative (drawn into since the last CPU write).
    bool gpu_valid{};
};

struct Player {
    GLuint program{};
    Uniforms uniforms{};
    std::array<StreamSet, kStreamSets> streams{};
    std::size_t next_stream{};
    StreamSet *stream{&streams[0]};  // set the pending batches draw from
    // Bound for untextured draws: texture 0 is incomplete, and some drivers
    // (macOS) warn about any incomplete texture on a sampled unit.
    GLuint white_texture{};
    std::unordered_map<std::uint32_t, PlayerTarget> targets;
    std::unordered_map<std::uint32_t, GLuint> textures;  // Recorder id -> GL name
    Scratch upload;
    Scratch readback;
    Scratch present;
    // Last state applied to GL, to skip redundant calls.
    DrawState applied{};
    bool applied_valid{};
    std::uint32_t bound_target{~0u};
};

struct GlBackend {
    bool active{};
    std::uint32_t scale{1u};
    const psprecomp::GuestMemory *memory{};
    // Recording side and Player each write their own fields only.
    GeGpuBackendReport report{};
    std::atomic<std::uint32_t> display_framebuffer{};
    std::uint32_t max_transform_texels{};
    Recorder rec;
    Player play;
    // Sealed commands not replayed yet, and segments to recycle.
    std::mutex handoff_mutex;
    std::vector<Command> ready;
    std::vector<DrawSegment> spare_segments;
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

RecordedTexture *find_texture(GlBackend &b, std::uint64_t key) {
    Recorder &r = b.rec;
    if (r.last_texture != nullptr && r.last_texture_key == key) return r.last_texture;
    const auto found = r.textures.find(key);
    if (found == r.textures.end()) return nullptr;
    r.last_texture_key = key;
    r.last_texture = &found->second;
    return r.last_texture;
}

// ---- shaders ----------------------------------------------------------------

constexpr const char *kVertexShader = R"(#version 330 core
layout(location = 0) in vec4 a_position;   // screen: PSP pixels x, y; Z 0..65535; clip W
                                           // hardware: model/world x, y, z
layout(location = 1) in vec4 a_color;
layout(location = 2) in vec4 a_uvqf;       // screen: u, v (texels), q, fog factor
                                           // hardware: raw u, v, q
layout(location = 3) in uint a_draw;
layout(location = 4) in vec3 a_normal;
uniform vec2 u_target_size;
uniform vec2 u_texture_size;
uniform samplerBuffer u_transforms;
out vec4 v_color;
out vec3 v_uvq;
out float v_fog;
out float gl_ClipDistance[6];
void main() {
    v_color = a_color;
    if (a_draw == 0u) {
        float w = a_position.w > 0.0 ? a_position.w : 1.0;
        vec2 ndc = vec2(a_position.x / u_target_size.x * 2.0 - 1.0,
                        1.0 - a_position.y / u_target_size.y * 2.0);
        float z = a_position.z / 65535.0 * 2.0 - 1.0;
        gl_Position = vec4(ndc * w, z * w, w);
        v_uvq = a_uvqf.xyz;
        v_fog = a_uvqf.w;
        for (int i = 0; i < 6; ++i) gl_ClipDistance[i] = 1.0;
        return;
    }
    int base = int(a_draw - 1u);
    mat4 model_to_clip = mat4(texelFetch(u_transforms, base), texelFetch(u_transforms, base + 1),
                              texelFetch(u_transforms, base + 2), texelFetch(u_transforms, base + 3));
    vec4 scale = texelFetch(u_transforms, base + 4);   // viewport scale xyz, depth clip
    vec4 centre = texelFetch(u_transforms, base + 5);  // viewport centre - offset xyz
    vec4 view_z = texelFetch(u_transforms, base + 6);
    vec4 fog = texelFetch(u_transforms, base + 7);     // end, slope
    vec4 uv = texelFetch(u_transforms, base + 8);      // scale u, v, offset u, v
    vec4 position = vec4(a_position.xyz, 1.0);
    vec4 clip = model_to_clip * position;
    float w = clip.w;
    // Screen position times W: x/w * scale + centre, kept homogeneous so GL
    // interpolates perspective-correctly, as it does for CPU vertices.
    vec3 screen_w = clip.xyz * scale.xyz + centre.xyz * w;
    gl_Position = vec4(screen_w.x / u_target_size.x * 2.0 - w,
                       w - screen_w.y / u_target_size.y * 2.0,
                       screen_w.z / 65535.0 * 2.0 - w, w);
    // The CPU clipper's planes: the projection frustum in X/Y, and Z only
    // with depth clipping on (DEPTH_CLAMP covers the rest).
    gl_ClipDistance[0] = clip.x + w;
    gl_ClipDistance[1] = w - clip.x;
    gl_ClipDistance[2] = clip.y + w;
    gl_ClipDistance[3] = w - clip.y;
    gl_ClipDistance[4] = scale.w != 0.0 ? clip.z + w : 1.0;
    gl_ClipDistance[5] = scale.w != 0.0 ? w - clip.z : 1.0;
    float fog_factor = (dot(view_z, position) + fog.x) * fog.y;
    v_fog = (isnan(fog_factor) || isinf(fog_factor)) ? 1.0 : clamp(fog_factor, 0.0, 1.0);
    v_uvq = vec3((a_uvqf.xy * uv.xy + uv.zw) * u_texture_size, a_uvqf.z);
    if (fog.z != 0.0) {
        // ge_renderer's apply_prepared_lighting() for directional, diffuse-only
        // lights: world normal, normalized (0,0,1 when degenerate), then
        // base + sum(max(dot(L, n), 0) * diffuse), rounded to 8 bits.
        vec4 world0 = texelFetch(u_transforms, base + 9);
        vec4 world1 = texelFetch(u_transforms, base + 10);
        vec4 world2 = texelFetch(u_transforms, base + 11);
        vec4 light_base = texelFetch(u_transforms, base + 12);
        vec3 n = mat3(world0.xyz, world1.xyz, world2.xyz) * a_normal;
        if (world0.w != 0.0) n = -n;
        float length2 = dot(n, n);
        n = (isinf(length2) || isnan(length2) || length2 <= 1.0e-30) ? vec3(0.0, 0.0, 1.0)
                                                                      : n * (1.0 / sqrt(length2));
        vec3 c = light_base.rgb;
        for (int i = 0; i < 4; ++i) {
            vec4 direction = texelFetch(u_transforms, base + 13 + i * 2);
            vec4 diffuse = texelFetch(u_transforms, base + 14 + i * 2);
            float d = dot(direction.xyz, n);
            if (direction.w != 0.0 && d > 0.0) c += diffuse.rgb * d;
        }
        v_color = clamp(floor(vec4(c, light_base.a) * 255.0 + 0.5), 0.0, 255.0) / 255.0;
    }
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


bool build_program(Player &p, std::string &error) {
    const GLuint vs = compile_shader(VERTEX_SHADER, kVertexShader, error);
    if (vs == 0u) return false;
    const GLuint fs = compile_shader(FRAGMENT_SHADER, kFragmentShader, error);
    if (fs == 0u) { api.DeleteShader(vs); return false; }
    p.program = api.CreateProgram();
    api.AttachShader(p.program, vs);
    api.AttachShader(p.program, fs);
    api.LinkProgram(p.program);
    api.DeleteShader(vs);
    api.DeleteShader(fs);
    GLint ok = 0;
    api.GetProgramiv(p.program, LINK_STATUS, &ok);
    if (ok == 0) {
        GLint length = 0;
        api.GetProgramiv(p.program, INFO_LOG_LENGTH, &length);
        std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
        api.GetProgramInfoLog(p.program, length, nullptr, log.data());
        error = "program link: " + log;
        return false;
    }
    Uniforms &u = p.uniforms;
    u.target_size = api.GetUniformLocation(p.program, "u_target_size");
    u.texture_sampler = api.GetUniformLocation(p.program, "u_texture");
    u.texture_enabled = api.GetUniformLocation(p.program, "u_texture_enabled");
    u.texture_control = api.GetUniformLocation(p.program, "u_texture_control");
    u.texture_env = api.GetUniformLocation(p.program, "u_texture_env");
    u.texture_size = api.GetUniformLocation(p.program, "u_texture_size");
    u.alpha_control = api.GetUniformLocation(p.program, "u_alpha_control");
    u.fog_enabled = api.GetUniformLocation(p.program, "u_fog_enabled");
    u.fog_color = api.GetUniformLocation(p.program, "u_fog_color");
    u.premultiply = api.GetUniformLocation(p.program, "u_premultiply");
    u.premultiply_color = api.GetUniformLocation(p.program, "u_premultiply_color");
    u.transforms = api.GetUniformLocation(p.program, "u_transforms");
    return true;
}

// ---- recording -----------------------------------------------------------------

DrawSegment take_segment(GlBackend &b) {
    std::lock_guard lock(b.handoff_mutex);
    if (b.spare_segments.empty()) return {};
    DrawSegment segment = std::move(b.spare_segments.back());
    b.spare_segments.pop_back();
    return segment;
}

// Ends the current geometry segment: commands queued after it (a texture
// replaced, a CPU upload) run after its draws.
void close_segment(GlBackend &b) {
    Recorder &r = b.rec;
    ++r.flush_serial;
    if (r.segment.batches.empty()) return;
    r.commands.emplace_back(std::move(r.segment));
    r.segment = take_segment(b);
}

RecordedTarget *find_target(GlBackend &b, std::uint32_t address) {
    const auto found = b.rec.targets.find(target_key(address));
    return found == b.rec.targets.end() ? nullptr : &found->second;
}

RecordedTarget &ensure_target(GlBackend &b, std::uint32_t address, std::uint32_t stride, std::uint32_t format) {
    Recorder &r = b.rec;
    const std::uint32_t key = target_key(address);
    if (r.last_target != nullptr && r.last_target_key == key) {
        if (stride != 0u) r.last_target->stride = stride;
        r.last_target->format = format;
        return *r.last_target;
    }
    auto [it, inserted] = r.targets.try_emplace(key);
    RecordedTarget &target = it->second;
    r.last_target_key = key;
    r.last_target = &target;
    if (stride != 0u) target.stride = stride;
    target.format = format;
    if (inserted) {
        target.address = key;
        r.commands.emplace_back(CreateTarget{key});
    }
    return target;
}

// Draws into a target the CPU wrote last continue from the VRAM image, in
// order with any batch already queued for it.
void prepare_target(GlBackend &b, const GeGpuDrawDescriptor &draw) {
    RecordedTarget &target = ensure_target(b, draw.framebuffer_address, draw.framebuffer_stride,
                                           draw.framebuffer_format);
    if (!target.needs_upload) return;
    target.needs_upload = false;
    close_segment(b);
    if (b.memory == nullptr) return;
    try {
        const FramebufferDescription description{0x04000000u | target.address, kTargetWidth, kTargetHeight,
                                                 target.stride, target.format};
        b.rec.commands.emplace_back(UploadTarget{target.address, decode_framebuffer_rgba(*b.memory, description)});
    } catch (const std::exception &error) {
        runtime_log_error("gl target upload", error.what());
    }
}

void append_batch(GlBackend &b, const DrawState &state, std::uint32_t first_index, std::uint32_t count) {
    std::vector<Batch> &batches = b.rec.segment.batches;
    if (!batches.empty() && same_state(batches.back().state, state) &&
        batches.back().first + batches.back().count == first_index) {
        batches.back().count += count;
    } else {
        batches.push_back({state, first_index, count});
    }
}

void evict_textures(GlBackend &b) {
    Recorder &r = b.rec;
    while (r.texture_bytes > r.texture_budget) {
        auto oldest = r.textures.end();
        for (auto it = r.textures.begin(); it != r.textures.end(); ++it) {
            if (it->second.last_used_epoch == r.frame_epoch) continue;
            if (oldest == r.textures.end() || it->second.last_used_epoch < oldest->second.last_used_epoch)
                oldest = it;
        }
        if (oldest == r.textures.end()) return;
        if (oldest->second.id != 0u) r.commands.emplace_back(DeleteTexture{oldest->second.id});
        r.texture_bytes -= oldest->second.bytes;
        if (r.last_texture == &oldest->second) r.last_texture = nullptr;
        r.textures.erase(oldest);
        ++b.report.evicted_textures;
    }
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
        if (RecordedTexture *texture = find_texture(b, texture_key(draw)); texture != nullptr && texture->id != 0u) {
            state.texture = texture->id;
            state.texture_width = static_cast<float>(std::max(1u, draw.texture_width));
            state.texture_height = static_cast<float>(std::max(1u, draw.texture_height));
            texture->pending_flush = b.rec.flush_serial;
            texture->last_used_epoch = b.rec.frame_epoch;
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

// ---- replay ---------------------------------------------------------------------

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

void invalidate_bindings(Player &p) {
    p.bound_target = ~0u;
    p.applied_valid = false;
}

// Copies a top-row-first RGBA8 image into `fbo` (which is `width` x `height`
// device pixels), flipping it into GL orientation and scaling.
void upload_rgba_flipped(Player &p, std::span<const std::byte> rgba, std::uint32_t width,
                         std::uint32_t height, GLuint fbo, std::uint32_t fbo_width,
                         std::uint32_t fbo_height, bool linear) {
    ensure_scratch(p.upload, width, height);
    api.BindTexture(TEXTURE_2D, p.upload.texture);
    api.PixelStorei(UNPACK_ALIGNMENT, 1);
    api.TexSubImage2D(TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height),
                      RGBA, UNSIGNED_BYTE, rgba.data());
    api.Disable(SCISSOR_TEST);
    api.BindFramebuffer(READ_FRAMEBUFFER, p.upload.fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, fbo);
    // Source rows are top-first: read them bottom-up to flip.
    api.BlitFramebuffer(0, static_cast<GLint>(height), static_cast<GLint>(width), 0,
                        0, 0, static_cast<GLint>(fbo_width), static_cast<GLint>(fbo_height),
                        COLOR_BUFFER_BIT, linear ? LINEAR : NEAREST);
    invalidate_bindings(p);
}

PlayerTarget *find_player_target(Player &p, std::uint32_t address) {
    const auto found = p.targets.find(target_key(address));
    return found == p.targets.end() ? nullptr : &found->second;
}

void create_target(GlBackend &b, std::uint32_t key) {
    Player &p = b.play;
    auto [it, inserted] = p.targets.try_emplace(key);
    if (!inserted) return;
    PlayerTarget &target = it->second;
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
    invalidate_bindings(p);
    ++b.report.framebuffer_targets_observed;
    std::cerr << "[gl] render target 0x" << std::hex << key << std::dec << " " << width << "x" << height
              << " (scale " << b.scale << ")\n";
}

void set_color(GLint location, std::uint32_t rgb) {
    api.Uniform3f(location, static_cast<float>(rgb & 0xFFu) / 255.0f,
                  static_cast<float>((rgb >> 8u) & 0xFFu) / 255.0f,
                  static_cast<float>((rgb >> 16u) & 0xFFu) / 255.0f);
}

constexpr std::array<GLenum, 8> kDepthFunctions{0x0200u, 0x0207u, 0x0202u, 0x0205u,
                                                0x0201u, 0x0203u, 0x0204u, 0x0206u};

void apply_state(GlBackend &b, const DrawState &state) {
    Player &p = b.play;
    const DrawState &old = p.applied;
    const bool all = !p.applied_valid;
    if (all || p.bound_target != state.target) {
        PlayerTarget *target = find_player_target(p, state.target);
        api.BindFramebuffer(FRAMEBUFFER, target != nullptr ? target->fbo : 0u);
        api.Viewport(0, 0, static_cast<GLsizei>(kTargetWidth * b.scale),
                     static_cast<GLsizei>(kTargetHeight * b.scale));
        p.bound_target = state.target;
    }
    if (all) {
        api.UseProgram(p.program);
        api.BindVertexArray(p.stream->vao);
        api.Uniform2f(p.uniforms.target_size, static_cast<float>(kTargetWidth),
                      static_cast<float>(kTargetHeight));
        api.Uniform1i(p.uniforms.texture_sampler, 0);
        api.ActiveTexture(TEXTURE0);
        api.Uniform1i(p.uniforms.transforms, 1);
        api.ActiveTexture(TEXTURE1);
        api.BindTexture(TEXTURE_BUFFER, p.stream->transform_texture);
        api.ActiveTexture(TEXTURE0);
        api.Enable(SCISSOR_TEST);
        api.Enable(DEPTH_CLAMP);
        api.Disable(DITHER);
        api.CullFace(BACK);
        for (GLenum plane = 0u; plane < 6u; ++plane) api.Enable(CLIP_DISTANCE0 + plane);
    }
    if (all || old.cull != state.cull) {
        if (state.cull == 0u) {
            api.Disable(CULL_FACE);
        } else {
            api.Enable(CULL_FACE);
            api.FrontFace(state.cull == 1u ? CCW : CW);
        }
    }
    if (all || old.texture != state.texture) {
        GLuint name = p.white_texture;
        if (state.texture != 0u) {
            if (const auto found = p.textures.find(state.texture); found != p.textures.end()) name = found->second;
        }
        api.BindTexture(TEXTURE_2D, name);
        api.Uniform1i(p.uniforms.texture_enabled, state.texture != 0u ? 1 : 0);
    }
    if (all || old.texture_function != state.texture_function || old.texture_use_alpha != state.texture_use_alpha ||
        old.texture_double != state.texture_double)
        api.Uniform4i(p.uniforms.texture_control, static_cast<GLint>(state.texture_function),
                      static_cast<GLint>(state.texture_use_alpha), static_cast<GLint>(state.texture_double), 0);
    if (all || old.texture_env != state.texture_env) set_color(p.uniforms.texture_env, state.texture_env);
    if (all || old.texture_width != state.texture_width || old.texture_height != state.texture_height)
        api.Uniform2f(p.uniforms.texture_size, state.texture_width, state.texture_height);
    if (all || old.alpha_test != state.alpha_test || old.alpha_function != state.alpha_function ||
        old.alpha_reference != state.alpha_reference || old.alpha_mask != state.alpha_mask)
        api.Uniform4i(p.uniforms.alpha_control, static_cast<GLint>(state.alpha_test),
                      static_cast<GLint>(state.alpha_function), static_cast<GLint>(state.alpha_reference),
                      static_cast<GLint>(state.alpha_mask));
    if (all || old.fog != state.fog) api.Uniform1i(p.uniforms.fog_enabled, static_cast<GLint>(state.fog));
    if (all || old.fog_color != state.fog_color) set_color(p.uniforms.fog_color, state.fog_color);
    if (all || old.source_premultiply != state.source_premultiply)
        api.Uniform1i(p.uniforms.premultiply, static_cast<GLint>(state.source_premultiply));
    if (all || old.premultiply_color != state.premultiply_color)
        set_color(p.uniforms.premultiply_color, state.premultiply_color);
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
    p.applied = state;
    p.applied_valid = true;
}


// Writes `bytes` at the start of `buffer`, growing its storage (by half again)
// only when it is too small.
void upload_stream(GLenum target, GLuint buffer, std::size_t &capacity, const void *data, std::size_t bytes) {
    api.BindBuffer(target, buffer);
    if (bytes > capacity) {
        capacity = std::max<std::size_t>(bytes + bytes / 2u, 64u * 1024u);
        api.BufferData(target, static_cast<GLsizeiptr>(capacity), nullptr, DYNAMIC_DRAW);
    }
    api.BufferSubData(target, 0, static_cast<GLsizeiptr>(bytes), data);
}

void play_segment(GlBackend &b, DrawSegment &segment) {
    Player &p = b.play;
    StreamSet &set = p.streams[p.next_stream];
    p.next_stream = (p.next_stream + 1u) % kStreamSets;
    p.stream = &set;
    api.BindVertexArray(set.vao);
    upload_stream(ARRAY_BUFFER, set.vbo, set.vbo_capacity, segment.vertices.data(),
                  segment.vertices.size() * sizeof(GlVertex));
    upload_stream(ELEMENT_ARRAY_BUFFER, set.ebo, set.ebo_capacity, segment.indices.data(),
                  segment.indices.size() * sizeof(std::uint32_t));
    if (!segment.transforms.empty())
        upload_stream(TEXTURE_BUFFER, set.transform_buffer, set.transform_capacity, segment.transforms.data(),
                      segment.transforms.size() * sizeof(float));
    invalidate_bindings(p);
    for (const Batch &batch : segment.batches) {
        apply_state(b, batch.state);
        if (PlayerTarget *target = find_player_target(p, batch.state.target)) target->gpu_valid = true;
        api.DrawElements(TRIANGLES, static_cast<GLsizei>(batch.count), UNSIGNED_INT,
                         reinterpret_cast<const void *>(static_cast<std::uintptr_t>(batch.first) *
                                                        sizeof(std::uint32_t)));
    }
    b.report.game_draw_calls += segment.batches.size();
}

void play_texture_upload(GlBackend &b, const UploadTexture &upload) {
    Player &p = b.play;
    GLuint &name = p.textures[upload.id];
    if (name == 0u) api.GenTextures(1, &name);
    api.BindTexture(TEXTURE_2D, name);
    api.PixelStorei(UNPACK_ALIGNMENT, 1);
    std::size_t offset = 0u;
    std::uint32_t level_width = upload.width;
    std::uint32_t level_height = upload.height;
    for (std::uint32_t level = 0u; level < upload.levels; ++level) {
        api.TexImage2D(TEXTURE_2D, static_cast<GLint>(level), static_cast<GLint>(RGBA8),
                       static_cast<GLsizei>(level_width), static_cast<GLsizei>(level_height), 0, RGBA,
                       UNSIGNED_BYTE, upload.rgba.data() + offset);
        offset += static_cast<std::size_t>(level_width) * level_height * 4u;
        level_width = std::max(1u, level_width >> 1u);
        level_height = std::max(1u, level_height >> 1u);
    }
    api.TexParameteri(TEXTURE_2D, TEXTURE_MIN_FILTER, static_cast<GLint>(upload.min_filter));
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAG_FILTER, static_cast<GLint>(upload.mag_filter));
    api.TexParameteri(TEXTURE_2D, TEXTURE_WRAP_S, static_cast<GLint>(upload.wrap_s));
    api.TexParameteri(TEXTURE_2D, TEXTURE_WRAP_T, static_cast<GLint>(upload.wrap_t));
    api.TexParameteri(TEXTURE_2D, TEXTURE_BASE_LEVEL, 0);
    api.TexParameteri(TEXTURE_2D, TEXTURE_MAX_LEVEL, static_cast<GLint>(upload.max_level));
    p.applied_valid = false;  // the texture binding changed
}

// Runs every sealed command on the GL thread, in GE order.
void replay(GlBackend &b) {
    std::vector<Command> commands;
    {
        std::lock_guard lock(b.handoff_mutex);
        commands.swap(b.ready);
    }
    if (commands.empty()) return;
    Player &p = b.play;
    std::vector<DrawSegment> spent;
    for (Command &command : commands) {
        if (auto *segment = std::get_if<DrawSegment>(&command)) {
            play_segment(b, *segment);
            segment->vertices.clear();
            segment->indices.clear();
            segment->transforms.clear();
            segment->batches.clear();
            spent.push_back(std::move(*segment));
        } else if (const auto *create = std::get_if<CreateTarget>(&command)) {
            create_target(b, create->key);
        } else if (const auto *upload = std::get_if<UploadTarget>(&command)) {
            if (PlayerTarget *target = find_player_target(p, upload->key)) {
                upload_rgba_flipped(p, upload->rgba, kTargetWidth, kTargetHeight, target->fbo,
                                    kTargetWidth * b.scale, kTargetHeight * b.scale, false);
                target->gpu_valid = true;
            }
        } else if (const auto *invalidate = std::get_if<InvalidateTarget>(&command)) {
            if (PlayerTarget *target = find_player_target(p, invalidate->key)) target->gpu_valid = false;
        } else if (const auto *texture = std::get_if<UploadTexture>(&command)) {
            play_texture_upload(b, *texture);
        } else if (const auto *drop = std::get_if<DeleteTexture>(&command)) {
            if (const auto found = p.textures.find(drop->id); found != p.textures.end()) {
                api.DeleteTextures(1, &found->second);
                p.textures.erase(found);
            }
            p.applied_valid = false;
        }
    }
    std::lock_guard lock(b.handoff_mutex);
    for (DrawSegment &segment : spent) b.spare_segments.push_back(std::move(segment));
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
    if (!build_program(b.play, error)) return false;
    for (StreamSet &set : b.play.streams) {
        api.GenVertexArrays(1, &set.vao);
        api.GenBuffers(1, &set.vbo);
        api.BindVertexArray(set.vao);
        api.BindBuffer(ARRAY_BUFFER, set.vbo);
        api.EnableVertexAttribArray(0);
        api.VertexAttribPointer(0, 4, FLOAT, BOOL_FALSE, sizeof(GlVertex), reinterpret_cast<const void *>(0));
        api.EnableVertexAttribArray(1);
        api.VertexAttribPointer(1, 4, UNSIGNED_BYTE, BOOL_TRUE, sizeof(GlVertex),
                                reinterpret_cast<const void *>(offsetof(GlVertex, rgba)));
        api.EnableVertexAttribArray(2);
        api.VertexAttribPointer(2, 4, FLOAT, BOOL_FALSE, sizeof(GlVertex),
                                reinterpret_cast<const void *>(offsetof(GlVertex, u)));
        api.EnableVertexAttribArray(3);
        api.VertexAttribIPointer(3, 1, UNSIGNED_INT, sizeof(GlVertex),
                                 reinterpret_cast<const void *>(offsetof(GlVertex, draw)));
        api.EnableVertexAttribArray(4);
        api.VertexAttribPointer(4, 3, FLOAT, BOOL_FALSE, sizeof(GlVertex),
                                reinterpret_cast<const void *>(offsetof(GlVertex, nx)));
        api.GenBuffers(1, &set.ebo);
        api.BindBuffer(ELEMENT_ARRAY_BUFFER, set.ebo);  // part of the VAO state
        api.GenBuffers(1, &set.transform_buffer);
        api.BindBuffer(TEXTURE_BUFFER, set.transform_buffer);
        set.transform_capacity = 64u * 1024u;
        api.BufferData(TEXTURE_BUFFER, static_cast<GLsizeiptr>(set.transform_capacity), nullptr, DYNAMIC_DRAW);
        api.GenTextures(1, &set.transform_texture);
        api.BindTexture(TEXTURE_BUFFER, set.transform_texture);
        api.TexBuffer(TEXTURE_BUFFER, RGBA32F, set.transform_buffer);
        api.BindTexture(TEXTURE_BUFFER, 0u);
    }
    GLint max_texels = 0;
    api.GetIntegerv(MAX_TEXTURE_BUFFER_SIZE, &max_texels);
    b.max_transform_texels = static_cast<std::uint32_t>(std::max(max_texels, 65536));
    const std::uint32_t white = 0xFFFFFFFFu;
    api.GenTextures(1, &b.play.white_texture);
    api.BindTexture(TEXTURE_2D, b.play.white_texture);
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
    std::cerr << "[gl] draws=" << b.report.draw_calls << " hw_transform_draws=" << b.report.hw_transform_draw_calls
              << " batches=" << b.report.game_draw_calls
              << " textures=" << b.rec.textures.size() << " (" << (b.rec.texture_bytes >> 20u) << " MiB)"
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

void ge_gpu_backend_seal_recording() noexcept {
    GlBackend &b = backend();
    if (!b.active) return;
    close_segment(b);
    evict_textures(b);
    ++b.rec.frame_epoch;
    std::lock_guard lock(b.handoff_mutex);
    if (b.ready.empty()) {
        b.ready.swap(b.rec.commands);
    } else {
        for (Command &command : b.rec.commands) b.ready.push_back(std::move(command));
        b.rec.commands.clear();
    }
}

// ---- textures -------------------------------------------------------------------

bool ge_gpu_backend_texture_needed(const GeGpuDrawDescriptor &draw) noexcept {
    GlBackend &b = backend();
    if (!b.active || !draw.texture_enabled || draw.texture_format > 10u || draw.texture_width == 0u ||
        draw.texture_height == 0u) return false;
    ++b.report.texture_decode_requests;
    RecordedTexture *found = find_texture(b, texture_key(draw));
    if (found == nullptr) return true;
    found->signature_epoch = b.rec.frame_epoch;
    found->last_used_epoch = b.rec.frame_epoch;
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
    RecordedTexture *found = find_texture(b, texture_key(draw));
    return found == nullptr || found->signature_epoch != b.rec.frame_epoch;
}

bool ge_gpu_backend_is_framebuffer_feedback_texture(const GeGpuDrawDescriptor &) noexcept { return false; }
void ge_gpu_backend_note_through_extent(const GeGpuDrawDescriptor &, float, float) noexcept {}
bool ge_gpu_backend_adopt_shared_texture(const GeGpuDrawDescriptor &) noexcept { return false; }

bool ge_gpu_backend_texture_available(const GeGpuDrawDescriptor &draw) noexcept {
    GlBackend &b = backend();
    if (!b.active || !draw.texture_enabled) return false;
    const RecordedTexture *found = find_texture(b, texture_key(draw));
    return found != nullptr && found->id != 0u;
}

bool ge_gpu_backend_upload_decoded_texture_chain_packed(const GeGpuDrawDescriptor &draw, std::uint32_t width,
                                                        std::uint32_t height, std::uint32_t mip_levels,
                                                        std::vector<std::byte> rgba8) noexcept {
    GlBackend &b = backend();
    if (!b.active || width == 0u || height == 0u || mip_levels == 0u || rgba8.empty()) return false;
    Recorder &r = b.rec;
    const std::uint64_t key = texture_key(draw);
    RecordedTexture &texture = r.textures[key];
    // Pending draws sample the old texels: they go first.
    if (texture.id != 0u && texture.pending_flush == r.flush_serial) close_segment(b);
    if (texture.id == 0u) texture.id = r.next_texture_id++;
    std::size_t bytes = 0u;
    std::uint32_t level_width = width;
    std::uint32_t level_height = height;
    std::uint32_t levels = 0u;
    for (; levels < mip_levels; ++levels) {
        const std::size_t level_bytes = static_cast<std::size_t>(level_width) * level_height * 4u;
        if (bytes + level_bytes > rgba8.size()) break;
        bytes += level_bytes;
        level_width = std::max(1u, level_width >> 1u);
        level_height = std::max(1u, level_height >> 1u);
    }
    const bool mipmapped = levels > 1u && draw.texture_mipmap_enabled;
    UploadTexture upload{};
    upload.id = texture.id;
    upload.width = width;
    upload.height = height;
    upload.levels = levels;
    upload.min_filter = mipmapped
        ? (draw.texture_min_linear ? (draw.texture_mipmap_linear ? LINEAR_MIPMAP_LINEAR : LINEAR_MIPMAP_NEAREST)
                                   : (draw.texture_mipmap_linear ? NEAREST_MIPMAP_LINEAR : NEAREST_MIPMAP_NEAREST))
        : (draw.texture_min_linear ? LINEAR : NEAREST);
    upload.mag_filter = draw.texture_mag_linear ? LINEAR : NEAREST;
    upload.wrap_s = draw.texture_clamp_u ? CLAMP_TO_EDGE : REPEAT;
    upload.wrap_t = draw.texture_clamp_v ? CLAMP_TO_EDGE : REPEAT;
    upload.max_level = mipmapped ? levels - 1u : 0u;
    upload.rgba = std::move(rgba8);
    r.commands.emplace_back(std::move(upload));
    r.texture_bytes -= texture.bytes;
    texture.bytes = bytes;
    r.texture_bytes += texture.bytes;
    texture.width = width;
    texture.height = height;
    texture.signature = draw.texture_content_signature;
    texture.signature_epoch = r.frame_epoch;
    texture.last_used_epoch = r.frame_epoch;
    ++b.report.texture_image_uploads;
    b.report.texture_image_upload_bytes += bytes;
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
    prepare_target(b, draw);
    const DrawState state = make_state(b, draw);
    if (draw.texture_enabled && state.texture == 0u) ++b.report.missing_texture_draw_calls;
    const std::size_t count = triangle_vertices.size() - triangle_vertices.size() % 3u;
    const auto first_vertex = static_cast<std::uint32_t>(b.rec.segment.vertices.size());
    const auto first_index = static_cast<std::uint32_t>(b.rec.segment.indices.size());
    b.rec.segment.vertices.reserve(b.rec.segment.vertices.size() + count);
    b.rec.segment.indices.reserve(b.rec.segment.indices.size() + count);
    for (std::size_t i = 0u; i < count; ++i) {
        const GeGpuVertex &v = triangle_vertices[i];
        b.rec.segment.vertices.push_back({v.x, v.y, v.z, v.w, v.rgba, v.u, v.v, v.q, v.fog_factor, 0u, 0.0f, 0.0f, 1.0f});
        b.rec.segment.indices.push_back(first_vertex + static_cast<std::uint32_t>(i));
    }
    append_batch(b, state, first_index, static_cast<std::uint32_t>(count));
    b.report.game_triangles += count / 3u;
    b.report.game_vertices += count;
}

namespace {

// Common part of a hardware-transform draw: target, state, draw record and the
// model-space vertices. Returns the index of the first vertex appended, or
// ~0u when there is nothing to draw.
std::uint32_t begin_hardware_draw(GlBackend &b, const GeGpuDrawDescriptor &draw,
                                  const GeGpuHardwareTransform &hw,
                                  std::span<const GeGpuVertex> vertices, DrawState &state) {
    prepare_target(b, draw);
    const std::uint32_t record_texels = kTransformTexels + (hw.vertex_lighting ? kLightingTexels : 0u);
    if (b.rec.segment.transforms.size() / 4u + record_texels > b.max_transform_texels) close_segment(b);
    state = make_state(b, draw);
    if (draw.texture_enabled && state.texture == 0u) ++b.report.missing_texture_draw_calls;
    // ge_renderer's edge_function is the signed area in GL window orientation,
    // and it calls a triangle counter-clockwise when that area is negative.
    if (hw.cull_enabled) state.cull = hw.accept_counter_clockwise ? 2u : 1u;

    const std::array<float, kTransformTexels * 4u> record{
        hw.model_to_clip[0], hw.model_to_clip[1], hw.model_to_clip[2], hw.model_to_clip[3],
        hw.model_to_clip[4], hw.model_to_clip[5], hw.model_to_clip[6], hw.model_to_clip[7],
        hw.model_to_clip[8], hw.model_to_clip[9], hw.model_to_clip[10], hw.model_to_clip[11],
        hw.model_to_clip[12], hw.model_to_clip[13], hw.model_to_clip[14], hw.model_to_clip[15],
        hw.viewport_scale_x, hw.viewport_scale_y, hw.viewport_scale_z, hw.depth_clip_enabled ? 1.0f : 0.0f,
        hw.viewport_center_x - hw.viewport_offset_x, hw.viewport_center_y - hw.viewport_offset_y,
        hw.viewport_center_z, 0.0f,
        hw.model_to_view_z[0], hw.model_to_view_z[1], hw.model_to_view_z[2], hw.model_to_view_z[3],
        hw.fog_end, hw.fog_slope, hw.vertex_lighting ? 1.0f : 0.0f, 0.0f,
        hw.uv_scale_u, hw.uv_scale_v, hw.uv_offset_u, hw.uv_offset_v,
    };
    const auto draw_id = static_cast<std::uint32_t>(b.rec.segment.transforms.size() / 4u) + 1u;
    b.rec.segment.transforms.insert(b.rec.segment.transforms.end(), record.begin(), record.end());
    if (hw.vertex_lighting) {
        const auto &m = hw.world;
        const auto &l = hw.light_direction;
        const auto &d = hw.light_diffuse;
        const std::array<float, kLightingTexels * 4u> lighting{
            m[0], m[1], m[2], hw.reverse_normals ? 1.0f : 0.0f,
            m[3], m[4], m[5], 0.0f,
            m[6], m[7], m[8], 0.0f,
            hw.light_base[0], hw.light_base[1], hw.light_base[2], hw.light_base[3],
            l[0][0], l[0][1], l[0][2], l[0][3], d[0][0], d[0][1], d[0][2], 0.0f,
            l[1][0], l[1][1], l[1][2], l[1][3], d[1][0], d[1][1], d[1][2], 0.0f,
            l[2][0], l[2][1], l[2][2], l[2][3], d[2][0], d[2][1], d[2][2], 0.0f,
            l[3][0], l[3][1], l[3][2], l[3][3], d[3][0], d[3][1], d[3][2], 0.0f,
        };
        b.rec.segment.transforms.insert(b.rec.segment.transforms.end(), lighting.begin(), lighting.end());
    }

    // Sized once and written through pointers: push_back's capacity check per
    // element was a visible share of this per-draw path.
    const auto first_vertex = static_cast<std::uint32_t>(b.rec.segment.vertices.size());
    b.rec.segment.vertices.resize(b.rec.segment.vertices.size() + vertices.size());
    GlVertex *out_vertex = b.rec.segment.vertices.data() + first_vertex;
    for (const GeGpuVertex &v : vertices)
        *out_vertex++ = {v.x, v.y, v.z, 1.0f, v.rgba, v.u, v.v, v.q, 1.0f, draw_id, v.nx, v.ny, v.nz};
    ++b.report.hw_transform_draw_calls;
    b.report.hw_transform_vertices += vertices.size();
    return first_vertex;
}

void finish_hardware_draw(GlBackend &b, const DrawState &state, std::uint32_t first_index) {
    const auto count = static_cast<std::uint32_t>(b.rec.segment.indices.size()) - first_index;
    append_batch(b, state, first_index, count);
    b.report.game_triangles += count / 3u;
    b.report.game_vertices += count;
}

} // namespace

void ge_gpu_backend_accumulate_hardware_triangles(const GeGpuDrawDescriptor &draw,
                                                  const GeGpuHardwareTransform &hw,
                                                  std::span<const GeGpuVertex> vertices,
                                                  std::span<const std::uint32_t> triangle_indices) noexcept {
    GlBackend &b = backend();
    if (!b.active || vertices.empty()) return;
    // Empty index span: the vertices already are a triangle list.
    const std::size_t count = triangle_indices.empty()
        ? vertices.size() - vertices.size() % 3u
        : triangle_indices.size() - triangle_indices.size() % 3u;
    if (count == 0u) return;
    DrawState state;
    const std::uint32_t first_vertex = begin_hardware_draw(b, draw, hw, vertices, state);
    const auto first_index = static_cast<std::uint32_t>(b.rec.segment.indices.size());
    b.rec.segment.indices.resize(b.rec.segment.indices.size() + count);
    std::uint32_t *out_index = b.rec.segment.indices.data() + first_index;
    if (triangle_indices.empty()) {
        for (std::size_t i = 0u; i < count; ++i)
            out_index[i] = first_vertex + static_cast<std::uint32_t>(i);
    } else {
        for (std::size_t i = 0u; i < count; ++i)
            out_index[i] = first_vertex + triangle_indices[i];
    }
    finish_hardware_draw(b, state, first_index);
}

void ge_gpu_backend_accumulate_hardware_primitive(const GeGpuDrawDescriptor &draw,
                                                  const GeGpuHardwareTransform &hw,
                                                  std::uint32_t primitive,
                                                  std::span<const GeGpuVertex> vertices,
                                                  std::span<const std::uint32_t> order,
                                                  std::uint32_t order_base) noexcept {
    GlBackend &b = backend();
    if (!b.active || vertices.empty() || primitive < 3u || primitive > 5u) return;
    const std::size_t count = order.empty() ? vertices.size() : order.size();
    const std::size_t triangles = primitive == 3u ? count / 3u : (count > 2u ? count - 2u : 0u);
    if (triangles == 0u) return;
    DrawState state;
    const std::uint32_t first_vertex = begin_hardware_draw(b, draw, hw, vertices, state);
    const auto first_index = static_cast<std::uint32_t>(b.rec.segment.indices.size());
    b.rec.segment.indices.resize(b.rec.segment.indices.size() + triangles * 3u);
    std::uint32_t *out = b.rec.segment.indices.data() + first_index;
    const std::uint32_t base = first_vertex - (order.empty() ? 0u : order_base);
    const auto slot = [&](std::size_t i) noexcept {
        return base + (order.empty() ? static_cast<std::uint32_t>(i) : order[i]);
    };
    // Same triangles, in the same vertex order, as the software path:
    // odd strip triangles swap their first two vertices, fans pivot on 0.
    if (primitive == 3u) {
        for (std::size_t i = 0u; i < triangles * 3u; ++i) out[i] = slot(i);
    } else if (primitive == 4u) {
        for (std::size_t i = 0u; i < triangles; ++i, out += 3) {
            const bool odd = (i & 1u) != 0u;
            out[0] = slot(odd ? i + 1u : i);
            out[1] = slot(odd ? i : i + 1u);
            out[2] = slot(i + 2u);
        }
    } else {
        for (std::size_t i = 0u; i < triangles; ++i, out += 3) {
            out[0] = slot(0u);
            out[1] = slot(i + 1u);
            out[2] = slot(i + 2u);
        }
    }
    finish_hardware_draw(b, state, first_index);
}

bool ge_gpu_backend_accumulate_hardware_packed_0115(const GeGpuDrawDescriptor &, const GeGpuHardwareTransform &,
                                                    std::span<const std::byte>, std::uint32_t,
                                                    std::span<const std::uint32_t>) noexcept {
    return false;
}

// ---- framebuffers ---------------------------------------------------------------

void ge_gpu_backend_set_native_window(void *) noexcept {}

void ge_gpu_backend_set_display_framebuffer(std::uint32_t address) noexcept {
    backend().display_framebuffer.store(target_key(address), std::memory_order_relaxed);
}

bool ge_gpu_backend_finish_color_frame(std::uint64_t vblank) noexcept {
    GlBackend &b = backend();
    if (!b.active) return false;
    replay(b);
    ++b.report.game_frames;
    b.report.game_frame_vblank = vblank;
    return true;
}

// Called by the recording side (the renderer) only.
bool ge_gpu_backend_owns_framebuffer(std::uint32_t address) noexcept {
    GlBackend &b = backend();
    return b.active && find_target(b, address) != nullptr;
}

// The CPU wrote VRAM; the GE is idle (the HLE waits for it first).
void ge_gpu_backend_invalidate_framebuffer(std::uint32_t address, std::uint32_t size) noexcept {
    GlBackend &b = backend();
    if (!b.active || size == 0u || (address & 0x0F000000u) != 0x04000000u) return;  // VRAM only
    const std::uint32_t begin = target_key(address);
    const std::uint32_t end = begin + size;
    for (auto &[key, target] : b.rec.targets) {
        const std::uint32_t bytes = target.stride * kTargetHeight * (target.format == 3u ? 4u : 2u);
        if (begin < key + bytes && key < end) {
            target.needs_upload = true;
            b.rec.commands.emplace_back(InvalidateTarget{key});
        }
    }
}

bool ge_gpu_backend_read_framebuffer_rgba(std::uint32_t address, std::uint32_t width, std::uint32_t height,
                                          std::vector<std::byte> &rgba) noexcept {
    GlBackend &b = backend();
    if (!b.active) return false;
    replay(b);
    Player &p = b.play;
    PlayerTarget *target = find_player_target(p, address);
    if (target == nullptr || !target->gpu_valid || width == 0u || height == 0u) return false;
    ensure_scratch(p.readback, width, height);
    api.Disable(SCISSOR_TEST);
    api.BindFramebuffer(READ_FRAMEBUFFER, target->fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, p.readback.fbo);
    // Flip so that the first row read back is the PSP's top row.
    const auto source_width = static_cast<GLint>(std::min(width, kTargetWidth) * b.scale);
    const auto source_height = static_cast<GLint>(std::min(height, kTargetHeight) * b.scale);
    api.BlitFramebuffer(0, static_cast<GLint>(kTargetHeight * b.scale) - source_height, source_width,
                        static_cast<GLint>(kTargetHeight * b.scale), 0, static_cast<GLint>(height),
                        static_cast<GLint>(width), 0, COLOR_BUFFER_BIT, b.scale == 1u ? NEAREST : LINEAR);
    api.BindFramebuffer(READ_FRAMEBUFFER, p.readback.fbo);
    api.PixelStorei(PACK_ALIGNMENT, 1);
    rgba.resize(static_cast<std::size_t>(width) * height * 4u);
    api.ReadPixels(0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height), RGBA, UNSIGNED_BYTE,
                   rgba.data());
    invalidate_bindings(p);
    b.report.game_frame_readback_bytes += rgba.size();
    return true;
}

bool ge_gpu_backend_present_framebuffer(std::uint32_t address, int drawable_width, int drawable_height) noexcept {
    GlBackend &b = backend();
    if (!b.active) return false;
    replay(b);
    Player &p = b.play;
    PlayerTarget *target = find_player_target(p, address);
    if (target == nullptr || !target->gpu_valid) return false;
    clear_default_framebuffer(drawable_width, drawable_height);
    int x = 0, y = 0, width = 0, height = 0;
    letterbox(drawable_width, drawable_height, x, y, width, height);
    api.BindFramebuffer(READ_FRAMEBUFFER, target->fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, 0u);
    api.BlitFramebuffer(0, 0, static_cast<GLint>(kTargetWidth * b.scale), static_cast<GLint>(kTargetHeight * b.scale),
                        x, y, x + width, y + height, COLOR_BUFFER_BIT, LINEAR);
    invalidate_bindings(p);
    b.report.gpu_frame_presented_to_window = true;
    return true;
}

void ge_gpu_backend_present_rgba(std::span<const std::byte> rgba, std::uint32_t width, std::uint32_t height,
                                 int drawable_width, int drawable_height) noexcept {
    GlBackend &b = backend();
    if (!b.active || width == 0u || height == 0u ||
        rgba.size() < static_cast<std::size_t>(width) * height * 4u) return;
    replay(b);
    Player &p = b.play;
    ensure_scratch(p.present, width, height);
    api.BindTexture(TEXTURE_2D, p.present.texture);
    api.PixelStorei(UNPACK_ALIGNMENT, 1);
    api.TexSubImage2D(TEXTURE_2D, 0, 0, 0, static_cast<GLsizei>(width), static_cast<GLsizei>(height), RGBA,
                      UNSIGNED_BYTE, rgba.data());
    clear_default_framebuffer(drawable_width, drawable_height);
    int x = 0, y = 0, w = 0, h = 0;
    letterbox(drawable_width, drawable_height, x, y, w, h);
    api.BindFramebuffer(READ_FRAMEBUFFER, p.present.fbo);
    api.BindFramebuffer(DRAW_FRAMEBUFFER, 0u);
    // Top-first rows: read bottom-up to flip.
    api.BlitFramebuffer(0, static_cast<GLint>(height), static_cast<GLint>(width), 0, x, y, x + w, y + h,
                        COLOR_BUFFER_BIT, LINEAR);
    invalidate_bindings(p);
}

bool ge_gpu_backend_copy_game_frame_rgba(std::span<std::byte>) noexcept { return false; }
bool ge_gpu_backend_presents_directly() noexcept { return false; }
std::uint32_t ge_gpu_backend_owned_framebuffer() noexcept { return 0u; }
std::uint32_t ge_gpu_backend_display_framebuffer() noexcept {
    return backend().display_framebuffer.load(std::memory_order_relaxed);
}
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
