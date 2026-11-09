// threejs_materials.gltf.inject_materials, ported step by step, with the part of pygltflib it
// relies on: loading a .gltf/.glb into pygltflib's dataclasses (their field order and
// defaults, and dataclasses_json's type coercion), and saving them as pygltflib does
// (delete_empty_keys, json.dumps, the GLB buffer repacking).
//
// The file is kept as JSON in pygltflib's form: every object has all the fields of its
// dataclass, in order, None as null; saving drops the empty ones.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "build123d/json.hpp"
#include "build123d/pbr.hpp"
#include "pbr_internal.hpp"
#include "pil_image.hpp"
#include "pyformat.hpp"

namespace build123d {

namespace {

namespace fs = std::filesystem;
namespace pil = detail::pil;
using detail::pbr::is_data_uri;
using detail::pbr::normalize_srgb_color;
using detail::pbr::open_texture_image;
using detail::pbr::resolve_to_data_uri;
using detail::pbr::srgb_to_linear;

// ---------------------------------------------------------------------------
// pygltflib's dataclasses
// ---------------------------------------------------------------------------

enum class F { INT, FLOAT, STR, BOOL, ANY, LIST_INT, LIST_FLOAT, LIST_STR, LIST_ANY, ATTRIBUTES, OBJECT, LIST_OBJECT };

struct FieldSpec {
    const char* name;
    F type;
    const char* cls;            // the dataclass of OBJECT / LIST_OBJECT fields
    const char* default_json;   // the default value
};

using Schema = std::map<std::string, std::vector<FieldSpec>>;

const Schema& schema() {
    static const Schema s = [] {
        Schema t = {
            {"Asset",
             {{"generator", F::STR, nullptr, "\"pygltflib@v1.16.5\""},
              {"copyright", F::STR, nullptr, "null"},
              {"version", F::STR, nullptr, "\"2.0\""},
              {"minVersion", F::STR, nullptr, "null"}}},
            {"Primitive",
             {{"attributes", F::ATTRIBUTES, nullptr, "null"},
              {"indices", F::INT, nullptr, "null"},
              {"mode", F::INT, nullptr, "4"},
              {"material", F::INT, nullptr, "null"},
              {"targets", F::LIST_ANY, nullptr, "[]"}}},
            {"Mesh",
             {{"primitives", F::LIST_OBJECT, "Primitive", "[]"},
              {"weights", F::LIST_FLOAT, nullptr, "[]"},
              {"name", F::STR, nullptr, "null"}}},
            {"AccessorSparseIndices",
             {{"bufferView", F::INT, nullptr, "null"},
              {"byteOffset", F::INT, nullptr, "0"},
              {"componentType", F::INT, nullptr, "null"}}},
            {"AccessorSparseValues", {{"bufferView", F::INT, nullptr, "null"}, {"byteOffset", F::INT, nullptr, "0"}}},
            {"Sparse",
             {{"count", F::INT, nullptr, "null"},
              {"indices", F::OBJECT, "AccessorSparseIndices", "null"},
              {"values", F::OBJECT, "AccessorSparseValues", "null"}}},
            {"Accessor",
             {{"bufferView", F::INT, nullptr, "null"},
              {"byteOffset", F::INT, nullptr, "0"},
              {"componentType", F::INT, nullptr, "null"},
              {"normalized", F::BOOL, nullptr, "false"},
              {"count", F::INT, nullptr, "null"},
              {"type", F::STR, nullptr, "null"},
              {"sparse", F::OBJECT, "Sparse", "null"},
              {"max", F::LIST_FLOAT, nullptr, "[]"},
              {"min", F::LIST_FLOAT, nullptr, "[]"},
              {"name", F::STR, nullptr, "null"}}},
            {"BufferView",
             {{"buffer", F::INT, nullptr, "null"},
              {"byteOffset", F::INT, nullptr, "0"},
              {"byteLength", F::INT, nullptr, "null"},
              {"byteStride", F::INT, nullptr, "null"},
              {"target", F::INT, nullptr, "null"},
              {"name", F::STR, nullptr, "null"}}},
            {"Buffer", {{"uri", F::STR, nullptr, "null"}, {"byteLength", F::INT, nullptr, "null"}}},
            {"Perspective",
             {{"aspectRatio", F::FLOAT, nullptr, "null"},
              {"yfov", F::FLOAT, nullptr, "null"},
              {"zfar", F::FLOAT, nullptr, "null"},
              {"znear", F::FLOAT, nullptr, "null"}}},
            {"Orthographic",
             {{"xmag", F::FLOAT, nullptr, "null"},
              {"ymag", F::FLOAT, nullptr, "null"},
              {"zfar", F::FLOAT, nullptr, "null"},
              {"znear", F::FLOAT, nullptr, "null"}}},
            {"Camera",
             {{"perspective", F::OBJECT, "Perspective", "null"},
              {"orthographic", F::OBJECT, "Orthographic", "null"},
              {"type", F::STR, nullptr, "null"},
              {"name", F::STR, nullptr, "null"}}},
            {"TextureInfo", {{"index", F::INT, nullptr, "null"}, {"texCoord", F::INT, nullptr, "0"}}},
            {"OcclusionTextureInfo",
             {{"index", F::INT, nullptr, "null"},
              {"texCoord", F::INT, nullptr, "null"},
              {"strength", F::FLOAT, nullptr, "1.0"}}},
            {"NormalMaterialTexture",
             {{"index", F::INT, nullptr, "null"},
              {"texCoord", F::INT, nullptr, "null"},
              {"scale", F::FLOAT, nullptr, "1.0"}}},
            {"PbrMetallicRoughness",
             {{"baseColorFactor", F::LIST_FLOAT, nullptr, "[1.0, 1.0, 1.0, 1.0]"},
              {"metallicFactor", F::FLOAT, nullptr, "1.0"},
              {"roughnessFactor", F::FLOAT, nullptr, "1.0"},
              {"baseColorTexture", F::OBJECT, "TextureInfo", "null"},
              {"metallicRoughnessTexture", F::OBJECT, "TextureInfo", "null"}}},
            {"Material",
             {{"pbrMetallicRoughness", F::OBJECT, "PbrMetallicRoughness", "null"},
              {"normalTexture", F::OBJECT, "NormalMaterialTexture", "null"},
              {"occlusionTexture", F::OBJECT, "OcclusionTextureInfo", "null"},
              {"emissiveFactor", F::LIST_FLOAT, nullptr, "[0.0, 0.0, 0.0]"},
              {"emissiveTexture", F::OBJECT, "TextureInfo", "null"},
              {"alphaMode", F::STR, nullptr, "\"OPAQUE\""},
              {"alphaCutoff", F::FLOAT, nullptr, "null"},
              {"doubleSided", F::BOOL, nullptr, "false"},
              {"name", F::STR, nullptr, "null"}}},
            {"Sampler",
             {{"input", F::INT, nullptr, "null"},
              {"interpolation", F::STR, nullptr, "null"},
              {"output", F::INT, nullptr, "null"},
              {"magFilter", F::INT, nullptr, "null"},
              {"minFilter", F::INT, nullptr, "null"},
              {"wrapS", F::INT, nullptr, "10497"},
              {"wrapT", F::INT, nullptr, "10497"}}},
            {"Node",
             {{"mesh", F::INT, nullptr, "null"},
              {"skin", F::INT, nullptr, "null"},
              {"rotation", F::LIST_FLOAT, nullptr, "null"},
              {"translation", F::LIST_FLOAT, nullptr, "null"},
              {"scale", F::LIST_FLOAT, nullptr, "null"},
              {"children", F::LIST_INT, nullptr, "[]"},
              {"matrix", F::LIST_FLOAT, nullptr, "null"},
              {"camera", F::INT, nullptr, "null"},
              {"name", F::STR, nullptr, "null"}}},
            {"Skin",
             {{"inverseBindMatrices", F::INT, nullptr, "null"},
              {"skeleton", F::INT, nullptr, "null"},
              {"joints", F::LIST_INT, nullptr, "[]"},
              {"name", F::STR, nullptr, "null"}}},
            {"Scene", {{"name", F::STR, nullptr, "null"}, {"nodes", F::LIST_INT, nullptr, "[]"}}},
            {"Texture",
             {{"sampler", F::INT, nullptr, "null"}, {"source", F::INT, nullptr, "null"}, {"name", F::STR, nullptr, "null"}}},
            {"Image",
             {{"uri", F::STR, nullptr, "null"},
              {"mimeType", F::STR, nullptr, "null"},
              {"bufferView", F::INT, nullptr, "null"},
              {"name", F::STR, nullptr, "null"}}},
            {"AnimationChannelTarget", {{"node", F::INT, nullptr, "null"}, {"path", F::STR, nullptr, "null"}}},
            {"AnimationSampler",
             {{"input", F::INT, nullptr, "null"},
              {"interpolation", F::STR, nullptr, "\"LINEAR\""},
              {"output", F::INT, nullptr, "null"}}},
            {"AnimationChannel",
             {{"sampler", F::INT, nullptr, "null"}, {"target", F::OBJECT, "AnimationChannelTarget", "null"}}},
            {"Animation",
             {{"name", F::STR, nullptr, "null"},
              {"channels", F::LIST_OBJECT, "AnimationChannel", "[]"},
              {"samplers", F::LIST_OBJECT, "AnimationSampler", "[]"}}},
            {"GLTF2",
             {{"accessors", F::LIST_OBJECT, "Accessor", "[]"},
              {"animations", F::LIST_OBJECT, "Animation", "[]"},
              {"asset", F::OBJECT, "Asset", "{}"},
              {"bufferViews", F::LIST_OBJECT, "BufferView", "[]"},
              {"buffers", F::LIST_OBJECT, "Buffer", "[]"},
              {"cameras", F::LIST_OBJECT, "Camera", "[]"},
              {"extensionsUsed", F::LIST_STR, nullptr, "[]"},
              {"extensionsRequired", F::LIST_STR, nullptr, "[]"},
              {"images", F::LIST_OBJECT, "Image", "[]"},
              {"materials", F::LIST_OBJECT, "Material", "[]"},
              {"meshes", F::LIST_OBJECT, "Mesh", "[]"},
              {"nodes", F::LIST_OBJECT, "Node", "[]"},
              {"samplers", F::LIST_OBJECT, "Sampler", "[]"},
              {"scene", F::INT, nullptr, "null"},
              {"scenes", F::LIST_OBJECT, "Scene", "[]"},
              {"skins", F::LIST_OBJECT, "Skin", "[]"},
              {"textures", F::LIST_OBJECT, "Texture", "[]"}}},
        };
        // every dataclass is a Property: extensions and extras come first
        for (auto& [name, fields] : t) {
            fields.insert(fields.begin(), {{"extensions", F::ANY, nullptr, "{}"}, {"extras", F::ANY, nullptr, "{}"}});
        }
        return t;
    }();
    return s;
}

std::string python_str(const Json& v) {
    switch (v.type()) {
        case Json::Type::STRING: return v.as_string();
        case Json::Type::BOOL: return v.as_bool() ? "True" : "False";
        case Json::Type::INT: return std::to_string(v.as_int());
        case Json::Type::FLOAT: return pyfmt::repr(v.as_double());
        default: break;
    }
    throw std::invalid_argument("can't convert a JSON container to str");
}

// dataclasses_json's _support_extended_types: `value if isinstance(value, T) else T(value)`
Json to_int(const Json& v) {
    switch (v.type()) {
        case Json::Type::INT:
        case Json::Type::BOOL: return v;  // isinstance(True, int)
        case Json::Type::FLOAT: {
            const double d = v.as_double();
            if (!std::isfinite(d)) throw std::invalid_argument("cannot convert float NaN or infinity to integer");
            return Json(static_cast<std::int64_t>(std::trunc(d)));
        }
        case Json::Type::STRING: {
            std::size_t used = 0;
            const std::string s = detail::pbr::py_strip(v.as_string());
            const long long i = std::stoll(s, &used);
            if (used != s.size()) throw std::invalid_argument("invalid literal for int(): " + v.as_string());
            return Json(i);
        }
        default: break;
    }
    throw std::invalid_argument("int() argument must be a string or a real number");
}

Json to_float(const Json& v) {
    switch (v.type()) {
        case Json::Type::FLOAT: return v;
        case Json::Type::INT: return Json(static_cast<double>(v.as_int()));
        case Json::Type::BOOL: return Json(v.as_bool() ? 1.0 : 0.0);
        case Json::Type::STRING: return Json(std::stod(v.as_string()));
        default: break;
    }
    throw std::invalid_argument("float() argument must be a string or a real number");
}

Json to_bool(const Json& v) {
    switch (v.type()) {
        case Json::Type::BOOL: return v;
        case Json::Type::INT: return Json(v.as_int() != 0);
        case Json::Type::FLOAT: return Json(v.as_double() != 0.0);
        case Json::Type::STRING: return Json(!v.as_string().empty());
        case Json::Type::ARRAY:
        case Json::Type::OBJECT: return Json(v.size() > 0);
        default: break;
    }
    return Json(false);
}

Json decode_object(const std::string& cls, const Json& kvs);

Json list_of(const Json& v, Json (*convert)(const Json&)) {
    if (!v.is_array()) throw std::invalid_argument("expected a list");
    Json out = Json::array();
    for (const Json& x : v.as_array()) out.push_back(x.is_null() ? x : convert(x));
    return out;
}

/// Attributes(**raw): the standard attributes first, then the others in their order.
Json decode_attributes(const Json& v) {
    if (!v.is_object()) throw std::invalid_argument("argument after ** must be a mapping");
    if (v.size() == 0) return v;
    static const char* const standard[] = {"POSITION",   "NORMAL",  "TANGENT",  "TEXCOORD_0",
                                           "TEXCOORD_1", "COLOR_0", "JOINTS_0", "WEIGHTS_0"};
    Json out = Json::object();
    for (const char* key : standard) out[key] = v.contains(key) ? v[key] : Json(nullptr);
    for (const auto& [key, value] : v.as_object()) {
        if (!out.contains(key)) out[key] = value;
    }
    return out;
}

Json decode_value(const FieldSpec& f, const Json& v) {
    if (v.is_null()) return v;
    switch (f.type) {
        case F::INT: return to_int(v);
        case F::FLOAT: return to_float(v);
        case F::STR: return Json(python_str(v));
        case F::BOOL: return to_bool(v);
        case F::ANY:
        case F::LIST_ANY: return v;
        case F::LIST_INT: return list_of(v, to_int);
        case F::LIST_FLOAT: return list_of(v, to_float);
        case F::LIST_STR: return list_of(v, [](const Json& x) { return Json(python_str(x)); });
        case F::ATTRIBUTES: return decode_attributes(v);
        case F::OBJECT: return decode_object(f.cls, v);
        case F::LIST_OBJECT: {
            if (!v.is_array()) throw std::invalid_argument("expected a list");
            Json out = Json::array();
            for (const Json& x : v.as_array()) out.push_back(x.is_null() ? x : decode_object(f.cls, x));
            return out;
        }
    }
    return v;
}

/// _decode_dataclass(cls, kvs, infer_missing=True): the fields in order, missing ones at their
/// defaults, unknown keys dropped.
Json decode_object(const std::string& cls, const Json& kvs) {
    if (!kvs.is_object()) throw std::invalid_argument("expected a JSON object for " + cls);
    Json out = Json::object();
    for (const FieldSpec& f : schema().at(cls)) {
        if (kvs.contains(f.name)) {
            out[f.name] = decode_value(f, kvs[f.name]);
        } else if (f.type == F::ATTRIBUTES) {
            throw std::invalid_argument("argument after ** must be a mapping, not Attributes");
        } else if (f.type == F::OBJECT && std::strcmp(f.default_json, "{}") == 0) {
            out[f.name] = decode_object(f.cls, Json::object());
        } else {
            out[f.name] = Json::parse(f.default_json);
        }
    }
    return out;
}

/// A new dataclass object (a constructor call): the given fields, the others at their defaults.
Json make(const std::string& cls, std::initializer_list<std::pair<std::string, Json>> fields) {
    return decode_object(cls, Json::object(fields));
}

/// pygltflib.delete_empty_keys
void delete_empty_keys(Json& dictionary) {
    Json::Object kept;
    for (auto& [key, value] : dictionary.as_object()) {
        const bool iterable = value.is_string() || value.is_array() || value.is_object();
        if (value.is_null() || (iterable && (value.is_string() ? value.as_string().empty() : value.size() == 0))) {
            continue;
        }
        if (value.is_object() && key != "extensions") {
            delete_empty_keys(value);
        } else if (value.is_array()) {
            for (Json& item : value.as_array()) {
                if (item.is_object()) delete_empty_keys(item);
            }
        }
        kept.emplace_back(key, std::move(value));
    }
    dictionary = Json(std::move(kept));
}

void check_finite(const Json& v) {
    if (v.type() == Json::Type::FLOAT && !std::isfinite(v.as_double())) {
        throw std::invalid_argument("Out of range float values are not JSON compliant");
    }
    if (v.is_array()) {
        for (const Json& x : v.as_array()) check_finite(x);
    } else if (v.is_object()) {
        for (const auto& [k, x] : v.as_object()) check_finite(x);
    }
}

/// GLTF2.gltf_to_json: delete_empty_keys, json.dumps(allow_nan=False)
std::string gltf_to_json(const Json& gltf, bool compact) {
    Json data = gltf;
    delete_empty_keys(data);
    check_finite(data);
    return compact ? data.dump_compact() : data.dump(2);
}

// ---------------------------------------------------------------------------
// pygltflib's files
// ---------------------------------------------------------------------------

struct Gltf {
    Json json;
    std::optional<pil::Bytes> blob;  // binary_blob()
};

std::uint32_t u32(const pil::Bytes& data, std::size_t at) {
    if (at + 4 > data.size()) throw std::runtime_error("truncated glb file");
    return static_cast<std::uint32_t>(data[at]) | (static_cast<std::uint32_t>(data[at + 1]) << 8) |
           (static_cast<std::uint32_t>(data[at + 2]) << 16) | (static_cast<std::uint32_t>(data[at + 3]) << 24);
}

void put_u32(std::string& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) out += static_cast<char>((v >> (8 * i)) & 0xFF);
}

Gltf from_json(const std::string& text) { return {decode_object("GLTF2", Json::parse(text)), std::nullopt}; }

/// GLTF2.load_from_bytes
Gltf load_binary(const pil::Bytes& data) {
    if (data.size() < 12 || std::memcmp(data.data(), "glTF", 4) != 0) {
        throw std::runtime_error("Unable to load binary gltf file. Header does not appear to be valid glb format.");
    }
    const std::uint32_t length = u32(data, 8);
    std::size_t index = 12;
    std::optional<Gltf> obj;
    while (index < length) {
        const std::uint32_t chunk_length = u32(data, index);
        index += 4;
        const std::string type(reinterpret_cast<const char*>(data.data()) + index, 4);
        index += 4;
        if (index + chunk_length > data.size()) throw std::runtime_error("truncated glb file");
        if (type == "JSON") {
            obj = from_json(std::string(reinterpret_cast<const char*>(data.data()) + index, chunk_length));
        } else if (type == std::string("BIN\0", 4)) {
            if (!obj) throw std::runtime_error("glb BIN chunk before its JSON chunk");
            obj->blob = pil::Bytes(data.begin() + static_cast<std::ptrdiff_t>(index),
                                   data.begin() + static_cast<std::ptrdiff_t>(index + chunk_length));
        }
        index += chunk_length;
    }
    if (!obj) throw std::runtime_error("glb file without JSON chunk");
    return *obj;
}

std::string lower_ext(const std::string& path) { return detail::pbr::py_lower(fs::path(path).extension().string()); }

/// GLTF2.load
Gltf load(const std::string& path) {
    const std::string ext = lower_ext(path);
    const pil::Bytes data = pil::read_file(path);
    if (ext == ".bin" || ext == ".glb") return load_binary(data);
    return from_json(std::string(data.begin(), data.end()));
}

void write_file(const fs::path& path, const std::string& data) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("can't write " + path.string());
    f.write(data.data(), static_cast<std::streamsize>(data.size()));
    if (!f) throw std::runtime_error("can't write " + path.string());
}

/// GLTF2.required_alignment
std::size_t required_alignment(const Json& gltf) {
    auto has = [](const Json& list) {
        for (const Json& x : list.as_array()) {
            if (x.is_string() && x.as_string() == "EXT_structural_metadata") return true;
        }
        return false;
    };
    const bool metadata = (gltf["extensionsUsed"].is_array() && has(gltf["extensionsUsed"])) ||
                          (gltf["extensionsRequired"].is_array() && has(gltf["extensionsRequired"])) ||
                          (gltf["extensions"].is_object() && gltf["extensions"].contains("EXT_structural_metadata"));
    return metadata ? 8 : 4;
}

/// GLTF2.save_binary (save_to_bytes, buffers_to_binary_blob)
void save_binary(const Gltf& gltf, const std::string& path) {
    Json json = gltf.json;
    const std::size_t alignment = required_alignment(json);
    std::string blob;
    std::size_t offset = 0;
    for (Json& bv : json["bufferViews"].as_array()) {
        const Json& buffer = json["buffers"].at(static_cast<std::size_t>(bv["buffer"].as_int()));
        if (!buffer["uri"].is_null()) {
            // pygltflib reads data: URIs and files here too; export_gltf's files have neither
            throw std::runtime_error("unsupported buffer uri in a glb file");
        }
        if (!gltf.blob) throw std::runtime_error("object of type 'NoneType' has no len()");
        const std::size_t byte_offset = bv["byteOffset"].is_null() ? 0 : static_cast<std::size_t>(bv["byteOffset"].as_int());
        const std::size_t byte_length = static_cast<std::size_t>(bv["byteLength"].as_int());
        bv["byteOffset"] = Json(static_cast<std::int64_t>(offset));
        bv["byteLength"] = Json(static_cast<std::int64_t>(byte_length));
        bv["buffer"] = Json(0);
        const std::size_t end = std::min(gltf.blob->size(), byte_offset + byte_length);
        if (byte_offset < end) blob.append(reinterpret_cast<const char*>(gltf.blob->data()) + byte_offset, end - byte_offset);
        const std::size_t padding = (alignment - byte_length % alignment) % alignment;
        blob.append(padding, '\0');
        offset += padding + byte_length;
    }
    json["buffers"] = Json::array({make("Buffer", {{"byteLength", Json(static_cast<std::int64_t>(blob.size()))}})});
    std::string json_blob = gltf_to_json(json, true);
    const std::size_t pad = (alignment - (12 + json_blob.size()) % alignment) % alignment;
    json_blob.append(pad, ' ');
    std::string out = "glTF";
    put_u32(out, 2);
    put_u32(out, static_cast<std::uint32_t>(12 + 16 + json_blob.size() + blob.size()));
    put_u32(out, static_cast<std::uint32_t>(json_blob.size()));
    out += "JSON";
    out += json_blob;
    put_u32(out, static_cast<std::uint32_t>(blob.size()));
    out += std::string("BIN\0", 4);
    out += blob;
    write_file(path, out);
}

/// GLTF2.save(fname): the asset becomes pygltflib's, then a .glb or .gltf file.
void save(Gltf gltf, const std::string& path) {
    gltf.json["asset"] = make("Asset", {});
    if (lower_ext(path) == ".glb") {
        save_binary(gltf, path);
        return;
    }
    // save_json: a buffer without uri gets the blob as <stem>.bin
    Json json = gltf.json;
    for (Json& buffer : json["buffers"].as_array()) {
        if (buffer["uri"].is_null() && gltf.blob && !gltf.blob->empty()) {
            buffer["uri"] = Json(fs::path(path).stem().string() + ".bin");
            fs::path bin = path;
            write_file(bin.replace_extension(".bin"), std::string(gltf.blob->begin(), gltf.blob->end()));
        }
    }
    write_file(path, gltf_to_json(json, false));
}

// ---------------------------------------------------------------------------
// gltf.py: images and materials
// ---------------------------------------------------------------------------

std::string data_uri_of_png(const pil::Image& img) { return "data:image/png;base64," + pil::base64_encode(pil::encode_png(img)); }

pil::Image open_ref(const std::string& ref, const std::optional<std::string>& texture_dir) {
    return open_texture_image(ref, texture_dir);
}

/// numpy's view of an image as an array: channel `band` of an (h, w, n) array, or the 2D array
std::vector<std::uint8_t> band_of(const pil::Image& img, int band) {
    const int n = img.bands();
    if (n == 1) return img.pixels;
    if (band >= n) throw std::out_of_range("index " + std::to_string(band) + " is out of bounds for axis 2");
    std::vector<std::uint8_t> out(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height));
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = img.pixels[i * static_cast<std::size_t>(n) + static_cast<std::size_t>(band)];
    return out;
}

/// _pack_metallic_roughness: metalness in B, roughness in G
std::string pack_metallic_roughness(const std::optional<std::string>& metalness_ref,
                                    const std::optional<std::string>& roughness_ref, double metalness_scalar,
                                    double roughness_scalar, const std::optional<std::string>& texture_dir) {
    std::optional<pil::Image> m_img, r_img;
    std::pair<int, int> size{0, 0};
    if (metalness_ref && !metalness_ref->empty()) {
        m_img = open_ref(*metalness_ref, texture_dir);
        size = {m_img->width, m_img->height};
    }
    if (roughness_ref && !roughness_ref->empty()) {
        r_img = open_ref(*roughness_ref, texture_dir);
        size = {r_img->width, r_img->height};
    }
    if (m_img && r_img && (m_img->width != r_img->width || m_img->height != r_img->height)) {
        r_img = pil::resize_lanczos(*r_img, m_img->width, m_img->height);
        size = {m_img->width, m_img->height};
    }
    const std::size_t count = static_cast<std::size_t>(size.first) * static_cast<std::size_t>(size.second);
    const std::vector<std::uint8_t> g = r_img ? band_of(*r_img, 1)
                                              : std::vector<std::uint8_t>(count, static_cast<std::uint8_t>(static_cast<int>(roughness_scalar * 255 + 0.5)));
    const std::vector<std::uint8_t> b = m_img ? band_of(*m_img, 2)
                                              : std::vector<std::uint8_t>(count, static_cast<std::uint8_t>(static_cast<int>(metalness_scalar * 255 + 0.5)));
    pil::Image packed;
    packed.mode = "RGB";
    packed.width = size.first;
    packed.height = size.second;
    packed.pixels.resize(count * 3);
    for (std::size_t i = 0; i < count; ++i) {
        packed.pixels[i * 3 + 1] = g.at(i);
        packed.pixels[i * 3 + 2] = b.at(i);
    }
    return data_uri_of_png(packed);
}

/// _merge_opacity_into_color: the opacity map as the color map's alpha
std::string merge_opacity_into_color(const std::optional<std::string>& color_ref, const std::string& opacity_ref) {
    pil::Image opacity = pil::convert(open_ref(opacity_ref, std::nullopt), "L");
    pil::Image color;
    if (color_ref && !color_ref->empty()) {
        color = pil::convert(open_ref(*color_ref, std::nullopt), "RGB");
        if (color.width != opacity.width || color.height != opacity.height) {
            opacity = pil::resize_lanczos(opacity, color.width, color.height);
        }
    } else {
        color = pil::new_image("RGB", opacity.width, opacity.height, {255, 255, 255});
    }
    return data_uri_of_png(pil::put_alpha(color, opacity));
}

/// _texture_transform
std::optional<Json> texture_transform(const std::optional<std::array<double, 2>>& repeat, std::optional<double> rotation) {
    Json transform = Json::object();
    if (repeat) transform["scale"] = Json::array({Json((*repeat)[0]), Json((*repeat)[1])});
    if (rotation && *rotation != 0.0) transform["rotation"] = Json(*rotation * (3.141592653589793 / 180.0));
    if (transform.size() == 0) return std::nullopt;
    return transform;
}

/// threejs_materials.gltf._GltfBuilder
class GltfBuilder {
public:
    Json materials = Json::array();
    Json images = Json::array();
    Json textures = Json::array();
    Json samplers = Json::array({make("Sampler", {{"magFilter", Json(9729)},
                                                  {"minFilter", Json(9987)},
                                                  {"wrapS", Json(10497)},
                                                  {"wrapT", Json(10497)}})});
    std::set<std::string> extensions_used;

    void add_material(const PbrProperties& material, const std::optional<std::string>& name) {
        vals_ = material.values.to_dict();
        texs_ = material.maps.to_dict();
        texture_dir_ = material.maps_dir;
        repeat_ = material.texture_repeat;
        // only the tuple (1.0, 1.0) equals (1.0, 1.0)
        if (repeat_ && material.texture_repeat_tuple && (*repeat_)[0] == 1.0 && (*repeat_)[1] == 1.0) repeat_.reset();
        rotation_ = material.texture_rotation;

        Json pbr = build_pbr();
        Json extensions = build_extensions();

        std::string alpha_mode = "OPAQUE";
        Json alpha_cutoff = nullptr;
        const Json alpha_test = val("alphaTest");
        if (!alpha_test.is_null()) {
            alpha_mode = "MASK";
            alpha_cutoff = alpha_test;
        } else if (val("transparent").is_bool() && val("transparent").as_bool()) {
            alpha_mode = "BLEND";
        } else if (texs_.contains("opacity") && !texs_["opacity"].as_string().empty()) {
            alpha_mode = "MASK";
            alpha_cutoff = Json(0.5);
        }
        const Json emissive_val = val("emissive");
        Json emissive_factor = Json::array({Json(0.0), Json(0.0), Json(0.0)});
        if (emissive_val.is_array()) {
            emissive_factor = Json::array();
            for (std::size_t i = 0; i < std::min<std::size_t>(3, emissive_val.size()); ++i) {
                emissive_factor.push_back(emissive_val.at(i));
            }
        }
        // the keyword arguments of Material(...), evaluated in order
        Json normal = build_normal();
        Json occlusion = build_occlusion();
        Json emissive_texture = tex_info("emissive");
        const Json side = val("side");
        const bool double_sided = side.is_number() && side.as_double() == 2.0;
        Json gmat = make("Material", {{"pbrMetallicRoughness", pbr},
                                      {"normalTexture", normal},
                                      {"occlusionTexture", occlusion},
                                      {"emissiveFactor", emissive_factor},
                                      {"emissiveTexture", emissive_texture},
                                      {"alphaMode", Json(alpha_mode)},
                                      {"alphaCutoff", alpha_cutoff},
                                      {"doubleSided", Json(double_sided)},
                                      {"name", Json(name && !name->empty() ? *name : material.name)}});
        if (extensions.size() > 0) {
            for (const auto& [key, value] : extensions.as_object()) extensions_used.insert(key);
            gmat["extensions"] = extensions;
        }
        materials.push_back(gmat);
    }

private:
    Json vals_, texs_;
    std::optional<std::string> texture_dir_;
    std::optional<std::array<double, 2>> repeat_;
    std::optional<double> rotation_;
    std::map<std::string, int> uri_to_index_;

    Json val(const char* name) const { return vals_.contains(name) ? vals_[name] : Json(nullptr); }

    void apply_tex_transform(Json& ti) {
        const auto transform = texture_transform(repeat_, rotation_);
        if (!transform) return;
        if (ti["extensions"].is_null()) ti["extensions"] = Json::object();
        ti["extensions"]["KHR_texture_transform"] = *transform;
        extensions_used.insert("KHR_texture_transform");
    }

    int register_image(const std::string& uri, const std::optional<std::string>& name) {
        const auto found = uri_to_index_.find(uri);
        if (found != uri_to_index_.end()) return found->second;
        const int index = static_cast<int>(images.size());
        uri_to_index_[uri] = index;
        std::string mime = "image/png";
        if (is_data_uri(uri)) {
            const std::string after = uri.substr(uri.find(':') + 1);
            mime = after.substr(0, std::min(after.find(';'), after.find(':')));
        }
        const std::string ext = mime == "image/jpeg" ? ".jpg" : ".png";
        const std::string img_name = (name && !name->empty() ? *name : "texture_" + std::to_string(index)) + ext;
        images.push_back(make("Image", {{"uri", Json(uri)}, {"mimeType", Json(mime)}, {"name", Json(img_name)}}));
        textures.push_back(make("Texture", {{"source", Json(static_cast<int>(images.size()) - 1)}, {"sampler", Json(0)}}));
        return index;
    }

    std::optional<std::string> tex_uri(const char* prop_name) const {
        if (!texs_.contains(prop_name) || texs_[prop_name].is_null()) return std::nullopt;
        const std::string& ref = texs_[prop_name].as_string();
        if (is_data_uri(ref)) return ref;
        if (texture_dir_) return resolve_to_data_uri(ref, *texture_dir_);
        return std::nullopt;
    }

    Json tex_info(const char* prop_name) {
        const auto uri = tex_uri(prop_name);
        if (!uri) return nullptr;
        return make_tex_info(*uri, prop_name);
    }

    Json make_tex_info(const std::string& uri, const char* name) {
        Json ti = make("TextureInfo", {{"index", Json(register_image(uri, std::string(name)))}});
        apply_tex_transform(ti);
        return ti;
    }

    static Json tex_ref(const Json& ti) {
        if (ti.is_null()) return nullptr;
        Json ref = Json::object({{"index", ti["index"]}});
        if (ti["extensions"].is_object() && ti["extensions"].size() > 0) ref["extensions"] = ti["extensions"];
        return ref;
    }

    Json build_pbr() {
        const Json color_val = val("color");
        const Json opacity_val = val("opacity");
        const double alpha = opacity_val.is_number() || opacity_val.is_bool()
                                 ? (opacity_val.is_bool() ? (opacity_val.as_bool() ? 1.0 : 0.0) : opacity_val.as_double())
                                 : 1.0;
        Json base_color_factor = nullptr;
        if (!color_val.is_null()) {
            std::vector<double> c;
            for (const Json& x : color_val.as_array()) c.push_back(x.as_double());
            const auto rgb = normalize_srgb_color(ColorSpec(c)).first;
            base_color_factor = Json::array({Json(srgb_to_linear(rgb[0])), Json(srgb_to_linear(rgb[1])),
                                             Json(srgb_to_linear(rgb[2])), Json(alpha)});
        } else if (alpha < 1.0) {
            base_color_factor = Json::array({Json(1.0), Json(1.0), Json(1.0), Json(alpha)});
        }
        const auto color_tex_uri = tex_uri("color");
        const auto opacity_tex_uri = tex_uri("opacity");
        Json base_color_texture;
        if (color_tex_uri && opacity_tex_uri) {
            base_color_texture = make_tex_info(merge_opacity_into_color(color_tex_uri, *opacity_tex_uri), "color");
        } else if (opacity_tex_uri) {
            base_color_texture = make_tex_info(merge_opacity_into_color(std::nullopt, *opacity_tex_uri), "color");
        } else {
            base_color_texture = tex_info("color");
        }
        Json mr_ti = tex_info("metallicRoughness");
        if (mr_ti.is_null()) {
            const auto m_uri = tex_uri("metalness");
            const auto r_uri = tex_uri("roughness");
            if ((m_uri && !m_uri->empty()) || (r_uri && !r_uri->empty())) {
                const std::string packed =
                    pack_metallic_roughness(m_uri, r_uri, val("metalness").is_null() ? 1.0 : val("metalness").as_double(),
                                            val("roughness").is_null() ? 1.0 : val("roughness").as_double(), texture_dir_);
                mr_ti = make_tex_info(packed, "metallicRoughness");
            }
        }
        return make("PbrMetallicRoughness",
                    {{"baseColorFactor", base_color_factor.is_null()
                                             ? Json::array({Json(1.0), Json(1.0), Json(1.0), Json(1.0)})
                                             : base_color_factor},
                     {"baseColorTexture", base_color_texture},
                     {"metallicFactor", val("metalness").is_null() ? Json(1.0) : val("metalness")},
                     {"roughnessFactor", val("roughness").is_null() ? Json(1.0) : val("roughness")},
                     {"metallicRoughnessTexture", mr_ti}});
    }

    Json build_normal() {
        const auto uri = tex_uri("normal");
        if (!uri) return nullptr;
        Json scale = val("normalScale");
        if (scale.is_array()) scale = scale.at(0);
        Json nmt = make("NormalMaterialTexture", {{"index", Json(register_image(*uri, std::string("normal")))},
                                                  {"scale", scale.is_null() ? Json(1.0) : scale}});
        apply_tex_transform(nmt);
        return nmt;
    }

    Json build_occlusion() {
        const auto uri = tex_uri("ao");
        if (!uri) return nullptr;
        Json oti = make("OcclusionTextureInfo", {{"index", Json(register_image(*uri, std::string("ao")))}});
        apply_tex_transform(oti);
        return oti;
    }

    static bool positive(const Json& v) { return !v.is_null() && v.as_double() > 0; }

    static Json first3(const Json& v) {
        if (!v.is_array()) return v;
        Json out = Json::array();
        for (std::size_t i = 0; i < std::min<std::size_t>(3, v.size()); ++i) out.push_back(v.at(i));
        return out;
    }

    Json build_extensions() {
        Json extensions = Json::object();
        const Json ior = val("ior");
        if (!ior.is_null()) extensions["KHR_materials_ior"] = Json::object({{"ior", ior}});
        const Json transmission = val("transmission");
        if (positive(transmission)) {
            Json ext = Json::object({{"transmissionFactor", transmission}});
            if (const Json ref = tex_ref(tex_info("transmission")); !ref.is_null()) ext["transmissionTexture"] = ref;
            extensions["KHR_materials_transmission"] = ext;
        }
        Json volume = Json::object();
        const Json thickness = val("thickness");
        if (positive(thickness)) {
            volume["thicknessFactor"] = thickness;
            if (const Json ref = tex_ref(tex_info("thickness")); !ref.is_null()) volume["thicknessTexture"] = ref;
        }
        const Json att_color = val("attenuationColor");
        if (!att_color.is_null()) volume["attenuationColor"] = first3(att_color);
        const Json att_dist = val("attenuationDistance");
        if (!att_dist.is_null()) volume["attenuationDistance"] = att_dist;
        if (volume.size() > 0) extensions["KHR_materials_volume"] = volume;

        const Json clearcoat = val("clearcoat");
        if (positive(clearcoat)) {
            Json ext = Json::object({{"clearcoatFactor", clearcoat}});
            if (const Json ref = tex_ref(tex_info("clearcoat")); !ref.is_null()) ext["clearcoatTexture"] = ref;
            const Json cc_rough = val("clearcoatRoughness");
            if (!cc_rough.is_null()) ext["clearcoatRoughnessFactor"] = cc_rough;
            if (const Json ref = tex_ref(tex_info("clearcoatRoughness")); !ref.is_null()) {
                ext["clearcoatRoughnessTexture"] = ref;
            }
            if (const auto cc_uri = tex_uri("clearcoatNormal")) {
                ext["clearcoatNormalTexture"] =
                    Json::object({{"index", Json(register_image(*cc_uri, std::string("clearcoatNormal")))}});
            }
            extensions["KHR_materials_clearcoat"] = ext;
        }
        const Json sheen = val("sheen");
        if (positive(sheen)) {
            Json ext = Json::object();
            const Json sheen_color = val("sheenColor");
            if (!sheen_color.is_null()) ext["sheenColorFactor"] = first3(sheen_color);
            if (const Json ref = tex_ref(tex_info("sheenColor")); !ref.is_null()) ext["sheenColorTexture"] = ref;
            const Json sheen_rough = val("sheenRoughness");
            if (!sheen_rough.is_null()) ext["sheenRoughnessFactor"] = sheen_rough;
            if (const Json ref = tex_ref(tex_info("sheenRoughness")); !ref.is_null()) ext["sheenRoughnessTexture"] = ref;
            extensions["KHR_materials_sheen"] = ext;
        }
        const Json iridescence = val("iridescence");
        if (positive(iridescence)) {
            Json ext = Json::object({{"iridescenceFactor", iridescence}});
            if (const Json ref = tex_ref(tex_info("iridescence")); !ref.is_null()) ext["iridescenceTexture"] = ref;
            const Json iri_ior = val("iridescenceIOR");
            if (!iri_ior.is_null()) ext["iridescenceIor"] = iri_ior;
            const Json iri_range = val("iridescenceThicknessRange");
            if (iri_range.is_array() && iri_range.size() == 2) {
                ext["iridescenceThicknessMinimum"] = iri_range.at(0);
                ext["iridescenceThicknessMaximum"] = iri_range.at(1);
            }
            extensions["KHR_materials_iridescence"] = ext;
        }
        const Json anisotropy = val("anisotropy");
        if (positive(anisotropy)) {
            Json ext = Json::object({{"anisotropyStrength", anisotropy}});
            const Json aniso_rot = val("anisotropyRotation");
            if (!aniso_rot.is_null()) ext["anisotropyRotation"] = aniso_rot;
            if (const Json ref = tex_ref(tex_info("anisotropy")); !ref.is_null()) ext["anisotropyTexture"] = ref;
            extensions["KHR_materials_anisotropy"] = ext;
        }
        const Json spec_intensity = val("specularIntensity");
        const Json spec_color = val("specularColor");
        if (!spec_intensity.is_null() || !spec_color.is_null()) {
            Json ext = Json::object();
            if (!spec_intensity.is_null()) ext["specularFactor"] = spec_intensity;
            if (const Json ref = tex_ref(tex_info("specularIntensity")); !ref.is_null()) ext["specularTexture"] = ref;
            if (!spec_color.is_null()) ext["specularColorFactor"] = first3(spec_color);
            if (const Json ref = tex_ref(tex_info("specularColor")); !ref.is_null()) ext["specularColorTexture"] = ref;
            extensions["KHR_materials_specular"] = ext;
        }
        const Json emissive_intensity = val("emissiveIntensity");
        if (!emissive_intensity.is_null() && emissive_intensity.as_double() != 1.0) {
            extensions["KHR_materials_emissive_strength"] = Json::object({{"emissiveStrength", emissive_intensity}});
        }
        const Json dispersion = val("dispersion");
        if (positive(dispersion)) {
            extensions["KHR_materials_dispersion"] = Json::object({{"dispersion", dispersion}});
            if (!extensions.contains("KHR_materials_volume")) {
                extensions["KHR_materials_volume"] = Json::object({{"thicknessFactor", Json(0)}});
            }
        }
        return extensions;
    }
};

/// collect_gltf_textures / _build_gltf: the materials (by name) of an empty glTF, with their
/// images (embedded in a binary blob for a .glb).
struct Merged {
    Json materials, images, textures, samplers, buffer_views = Json::array();
    std::optional<pil::Bytes> blob;
    std::vector<std::string> extensions_used;
};

Merged collect_gltf_textures(const std::vector<std::pair<std::string, const PbrProperties*>>& materials, bool binary) {
    GltfBuilder builder;
    for (const auto& [name, material] : materials) builder.add_material(*material, name);
    Merged merged{builder.materials, builder.images, builder.textures, builder.samplers, Json::array(), std::nullopt,
                  std::vector<std::string>(builder.extensions_used.begin(), builder.extensions_used.end())};
    if (binary && merged.images.size() > 0) {
        // _embed_data_uri_images
        pil::Bytes blob;
        for (Json& image : merged.images.as_array()) {
            if (!image["uri"].is_string() || !is_data_uri(image["uri"].as_string())) continue;
            const std::string& uri = image["uri"].as_string();
            const pil::Bytes raw = pil::base64_decode(uri.substr(uri.find(',') + 1));
            blob.resize(blob.size() + (4 - blob.size() % 4) % 4, 0);
            const std::size_t byte_offset = blob.size();
            blob.insert(blob.end(), raw.begin(), raw.end());
            image["bufferView"] = Json(static_cast<int>(merged.buffer_views.size()));
            merged.buffer_views.push_back(make("BufferView", {{"buffer", Json(0)},
                                                              {"byteOffset", Json(static_cast<std::int64_t>(byte_offset))},
                                                              {"byteLength", Json(static_cast<std::int64_t>(raw.size()))}}));
            image["uri"] = nullptr;
        }
        merged.blob = blob;
    }
    return merged;
}

// ---------------------------------------------------------------------------
// gltf.py: UV normalization
// ---------------------------------------------------------------------------

struct AccessorLayout {
    int component_type;
    std::size_t n_components, count, offset, stride, component_size;
};

std::size_t component_size(int component_type) {
    switch (component_type) {
        case 5120:
        case 5121: return 1;
        case 5122:
        case 5123: return 2;
        case 5125:
        case 5126: return 4;
    }
    throw std::out_of_range("componentType " + std::to_string(component_type));
}

AccessorLayout layout(const Gltf& gltf, int accessor_idx) {
    const Json& acc = gltf.json["accessors"].at(static_cast<std::size_t>(accessor_idx));
    if (acc["bufferView"].is_null()) throw std::runtime_error("accessor without bufferView");
    const Json& bv = gltf.json["bufferViews"].at(static_cast<std::size_t>(acc["bufferView"].as_int()));
    static const std::map<std::string, std::size_t> counts = {{"SCALAR", 1}, {"VEC2", 2}, {"VEC3", 3}, {"VEC4", 4}, {"MAT4", 16}};
    AccessorLayout l{};
    l.component_type = static_cast<int>(acc["componentType"].as_int());
    l.n_components = counts.at(acc["type"].as_string());
    l.count = static_cast<std::size_t>(acc["count"].as_int());
    l.offset = static_cast<std::size_t>((bv["byteOffset"].is_null() ? 0 : bv["byteOffset"].as_int()) +
                                        (acc["byteOffset"].is_null() ? 0 : acc["byteOffset"].as_int()));
    l.component_size = component_size(l.component_type);
    const std::size_t elem_size = l.component_size * l.n_components;
    const std::int64_t stride = bv["byteStride"].is_null() ? 0 : bv["byteStride"].as_int();
    l.stride = stride != 0 && static_cast<std::size_t>(stride) != elem_size ? static_cast<std::size_t>(stride) : elem_size;
    return l;
}

/// _read_accessor, as float64 (or int64 for the indices)
std::vector<double> read_accessor(const Gltf& gltf, int accessor_idx, std::size_t* n_components = nullptr) {
    const AccessorLayout l = layout(gltf, accessor_idx);
    if (!gltf.blob) throw std::runtime_error("glTF without binary data");
    const pil::Bytes& blob = *gltf.blob;
    std::vector<double> out;
    out.reserve(l.count * l.n_components);
    for (std::size_t i = 0; i < l.count; ++i) {
        for (std::size_t c = 0; c < l.n_components; ++c) {
            const std::size_t at = l.offset + i * l.stride + c * l.component_size;
            if (at + l.component_size > blob.size()) throw std::runtime_error("buffer is smaller than requested size");
            const std::uint8_t* p = blob.data() + at;
            double v = 0;
            switch (l.component_type) {
                case 5120: v = static_cast<std::int8_t>(p[0]); break;
                case 5121: v = p[0]; break;
                case 5122: {
                    std::int16_t x;
                    std::memcpy(&x, p, 2);
                    v = x;
                    break;
                }
                case 5123: {
                    std::uint16_t x;
                    std::memcpy(&x, p, 2);
                    v = x;
                    break;
                }
                case 5125: {
                    std::uint32_t x;
                    std::memcpy(&x, p, 4);
                    v = x;
                    break;
                }
                default: {
                    float x;
                    std::memcpy(&x, p, 4);
                    v = static_cast<double>(x);
                }
            }
            out.push_back(v);
        }
    }
    if (n_components != nullptr) *n_components = l.n_components;
    return out;
}

/// _write_accessor for float32 data (n rows of the accessor's components)
void write_accessor(Gltf& gltf, int accessor_idx, const std::vector<float>& data) {
    const AccessorLayout l = layout(gltf, accessor_idx);
    pil::Bytes& blob = *gltf.blob;
    for (std::size_t i = 0; i < l.count; ++i) {
        for (std::size_t c = 0; c < l.n_components; ++c) {
            const float x = data[i * l.n_components + c];
            std::uint8_t* p = blob.data() + l.offset + i * l.stride + c * l.component_size;
            switch (l.component_type) {
                case 5126: std::memcpy(p, &x, 4); break;
                case 5120:
                case 5121: p[0] = static_cast<std::uint8_t>(static_cast<std::int64_t>(x)); break;
                case 5122:
                case 5123: {
                    const auto v = static_cast<std::uint16_t>(static_cast<std::int64_t>(x));
                    std::memcpy(p, &v, 2);
                    break;
                }
                default: {
                    const auto v = static_cast<std::uint32_t>(static_cast<std::int64_t>(x));
                    std::memcpy(p, &v, 4);
                }
            }
        }
    }
    Json& acc = gltf.json["accessors"].at(static_cast<std::size_t>(accessor_idx));
    Json mins = Json::array(), maxs = Json::array();
    // data.min(axis=0), data.max(axis=0): NaN wins, as in numpy
    for (std::size_t c = 0; c < l.n_components; ++c) {
        float lo = data[c], hi = data[c];
        for (std::size_t i = 1; i < l.count; ++i) {
            const float x = data[i * l.n_components + c];
            if (std::isnan(x) || (!std::isnan(lo) && x < lo)) lo = std::isnan(lo) ? lo : x;
            if (std::isnan(x) || (!std::isnan(hi) && x > hi)) hi = std::isnan(hi) ? hi : x;
        }
        mins.push_back(Json(static_cast<double>(lo)));
        maxs.push_back(Json(static_cast<double>(hi)));
    }
    acc["min"] = mins;
    acc["max"] = maxs;
    if (gltf.json["buffers"].size() > 0) gltf.json["buffers"].at(0)["byteLength"] = Json(static_cast<std::int64_t>(blob.size()));
}

/// _normalize_primitive_uvs: the same texture size on every part, whatever its size.
void normalize_primitive_uvs(Gltf& gltf, std::size_t mesh_idx, std::size_t prim_idx, double bbox_max_dim) {
    const Json& prim = gltf.json["meshes"].at(mesh_idx)["primitives"].at(prim_idx);
    const Json& attributes = prim["attributes"];
    if (!attributes.is_object() || !attributes.contains("TEXCOORD_0") || !attributes.contains("POSITION")) return;
    const Json texcoord = attributes["TEXCOORD_0"], position = attributes["POSITION"];
    if (texcoord.is_null() || position.is_null()) return;
    std::size_t pn = 0, un = 0;
    const std::vector<double> positions = read_accessor(gltf, static_cast<int>(position.as_int()), &pn);
    const std::vector<double> uvs = read_accessor(gltf, static_cast<int>(texcoord.as_int()), &un);
    if (pn != 3 || un != 2) throw std::runtime_error("unexpected POSITION/TEXCOORD_0 accessor types");
    const std::size_t n = positions.size() / 3;
    std::vector<std::int64_t> indices;
    if (!prim["indices"].is_null()) {
        for (const double i : read_accessor(gltf, static_cast<int>(prim["indices"].as_int()))) {
            indices.push_back(static_cast<std::int32_t>(static_cast<std::int64_t>(i)));
        }
    } else {
        for (std::size_t i = 0; i < n; ++i) indices.push_back(static_cast<std::int64_t>(i));
    }
    const std::size_t num_triangles = indices.size() / 3;
    if (num_triangles == 0) return;
    if (indices.size() % 3 != 0) throw std::invalid_argument("cannot reshape array into shape (3)");
    auto p = [&positions](std::int64_t i, int k) { return positions.at(static_cast<std::size_t>(i) * 3 + static_cast<std::size_t>(k)); };
    auto uv = [&uvs](std::int64_t i, int k) { return uvs.at(static_cast<std::size_t>(i) * 2 + static_cast<std::size_t>(k)); };
    // the triangle whose centroid is nearest the mesh's centroid (numpy sums in order here)
    double centroid[3] = {0, 0, 0};
    for (std::size_t i = 0; i < n; ++i) {
        for (int k = 0; k < 3; ++k) centroid[k] += positions[i * 3 + static_cast<std::size_t>(k)];
    }
    for (double& c : centroid) c /= static_cast<double>(n);
    std::size_t mid_tri = 0;
    double best = 0;
    for (std::size_t t = 0; t < num_triangles; ++t) {
        double d2 = 0;
        for (int k = 0; k < 3; ++k) {
            const double c = (p(indices[t * 3], k) + p(indices[t * 3 + 1], k) + p(indices[t * 3 + 2], k)) / 3.0;
            const double diff = c - centroid[k];
            d2 += diff * diff;
        }
        const double dist = std::sqrt(d2);
        if (t == 0 || dist < best || (std::isnan(dist) && !std::isnan(best))) {
            mid_tri = t;
            best = dist;
        }
        if (std::isnan(best)) break;
    }
    const std::int64_t i0 = indices[mid_tri * 3], i1 = indices[mid_tri * 3 + 1], i2 = indices[mid_tri * 3 + 2];
    double dp1[3], dp2[3];
    for (int k = 0; k < 3; ++k) {
        dp1[k] = p(i1, k) - p(i0, k);
        dp2[k] = p(i2, k) - p(i0, k);
    }
    const double duv1[2] = {uv(i1, 0) - uv(i0, 0), uv(i1, 1) - uv(i0, 1)};
    const double duv2[2] = {uv(i2, 0) - uv(i0, 0), uv(i2, 1) - uv(i0, 1)};
    const double det = duv1[0] * duv2[1] - duv2[0] * duv1[1];
    if (std::abs(det) < 1e-20) return;
    double dpdu[3], dpdv[3];
    for (int k = 0; k < 3; ++k) {
        dpdu[k] = (duv2[1] * dp1[k] - duv1[1] * dp2[k]) / det;
        dpdv[k] = (-duv2[0] * dp1[k] + duv1[0] * dp2[k]) / det;
    }
    const double scale_u = std::sqrt(dpdu[0] * dpdu[0] + dpdu[1] * dpdu[1] + dpdu[2] * dpdu[2]);
    const double scale_v = std::sqrt(dpdv[0] * dpdv[0] + dpdv[1] * dpdv[1] + dpdv[2] * dpdv[2]);
    double uv_min[2] = {uvs[0], uvs[1]};  // uvs.min(axis=0): NaN wins
    for (std::size_t i = 1; i < n; ++i) {
        for (int k = 0; k < 2; ++k) {
            const double x = uvs[i * 2 + static_cast<std::size_t>(k)];
            if (std::isnan(uv_min[k])) continue;
            if (std::isnan(x) || x < uv_min[k]) uv_min[k] = x;
        }
    }
    std::vector<float> new_uvs(uvs.size());
    for (std::size_t i = 0; i < n; ++i) {
        new_uvs[i * 2] = static_cast<float>(scale_u > 1e-10 ? (uvs[i * 2] - uv_min[0]) * scale_u / bbox_max_dim : 0.5);
        new_uvs[i * 2 + 1] =
            static_cast<float>(scale_v > 1e-10 ? (uvs[i * 2 + 1] - uv_min[1]) * scale_v / bbox_max_dim : 0.5);
    }
    write_accessor(gltf, static_cast<int>(texcoord.as_int()), new_uvs);
}

/// A copy of a Json value with every texture "index" in a material's extensions moved.
void remap_extensions(Json& extensions, std::int64_t offset) {
    if (!extensions.is_object()) return;
    for (auto& [name, ext] : extensions.as_object()) {
        if (!ext.is_object()) continue;
        for (auto& [key, v] : ext.as_object()) {
            if (v.is_object() && v.contains("index")) v["index"] = Json(v["index"].as_int() + offset);
        }
    }
}

void remap_tex_info(Json& ti, std::int64_t offset) {
    if (ti.is_object() && ti.contains("index") && !ti["index"].is_null()) ti["index"] = Json(ti["index"].as_int() + offset);
}

}  // namespace

// ---------------------------------------------------------------------------
// inject_materials
// ---------------------------------------------------------------------------

void inject_materials(const std::string& target_path,
                      const std::vector<std::pair<int, std::shared_ptr<const PbrProperties>>>& node_materials) {
    if (node_materials.empty()) return;
    const bool is_binary = target_path.size() >= 4 && target_path.compare(target_path.size() - 4, 4, ".glb") == 0;
    Gltf gltf = load(target_path);

    // a .gltf's buffer is the .bin beside it
    std::optional<fs::path> external_bin_path;
    if (!is_binary && !gltf.blob && gltf.json["buffers"].size() > 0) {
        const Json& buf_uri = gltf.json["buffers"].at(0)["uri"];
        if (buf_uri.is_string() && !buf_uri.as_string().empty() && !is_data_uri(buf_uri.as_string())) {
            external_bin_path = fs::path(target_path).parent_path() / buf_uri.as_string();
            if (fs::exists(*external_bin_path)) gltf.blob = pil::read_file(external_bin_path->string());
        }
    }

    std::map<int, std::shared_ptr<const PbrProperties>> resolved;
    for (const auto& [index, pbr] : node_materials) resolved[index] = pbr;

    // one glTF material per look object, in the order of the nodes
    std::map<const PbrProperties*, int> pbr_id_to_idx;
    std::vector<std::shared_ptr<const PbrProperties>> mat_map;
    Json& nodes = gltf.json["nodes"];
    for (std::size_t node_idx = 0; node_idx < nodes.size(); ++node_idx) {
        const auto found = resolved.find(static_cast<int>(node_idx));
        const Json& mesh = nodes.at(node_idx)["mesh"];
        if (found == resolved.end() || mesh.is_null()) continue;
        const PbrProperties* id = found->second.get();
        if (pbr_id_to_idx.count(id) == 0) {
            pbr_id_to_idx[id] = static_cast<int>(mat_map.size());
            mat_map.push_back(found->second);
        }
        for (Json& prim : gltf.json["meshes"].at(static_cast<std::size_t>(mesh.as_int()))["primitives"].as_array()) {
            prim["material"] = Json(pbr_id_to_idx[id]);
        }
    }
    if (mat_map.empty()) return;

    // unique names
    std::vector<std::pair<std::string, const PbrProperties*>> tm_materials;
    std::vector<std::pair<int, std::string>> idx_to_name;
    for (std::size_t mat_idx = 0; mat_idx < mat_map.size(); ++mat_idx) {
        const PbrProperties* material = mat_map[mat_idx].get();
        std::string name = material->name.empty() ? "mat_" + std::to_string(mat_idx) : material->name;
        auto find = [&tm_materials](const std::string& n) {
            return std::find_if(tm_materials.begin(), tm_materials.end(), [&n](const auto& e) { return e.first == n; });
        };
        if (find(name) != tm_materials.end()) name += "_" + std::to_string(mat_idx);
        if (const auto it = find(name); it != tm_materials.end()) {
            it->second = material;
        } else {
            tm_materials.emplace_back(name, material);
        }
        idx_to_name.emplace_back(static_cast<int>(mat_idx), name);
    }

    Merged merged = collect_gltf_textures(tm_materials, is_binary);
    std::map<std::string, std::size_t> merged_name_to_idx;
    for (std::size_t i = 0; i < merged.materials.size(); ++i) {
        const Json& name = merged.materials.at(i)["name"];
        if (name.is_string() && !name.as_string().empty()) merged_name_to_idx[name.as_string()] = i;
    }

    const auto sampler_offset = static_cast<std::int64_t>(gltf.json["samplers"].size());
    const auto image_offset = static_cast<std::int64_t>(gltf.json["images"].size());
    const auto texture_offset = static_cast<std::int64_t>(gltf.json["textures"].size());
    const auto buffer_view_offset = static_cast<std::int64_t>(gltf.json["bufferViews"].size());

    if (merged.blob && !merged.blob->empty()) {
        pil::Bytes existing = gltf.blob ? *gltf.blob : pil::Bytes();
        existing.resize(existing.size() + (4 - existing.size() % 4) % 4, 0);
        const auto blob_offset = static_cast<std::int64_t>(existing.size());
        existing.insert(existing.end(), merged.blob->begin(), merged.blob->end());
        gltf.blob = existing;
        if (gltf.json["buffers"].size() > 0) {
            gltf.json["buffers"].at(0)["byteLength"] = Json(static_cast<std::int64_t>(existing.size()));
        }
        for (const Json& bv : merged.buffer_views.as_array()) {
            Json new_bv = bv;
            new_bv["buffer"] = Json(0);
            if (!new_bv["byteOffset"].is_null()) new_bv["byteOffset"] = Json(new_bv["byteOffset"].as_int() + blob_offset);
            gltf.json["bufferViews"].push_back(new_bv);
        }
    }

    if (!is_binary && merged.images.size() > 0) {
        // the images as files in <name>/ beside the .gltf (convert_images(ImageFormat.FILE))
        const fs::path tex_dir = fs::path(target_path).replace_extension("");
        fs::create_directories(tex_dir);
        for (Json& image : merged.images.as_array()) {
            if (!image["uri"].is_string() || !is_data_uri(image["uri"].as_string())) continue;
            const std::string& uri = image["uri"].as_string();
            const std::string file_name = image["name"].as_string();
            const pil::Bytes data = pil::base64_decode(uri.substr(uri.find(',') + 1));
            write_file(tex_dir / file_name, std::string(data.begin(), data.end()));
            image["uri"] = Json(tex_dir.filename().string() + "/" + file_name);
        }
    }

    for (const Json& sampler : merged.samplers.as_array()) gltf.json["samplers"].push_back(sampler);
    for (const Json& image : merged.images.as_array()) {
        Json img = image;
        if (!img["bufferView"].is_null()) img["bufferView"] = Json(img["bufferView"].as_int() + buffer_view_offset);
        gltf.json["images"].push_back(img);
    }
    for (const Json& texture : merged.textures.as_array()) {
        Json tex = texture;
        if (!tex["source"].is_null()) tex["source"] = Json(tex["source"].as_int() + image_offset);
        if (!tex["sampler"].is_null()) tex["sampler"] = Json(tex["sampler"].as_int() + sampler_offset);
        gltf.json["textures"].push_back(tex);
    }

    Json& materials = gltf.json["materials"];
    const int max_idx = idx_to_name.empty() ? -1 : idx_to_name.back().first;
    while (static_cast<int>(materials.size()) <= max_idx) materials.push_back(make("Material", {}));
    for (const auto& [mat_idx, name] : idx_to_name) {
        const auto merged_idx = merged_name_to_idx.find(name);
        if (merged_idx == merged_name_to_idx.end()) continue;
        Json mat = merged.materials.at(merged_idx->second);
        if (mat["pbrMetallicRoughness"].is_object()) {
            remap_tex_info(mat["pbrMetallicRoughness"]["baseColorTexture"], texture_offset);
            remap_tex_info(mat["pbrMetallicRoughness"]["metallicRoughnessTexture"], texture_offset);
        }
        remap_tex_info(mat["normalTexture"], texture_offset);
        remap_tex_info(mat["occlusionTexture"], texture_offset);
        remap_tex_info(mat["emissiveTexture"], texture_offset);
        remap_extensions(mat["extensions"], texture_offset);
        materials.at(static_cast<std::size_t>(mat_idx)) = mat;
    }

    // the UVs of the materials with normalize_uvs
    std::set<std::int64_t> normalize_mat_indices;
    for (std::size_t i = 0; i < mat_map.size(); ++i) {
        if (mat_map[i]->normalize_uvs) normalize_mat_indices.insert(static_cast<std::int64_t>(i));
    }
    if (!normalize_mat_indices.empty()) {
        const double inf = std::numeric_limits<double>::infinity();
        double pos_min[3] = {inf, inf, inf}, pos_max[3] = {-inf, -inf, -inf};
        for (const Json& mesh : gltf.json["meshes"].as_array()) {
            for (const Json& prim : mesh["primitives"].as_array()) {
                if (!prim["attributes"].is_object() || !prim["attributes"].contains("POSITION") ||
                    prim["attributes"]["POSITION"].is_null()) {
                    continue;
                }
                const Json& acc = gltf.json["accessors"].at(static_cast<std::size_t>(prim["attributes"]["POSITION"].as_int()));
                if (acc["min"].is_array() && acc["min"].size() > 0 && acc["max"].is_array() && acc["max"].size() > 0) {
                    for (int k = 0; k < 3; ++k) {
                        const double lo = acc["min"].at(static_cast<std::size_t>(k)).as_double();
                        const double hi = acc["max"].at(static_cast<std::size_t>(k)).as_double();
                        pos_min[k] = std::isnan(lo) || std::isnan(pos_min[k]) ? std::nan("") : std::min(pos_min[k], lo);
                        pos_max[k] = std::isnan(hi) || std::isnan(pos_max[k]) ? std::nan("") : std::max(pos_max[k], hi);
                    }
                }
            }
        }
        const double dims_sum = (pos_max[0] - pos_min[0]) + (pos_max[1] - pos_min[1]) + (pos_max[2] - pos_min[2]);
        const double mean = dims_sum / 3.0;
        const double bbox_max_dim = std::isnan(mean) ? mean : std::max(mean, 1e-10);
        Json& meshes = gltf.json["meshes"];
        for (std::size_t mesh_idx = 0; mesh_idx < meshes.size(); ++mesh_idx) {
            const std::size_t prims = meshes.at(mesh_idx)["primitives"].size();
            for (std::size_t prim_idx = 0; prim_idx < prims; ++prim_idx) {
                const Json& material = meshes.at(mesh_idx)["primitives"].at(prim_idx)["material"];
                if (material.type() == Json::Type::INT && normalize_mat_indices.count(material.as_int()) > 0) {
                    normalize_primitive_uvs(gltf, mesh_idx, prim_idx, bbox_max_dim);
                }
            }
        }
    }

    std::set<std::string> extensions_used;
    for (const Json& e : gltf.json["extensionsUsed"].as_array()) extensions_used.insert(e.as_string());
    extensions_used.insert(merged.extensions_used.begin(), merged.extensions_used.end());
    Json used = Json::array();
    for (const std::string& e : extensions_used) used.push_back(Json(e));
    gltf.json["extensionsUsed"] = used;

    if (is_binary) {
        save_binary(gltf, target_path);
        return;
    }
    if (external_bin_path) {
        if (gltf.blob && !gltf.blob->empty()) {
            write_file(*external_bin_path, std::string(gltf.blob->begin(), gltf.blob->end()));
            gltf.json["buffers"].at(0)["byteLength"] = Json(static_cast<std::int64_t>(gltf.blob->size()));
        }
        gltf.blob.reset();
        gltf.json["buffers"].at(0)["uri"] = Json(external_bin_path->filename().string());
    }
    save(gltf, target_path);
}

}  // namespace build123d
