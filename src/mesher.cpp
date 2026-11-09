// Mesher (mesher.py): 3MF and STL meshes through lib3mf.
#include "build123d/mesher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <lib3mf_implicit.hpp>

#include "build123d/viewers.hpp"
#include "build123d/warnings.hpp"
#include "internal.hpp"
#include "io_internal.hpp"
#include "pyformat.hpp"

namespace build123d {

using namespace detail;

namespace {

/// Runs a lib3mf call, turning its exceptions into std::runtime_error.
template <class F>
auto lib3mf(const char* context, F&& f) -> decltype(f()) {
    try {
        return f();
    } catch (const Lib3MF::ELib3MFException& e) {
        throw std::runtime_error(std::string(context) + ": " + e.what());
    }
}

Lib3MF::eModelUnit to_3mf_unit(Unit unit) {
    switch (unit) {
        case Unit::MC: return Lib3MF::eModelUnit::MicroMeter;
        case Unit::MM: return Lib3MF::eModelUnit::MilliMeter;
        case Unit::CM: return Lib3MF::eModelUnit::CentiMeter;
        case Unit::IN: return Lib3MF::eModelUnit::Inch;
        case Unit::FT: return Lib3MF::eModelUnit::Foot;
        case Unit::M: return Lib3MF::eModelUnit::Meter;
        default: break;
    }
    throw std::invalid_argument("Unit not supported by 3MF (MC, MM, CM, M, IN and FT are)");
}

Unit to_b3d_unit(Lib3MF::eModelUnit unit) {
    switch (unit) {
        case Lib3MF::eModelUnit::MicroMeter: return Unit::MC;
        case Lib3MF::eModelUnit::MilliMeter: return Unit::MM;
        case Lib3MF::eModelUnit::CentiMeter: return Unit::CM;
        case Lib3MF::eModelUnit::Inch: return Unit::IN;
        case Lib3MF::eModelUnit::Foot: return Unit::FT;
        case Lib3MF::eModelUnit::Meter: return Unit::M;
    }
    throw std::out_of_range("Unknown 3MF model unit");  // Python's KeyError
}

Lib3MF::eObjectType to_3mf_type(MeshType type) {
    switch (type) {
        case MeshType::OTHER: return Lib3MF::eObjectType::Other;
        case MeshType::MODEL: return Lib3MF::eObjectType::Model;
        case MeshType::SUPPORT: return Lib3MF::eObjectType::Support;
        case MeshType::SOLIDSUPPORT: return Lib3MF::eObjectType::SolidSupport;
    }
    return Lib3MF::eObjectType::Model;
}

MeshType to_b3d_type(Lib3MF::eObjectType type) {
    switch (type) {
        case Lib3MF::eObjectType::Other: return MeshType::OTHER;
        case Lib3MF::eObjectType::Model: return MeshType::MODEL;
        case Lib3MF::eObjectType::Support: return MeshType::SUPPORT;
        case Lib3MF::eObjectType::SolidSupport: return MeshType::SOLIDSUPPORT;
    }
    throw std::out_of_range("Unknown 3MF object type");  // Python's KeyError
}

/// os.path.splitext(file_name)[1]
std::string extension(const std::string& file_name) { return std::filesystem::path(file_name).extension().string(); }

/// Mesher._create_3mf_mesh: vertices merged after rounding to TOLERANCE, degenerate
/// triangles dropped.
std::pair<std::vector<Lib3MF::sPosition>, std::vector<Lib3MF::sTriangle>> create_3mf_mesh(const ShapeMesh& mesh) {
    const std::vector<Vector>& ocp_mesh_vertices = mesh.vertices;
    const int digits = -static_cast<int>(pyfmt::round(std::log10(TOLERANCE), 1));
    std::map<std::array<double, 3>, int> vertex_to_idx;
    std::vector<std::array<double, 3>> keys;
    std::vector<int> vert_table(ocp_mesh_vertices.size());
    for (std::size_t i = 0; i < ocp_mesh_vertices.size(); ++i) {
        const Vector& v = ocp_mesh_vertices[i];
        const std::array<double, 3> key{pyfmt::round(v.X, digits), pyfmt::round(v.Y, digits), pyfmt::round(v.Z, digits)};
        const auto it = vertex_to_idx.find(key);
        if (it == vertex_to_idx.end()) {
            vertex_to_idx.emplace(key, static_cast<int>(keys.size()));
            vert_table[i] = static_cast<int>(keys.size());
            keys.push_back(key);
        } else {
            vert_table[i] = it->second;
        }
    }
    std::vector<Lib3MF::sPosition> vertices_3mf;
    for (const auto& v : keys) {
        Lib3MF::sPosition p;
        for (int k = 0; k < 3; ++k) p.m_Coordinates[k] = static_cast<Lib3MF_single>(v[static_cast<std::size_t>(k)]);
        vertices_3mf.push_back(p);
    }
    std::vector<Lib3MF::sTriangle> triangles_3mf;
    for (const auto& tri : mesh.triangles) {
        const int a = vert_table[static_cast<std::size_t>(tri[0])];
        const int b = vert_table[static_cast<std::size_t>(tri[1])];
        const int c = vert_table[static_cast<std::size_t>(tri[2])];
        if (a != b && b != c && c != a) {
            Lib3MF::sTriangle t;
            t.m_Indices[0] = static_cast<Lib3MF_uint32>(a);
            t.m_Indices[1] = static_cast<Lib3MF_uint32>(b);
            t.m_Indices[2] = static_cast<Lib3MF_uint32>(c);
            triangles_3mf.push_back(t);
        }
    }
    return {vertices_3mf, triangles_3mf};
}

const char* type_name(const TopoDS_Shape& s) {
    if (s.IsNull()) return "Compound";
    switch (s.ShapeType()) {
        case TopAbs_VERTEX: return "Vertex";
        case TopAbs_EDGE: return "Edge";
        case TopAbs_WIRE: return "Wire";
        case TopAbs_FACE: return "Face";
        case TopAbs_SHELL: return "Shell";
        case TopAbs_SOLID: return "Solid";
        default: return "Compound";
    }
}

}  // namespace

struct Mesher::Impl {
    Unit unit = Unit::MM;
    Lib3MF::PWrapper wrapper;
    Lib3MF::PModel model;
    std::vector<Lib3MF::PMeshObject> meshes;

    /// Mesher._add_color
    void add_color(const std::optional<Color>& color, const Lib3MF::PMeshObject& mesh_3mf) {
        if (!color) return;
        const Lib3MF::PBaseMaterialGroup base_material_group = model->AddBaseMaterialGroup();
        const auto t = python_color_tuple(*color);
        const Lib3MF::sColor color_lib3mf =
            wrapper->FloatRGBAToColor(static_cast<Lib3MF_single>(t[0]), static_cast<Lib3MF_single>(t[1]),
                                      static_cast<Lib3MF_single>(t[2]), static_cast<Lib3MF_single>(t[3]));
        const Lib3MF_uint32 base_material_id = base_material_group->AddMaterial(python_color_str(*color), color_lib3mf);
        mesh_3mf->SetObjectLevelProperty(base_material_group->GetResourceID(), base_material_id);
    }

    /// Mesher._get_shape
    Shape get_shape(const Lib3MF::PMeshObject& mesh_3mf) const {
        std::vector<Lib3MF::sPosition> positions;
        mesh_3mf->GetVertices(positions);
        std::vector<gp_Pnt> gp_pnts;
        gp_pnts.reserve(positions.size());
        for (const auto& p : positions) gp_pnts.emplace_back(p.m_Coordinates[0], p.m_Coordinates[1], p.m_Coordinates[2]);

        BRepBuilderAPI_Sewing shell_builder;
        const Lib3MF_uint32 count = mesh_3mf->GetTriangleCount();
        for (Lib3MF_uint32 i = 0; i < count; ++i) {
            const Lib3MF::sTriangle tri = mesh_3mf->GetTriangle(i);
            BRepBuilderAPI_MakePolygon polygon_builder(gp_pnts.at(tri.m_Indices[0]), gp_pnts.at(tri.m_Indices[1]),
                                                       gp_pnts.at(tri.m_Indices[2]), true);
            BRepBuilderAPI_MakeFace face_builder(polygon_builder.Wire());
            const TopoDS_Face facet = face_builder.Face();
            GProp_GProps facet_properties;
            BRepGProp::SurfaceProperties(facet, facet_properties);
            if (facet_properties.Mass() != 0) shell_builder.Add(facet);  // Area==0 is an invalid facet
        }
        // The shell(s): if the object has voids there will be several
        shell_builder.Perform();
        const TopoDS_Shape occ_sewed_shape = shell_builder.SewedShape();
        std::vector<TopoDS_Shell> bd_shells;
        if (!occ_sewed_shape.IsNull() && occ_sewed_shape.ShapeType() == TopAbs_COMPOUND) {
            for (TopExp_Explorer explorer(occ_sewed_shape, TopAbs_SHELL); explorer.More(); explorer.Next()) {
                bd_shells.push_back(TopoDS::Shell(explorer.Current()));
            }
        } else {
            if (occ_sewed_shape.IsNull() || occ_sewed_shape.ShapeType() != TopAbs_SHELL) {
                // Python's assert isinstance(occ_sewed_shape, TopoDS_Shell)
                throw std::runtime_error(std::string("Mesher: the mesh sewed into a ") + type_name(occ_sewed_shape) +
                                         ", not a shell");
            }
            bd_shells.push_back(TopoDS::Shell(occ_sewed_shape));
        }
        if (bd_shells.empty()) throw std::invalid_argument("max() arg is an empty sequence");
        // The outer shell: the one with the largest bounding box volume (the first of them)
        std::size_t outer = 0;
        double largest = 0.0;
        for (std::size_t i = 0; i < bd_shells.size(); ++i) {
            const Vector size = Shell(bd_shells[i]).bounding_box().size;
            const double volume = size.X * size.Y * size.Z;
            if (i == 0 || volume > largest) {
                largest = volume;
                outer = i;
            }
        }
        const Shell outer_shell(bd_shells[outer]);
        // The shell isn't water tight: return it, else make a solid
        if (!outer_shell.is_manifold()) return outer_shell;
        BRepBuilderAPI_MakeSolid solid_builder(bd_shells[outer]);
        for (std::size_t i = 0; i < bd_shells.size(); ++i) {
            if (i != outer) solid_builder.Add(bd_shells[i]);
        }
        return Solid(solid_builder.Solid());
    }
};

Mesher::Mesher(Unit unit) : impl_(std::make_unique<Impl>()) {
    impl_->unit = unit;
    const Lib3MF::eModelUnit model_unit = to_3mf_unit(unit);
    lib3mf("Mesher", [&] {
        impl_->wrapper = Lib3MF::CWrapper::loadLibrary();
        impl_->model = impl_->wrapper->CreateModel();
        impl_->model->SetUnit(model_unit);
        return 0;
    });
}

Mesher::~Mesher() = default;
Mesher::Mesher(Mesher&&) noexcept = default;
Mesher& Mesher::operator=(Mesher&&) noexcept = default;

Unit Mesher::model_unit() const { return impl_->unit; }

std::vector<int> Mesher::triangle_counts() const {
    std::vector<int> out;
    for (const auto& m : impl_->meshes) out.push_back(static_cast<int>(m->GetTriangleCount()));
    return out;
}

std::vector<int> Mesher::vertex_counts() const {
    std::vector<int> out;
    for (const auto& m : impl_->meshes) out.push_back(static_cast<int>(m->GetVertexCount()));
    return out;
}

int Mesher::mesh_count() const {
    return lib3mf("Mesher.mesh_count", [&] { return static_cast<int>(impl_->model->GetMeshObjects()->Count()); });
}

std::string Mesher::library_version() const {
    Lib3MF_uint32 major = 0, minor = 0, micro = 0;
    impl_->wrapper->GetLibraryVersion(major, minor, micro);
    return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(micro);
}

void Mesher::add_meta_data(const std::string& name_space, const std::string& name, const std::string& value,
                           const std::string& metadata_type, bool must_preserve) {
    lib3mf("Mesher.add_meta_data", [&] {
        // Get an existing meta data group if there is one
        Lib3MF::PMetaDataGroup mdg = impl_->model->GetMetaDataGroup();
        if (!mdg) {
            // Create a components object to attach the meta data group
            const Lib3MF::PComponentsObject components = impl_->model->AddComponentsObject();
            mdg = components->GetMetaDataGroup();
        }
        mdg->AddMetaData(name_space, name, value, metadata_type, must_preserve);
        return 0;
    });
}

void Mesher::add_code_to_metadata(const std::string& source_file) {
    std::ifstream in(source_file, std::ios::binary);
    if (!in) throw std::runtime_error("No such file or directory: '" + source_file + "'");
    std::ostringstream source_code;
    source_code << in.rdbuf();
    add_meta_data("build123d", std::filesystem::path(source_file).filename().string(), source_code.str(), "cpp",
                  false);
}

std::vector<MeshMetaData> Mesher::get_meta_data() const {
    return lib3mf("Mesher.get_meta_data", [&] {
        const Lib3MF::PMetaDataGroup group = impl_->model->GetMetaDataGroup();
        std::vector<MeshMetaData> contents;
        for (Lib3MF_uint32 i = 0; i < group->GetMetaDataCount(); ++i) {
            const Lib3MF::PMetaData meta_data = group->GetMetaData(i);
            contents.push_back({meta_data->GetNameSpace(), meta_data->GetName(), meta_data->GetType(),
                                meta_data->GetValue()});
        }
        return contents;
    });
}

MeshMetaDataValue Mesher::get_meta_data_by_key(const std::string& name_space, const std::string& name) const {
    return lib3mf("Mesher.get_meta_data_by_key", [&] {
        const Lib3MF::PMetaData meta_data = impl_->model->GetMetaDataGroup()->GetMetaDataByKey(name_space, name);
        if (!meta_data) throw std::runtime_error("No metadata " + name_space + ":" + name);
        return MeshMetaDataValue{meta_data->GetType(), meta_data->GetValue()};
    });
}

std::vector<MeshProperties> Mesher::get_mesh_properties() const {
    return lib3mf("Mesher.get_mesh_properties", [&] {
        std::vector<MeshProperties> properties;
        for (const auto& mesh : impl_->meshes) {
            MeshProperties p;
            p.name = mesh->GetName();
            p.part_number = mesh->GetPartNumber();
            p.type = to_b3d_type(mesh->GetType());
            bool has_uuid = false;
            p.uuid = mesh->GetUUID(has_uuid);
            properties.push_back(p);
        }
        return properties;
    });
}

void Mesher::add_shape(const Shape& shape, double linear_deflection, double angular_deflection, MeshType mesh_type,
                       const std::optional<std::string>& part_number, const std::optional<std::string>& uuid_value) {
    add_shapes({shape}, linear_deflection, angular_deflection, mesh_type, part_number, uuid_value);
}

void Mesher::add_shapes(const std::vector<Shape>& input_shapes, double linear_deflection, double angular_deflection,
                        MeshType mesh_type, const std::optional<std::string>& part_number,
                        const std::optional<std::string>& uuid_value) {
    // The shapes to mesh, with the color Python's color getter gives them (a child without
    // a color takes its parent's)
    std::vector<std::pair<Shape, std::optional<Color>>> shapes;
    for (const Shape& input_shape : input_shapes) {
        // A null shape is an empty Compound (Python's Part())
        const bool compound = input_shape.is_null() || input_shape.wrapped().ShapeType() == TopAbs_COMPOUND;
        if (compound) {
            if (!input_shape.children.empty()) {
                for (const Shape& child : input_shape.children) {
                    shapes.emplace_back(child, child.color ? child.color : input_shape.color);
                }
            } else if (!input_shape.label.empty() || input_shape.color) {
                shapes.emplace_back(input_shape, input_shape.color);
            } else {
                for (const TopoDS_Shape& s : children(input_shape.wrapped())) shapes.emplace_back(Shape(s), std::nullopt);
            }
        } else {
            shapes.emplace_back(input_shape, input_shape.color);
        }
    }

    Impl& d = *impl_;
    for (const auto& [b3d_shape, color] : shapes) {
        // A 3MF mesh object
        const Lib3MF::PMeshObject mesh_3mf = lib3mf("Mesher.add_shape", [&] { return d.model->AddMeshObject(); });

        // Mesh a copy of the shape (without its triangulation)
        if (b3d_shape.is_null()) throw std::invalid_argument("Cannot mesh an empty shape");
        const ShapeMesh mesh = occt("Mesher.add_shape", [&] {
            const Shape copy(BRepBuilderAPI_Copy(b3d_shape.wrapped(), true, false).Shape());
            return mesh_shape(copy, linear_deflection, angular_deflection);
        });

        // Skip invalid meshes
        if (mesh.vertices.size() < 3 || mesh.triangles.empty()) {
            warn(std::string("Degenerate shape ") + type_name(b3d_shape.wrapped()) + " - skipped");
            continue;
        }

        const auto [vertices_3mf, triangles_3mf] = create_3mf_mesh(mesh);
        lib3mf("Mesher.add_shape", [&] {
            mesh_3mf->SetGeometry(vertices_3mf, triangles_3mf);
            // The mesh properties
            mesh_3mf->SetType(to_3mf_type(mesh_type));
            if (!b3d_shape.label.empty()) mesh_3mf->SetName(b3d_shape.label);
            if (part_number && !part_number->empty()) mesh_3mf->SetPartNumber(*part_number);
            if (uuid_value && !uuid_value->empty()) mesh_3mf->SetUUID(*uuid_value);
            d.add_color(color, mesh_3mf);
            return 0;
        });

        // Check the mesh
        if (!mesh_3mf->IsValid()) throw std::runtime_error("3mf mesh is invalid");
        if (!mesh_3mf->IsManifoldAndOriented()) warn("3mf mesh is not manifold");

        lib3mf("Mesher.add_shape", [&] {
            d.meshes.push_back(mesh_3mf);
            d.model->AddBuildItem(mesh_3mf.get(), d.wrapper->GetIdentityTransform());
            // As Python (which isn't sure it's required)
            const Lib3MF::PComponentsObject components = d.model->AddComponentsObject();
            components->AddComponent(mesh_3mf.get(), d.wrapper->GetIdentityTransform());
            return 0;
        });
    }
}

ShapeList<Shape> Mesher::read(const std::string& file_name) {
    const std::string ext = extension(file_name);
    if (ext != ".3mf" && ext != ".stl") throw std::invalid_argument("Unknown file format " + ext);
    Impl& d = *impl_;
    lib3mf("Mesher.read", [&] {
        const Lib3MF::PReader reader = d.model->QueryReader(ext.substr(1));
        reader->ReadFromFile(file_name);
        return 0;
    });
    d.unit = to_b3d_unit(d.model->GetUnit());

    // The 3MF meshes as OCCT shapes
    lib3mf("Mesher.read", [&] {
        const Lib3MF::PMeshObjectIterator mesh_iterator = d.model->GetMeshObjects();
        const Lib3MF_uint64 count = mesh_iterator->Count();
        for (Lib3MF_uint64 i = 0; i < count; ++i) {
            mesh_iterator->MoveNext();
            d.meshes.push_back(mesh_iterator->GetCurrentMeshObject());
        }
        return 0;
    });
    ShapeList<Shape> shapes;
    for (const auto& mesh : d.meshes) {
        Shape shape = occt("Mesher.read", [&] { return lib3mf("Mesher.read", [&] { return d.get_shape(mesh); }); });
        lib3mf("Mesher.read", [&] {
            shape.label = mesh->GetName();
            // The color
            Lib3MF_uint32 base_mat_id = 0, color_index = 0;
            if (mesh->GetObjectLevelProperty(base_mat_id, color_index)) {
                const Lib3MF::PBaseMaterialGroup base_mat = d.model->GetBaseMaterialGroupByID(base_mat_id);
                const Lib3MF::sColor base_mat_color = base_mat->GetDisplayColor(color_index);
                Lib3MF_single r = 0, g = 0, b = 0, a = 0;
                d.wrapper->ColorToFloatRGBA(base_mat_color, r, g, b, a);
                shape.color = Color(r, g, b, a);
            }
            return 0;
        });
        shapes.push_back(shape);
    }
    return shapes;
}

void Mesher::write(const std::string& file_name) const {
    const std::string ext = extension(file_name);
    if (ext != ".3mf" && ext != ".stl") throw std::invalid_argument("Unknown file format " + ext);
    lib3mf("Mesher.write", [&] {
        impl_->model->QueryWriter(ext.substr(1))->WriteToFile(file_name);
        return 0;
    });
}

void Mesher::write_stream(std::ostream& stream, const std::string& file_type) const {
    if (file_type != "3mf" && file_type != "stl") {
        throw std::invalid_argument("Unknown file format - must be 3mf or stl");
    }
    std::vector<Lib3MF_uint8> buffer;
    lib3mf("Mesher.write_stream", [&] {
        impl_->model->QueryWriter(file_type)->WriteToBuffer(buffer);
        return 0;
    });
    stream.write(reinterpret_cast<const char*>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
}

}  // namespace build123d
