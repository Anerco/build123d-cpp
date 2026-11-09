// 3D import and export (exporters3d.py, importers.py).
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <iterator>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

#include <APIHeaderSection_MakeHeader.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_PrinterOStream.hxx>
#include <Message_ProgressRange.hxx>
#include <Poly_Triangulation.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <RWGltf_CafWriter.hxx>
#include <RWStl.hxx>
#include <STEPCAFControl_Controller.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Controller.hxx>
#include <STEPControl_StepModelType.hxx>
#include <StdFail_NotDone.hxx>
#include <StlAPI_Writer.hxx>
#include <Standard_Version.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XSControl_WorkSession.hxx>
#include <gp_Trsf.hxx>
#if OCC_VERSION_HEX < 0x080000
#include <TColStd_IndexedDataMapOfStringString.hxx>
#endif

#include "build123d/io.hpp"
#include "internal.hpp"
#include "io_internal.hpp"
#include "pyformat.hpp"

namespace build123d {

using namespace detail;

void detail::quiet_messages() {
    const Handle(Message_Messenger)& messenger = Message::DefaultMessenger();
    for (const auto& printer : messenger->Printers()) printer->SetTraceLevel(Message_Fail);
}

namespace {

// As build123d: 1 / UNITS_PER_METER[unit].
double meters_per_unit(Unit unit) { return 1.0 / units_per_meter(unit); }

TDF_Label referred(const TDF_Label& label) {
    if (label.IsNull()) return label;
    if (XCAFDoc_ShapeTool::IsReference(label)) {
        TDF_Label ref;
        if (XCAFDoc_ShapeTool::GetReferredShape(label, ref) && !ref.IsNull()) return ref;
    }
    return label;
}

void set_name_and_color(const Shape& node, const std::optional<Color>& node_color, const TDF_Label& label,
                        const Handle(XCAFDoc_ShapeTool)& shape_tool, const Handle(XCAFDoc_ColorTool)& color_tool) {
    if (label.IsNull()) return;
    if (!node.label.empty()) {
        TDataStd_Name::Set(label, TCollection_ExtendedString(node.label.c_str(), true));
        if (XCAFDoc_ShapeTool::IsReference(label)) {
            TDataStd_Name::Set(referred(label), TCollection_ExtendedString(node.label.c_str(), true));
        }
    }
    if (node_color) {
        const Quantity_ColorRGBA c = node_color->wrapped();
        color_tool->SetColor(label, c, XCAFDoc_ColorGen);
        if (XCAFDoc_ShapeTool::IsReference(label)) color_tool->SetColor(referred(label), c, XCAFDoc_ColorGen);
        if (node.wrapped().ShapeType() == TopAbs_COMPOUND && node.children.empty()) {
            TopAbs_ShapeEnum type = TopAbs_SHAPE;
            XCAFDoc_ColorType color_type = XCAFDoc_ColorSurf;
            switch (node.dim()) {
                case 3: type = TopAbs_SOLID; break;
                case 2: type = TopAbs_FACE; break;
                case 1:
                    type = TopAbs_EDGE;
                    color_type = XCAFDoc_ColorCurv;
                    break;
                default: break;
            }
            if (type != TopAbs_SHAPE) {
                const TDF_Label base = referred(label);
                for (TopExp_Explorer exp(node.wrapped(), type); exp.More(); exp.Next()) {
                    const TDF_Label sub = shape_tool->AddSubShape(base, exp.Current());
                    if (!sub.IsNull()) color_tool->SetColor(sub, c, color_type);
                }
            }
        }
    }
}

/// Adds a node of the assembly tree and its children (_create_xde's PreOrderIter pass). As
/// Python's Shape.color getter, a node without a color takes its nearest ancestor's.
void add_node(const Shape& node, const TDF_Label& parent, const Handle(XCAFDoc_ShapeTool)& shape_tool,
              const Handle(XCAFDoc_ColorTool)& color_tool, const std::optional<Color>& inherited_color) {
    const std::optional<Color> node_color = node.color ? node.color : inherited_color;
    if (node.is_null()) return;
    TDF_Label label;
    if (parent.IsNull()) {
        label = shape_tool->AddShape(node.wrapped(), false);
    } else {
        const TDF_Label parent_label = referred(parent);
        if (parent_label.IsNull()) return;
        label = shape_tool->AddComponent(parent_label, node.wrapped());
    }
    if (label.IsNull()) return;
    if (!node.label.empty() || node_color) set_name_and_color(node, node_color, label, shape_tool, color_tool);
    for (const auto& child : node.children) add_node(child, label, shape_tool, color_tool, node_color);
}

Handle(TDocStd_Document) create_xde(const Shape& to_export, Unit unit, bool auto_naming) {
    Handle(TDocStd_Document) doc;
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    app->NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc);
    XCAFDoc_DocumentTool::SetLengthUnit(doc, meters_per_unit(unit));
    Handle(XCAFDoc_ShapeTool) shape_tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) color_tool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    XCAFDoc_ShapeTool::SetAutoNaming(auto_naming);
    add_node(to_export, TDF_Label(), shape_tool, color_tool, std::nullopt);
    shape_tool->UpdateAssemblies();
    return doc;
}

/// export_step up to the point of writing; `write` writes the file or stream.
bool write_step(const Shape& to_export, Unit unit, bool write_pcurves, PrecisionMode precision_mode,
                const std::optional<std::string>& timestamp,
                const std::function<IFSelect_ReturnStatus(STEPCAFControl_Writer&)>& write) {
    if (to_export.is_null()) throw std::invalid_argument("Cannot export an empty shape");
    return occt("export_step", [&] {
        Handle(TDocStd_Document) doc = create_xde(to_export, unit, true);
        quiet_messages();
        Handle(XSControl_WorkSession) session = new XSControl_WorkSession();
        STEPCAFControl_Writer writer(session, false);
        writer.SetColorMode(true);
        writer.SetLayerMode(true);
        writer.SetNameMode(true);
        APIHeaderSection_MakeHeader header(writer.ChangeWriter().Model());
        if (!header.IsDone()) {
            header = APIHeaderSection_MakeHeader(0);
            header.Apply(writer.ChangeWriter().Model());
        }
        if (!to_export.label.empty()) header.SetName(new TCollection_HAsciiString(to_export.label.c_str()));
        if (timestamp) header.SetTimeStamp(new TCollection_HAsciiString(timestamp->c_str()));
        header.SetOriginatingSystem(new TCollection_HAsciiString("build123d"));
        STEPCAFControl_Controller::Init();
        STEPControl_Controller::Init();
        Interface_Static::SetIVal("write.surfacecurve.mode", write_pcurves ? 1 : 0);
        Interface_Static::SetIVal("write.precision.mode", static_cast<int>(precision_mode));
        writer.Transfer(doc, STEPControl_AsIs);
        if (write(writer) != IFSelect_RetDone) throw std::runtime_error("Failed to write STEP file");
        return true;
    });
}

}  // namespace

bool export_step(const Shape& to_export, const std::string& file_path, Unit unit, bool write_pcurves,
                 PrecisionMode precision_mode, const std::optional<std::string>& timestamp) {
    return write_step(to_export, unit, write_pcurves, precision_mode, timestamp,
                      [&](STEPCAFControl_Writer& writer) { return writer.Write(file_path.c_str()); });
}

bool export_step(const Shape& to_export, std::ostream& stream, Unit unit, bool write_pcurves,
                 PrecisionMode precision_mode, const std::optional<std::string>& timestamp) {
    return write_step(to_export, unit, write_pcurves, precision_mode, timestamp, [&](STEPCAFControl_Writer& writer) {
#if OCC_VERSION_HEX >= 0x070700
        return writer.WriteStream(stream);
#else
        // No stream output before OCCT 7.7: go through a temporary file
        const auto path = std::filesystem::temp_directory_path() /
                          ("build123d_step_" + std::to_string(reinterpret_cast<std::uintptr_t>(&writer)) + ".step");
        const IFSelect_ReturnStatus status = writer.Write(path.string().c_str());
        if (status == IFSelect_RetDone) {
            std::ifstream in(path, std::ios::binary);
            stream << in.rdbuf();
        }
        std::error_code ec;
        std::filesystem::remove(path, ec);
        return status;
#endif
    });
}

std::string isoformat(const std::tm& time) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d", time.tm_year + 1900, time.tm_mon + 1,
                  time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    return buf;
}

bool export_stl(const Shape& to_export, const std::string& file_path, double tolerance, double angular_tolerance,
                bool ascii_format) {
    const auto parent = std::filesystem::path(file_path).parent_path();
    if (!std::filesystem::is_directory(parent.empty() ? std::filesystem::path(".") : parent)) {
        throw std::runtime_error("Directory does not exist: " + parent.string());
    }
    if (to_export.is_null()) throw std::invalid_argument("Cannot export an empty shape");
    return occt("export_stl", [&] {
        BRepMesh_IncrementalMesh mesh(to_export.wrapped(), tolerance, true, angular_tolerance, true);
        mesh.Perform();
        StlAPI_Writer writer;
        writer.ASCIIMode() = ascii_format;
        return static_cast<bool>(writer.Write(to_export.wrapped(), file_path.c_str()));
    });
}

bool export_brep(const Shape& to_export, const std::string& file_path) {
    if (to_export.is_null()) throw std::invalid_argument("Cannot export an empty shape");
    return occt("export_brep", [&] { return static_cast<bool>(BRepTools::Write(to_export.wrapped(), file_path.c_str())); });
}

bool export_brep(const Shape& to_export, std::ostream& stream) {
    if (to_export.is_null()) throw std::invalid_argument("Cannot export an empty shape");
    return occt("export_brep", [&] {
        BRepTools::Write(to_export.wrapped(), stream);
        return static_cast<bool>(stream);
    });
}

namespace {

/// PreOrderIter over an assembly tree.
void preorder(const Shape& node, const std::function<void(const Shape&)>& visit) {
    visit(node);
    for (const auto& child : node.children) preorder(child, visit);
}

}  // namespace

bool export_gltf(const Shape& to_export, const std::string& file_path, Unit unit, bool binary,
                 double linear_deflection, double angular_deflection) {
    if (to_export.is_null()) throw std::invalid_argument("Can't find the location of an empty shape");
    // Map from OCCT's +Z up to glTF's +Y up
    Shape rotated = to_export;
    rotated.set_location(to_export.location() * Location(Vector(0, 0, 0), Vector(1, 0, 0), -90.0));
    return occt("export_gltf", [&] {
        // Tessellate the object(s)
        preorder(rotated, [&](const Shape& node) {
            if (!node.is_null()) node.mesh(linear_deflection, angular_deflection);
        });
        Handle(TDocStd_Document) doc = create_xde(rotated, unit, false);
        RWGltf_CafWriter writer(TCollection_AsciiString(file_path.c_str()), binary);
#if OCC_VERSION_HEX >= 0x070700
        writer.SetParallel(true);
#endif
        // Textures need UVs in the gltf export
        writer.SetForcedUVExport(true);
#if OCC_VERSION_HEX >= 0x080000
        NCollection_IndexedDataMap<TCollection_AsciiString, TCollection_AsciiString> file_info;
#else
        TColStd_IndexedDataMapOfStringString file_info;
#endif
        quiet_messages();
        const bool status = writer.Perform(doc, file_info, Message_ProgressRange());
        // Reset tessellation (removes all triangulations including any UV changes)
        BRepTools::Clean(rotated.wrapped());
        return status;
    });
}

// ---------------------------------------------------------------------------
// OBJ
// ---------------------------------------------------------------------------

bool export_obj(const Shape& to_export, std::ostream& stream, double linear_deflection, double angular_deflection,
                bool include_uvs, bool atlas_packing, double atlas_gutter) {
    if (to_export.is_null()) throw std::invalid_argument("Cannot tessellate an empty shape");
    UvMesh mesh;
    if (include_uvs) {
        mesh = tessellate_with_uvs(to_export, linear_deflection, angular_deflection, atlas_packing, atlas_gutter);
    } else {
        auto [vertices, triangles] = to_export.tessellate(linear_deflection, angular_deflection);
        mesh.vertices = std::move(vertices);
        mesh.triangles = std::move(triangles);
    }
    std::string content = "# Wavefront OBJ exported by build123d\n";
    auto g = [](double x) { return pyfmt::format_g(x, 9); };
    for (const auto& v : mesh.vertices) content += "v " + g(v.X) + " " + g(v.Y) + " " + g(v.Z) + "\n";
    if (include_uvs) {
        for (const auto& [u, v] : mesh.uvs) content += "vt " + g(u) + " " + g(v) + "\n";
        for (const auto& n : mesh.normals) content += "vn " + g(n.X) + " " + g(n.Y) + " " + g(n.Z) + "\n";
        for (const auto& t : mesh.triangles) {
            const std::string a = std::to_string(t[0] + 1), b = std::to_string(t[1] + 1), c = std::to_string(t[2] + 1);
            content += "f " + a + "/" + a + "/" + a + " " + b + "/" + b + "/" + b + " " + c + "/" + c + "/" + c + "\n";
        }
    } else {
        for (const auto& t : mesh.triangles) {
            content += "f " + std::to_string(t[0] + 1) + " " + std::to_string(t[1] + 1) + " " +
                       std::to_string(t[2] + 1) + "\n";
        }
    }
    stream << content;
    return static_cast<bool>(stream);
}

bool export_obj(const Shape& to_export, const std::string& file_path, double linear_deflection,
                double angular_deflection, bool include_uvs, bool atlas_packing, double atlas_gutter) {
    std::ostringstream content;
    export_obj(to_export, content, linear_deflection, angular_deflection, include_uvs, atlas_packing, atlas_gutter);
    std::ofstream out(file_path, std::ios::binary);
    if (!out) throw std::runtime_error("No such file or directory: '" + file_path + "'");
    out << content.str();
    return static_cast<bool>(out);
}

// ---------------------------------------------------------------------------
// Importers
// ---------------------------------------------------------------------------

Shape import_brep(const std::string& file_path) {
    return occt("import_brep", [&] {
        TopoDS_Shape shape;
        BRep_Builder builder;
        BRepTools::Read(shape, file_path.c_str(), builder);
        if (shape.IsNull()) throw std::invalid_argument("Could not import " + file_path);
        return Shape(shape);
    });
}

namespace {

const std::pair<char32_t, char32_t> kControlRanges[] = {
#include "unicode_c_ranges.inc"
};

/// Python's unicodedata.category(ch)[0] == "C".
bool is_unicode_c(char32_t cp) {
    const auto* begin = std::begin(kControlRanges);
    const auto* end = std::end(kControlRanges);
    const auto* it = std::upper_bound(begin, end, cp, [](char32_t c, const auto& r) { return c < r.first; });
    return it != begin && cp <= (it - 1)->second;
}

std::u32string utf8_decode(const std::string& s) {
    std::u32string out;
    const auto* p = reinterpret_cast<const unsigned char*>(s.data());
    const auto* end = p + s.size();
    while (p < end) {
        char32_t cp = *p;
        int extra = 0;
        if (cp >= 0xF0) {
            cp &= 0x07;
            extra = 3;
        } else if (cp >= 0xE0) {
            cp &= 0x0F;
            extra = 2;
        } else if (cp >= 0xC0) {
            cp &= 0x1F;
            extra = 1;
        }
        ++p;
        for (int i = 0; i < extra && p < end; ++i, ++p) cp = (cp << 6) | (*p & 0x3F);
        out += cp;
    }
    return out;
}

void utf8_append(std::string& out, char32_t cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

/// import_step's get_name: the label's name without control/format/unassigned characters,
/// with " .()" replaced by "_".
std::string get_name(const TDF_Label& label) {
    std::string name;
    Handle(TDataStd_Name) std_name;
    if (label.IsAttribute(TDataStd_Name::GetID()) && label.FindAttribute(TDataStd_Name::GetID(), std_name)) {
        name = TCollection_AsciiString(std_name->Get()).ToCString();
    }
    std::string clean;
    for (const char32_t cp : utf8_decode(name)) {
        if (is_unicode_c(cp)) continue;
        if (cp == ' ' || cp == '.' || cp == '(' || cp == ')') {
            clean += '_';
        } else {
            utf8_append(clean, cp);
        }
    }
    return clean;
}

Color to_color(const Quantity_ColorRGBA& c) {
    double r, g, b;
    c.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
    return Color(r, g, b, c.Alpha());
}

class StepReader {
public:
    StepReader(Handle(XCAFDoc_ShapeTool) shape_tool, Handle(XCAFDoc_ColorTool) color_tool)
        : shape_tool_(std::move(shape_tool)), color_tool_(std::move(color_tool)) {}

    /// Recursively extract objects into an assembly.
    std::vector<Shape> build_assembly(const TDF_Label* parent_tdf_label = nullptr) {
        TDF_LabelSequence sub_tdf_labels;
        if (parent_tdf_label == nullptr) {
            shape_tool_->GetFreeShapes(sub_tdf_labels);
        } else {
            XCAFDoc_ShapeTool::GetComponents(*parent_tdf_label, sub_tdf_labels);
        }
        std::vector<Shape> sub_shapes;
        for (int i = 1; i <= sub_tdf_labels.Length(); ++i) {
            const TDF_Label sub_tdf_label = sub_tdf_labels.Value(i);
            TDF_Label ref_tdf_label;
            if (XCAFDoc_ShapeTool::IsReference(sub_tdf_label)) {
                XCAFDoc_ShapeTool::GetReferredShape(sub_tdf_label, ref_tdf_label);
            } else {
                ref_tdf_label = sub_tdf_label;
            }
            const TopoDS_Shape sub_topo_shape = XCAFDoc_ShapeTool::GetShape(ref_tdf_label);
            Shape sub_shape;
            if (XCAFDoc_ShapeTool::IsAssembly(ref_tdf_label)) {
                // An empty Compound whose children are the components
                sub_shape = Compound::make_assembly(build_assembly(&ref_tdf_label));
            } else {
                sub_shape = Shape(sub_topo_shape);
            }
            // Priority: instance/component label -> referred shape label -> geometric fallback
            const auto instance_color = get_label_color(sub_tdf_label);
            const auto referred_color = get_label_color(ref_tdf_label);
            const auto shape_fallback_color = get_color(sub_topo_shape);
            const auto& color = instance_color ? instance_color : (referred_color ? referred_color : shape_fallback_color);
            sub_shape.color = color ? std::optional<Color>(to_color(*color)) : std::nullopt;
            sub_shape.label = get_name(ref_tdf_label);
            if (!sub_shape.is_null()) sub_shape.move(Location(XCAFDoc_ShapeTool::GetLocation(sub_tdf_label)));
            sub_shapes.push_back(std::move(sub_shape));
        }
        return sub_shapes;
    }

private:
    Handle(XCAFDoc_ShapeTool) shape_tool_;
    Handle(XCAFDoc_ColorTool) color_tool_;
    // Retrieving color info is expensive so cache the lookups
    std::map<const void*, std::optional<Quantity_ColorRGBA>> color_cache_;

    std::optional<Quantity_ColorRGBA> get_shape_color_from_cache(const TopoDS_Shape& obj) {
        const void* key = obj.TShape().get();
        const auto it = color_cache_.find(key);
        if (it != color_cache_.end()) return it->second;
        Quantity_ColorRGBA col;
        const bool has_color = color_tool_->GetColor(obj, XCAFDoc_ColorCurv, col) ||
                               color_tool_->GetColor(obj, XCAFDoc_ColorGen, col) ||
                               color_tool_->GetColor(obj, XCAFDoc_ColorSurf, col);
        std::optional<Quantity_ColorRGBA> result;
        if (has_color) result = col;
        color_cache_[key] = result;
        return result;
    }

    /// The shape's color, else that of its largest colored face.
    std::optional<Quantity_ColorRGBA> get_color(const TopoDS_Shape& shape) {
        if (shape.IsNull()) return std::nullopt;
        if (auto shape_color = get_shape_color_from_cache(shape)) return shape_color;
        double max_extent = -1.0;
        std::optional<Quantity_ColorRGBA> winner;
        for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
            const TopoDS_Shape& face = exp.Current();
            const auto col = get_shape_color_from_cache(face);
            if (col) {
                Bnd_Box box;
                BRepBndLib::Add(face, box);
                const double extent = box.SquareExtent();
                if (extent > max_extent) {
                    max_extent = extent;
                    winner = col;
                }
            }
        }
        return winner;
    }

    static std::optional<Quantity_ColorRGBA> get_label_color(const TDF_Label& label) {
        Quantity_ColorRGBA col;
        if (XCAFDoc_ColorTool::GetColor(label, XCAFDoc_ColorCurv, col) ||
            XCAFDoc_ColorTool::GetColor(label, XCAFDoc_ColorGen, col) ||
            XCAFDoc_ColorTool::GetColor(label, XCAFDoc_ColorSurf, col)) {
            return col;
        }
        return std::nullopt;
    }
};

}  // namespace

Shape import_step(const std::string& file_path) {
    if (!std::filesystem::exists(file_path)) throw std::runtime_error("File not found: " + file_path);
    return occt("import_step", [&] {
        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc);
        quiet_messages();
        STEPCAFControl_Reader reader;
        reader.SetNameMode(true);
        reader.SetColorMode(true);
        reader.SetLayerMode(true);
        reader.ReadFile(file_path.c_str());
        reader.Transfer(doc);
        StepReader step(XCAFDoc_DocumentTool::ShapeTool(doc->Main()), XCAFDoc_DocumentTool::ColorTool(doc->Main()));
        std::vector<Shape> children = step.build_assembly();
        // Remove the empty Compound wrapper if there is a single free object
        if (children.size() == 1) return children[0];
        return static_cast<Shape>(Compound::make_assembly(children));
    });
}

Face import_stl(const std::string& file_path, Unit model_unit) {
    if (!std::filesystem::exists(file_path)) throw std::runtime_error("File not found: " + file_path);
    return occt("import_stl", [&] {
        Handle(Poly_Triangulation) reader = RWStl::ReadFile(file_path.c_str());
        if (reader.IsNull()) throw std::runtime_error("Failed to read " + file_path);
        // Check for any required scaling
        if (model_unit != Unit::MM) {
            double scale_factor = 1.0;
            switch (model_unit) {
                case Unit::MC: scale_factor = MC; break;
                case Unit::CM: scale_factor = CM; break;
                case Unit::M: scale_factor = M; break;
                case Unit::IN: scale_factor = IN; break;
                case Unit::FT: scale_factor = FT; break;
                default:
                    throw std::invalid_argument("model_scale must be one of: MC, MM, CM, M, IN, FT");
            }
            gp_Trsf transformation;
            transformation.SetScaleFactor(scale_factor);
            for (int i = 1; i <= reader->NbNodes(); ++i) {
                gp_Pnt p = reader->Node(i);
                p.Transform(transformation);
                reader->SetNode(i, p);
            }
        }
        TopoDS_Face face;
        BRep_Builder().MakeFace(face, reader);
        return Face(face);
    });
}

}  // namespace build123d
