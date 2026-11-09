#include <filesystem>

#include <APIHeaderSection_MakeHeader.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
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
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XSControl_WorkSession.hxx>
#if OCC_VERSION_HEX < 0x080000
#include <TColStd_IndexedDataMapOfStringString.hxx>
#endif

#include "build123d/io.hpp"
#include "internal.hpp"

namespace build123d {

using namespace detail;

namespace {

// As build123d: 1 / UNITS_PER_METER[unit].
double meters_per_unit(Unit unit) { return 1.0 / units_per_meter(unit); }

void quiet_messages() {
    const Handle(Message_Messenger)& messenger = Message::DefaultMessenger();
    for (const auto& printer : messenger->Printers()) printer->SetTraceLevel(Message_Fail);
}

void check_directory(const std::string& file_path) {
    const auto parent = std::filesystem::path(file_path).parent_path();
    if (!parent.empty() && !std::filesystem::is_directory(parent)) {
        throw std::runtime_error("Directory does not exist: " + parent.string());
    }
}

TDF_Label referred(const TDF_Label& label) {
    if (label.IsNull()) return label;
    if (XCAFDoc_ShapeTool::IsReference(label)) {
        TDF_Label ref;
        if (XCAFDoc_ShapeTool::GetReferredShape(label, ref) && !ref.IsNull()) return ref;
    }
    return label;
}

void set_name_and_color(const Shape& node, const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shape_tool,
                        const Handle(XCAFDoc_ColorTool)& color_tool) {
    if (label.IsNull()) return;
    if (!node.label.empty()) {
        TDataStd_Name::Set(label, TCollection_ExtendedString(node.label.c_str(), true));
        if (XCAFDoc_ShapeTool::IsReference(label)) {
            TDataStd_Name::Set(referred(label), TCollection_ExtendedString(node.label.c_str(), true));
        }
    }
    if (node.color) {
        const Quantity_ColorRGBA c = node.color->wrapped();
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

void add_node(const Shape& node, const TDF_Label& parent, const Handle(XCAFDoc_ShapeTool)& shape_tool,
              const Handle(XCAFDoc_ColorTool)& color_tool) {
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
    if (!node.label.empty() || node.color) set_name_and_color(node, label, shape_tool, color_tool);
    for (const auto& child : node.children) add_node(child, label, shape_tool, color_tool);
}

Handle(TDocStd_Document) create_xde(const Shape& to_export, Unit unit, bool auto_naming) {
    Handle(TDocStd_Document) doc;
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    app->NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc);
    XCAFDoc_DocumentTool::SetLengthUnit(doc, meters_per_unit(unit));
    Handle(XCAFDoc_ShapeTool) shape_tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) color_tool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    XCAFDoc_ShapeTool::SetAutoNaming(auto_naming);
    add_node(to_export, TDF_Label(), shape_tool, color_tool);
    shape_tool->UpdateAssemblies();
    return doc;
}

}  // namespace

bool export_step(const Shape& to_export, const std::string& file_path, Unit unit, bool write_pcurves,
                 PrecisionMode precision_mode) {
    if (to_export.is_null()) throw std::invalid_argument("Cannot export an empty shape");
    check_directory(file_path);
    return occt("export_step", [&] {
        Handle(TDocStd_Document) doc = create_xde(to_export, unit, true);
        quiet_messages();
        STEPCAFControl_Controller::Init();
        STEPControl_Controller::Init();
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
        header.SetOriginatingSystem(new TCollection_HAsciiString("build123d-cpp"));
        Interface_Static::SetIVal("write.surfacecurve.mode", write_pcurves ? 1 : 0);
        Interface_Static::SetIVal("write.precision.mode", static_cast<int>(precision_mode));
        writer.Transfer(doc, STEPControl_AsIs);
        const IFSelect_ReturnStatus status = writer.Write(file_path.c_str());
        if (status != IFSelect_RetDone) throw std::runtime_error("Failed to write STEP file");
        return true;
    });
}

bool export_stl(const Shape& to_export, const std::string& file_path, double tolerance, double angular_tolerance,
                bool ascii_format) {
    if (to_export.is_null()) throw std::invalid_argument("Cannot export an empty shape");
    check_directory(file_path);
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
    check_directory(file_path);
    return occt("export_brep", [&] { return static_cast<bool>(BRepTools::Write(to_export.wrapped(), file_path.c_str())); });
}

bool export_gltf(const Shape& to_export, const std::string& file_path, Unit unit, bool binary,
                 double linear_deflection, double angular_deflection) {
    if (to_export.is_null()) throw std::invalid_argument("Cannot export an empty shape");
    check_directory(file_path);
    // OCCT is +Z up, glTF is +Y up.
    Shape rotated = to_export;
    rotated.set_location(to_export.location() * Location(Vector(0, 0, 0), Vector(1, 0, 0), -90.0));
    return occt("export_gltf", [&] {
        BRepMesh_IncrementalMesh(rotated.wrapped(), linear_deflection, true, angular_deflection, true);
        for (const auto& child : rotated.children) {
            if (!child.is_null()) BRepMesh_IncrementalMesh(child.wrapped(), linear_deflection, true, angular_deflection, true);
        }
        Handle(TDocStd_Document) doc = create_xde(rotated, unit, false);
        RWGltf_CafWriter writer(TCollection_AsciiString(file_path.c_str()), binary);
#if OCC_VERSION_HEX >= 0x070700
        writer.SetParallel(true);
#endif
#if OCC_VERSION_HEX >= 0x080000
        NCollection_IndexedDataMap<TCollection_AsciiString, TCollection_AsciiString> file_info;
#else
        TColStd_IndexedDataMapOfStringString file_info;
#endif
        quiet_messages();
        const bool status = writer.Perform(doc, file_info, Message_ProgressRange());
        BRepTools::Clean(rotated.wrapped());
        return status;
    });
}

Shape import_brep(const std::string& file_path) {
    if (!std::filesystem::exists(file_path)) throw std::runtime_error("File not found: " + file_path);
    return occt("import_brep", [&] {
        TopoDS_Shape shape;
        BRep_Builder builder;
        if (!BRepTools::Read(shape, file_path.c_str(), builder)) throw std::runtime_error("Failed to read " + file_path);
        return Shape(shape);
    });
}

namespace {

std::string label_name(const TDF_Label& label) {
    Handle(TDataStd_Name) name;
    if (label.FindAttribute(TDataStd_Name::GetID(), name)) {
        return TCollection_AsciiString(name->Get()).ToCString();
    }
    return "";
}

std::optional<Color> label_color(const TDF_Label& label, const Handle(XCAFDoc_ColorTool)& color_tool) {
    Quantity_ColorRGBA c;
    for (auto type : {XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_ColorCurv}) {
        if (color_tool->GetColor(label, type, c)) {
            double r, g, b;
            c.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
            return Color(r, g, b, c.Alpha());
        }
    }
    return std::nullopt;
}

Shape build_node(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shape_tool,
                 const Handle(XCAFDoc_ColorTool)& color_tool) {
    const TDF_Label ref = referred(label);
    Shape node(XCAFDoc_ShapeTool::GetShape(label));
    node.label = label_name(label);
    if (node.label.empty()) node.label = label_name(ref);
    node.color = label_color(label, color_tool);
    if (!node.color) node.color = label_color(ref, color_tool);
    if (XCAFDoc_ShapeTool::IsAssembly(ref)) {
        TDF_LabelSequence components;
        XCAFDoc_ShapeTool::GetComponents(ref, components);
        for (int i = 1; i <= components.Length(); ++i) {
            node.children.push_back(build_node(components.Value(i), shape_tool, color_tool));
        }
    }
    return node;
}

}  // namespace

Compound import_step(const std::string& file_path) {
    if (!std::filesystem::exists(file_path)) throw std::runtime_error("File not found: " + file_path);
    return occt("import_step", [&] {
        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument(TCollection_ExtendedString("MDTV-XCAF"), doc);
        quiet_messages();
        STEPCAFControl_Reader reader;
        reader.SetColorMode(true);
        reader.SetNameMode(true);
        reader.SetLayerMode(true);
        if (reader.ReadFile(file_path.c_str()) != IFSelect_RetDone) throw std::runtime_error("Failed to read " + file_path);
        if (!reader.Transfer(doc)) throw std::runtime_error("Failed to transfer " + file_path);
        Handle(XCAFDoc_ShapeTool) shape_tool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        Handle(XCAFDoc_ColorTool) color_tool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
        TDF_LabelSequence free_shapes;
        shape_tool->GetFreeShapes(free_shapes);
        std::vector<Shape> roots;
        for (int i = 1; i <= free_shapes.Length(); ++i) roots.push_back(build_node(free_shapes.Value(i), shape_tool, color_tool));
        if (roots.size() == 1 && roots[0].wrapped().ShapeType() == TopAbs_COMPOUND) {
            Compound result(roots[0].wrapped());
            result.label = roots[0].label;
            result.color = roots[0].color;
            result.children = roots[0].children;
            return result;
        }
        Compound result = Compound::make_assembly(roots);
        if (roots.size() == 1) {
            // A single free shape: no assembly children.
            Compound single(result.wrapped());
            single.label = roots[0].label;
            single.color = roots[0].color;
            return single;
        }
        return result;
    });
}

Face import_stl(const std::string& file_path) {
    if (!std::filesystem::exists(file_path)) throw std::runtime_error("File not found: " + file_path);
    return occt("import_stl", [&] {
        Handle(Poly_Triangulation) mesh = RWStl::ReadFile(file_path.c_str());
        if (mesh.IsNull()) throw std::runtime_error("Failed to read " + file_path);
        TopoDS_Face face;
        BRep_Builder builder;
        builder.MakeFace(face, mesh);
        return Face(face);
    });
}

}  // namespace build123d
