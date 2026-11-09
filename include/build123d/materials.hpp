// build123d-cpp: the material of a shape: the bd_materials package that build123d uses
// (Shape.material, mass(), compute_mass(), the looks export_gltf writes).
//
// The names of bd_materials' top level and of its core module are in build123d; its other
// modules are namespaces: finishes, processes, registry, applicability, and the catalog
// modules metals, plastics, resins, glass, wood, paper and textile (material_catalog.hpp).
//
//   FinishedMaterial a = metals::aluminum(metals::Alu::G7075_T6, {.finish = finishes::anodize("red")});
//   FinishedMaterial b = plastics::petg({.color = "#336699", .process = processes::fdm(0.1)});
//   FinishedMaterial c = "oak";                       // resolve_material("oak")
//   PbrProperties look = a.pbr();                     // its physically based look
#pragma once

#include <initializer_list>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "build123d/geometry.hpp"
#include "build123d/pbr.hpp"

namespace build123d {

// ---------------------------------------------------------------------------
// bd_materials.finishes
// ---------------------------------------------------------------------------

namespace finishes {

/// Sheen: how glossy a coat is.
enum class Sheen { GLOSS = 1, MATTE = 2 };

/// Finish: a surface treatment, compared by value (name and notes), as the dataclass.
struct Finish {
    std::string name;
    std::optional<std::string> notes{};

    /// repr(): "Finish(name='Brushed', notes=None)"
    std::string repr() const;
    bool operator==(const Finish& other) const { return name == other.name && notes == other.notes; }
    bool operator!=(const Finish& other) const { return !(*this == other); }
};

enum class Mechanical { BEAD_BLAST = 1, BRUSHED, FINE_SANDING, SMOOTH_MACHINING, ELECTROPOLISHED };
enum class Chemical { ANODIZED = 1, CHEM_FILM, CONDUCTIVE_OXIDATION, BLACK_OXIDE, PASSIVATION, PICKLING, DYEING };
enum class MetalPlating {
    CHROME_PLATING = 1,
    GOLD_PLATING,
    NICKEL_PLATING,
    SILVER_PLATING,
    TIN_PLATING,
    PVD,
    ZINC_PLATING,
    VACUUM_PLATING
};
enum class Coating { POWDER_COAT = 1, ELECTROPHORESIS, SPRAY_PAINT };
enum class Marking { LASER_ENGRAVING = 1, ETCHING, SILKSCREEN };

extern const std::map<Mechanical, Finish> MECHANICAL_FINISHES;
extern const std::map<Chemical, Finish> CHEMICAL_FINISHES;
extern const std::map<MetalPlating, Finish> METAL_PLATING_FINISHES;
extern const std::map<Coating, Finish> COATING_FINISHES;
extern const std::map<Marking, Finish> MARKING_FINISHES;
/// Every finish, table by table.
extern const std::vector<Finish> ALL_FINISHES;

/// AppliedFinish: a finish with its per-part choices.
struct AppliedFinish {
    Finish finish;
    std::optional<ColorSpec> color{};
    std::optional<Sheen> sheen{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;

    /// repr(): "AppliedFinish(finish=Finish(...), color=None, sheen=None, scale=(1.0, 1.0), rotation=0.0)"
    std::string repr() const;
    bool operator==(const AppliedFinish& other) const;
    bool operator!=(const AppliedFinish& other) const { return !(*this == other); }
};

// Mechanical finishes. `scale` sizes the texture (u, v) and `rotation` turns it (degrees).
AppliedFinish bead_blast(std::pair<double, double> scale = {1.0, 1.0});
AppliedFinish brushed(std::pair<double, double> scale = {1.0, 1.0}, double rotation = 0.0);
AppliedFinish fine_sanding(std::pair<double, double> scale = {1.0, 1.0});
AppliedFinish smooth_machining();
AppliedFinish electropolish();
// Chemical finishes.
AppliedFinish anodize(std::optional<ColorSpec> color);
AppliedFinish chem_film();
AppliedFinish conductive_oxidation();
AppliedFinish black_oxide();
AppliedFinish passivate();
AppliedFinish pickle();
AppliedFinish dye(std::optional<ColorSpec> color);
// Metal plating.
AppliedFinish chrome();
AppliedFinish gold_plate();
AppliedFinish nickel_plate();
AppliedFinish silver_plate();
AppliedFinish tin_plate();
AppliedFinish pvd(std::optional<ColorSpec> color = ColorSpec("clear"));
AppliedFinish zinc_plate(std::optional<ColorSpec> color = ColorSpec("clear"));
AppliedFinish vacuum_plating(std::optional<ColorSpec> color, std::optional<Sheen> sheen = Sheen::GLOSS);
// Coatings.
AppliedFinish powder_coat(std::optional<ColorSpec> color, std::optional<Sheen> sheen = Sheen::GLOSS);
AppliedFinish spray_paint(std::optional<ColorSpec> color, std::optional<Sheen> sheen = Sheen::GLOSS);
AppliedFinish electrophoresis(std::optional<ColorSpec> color = ColorSpec("black"));
// Markings.
AppliedFinish laser_engrave();
AppliedFinish etch();
AppliedFinish silkscreen(std::optional<ColorSpec> color = ColorSpec("black"));

}  // namespace finishes

// ---------------------------------------------------------------------------
// bd_materials.processes
// ---------------------------------------------------------------------------

namespace processes {

/// The layer height and line width the FDM textures were drawn for, and the skin's rotation.
inline constexpr double AUTHORED_LAYER_HEIGHT_MM = 0.2;
inline constexpr double AUTHORED_LINE_WIDTH_MM = 0.4;
inline constexpr double AUTHORED_SKIN_ROTATION = 45.0;

/// Process: how a part is made (its raw surface).
enum class Process { FDM, SLS, MJF, SLM, VAT, MOLDED, MACHINED, CAST, WROUGHT };
/// Process.value: "fdm", "sls", ...
std::string to_string(Process process);

/// AppliedProcess: a process with the per-part geometry of its surface.
struct AppliedProcess {
    Process process{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;
    /// The FDM layer height or line width (mm).
    std::optional<double> pitch_mm{};
    /// FDM top/bottom skin instead of the layer lines.
    bool skin = false;
    double mm_per_uv = 1.0;

    /// repr(): "AppliedProcess(process=<Process.FDM: 'fdm'>, scale=(1.0, 1.0), rotation=0.0, ...)"
    std::string repr() const;
    bool operator==(const AppliedProcess& other) const;
    bool operator!=(const AppliedProcess& other) const { return !(*this == other); }
};

/// FDM layer lines: `layer_height_mm` apart, turned by `rotation` degrees; `mm_per_uv` is the
/// model's millimetres per UV unit.
AppliedProcess fdm(double layer_height_mm = AUTHORED_LAYER_HEIGHT_MM, double rotation = 0.0, double mm_per_uv = 1.0);
/// The FDM top/bottom skin: lines `line_width_mm` wide at `rotation` degrees.
AppliedProcess fdm_skin(double line_width_mm = AUTHORED_LINE_WIDTH_MM, double rotation = AUTHORED_SKIN_ROTATION);
AppliedProcess sls(std::pair<double, double> scale = {1.0, 1.0});
AppliedProcess mjf(std::pair<double, double> scale = {1.0, 1.0});
AppliedProcess slm(std::pair<double, double> scale = {1.0, 1.0});
AppliedProcess vat();
AppliedProcess molded();
AppliedProcess machined();
AppliedProcess cast();
AppliedProcess wrought();

}  // namespace processes

/// bd_materials' FinishSpec: one finish or a list of them (a later finish of the same kind
/// replaces an earlier one).
class FinishSpec {
public:
    FinishSpec(finishes::AppliedFinish finish) : items_{std::move(finish)} {}  // NOLINT(google-explicit-constructor)
    FinishSpec(std::vector<finishes::AppliedFinish> finishes) : items_(std::move(finishes)), list_(true) {}  // NOLINT
    FinishSpec(std::initializer_list<finishes::AppliedFinish> finishes) : items_(finishes), list_(true) {}

    /// The finishes (one for a single finish).
    const std::vector<finishes::AppliedFinish>& items() const { return items_; }
    /// Whether it's a list (repr() shows it as one).
    bool is_list() const { return list_; }
    std::string repr() const;

private:
    std::vector<finishes::AppliedFinish> items_;
    bool list_ = false;
};

/// bd_materials' ProcessSpec: an AppliedProcess, or a Process (deprecated: FinishedMaterial
/// warns and applies it as AppliedProcess{process}).
using ProcessSpec = std::variant<processes::AppliedProcess, processes::Process>;

// ---------------------------------------------------------------------------
// bd_materials.core
// ---------------------------------------------------------------------------

/// bd_materials.core.Range: the range of a property's values. Range(v) is as_range(v), the
/// range of a single value.
struct Range {
    double min;
    double max;

    /// std::invalid_argument if min > max.
    Range(double min, double max);
    Range(double value) : Range(value, value) {}  // NOLINT(google-explicit-constructor)
    /// The value a fraction `r` of the way from min to max.
    double value_at(double r) const { return min + r * (max - min); }
    bool operator==(const Range& other) const { return min == other.min && max == other.max; }
    bool operator!=(const Range& other) const { return !(*this == other); }
};

/// A property for which the material isn't suitable (NaN to NaN).
extern const Range NOT_SUITABLE;
/// The units of the properties, in bd_materials' order.
extern const std::vector<std::pair<std::string, std::string>> MECHANICAL_PROPERTY_UNITS;
extern const std::vector<std::pair<std::string, std::string>> THERMAL_PROPERTY_UNITS;
extern const std::vector<std::pair<std::string, std::string>> PROPERTY_UNITS;
/// The material categories.
extern const std::set<std::string> ALLOWED_CATEGORIES;
/// Groups of substrates (categories and families) for the finishes.
extern const std::set<std::string> FERROUS;
extern const std::set<std::string> METAL;
extern const std::set<std::string> PAINTABLE;
extern const std::set<std::string> COATABLE_POLY;
extern const std::set<std::string> EVERYTHING;

/// bd_materials' RangeMaterial and its subclasses: a material's physical data. `kind` names the
/// bd_materials class (RangeMaterial, SolidMaterial, PolymerMaterial, ArealMaterial,
/// MetalMaterial, PlasticMaterial, ResinMaterial, GlassMaterial, WoodMaterial, PaperMaterial,
/// TextileMaterial); fields() lists the fields it has. A property the class has but the material
/// doesn't give is std::nullopt (Python's None).
struct RangeMaterial {
    /// Catalog (grade) name, e.g. "Alu_G6061_T6".
    std::string name{};
    /// Density in kg/m³.
    double density = 0.0;
    /// Material class ("metal", "plastic", ...).
    std::string category = "generic";
    /// PBR / identity key, e.g. "aluminum".
    std::optional<std::string> family{};
    /// See-through material (needs a part thickness to render).
    bool transparent = false;
    /// The bd_materials class.
    std::string kind = "RangeMaterial";
    // SolidMaterial
    std::optional<Range> tensile_strength{};
    std::optional<Range> modulus_of_elasticity{};
    std::optional<Range> shear_modulus{};
    std::optional<Range> poisson_ratio{};
    std::optional<Range> specific_heat_capacity{};
    std::optional<Range> max_service_temp{};
    std::optional<Range> thermal_expansion{};
    std::optional<Range> thermal_conductivity{};
    // PolymerMaterial, MetalMaterial, GlassMaterial
    std::optional<Range> yield_strength{};
    std::optional<Range> shear_strength{};
    std::optional<Range> elongation_at_break{};
    std::optional<Range> glass_transition_temperature{};
    std::optional<Range> heat_deflection_temperature{};
    std::optional<Range> hardness{};
    std::string hardness_scale{};
    std::optional<Range> melting_temperature{};
    // WoodMaterial
    std::optional<Range> modulus_of_rupture{};
    std::optional<Range> compressive_strength_parallel{};
    std::optional<Range> janka_hardness{};
    // ArealMaterial
    std::optional<Range> areal_density{};
    std::optional<Range> thickness{};

    /// Part mass in grams from a volume in mm³ (m = density * V).
    double mass(double volume_mm3) const { return density * volume_mm3 * 1e-6; }
    /// The fields of the class, in its order (std::invalid_argument for an unknown kind).
    std::vector<std::string> fields() const;
    /// A Range property by its field name (std::nullopt for None and for fields the class
    /// doesn't have).
    std::optional<Range> property(const std::string& field) const;
    /// str(): the name and the mechanical and thermal properties with their units.
    std::string str() const;
};

/// with_density: the material with another density (none: the same).
RangeMaterial with_density(const RangeMaterial& material, std::optional<double> density);

// ---------------------------------------------------------------------------
// bd_materials.FinishedMaterial
// ---------------------------------------------------------------------------

/// The keyword arguments of FinishedMaterial, in Python's order. `pbr` replaces the whole look;
/// materials given the same pointer share it (export_gltf writes one glTF material for it, as
/// Python does for one PbrProperties object).
struct FinishedMaterialOptions {
    std::optional<ColorSpec> color{};
    std::optional<double> thickness_mm{};
    std::optional<double> opacity{};
    std::optional<double> roughness{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;
    std::optional<ProcessSpec> process{};
    std::shared_ptr<const PbrProperties> pbr{};
};

/// bd_materials' FinishedMaterial: a RangeMaterial plus the per-part choices (finish, color,
/// process, ...). Shape::material holds one; mass() reads `material.density`; pbr() is its look.
struct FinishedMaterial {
    RangeMaterial material{};
    std::optional<FinishSpec> finish{};
    /// A base color (paint, dye, filament), if any.
    std::optional<ColorSpec> color{};
    /// The part's thickness for transparent materials (mm).
    std::optional<double> thickness_mm{};
    std::optional<double> opacity{};
    std::optional<double> roughness{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;
    std::optional<processes::AppliedProcess> process{};

    FinishedMaterial() = default;
    /// std::invalid_argument if `pbr` comes with other choices, or `finish` with `process`.
    explicit FinishedMaterial(RangeMaterial material, std::optional<FinishSpec> finish = std::nullopt,
                              const FinishedMaterialOptions& options = {});
    FinishedMaterial(RangeMaterial material, const FinishedMaterialOptions& options)
        : FinishedMaterial(std::move(material), std::nullopt, options) {}
    /// A catalog material by name, like build123d's `shape.material = "aluminum"`.
    FinishedMaterial(const std::string& name);  // NOLINT(google-explicit-constructor)
    FinishedMaterial(const char* name) : FinishedMaterial(std::string(name)) {}  // NOLINT

    /// The material's catalog name.
    const std::string& name() const { return material.name; }
    /// The look: the pbr override, or the one bd_materials computes from the material and the
    /// choices (get_pbr_properties), anew at each call as Python's property.
    PbrProperties pbr() const;
    /// pbr() as Python's object: the override itself (shared by the copies of this material)
    /// or a new computed look.
    std::shared_ptr<const PbrProperties> pbr_ptr() const;
    /// The pbr override (nullptr: none).
    const std::shared_ptr<const PbrProperties>& pbr_override() const { return pbr_; }
    /// Python's `material.pbr = value`; nullptr removes the override.
    void set_pbr(std::shared_ptr<const PbrProperties> value) { pbr_ = std::move(value); }
    void set_pbr(const PbrProperties& value) { pbr_ = std::make_shared<const PbrProperties>(value); }
    /// str(): the material's property table.
    std::string str() const { return material.str(); }
    /// repr(): "FinishedMaterial(Alu_G6061_T6, finish=AppliedFinish(...))"
    std::string repr() const;

private:
    std::shared_ptr<const PbrProperties> pbr_;
};

// The keyword arguments of the family functions (material_catalog.hpp), in Python's order:

/// metals.<family>(grade, finish, process, density)
struct MetalOptions {
    std::optional<FinishSpec> finish{};
    std::optional<ProcessSpec> process{};
    std::optional<double> density{};
};
/// plastics.<family>(grade, color, finish, process, density), and resins
struct PolymerOptions {
    std::optional<ColorSpec> color{};
    std::optional<FinishSpec> finish{};
    std::optional<ProcessSpec> process{};
    std::optional<double> density{};
};
/// glass.<family>, plastics.pc/pmma and resins.transparent: see-through materials
struct TransparentOptions {
    std::optional<ColorSpec> color{};
    std::optional<double> thickness_mm{};
    std::optional<double> opacity{};
    std::optional<double> roughness{};
    std::optional<FinishSpec> finish{};
    std::optional<ProcessSpec> process{};
    std::optional<double> density{};
};
/// paper.paper and paper.foamboard (white unless told otherwise)
struct PaperOptions {
    std::optional<ColorSpec> color = ColorSpec("white");
    std::optional<FinishSpec> finish{};
    std::optional<ProcessSpec> process{};
    std::optional<double> density{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;
};
/// wood.<family> and paper.cardboard
struct TexturedOptions {
    std::optional<FinishSpec> finish{};
    std::optional<ProcessSpec> process{};
    std::optional<double> density{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;
};
/// textile.<family>
struct TextileOptions {
    std::optional<ColorSpec> color{};
    std::optional<FinishSpec> finish{};
    std::optional<ProcessSpec> process{};
    std::optional<double> density{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;
};

// ---------------------------------------------------------------------------
// bd_materials.pbr
// ---------------------------------------------------------------------------

/// The keyword arguments of get_pbr_properties, in Python's order.
struct GetPbrPropertiesOptions {
    std::optional<ColorSpec> color{};
    std::optional<double> thickness_mm{};
    std::optional<double> opacity{};
    std::optional<double> roughness{};
    std::pair<double, double> scale{1.0, 1.0};
    double rotation = 0.0;
    std::shared_ptr<const PbrProperties> pbr{};
};

/// bd_materials.pbr.get_pbr_properties: the look of a material with its finish(es), process
/// and choices, from threejs_materials' presets (pbr_preset). Looks with textures need them
/// at run time (see pbr.hpp).
PbrProperties get_pbr_properties(const RangeMaterial& material, const std::optional<FinishSpec>& finish = std::nullopt,
                                 const std::optional<processes::AppliedProcess>& process = std::nullopt,
                                 const GetPbrPropertiesOptions& options = {});

// ---------------------------------------------------------------------------
// bd_materials.registry
// ---------------------------------------------------------------------------

namespace registry {

/// The keyword arguments a factory takes. Each family function takes only some of them
/// (std::invalid_argument for the others, as Python's TypeError); unset ones keep the
/// function's defaults.
struct MaterialArgs {
    std::optional<FinishSpec> finish{};
    std::optional<ColorSpec> color{};
    std::optional<double> thickness_mm{};
    std::optional<double> opacity{};
    std::optional<double> roughness{};
    std::optional<std::pair<double, double>> scale{};
    std::optional<double> rotation{};
    std::optional<ProcessSpec> process{};
    std::optional<double> density{};
};

/// A catalog name's factory: its family function, with the grade given for a grade name.
class Factory {
public:
    using Function = FinishedMaterial (*)(int grade, const MaterialArgs& args);
    Factory(Function function, int grade) : function_(function), grade_(grade) {}
    FinishedMaterial operator()(const MaterialArgs& args = {}) const { return function_(grade_, args); }

private:
    Function function_;
    int grade_;
};

/// The factory of a catalog name (matched case-insensitively, with spaces and hyphens read as
/// underscores); std::invalid_argument naming close matches for an unknown name.
Factory factory(const std::string& name);
/// factory(name)(): the FinishedMaterial of a name; a FinishedMaterial as it is, a
/// RangeMaterial as FinishedMaterial(material).
FinishedMaterial resolve(const std::string& name);
FinishedMaterial resolve(const char* name);
FinishedMaterial resolve(const FinishedMaterial& material);
FinishedMaterial resolve(const RangeMaterial& material);
/// Every name resolve() accepts, sorted.
std::vector<std::string> material_names();
/// The catalog name of a material, std::invalid_argument for a material of your own.
std::string canonical_name(const FinishedMaterial& material);
std::string canonical_name(const RangeMaterial& material);

}  // namespace registry

/// bd_materials.resolve: the FinishedMaterial for a catalog name ("aluminum",
/// "Alu_G7075_T6", ...), matched case-insensitively with spaces and hyphens read as
/// underscores. std::invalid_argument if the name is unknown.
FinishedMaterial resolve_material(const std::string& name);
/// bd_materials.material_names: every name resolve_material() accepts, sorted.
std::vector<std::string> material_names();

// ---------------------------------------------------------------------------
// bd_materials.applicability
// ---------------------------------------------------------------------------

namespace applicability {

/// A finish's key in TYPICAL_SUBSTRATES.
using FinishKey =
    std::variant<finishes::Mechanical, finishes::Chemical, finishes::MetalPlating, finishes::Coating, finishes::Marking>;
/// The categories and families each finish typically goes on, in bd_materials' order.
extern const std::vector<std::pair<FinishKey, std::set<std::string>>> TYPICAL_SUBSTRATES;

/// The finishes that typically go on a material.
std::vector<finishes::Finish> typical_finishes(const RangeMaterial& material);
/// The catalog materials a finish typically goes on (std::out_of_range for a Finish that's in
/// none of the tables, as Python's KeyError).
std::vector<RangeMaterial> typical_materials(const FinishKey& finish);
std::vector<RangeMaterial> typical_materials(const finishes::Finish& finish);
std::vector<RangeMaterial> typical_materials(const finishes::AppliedFinish& finish);

}  // namespace applicability

using applicability::typical_finishes;
using applicability::typical_materials;

}  // namespace build123d
