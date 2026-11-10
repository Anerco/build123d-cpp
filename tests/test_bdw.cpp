// Unit tests for the C++-only parts of the bd_warehouse port: the Dict and Param types, the
// option structs (each equal to the positional defaults), copies of parts with their joints, the
// builder-mode emulation, and the bits of Python the port reproduces (csv module, eval of safe
// numbers, str/repr/round of floats). The models themselves are compared with Python in
// tests/compare/cases_bdw_*.
#include <build123d/build123d.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "../src/bd_warehouse/builder.hpp"
#include "../src/bd_warehouse/data.hpp"
#include "bd_warehouse/bd_warehouse.hpp"
#include "test_framework.hpp"

using namespace build123d;
using namespace bd_warehouse;
namespace bdw = bd_warehouse::detail;

namespace {

bool same_shape(const Shape& a, const Shape& b) {
    return std::abs(a.volume() - b.volume()) < 1e-9 * std::max(1.0, std::abs(a.volume())) &&
           std::abs(a.area() - b.area()) < 1e-9 * std::max(1.0, std::abs(a.area())) &&
           a.faces().size() == b.faces().size();
}

template <class F>
std::string message_of(F call) {
    try {
        call();
    } catch (const std::exception& e) {
        return e.what();
    }
    return "";
}

}  // namespace

// ---------------------------------------------------------------------------
// Dict, Param
// ---------------------------------------------------------------------------

TEST(bdw_dict_is_insertion_ordered_with_python_errors) {
    Dict<std::string, int> d = {{"b", 1}, {"a", 2}, {"b", 3}};  // a repeated key keeps its place
    CHECK((d.keys() == std::vector<std::string>{"b", "a"}));
    CHECK((d.values() == std::vector<int>{3, 2}));
    d["c"] = 4;
    CHECK(d.size() == 3 && d.contains("c") && !d.contains("z"));
    CHECK(d.at("a") == 2);
    CHECK(!d.get("z").has_value());
    CHECK(*d.get("c") == 4);
    CHECK_THROWS(d.at("z"), std::out_of_range);
    CHECK(message_of([&] { (void)d.at("z"); }) == "'z'");  // Python's KeyError: the key's repr
    CHECK(d.pop("b") == 3);
    CHECK((d.keys() == std::vector<std::string>{"a", "c"}));
    CHECK(d.at("c") == 4);  // positions after the popped key still resolve
    const Dict<std::string, int> same = {{"c", 4}, {"a", 2}};
    CHECK(d == same);  // order doesn't matter for equality, as in Python
    const Dict<int, std::string> by_int = {{3, "x"}};
    CHECK(message_of([&] { (void)by_int.at(7); }) == "7");
}

TEST(bdw_param_holds_numbers_or_text) {
    const Param number = 2.5;
    const Param integer = 4;
    const Param text = "5/8";
    CHECK(number.is_number() && number.number() == 2.5);
    CHECK(integer.is_number() && static_cast<double>(integer) == 4.0);
    CHECK(!text.is_number() && text.text() == "5/8");
    CHECK_THROWS(text.number(), std::invalid_argument);  // Python: TypeError calculating with str
    CHECK_THROWS(number.text(), std::invalid_argument);
    CHECK(Param(1.0) == Param(1));
    CHECK(Param("1") != Param(1));
}

// ---------------------------------------------------------------------------
// The Python the port reproduces
// ---------------------------------------------------------------------------

TEST(bdw_csv_reader_is_the_excel_dialect) {
    const auto rows = bdw::csv_reader("a,\"b,c\",d\n\n\"x \"\"y\"\"\",z\n");
    CHECK(rows.size() == 3);
    CHECK((rows[0] == std::vector<std::string>{"a", "b,c", "d"}));
    CHECK(rows[1].empty());  // an empty line is an empty row
    CHECK((rows[2] == std::vector<std::string>{"x \"y\"", "z"}));

    const bdw::DictReader reader = bdw::dict_reader("a,b,c\n1,2\n\n1,2,3,4\n");
    CHECK((*reader.fieldnames == std::vector<std::string>{"a", "b", "c"}));
    CHECK(reader.rows.size() == 2);  // the empty line is skipped
    CHECK(reader.rows[0].str("b") == "2");
    CHECK(!reader.rows[0].at("c").has_value());  // short row: None
    CHECK(!reader.rows[0].rest.has_value());
    CHECK(*reader.rows[1].at("c") == "3");
    CHECK((*reader.rows[1].rest == std::vector<std::string>{"4"}));  // long row: restkey None
    CHECK_THROWS(reader.rows[0].at("d"), std::out_of_range);
    CHECK_THROWS(reader.rows[0].str("c"), std::invalid_argument);
    CHECK(!bdw::dict_reader("").fieldnames.has_value());
}

TEST(bdw_eval_of_safe_numbers_as_python) {
    const bdw::PyNumber fraction = bdw::py_eval_number("5/16");
    CHECK(fraction.value == 0.3125 && !fraction.is_int);
    const bdw::PyNumber whole = bdw::py_eval_number("3");
    CHECK(whole.value == 3 && whole.is_int);
    CHECK(bdw::py_eval_number("1+1/4").value == 1.25);  // imperial_str_to_float's "1 1/4"
    CHECK(!bdw::py_eval_number("10/2").is_int);         // true division gives a float
    CHECK(bdw::py_eval_number("2.5").value == 2.5);
    CHECK_THROWS(bdw::py_eval_number("1/0"), std::domain_error);       // ZeroDivisionError
    CHECK_THROWS(bdw::py_eval_number("1 1/4"), std::invalid_argument);  // SyntaxError
}

TEST(bdw_str_and_repr_as_python) {
    CHECK(bdw::py_str(1.0) == "1.0");
    CHECK(bdw::py_str(0.1) == "0.1");
    CHECK(bdw::py_str(1e16) == "1e+16");
    CHECK(bdw::py_str(1e-5) == "1e-05");
    CHECK(bdw::py_str(123456789.0) == "123456789.0");
    CHECK(bdw::py_str(1.0 / 3) == "0.3333333333333333");
    CHECK(bdw::py_str(-0.0) == "-0.0");
    CHECK(bdw::py_str(std::numeric_limits<double>::infinity()) == "inf");
    CHECK(bdw::py_str(std::nan("")) == "nan");
    CHECK(bdw::py_str(2.5e-7) == "2.5e-07");
    CHECK(bdw::py_str(1234567890123456.0) == "1234567890123456.0");
    CHECK(bdw::py_str(7) == "7");
    CHECK(bdw::py_repr("it's") == "\"it's\"");
    CHECK(bdw::py_repr("a\"b") == "'a\"b'");
    CHECK(bdw::py_repr("a\\b") == "'a\\\\b'");
    CHECK(bdw::py_repr("tab\there") == "'tab\\there'");
    CHECK(bdw::py_list_repr(std::vector<std::string>{"M6-1", "M8-1.25"}) == "['M6-1', 'M8-1.25']");
    CHECK(bdw::py_tuple_repr(std::vector<std::string>{"6mm", "1/4in"}) == "('6mm', '1/4in')");
    CHECK(bdw::py_tuple_repr(std::vector<double>{20.741, 20.955}) == "(20.741, 20.955)");
}

TEST(bdw_numbers_as_python) {
    CHECK(bdw::py_round(2.675, 2) == 2.67);  // the double below 2.675
    CHECK(bdw::py_round(0.125, 2) == 0.12);  // ties to even
    CHECK(bdw::py_round(0.375, 2) == 0.38);
    CHECK(bdw::py_round(2.5, 0) == 2.0);
    CHECK(bdw::py_round(3.5, 0) == 4.0);
    CHECK(bdw::radians(60) == 60 * (3.141592653589793 / 180.0));
    CHECK(bdw::py_int_str(" 7 ") == 7);
    CHECK(bdw::py_float("1e3") == 1000.0);
    CHECK(message_of([] { bdw::py_int_str("1.5"); }) == "invalid literal for int() with base 10: '1.5'");
    CHECK(message_of([] { bdw::py_float("abc"); }) == "could not convert string to float: 'abc'");
    CHECK((bdw::py_split("a,b,,c", ",") == std::vector<std::string>{"a", "b", "", "c"}));
    CHECK((bdw::py_split(" a  b ") == std::vector<std::string>{"a", "b"}));
}

TEST(bdw_data_files_are_embedded) {
    CHECK(!bdw::data_file("data/din471.csv").empty());
    CHECK(!bdw::data_file("pipe.csv").empty());
    CHECK_THROWS(bdw::data_file("data/missing.csv"), std::runtime_error);  // FileNotFoundError
    CHECK(read_fastener_parameters_from_csv("clearance_hole_sizes.csv").contains("M5"));
    CHECK(drilling_class_150().at("1/2").at(6).text() == "\xe2\x80\xa6");  // the tables' "…"
    CHECK(bdw::class_methods("HexNut") != nullptr);
    CHECK(bdw::class_methods("Box") == nullptr);
}

// ---------------------------------------------------------------------------
// Builder-mode emulation
// ---------------------------------------------------------------------------

TEST(bdw_builder_combines_like_build123d) {
    bdw::Builder part(3);
    part.add(Box(10, 10, 10));
    part.add(bdw::place(Box(2, 2, 20), {Location(Vector(3, 3, 0)), Location(Vector(-3, -3, 0))}),
             bdw::Mode::SUBTRACT);
    CHECK(part.solids().size() == 1);
    CHECK(std::abs(part.part().volume() - (1000 - 2 * 2 * 2 * 10)) < 1e-6);
    // the operation's record, as build123d's: the holes' faces come from the boxes brought in
    // (LAST), their edges partly from the cut (NEW)
    CHECK(part.faces(Select::NEW).empty() && part.faces(Select::LAST).size() == 8);
    CHECK(part.edges(Select::NEW).size() == 16 && part.edges(Select::LAST).size() == 24);

    // a sketch published on a workplane gives the part pending faces, extruded along its normal
    bdw::Builder sketch(2);
    sketch.add(Rectangle(4, 4));
    part.publish(sketch.product(), bdw::Mode::ADD, {Location()}, {Location(Plane::XY.offset(5))});
    CHECK(part.pending_faces.size() == 1);
    bdw::extrude(part, 3);
    CHECK(part.pending_faces.empty());
    CHECK(std::abs(part.part().bounding_box().max.Z - 8) < 1e-9);

    // nested Locations: each outer location times each inner one
    const auto composed = bdw::compose({Location(Vector(10, 0, 0)), Location(Vector(20, 0, 0))},
                                          {Location(Vector(0, 1, 0)), Location(Vector(0, 2, 0))});
    CHECK(composed.size() == 4);
    CHECK(composed[1].position() == Vector(10, 2, 0));
    CHECK(composed[2].position() == Vector(20, 1, 0));

    // Hole without a depth goes through the part; with one it is centred on the location
    bdw::Builder plate(3);
    plate.add(Box(20, 20, 4));
    CHECK(std::abs(bdw::max_dimension(plate) - Vector(20, 20, 4).length()) < 1e-6);
    bdw::hole(plate, 1, std::nullopt, {Location(Vector(5, 5, 2))});
    bdw::hole(plate, 1, 1.0, {Location(Vector(-5, -5, 2))});
    CHECK(std::abs(plate.part().volume() - (1600 - M_PI * 4 - M_PI)) < 1e-6);
    CHECK_THROWS(bdw::through_hole_depth(bdw::Builder(3), {Location()}), std::invalid_argument);
}

// ---------------------------------------------------------------------------
// Option structs: the same defaults as the positional parameters
// ---------------------------------------------------------------------------

TEST(bdw_option_structs_match_the_positional_defaults) {
    CHECK(same_shape(ORing("025"), ORing("025", ORingOptions{})));
    CHECK(same_shape(ShaftKey(12, 20), ShaftKey(12, 20, ShaftKeyOptions{})));
    const ShaftKey key(12, 20);
    CHECK(same_shape(Keyway(key), Keyway(key, KeywayOptions{})));
    CHECK(same_shape(SpurGear(2, 12, 14.5, 5), SpurGear(2, 12, 14.5, 5, SpurGearOptions{})));
    CHECK(same_shape(RackGear(2, 6, 20, 4), RackGear(2, 6, 20, 4, RackGearOptions{})));
    CHECK(same_shape(HelicalGear(2, 13, 20, 30, 6), HelicalGear(2, 13, 20, 30, 6, HelicalGearOptions{})));
    CHECK(same_shape(Sprocket(16), Sprocket(16, SprocketOptions{})));
    const SprocketOptions sprocket;
    CHECK(sprocket.chain_pitch == 0.5 * IN && sprocket.roller_diameter == (5.0 / 16) * IN &&
          sprocket.thickness == 0.084 * IN);
    const CBeamAssemblyOptions assembly;
    CHECK(!assembly.gantry_position && !assembly.screw_length && assembly.end_plate && assembly.simple);
    const WormOptions worm;
    CHECK(worm.pressure_angle == 20 && worm.flank_form == "ZI" && worm.hand == "right");
    const ScrewOptions screw;
    CHECK(!screw.fastener_type && screw.hand == "right" && screw.simple);
    const TrapezoidalThreadOptions thread;
    CHECK(thread.external && thread.starts == 1 && thread.end_finishes.first == "fade");
    // the keyword form with one argument changed equals the positional form
    CHECK(same_shape(SpurGear(2, 12, 14.5, 5, 0.5), SpurGear(2, 12, 14.5, 5, {.root_fillet = 0.5})));
}

// ---------------------------------------------------------------------------
// Parts: copies, joints, the C++ forms of Python's class-level values
// ---------------------------------------------------------------------------

TEST(bdw_copies_keep_their_attributes_and_rebind_joints) {
    const SingleRowDeepGrooveBallBearing bearing("M8-22-7");
    SingleRowDeepGrooveBallBearing copy = bearing;
    CHECK(copy.bearing_size == "M8-22-7" && copy.children.size() == bearing.children.size());
    CHECK(&copy.joints["a"].parent() == &copy);
    copy.move(Pos(0, 0, 10));
    CHECK(std::abs(copy.joints["a"].location().position().Z - (bearing.joints["a"].location().position().Z + 10)) <
          1e-9);

    AluminumSpacer spacer("6mm");
    SingleRowCappedDeepGrooveBallBearing capped("M5-10-4", "OpenBuilds");
    capped.joints["b"].connect_to(spacer.joints["a"]);  // moves the spacer onto the bearing
    CHECK((spacer.joints["a"].location().position() - capped.joints["b"].location().position()).length() < 1e-9);
}

TEST(bdw_class_names_stand_for_python_classes) {
    CHECK(method_exists("HexNut", "nut_profile"));
    CHECK(!method_exists("HexNut", "head_profile"));
    CHECK_THROWS(method_exists("Box", "volume"), std::invalid_argument);
    CHECK(SingleRowCappedDeepGrooveBallBearing("M8-16-5").method_exists("cap"));
    CHECK(!SingleRowDeepGrooveBallBearing("M8-22-7").method_exists("cage"));
    CHECK(SingleRowDeepGrooveBallBearing("M8-22-7").bearing_class() == "SingleRowDeepGrooveBallBearing");
    CHECK(ExternalSnapRing::standard == "DIN471" && InternalSnapRing::standard == "DIN472");
    CHECK(ExternalSnapRing("20").standard == "DIN471");
}

TEST(bdw_algebra_holes_need_a_depth) {
    const SocketHeadCapScrew screw("M5-0.8", 20);
    CHECK(message_of([&] { ClearanceHole hole(screw); }) == "No depth provided");
    CHECK(message_of([&] { TapHole hole(screw); }) == "No depth provided");
    CHECK(message_of([&] { ThreadedHole hole(screw); }) == "No depth provided");
    const ThreadedHole hole(screw, "Soft", "Normal", 10);
    CHECK(hole.hole_depth == 10 && !hole.thread && hole.thread_locations.empty());
    CHECK(screw.hole_locations.empty());  // only bd_warehouse's builder mode records them
}

TEST(bdw_open_builds_module_values) {
    CHECK(open_builds::CAVITY_RADIUS == 2.3 && open_builds::FILLET_RADIUS == 1.5);
    CHECK(&open_builds::m5() == &open_builds::m5());
    CHECK(open_builds::m5().length == 20);
    const SpacerBlock block;  // made with threaded holes for m5
    CHECK(open_builds::m5().hole_locations.empty());  // the shared screw is left unchanged
    CHECK(block.label == "SpacerBlock");
    const LockCollar collar("8mm");  // its own set screw sits in the hole it made
    CHECK(collar.children.size() == 2);
    CHECK((collar.children[1].location().position() - Vector(0, 7, 3.5)).length() < 1e-9);
    CHECK_THROWS(RouterSpindleMount("base"), std::runtime_error);  // as bd_warehouse 0.4.0
}

TEST(bdw_tapered_bearing_caches_its_sections) {
    const SingleRowTaperedRollerBearing bearing("M25-52-19.25");
    CHECK(bearing.taper_length > 0);
    CHECK(bearing.roller().is_same(bearing.roller()));  // made once, kept
    CHECK(bearing.roller_count() > 0 && bearing.race_center_radius() > 0);
}
