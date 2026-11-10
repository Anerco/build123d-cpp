// bd_warehouse's pipe.py, compared with cases_bdw_pipe_py.py.
#include <functional>

#include "bdw_util.hpp"

using namespace build123d;
using namespace bd_warehouse;
using namespace bdw_cases;

namespace {

using Spec = std::array<std::string, 3>;

std::vector<Spec> valid_specs() {
    std::vector<Spec> specs;
    for (const auto& nps : pipe_nps()) {
        for (const char* material : {"abs", "copper", "stainless"}) {
            for (const auto& identifier : pipe_identifiers()) {
                // (14 stainless 5S has a wall of 156 in in the table: its section fails, see below)
                if (PipeSection::pipe_data().contains(nps + material + identifier) &&
                    nps + material + identifier != "14stainless5S") {
                    specs.push_back({nps, material, identifier});
                }
            }
        }
    }
    return specs;
}

V section_values(const PipeSection& s) {
    return V::list({s.nps, s.sch, s.od, s.thickness, s.id, !s.material.get().has_value(), s.area(),
                    loc_values(s.location())});
}

V pipe_values(const Pipe& p) {
    return V::list({p.od, p.id, p.thickness, p.length, p.material.get()->material.name, p.label,
                    loc_values(p.joints.get<RigidJoint>("inlet").relative_location()),
                    loc_values(p.joints.get<RigidJoint>("outlet").relative_location())});
}

Wire bent_path() {
    return Wire(std::vector<Edge>{Line(Vector(0, 0, 0), Vector(0, 0, 50)), JernArc(Vector(0, 0, 50), Vector(0, 0, 1), 40, 90),
                                  Line(Vector(40, 0, 90), Vector(120, 0, 90))});
}

/// The paths: an Edge or a Wire.
struct Path {
    std::string name;
    std::function<Pipe(const Spec&)> make;
};

template <class P>
std::function<Pipe(const Spec&)> pipe_along(std::function<P()> path) {
    return [path](const Spec& s) { return Pipe(s[0], s[1], s[2], path()); };
}

std::string key_of(const Spec& s) {
    std::string out;
    for (const auto& x : s) {
        std::string part = x;
        for (auto& c : part) {
            if (c == ' ' || c == '/') c = '_';
        }
        out += (out.empty() ? "" : "_") + part;
    }
    return out;
}

}  // namespace

COMPARE_CASES {
    using compare::add;
    using compare::add_value;

    add_value("bdw_pipe_values", [] {
        return V::list({V::list_of(pipe_nps()), V::list_of(pipe_identifiers()), V::list_of(pipe_materials())});
    });
    add_value("bdw_pipe_data", [] {
        std::vector<V> out;
        for (const auto& [k, v] : PipeSection::pipe_data()) out.push_back(V::list({k, V::list({v.first, v.second})}));
        return V::list(out);
    });
    add_value("bdw_pipe_all_sections", [] {
        std::vector<V> out;
        for (const auto& s : valid_specs()) out.push_back(section_values(PipeSection(s[0], s[1], s[2])));
        return V::list(out);
    });
    add_value("bdw_pipe_section_errors", [] {
        return V::list({error_of([] { PipeSection("7", "copper", "K"); }), error_of([] { PipeSection("1", "copper", "Z"); }),
                        error_of([] { PipeSection("1", "aluminum", "K"); }),
                        error_of([] { PipeSection("1", "copper", "40"); }), error_of([] { PipeSection("1", "iron", "STD"); }),
                        error_of([] { PipeSection("1", "pvc", "40"); }), error_of([] { PipeSection("1", "steel", "40"); }),
                        error_of([] { PipeSection("1", "wood", "40"); }),
                        error_of([] { Pipe("2", "steel", "80", Edge::make_line(Vector(0, 0, 0), Vector(0, 0, 10))); })});
    });
    add("bdw_pipe_section_bad_table_row", [] { return PipeSection("14", "stainless", "5S"); });
    add("bdw_pipe_section_align_min", [] { return PipeSection("2", "copper", "L", Align2(Align::MIN)); });
    add("bdw_pipe_section_align_none", [] { return PipeSection("2", "copper", "L", std::nullopt); });
    add("bdw_pipe_section_align_mixed", [] { return PipeSection("2", "copper", "L", Align2(Align::MAX, Align::MIN)); });
    add("bdw_pipe_rotated", [] {
        return Pipe("1", "copper", "K", Edge::make_line(Vector(0, 0, 0), Vector(0, 0, 100)), Rotation(90, 0, 0));
    });
    add("bdw_pipe_aligned", [] { return Pipe("1", "copper", "K", bent_path(), Rotation(), Align3(Align::MIN)); });
    add_value("bdw_pipe_aligned_values", [] {
        return pipe_values(Pipe("1", "copper", "K", bent_path(), Rotation(0, 0, 45),
                                Align3(Align::CENTER, Align::CENTER, Align::MIN)));
    });

    for (const Spec& s : std::vector<Spec>{{"1/8", "stainless", "10S"},
                                           {"1", "copper", "K"},
                                           {"4", "abs", "40"},
                                           {"24", "stainless", "10S"},
                                           {"1 1/2", "copper", "M"}}) {
        add("bdw_pipe_section_" + key_of(s), [s] { return PipeSection(s[0], s[1], s[2]); });
    }

    const std::vector<Path> paths = {
        {"line", pipe_along<Edge>([] { return Edge::make_line(Vector(0, 0, 0), Vector(0, 0, 100)); })},
        {"slanted", pipe_along<Edge>([] { return Edge(Line(Vector(10, 0, 0), Vector(60, 30, 40))); })},
        {"arc", pipe_along<Edge>([] { return Edge(JernArc(Vector(0, 0, 0), Vector(1, 0, 0), 60, 120)); })},
        {"bent", pipe_along<Wire>(bent_path)},
        {"polyline", pipe_along<Wire>([] {
             return Wire(Polyline({Vector(0, 0, 0), Vector(100, 0, 0), Vector(100, 80, 0)}));
         })},
    };
    for (const auto& path : paths) {
        for (const Spec& s : std::vector<Spec>{{"1", "copper", "K"}, {"3", "stainless", "40S"}, {"2", "abs", "40"}}) {
            const std::string key = path.name + "_" + s[0] + "_" + s[1] + "_" + s[2];
            const auto make = path.make;
            add("bdw_pipe_" + key, [make, s] { return make(s); });
            add_value("bdw_pipe_" + key + "_values", [make, s] { return pipe_values(make(s)); });
        }
    }
}
