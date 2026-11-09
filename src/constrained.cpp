// Lines and arcs constrained against other objects: a port of build123d's
// topology/constrained_lines.py and of Edge.make_constrained_arcs/make_constrained_lines
// (topology/one_d.py).
#include <algorithm>
#include <cmath>
#include <variant>

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepLib.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GccEnt_Position.hxx>
#include <Geom2dAPI_InterCurveCurve.hxx>
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Geom2dGcc_Circ2d2TanOn.hxx>
#include <Geom2dGcc_Circ2d2TanRad.hxx>
#include <Geom2dGcc_Circ2d3Tan.hxx>
#include <Geom2dGcc_Circ2dTanCen.hxx>
#include <Geom2dGcc_Circ2dTanOnRad.hxx>
#include <Geom2dGcc_Lin2d2Tan.hxx>
#include <Geom2dGcc_Lin2dTanObl.hxx>
#include <Geom2dGcc_QualifiedCurve.hxx>
#include <Geom2d_CartesianPoint.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAPI.hxx>
#include <Geom_Plane.hxx>
#include <IntAna2d_AnaIntersection.hxx>
#include <IntAna2d_IntPoint.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec2d.hxx>

#include "internal.hpp"
#include "py_compat.hpp"

namespace build123d {

using namespace detail;

// ---------------------------------------------------------------------------
// TangencyTarget
// ---------------------------------------------------------------------------

TangencyTarget::TangencyTarget(const Edge& edge) : kind_(Kind::EDGE), edge_(edge) {}
TangencyTarget::TangencyTarget(const Edge& edge, Tangency tangency)
    : kind_(Kind::EDGE), qualified_(true), tangency_(tangency), edge_(edge) {}
TangencyTarget::TangencyTarget(const Axis& axis) : kind_(Kind::AXIS), axis_(axis) {}
TangencyTarget::TangencyTarget(const Axis& axis, Tangency tangency)
    : kind_(Kind::AXIS), qualified_(true), tangency_(tangency), axis_(axis) {}
TangencyTarget::TangencyTarget(const Vertex& vertex) : kind_(Kind::VERTEX), vertex_(vertex) {}
TangencyTarget::TangencyTarget(const Vector& point) : kind_(Kind::POINT), point_(point) {}
TangencyTarget::TangencyTarget(double x, double y, double z) : kind_(Kind::POINT), point_(x, y, z) {}

const Edge& TangencyTarget::edge() const {
    if (!edge_) throw std::invalid_argument("The tangency target is not an Edge");
    return *edge_;
}
const Axis& TangencyTarget::axis() const {
    if (!axis_) throw std::invalid_argument("The tangency target is not an Axis");
    return *axis_;
}
const Vertex& TangencyTarget::vertex() const {
    if (!vertex_) throw std::invalid_argument("The tangency target is not a Vertex");
    return *vertex_;
}
const Vector& TangencyTarget::point() const {
    if (kind_ != Kind::POINT) throw std::invalid_argument("The tangency target is not a point");
    return point_;
}

namespace {

// A normalized solver input, Python's `tuple[Edge, Tangency] | Edge | Vector`.
struct Tangent {
    std::optional<Edge> edge;  // set for curves
    Vector point;              // for points
    bool qualified = false;    // given as a (edge, Tangency) tuple
    Tangency tangency = Tangency::UNQUALIFIED;
    bool is_point() const { return !edge.has_value(); }
};

Tangent tangent_edge(const Edge& e) { return Tangent{e, Vector(), false, Tangency::UNQUALIFIED}; }
Tangent tangent_point(const Vector& p) { return Tangent{std::nullopt, p, false, Tangency::UNQUALIFIED}; }

// ---------------------------------------------------------------------------
// Normalization utilities
// ---------------------------------------------------------------------------

const gp_Pln& pln_xy() {
    static const gp_Pln pln(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)));
    return pln;
}

const Handle(Geom_Plane)& surf_xy() {
    static const Handle(Geom_Plane) surf = new Geom_Plane(pln_xy());
    return surf;
}

/// Map parameter u into [first, first+per).
double norm_on_period(double u, double first, double period) {
    if (std::abs(u - first) < TOLERANCE) return first;
    return py_mod(u - first, period) + first;
}

/// Forward (positive) delta from u1 to u2 on a periodic domain anchored at 'first'.
double forward_delta(double u1, double u2, double first, double period) {
    const double u1n = norm_on_period(u1, first, period);
    const double u2n = norm_on_period(u2, first, period);
    double delta = u2n - u1n;
    if (delta < 0.0) delta += period;
    return delta;
}

// ---------------------------------------------------------------------------
// Core helpers
// ---------------------------------------------------------------------------

/// The GCC argument of a tangent: (q_obj, h2d, first, last, is_edge).
struct GccArg {
    std::optional<Geom2dGcc_QualifiedCurve> qcurve;  // curves
    Handle(Geom2d_CartesianPoint) point;            // points
    Handle(Geom2d_Curve) h2d;
    double first = 0.0, last = 0.0;
    bool is_edge = false;
    std::optional<Geom2dAdaptor_Curve> adapt2d;
};

/// Convert a TopoDS_Edge into 2d curve & extract properties.
GccArg edge_to_qualified_2d(const TopoDS_Edge& edge, Tangency position_constraint) {
    double f = 0.0, l = 0.0;
    // 1) Underlying curve + range (also retrieve location to be safe)
    Handle(Geom_Curve) hcurve3d = BRep_Tool::Curve(edge, f, l);
    double first, last;
    BRep_Tool::Range(edge, first, last);
    // 2) Convert to 2D on Plane.XY (Z-up frame at origin)
    Handle(Geom2d_Curve) hcurve2d = GeomAPI::To2d(hcurve3d, pln_xy());
    // 3) Wrap in an adaptor using the same parametric range
    Geom2dAdaptor_Curve adapt2d(hcurve2d, first, last);
    // 4) Create the qualified curve
    GccArg arg;
    arg.qcurve.emplace(adapt2d, static_cast<GccEnt_Position>(static_cast<int>(position_constraint)));
    arg.h2d = hcurve2d;
    arg.first = first;
    arg.last = last;
    arg.is_edge = true;
    arg.adapt2d = adapt2d;
    return arg;
}

/// Build a 3D edge on XY from a trimmed 2D circle segment [u1, u2].
TopoDS_Edge edge_from_circle(const Handle(Geom2d_Circle)& h2d_circle, double u1, double u2) {
    Handle(Geom2d_TrimmedCurve) arc2d = new Geom2d_TrimmedCurve(h2d_circle, u1, u2, true);
    TopoDS_Edge circle_edge = BRepBuilderAPI_MakeEdge(arc2d, surf_xy()).Edge();
    BRepLib::BuildCurves3d(circle_edge);
    return circle_edge;
}

/// Normalize (if periodic) then test [first, last] with tolerance.
bool param_in_trim(double u, double first, double last, const Handle(Geom2d_Curve)& h2d) {
    if (h2d.IsNull()) throw std::invalid_argument("Invalid parameters to _param_in_trim");
    u = h2d->IsPeriodic() ? norm_on_period(u, first, h2d->Period()) : u;
    return first - TOLERANCE <= u && u <= last + TOLERANCE;
}

/// Normalize input to a GCC argument: an Edge becomes a qualified curve, a point a
/// Geom2d_CartesianPoint.
GccArg as_gcc_arg(const Tangent& obj) {
    if (!obj.is_point()) {
        if (obj.edge->is_null()) throw std::invalid_argument("Can't create a qualified curve from empty edge");
        return edge_to_qualified_2d(TopoDS::Edge(obj.edge->wrapped()), obj.tangency);
    }
    GccArg arg;
    arg.point = new Geom2d_CartesianPoint(gp_Pnt2d(obj.point.X, obj.point.Y));
    return arg;
}

/// Both the forward (minor) and complementary (major) arcs between two circle parameters.
std::vector<TopoDS_Edge> two_arc_edges_from_params(const gp_Circ2d& circ, double u1, double u2) {
    Handle(Geom2d_Circle) h2d_circle = new Geom2d_Circle(circ);
    const double period = h2d_circle->Period();  // usually 2*pi

    // Minor (forward) span
    const double d = forward_delta(u1, u2, 0.0, period);  // anchor at 0 for circle convenience
    const double u1n = norm_on_period(u1, 0.0, period);
    const double u2n = norm_on_period(u2, 0.0, period);

    // Guard degeneracy
    if (d <= TOLERANCE || std::abs(period - d) <= TOLERANCE) return {};

    const TopoDS_Edge minor = edge_from_circle(h2d_circle, u1n, u1n + d);
    const TopoDS_Edge major = edge_from_circle(h2d_circle, u2n, u2n + (period - d));
    return {minor, major};
}

/// Build a finite Edge from two 2D contact points.
TopoDS_Edge edge_from_line(const gp_Pnt2d& p1, const gp_Pnt2d& p2) {
    const TopoDS_Vertex v1 = BRepBuilderAPI_MakeVertex(gp_Pnt(p1.X(), p1.Y(), 0)).Vertex();
    const TopoDS_Vertex v2 = BRepBuilderAPI_MakeVertex(gp_Pnt(p2.X(), p2.Y(), 0)).Vertex();
    BRepBuilderAPI_MakeEdge mk_edge(v1, v2);
    if (!mk_edge.IsDone()) throw std::runtime_error("Failed to build edge from line contacts");
    return mk_edge.Edge();
}

/// Build a 2D reference line from an Axis (XY plane).
gp_Lin2d gp_lin2d_from_axis(const Axis& ax) {
    const gp_Pnt2d p(ax.position.X, ax.position.Y);
    const gp_Dir2d d(ax.direction.X, ax.direction.Y);
    return gp_Lin2d(gp_Ax2d(p, d));
}

/// Adjust solution-circle parameters by adding pi if the solution circle is enclosed within
/// a tangent circular edge (only when at least one tangent input is non-circular).
std::vector<double> enclosed_circ_param_offset(const std::vector<Tangent>& tangent_tuples, const gp_Circ2d& circ,
                                               const std::vector<double>& params) {
    std::vector<bool> is_circ;
    for (const auto& t : tangent_tuples) {
        bool circular = false;
        if (!t.is_point() && !t.edge->is_null()) {
            const BRepAdaptor_Curve adapt(TopoDS::Edge(t.edge->wrapped()));
            circular = adapt.GetType() == GeomAbs_Circle;
        }
        is_circ.push_back(circular);
    }
    if (std::all_of(is_circ.begin(), is_circ.end(), [](bool c) { return c; })) return params;

    const gp_Pnt2d center_pnt = circ.Location();
    const Vector center_vrt(center_pnt.X(), center_pnt.Y(), 0);

    std::vector<double> pars = params;
    for (std::size_t i = 0; i < params.size() && i < tangent_tuples.size(); ++i) {
        const Tangent& t = tangent_tuples[i];
        if (t.is_point()) continue;
        if (is_circ[i] && (center_vrt - t.edge->arc_center()).length() < t.edge->radius()) pars[i] = params[i] + PI;
    }
    return pars;
}

double edge_length(const TopoDS_Edge& e) { return GCPnts_AbscissaPoint::Length(BRepAdaptor_Curve(e)); }

/// Keep both arcs (Sagitta::BOTH) or the shortest/longest one.
void add_arcs(std::vector<TopoDS_Edge>& solutions, const gp_Circ2d& circ, double u1, double u2, Sagitta sagitta) {
    std::vector<TopoDS_Edge> arcs = two_arc_edges_from_params(circ, u1, u2);
    if (sagitta == Sagitta::BOTH) {
        solutions.insert(solutions.end(), arcs.begin(), arcs.end());
        return;
    }
    std::stable_sort(arcs.begin(), arcs.end(),
                     [](const TopoDS_Edge& a, const TopoDS_Edge& b) { return edge_length(a) < edge_length(b); });
    if (arcs.empty()) throw std::out_of_range("list index out of range");
    solutions.push_back(sagitta == Sagitta::SHORT ? arcs.front() : arcs.back());
}

ShapeList<Edge> to_edges(const std::vector<TopoDS_Edge>& topods) {
    ShapeList<Edge> out;
    for (const auto& e : topods) out.push_back(Edge(e));
    return out;
}

// ---------------------------------------------------------------------------
// Arc solvers
// ---------------------------------------------------------------------------

/// Arcs of a given radius tangent to curve inputs or through point inputs on XY.
ShapeList<Edge> make_2tan_rad_arcs(const std::vector<Tangent>& tangent_tuples, double radius, Sagitta sagitta) {
    std::vector<GccArg> args;
    for (const auto& t : tangent_tuples) args.push_back(as_gcc_arg(t));

    std::optional<Geom2dGcc_Circ2d2TanRad> gcc_opt;
    occt("ConstrainedArcs", [&] {
        if (args[0].is_edge && args[1].is_edge) {
            gcc_opt.emplace(*args[0].qcurve, *args[1].qcurve, radius, TOLERANCE);
        } else if (args[0].is_edge) {
            gcc_opt.emplace(*args[0].qcurve, args[1].point, radius, TOLERANCE);
        } else {
            gcc_opt.emplace(args[0].point, args[1].point, radius, TOLERANCE);
        }
        return 0;
    });
    const Geom2dGcc_Circ2d2TanRad& gcc = *gcc_opt;
    if (!gcc.IsDone() || gcc.NbSolutions() == 0) throw std::runtime_error("Unable to find a tangent arc");

    auto ok = [&](std::size_t i, double u) {
        return !args[i].is_edge || param_in_trim(u, args[i].first, args[i].last, args[i].h2d);
    };

    std::vector<TopoDS_Edge> solutions;
    occt("ConstrainedArcs", [&] {
        for (int i = 1; i <= gcc.NbSolutions(); ++i) {
            const gp_Circ2d circ = gcc.ThisSolution(i);
            const gp_Pnt2d center = circ.Location();
            const double sol_radius = circ.Radius();

            // "Verify then Correct" the solver's tangency parameters.
            std::vector<std::pair<double, double>> corrected_params;
            bool valid_solution = true;

            gp_Pnt2d p_tmp;
            double u_circ1_s, u_arg1_s, u_circ2_s, u_arg2_s;
            gcc.Tangency1(i, u_circ1_s, u_arg1_s, p_tmp);
            gcc.Tangency2(i, u_circ2_s, u_arg2_s, p_tmp);
            const std::pair<double, double> solver_candidates[2] = {{u_circ1_s, u_arg1_s}, {u_circ2_s, u_arg2_s}};

            for (std::size_t j = 0; j < 2; ++j) {
                const double u_circ_s = solver_candidates[j].first, u_arg_s = solver_candidates[j].second;
                if (!args[j].is_edge) {
                    // Point input: the tangency point is the point itself.
                    const gp_Pnt2d p_s_pnt(args[j].point->X(), args[j].point->Y());
                    const double dist = p_s_pnt.Distance(center);
                    if (std::abs(dist - sol_radius) < TOLERANCE) {
                        const gp_Vec2d vec(p_s_pnt.X() - center.X(), p_s_pnt.Y() - center.Y());
                        corrected_params.emplace_back(std::atan2(vec.Y(), vec.X()), u_arg_s);
                    } else {
                        valid_solution = false;
                        break;
                    }
                    continue;
                }
                // Curve input
                const gp_Pnt2d p_s = args[j].h2d->Value(u_arg_s);
                const double dist = p_s.Distance(center);
                // 1. Verify: If solver's point is correct and within trimmed range, use it.
                if (std::abs(dist - sol_radius) < TOLERANCE && ok(j, u_arg_s)) {
                    corrected_params.emplace_back(u_circ_s, u_arg_s);
                } else {
                    // 2. Correct: Project center onto curve to find the actual tangency point.
                    Geom2dAPI_ProjectPointOnCurve proj(gp_Pnt2d(center.X(), center.Y()), args[j].h2d);
                    if (proj.NbPoints() == 0) {
                        valid_solution = false;
                        break;
                    }
                    const double u_arg_act = proj.LowerDistanceParameter();
                    const gp_Pnt2d p_act = args[j].h2d->Value(u_arg_act);
                    if (std::abs(p_act.Distance(center) - sol_radius) > TOLERANCE) {
                        valid_solution = false;
                        break;
                    }
                    if (!ok(j, u_arg_act)) {
                        valid_solution = false;
                        break;
                    }
                    const gp_Vec2d vec(p_act.X() - center.X(), p_act.Y() - center.Y());
                    corrected_params.emplace_back(std::atan2(vec.Y(), vec.X()), u_arg_act);
                }
            }
            if (!valid_solution) continue;

            add_arcs(solutions, circ, corrected_params[0].first, corrected_params[1].first, sagitta);
        }
        return 0;
    });
    return to_edges(solutions);
}

/// Arcs tangent to two objects whose center lies on a given locus on XY.
ShapeList<Edge> make_2tan_on_arcs(std::vector<Tangent> tangent_tuples, const Tangent& center_on, Sagitta sagitta) {
    tangent_tuples.push_back(center_on);
    std::vector<GccArg> args;
    for (const auto& t : tangent_tuples) args.push_back(as_gcc_arg(t));
    if (!args[2].is_edge) throw std::invalid_argument("center_on must be an Edge or an Axis");
    const Geom2dAdaptor_Curve adapt_on(args[2].h2d, args[2].first, args[2].last);

    // Provide initial middle guess parameters for all of the edges
    std::vector<double> guesses;
    for (const auto& a : args) {
        if (a.is_edge) guesses.push_back((a.last - a.first) / 2 + a.first);
    }

    std::optional<Geom2dGcc_Circ2d2TanOn> gcc_opt;
    occt("ConstrainedArcs", [&] {
        if (guesses.size() > 1) {
            if (args[1].is_edge) {
                gcc_opt.emplace(*args[0].qcurve, *args[1].qcurve, adapt_on, TOLERANCE, guesses[0], guesses[1],
                                guesses[2]);
            } else {
                gcc_opt.emplace(*args[0].qcurve, args[1].point, adapt_on, TOLERANCE, guesses[0], guesses[1]);
            }
        } else {
            gcc_opt.emplace(args[0].point, args[1].point, adapt_on, TOLERANCE);
        }
        return 0;
    });
    const Geom2dGcc_Circ2d2TanOn& gcc = *gcc_opt;
    if (!gcc.IsDone() || gcc.NbSolutions() == 0) {
        throw std::runtime_error("Unable to find a tangent arc with center_on constraint");
    }

    auto ok = [&](std::size_t i, double u) {
        return !args[i].is_edge || param_in_trim(u, args[i].first, args[i].last, args[i].h2d);
    };

    std::vector<TopoDS_Edge> solutions;
    occt("ConstrainedArcs", [&] {
        for (int i = 1; i <= gcc.NbSolutions(); ++i) {
            const gp_Circ2d circ = gcc.ThisSolution(i);
            gp_Pnt2d p1, p2;
            double u_circ1, u_arg1, u_circ2, u_arg2;
            gcc.Tangency1(i, u_circ1, u_arg1, p1);
            if (!ok(0, u_arg1)) continue;
            gcc.Tangency2(i, u_circ2, u_arg2, p2);
            if (!ok(1, u_arg2)) continue;
            const auto pars = enclosed_circ_param_offset(tangent_tuples, circ, {u_circ1, u_circ2});
            add_arcs(solutions, circ, pars[0], pars[1], sagitta);
        }
        return 0;
    });
    return to_edges(solutions);
}

/// Arcs on XY tangent to three objects, trimmed between the first two tangency points.
ShapeList<Edge> make_3tan_arcs(const std::vector<Tangent>& tangent_tuples, Sagitta sagitta) {
    std::vector<GccArg> args;
    for (const auto& t : tangent_tuples) args.push_back(as_gcc_arg(t));
    for (const auto& a : args) {
        // Python computes the middle guess of every input, which fails for points.
        if (!a.is_edge) throw std::invalid_argument("unsupported operand type(s) for -: 'NoneType' and 'NoneType'");
    }
    double guesses[3];
    for (std::size_t i = 0; i < 3; ++i) guesses[i] = (args[i].last - args[i].first) / 2 + args[i].first;

    const char* msg = "Unable to find a circle tangent to all three objects";
    std::optional<Geom2dGcc_Circ2d3Tan> gcc_opt;
    try {
        gcc_opt.emplace(*args[0].qcurve, *args[1].qcurve, *args[2].qcurve, TOLERANCE, guesses[0], guesses[1],
                        guesses[2]);
    } catch (const Standard_Failure&) {
        throw std::runtime_error(msg);
    }
    const Geom2dGcc_Circ2d3Tan& gcc = *gcc_opt;
    if (!gcc.IsDone() || gcc.NbSolutions() == 0) throw std::runtime_error(msg);

    auto ok = [&](std::size_t i, double u) {
        return !args[i].is_edge || param_in_trim(u, args[i].first, args[i].last, args[i].h2d);
    };

    std::vector<TopoDS_Edge> out_topos;
    occt("ConstrainedArcs", [&] {
        for (int i = 1; i <= gcc.NbSolutions(); ++i) {
            const gp_Circ2d circ = gcc.ThisSolution(i);
            gp_Pnt2d p1, p2, p3;
            double u_circ1, u_arg1, u_circ2, u_arg2, u_circ3, u_arg3;
            gcc.Tangency1(i, u_circ1, u_arg1, p1);
            if (!ok(0, u_arg1)) continue;
            gcc.Tangency2(i, u_circ2, u_arg2, p2);
            if (!ok(1, u_arg2)) continue;
            gcc.Tangency3(i, u_circ3, u_arg3, p3);
            if (!ok(2, u_arg3)) continue;
            const auto pars = enclosed_circ_param_offset(tangent_tuples, circ, {u_circ1, u_circ2, u_circ3});
            add_arcs(out_topos, circ, pars[0], pars[1], sagitta);
        }
        return 0;
    });
    return to_edges(out_topos);
}

/// Full circles on XY with a fixed center, tangent to a curve or through a point.
ShapeList<Edge> make_tan_cen_arcs(const Tangent& tangency, const TangencyTarget& center) {
    // Build fixed center (gp_Pnt2d)
    gp_Pnt2d c2d;
    if (center.kind() == TangencyTarget::Kind::VERTEX) {
        const Vector loc_xyz = center.vertex().position();
        const Vector base(center.vertex());
        c2d = gp_Pnt2d(base.X + loc_xyz.X, base.Y + loc_xyz.Y);
    } else if (center.kind() == TangencyTarget::Kind::POINT) {
        c2d = gp_Pnt2d(center.point().X, center.point().Y);
    } else {
        throw std::invalid_argument("center must be a point");
    }

    const GccArg arg = as_gcc_arg(tangency);
    std::vector<TopoDS_Edge> solutions_topo;
    occt("ConstrainedArcs", [&] {
        if (!arg.is_edge) {
            // Case A: tangency target is a point -> circle passes through that point
            const gp_Pnt2d p = arg.point->Pnt2d();
            const double dx = p.X() - c2d.X(), dy = p.Y() - c2d.Y();
            const double r = std::pow(dx * dx + dy * dy, 0.5);
            if (r <= TOLERANCE) return 0;  // center coincides with point: no valid circle
            const gp_Circ2d circ(gp_Ax2d(c2d, gp_Dir2d(1.0, 0.0)), r);
            Handle(Geom2d_Circle) h2d = new Geom2d_Circle(circ);
            solutions_topo.push_back(edge_from_circle(h2d, 0.0, h2d->Period()));
        } else {
            // Case B: tangency target is a curve/edge (qualified curve)
            Handle(Geom2d_CartesianPoint) center_point = new Geom2d_CartesianPoint(c2d);
            const Geom2dGcc_Circ2dTanCen gcc(*arg.qcurve, center_point, TOLERANCE);
            if (!(gcc.IsDone() && gcc.NbSolutions() > 0)) {
                throw std::runtime_error("Unexpected: GCC failed to return a tangent circle");
            }
            for (int i = 1; i <= gcc.NbSolutions(); ++i) {
                const gp_Circ2d circ = gcc.ThisSolution(i);
                gp_Pnt2d p1;
                double u_on_circ, u_on_arg;
                gcc.Tangency1(i, u_on_circ, u_on_arg, p1);
                if (!param_in_trim(u_on_arg, arg.first, arg.last, arg.h2d)) continue;
                Handle(Geom2d_Circle) h2d = new Geom2d_Circle(circ);
                solutions_topo.push_back(edge_from_circle(h2d, 0.0, h2d->Period()));
            }
        }
        return 0;
    });
    return to_edges(solutions_topo);
}

/// Full circles on XY of a fixed radius, tangent to a curve (or through a point), with their
/// center on a locus curve.
ShapeList<Edge> make_tan_on_rad_arcs(const Tangent& tangency, const Tangent& center_on, double radius) {
    const GccArg arg = as_gcc_arg(tangency);
    if (center_on.is_point()) throw std::invalid_argument("center_on must be an Edge (line/circle/curve) for TanOnRad.");
    const GccArg on = edge_to_qualified_2d(TopoDS::Edge(center_on.edge->wrapped()), Tangency::UNQUALIFIED);

    std::optional<Geom2dGcc_Circ2dTanOnRad> gcc_opt;
    occt("ConstrainedArcs", [&] {
        if (arg.is_edge) {
            gcc_opt.emplace(*arg.qcurve, *on.adapt2d, radius, TOLERANCE);
        } else {
            gcc_opt.emplace(arg.point, *on.adapt2d, radius, TOLERANCE);
        }
        return 0;
    });
    const Geom2dGcc_Circ2dTanOnRad& gcc = *gcc_opt;
    if (!gcc.IsDone() || gcc.NbSolutions() == 0) {
        throw std::runtime_error("Unable to find circle(s) for TanOnRad constraints");
    }

    std::vector<TopoDS_Edge> out_topos;
    occt("ConstrainedArcs", [&] {
        for (int i = 1; i <= gcc.NbSolutions(); ++i) {
            const gp_Circ2d circ = gcc.ThisSolution(i);
            gp_Pnt2d p;
            double u_on_circ, u_on_arg;
            gcc.Tangency1(i, u_on_circ, u_on_arg, p);
            if (arg.is_edge && !param_in_trim(u_on_arg, arg.first, arg.last, arg.h2d)) continue;
            // Center must lie on the trimmed center_on curve segment
            const gp_Pnt2d center2d = circ.Location();
            Geom2dAPI_ProjectPointOnCurve proj(center2d, on.h2d);
            const double u_on = proj.Parameter(1);
            if (!param_in_trim(u_on, on.first, on.last, on.h2d)) continue;
            Handle(Geom2d_Circle) h2d = new Geom2d_Circle(circ);
            out_topos.push_back(edge_from_circle(h2d, 0.0, h2d->Period()));
        }
        return 0;
    });
    return to_edges(out_topos);
}

// ---------------------------------------------------------------------------
// Line solvers
// ---------------------------------------------------------------------------

/// Lines tangent to two curves, or to a curve and through a point.
ShapeList<Edge> make_2tan_lines(const Tangent& tangency1, const Tangent& tangency2) {
    const GccArg a1 = as_gcc_arg(tangency1);
    std::optional<GccArg> a2;
    std::optional<Geom2dGcc_Lin2d2Tan> gcc_opt;
    occt("ConstrainedLines", [&] {
        if (tangency2.is_point()) {
            const gp_Pnt2d pnt_2d(tangency2.point.X, tangency2.point.Y);
            gcc_opt.emplace(*a1.qcurve, pnt_2d, TOLERANCE);
        } else {
            a2 = as_gcc_arg(tangency2);
            gcc_opt.emplace(*a1.qcurve, *a2->qcurve, TOLERANCE);
        }
        return 0;
    });
    const Geom2dGcc_Lin2d2Tan& gcc = *gcc_opt;
    if (!gcc.IsDone() || gcc.NbSolutions() == 0) throw std::runtime_error("Unable to find common tangent line(s)");

    std::vector<TopoDS_Edge> out_edges;
    occt("ConstrainedLines", [&] {
        for (int i = 1; i <= gcc.NbSolutions(); ++i) {
            Handle(Geom2d_Line) lin2d = new Geom2d_Line(gcc.ThisSolution(i));
            // Two tangency points - Tangency1/Tangency2 can use different indices for the same line
            Geom2dAPI_InterCurveCurve inter_cc(lin2d, a1.h2d);
            const gp_Pnt2d pt1 = inter_cc.Point(1);  // there will always be one tangent intersection
            gp_Pnt2d pt2;
            if (tangency2.is_point()) {
                pt2 = gp_Pnt2d(tangency2.point.X, tangency2.point.Y);
            } else {
                Geom2dAPI_InterCurveCurve inter_cc2(lin2d, a2->h2d);
                pt2 = inter_cc2.Point(1);
            }
            // Skip degenerate lines
            const double separation = pt1.Distance(pt2);
            if (std::isnan(separation) || separation < TOLERANCE) continue;
            out_edges.push_back(edge_from_line(pt1, pt2));
        }
        return 0;
    });
    return to_edges(out_edges);
}

/// Lines tangent to a curve at an angle (radians) to a reference Axis, trimmed between the
/// tangency point and the reference.
ShapeList<Edge> make_tan_oriented_lines(const Tangent& tangency, const Axis& reference, double angle) {
    if (std::abs(std::abs(reference.direction.Z) - 1) < TOLERANCE) {
        throw std::invalid_argument("reference Axis can't be perpendicular to Plane.XY");
    }
    const GccArg q = as_gcc_arg(tangency);

    const Vector ref_dir = reference.direction;
    const double theta_ref = std::atan2(ref_dir.Y, ref_dir.X);
    const double theta_abs = theta_ref + angle;
    const gp_Dir2d dir2d(std::cos(theta_abs), std::sin(theta_abs));
    const gp_Lin2d ref_lin = gp_lin2d_from_axis(reference);

    std::vector<TopoDS_Edge> out;
    occt("ConstrainedLines", [&] {
        const Geom2dGcc_Lin2dTanObl gcc(*q.qcurve, ref_lin, TOLERANCE, angle);
        for (int i = 1; i <= gcc.NbSolutions(); ++i) {
            gp_Pnt2d p_tan;
            double par_sol, par_arg;
            gcc.Tangency1(i, par_sol, par_arg, p_tan);
            const gp_Lin2d tan_line(p_tan, dir2d);
            // Intersect with reference axis (Intersection2 doesn't seem reliable)
            const IntAna2d_AnaIntersection inter(tan_line, ref_lin);
            if (!inter.IsDone() || inter.NbPoints() == 0) continue;
            const gp_Pnt2d p_isect = inter.Point(1).Value();
            const double separation = p_tan.Distance(p_isect);
            if (std::isnan(separation) || separation < TOLERANCE) continue;
            out.push_back(edge_from_line(p_tan, p_isect));
        }
        return 0;
    });
    return to_edges(out);
}

/// Edge.make_constrained_arcs: normalize the targets, then pick the solver.
ShapeList<Edge> constrained_arcs(const std::vector<TangencyTarget>& tangency_args, const ConstrainedArcOptions& opt) {
    std::vector<Tangent> tangencies;
    for (const auto& t : tangency_args) {
        switch (t.kind()) {
            case TangencyTarget::Kind::AXIS:
                if (t.qualified()) {
                    // Python turns (Axis, Tangency) into tuple(Edge(axis, tangency)), which fails.
                    throw std::invalid_argument("'Edge' object is not iterable");
                }
                tangencies.push_back(tangent_edge(Edge(t.axis())));
                break;
            case TangencyTarget::Kind::EDGE: {
                Tangent tangent = tangent_edge(t.edge());
                tangent.qualified = t.qualified();
                tangent.tangency = t.tangency();
                tangencies.push_back(tangent);
                break;
            }
            case TangencyTarget::Kind::VERTEX:
                tangencies.push_back(tangent_point(Vector(t.vertex()) + t.vertex().position()));
                break;
            case TangencyTarget::Kind::POINT: tangencies.push_back(tangent_point(t.point())); break;
        }
    }
    // Sort the tangency inputs so points are always last
    std::stable_sort(tangencies.begin(), tangencies.end(),
                     [](const Tangent& a, const Tangent& b) { return !a.is_point() && b.is_point(); });

    const std::size_t tan_count = tangencies.size();
    if (tan_count < 1 || tan_count > 3) throw std::invalid_argument("Provide 1 to 3 tangency targets.");
    if (opt.radius && *opt.radius <= 0) throw std::invalid_argument("radius must be > 0.0");

    std::optional<Tangent> center_on;
    if (opt.center_on) {
        const TangencyTarget& c = *opt.center_on;
        switch (c.kind()) {
            case TangencyTarget::Kind::AXIS: center_on = tangent_edge(Edge(c.axis())); break;
            case TangencyTarget::Kind::EDGE: {
                Tangent tangent = tangent_edge(c.edge());
                tangent.qualified = c.qualified();
                tangent.tangency = c.tangency();
                center_on = tangent;
                break;
            }
            case TangencyTarget::Kind::VERTEX: center_on = tangent_point(Vector(c.vertex())); break;
            case TangencyTarget::Kind::POINT: center_on = tangent_point(c.point()); break;
        }
    }

    if (tan_count == 2 && opt.radius && !opt.center && !center_on) {
        return make_2tan_rad_arcs(tangencies, *opt.radius, opt.sagitta);
    }
    if (tan_count == 2 && center_on && !opt.radius && !opt.center) {
        return make_2tan_on_arcs(tangencies, *center_on, opt.sagitta);
    }
    if (tan_count == 3 && !opt.radius && !opt.center && !center_on) return make_3tan_arcs(tangencies, opt.sagitta);
    if (tan_count == 1 && opt.center && !opt.radius && !center_on) return make_tan_cen_arcs(tangencies[0], *opt.center);
    if (tan_count == 1 && center_on && opt.radius) return make_tan_on_rad_arcs(tangencies[0], *center_on, *opt.radius);
    throw std::invalid_argument("Unsupported or ambiguous combination of constraints.");
}

}  // namespace

ShapeList<Edge> Edge::make_constrained_arcs(const TangencyTarget& tangency_one, const ConstrainedArcOptions& options) {
    return constrained_arcs({tangency_one}, options);
}

ShapeList<Edge> Edge::make_constrained_arcs(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                                            const ConstrainedArcOptions& options) {
    return constrained_arcs({tangency_one, tangency_two}, options);
}

ShapeList<Edge> Edge::make_constrained_arcs(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                                            const TangencyTarget& tangency_three) {
    return constrained_arcs({tangency_one, tangency_two, tangency_three}, ConstrainedArcOptions{});
}

ShapeList<Edge> Edge::make_constrained_arcs(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                                            const TangencyTarget& tangency_three,
                                            const ConstrainedArcOptions& options) {
    return constrained_arcs({tangency_one, tangency_two, tangency_three}, options);
}

ShapeList<Edge> Edge::make_constrained_lines(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two) {
    return make_constrained_lines(tangency_one, tangency_two, ConstrainedLineOptions{});
}

ShapeList<Edge> Edge::make_constrained_lines(const TangencyTarget& tangency_one, const TangencyTarget& tangency_two,
                                             const ConstrainedLineOptions& options) {
    const bool is_ref = options.angle.has_value() || options.direction.has_value();

    // An Axis is the reference of an oriented line (2nd argument) or an infinite Edge.
    struct Item {
        Tangent tangent;
        std::optional<Axis> axis;  // a reference axis
    };
    std::vector<Item> tangencies;
    const TangencyTarget* targets[2] = {&tangency_one, &tangency_two};
    for (int i = 0; i < 2; ++i) {
        const TangencyTarget& t = *targets[i];
        switch (t.kind()) {
            case TangencyTarget::Kind::AXIS:
                if (t.qualified()) {
                    // (Axis, Tangency) tuples aren't accepted: Python falls back to Vector(tuple).
                    throw std::invalid_argument("Invalid tangency");
                }
                if (i == 1 && is_ref) {
                    tangencies.push_back({tangent_point(Vector()), t.axis()});
                } else {
                    tangencies.push_back({tangent_edge(Edge(t.axis())), std::nullopt});
                }
                break;
            case TangencyTarget::Kind::EDGE: {
                Tangent tangent = tangent_edge(t.edge());
                tangent.qualified = t.qualified();
                tangent.tangency = t.tangency();
                tangencies.push_back({tangent, std::nullopt});
                break;
            }
            case TangencyTarget::Kind::VERTEX: tangencies.push_back({tangent_point(Vector(t.vertex())), std::nullopt}); break;
            case TangencyTarget::Kind::POINT: tangencies.push_back({tangent_point(t.point()), std::nullopt}); break;
        }
    }
    // Sort so Vector (point) | Axis is always last
    auto last_kind = [](const Item& item) { return item.axis.has_value() || item.tangent.is_point(); };
    std::stable_sort(tangencies.begin(), tangencies.end(),
                     [&](const Item& a, const Item& b) { return !last_kind(a) && last_kind(b); });

    if (is_ref) {
        if (last_kind(tangencies[0])) throw std::invalid_argument("Internal error - 1st tangency must be Edge");
        double ang_rad;
        if (options.angle) {
            ang_rad = *options.angle * DEG2RAD;
        } else {
            ang_rad = std::atan2(options.direction->Y, options.direction->X);
        }
        if (!tangencies[1].axis) throw std::invalid_argument("Internal error - 2nd tangency must be an Axis");
        return make_tan_oriented_lines(tangencies[0].tangent, *tangencies[1].axis, ang_rad);
    }
    if (last_kind(tangencies[0])) throw std::invalid_argument("Internal error - 1st tangency can't be an Axis | Vector");
    if (tangencies[1].axis) throw std::invalid_argument("Internal error - 2nd tangency can't be an Axis");
    return make_2tan_lines(tangencies[0].tangent, tangencies[1].tangent);
}

}  // namespace build123d
