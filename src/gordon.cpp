// Gordon surfaces, part 5: the curve network interpolation (ocp_gordon/internal/
// curve_network_sorter.py, gordon_surface_builder.py and interpolate_curve_network.py) and
// build123d's Face.make_gordon_surface (topology/two_d.py).
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <string>
#include <tuple>
#include <variant>

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Tool.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Precision.hxx>
#include <TopoDS.hxx>

#include "build123d/topology.hpp"
#include "gordon_internal.hpp"
#include "internal.hpp"

namespace build123d {

namespace gordon {

namespace {

/// Is the curve a point (a B-spline whose poles are all within tol of the first)?
bool is_zero_length_curve(const Handle(Geom_Curve)& curve, double tol = 1e-9) {
    const Handle(Geom_BSplineCurve) bspline = Handle(Geom_BSplineCurve)::DownCast(curve);
    if (bspline.IsNull()) return false;  // only B-splines are points in this context
    if (bspline->NbPoles() == 0) return true;
    const gp_Pnt ref = bspline->Pole(1);
    for (int i = 2; i <= bspline->NbPoles(); ++i) {
        if (ref.Distance(bspline->Pole(i)) > tol) return false;
    }
    return true;
}

int find_first_non_zero_length_index(const std::vector<Handle(Geom_BSplineCurve)>& curves) {
    for (std::size_t i = 0; i < curves.size(); ++i) {
        if (!is_zero_length_curve(curves[i])) return static_cast<int>(i);
    }
    return -1;
}

// ---------------------------------------------------------------------------
// CurveNetworkSorter
// ---------------------------------------------------------------------------

/// Orders the profiles and guides so that they and their intersection parameters ascend.
class CurveNetworkSorter {
public:
    CurveNetworkSorter(std::vector<Handle(Geom_BSplineCurve)> profiles, std::vector<Handle(Geom_BSplineCurve)> guides,
                       Matrix parms_inters_profiles, Matrix parms_inters_guides)
        : profiles_(std::move(profiles)),
          guides_(std::move(guides)),
          parms_inters_profiles_(std::move(parms_inters_profiles)),
          parms_inters_guides_(std::move(parms_inters_guides)) {
        const int n_profiles = static_cast<int>(profiles_.size());
        const int n_guides = static_cast<int>(guides_.size());
        if (n_profiles != parms_inters_profiles_.rows()) throw std::runtime_error("Invalid row size of parms_inters_profiles matrix.");
        if (n_profiles != parms_inters_guides_.rows()) throw std::runtime_error("Invalid row size of parms_inters_guides matrix.");
        if (n_guides != parms_inters_profiles_.cols()) throw std::runtime_error("Invalid col size of parms_inters_profiles matrix.");
        if (n_guides != parms_inters_guides_.cols()) throw std::runtime_error("Invalid col size of parms_inters_guides matrix.");
    }

    void perform() {
        if (has_performed_) return;
        const auto [prof_start, guide_start, guide_must_be_reversed] = get_start_curve_indices();
        swap_profiles(0, prof_start);
        swap_guides(0, guide_start);
        if (guide_must_be_reversed) reverse_guide(0);
        const int n_guides = static_cast<int>(guides_.size());
        const int n_profiles = static_cast<int>(profiles_.size());
        // the guides ascend along the first profile (bubble sort)
        for (int n = n_guides; n > 1; --n) {
            for (int j = 0; j < n - 1; ++j) {
                if (parms_inters_profiles_(0, j) > parms_inters_profiles_(0, j + 1)) swap_guides(j, j + 1);
            }
        }
        // the profiles ascend along the first guide that is not a point
        const int first_non_zero_guide_idx = find_first_non_zero_length_index(guides_);
        if (first_non_zero_guide_idx == -1) throw std::runtime_error("No non-zero-length guide found to sort profiles.");
        for (int n = n_profiles; n > 1; --n) {
            for (int i = 0; i < n - 1; ++i) {
                if (parms_inters_guides_(i, first_non_zero_guide_idx) > parms_inters_guides_(i + 1, first_non_zero_guide_idx)) {
                    swap_profiles(i, i + 1);
                }
            }
        }
        // the profiles and guides run in the same direction as the first ones
        for (int i_prof = 1; i_prof < n_profiles; ++i_prof) {
            if (parms_inters_profiles_(i_prof, 0) > parms_inters_profiles_(i_prof, n_guides - 1)) reverse_profile(i_prof);
        }
        for (int i_guid = 1; i_guid < n_guides; ++i_guid) {
            if (parms_inters_guides_(0, i_guid) > parms_inters_guides_(n_profiles - 1, i_guid)) reverse_guide(i_guid);
        }
        has_performed_ = true;
    }

    std::vector<Handle(Geom_BSplineCurve)> profiles_;
    std::vector<Handle(Geom_BSplineCurve)> guides_;
    Matrix parms_inters_profiles_;
    Matrix parms_inters_guides_;

private:
    int max_col_index(const Matrix& m, int jcol) const {
        double max_val = -std::numeric_limits<double>::infinity();
        int imax = -1;
        for (int irow = 0; irow < m.rows(); ++irow) {
            if (is_zero_length_curve(profiles_[static_cast<std::size_t>(irow)])) continue;
            if (imax == -1 || m(irow, jcol) > max_val) {
                max_val = m(irow, jcol);
                imax = irow;
            }
        }
        return imax;
    }
    int min_row_index(const Matrix& m, int irow) const {
        double min_val = std::numeric_limits<double>::infinity();
        int jmin = -1;
        for (int jcol = 0; jcol < m.cols(); ++jcol) {
            if (is_zero_length_curve(guides_[static_cast<std::size_t>(jcol)])) continue;
            if (jmin == -1 || m(irow, jcol) < min_val) {
                min_val = m(irow, jcol);
                jmin = jcol;
            }
        }
        return jmin;
    }
    int min_col_index(const Matrix& m, int jcol) const {
        double min_val = std::numeric_limits<double>::infinity();
        int imin = -1;
        for (int irow = 0; irow < m.rows(); ++irow) {
            if (is_zero_length_curve(profiles_[static_cast<std::size_t>(irow)])) continue;
            if (imin == -1 || m(irow, jcol) < min_val) {
                min_val = m(irow, jcol);
                imin = irow;
            }
        }
        return imin;
    }

    void swap_profiles(int idx1, int idx2) {
        if (idx1 == idx2) return;
        std::swap(profiles_[static_cast<std::size_t>(idx1)], profiles_[static_cast<std::size_t>(idx2)]);
        for (int c = 0; c < parms_inters_guides_.cols(); ++c) std::swap(parms_inters_guides_(idx1, c), parms_inters_guides_(idx2, c));
        for (int c = 0; c < parms_inters_profiles_.cols(); ++c) {
            std::swap(parms_inters_profiles_(idx1, c), parms_inters_profiles_(idx2, c));
        }
    }

    void swap_guides(int idx1, int idx2) {
        if (idx1 == idx2) return;
        std::swap(guides_[static_cast<std::size_t>(idx1)], guides_[static_cast<std::size_t>(idx2)]);
        for (int r = 0; r < parms_inters_guides_.rows(); ++r) std::swap(parms_inters_guides_(r, idx1), parms_inters_guides_(r, idx2));
        for (int r = 0; r < parms_inters_profiles_.rows(); ++r) {
            std::swap(parms_inters_profiles_(r, idx1), parms_inters_profiles_(r, idx2));
        }
    }

    std::tuple<int, int, bool> get_start_curve_indices() const {
        // a profile and a guide that both start at their intersection
        for (int irow = 0; irow < static_cast<int>(profiles_.size()); ++irow) {
            if (is_zero_length_curve(profiles_[static_cast<std::size_t>(irow)])) continue;
            const int jmin = min_row_index(parms_inters_profiles_, irow);
            if (jmin == -1) continue;
            const int imin = min_col_index(parms_inters_guides_, jmin);
            if (imin == -1) continue;
            if (imin == irow) return {imin, jmin, false};
        }
        // or a profile that starts where a guide ends (the guide is reversed)
        for (int irow = 0; irow < static_cast<int>(profiles_.size()); ++irow) {
            if (is_zero_length_curve(profiles_[static_cast<std::size_t>(irow)])) continue;
            const int jmin = min_row_index(parms_inters_profiles_, irow);
            if (jmin == -1) continue;
            const int imax = max_col_index(parms_inters_guides_, jmin);
            if (imax == -1) continue;
            if (imax == irow) return {imax, jmin, true};
        }
        throw std::runtime_error("Cannot find starting curves of curve network.");
    }

    void reverse_profile(int profile_idx) {
        const Handle(Geom_BSplineCurve)& profile = profiles_[static_cast<std::size_t>(profile_idx)];
        const double last_parm = profile->LastParameter();
        const double first_parm = profile->FirstParameter();
        for (int icol = 0; icol < static_cast<int>(guides_.size()); ++icol) {
            parms_inters_profiles_(profile_idx, icol) = -parms_inters_profiles_(profile_idx, icol) + first_parm + last_parm;
        }
        profile->Reverse();
    }

    void reverse_guide(int guide_idx) {
        const Handle(Geom_BSplineCurve)& guide = guides_[static_cast<std::size_t>(guide_idx)];
        const double last_parm = guide->LastParameter();
        const double first_parm = guide->FirstParameter();
        for (int irow = 0; irow < static_cast<int>(profiles_.size()); ++irow) {
            parms_inters_guides_(irow, guide_idx) = -parms_inters_guides_(irow, guide_idx) + first_parm + last_parm;
        }
        guide->Reverse();
    }

    bool has_performed_ = false;
};

// ---------------------------------------------------------------------------
// GordonSurfaceBuilder
// ---------------------------------------------------------------------------

/// The Gordon surface: the profile skin plus the guide skin minus the tensor product surface of
/// their intersections.
class GordonSurfaceBuilder {
public:
    GordonSurfaceBuilder(std::vector<Handle(Geom_BSplineCurve)> profiles, std::vector<Handle(Geom_BSplineCurve)> guides,
                         std::vector<double> intersection_params_spline_u,
                         std::vector<double> intersection_params_spline_v, double tolerance)
        : profiles_(std::move(profiles)),
          guides_(std::move(guides)),
          intersection_params_spline_u_(std::move(intersection_params_spline_u)),
          intersection_params_spline_v_(std::move(intersection_params_spline_v)),
          tolerance_(tolerance) {}

    Handle(Geom_BSplineSurface) surface_gordon() {
        perform();
        return surface_gordon_;
    }

private:
    void perform() {
        if (has_performed_) return;
        create_gordon_surface(profiles_, guides_, intersection_params_spline_u_, intersection_params_spline_v_);
        has_performed_ = true;
    }

    static Handle(Geom_BSplineSurface) points_to_surface_internal(const TColgp_Array2OfPnt& points,
                                                                  const std::vector<double>& u_params,
                                                                  const std::vector<double>& v_params, bool make_u_closed,
                                                                  bool make_v_closed) {
        // interpolate the columns in u, then skin the curves in v
        std::vector<Handle(Geom_BSplineCurve)> u_direction_curves;
        for (int col_idx = points.LowerCol(); col_idx <= points.UpperCol(); ++col_idx) {
            const auto points_u_line = pnt_array2_get_column(points, col_idx);
            const PointsToBSplineInterpolation interpolator(points_u_line, u_params, 3, make_u_closed);
            u_direction_curves.push_back(interpolator.curve());
        }
        CurvesToSurface skinner(u_direction_curves, v_params, make_v_closed);
        return skinner.surface();
    }

    static std::vector<Handle(Geom_BSplineSurface)> create_common_knots_vector_surface_internal(
        const std::vector<Handle(Geom_BSplineSurface)>& surfaces_vector, double tol = 1e-7) {
        if (surfaces_vector.empty()) return {};
        std::vector<Handle(Geom_BSplineSurface)> result_surfaces;
        for (const auto& s : surfaces_vector) result_surfaces.push_back(clone_bspline_surface(s));
        // the knots of every surface in u, then in v, with their largest multiplicities
        for (const bool u_dir : {true, false}) {
            std::set<double> all_knots;
            for (const auto& surf : result_surfaces) {
                const int n_knots = u_dir ? surf->NbUKnots() : surf->NbVKnots();
                for (int i = 1; i <= n_knots; ++i) all_knots.insert(u_dir ? surf->UKnot(i) : surf->VKnot(i));
            }
            TColStd_Array1OfReal common_knots(1, static_cast<int>(all_knots.size()));
            TColStd_Array1OfInteger common_mults(1, static_cast<int>(all_knots.size()));
            int idx = 1;
            for (double knot : all_knots) {
                int max_mult = 0;
                for (const auto& surf : result_surfaces) {
                    const int n_knots = u_dir ? surf->NbUKnots() : surf->NbVKnots();
                    for (int i = 1; i <= n_knots; ++i) {
                        if (std::abs((u_dir ? surf->UKnot(i) : surf->VKnot(i)) - knot) < tol) {
                            max_mult = std::max(max_mult, u_dir ? surf->UMultiplicity(i) : surf->VMultiplicity(i));
                            break;  // found the knot, move to the next surface
                        }
                    }
                }
                common_knots.SetValue(idx, knot);
                common_mults.SetValue(idx, max_mult);
                ++idx;
            }
            for (const auto& surf : result_surfaces) {
                if (u_dir) {
                    surf->InsertUKnots(common_knots, common_mults, tol, false);
                } else {
                    surf->InsertVKnots(common_knots, common_mults, tol, false);
                }
            }
        }
        return result_surfaces;
    }

    static void assert_range(const Handle(Geom_BSplineCurve)& curve, double umin, double umax, double tol = 1e-7) {
        if (std::abs(curve->FirstParameter() - umin) > tol || std::abs(curve->LastParameter() - umax) > tol) {
            throw std::runtime_error("Curve not in range [" + detail::py_repr(umin) + ", " + detail::py_repr(umax) + "].");
        }
    }

    static void check_curve_network_compatibility(const std::vector<Handle(Geom_BSplineCurve)>& profiles,
                                                  const std::vector<Handle(Geom_BSplineCurve)>& guides,
                                                  const std::vector<double>& intersection_params_spline_u,
                                                  const std::vector<double>& intersection_params_spline_v, double tol) {
        const double splines_scale = 0.5 * (scale(profiles) + scale(guides));
        if (std::abs(intersection_params_spline_u.front()) > splines_scale * tol ||
            std::abs(intersection_params_spline_u.back() - 1.0) > splines_scale * tol) {
            throw std::runtime_error(
                "WARNING: B-splines in u-direction mustn't stick out, spline network must be 'closed'!");
        }
        if (std::abs(intersection_params_spline_v.front()) > splines_scale * tol ||
            std::abs(intersection_params_spline_v.back() - 1.0) > splines_scale * tol) {
            throw std::runtime_error(
                "WARNING: B-splines in v-direction mustn't stick out, spline network must be 'closed'!");
        }
        // the profiles and guides meet at the intersection parameters
        for (std::size_t u_param_idx = 0; u_param_idx < intersection_params_spline_u.size(); ++u_param_idx) {
            const double spline_u_param = intersection_params_spline_u[u_param_idx];
            const Handle(Geom_BSplineCurve)& spline_v = guides.at(u_param_idx);
            for (std::size_t v_param_idx = 0; v_param_idx < intersection_params_spline_v.size(); ++v_param_idx) {
                const Handle(Geom_BSplineCurve)& spline_u = profiles.at(v_param_idx);
                const double spline_v_param = intersection_params_spline_v[v_param_idx];
                const gp_Pnt p_prof = spline_u->Value(spline_u_param);
                const gp_Pnt p_guid = spline_v->Value(spline_v_param);
                if (p_prof.Distance(p_guid) > splines_scale * tol) {
                    throw std::runtime_error(
                        "B-spline network is incompatible (e.g. wrong parametrization) or intersection parameters are in "
                        "a wrong order!");
                }
            }
        }
    }

    void create_gordon_surface(const std::vector<Handle(Geom_BSplineCurve)>& profiles,
                               const std::vector<Handle(Geom_BSplineCurve)>& guides,
                               const std::vector<double>& intersection_params_spline_u,
                               const std::vector<double>& intersection_params_spline_v) {
        if (profiles.size() < 2) throw std::runtime_error("There must be at least two profiles for the gordon surface.");
        if (guides.size() < 2) throw std::runtime_error("There must be at least two guides for the gordon surface.");
        // all the profiles and all the guides have the same parameter range
        const double umin = profiles_[0]->FirstParameter(), umax = profiles_[0]->LastParameter();
        for (const auto& profile : profiles_) assert_range(profile, umin, umax, 1e-5);
        const double vmin = guides_[0]->FirstParameter(), vmax = guides_[0]->LastParameter();
        for (const auto& guide : guides_) assert_range(guide, vmin, vmax, 1e-5);
        check_curve_network_compatibility(profiles, guides, intersection_params_spline_u, intersection_params_spline_v,
                                          tolerance_);
        // the intersection points
        const int n_u_params = static_cast<int>(intersection_params_spline_u.size());
        const int n_v_params = static_cast<int>(intersection_params_spline_v.size());
        TColgp_Array2OfPnt intersection_points(1, n_u_params, 1, n_v_params);
        for (std::size_t spline_idx = 0; spline_idx < profiles.size(); ++spline_idx) {
            for (int intersection_idx = 0; intersection_idx < n_u_params; ++intersection_idx) {
                const double parameter = intersection_params_spline_u[static_cast<std::size_t>(intersection_idx)];
                if (static_cast<int>(spline_idx) + 1 > n_v_params) throw std::runtime_error("Standard_OutOfRange");
                intersection_points.SetValue(intersection_idx + 1, static_cast<int>(spline_idx) + 1,
                                             profiles[spline_idx]->Value(parameter));
            }
        }
        // closed in u or v when the first and last curves and intersections coincide
        const double curve_u_tolerance = REL_TOL_CLOSED * scale(guides);
        const double curve_v_tolerance = REL_TOL_CLOSED * scale(profiles);
        const double tp_tolerance = REL_TOL_CLOSED * scale(intersection_points);
        const bool make_u_closed =
            is_u_dir_closed(intersection_points, tp_tolerance) && guides.front()->IsEqual(guides.back(), curve_u_tolerance);
        const bool make_v_closed = is_v_dir_closed(intersection_points, tp_tolerance) &&
                                   profiles.front()->IsEqual(profiles.back(), curve_v_tolerance);
        // the skins of the profiles and of the guides, and the tensor product surface
        CurvesToSurface surf_profiles_skinner(profiles, intersection_params_spline_v, make_v_closed);
        const Handle(Geom_BSplineSurface) surf_profiles = surf_profiles_skinner.surface();
        CurvesToSurface surf_guides_skinner(guides, intersection_params_spline_u, make_u_closed);
        Handle(Geom_BSplineSurface) surf_guides = surf_guides_skinner.surface();
        // the guides surface goes in the other direction
        surf_guides = flip_surface(surf_guides);
        const Handle(Geom_BSplineSurface) tensor_prod_surf = points_to_surface_internal(
            intersection_points, intersection_params_spline_u, intersection_params_spline_v, make_u_closed, make_v_closed);
        // the same degrees and knots
        const int degree_u = std::max({surf_guides->UDegree(), surf_profiles->UDegree(), tensor_prod_surf->UDegree()});
        const int degree_v = std::max({surf_guides->VDegree(), surf_profiles->VDegree(), tensor_prod_surf->VDegree()});
        surf_guides->IncreaseDegree(degree_u, degree_v);
        surf_profiles->IncreaseDegree(degree_u, degree_v);
        tensor_prod_surf->IncreaseDegree(degree_u, degree_v);
        const auto surfaces_vector =
            create_common_knots_vector_surface_internal({surf_guides, surf_profiles, tensor_prod_surf});
        const Handle(Geom_BSplineSurface)& surface_guides = surfaces_vector[0];
        const Handle(Geom_BSplineSurface)& surface_profiles = surfaces_vector[1];
        const Handle(Geom_BSplineSurface)& surface_intersections = surfaces_vector[2];
        if (!(surface_guides->NbUPoles() == surface_profiles->NbUPoles() &&
              surface_profiles->NbUPoles() == surface_intersections->NbUPoles()) ||
            !(surface_guides->NbVPoles() == surface_profiles->NbVPoles() &&
              surface_profiles->NbVPoles() == surface_intersections->NbVPoles())) {
            throw std::runtime_error("AssertionError");
        }
        // the Gordon surface: profiles + guides - intersections
        surface_gordon_ = clone_bspline_surface(surface_profiles);
        for (int cp_u_idx = 1; cp_u_idx <= surface_gordon_->NbUPoles(); ++cp_u_idx) {
            for (int cp_v_idx = 1; cp_v_idx <= surface_gordon_->NbVPoles(); ++cp_v_idx) {
                const gp_Pnt cp_surf_u = surface_profiles->Pole(cp_u_idx, cp_v_idx);
                const gp_Pnt cp_surf_v = surface_guides->Pole(cp_u_idx, cp_v_idx);
                const gp_Pnt cp_tensor = surface_intersections->Pole(cp_u_idx, cp_v_idx);
                const gp_XYZ new_pole_xyz = cp_surf_u.XYZ() + cp_surf_v.XYZ() - cp_tensor.XYZ();
                surface_gordon_->SetPole(cp_u_idx, cp_v_idx, gp_Pnt(new_pole_xyz.X(), new_pole_xyz.Y(), new_pole_xyz.Z()));
            }
        }
    }

    std::vector<Handle(Geom_BSplineCurve)> profiles_;
    std::vector<Handle(Geom_BSplineCurve)> guides_;
    std::vector<double> intersection_params_spline_u_;
    std::vector<double> intersection_params_spline_v_;
    double tolerance_;
    Handle(Geom_BSplineSurface) surface_gordon_;
    bool has_performed_ = false;
};

// ---------------------------------------------------------------------------
// InterpolateCurveNetwork
// ---------------------------------------------------------------------------

/// The curve network interpolation: the curves are made compatible (sorted, reparametrized to
/// meet at common parameters) and a Gordon surface is built through them.
class InterpolateCurveNetwork {
public:
    InterpolateCurveNetwork(const std::vector<Handle(Geom_BSplineCurve)>& profiles,
                            const std::vector<Handle(Geom_BSplineCurve)>& guides, double spatial_tolerance)
        : spatial_tol_(spatial_tolerance) {
        if (profiles.size() < 2) {
            throw std::runtime_error("There must be at least two profiles for the curve network interpolation.");
        }
        if (guides.size() < 2) {
            throw std::runtime_error("There must be at least two guides for the curve network interpolation.");
        }
        // duplicate curves are ignored
        const auto unique_curves = [](const std::vector<Handle(Geom_BSplineCurve)>& curves) {
            std::vector<Handle(Geom_BSplineCurve)> unique;
            for (const auto& curve : curves) {
                const bool is_unique = std::none_of(unique.begin(), unique.end(), [&](const Handle(Geom_BSplineCurve)& u) {
                    return curve->IsEqual(u, Precision::PConfusion());
                });
                if (is_unique) unique.push_back(curve);
            }
            return unique;
        };
        profiles_ = unique_curves(profiles);
        guides_ = unique_curves(guides);
        if (profiles_.size() < 2) {
            throw std::runtime_error("There must be at least two unique profiles for the curve network interpolation.");
        }
        if (guides_.size() < 2) {
            throw std::runtime_error("There must be at least two unique guides for the curve network interpolation.");
        }
    }

    Handle(Geom_BSplineSurface) surface() {
        perform();
        return gordon_surf_;
    }

private:
    void perform() {
        if (has_performed_) return;
        make_curves_compatible();
        GordonSurfaceBuilder builder(profiles_, guides_, intersection_params_u_, intersection_params_v_, spatial_tol_);
        gordon_surf_ = builder.surface_gordon();
        ensure_c2();
        has_performed_ = true;
    }

    void compute_intersections_matrix(Matrix& initial_intersection_params_u, Matrix& initial_intersection_params_v) const {
        const int n_profiles = static_cast<int>(profiles_.size());
        const int n_guides = static_cast<int>(guides_.size());
        for (int spline_u_idx = 0; spline_u_idx < n_profiles; ++spline_u_idx) {
            for (int spline_v_idx = 0; spline_v_idx < n_guides; ++spline_v_idx) {
                const auto intersections = intersect_bsplines(profiles_[static_cast<std::size_t>(spline_u_idx)],
                                                              guides_[static_cast<std::size_t>(spline_v_idx)], spatial_tol_);
                const std::string pair = std::to_string(spline_u_idx) + " and V-directional B-spline " +
                                         std::to_string(spline_v_idx);
                if (intersections.empty()) {
                    throw std::runtime_error("U-directional B-spline " + pair + " don't intersect!");
                } else if (intersections.size() == 1) {
                    initial_intersection_params_u(spline_u_idx, spline_v_idx) = intersections[0].parm_on_curve1;
                    initial_intersection_params_v(spline_u_idx, spline_v_idx) = intersections[0].parm_on_curve2;
                } else if (intersections.size() == 2) {
                    // a closed curve meets the other twice: its start
                    initial_intersection_params_u(spline_u_idx, spline_v_idx) =
                        std::min(intersections[0].parm_on_curve1, intersections[1].parm_on_curve1);
                    initial_intersection_params_v(spline_u_idx, spline_v_idx) =
                        std::min(intersections[0].parm_on_curve2, intersections[1].parm_on_curve2);
                } else {
                    throw std::runtime_error("U-directional B-spline " + pair +
                                             " have more than two intersections! Closed in both U and V directions "
                                             "surface isn't supported at this time.");
                }
            }
        }
    }

    void make_curves_compatible() {
        // all the curves are parametrized on [0, 1]
        for (const auto& profile : profiles_) reparametrize_bspline(profile, 0.0, 1.0, 1e-15);
        for (const auto& guide : guides_) reparametrize_bspline(guide, 0.0, 1.0, 1e-15);
        const int n_profiles = static_cast<int>(profiles_.size());
        const int n_guides = static_cast<int>(guides_.size());
        Matrix initial_intersection_params_u(n_profiles, n_guides);
        Matrix initial_intersection_params_v(n_profiles, n_guides);
        compute_intersections_matrix(initial_intersection_params_u, initial_intersection_params_v);
        // sort the curves (the sorter reverses curves in place)
        CurveNetworkSorter sorter(profiles_, guides_, initial_intersection_params_u, initial_intersection_params_v);
        sorter.perform();
        profiles_ = to_bsplines(sorter.profiles_);
        guides_ = to_bsplines(sorter.guides_);
        // points may only be the first or last curves
        for (std::size_t i = 1; i + 1 < profiles_.size(); ++i) {
            if (is_zero_length_curve(profiles_[i])) {
                throw std::invalid_argument("Profile#" + std::to_string(i) +
                                            " is a point. Points are only permitted at the beginning and end.");
            }
        }
        for (std::size_t i = 1; i + 1 < guides_.size(); ++i) {
            if (is_zero_length_curve(guides_[i])) {
                throw std::invalid_argument("Guides#" + std::to_string(i) +
                                            " is a point. Points are only permitted at the beginning and end.");
            }
        }
        const Matrix& sorted_intersection_params_u = sorter.parms_inters_profiles_;
        const Matrix& sorted_intersection_params_v = sorter.parms_inters_guides_;
        // closed profiles or guides?
        const int first_non_zero_profile_idx = find_first_non_zero_length_index(profiles_);
        const int first_non_zero_guide_idx = find_first_non_zero_length_index(guides_);
        if (first_non_zero_profile_idx == -1) {
            throw std::runtime_error("No non-zero-length profile found for closed curve check.");
        }
        if (first_non_zero_guide_idx == -1) throw std::runtime_error("No non-zero-length guide found for closed curve check.");
        const Handle(Geom_BSplineCurve)& first_non_zero_profile = profiles_[static_cast<std::size_t>(first_non_zero_profile_idx)];
        const Handle(Geom_BSplineCurve)& first_non_zero_guide = guides_[static_cast<std::size_t>(first_non_zero_guide_idx)];
        const bool is_closed_profile = first_non_zero_profile->IsClosed() || first_non_zero_profile->IsPeriodic();
        const bool is_closed_guides = first_non_zero_guide->IsClosed() || first_non_zero_guide->IsPeriodic();
        if (is_closed_profile && is_closed_guides) {
            throw std::runtime_error("Both profiles and guides cannot be closed simultaneously.");
        }
        int current_n_profiles = static_cast<int>(profiles_.size());
        int current_n_guides = static_cast<int>(guides_.size());
        Matrix final_intersection_params_u;
        Matrix final_intersection_params_v;
        if (is_closed_profile) {
            // a closed profile: the first guide closes the network again at the end
            guides_.push_back(guides_[0]);
            current_n_guides += 1;
            final_intersection_params_u = Matrix(current_n_profiles, current_n_guides);
            final_intersection_params_v = Matrix(current_n_profiles, current_n_guides);
            const int original_n_profiles = current_n_profiles;
            const int original_n_guides = current_n_guides - 1;
            for (int i = 0; i < original_n_profiles; ++i) {
                for (int j = 0; j < original_n_guides; ++j) {
                    final_intersection_params_u(i, j) = sorted_intersection_params_u(i, j);
                    final_intersection_params_v(i, j) = sorted_intersection_params_v(i, j);
                }
            }
            for (int i = 0; i < original_n_profiles; ++i) {
                const double val_u = sorted_intersection_params_u(i, 0);
                final_intersection_params_u(i, original_n_guides) = std::abs(val_u) < PAR_CHECK_TOL ? 1.0 : val_u;
                final_intersection_params_v(i, original_n_guides) = sorted_intersection_params_v(i, 0);
            }
        } else if (is_closed_guides) {
            // closed guides: the first profile closes the network again at the end
            profiles_.push_back(profiles_[0]);
            current_n_profiles += 1;
            final_intersection_params_u = Matrix(current_n_profiles, current_n_guides);
            final_intersection_params_v = Matrix(current_n_profiles, current_n_guides);
            const int original_n_profiles = current_n_profiles - 1;
            const int original_n_guides = current_n_guides;
            for (int i = 0; i < original_n_profiles; ++i) {
                for (int j = 0; j < original_n_guides; ++j) {
                    final_intersection_params_u(i, j) = sorted_intersection_params_u(i, j);
                    final_intersection_params_v(i, j) = sorted_intersection_params_v(i, j);
                }
            }
            for (int j = 0; j < original_n_guides; ++j) {
                const double val_v = sorted_intersection_params_v(0, j);
                final_intersection_params_u(original_n_profiles, j) = sorted_intersection_params_u(0, j);
                final_intersection_params_v(original_n_profiles, j) = std::abs(val_v) < PAR_CHECK_TOL ? 1.0 : val_v;
            }
        } else {
            final_intersection_params_u = sorted_intersection_params_u;
            final_intersection_params_v = sorted_intersection_params_v;
        }
        eliminate_inaccuracies_network_intersections(profiles_, guides_, final_intersection_params_u,
                                                     final_intersection_params_v);
        // the mean intersection parameters of each guide (on the profiles) and of each profile
        current_n_profiles = static_cast<int>(profiles_.size());
        current_n_guides = static_cast<int>(guides_.size());
        std::vector<double> new_parameters_profiles;
        for (int j = 0; j < current_n_guides; ++j) {
            double sum_u = 0.0;
            int count_non_zero_profiles = 0;
            for (int i = 0; i < current_n_profiles; ++i) {
                if (is_zero_length_curve(profiles_[static_cast<std::size_t>(i)])) continue;  // points are excluded
                sum_u += final_intersection_params_u(i, j);
                count_non_zero_profiles += 1;
            }
            new_parameters_profiles.push_back(count_non_zero_profiles > 0 ? sum_u / count_non_zero_profiles : 0.0);
        }
        std::vector<double> new_parameters_guides;
        for (int i = 0; i < current_n_profiles; ++i) {
            double sum_v = 0.0;
            int count_non_zero_guides = 0;
            for (int j = 0; j < current_n_guides; ++j) {
                if (is_zero_length_curve(guides_[static_cast<std::size_t>(j)])) continue;  // points are excluded
                sum_v += final_intersection_params_v(i, j);
                count_non_zero_guides += 1;
            }
            new_parameters_guides.push_back(count_non_zero_guides > 0 ? sum_v / count_non_zero_guides : 0.0);
        }
        if (new_parameters_profiles[0] > PAR_CHECK_TOL || new_parameters_guides[0] > PAR_CHECK_TOL) {
            throw std::runtime_error("At least one B-spline has no intersection at the beginning.");
        }
        // the number of control points of the reparametrized curves
        int max_cp_u_orig = 0, max_cp_v_orig = 0;
        for (const auto& profile : profiles_) max_cp_u_orig = std::max(max_cp_u_orig, profile->NbPoles());
        for (const auto& guide : guides_) max_cp_v_orig = std::max(max_cp_v_orig, guide->NbPoles());
        const int min_cp = 10;
        const int max_cp = 120;
        const int min_u = std::max(current_n_guides + 2, min_cp);
        const int min_v = std::max(current_n_profiles + 2, min_cp);
        const int max_u = std::max(min_u, max_cp);
        const int max_v = std::max(min_v, max_cp);
        const int final_max_cp_u = clamp(max_cp_u_orig + 10, min_u, max_u);
        const int final_max_cp_v = clamp(max_cp_v_orig + 10, min_v, max_v);
        const auto skip_reparametrize = [](const Handle(Geom_BSplineCurve)& curve, const std::vector<double>& old_parameters,
                                           const std::vector<double>& new_parameters) {
            if (curve->IsRational() || curve->IsPeriodic()) return false;
            for (std::size_t i = 0; i < old_parameters.size(); ++i) {
                if (!(std::abs(old_parameters[i] - new_parameters[i]) < Precision::PConfusion())) return false;
            }
            return true;
        };
        // reparametrize the profiles so that they meet the guides at the mean parameters
        for (int i = 0; i < current_n_profiles; ++i) {
            const auto ui = static_cast<std::size_t>(i);
            if (is_zero_length_curve(profiles_[ui])) continue;  // points are skipped
            std::vector<double> old_parameters;
            for (int j = 0; j < current_n_guides; ++j) old_parameters.push_back(final_intersection_params_u(i, j));
            if (skip_reparametrize(profiles_[ui], old_parameters, new_parameters_profiles)) continue;
            if (std::abs(old_parameters.front()) < PAR_CHECK_TOL) old_parameters.front() = 0.0;
            if (std::abs(new_parameters_profiles.front()) < PAR_CHECK_TOL) new_parameters_profiles.front() = 0.0;
            if (std::abs(old_parameters.back() - 1.0) < PAR_CHECK_TOL) old_parameters.back() = 1.0;
            if (std::abs(new_parameters_profiles.back() - 1.0) < PAR_CHECK_TOL) new_parameters_profiles.back() = 1.0;
            const ApproxResult result = reparametrize_bspline_continuously_approx(profiles_[ui], old_parameters,
                                                                                  new_parameters_profiles, final_max_cp_u);
            if (!result.curve.IsNull()) profiles_[ui] = result.curve;
        }
        // and the guides likewise
        for (int j = 0; j < current_n_guides; ++j) {
            const auto uj = static_cast<std::size_t>(j);
            if (is_zero_length_curve(guides_[uj])) continue;  // points are skipped
            std::vector<double> old_parameters;
            for (int i = 0; i < current_n_profiles; ++i) old_parameters.push_back(final_intersection_params_v(i, j));
            if (skip_reparametrize(guides_[uj], old_parameters, new_parameters_guides)) continue;
            if (std::abs(old_parameters.front()) < PAR_CHECK_TOL) old_parameters.front() = 0.0;
            if (std::abs(new_parameters_guides.front()) < PAR_CHECK_TOL) new_parameters_guides.front() = 0.0;
            if (std::abs(old_parameters.back() - 1.0) < PAR_CHECK_TOL) old_parameters.back() = 1.0;
            if (std::abs(new_parameters_guides.back() - 1.0) < PAR_CHECK_TOL) new_parameters_guides.back() = 1.0;
            const ApproxResult result = reparametrize_bspline_continuously_approx(guides_[uj], old_parameters,
                                                                                  new_parameters_guides, final_max_cp_v);
            if (!result.curve.IsNull()) guides_[uj] = result.curve;
        }
        intersection_params_u_ = new_parameters_profiles;
        intersection_params_v_ = new_parameters_guides;
    }

    static void eliminate_inaccuracies_network_intersections(const std::vector<Handle(Geom_BSplineCurve)>& sorted_splines_u,
                                                             const std::vector<Handle(Geom_BSplineCurve)>& sorted_splines_v,
                                                             Matrix& intersection_params_u, Matrix& intersection_params_v) {
        const int n_profiles = static_cast<int>(sorted_splines_u.size());
        const int n_guides = static_cast<int>(sorted_splines_v.size());
        // the intersection parameters near the ends are the ends
        for (int spline_u_idx = 0; spline_u_idx < n_profiles; ++spline_u_idx) {
            if (std::abs(intersection_params_u(spline_u_idx, 0) - sorted_splines_u[0]->Knot(1)) < 0.001) {
                intersection_params_u(spline_u_idx, 0) =
                    std::abs(sorted_splines_u[0]->Knot(1)) < 1e-10 ? 0 : sorted_splines_u[0]->Knot(1);
            }
        }
        for (int spline_v_idx = 0; spline_v_idx < n_guides; ++spline_v_idx) {
            if (std::abs(intersection_params_v(0, spline_v_idx) - sorted_splines_v[0]->Knot(1)) < 0.001) {
                intersection_params_v(0, spline_v_idx) =
                    std::abs(sorted_splines_v[0]->Knot(1)) < 1e-10 ? 0 : sorted_splines_v[0]->Knot(1);
            }
        }
        for (int spline_u_idx = 0; spline_u_idx < n_profiles; ++spline_u_idx) {
            const double last_knot_u = sorted_splines_u[0]->Knot(sorted_splines_u[0]->NbKnots());
            if (std::abs(intersection_params_u(spline_u_idx, n_guides - 1) - last_knot_u) < 0.001) {
                intersection_params_u(spline_u_idx, n_guides - 1) = last_knot_u;
            }
        }
        for (int spline_v_idx = 0; spline_v_idx < n_guides; ++spline_v_idx) {
            const double last_knot_v = sorted_splines_v[0]->Knot(sorted_splines_v[0]->NbKnots());
            if (std::abs(intersection_params_v(n_profiles - 1, spline_v_idx) - last_knot_v) < 0.001) {
                intersection_params_v(n_profiles - 1, spline_v_idx) = last_knot_v;
            }
        }
    }

    void ensure_c2() {
        if (gordon_surf_.IsNull()) return;
        // the inner knots' multiplicities are reduced to degree - 2
        const int u_degree = gordon_surf_->UDegree();
        const int v_degree = gordon_surf_->VDegree();
        const int min_u_mult = std::max(1, u_degree - 2);
        const int min_v_mult = std::max(1, v_degree - 2);
        const int n_u_knots = gordon_surf_->NbUKnots();
        for (int i = 2; i < n_u_knots; ++i) {
            if (gordon_surf_->UMultiplicity(i) > min_u_mult) gordon_surf_->RemoveUKnot(i, min_u_mult, spatial_tol_);
        }
        const int n_v_knots = gordon_surf_->NbVKnots();
        for (int i = 2; i < n_v_knots; ++i) {
            if (gordon_surf_->VMultiplicity(i) > min_v_mult) gordon_surf_->RemoveVKnot(i, min_v_mult, spatial_tol_);
        }
    }

    static int clamp(int val, int min_val, int max_val) {
        if (min_val > max_val) throw std::invalid_argument("Minimum may not be larger than maximum in clamp!");
        return std::max(min_val, std::min(val, max_val));
    }

    std::vector<Handle(Geom_BSplineCurve)> profiles_;
    std::vector<Handle(Geom_BSplineCurve)> guides_;
    double spatial_tol_;
    bool has_performed_ = false;
    Handle(Geom_BSplineSurface) gordon_surf_;
    std::vector<double> intersection_params_u_;
    std::vector<double> intersection_params_v_;
};

}  // namespace

Handle(Geom_BSplineSurface) interpolate_curve_network(const std::vector<Handle(Geom_Curve)>& ucurves,
                                                      const std::vector<Handle(Geom_Curve)>& vcurves, double tolerance) {
    // B-splines of the curves
    const auto u_bsplines = to_bsplines(ucurves);
    const auto v_bsplines = to_bsplines(vcurves);
    InterpolateCurveNetwork interpolator(u_bsplines, v_bsplines, tolerance);
    return interpolator.surface();
}

}  // namespace gordon

// ---------------------------------------------------------------------------
// Face::make_gordon_surface
// ---------------------------------------------------------------------------

namespace {

/// A point as a zero-length B-spline of degree `degree`.
Handle(Geom_BSplineCurve) create_zero_length_bspline_curve(const gp_Pnt& point, int degree = 1) {
    TColgp_Array1OfPnt control_points(1, 2);
    control_points.SetValue(1, point);
    control_points.SetValue(2, point);
    TColStd_Array1OfReal knots(1, 2);
    knots.SetValue(1, 0.0);
    knots.SetValue(2, 1.0);
    TColStd_Array1OfInteger multiplicities(1, 2);
    multiplicities.SetValue(1, degree + 1);
    multiplicities.SetValue(2, degree + 1);
    return new Geom_BSplineCurve(control_points, knots, multiplicities, degree);
}

Handle(Geom_Curve) to_geom_curve(const std::variant<Vector, Edge>& shape) {
    if (const auto* point = std::get_if<Vector>(&shape)) return create_zero_length_bspline_curve(point->to_pnt());
    const Edge& edge = std::get<Edge>(shape);
    if (edge.is_null()) throw std::invalid_argument("input Edge cannot be empty");
    const TopoDS_Edge& topods_edge = TopoDS::Edge(edge.wrapped());
    const BRepAdaptor_Curve adaptor(topods_edge);
    double first = 0, last = 1;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(topods_edge, first, last);
    // the edge's own range, unless the curve is a closed periodic curve, a B-spline or a Bezier
    if (!((adaptor.IsPeriodic() && adaptor.IsClosed()) || adaptor.GetType() == GeomAbs_BSplineCurve ||
          adaptor.GetType() == GeomAbs_BezierCurve)) {
        curve = new Geom_TrimmedCurve(curve, adaptor.FirstParameter(), adaptor.LastParameter());
    }
    return curve;
}

}  // namespace

Face Face::make_gordon_surface(const std::vector<std::variant<Vector, Edge>>& profiles,
                               const std::vector<std::variant<Vector, Edge>>& guides, double tolerance) {
    return detail::occt("make_gordon_surface", [&] {
        std::vector<Handle(Geom_Curve)> ocp_profiles, ocp_guides;
        for (const auto& shape : profiles) ocp_profiles.push_back(to_geom_curve(shape));
        for (const auto& shape : guides) ocp_guides.push_back(to_geom_curve(shape));
        const Handle(Geom_BSplineSurface) gordon_bspline_surface =
            gordon::interpolate_curve_network(ocp_profiles, ocp_guides, tolerance);
        return Face(BRepBuilderAPI_MakeFace(gordon_bspline_surface, Precision::Confusion()).Face());
    });
}

Face Face::make_gordon_surface(const std::vector<Edge>& profiles, const std::vector<Edge>& guides, double tolerance) {
    return make_gordon_surface(std::vector<std::variant<Vector, Edge>>(profiles.begin(), profiles.end()),
                               std::vector<std::variant<Vector, Edge>>(guides.begin(), guides.end()), tolerance);
}

}  // namespace build123d
