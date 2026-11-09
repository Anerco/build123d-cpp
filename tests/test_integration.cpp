// Unit tests for C++-only details of features that span the work areas.
#include <build123d/build123d.hpp>

#include <BRepAdaptor_CompCurve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>

#include "test_framework.hpp"

using namespace build123d;

TEST(geom_adaptors) {
    const Edge line = Edge::make_line(Vector(0, 0, 0), Vector(3, 4, 0));
    CHECK(line.geom_adaptor()->GetType() == GeomAbs_Line);
    CHECK_NEAR(GCPnts_AbscissaPoint::Length(*line.geom_adaptor()), 5, 1e-9);
    const Wire wire = Polyline({{0, 0}, {1, 0}, {1, 2}});
    CHECK_NEAR(GCPnts_AbscissaPoint::Length(*wire.geom_adaptor()), 3, 1e-9);
    CHECK(!Handle(Geom_Plane)::DownCast(Rectangle(1, 2).face().geom_adaptor()).IsNull());
    CHECK_THROWS(Edge().geom_adaptor(), std::invalid_argument);
    CHECK_THROWS(Wire().geom_adaptor(), std::invalid_argument);
}

TEST(dxf_metric_units) {
    CHECK(ExportDXF::METRIC_UNITS.count(Unit::MM) == 1);
    CHECK(ExportDXF::METRIC_UNITS.count(Unit::IN) == 0);
    CHECK(ExportDXF::METRIC_UNITS.size() == 3);
}
