#ifndef FLEETMAN_GEOFENCE_HPP
#define FLEETMAN_GEOFENCE_HPP

/* For geometry operations */
#include <geos/geom/GeometryFactory.h>
#include <geos/geom/Geometry.h>

/* For WKT read/write */
#include <geos/io/WKTReader.h>
#include <geos/io/WKTWriter.h>

/* Geometry/GeometryFactory */
using namespace geos::geom;

/* WKTReader/WKTWriter */
using namespace geos::io;

class GeoFence {
	public:
		GeoFence();
		bool inside_fence(std::string lat, std::string lon);
		std::unique_ptr<Geometry> geofence;
		std::unique_ptr<Coordinate> center;

	private:
		GeometryFactory::Ptr factory;
		WKTReader reader;

		void load_geofence();
		std::string parse_wkt_file(std::string filename);
		bool isPolygonClosed(const Polygon* polygon);
};

#endif //FLEETMAN_GEOFENCE_HPP