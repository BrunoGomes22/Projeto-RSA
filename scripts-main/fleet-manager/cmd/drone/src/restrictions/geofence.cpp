#include "drone_core/restrictions/geofence.hpp"

#include <geos/geom/Point.h>
#include <fstream>

GeoFence::GeoFence() {
	this->factory = GeometryFactory::create();
	this->reader = WKTReader(*this->factory);

	this->load_geofence();

	// print center
	std::cout << "[GeoFence] Geofence center: " << this->center->y << ", " << this->center->x << std::endl;
}

bool GeoFence::inside_fence(std::string lat, std::string lon) {
	// Create point
	std::string wkt = "POINT(" + lat + " " + lon + ")";
	std::unique_ptr<Geometry> point(this->reader.read(wkt));

	// Check if point is inside geofence
	return this->geofence.get()->contains(point.get());
}

void GeoFence::load_geofence() {
	std::string wkt = this->parse_wkt_file("src/drone/geofence.wkt");
	this->geofence = std::unique_ptr<Geometry>(this->reader.read(wkt));
	this->center = std::make_unique<Coordinate>(*this->geofence->getCentroid()->getCoordinate());

	// print center
	std::cout << "[Load Geofence] Geofence center: " << this->center->y << ", " << this->center->x << std::endl;

	if (!this->geofence.get()->isValid()) {
		std::cerr << "Geofence is not valid!" << std::endl;
		exit(1);
	}

	std::string type = this->geofence->getGeometryType();
	if (type != "Polygon") {
		std::cerr << "Geofence must be a polygon." << std::endl;
		exit(1);\
	}

    const Polygon* polygon = dynamic_cast<const Polygon*>(this->geofence.get());
	if (!isPolygonClosed(polygon)) {
		std::cerr << "Polygon is not closed!" << std::endl;
		exit(1);
	}
}

std::string GeoFence::parse_wkt_file(std::string filename) {
	std::ifstream file(filename);
	std::string wkt;
	std::string line;
	while (std::getline(file, line)) {
		// remove tabs
        line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
		wkt += line + " ";
	}
	return wkt;
}

bool GeoFence::isPolygonClosed(const Polygon* polygon) {
    if (polygon) {
        const CoordinateSequence* coords = polygon->getExteriorRing()->getCoordinatesRO();
        const Coordinate& first = coords->getAt(0);
        const Coordinate& last = coords->getAt(coords->size() - 1);
        return (first.x == last.x && first.y == last.y);
    }
    return false;
}