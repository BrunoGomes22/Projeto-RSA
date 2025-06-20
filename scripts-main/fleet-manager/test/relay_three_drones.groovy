/*
 * relay_three_drones.groovy
 * Three drones are required for this mission.
 * Send one drone to a predefined set of coordinates.
 */

// Groundstation location
gs_coords_ieeta = [lat: 40.63319191970333, lon: -8.66055184994151]
gs_coords_library = [lat: 40.63086901310225, lon:-8.661135013183321]

// Coordinates to traverse
coords_110_ieeta = [[40.633406025480824, -8.660589393099743],
              [40.633558688274825, -8.66076641889473],
              [40.63380498351332, -8.66045260043998],
              [40.634057384302736, -8.660715456923446],
              [40.63423447138381, -8.66046869369407], // Furthest coord
              [40.63361568229512, -8.660876389464343],
              [40.63334292475734, -8.660578664263682]]
coords_150_library = [[40.63059638574363, -8.660991300429878],
                      [40.63055402277373, -8.660559095802846],
                      [40.63030710664297, -8.66061172588658],
                      [40.63031436889515, -8.660230556492259],
                      [40.630090449089906, -8.660390041594486],
                      [40.630095290607166, -8.660007277349143],
                      [40.629798747023244, -8.659954647265414]]

enable 'relay', target_dist: 60.m, step:4.m, alt_diff: 3.m, base_alt: 7.m, gs_coords: gs_coords_library
drone = assign 'drone01'
arm drone
takeoff drone, 7.m
alt = drone.position.alt
coords_150_library.each {coord ->
    move drone at 3.m/s to lat: coord[0], lon: coord[1], alt: alt
}
home drone