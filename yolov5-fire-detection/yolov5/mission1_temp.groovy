                            /*
                            * multi_waypoint_scout.groovy
                            * Sends one drone to multiple waypoints and returns home.
                            */

                            drone = assign 'drone01'
                            arm drone
                            takeoff drone, 5.meters
                            takeoff_alt = drone.position.alt

                            waypoints = [
                                [lat: 40.632240, lon: -8.660399],
[lat: 40.632240, lon: -8.660162],
[lat: 40.632240, lon: -8.659925],
[lat: 40.632420, lon: -8.660399],
[lat: 40.632420, lon: -8.659925],
[lat: 40.632600, lon: -8.660399],
[lat: 40.632600, lon: -8.660162],
[lat: 40.632600, lon: -8.659925]
                            ]

                            for (wp in waypoints) {
                                move drone, lat: wp.lat, lon: wp.lon, alt: takeoff_alt
                            }
                            home drone
