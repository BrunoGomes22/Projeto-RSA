                                /*
                                * single_drone_to_location.groovy
                                * Sends one drone to a target location and returns home.
                                */

                                drone = assign 'drone01'
                                target = [lat: 40.6345, lon: -8.6608]
                                arm drone
                                takeoff drone, 5.meters
                                takeoff_alt = drone.position.alt
                                move drone, lat: target.lat, lon: target.lon, alt: takeoff_alt
                                home drone
                                