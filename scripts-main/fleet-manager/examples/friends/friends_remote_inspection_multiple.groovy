/*
 * friends_remote_inspection_multiple.groovy
 * Any two drones can perform this mission, as long as the corresponding drone-side FRIENDS service is running.
 * Start remote inspection task, dividing the area of interest into two clusters, one for each drone.
 */

aoi = [[40.6334471, -8.66047],
       [40.6334586, -8.6605382],
       [40.633457, -8.6606345],
       [40.6336325, -8.6606869],
       [40.6336325, -8.6606869],
       [40.6337822, -8.6607092]]
clusters = k_means(2, aoi)

droneA = assign lat: clusters[0][0][0], lon: clusters[0][0][1]
droneB = assign lat: clusters[1][0][0], lon: clusters[1][0][1]

drone_params = [dmax_speed: 5,
                dmin_bat: 25]
sensor_params = [sdist: 0, sgrab_time: 30]

taskA = run { launch_task(droneA, clusters[0]) }
taskB = run { launch_task(droneB, clusters[1]) }
wait tasks: [taskA, taskB]

def launch_task(drone, aoi) {
    def mission_params = [mpath_algo: 'fast',
                      mheight: drone.position.alt + 7,
                      maoi: aoi,
                      mstart: drone.position,
                      mstop: drone.position,
                      mtype: 'inspection']
    def inspect = start inspection with drone: drone_params, mission: mission_params, sensor: sensor_params using drone
    wait task: inspect
    land drone
}

def k_means(double k, List aoi) {
    List init_centroids = [[]] * k
    k.times { i -> init_centroids[i] = aoi[i] }
    List cluster = calc_clusters(k, aoi, init_centroids)
    while (true) {
        List new_centroids = [[]] * k
        k.times { i -> new_centroids[i] = centroid(cluster[i])}
        if (new_centroids == init_centroids)
            break
        init_centroids = new_centroids
        cluster = calc_clusters(k, aoi, init_centroids)
    }
    return cluster
}

def calc_clusters(double k, List aoi, List centroids) {
    List cluster = [[]] * k
    aoi.each { coord ->
        double shortest_dist = Double.MAX_VALUE
        int shortest_idx = 0
        k.times {i ->
            double curr_dist = distance([lat: coord[0], lon: coord[1]], [lat: centroids[i][0], lon: centroids[i][1]]).amount
            if (curr_dist < shortest_dist) {
                shortest_dist = curr_dist
                shortest_idx = i
            }
        }
        cluster[shortest_idx] += [coord]
    }
    return cluster
}

def centroid(List coords) {
    double c_lat = 0, c_lon = 0
    coords.each { coord ->
        c_lat += coord[0]
        c_lon += coord[1]
    }
    return [c_lat / coords.size(), c_lon / coords.size()]
}