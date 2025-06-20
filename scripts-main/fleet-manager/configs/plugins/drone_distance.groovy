def init_gs = {
    groundstation.id = 'groundstation'
    groundstation.position = gs_coords
}

def main = { drones ->
    def nodes = [groundstation] + drones
    for (int i = 0; i < nodes.size-1; i++) {
        for (int j = i+1; j < nodes.size; j++) {
            println "${nodes[i].id} <-> ${nodes[j].id} = ${nodes[i].position.distance(nodes[j])}"
        }
    }
}


plugin {
    id      'distance'
    type    scheduled
    input   rate: Time,
            gs_coords: [Map, [lat: 40.633874, lon: -8.660311]]
    vars    groundstation: [:]
    callback main
    init    init_gs
}
