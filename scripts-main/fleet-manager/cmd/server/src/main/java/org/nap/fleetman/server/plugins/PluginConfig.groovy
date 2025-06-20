package org.nap.fleetman.server.plugins

class PluginConfig {
    String id
    PluginType type
    Map<String, Object> input = [:]
    Map<String, Object> vars = [:]
    Closure callback
    Closure init

    void id(String id) {
        this.id = id
    }

    void type(PluginType type) {
        this.type = type
    }

    void input(Map input) {
        this.input = input
    }

    void vars(Map vars) {
        this.vars = vars
    }

    void callback(Closure callback) {
        this.callback = callback
    }

    void init(Closure init) {
        this.init = init
    }

    Map<String, Object> getVariables(Map args) {
        def variables = args + vars
        input.findAll {var, _ -> !args.containsKey(var) }.each {var, value  ->
            variables.put(var, value[1])
        }
        variables
    }

    @Override
    String toString() {
        return "plugin id: $id"
    }
}
