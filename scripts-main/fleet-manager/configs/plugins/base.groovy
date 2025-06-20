def external_method = { hello ->
    println "calling external method: $hello"
}

def main_closure = { drone ->
    println "calling main closure"
    external_method(bar)
    println "drone: ${drone.id}"
    println "default value: $deft"
    println "foo: $foo"
    println "count: $count"
    count += foo
    println "new count: $count"
}

plugin {
    id 'base_plugin'
    type  monitor
    input foo: int,
          bar: String,
          deft: [String, "default value"]
    vars  count: 0
    callback main_closure
}
