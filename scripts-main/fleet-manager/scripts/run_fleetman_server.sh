#!/bin/bash
if [[ -v RECORD_BAGS ]] && $RECORD_BAGS ; then
  cd fleetman_logs/ && ros2 bag record -a &
fi
java -cp install/server/share/server/java/server-1.10-SNAPSHOT.jar:/opt/jardeps/* org.nap.fleetman.server.FleetmanServerApplication