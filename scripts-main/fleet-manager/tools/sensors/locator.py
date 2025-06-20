import argparse
import json
import math
import random
import time
import paho.mqtt.client as mqtt


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--coords', dest='coords', nargs=3, required=True)
    parser.add_argument('-i', '--id', dest='id', required=True)
    parser.add_argument('-s', '--silence', dest='silence', action='store_true')
    parser.add_argument('--no-silence', dest='silence', action='store_false')
    parser.set_defaults(export=False)
    parser.set_defaults(export=False)

    cli_args = parser.parse_args()
    return cli_args.id, cli_args.coords, cli_args.silence


def current_time_millis():
    return int(round(time.time() * 1000))


def calc_coord(coordinates, dist, bearing):
    earth_radius = 6371000  # earth radius in meters
    bearing = math.radians(bearing)
    lat = math.radians(coordinates[0])
    lon = math.radians(coordinates[1])
    lat2 = math.asin(
        math.sin(lat) * math.cos(dist / earth_radius) + math.cos(lat) * math.sin(dist / earth_radius) * math.cos(
            bearing))
    lon2 = lon + math.atan2(math.sin(bearing) * math.sin(dist / earth_radius) * math.cos(lat),
                            math.cos(dist / earth_radius) - math.sin(lat) * math.sin(lat2))
    return [math.degrees(lat2), math.degrees(lon2)]


class LocatorSensor:
    def __init__(self):
        self.sensor_topic = 'sensors'
        self.id, self.coords, self.silence = parse_args()
        self.coords = list(map(float, self.coords))
        self.alt = self.coords[2]
        self.remaining_steps = 0
        self.bearing = 0
        self.step = 0

        print('Start locator sensor with id "%s"' % self.id)

        self.client = mqtt.Client(self.id)
        self.client.connect("localhost")
        self.publish_coord()

    def move(self):
        self.calc_next_coord()
        self.publish_coord()

    def calc_next_coord(self):
        if self.remaining_steps > 1:
            self.remaining_steps -= 1
        else:
            self.remaining_steps = random.randint(5, 21)
            self.bearing = random.randint(0, 360)
            self.step = 0
            if not self.silence:
                print("Moving " + str(self.remaining_steps) + " meters in " + str(self.bearing) + "º")
        self.step += 1
        self.coords = calc_coord(self.coords, 1, self.bearing)

    def publish_coord(self):
        msg = json.dumps(
            {'sensorId': self.id, 'type': 'locator', 'timestamp': current_time_millis(),
             'value': {'lat': self.coords[0], 'lon': self.coords[1], 'alt': self.alt}})
        if not self.silence:
            print("Step " + str(self.step) + ": " + msg)
        self.client.publish(self.sensor_topic, msg)


def main(args=None):
    locator_sensor = LocatorSensor()
    while True:
        locator_sensor.move()
        time.sleep(1)


if __name__ == '__main__':
    main()
