import argparse
import json
import math
import sys
import time
from threading import Thread

import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from haversine import haversine, Unit
from rclpy.node import Node
from std_msgs.msg import String


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', required=True)
    parser.add_argument('-e', '--export', dest='export', action='store_true')
    parser.add_argument('--no-export', dest='export', action='store_false')
    parser.add_argument('-s', '--silence', dest='silence', action='store_true')
    parser.add_argument('--no-silence', dest='silence', action='store_false')
    parser.set_defaults(export=False)
    parser.set_defaults(export=False)

    cli_args = parser.parse_args()
    return yaml.safe_load(open(cli_args.config)), cli_args.export, cli_args.silence


def current_time_millis():
    return int(round(time.time() * 1000))


def calc_temperature(dist, radius, low, high):
    relative_dist = (dist / radius) - 1  # equivalent to dist - radius / radius
    temp_range = high - low
    relative_temp = temp_range * relative_dist
    return low - relative_temp


def calc_coord(coords, dist, bearing):
    earth_radius = 6371000  # earth radius in meters
    bearing = math.radians(bearing)
    lat = math.radians(coords[0])
    lon = math.radians(coords[1])

    lat2 = math.asin(math.sin(lat) * math.cos(dist / earth_radius) +
                     math.cos(lat) * math.sin(dist / earth_radius) * math.cos(bearing))
    lon2 = lon + math.atan2(math.sin(bearing) * math.sin(dist / earth_radius) * math.cos(lat),
                            math.cos(dist / earth_radius) - math.sin(lat) * math.sin(lat2))

    return math.degrees(lat2), math.degrees(lon2)


class TemperatureSensor(Node):

    def __init__(self):
        self.rate = 1
        self.sensor_topic = '/sensor/temperature'
        self.telem_topic = '/telem'
        self.env_temp = 20
        self.coords = None

        cfg, export, self.silence = parse_args()

        if cfg is None:
            print('[ERROR] Provided config file is empty.')
            sys.exit()
        if cfg.get('droneId') is None:
            print('[ERROR] Provided config file does not contain drone ID.')
            sys.exit()
        if cfg.get('heatSources') is None:
            print('[ERROR] Provided config file does not contain heat sources.')
            sys.exit()

        self.drone_id = cfg.get('droneId')
        self.heat_sources = cfg.get('heatSources')
        for z in self.heat_sources:
            z['coords'] = (z.get('coords').get('lat'), z.get('coords').get('lon'))

        if cfg.get('sensorTopic') is not None:
            self.sensor_topic = cfg.get('sensorTopic')
        if cfg.get('telemTopic') is not None:
            self.telem_topic = cfg.get('telemTopic')
        if cfg.get('rate') is not None:
            self.rate = cfg.get('rate') / 1000
        if cfg.get('environmentTemp') is not None:
            self.env_temp = cfg.get('environmentTemp')

        super().__init__('temperature', namespace=self.drone_id + '/sensor')

        if export:
            dim = cfg.get('plotDimension')
            if dim is None:
                dim = 100
            self.generate_heatmap(dim)

        self.subscription = \
            self.create_subscription(String, self.telem_topic, lambda msg: self.drone_telem_callback(msg.data), 10)
        self.publisher = None
        self.timer = None

        thread = Thread(target=self.init_pub)
        thread.start()

    def init_pub(self):
        self.get_logger().info('Waiting for drone "%s" telem data...' % self.drone_id)
        i = 0
        while self.coords is None:
            if i > 20:
                self.get_logger().info(
                    'Timed out waiting for drone "%s" data. Is fleetman drone running? Exiting.' % self.drone_id)
                rclpy.shutdown()
                sys.exit()
            time.sleep(1)
            i += 1

        self.get_logger().info('Start temperature sensor for drone "%s"' % self.drone_id)
        self.publisher = self.create_publisher(String, self.sensor_topic, 10)
        self.timer = self.create_timer(self.rate, self.pub_temperature_callback)

    def pub_temperature_callback(self):
        msg = String()
        msg.data = json.dumps(
            {'droneId': self.drone_id, 'sensorId': 'simulated', 'type': 'temperature',
             'timestamp': current_time_millis(), 'value': self.get_temperature()})
        if not self.silence:
            self.get_logger().info(msg.data)
        self.publisher.publish(msg)

    def drone_telem_callback(self, msg):
        telem = json.loads(msg)
        if telem.get('droneId') == self.drone_id and telem.get('position').get('lat') is not None:
            self.coords = (telem.get('position').get('lat'), telem.get('position').get('lon'))

    def get_temperature(self, line=0, max_temp=0):
        temp = []
        for zone in self.heat_sources:
            dist = haversine(self.coords, zone.get('coords'), unit=Unit.METERS)
            if dist < zone.get('radius'):
                temp.append(calc_temperature(dist, zone.get('radius'), self.env_temp, zone.get('temp')))

        if line > 0:
            if len(temp) == 0:
                return self.env_temp, 0
            elif len(temp) == 1:
                if temp[0] > line-max_temp*0.01 and temp[0] < line + max_temp*0.01:
                    return temp[0], 50
                return temp[0], 0
            else:
                if max(temp) > line-max_temp*0.01 and max(temp) < line + max_temp*0.01:
                    return max(temp), 50
                return max(temp), 0

        if len(temp) == 0:
            return self.env_temp
        elif len(temp) == 1:
            return temp[0]
        else:
            return max(temp)

    def generate_heatmap(self, plot_dim):
        start_coord, plot_shape, lat_step_size, lon_step_size = self.calc_starting_coord_and_step(plot_dim)
        grid = np.zeros(shape=plot_shape)
        grid_line = np.zeros(shape=plot_shape)
        max_temp = max([zone['temp'] for zone in self.heat_sources])

        for i, r in enumerate(grid):
            for j, c in enumerate(r):
                self.coords = (start_coord[0] - (i * lat_step_size), start_coord[1] + (j * lon_step_size))
                grid[i, j], grid_line[i,j] = self.get_temperature(40, max_temp)

        ''' 
        fig, axes = plt.subplots(ncols=2, figsize=plot_shape)
        ax1, ax2 = axes
        col1 = grid
        col2 = grid_line
        im1 = ax1.matshow(grid, cmap='hot', interpolation='bicubic')
        im2 = ax2.matshow(grid_line, cmap='hot', interpolation='bicubic')
        '''
        plt.imshow(grid_line, cmap='viridis', interpolation='bicubic')
        plt.axis('off')
        #plt.colorbar(plt.pcolor(grid), label='temperature (°C)')
        img_file = 'plots/heatmap_' + str(current_time_millis()) + '.jpg'
        plt.savefig(img_file, dpi=200)
        self.get_logger().info('Exported heatmap to file "%s"' % img_file)
        self.coords = None

    def calc_starting_coord_and_step(self, n_points):
        collected_coords = list()
        for zone in self.heat_sources:
            dist = math.sqrt(3 * math.pow(zone.get('radius'), 2))
            collected_coords.append(calc_coord(zone.get('coords'), dist, 315))
            collected_coords.append(calc_coord(zone.get('coords'), dist, 45))
            collected_coords.append(calc_coord(zone.get('coords'), dist, 225))

        max_lat = max(coord[0] for coord in collected_coords)
        min_lat = min(coord[0] for coord in collected_coords)
        max_lon = max(coord[1] for coord in collected_coords)
        min_lon = min(coord[1] for coord in collected_coords)

        top_left_bound = (max_lat, min_lon)

        lat_diff = max_lat - min_lat
        lon_diff = max_lon - min_lon

        h = haversine((max_lat, max_lon), (min_lat, max_lon), unit=Unit.METERS)
        w = haversine((max_lat, max_lon), (max_lat, min_lon), unit=Unit.METERS)

        if h > w:
            w_points = n_points
            h_points = math.ceil(n_points * h / w)
        else:
            w_points = math.ceil(n_points * w / h)
            h_points = n_points

        lat_step_size = lat_diff / (h_points - 1)
        lon_step_size = lon_diff / (w_points - 1)

        return top_left_bound, (h_points, w_points), lat_step_size, lon_step_size


def main(args=None):
    rclpy.init(args=args)
    temp_sensor = TemperatureSensor()
    rclpy.spin(temp_sensor)

    if temp_sensor.coords is not None:
        temp_sensor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
