# Based on https://github.com/ros2/rosbag2/blob/master/rosbag2_py/test/test_sequential_reader.py
import argparse
import collections
import datetime
import json
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import rosbag2_py
import sys
from gmplot import gmplot
from haversine import haversine, Unit
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def read_bags(paths, start_time, end_time):
    gs_coords = (40.63333359555946, -8.660312933450562)
    coords = []
    gs_distance = []
    waypoints = []
    altitude = []
    finished = False
    last_timestamp = math.floor(start_time/1000) - 1
    bad = []

    for i in range(len(paths)):
        if finished:
            break

        if i < len(paths) - 1:
            bag_timestamp = int(datetime.datetime.strptime(paths[i+1].split('rosbag2_')[1], "%Y_%m_%d-%H_%M_%S").timestamp() * 1000)
            if bag_timestamp < start_time:
                continue

        storage_options, converter_options = get_rosbag_options(paths[i])
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            if isinstance(msg, String):
                data = json.loads(msg.data)
                if topic == '/telem':
                    if data['timestamp'] >= start_time:
                        if data['timestamp'] > end_time:
                            finished = True
                            print("Finished capturing mission data")
                            break
                        time_diff = math.floor(data['timestamp']/1000) - last_timestamp
                        dist_to_gs = haversine(gs_coords, (data['position']['lat'], data['position']['lon']), unit=Unit.METERS)
                        if time_diff > 1:
                            prev_dist = gs_distance[-1]
                            dst = (dist_to_gs - prev_dist)/(time_diff+1)
                            for i in range(time_diff):
                                gs_distance.append(prev_dist+(dst*(i+1)))
                                bad.append(gs_distance[-1])
                        if time_diff > 0:
                            coords.append((data['position']['lat'], data['position']['lon']))
                            gs_distance.append(dist_to_gs)
                            last_timestamp = math.floor(data['timestamp']/1000)
                            bad.append(None)
                        altitude.append(data['position']['alt'])
                elif topic == '/status' and 'command' in data:
                    if data['command'] == 'goto' or data['command'] == 'move':
                        waypoints.append((data['coords']['lat'], data['coords']['lon']))

    return coords, gs_distance, bad, waypoints, altitude


def get_base_filename(output_dir, mission_id):
    if output_dir is None:
        filename = mission_id
    else:
        filename = output_dir + "/" + mission_id
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
    return filename


def plot_gs_distance(distance, bad, output_dir):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    print("Plotting ground station distance")
    time = np.linspace(0, len(distance), len(distance))
    plt.plot(time, distance, color=color['drone03'], label='telemetry data')
    plt.plot(time, bad, color='red', label='interpolated data')
    plt.xlabel('time (s)')
    plt.ylabel('distance to ground station (m)')
    plt.locator_params(axis="x", nbins=8)
    plt.legend(bbox_to_anchor=(0.1,1), loc="lower left", ncol=2)
    filename = get_base_filename(output_dir, 'single') + "_ground_dist.pdf"
    plt.savefig(filename, bbox_inches="tight")
    plt.clf()


def plot_drone_altitude(altitude, output_dir, start):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    print("Plotting drone altitude")
    time = np.linspace(0, len(altitude) / 5, len(altitude))
    plt.plot(time, altitude, color=color['drone01']) #, label=drone_id)
    plt.xlabel('time (s)')
    plt.ylabel('altitude above mean sea level (m)')
    #plt.legend(bbox_to_anchor=(0.25,1), loc="lower left", ncol=2)
    filename = get_base_filename(output_dir, start) + "_altitude.pdf"
    plt.savefig(filename, bbox_inches="tight")
    plt.clf()


def draw_coords(coords, api_key, output_dir, waypoints, start):
    # drone01: yellow, drone02: green, drone03: blue, drone04: red
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    print("Drawing coordinates")
    gmap = gmplot.GoogleMapPlotter(40.633931554120295, -8.660562817044159, 21, map_type='hybrid')
    gmap.apikey = api_key
    drone_coords = zip(*coords)
    gmap.plot(*drone_coords, color=color['drone01'], edge_width=4)
    #gmap.circle(0.633874, -8.660311, 1.4, edge_color="#222222", fc="#ffffff", edge_alpha=0.9, face_alpha=1, edge_width=4)
    for coord in waypoints:
        gmap.circle(coord[0], coord[1], 0.8, edge_color="#222222", fc=color['drone01'], edge_alpha=0.9,
                    face_alpha=1, edge_width=4)
    filename = get_base_filename(output_dir, 'coords_' + start) + ".html"
    gmap.draw(filename)
    print("Exported", filename)


def plot_iperf(iperf_file, output_dir):
    bitrate = []
    with open(iperf_file) as file:
        for line in file:
            line = line.rstrip()
            cols = line.split()
            bitrate.append(float(cols[6]))
    bitrate = bitrate[:-27]
    print(len(bitrate))
    x = range(0, len(bitrate))
    plt.plot(x, bitrate)
    plt.xlabel('time (s)')
    plt.ylabel('bitrate (Kbps)')
    plt.locator_params(axis="x", nbins=8)
    plt.locator_params(axis="y", nbins=10)
    filename = get_base_filename(output_dir, 'single') + "_iperf.pdf"
    plt.savefig(filename, bbox_inches="tight")
    plt.clf()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--bags', required=True, nargs='+')
    parser.add_argument('-o', '--outputdir', default=None)
    parser.add_argument('-k', '--apikey', default='')
    parser.add_argument('-s', '--start', required=True)
    parser.add_argument('-e', '--end', required=True)
    parser.add_argument('-i', '--iperf', default=None)
    cli_args = parser.parse_args()

    coords, gs_distance, bad, waypoints, altitude = read_bags(cli_args.bags, int(cli_args.start), int(cli_args.end))
    plot_drone_altitude(altitude, cli_args.outputdir, cli_args.start)
    plot_gs_distance(gs_distance, bad, cli_args.outputdir)
    draw_coords(coords, cli_args.apikey, cli_args.outputdir, waypoints, cli_args.start)
    if cli_args.iperf is not None:
        plot_iperf(cli_args.iperf, cli_args.outputdir)


if __name__ == '__main__':
    main()
