# Based on https://github.com/ros2/rosbag2/blob/master/rosbag2_py/test/test_sequential_reader.py
import argparse
import collections
import datetime
import json
import matplotlib.pyplot as plt
import math
import numpy as np
import os
import rosbag2_py
import sys
from gmplot import gmplot
from haversine import haversine, Unit
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from drone_interfaces.msg import MissionStatus
from std_msgs.msg import String

if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def read_bags(paths, missions, gs_coords, relay_drones, track_progress):
    mission_iter = iter(missions)
    current_mission = next(mission_iter)
    started_next_mission = False
    coords = dict()
    waypoints = dict()
    temperature = dict()
    gs_distance = dict()
    other_distance = dict()
    last_pos = dict()
    altitude = dict()
    height = dict()
    battery = dict()
    speed = dict()
    heading = dict()
    req_battery = dict()
    home_dist = dict()
    progress = dict()
    time = dict()
    save_next_waypoint = []
    finished = False

    for i in range(len(paths)):
        if finished:
            break

        if i < len(paths) - 1:
            next_bag_timestamp = int(
                datetime.datetime.strptime(paths[i + 1].split('rosbag2_')[1], "%Y_%m_%d-%H_%M_%S").timestamp() * 1000)
            if next_bag_timestamp < missions[current_mission]['start_time']:
                continue

        current_bag_timestamp = int(
            datetime.datetime.strptime(paths[i].split('rosbag2_')[1].replace("/", ""), "%Y_%m_%d-%H_%M_%S").timestamp() * 1000)
        while missions[current_mission]['finish_time'] < current_bag_timestamp:
            print("Mission " + current_mission + " finished before provided data")
            try:
                current_mission = next(mission_iter)
            except StopIteration:
                print("ROS2 bags do not contain data belonging to the provided missions")
                sys.exit()

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
                    last_pos[data['droneId']] = (data['position']['lat'], data['position']['lon'])
                    if data['droneId'] in missions[current_mission]['drones'] and data['position']['alt'] is not None:
                        if not started_next_mission and data['timestamp'] > missions[current_mission]['start_time']:
                            print("Start capturing mission", current_mission)
                            started_next_mission = True
                            coords[current_mission] = dict()
                            waypoints[current_mission] = dict()
                            gs_distance[current_mission] = dict()
                            height[current_mission] = dict()
                            altitude[current_mission] = dict()
                            speed[current_mission] = dict()
                            heading[current_mission] = dict()
                            battery[current_mission] = dict()
                            req_battery[current_mission] = dict()
                            home_dist[current_mission] = dict()
                            time[current_mission] = dict()
                            if len(relay_drones) > 0:
                                other_distance[current_mission] = dict()
                                if data['droneId'] != missions[current_mission]['drones'][0] and data['droneId'] in relay_drones:
                                    other_distance[current_mission][data['droneId']] = [get_drone_dist(data, last_pos, [missions[current_mission]['drones'][0]] + relay_drones)]
                            if 'height' in data:
                                height[current_mission][data['droneId']] = [data['height']]
                            altitude[current_mission][data['droneId']] = [data['position']['alt']]
                            speed[current_mission][data['droneId']] = [data['speed']]
                            heading[current_mission][data['droneId']] = [data['heading']]
                            battery[current_mission][data['droneId']] = [data['battery']['remaining_percent']]
                            home_dist[current_mission][data['droneId']] = [calc_home_dist(data)]
                            time[current_mission][data['droneId']] = [(data['timestamp'] - missions[current_mission]['start_time'])/1000.0]
                            req_battery[current_mission][data['droneId']] = [
                                0.0015 * home_dist[current_mission][data['droneId']][-1] + 0.27]
                            coords[current_mission][data['droneId']] = [
                                (data['position']['lat'], data['position']['lon'])]
                            if gs_coords is not None:
                                gs_distance[current_mission][data['droneId']] = [
                                    haversine(gs_coords, (data['position']['lat'], data['position']['lon']),
                                              unit=Unit.METERS)]
                        elif started_next_mission:
                            if data['timestamp'] > missions[current_mission]['finish_time']:
                                print("Stop capturing mission", current_mission)
                                started_next_mission = False
                                try:
                                    current_mission = next(mission_iter)
                                except StopIteration:
                                    finished = True
                                    print("Finished capturing mission data")
                                    break
                            else:
                                if data['droneId'] not in coords[current_mission]:
                                    if 'height' in data:
                                        height[current_mission][data['droneId']] = []
                                    coords[current_mission][data['droneId']] = []
                                    gs_distance[current_mission][data['droneId']] = []
                                    altitude[current_mission][data['droneId']] = []
                                    speed[current_mission][data['droneId']] = []
                                    heading[current_mission][data['droneId']] = []
                                    home_dist[current_mission][data['droneId']] = []
                                    time[current_mission][data['droneId']] = []
                                    battery[current_mission][data['droneId']] = []
                                    req_battery[current_mission][data['droneId']] = []
                                coords[current_mission][data['droneId']].append(
                                    (data['position']['lat'], data['position']['lon']))
                                if gs_coords is not None:
                                    gs_distance[current_mission][data['droneId']].append(
                                        haversine(gs_coords, (data['position']['lat'], data['position']['lon']),
                                                  unit=Unit.METERS))
                                if 'height' in data:
                                    height[current_mission][data['droneId']].append(data['height'])
                                altitude[current_mission][data['droneId']].append(data['position']['alt'])
                                speed[current_mission][data['droneId']].append(data['speed'])
                                heading[current_mission][data['droneId']].append(data['heading'])
                                battery[current_mission][data['droneId']].append(data['battery']['remaining_percent'])
                                home_dist[current_mission][data['droneId']].append(calc_home_dist(data))
                                time[current_mission][data['droneId']].append((data['timestamp'] - missions[current_mission]['start_time'])/1000.0)
                                req_battery[current_mission][data['droneId']].append(
                                    0.0015 * home_dist[current_mission][data['droneId']][-1] + 0.27)
                                if len(relay_drones) > 0 and data['droneId'] != missions[current_mission]['drones'][0] and data['droneId'] in relay_drones:
                                    if data['droneId'] not in other_distance[current_mission]:
                                        other_distance[current_mission][data['droneId']] = []
                                    other_distance[current_mission][data['droneId']].append(
                                        get_drone_dist(data, last_pos, [missions[current_mission]['drones'][0]] + relay_drones))
                elif started_next_mission:
                    if topic == '/cmd':
                        if data['cmd'] == 'goto' or data['cmd'] == 'move':
                            save_next_waypoint.append(data['droneId'])
                    elif topic == '/status':
                        if data['droneId'] in save_next_waypoint and data['state'] == 'start' and data[
                            'command'] == 'goto':
                            save_next_waypoint.remove(data['droneId'])
                            if data['droneId'] not in waypoints[current_mission]:
                                waypoints[current_mission][data['droneId']] = []
                            waypoints[current_mission][data['droneId']].append(
                                (data['coords']['lat'], data['coords']['lon']))
                    elif topic == "/sensors":
                        if data['type'] == 'temperature':
                            if current_mission not in temperature:
                                temperature[current_mission] = []
                            temperature[current_mission].append(data['value'])
            elif track_progress and isinstance(msg, MissionStatus):
                if msg.drone_id in missions[current_mission]['drones'] and msg.type == 'progress':
                    timestamp = int(str(msg.header.stamp.sec) + str(msg.header.stamp.nanosec)[:3])
                    if not current_mission in progress and timestamp > missions[current_mission]['start_time']:
                        progress[current_mission] = {'timestamp': [(timestamp - missions[current_mission]['start_time'])/1000.0], 'val': [float(msg.message)], 'task': msg.task}
                    elif started_next_mission:
                        progress[current_mission]['timestamp'].append((timestamp - missions[current_mission]['start_time'])/1000.0)
                        progress[current_mission]['val'].append(float(msg.message))

    waypoints_clean = waypoints.copy()
    for missionId in waypoints:
        if len(waypoints[missionId]) == 0:
            del waypoints_clean[missionId]
            del coords[missionId]
            temperature.pop(missionId, None)
            del gs_distance[missionId]
            other_distance.pop(missionId, None)
            altitude.pop(missionId, None)
            height.pop(missionId, None)
            speed.pop(missionId, None)
            heading.pop(missionId, None)
            battery.pop(missionId, None)
            req_battery.pop(missionId, None)
            home_dist.pop(missionId, None)
            time.pop(missionId, None)
    return coords, waypoints, temperature, gs_distance, other_distance, height, altitude, speed, heading, battery, req_battery, home_dist, progress, time


def calc_home_dist(data):
    lat_lon_dist = haversine((data['home']['lat'], data['position']['lon']),
                             (data['position']['lat'], data['position']['lon']), unit=Unit.METERS)
    return math.sqrt(math.pow(lat_lon_dist, 2) + math.pow(data['position']['alt'] - data['home']['alt'], 2))


def get_drone_dist(data, last_pos, drones):
    prev_drone_id = drones[drones.index(data['droneId']) - 1]
    return haversine(last_pos[prev_drone_id], (data['position']['lat'], data['position']['lon']), unit=Unit.METERS)


def process_gs_logs(logs, allowed_missions):
    missions = collections.OrderedDict()
    for log_file in logs:
        with open(log_file) as log:
            for line in log:
                line = line.rstrip()
                if 'Mission' in line:
                    missionId = line.replace("\"", "'").split("'")[1]
                    if missionId in allowed_missions or len(allowed_missions) == 0:
                        if 'started' in line:
                            if missionId not in missions:
                                missions[missionId] = {'drones': []}
                            timestamp = line.split(' ')[0] + " " + line.split(' ')[1]
                            try:
                                missions[missionId]['start_time'] = int(
                                    datetime.datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S.%f").timestamp() * 1000)
                            except ValueError:
                                missions[missionId]['start_time'] = int(
                                    datetime.datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S,%f").timestamp() * 1000)
                        if 'concluded' in line:
                            if len(missions[missionId]['drones']) == 0:
                                del missions[missionId]
                            else:
                                timestamp = line.split(' ')[0] + " " + line.split(' ')[1]
                                try:
                                    missions[missionId]['finish_time'] = int(
                                        datetime.datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S.%f").timestamp() * 1000)
                                except ValueError:
                                    missions[missionId]['finish_time'] = int(
                                        datetime.datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S,%f").timestamp() * 1000)
                elif 'Assigned' in line:
                    missionId = line.split("\"")[-2]
                    droneId = line.split("-")[-2].strip()
                    if (missionId in allowed_missions or len(allowed_missions) == 0) and droneId not in \
                            missions[missionId]['drones']:
                        missions[missionId]['drones'].append(droneId)

    missions_clean = missions.copy()
    for missionId in missions.keys():
        if 'start_time' not in missions[missionId] or 'finish_time' not in missions[missionId]:
            del missions_clean[missionId]
    print("Process missions:", list(missions_clean.keys()))
    return missions_clean


def get_base_filename(output_dir, mission_id):
    if output_dir is None:
        filename = mission_id
    else:
        filename = output_dir + "/" + mission_id
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
    return filename


def plot_temperature(temperature, output_dir):
    for missionId in temperature:
        print("Plotting temperature for mission", missionId)
        time = np.linspace(0, len(temperature[missionId]) / 2, len(temperature[missionId]))
        plt.plot(time, temperature[missionId], color='orange', label='drone01 temperature')
        plt.hlines(40, 0, len(temperature[missionId]) / 2, 'r', 'dashed', 'target temperature')
        plt.xlabel('time (s)')
        plt.ylabel('temperature (°C)')
        plt.legend(loc="lower center")
        filename = get_base_filename(output_dir, missionId) + "_temperature.pdf"
        plt.savefig(filename)
        plt.clf()


def plot_drone_height(height, output_dir, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in height:
        print("Plotting drone height for mission", missionId)
        for drone_id in sorted(height[missionId].keys(), key=lambda x: x.lower()):
            plt.plot(time[missionId][drone_id], height[missionId][drone_id], color=color[drone_id], label=drone_id)
        plt.xlabel('time (s)')
        plt.ylabel('height to ground (mm)')
        if len(height[missionId]) > 1:
            plt.legend(bbox_to_anchor=(0.25, 1), loc="lower left", ncol=2)
        filename = get_base_filename(output_dir, missionId) + "_height.pdf"
        plt.savefig(filename, bbox_inches="tight")
        plt.clf()


def plot_drone_altitude(altitude, output_dir, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in altitude:
        print("Plotting drone altitude for mission", missionId)
        for drone_id in sorted(altitude[missionId].keys(), key=lambda x: x.lower()):
            plt.plot(time[missionId][drone_id], altitude[missionId][drone_id], color=color[drone_id], label=drone_id)
        plt.xlabel('time (s)')
        plt.ylabel('altitude above mean sea level (m)')
        if len(altitude[missionId]) > 1:
            plt.legend(bbox_to_anchor=(0.25, 1), loc="lower left", ncol=2)
        filename = get_base_filename(output_dir, missionId) + "_altitude.pdf"
        plt.savefig(filename, bbox_inches="tight")
        plt.clf()


def plot_drone_speed(speed, output_dir, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in speed:
        print("Plotting drone speed for mission", missionId)
        for drone_id in sorted(speed[missionId].keys(), key=lambda x: x.lower()):
            plt.plot(time[missionId][drone_id], speed[missionId][drone_id], color=color[drone_id], label=drone_id)
        plt.xlabel('time (s)')
        plt.ylabel('speed (m/s)')
        if len(speed[missionId]) > 1:
            plt.legend(bbox_to_anchor=(0.25, 1), loc="lower left", ncol=2)
        filename = get_base_filename(output_dir, missionId) + "_speed.pdf"
        plt.savefig(filename, bbox_inches="tight")
        plt.clf()


def plot_drone_heading(heading, output_dir, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in heading:
        print("Plotting drone heading for mission", missionId)
        for drone_id in sorted(heading[missionId].keys(), key=lambda x: x.lower()):
            plt.plot(time[missionId][drone_id], heading[missionId][drone_id], color=color[drone_id], label=drone_id)
        plt.xlabel('time (s)')
        plt.ylabel('heading (°)')
        if len(heading[missionId]) > 1:
            plt.legend(bbox_to_anchor=(0.25, 1), loc="lower left", ncol=2)
        filename = get_base_filename(output_dir, missionId) + "_heading.pdf"
        plt.savefig(filename, bbox_inches="tight")
        plt.clf()


def plot_drone_req_battery(battery, req_battery, output_dir, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    color_req = {'drone01': '#ffdd8a', 'drone02': '#a4e0d8', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in battery:
        if len(battery[missionId]) > 1:
            print("Plotting drone battery for mission", missionId)
            for drone_id in sorted(battery[missionId].keys(), key=lambda x: x.lower()):
                plt.plot(time[missionId][drone_id], battery[missionId][drone_id], color=color[drone_id], label=drone_id + " battery")
                plt.plot(time[missionId][drone_id], req_battery[missionId][drone_id], color=color_req[drone_id],
                         label=drone_id + " required battery")
            plt.xlabel('time (s)')
            plt.vlines(112, 0, 1.05, 'gray', 'dashed', linewidth=1)
            plt.vlines(207, 0, 1.05, 'gray', 'dashed', linewidth=1)
            plt.ylim(bottom=0.15)
            plt.legend(bbox_to_anchor=(0, 1), loc="lower left", ncol=2)
            filename = get_base_filename(output_dir, missionId) + "_req_battery.pdf"
            plt.savefig(filename, bbox_inches="tight")
            plt.clf()


def plot_gs_distance(distance, output_dir, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in distance:
        if len(distance[missionId]) > 1:
            print("Plotting ground station distance for mission", missionId)
            for drone_id in sorted(distance[missionId].keys(), key=lambda x: x.lower()):
                plt.plot(time[missionId][drone_id], distance[missionId][drone_id], color=color[drone_id], label=drone_id)
            plt.xlabel('time (s)')
            plt.ylabel('distance to ground station (m)')
            plt.legend(bbox_to_anchor=(0.25, 1), loc="lower left", ncol=2)
            filename = get_base_filename(output_dir, missionId) + "_ground_dist.pdf"
            plt.savefig(filename, bbox_inches="tight")
            plt.clf()


def plot_home_distance(distance, output_dir, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in distance:
        if len(distance[missionId]) > 1:
            print("Plotting home distance for mission", missionId)
            for drone_id in sorted(distance[missionId].keys(), key=lambda x: x.lower()):
                plt.plot(time[missionId][drone_id], distance[missionId][drone_id], color=color[drone_id], label=drone_id)
            plt.xlabel('time (s)')
            plt.ylabel('distance to home position (m)')
            plt.legend(bbox_to_anchor=(0.25, 1), loc="lower left", ncol=2)
            filename = get_base_filename(output_dir, missionId) + "_home_dist.pdf"
            plt.savefig(filename, bbox_inches="tight")
            plt.clf()


def plot_progress(progress, output_dir):
    for missionId in progress:
        print("Plotting task progress for mission", missionId)
        plt.plot(progress[missionId]['timestamp'], progress[missionId]['val'], color='#FBC02D')
        plt.xlabel('time (s)')
        plt.ylabel(progress[missionId]['task'] + ' progress')
        plt.yticks([0,10,20,30,40,50,60,70,80,90,100])
        filename = get_base_filename(output_dir, missionId) + "_progress.pdf"
        plt.savefig(filename, bbox_inches="tight")
        plt.clf()


def plot_drone_distance(distance, output_dir, drones, target_dist, time):
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for missionId in distance:
        if len(drones) - 1 > len(distance[missionId].keys()):
            print("Skip plotting drone distance for mission", missionId)
            continue
        print("Plotting drone distance for mission", missionId)
        # FIXME should provide full list of drones in order
        drones = ['drone01'] + drones
        for drone_id in sorted(distance[missionId].keys(), key=lambda x: x.lower()):
            prev_drone_id = drones[drones.index(drone_id) - 1]
            plt.plot(time[missionId][drone_id], distance[missionId][drone_id], color=color[drone_id],
                     label="distance " + drone_id + " - " + prev_drone_id)
        plt.hlines(target_dist, 0, time[missionId][list(time[missionId].keys())[0]][-1], 'black', 'dashed', 'target distance')
        plt.xlabel('time (s)')
        plt.ylabel('distance (m)')
        plt.legend(bbox_to_anchor=(-0.02, 1), loc="lower left", ncol=2)
        filename = get_base_filename(output_dir, missionId) + "_drone_dist.pdf"
        plt.savefig(filename, bbox_inches="tight")
        plt.clf()

def calc_centroid(coords):
    num_coords = coords.shape[0]
    sum_lat = np.sum(coords[:,0])
    sum_lon = np.sum(coords[:,1])
    return sum_lat/num_coords, sum_lon/num_coords

def draw_coords(coords, waypoints, groundstation, api_key, output_dir):
    # drone01: yellow, drone02: green, drone03: blue, drone04: red
    color = {'drone01': '#FBC02D', 'drone02': '#00BFA5', 'drone03': '#00B0FF', 'drone04': '#FF5733'}
    for mission in coords:
        print("Drawing coordinates for mission", mission)
        centroid = calc_centroid(np.array(list(coords[mission].values())[0]))
        gmap = gmplot.GoogleMapPlotter(centroid[0], centroid[1], 20, map_type='hybrid')
        gmap.apikey = api_key
        for drone in coords[mission]:
            drone_coords = zip(*coords[mission][drone])
            gmap.plot(*drone_coords, color=color[drone], edge_width=4)
        for drone in waypoints[mission]:
            for coord in waypoints[mission][drone]:
                gmap.circle(coord[0], coord[1], 0.8, edge_color="#222222", fc=color[drone], edge_alpha=0.9,
                            face_alpha=1, edge_width=4)
        if groundstation is not None:
            gmap.circle(groundstation[0], groundstation[1], 1.4, edge_color="#222222", fc="#ffffff", edge_alpha=0.9,
                        face_alpha=1, edge_width=4)
        filename = get_base_filename(output_dir, mission) + ".html"
        gmap.draw(filename)
        print("Exported", filename)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--bags', required=True, nargs='+')
    parser.add_argument('-l', '--logs', required=True, nargs='+')
    parser.add_argument('-o', '--outputdir', default=None)
    parser.add_argument('-k', '--apikey', default='')
    parser.add_argument('-m', '--missions', nargs='+', default=[], type=str)
    parser.add_argument('-g', '--groundstation', default=None, nargs='+', type=float)
    parser.add_argument('-d', '--dronesrelay', default=[], nargs='+', type=str) # TODO use r for relay
    parser.add_argument('-t', '--targetdist', default=50, type=int) # TODO use d for distance
    parser.add_argument('-p', '--progress', default=False, action='store_true')

    cli_args = parser.parse_args()

    missions = process_gs_logs(cli_args.logs, cli_args.missions)
    if len(missions) == 0:
        print("No valid missions found in log file.")
        exit()

    coords, waypoints, temperature, gs_distance, other_distance, height, altitude, speed, heading, battery, req_battery,\
        home_dist, progress, time = read_bags(cli_args.bags, missions, cli_args.groundstation, cli_args.dronesrelay, cli_args.progress)

    plot_temperature(temperature, cli_args.outputdir)
    plot_drone_height(height, cli_args.outputdir, time)
    plot_drone_altitude(altitude, cli_args.outputdir, time)
    plot_drone_speed(speed, cli_args.outputdir, time)
    plot_drone_heading(heading, cli_args.outputdir, time)
    plot_drone_req_battery(battery, req_battery, cli_args.outputdir, time)
    plot_home_distance(home_dist, cli_args.outputdir, time)
    if cli_args.progress:
        plot_progress(progress, cli_args.outputdir)
    if cli_args.groundstation is not None:
        plot_gs_distance(gs_distance, cli_args.outputdir, time)
        if len(cli_args.dronesrelay) > 0:
            plot_drone_distance(other_distance, cli_args.outputdir,  cli_args.dronesrelay, cli_args.targetdist, time)
    draw_coords(coords, waypoints, cli_args.groundstation, cli_args.apikey, cli_args.outputdir)


if __name__ == '__main__':
    main()
