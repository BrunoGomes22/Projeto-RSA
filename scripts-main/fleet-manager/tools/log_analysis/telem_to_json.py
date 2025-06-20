# Based on https://github.com/ros2/rosbag2/blob/master/rosbag2_py/test/test_sequential_reader.py
import argparse
import json
import os
import rosbag2_py
import sys
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


def read_bags(paths, output_dir):
    out = dict()
    for bag_path in paths:
        storage_options, converter_options = get_rosbag_options(bag_path)
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)

            if isinstance(msg, String):
                telem = json.loads(msg.data)
                if topic == '/telem':
                    if telem['droneId'] not in out:
                        print("Detected", telem['droneId'])
                        out[telem['droneId']] = [telem]
                    else:
                        out[telem['droneId']].append(telem)
    for drone in out:
        filename = output_dir + "/" + drone + "_" + str(out[drone][0]['timestamp']) + ".json"
        with open(filename, 'w') as outfile:
            print("Writing to", filename)
            json.dump(out[drone], outfile)


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', '--bags', required=True, nargs='+')
    parser.add_argument('-o', '--output', required=False)
    cli_args = parser.parse_args()

    read_bags(cli_args.bags, cli_args.output)


if __name__ == '__main__':
    main()
