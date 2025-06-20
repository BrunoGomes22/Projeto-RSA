import json
from rclpy.node import Node
from std_msgs.msg import String
import time
import requests


def current_time_millis():
    return int(round(time.time() * 1000))


class CommNode(Node):

    def __init__(self, update_telem_callback):
        super().__init__('drone_dashboard')
        self.telem_topic = '/telem'
        self.status_topic = '/status'
        self.cmd_topic = '/cmd'
        self.update_telem_callback = update_telem_callback

        self.telem_sub = self.create_subscription(String, self.telem_topic,
                                                  lambda msg: self.drone_telem_callback(msg.data), 10)
        self.status_sub = self.create_subscription(String, self.status_topic,
                                                   lambda msg: self.drone_status_callback(msg.data), 10)
        self.cmd_pub = self.create_publisher(String, self.cmd_topic, 10)

    def drone_telem_callback(self, msg):
        telem = json.loads(msg)
        self.update_telem_callback(telem)

    # TODO
    def drone_status_callback(self, msg):
        pass

    def pub_action_cmd(self, id, cmd):
        msg = String()
        msg.data = json.dumps({'droneId': id, 'mode': 'action', 'cmd': cmd, 'timestamp': current_time_millis()})
        self.cmd_pub.publish(msg)

    def pub_cancel_cmd(self, id):
        msg = String()
        msg.data = json.dumps({'droneId': id, 'mode': 'custom', 'cmd': "cancel", 'timestamp': current_time_millis()})
        self.cmd_pub.publish(msg)

    def pub_move_cmd(self, id, forward, right, up):
        msg = String()
        msg.data = json.dumps({'droneId': id, 'mode': 'custom', 'cmd': 'move', 'x': right, 'y': forward, 'z': up,
                               'timestamp': current_time_millis()})
        self.cmd_pub.publish(msg)

    def pub_turn_cmd(self, id, degrees):
        msg = String()
        msg.data = json.dumps(
            {'droneId': id, 'mode': 'custom', 'cmd': 'turn', 'deg': degrees, 'timestamp': current_time_millis()})
        self.cmd_pub.publish(msg)

def submit_mission(mission):
    return requests.post('http://localhost:8001/mission',
                             headers = {
                                 'Accept': 'application/json',
                                 'Content-Type':'application/octet-stream'
                             },
                             data=mission)
def cancel_mission(mission_id):
    return requests.delete('http://localhost:8001/mission/%s'%mission_id,
                             headers = {
                                 'Accept': 'application/json',
                                 'Content-Type':'application/octet-stream'
                             })