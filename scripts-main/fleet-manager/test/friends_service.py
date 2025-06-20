from drone_interfaces.msg import MissionStatus
from drone_interfaces.srv import Scouting
from drone_interfaces.srv import Monitoring
from drone_interfaces.srv import Inspection

import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--drone-id', dest='id', required=True)
    parser.add_argument('-s', '--service', dest='service', required=True, choices=["scouting", "monitoring", "inspection"])
    parser.set_defaults(export=False)
    parser.set_defaults(export=False)

    cli_args = parser.parse_args()
    return cli_args.id, cli_args.service

class FriendsService(Node):

    def __init__(self):
        self.id, self.service = parse_args()
        super().__init__(self.service + '_service')
        if self.service == 'scouting':
            self.service_type = Scouting
        elif self.service == 'monitoring':
            self.service_type = Monitoring
        elif self.service == 'inspection':
            self.service_type = Inspection

        self.srv = self.create_service(self.service_type, self.id, self.handle_service)
        self.publisher = self.create_publisher(MissionStatus, "mission_status", 10)
        self.job = False
        self.job_counter = 0
        self.state = "idle"
        self.create_timer(1, self.do_job)

    def handle_service(self, request, response):
        # TODO return error if given in parameters
        # TODO result string isn't defined
        response.result = ""
        log_msg = request.action.capitalize() + " " + self.service + " mission with drone " + self.id
        self.get_logger().info(log_msg)
        self.pub_status_info_msg(log_msg)
        if request.action == "start":
            if self.state == "idle":
                self.job_counter = 0
                self.job = True
                self.state = "run"
            else:
                response.result = "error"
        elif request.action == "stop":
            if self.state == "run":
                self.job = False
                self.job_counter = 0
                self.state = "idle"
            else:
                response.result = "error"
        elif request.action == "pause":
            if self.state == "run":
                self.job = False
                self.state = "pause"
            else:
                response.result = "error"
        elif request.action == "resume":
            if self.state == "pause":
                self.job = True
                self.state = "run"
            else:
                response.result = "error"
        return response

    def pub_status_info_msg(self, message):
        msg = MissionStatus()
        msg.drone_id = self.id
        msg.task = self.service
        msg.type = 'info'
        msg.message = message
        self.publisher.publish(msg)

    def do_job(self):
        if self.job:
            msg = MissionStatus()
            msg.drone_id = self.id
            msg.task = self.service
            msg.type = 'progress'
            msg.message = str(self.job_counter*10)
            self.get_logger().info("Progress of " + self.service + " for drone '" + self.id + "': " +  msg.message)
            self.publisher.publish(msg)
            self.job_counter += 1
            if self.job_counter == 11:
                self.job = False
                self.state = "idle"
                log_msg = "Concluded " + self.service + " mission with drone " + self.id
                self.get_logger().info(log_msg)
                self.pub_status_info_msg(log_msg)



def main(args=None):
    rclpy.init(args=args)
    friends_service = FriendsService()
    rclpy.spin(friends_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
