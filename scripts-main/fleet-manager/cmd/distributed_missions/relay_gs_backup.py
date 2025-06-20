import json
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Global variables
publisherInfo = ''

# Info subscribers
currentInfo = """{"TO" : -1}"""
info_subscriber = None

# Drone List
dronesWithRelay = []
availableDrones = ["drone02", "drone03", "drone04"]

def publish_info(info):
    global publisherInfo

    msg = String()    
    msg.data = json.dumps(info)
    publisherInfo.publish(msg)
    print("publish = ",msg.data)


class InfoSubscriber(Node):    
    def __init__(self):
        name = 'info_subscriber_GroundStation_relay'

        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            '/info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg): 
        global currentInfo
                         
        currentInfo = msg.data  

def checkRelayRequests():
    global info_subscriber
    global availableDrones
    global dronesWithRelay

    # Message handler
    while True:
        rclpy.spin_once(info_subscriber, timeout_sec=0.5)
        tmp = json.loads(currentInfo)
        if tmp['TO'] == 'GroundStation': 
            # requestRelay handler
            if tmp['TYPE'] == 'requestRelay' and tmp['FROM'] not in dronesWithRelay:
                print("Request from",tmp['FROM'])

                # If there are available drones
                if len(availableDrones) > 0:
                    # Update Lists
                    droneToSend = availableDrones.pop(0) 
                    dronesWithRelay.append(tmp['FROM'])
                    
                    print("droneToSend =",droneToSend)
                    # Send message back to the drone that asked for the relay
                    msg = {'FROM' : 'GroundStation',
                        'TO' : tmp['FROM'],
                        'TYPE' : 'relayAccepted',
                        'droneAssigned' : droneToSend}
                    publish_info(msg)
                    sleep(0.5)
                    # Send message to the drone that was assigned to do relay
                    msg = {'FROM' : 'GroundStation',
                        'TO' : droneToSend,
                        'TYPE' : 'relayAssigned',                   
                        'droneAssigned' : tmp['FROM']}
                    publish_info(msg)
                # If there are not available drones, it'll cancel the mission
                else:
                    msg = {'FROM' : 'GroundStation',
                        'TO' : tmp['FROM'],
                        'TYPE' : 'relayDenied'}
                    publish_info(msg)
                    pass

            # cancelRelay handler
            if tmp['TYPE'] == 'cancelRelay' and tmp['FROM'] in dronesWithRelay:
                dronesWithRelay.remove(tmp['FROM']) 
                print("cancelRelay from",tmp['FROM'])

            # freeDrone handler
            if tmp['TYPE'] == 'freeDrone' and tmp['FROM'] not in availableDrones:
                availableDrones.append(tmp['FROM'])
                print("freeDrone from",tmp['FROM'])
        
        print("availableDrones = ",availableDrones)
        print("dronesWithRelay = ",dronesWithRelay)



def main(args=None):
    global info_subscriber
    global publisherInfo

    rclpy.init(args=args)

    # Initialize publishers
    node = rclpy.create_node('publisher_GroundStation_relay')
    publisherInfo = node.create_publisher(String, '/info', 10)

    # Initialize subscribers
    info_subscriber = InfoSubscriber()

    # To make sure every message is sent
    sleep(5)

    print("READY!")

    # Listen for relay requests from drones
    checkRelayRequests()


if __name__ == '__main__':
    main()
