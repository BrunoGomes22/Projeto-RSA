import json
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
import paho.mqtt.client as mqtt

# Global variables
publisherInfo = ''
serverURI = '192.168.1.184'
client = None

# Info subscribers
currentInfo = """{"TO" : -1}"""
info_subscriber = None

# Drone List
dronesWithRelay = []
availableDronesChase = ["drone01"]
availableDronesRelay = ["drone02", "drone03", "drone04"]

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
    global availableDronesRelay
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
                if len(availableDronesRelay) > 0:
                    # Update Lists
                    droneToSend = availableDronesRelay.pop(0) 
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

            # cancelRelay handler
            if tmp['TYPE'] == 'cancelRelay' and tmp['FROM'] in dronesWithRelay:
                dronesWithRelay.remove(tmp['FROM']) 
                print("cancelRelay from",tmp['FROM'])

            # freeDrone handler
            if tmp['TYPE'] == 'freeDrone' and tmp['FROM'] not in availableDronesRelay:
                availableDronesRelay.append(tmp['FROM'])
                print("freeDrone from",tmp['FROM'])

            # freeDrone handler
            if tmp['TYPE'] == 'freeDroneChase' and tmp['FROM'] not in availableDronesChase:
                availableDronesChase.append(tmp['FROM'])
                print("freeDrone from",tmp['FROM'])    
                
        print("availableDronesChase = ",availableDronesChase)
        print("availableDronesRelay = ",availableDronesRelay)
        print("dronesWithRelay = ",dronesWithRelay)       

        # Free thread
        sleep(0.01)

def MQTT():
    global client    

    while True:
        client.loop(0.5)
        # Free thread
        sleep(0.01)

def MQTTHandler(client, userdata, message):
    global availableDronesChase

    currentSensor = json.loads(message.payload)
    print("currentSensor =",currentSensor)
    #print("MQTTHandler")
    if currentSensor['type'] == "locatorRequest":
        print("locatorRequest")
        # If there are available drones 
        if len(availableDronesChase) > 0:
            droneToSend = availableDronesChase.pop(0) 
            msg = {'FROM' : 'GroundStation',
                    'TO' : droneToSend,
                    'TYPE' : 'chaseAssigned',                   
                    'sensorAssigned' : currentSensor['sensorId']}
            publish_info(msg)
            # Send message to phone to inform
            message = {'FROM' : 'GroundStation',
                             'TO' : currentSensor['sensorId'],
                             'type' : 'chaseAccepted',
                             'droneAssigned' : droneToSend}          

            client.publish("sensors",json.dumps(message))

        # If there are not available drones, it'll deny the request
        else:
            # Send message to phone
            message = {'FROM' : 'GroundStation',
                             'TO' : currentSensor['sensorId'],
                             'type' : 'chaseDenied'}          

            client.publish("sensors",json.dumps(message))



def main(args=None):
    global info_subscriber
    global publisherInfo
    global client

    rclpy.init(args=args)

    # Initialize publishers
    node = rclpy.create_node('publisher_GroundStation_relay')
    publisherInfo = node.create_publisher(String, '/info', 10)
    publisher = node.create_publisher(String, '/cmd', 10)

    # Initialize subscribers
    info_subscriber = InfoSubscriber()

    # MQTT
    client = mqtt.Client("subscriberGS")
    client.connect(serverURI)
    client.on_message = MQTTHandler
    client.subscribe("sensors")

    # To make sure every message is sent
    sleep(5)

    print("READY!")

    # Threads used for check for relays
    checkMQTT = threading.Thread(target=MQTT)
    checkMQTT.start()

    # Listen for relay requests from drones
    checkRelayRequests()


if __name__ == '__main__':
    main()
