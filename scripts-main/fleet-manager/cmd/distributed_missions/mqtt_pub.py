import json
import paho.mqtt.client as mqtt

currentSensor = None

def main():
    client = mqtt.Client("phone01")
    client.connect("192.168.1.184")

    message = {"sensorId": "phone01",  
                  "type": "locator", 
                  "timestamp": 1618933363047, 
                  "value": {"lat":40.63462205169691, 
                  "lon":  -8.65863869434911, 
                  "alt": 5.0}}

    '''message = {"type" : "locatorRequest",
               "sensorId" : "phone01"}'''

    '''message = {"type" : "locatorCancel",
               "sensorId" : "phone01"}'''

    client.publish("sensors",json.dumps(message))

if __name__ == '__main__':
    main()