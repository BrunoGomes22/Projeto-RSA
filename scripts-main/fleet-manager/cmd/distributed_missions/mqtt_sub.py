import json
import paho.mqtt.client as mqtt

currentSensor = None

def main():
    client = mqtt.Client("sub01")
    client.connect("localhost")
    client.on_message = callbackPhone
    client.subscribe("sensors")

    while True:
        client.loop(0.5)

def callbackPhone(client, userdata, message):
    msg = json.loads(message.payload)

    print("msg=",msg)

if __name__ == '__main__':
    main()