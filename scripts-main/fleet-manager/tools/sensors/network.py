import argparse
import json
import sys
import time
from threading import Thread
import re
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String
import subprocess as sp


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
    return yaml.load(open(cli_args.config)), cli_args.export, cli_args.silence


def current_time_millis():
    return int(round(time.time() * 1000))

def calc_network(id, ip, mac, interface, times):
    lista = []

    for i in range(times):
        # Measure Rxbit,Txbit and signal and get the average value to make it more stable
        try:
            res = sp.check_output('iw dev ' + interface + ' station get ' + mac
                                  + '| grep -E \'Station|tx bytes:|tx packets:|signal:|tx bitrate:|rx bitrate:|expected throughput:\'', shell = True)
            res = res.decode("utf-8")
        except:
            print("DEVICE NOT CONNECTED")
            return {}
        tx_byte = re.findall("(?<=tx bytes:\\s)\\d+",res)
        signal = re.findall("(?<=signal:\\s{2}\\t)(-?\\d+)",res)
        tx_bit = re.findall("(?<=tx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)
        rx_bit = re.findall("(?<=rx bitrate:\\s)(\d+.\\d\\s.+\\/s)",res)

        # Prevent error when RxBit is Unknown
        if len(rx_bit) != len(signal) :
            return {}

        lista.append({
            "station": mac,
            "ip": ip,
            "latency": 0,
            "signal": str(abs(int(signal[0]))),
            "txByte": tx_byte[0],
            "rxBit": rx_bit[0],
            "txBit" : tx_bit[0],
        })

    station =  {
        "station": mac,
        "ip": ip,
        "latency": 0,
        "signal": 0,
        "txByte": lista[0].get('txByte'),
        "rxBit": 0,
        "txBit" : 0,
        "relay" : id,
        }
    latencies = []

    # Retrieve the latency by performing pings
    for _ in range(1):
        try:
            latency = sp.check_output('ping -c 1 -w 1 ' + ip, shell = True)
            latency.decode("utf-8")
            latency = re.findall("(?<=time=)\d*.?\d*", str(latency))
            latencies.append(latency[0])
        except:
            latencies.append("1000")
    sum = 0
    for i in range(0, len(latencies)):    
        sum = sum + float(latencies[i])
    latency = sum / 1

    # Sum all the Rxbit, TxBit and signal parameters        
    for message in lista:
        station['signal'] = station['signal'] + float(message.get('signal'))
        station['rxBit'] = station['rxBit'] + float(message.get('rxBit').split(" ")[0])
        station['txBit'] = station['txBit'] + float(message.get('txBit').split(" ")[0])
        

    # Divide the Parameters to get the average
    rx_bit = round(station['rxBit']/times, 2)
    tx_bit = round(station['txBit']/times, 2)
    signal = station['signal'] /times
    station['latency'] = str(round(latency, 2)) + " ms"
    station['signal'] = str(signal)
    station['rxBit'] = str(rx_bit) + " MBit/s"
    station['txBit'] = str(tx_bit) + " MBit/s"

    # Make an estimate of the network quality based on some parameters
    if signal < 60 and latency < 50 and tx_bit > 5:
        station['networkQuality'] = "HIGH"
    elif signal <= 75 and latency < 100 and tx_bit > 3:
        station['networkQuality'] = "MEDIUM"
    else:
        station['networkQuality'] = "LOW"
    return station


class NetworkSensor(Node):

    def __init__(self):

        self.rate = 0.2
        self.sensor_topic = '/sensor/network'
        self.telem_topic = '/telem'
        self.info_topic = '/info'
        self.coords = None
        self.times = 0
        self.rate_low = 0.4
        self.rate_medium = 0.3
        self.rate_high = 0.2
        
        cfg, export, self.silence = parse_args()

        if cfg is None:
            print('[ERROR] Provided config file is empty.')
            sys.exit()
        if cfg.get('droneId') is None:
            print('[ERROR] Provided config file does not contain drone ID.')
            sys.exit()

        self.drone_id = cfg.get('droneId')
        
        if cfg.get('sensorTopic') is not None:
            self.sensor_topic = cfg.get('sensorTopic')
        
        if cfg.get('telemTopic') is not None:
            self.telem_topic = cfg.get('telemTopic')

        if cfg.get('rateHigh') is not None:
            self.rate_high = cfg.get('rateHigh') / 1000

        if cfg.get('rateMedium') is not None:
            self.rate_medium = cfg.get('rateMedium') / 1000

        if cfg.get('rateLow') is not None:
            self.rate_low = cfg.get('rateLow') / 1000

        
        if cfg.get('groundStation_IP') is None:
            print('[ERROR] Provided config file does not contain the ip addresses of GroundStation.')
            sys.exit()

        if cfg.get('groundStation_MAC') is None:
            print('[ERROR] Provided config file does not contain the mac addresses of GroundStation.')
            sys.exit()

        super().__init__('network', namespace=self.drone_id + '/sensor')

        self.relay = {"groundStation": [["groundStation", cfg.get('groundStation_IP'), cfg.get('groundStation_MAC')]],
                      "relay": [["groundStation", cfg.get('groundStation_IP'), cfg.get('groundStation_MAC')]]}
        
        self.interface = cfg.get('interface')

        self.subscription = \
            self.create_subscription(String, self.telem_topic, lambda msg: self.drone_telem_callback(msg.data), 10)
        
        self.subscription2 = \
            self.create_subscription(String, self.info_topic, lambda msg: self.drone_info_callback(msg.data), 10)
        

        self.publisher = None
        self.timer = None

        thread = Thread(target=self.init_pub)
        thread.start()

    def init_pub(self):
        self.get_logger().info('Waiting for drone "%s" telem data...' % self.drone_id)
        self.get_logger().info('Start network sensor for drone "%s"' % self.drone_id)
        self.publisher = self.create_publisher(String, self.sensor_topic, 10)
        self.timer = self.create_timer(self.rate, self.pub_network_callback)



    def pub_network_callback(self):
        # Publish the network quality in the topic
        msg = String()
        value = self.get_network()
        msg.data = json.dumps(
            {'droneId': self.drone_id, 'sensorId': 'real','type': 'network',
             'timestamp': current_time_millis(), 'value': value})

        if not self.silence:
            self.get_logger().info(msg.data)
        self.publisher.publish(msg)
        

    def drone_telem_callback(self, msg):
        #Receive drone telemetry messages
        telem = json.loads(msg)
        if telem.get('droneId') == self.drone_id and telem.get('position').get('lat') is not None:
            if self.times == 0:
                self.times = self.times + 1
            self.coords = (telem.get('position').get('lat'), telem.get('position').get('lon'))


    def drone_info_callback(self, msg): 
           info = json.loads(msg)

           # Retrieve messages asking for a specific connection
           if info.get('droneId') == self.drone_id and info.get('connectionsId') is not None and \
                   info.get('connectionsIp') is not None and info.get('connectionsMac') is not None:
               temp_array = []
               drone_ids = info.get('connectionsId')
               drone_ips = info.get('connectionsIp')
               drone_mac = info.get('connectionsMac')
               number_of_connections = len(drone_ids)
               if len(drone_ids) != len(drone_ips):
                   temp_array = []
               else:
                   for i in range(number_of_connections):
                       temp_array.append([drone_ids[i],drone_ips[i],drone_mac[i]])
               self.relay['groundStation'] = temp_array

           # Retrieve the message that indicates who the relay drones are
           elif info.get('droneId') == self.drone_id and info.get('relayId') is not None and info.get('relayIp') is not None and info.get('relayMac'):
               temp_array = []
               drone_ids = info.get('relayId')
               drone_ips = info.get('relayIp')
               drone_mac = info.get('relayMac')
               number_of_connections = len(drone_ids)
               if len(drone_ids) != len(drone_ips):
                   temp_array = []
               else:
                   for i in range(number_of_connections):
                       temp_array.append([drone_ids[i],drone_ips[i],drone_mac[i]])
               self.relay['relay'] = temp_array

    
    def get_network(self):
        return_val = {}
        return_msg_gs = []

        # Retrieve the network quality parameters to send to the ground Station
        for i in range(len(self.relay['groundStation'])):
            return_msg_gs.append(calc_network(self.relay['groundStation'][i][0],self.relay['groundStation'][i][1],self.relay['groundStation'][i][2],self.interface,100))
        return_val['groundStation'] = return_msg_gs

        mapper = {"HIGH" : 3 , "MEDIUM" : 2, "LOW" : 1}
        quality = "HIGH"

        # Retrieve the network quality between the relay drones
        for i in range(len(self.relay['relay'])):
            return_msg_relay = calc_network(self.relay['relay'][i][0],self.relay['relay'][i][1],self.relay['relay'][i][2],self.interface,100)
            print(return_msg_relay)
            if return_msg_relay != {}:
                return_msg_relay = return_msg_relay.get('networkQuality')
                
                if mapper[return_msg_relay] < mapper[quality]:
                    quality = return_msg_relay
            else:
                quality = "disconnected"

        # Change telemetry rate when network quality declines or gets better
        self.change_telem_rate(quality)

        return_val['relay'] = quality

        return [return_val]

    def change_telem_rate(self,quality):

        # Change telemetry sending rate and sensor sending rate when network quality changes
        if quality == "HIGH" and self.rate != self.rate_high:
            temp = str(int(self.rate_high * 1000))
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs " + temp, shell = True)
            self.destroy_timer(self.timer)
            self.rate = self.rate_high
            self.timer = self.create_timer(self.rate, self.pub_network_callback)

        elif quality == "MEDIUM" and self.rate != self.rate_medium:
            temp = str(int(self.rate_medium * 1000))
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs " + temp, shell = True)
            self.destroy_timer(self.timer)
            self.rate = self.rate_medium
            self.timer = self.create_timer(self.rate, self.pub_network_callback)

        elif quality == "LOW" and self.rate != self.rate_low:
            temp = str(int(self.rate_low * 1000))
            sp.run("ros2 param set /" + self.drone_id + " telemetryRateMs " + temp, shell = True)
            self.destroy_timer(self.timer)
            self.rate = self.rate_low
            self.timer = self.create_timer(self.rate, self.pub_network_callback)

def main(args=None):
    rclpy.init(args=args)
    net_sensor = NetworkSensor()
    rclpy.spin(net_sensor)
    if net_sensor.coords is not None:
        net_sensor.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
