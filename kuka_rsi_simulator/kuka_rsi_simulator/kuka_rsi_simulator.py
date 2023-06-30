#!/usr/bin/env python3

import argparse
import sys
import socket
import numpy as np
import time
import xml.etree.ElementTree as ET


import errno
import rclpy
from std_msgs.msg import String

def create_rsi_xml_rob(act_joint_pos, setpoint_joint_pos, timeout_count, ipoc):
    q = act_joint_pos
    qd = setpoint_joint_pos
    root = ET.Element('Rob', {'TYPE':'KUKA'})
    ET.SubElement(root, 'RIst', {'X':'0.0', 'Y':'0.0', 'Z':'0.0',
                                 'A':'0.0', 'B':'0.0', 'C':'0.0'})
    ET.SubElement(root, 'RSol', {'X':'0.0', 'Y':'0.0', 'Z':'0.0',
                                 'A':'0.0', 'B':'0.0', 'C':'0.0'})
    ET.SubElement(root, 'AIPos', {'A1':str(q[0]), 'A2':str(q[1]), 'A3':str(q[2]),
                                  'A4':str(q[3]), 'A5':str(q[4]), 'A6':str(q[5])})
    ET.SubElement(root, 'ASPos', {'A1':str(qd[0]), 'A2':str(qd[1]), 'A3':str(qd[2]),
                                  'A4':str(qd[3]), 'A5':str(qd[4]), 'A6':str(qd[5])})
    ET.SubElement(root, 'Delay', {'D':str(timeout_count)})
    ET.SubElement(root, 'IPOC').text=str(ipoc)
    return ET.tostring(root)

def parse_rsi_xml_sen(data):
    root = ET.fromstring(data)
    AK = root.find('AK').attrib
    desired_joint_correction = np.array([AK['A1'], AK['A2'], AK['A3'],
                                         AK['A4'], AK['A5'], AK['A6']]).astype(np.float64)
    IPOC = root.find('IPOC').text
    return desired_joint_correction, int(IPOC)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='KUKA RSI Simulation')
    parser.add_argument('--rsi_hw_iface_ip', default="127.0.0.1", help='The ip address of the RSI control interface (default=127.0.0.1)')
    parser.add_argument('--rsi_hw_iface_port', default=49152, help='The port of the RSI control interface (default=49152)')
    parser.add_argument('--sen', default='ImFree', help='Type attribute in RSI XML doc. E.g. <Sen Type:"ImFree">')
    # Only parse known arguments
    args, _ = parser.parse_known_args()
    host = args.rsi_hw_iface_ip
    port = int(args.rsi_hw_iface_port)
    sen_type = args.sen

    # Configuration
    node_name = 'kuka_rsi_simulation'
    cycle_time = 0.004
    act_joint_pos = np.array([0, -90, 90, 0, 90, 0]).astype(np.float64)
    cmd_joint_pos = act_joint_pos.copy()
    des_joint_correction_absolute = np.zeros(6)
    timeout_count = 0
    ipoc = 0

    node = rclpy.create_node(node_name)

    node.get_logger().info(f"Started '{node_name}' node.")

    rsi_act_pub = node.create_publisher(String, '~/rsi/state', 1)
    rsi_cmd_pub = node.create_publisher(String, '~/rsi/command', 1)

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        node.get_logger().info(f"Successfully created socket.")
        s.settimeout(1)
    except socket.error as e:
        node.get_logger().fatal(f"Could not create socket.")
        sys.exit()

    while rclpy.ok():
        time.sleep(0.001)  # FIXME: make this a ros2 node
        try:
            str_data = create_rsi_xml_rob(act_joint_pos, cmd_joint_pos, timeout_count, ipoc)
            msg = String();
            msg.data = str(str_data)
            rsi_act_pub.publish(msg)
            s.sendto(str_data, (host, port))
            recv_msg, addr = s.recvfrom(1024)
            msg = String();
            msg.data = str(recv_msg)
            rsi_cmd_pub.publish(msg)
            des_joint_correction_absolute, ipoc_recv = parse_rsi_xml_sen(recv_msg)
            act_joint_pos = cmd_joint_pos + des_joint_correction_absolute
            ipoc += 1
            time.sleep(cycle_time / 2)
        except socket.timeout:
            node.get_logger().warn(f"Socket timed out.")
            timeout_count += 1
        except socket.error as e:
            if e.errno != errno.EINTR:
                raise

    node.get_logger().info(f"Shutting down '{node_name}' node.")
    node.destroy_node()
    rclpy.shutdown()
    s.close()
