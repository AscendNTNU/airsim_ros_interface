#!/usr/bin/env python

import rospy
import time
import math
import signal
import sys
import msgpackrpc 

rpc_client = msgpackrpc.Client(msgpackrpc.Address("127.0.0.1", 41451), timeout = 3600, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')


def sendBackPropellerThrustSignal(value):
    rpc_client.call('setBackPropellerControlSignal', value, "SimpleFlight")

def collidesWithModule():
    return rpc_client.call("collidesWithModule", "SimpleFlight")

def signal_handler(sig, frame):
    # Wait until AirSim closes the connnection
    rospy.loginfo("Quitting, stopping connection to AirSim, please wait...")
    image_client.reset()
    rospy.signal_shutdown("Successfully shut down the airsim node")

def main():
    rospy.init_node('test', disable_signals=True)
    signal.signal(signal.SIGINT, signal_handler)


    sendBackPropellerThrustSignal(0.1);

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
