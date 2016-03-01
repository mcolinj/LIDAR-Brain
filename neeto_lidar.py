#!/usr/bin/env python 
#laser.py 
# Copyright 2012 Stephen Okay for Roadknight Labs
# Released under the GNU GPL V2

from __future__ import print_function
"""
XV11 utils 
This program provides methods to get ranging data from a bare(i.e. not in a) Neato Robotics XV11 Laser Distance Scanner
running 2.6.x firmware
"""

import sys
import os 
import serial 
import pdb
import struct
import binascii
import collections
import itertools
import math
from time import sleep
import socket
from udp_channels import *
from sensor_message import *
from analyzer import Analyzer
from lidar_logger import LidarLogger
from laser import Laser, Reading, Packet, Rotation

import logging
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    level=logging.DEBUG)
logger = logging.getLogger(__name__)


#
#   Open up the serial port, get lidar data and write it to a file
#   every few seconds.
#
if __name__ == '__main__':

        channel = UDPChannel(remote_ip='10.10.76.100', remote_port=5880,
                              local_ip='0.0.0.0', local_port=52954)
        #channel = UDPChannel()
        range_at_heading_message = LidarRangeAtHeadingMessage()
        periodic_message = LidarPeriodicMessage()
        lidar_logger = LidarLogger(logger)
        
        lp = None
        
        # open up the serial port device connected to the lidar
        try: 	
		lp = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
        except: 
                logger.error('Lidar port could not be opened.')

        # connect the port with the laser and initialize the laser object
        #lasr = OldLaser(lp)
        lasr = Laser(lp)

        #
        #
	slice_index = 0
        file_index = 1
        rotation_time = 0
        current_time = 0
        seconds_per_output = 10
        SECONDS_PER_MINUTE = 60.0
	while 1: 
		try:
                        rotation = Rotation(lasr.gather_full_rotation())
                        # rotation = OldRotation(lasr.gather_full_rotation())
                        #
                        # For now, we just output a lidar data snapshot every 10 seconds
                        # We will want this for debugging (maybe every second instead)
                        # change the "seconds_per_output" to tune that.
                        #
                        tgt_heading, tgt_range = Analyzer.range_at_heading(rotation.polar_data(), (Analyzer.start, Analyzer.stop))
                        logger.info("{:d} points yields {:.2f} inches at {:2d} degrees)".format(len(rotation.polar_data()),tgt_range, tgt_heading))

                        # push the newly calculated data into the message
                        range_at_heading_message.heading = tgt_heading
                        range_at_heading_message.range = tgt_range
                        channel.send_to(range_at_heading_message.encode_message())

                        # push periodic message to the bot
                        periodic_message.status = 'ok'
                        periodic_message.rpm = rotation.rpm()
                        channel.send_to(periodic_message.encode_message())

 		except IOError,e:
                        #  log and notify robot of error
                        periodic_message.status = 'error'
                        channel.send_to(periodic_message.encode_message())
                        logger.error("Failed to gather a full rotation of data.")

                #
                # get revised instructions from robot
                #
                try:
                        robot_data, robot_address = channel.receive_from()
                        message_from_robot = RobotMessage(robot_data)
                        if ((message_from_robot.sender == 'robot') and
                            (message_from_robot.message == 'sweep')):
                                Analyzer.start = message_from_robot.start
                                Analyzer.stop = message_from_robot.stop
                except socket.timeout:
                        logger.info("No message received from robot")

                
                elapsed_time = (SECONDS_PER_MINUTE/float(rotation.rpm()))
                rotation_time = rotation_time + elapsed_time
                current_time = current_time + elapsed_time
                if rotation_time > seconds_per_output:
                        lidar_logger.log_data(rotation.polar_data())
                        file_index = file_index + 1
                        rotation_time = 0
