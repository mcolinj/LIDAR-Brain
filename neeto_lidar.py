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
from analyzer import Analyzer, find_wall_midpoint
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

        robot_name = '10.10.76.2'
        
        channel = UDPChannel(remote_ip=robot_name, remote_port=5880,
                              local_ip='0.0.0.0', local_port=52954)
        range_at_heading_message = LidarRangeAtHeadingMessage()
        periodic_message = LidarPeriodicMessage()
        wall_message = LidarWallMessage()
        #lidar_logger = LidarLogger(logger)

        file_index = 1
        rotation_time = 0
        current_time = 0
        seconds_per_output = 1
        SECONDS_PER_MINUTE = 60.0
        
        calibrated_zero = 6
        lp = None
        lasr = None

        if len(sys.argv) > 1:
                serial_port_name = sys.argv[1]
        else:
                logger.critical('Missing script argument identifying serial port to lidar unit.')
                logger.critical('Usage: python neeto_lidar.py </dev/tty name for serial port> <calibrated zero>')
                logger.critical('I will assume the device name is /dev/ttyUSB0')
                logger.critical('I will assume the calibrated zero is {:d} degrees'.format(calibrated_zero))
                logger.critical('I will assume the start logging file index is 1.')
                logger.critical('maybe /dev/tty.wchusbserial1420, or maybe /dev/tty.usbserial')
                periodic_message.status = 'badarg'
                periodic_message.rpm = 0
                if not suppress_robot_comm:
                        channel.send_to(periodic_message.encode_message())
                serial_port_name = '/dev/ttyUSB0'
        
        if len(sys.argv) > 2:
                calibrated_zero = sys.argv[2]

        if len(sys.argv) > 3:
                file_index = int(sys.argv[3])

        if len(sys.argv) > 4:
                suppress_robot_comm = True
        else:
                suppress_robot_comm = False
        
	while 1: 
                if lp is None:
                        # open up the serial port device connected to the lidar
                        try:
		                #lp = serial.Serial('/dev/ttyUSB0',115200,timeout=1)
		                #lp = serial.Serial('/dev/tty.usbserial',115200,timeout=1)
		                #lp = serial.Serial('/dev/tty.wchusbserial1420',115200,timeout=1)
                                lp = serial.Serial(serial_port_name, 115200, timeout=1)
                                lasr = Laser(lp)
                        except: 
                                logger.critical('Unable to open lidar port: {}'.format(serial_port_name))
                                logger.critical('Try /dev/ttyUSB0, or maybe /dev/tty.wchusbserial1420, or maybe /dev/tty.usbserial')
                                periodic_message.status = 'down'
                                periodic_message.rpm = 0
                                if not suppress_robot_comm:
                                        channel.send_to(periodic_message.encode_message())
                                logger.error('Lidar port could not be opened.')

                if lasr is not None:
                        try:
                                # NOTE: because lidar is upside down, this needs to be reversed?
                                rotation = Rotation(lasr.gather_full_rotation(reverse_data=True))
                                
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
                                if not suppress_robot_comm:
                                        channel.send_to(range_at_heading_message.encode_message())
                                
                                # push periodic message to the bot
                                periodic_message.status = 'ok'
                                periodic_message.rpm = rotation.rpm()
                                if not suppress_robot_comm:
                                        channel.send_to(periodic_message.encode_message())
                                logger.info("reported rpm is {:d}".format(rotation.rpm()))

                                #
                                # send out the wall heading and distance report  (disabled)
                                #
                                if 0:
                                        (wall_heading, wall_distance, wall_orientation) = find_wall_midpoint(rotation.cartesian_data())
                                        logger.info("find_wall_midpoint => heading {:.1f}, range {:.1f}, orientation {:.1f})".format(wall_heading, wall_distance, wall_orientation))
                                        if (wall_heading, wall_distance, wall_orientation) == (0, 0, 0):
                                                wall_message.status = 'error'
                                        else:
                                                wall_message.status = 'ok'
                                                wall_message.range = wall_distance
                                                wall_message.heading = wall_heading
                                                wall_message.orientation = wall_orientation
                                                if not suppress_robot_comm:
                                                        channel.send_to(wall_message.encode_message())
                                
                                
 		        except IOError,e:
                                #  log and notify robot of error
                                periodic_message.status = 'error'
                                if not suppress_robot_comm:
                                        channel.send_to(periodic_message.encode_message())
                                logger.error("Failed to gather a full rotation of data.")

                        #
                        # get revised instructions from robot
                        #
                        if 0:
                                robot_data, robot_address = channel.receive_from()
                                message_from_robot = RobotMessage(robot_data)
                                if ((message_from_robot.sender == 'robot') and
                                    (message_from_robot.message == 'sweep')):
                                        Analyzer.start = message_from_robot.start
                                        Analyzer.stop = message_from_robot.stop
                        #except socket.timeout:
                        #        logger.info("No message received from robot")

                        elapsed_time = (SECONDS_PER_MINUTE/float(rotation.rpm()))
                        rotation_time = rotation_time + elapsed_time
                        logger.info("rotation time is: {:.1f}".format(rotation_time))
                        current_time = current_time + elapsed_time
                        if rotation_time > seconds_per_output:
                                fname = "data/lidar_snapshot_{:d}.dat".format(file_index)
                                LidarLogger.write_to_file(fname, rotation.polar_data())
                                file_index = file_index + 1
                                rotation_time = 0
