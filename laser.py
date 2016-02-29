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

import logging
logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

class Reading (object):
        """
        A reading is a unit of lidar data.  It contains both range and heading
        information.     There is also some error and warning information.
        """
        #
        # create the reading from two raw values
        #
        error_mask = 0x8000
        warning_mask = 0x4000
        MM_PER_INCH = 25.4
        #
        #
        #
        def __init__ (self, heading, raw_distance, raw_strength):
                self.heading = heading
                self.raw_distance = raw_distance
                self.strength = raw_strength

        @property
        def error(self):
                return self.error_mask & self.raw_distance

        @property
        def warning(self):
                return self.warning_mask & self.raw_distance

        @property
        def discard(self):
                return self.error or self.warning

        @property
        def range(self):
                return ~(self.error_mask | self.warning_mask) & self.raw_distance

        @property
        def range_in_inches(self):
                """Return the distance in inches.  999 if there is an error."""
                if (self.warning or self.error):
                        return 777
                else:
                        return self.range / Reading.MM_PER_INCH
        #
        #  The formal representation of the reading contains the index (degrees)
        #  and the range in inches.
        #
        def __str__(self):
                return "{:d},{:.2f}".format(self.heading, self.range_in_inches)


#
#  Packet class is the one-stop shop for information about the packet
#  It knows how to read the packet off the wire and how to parse/decode
#  the contents.
#  A filter can be run over an array of Packets in order to produce a flattened data set
#  The filter can also be used to remove data with
# basic packet format string.  Used to crack 22 byte stream
#   
#  <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
#  `byte 0 : <distance 7:0>`
#  `byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
#  `byte 2 : <signal strength 7:0>`
#  `byte 3 : <signal strength 15:8>`
#
class Packet (object):
        """
        Packet is the parseable unit read from the lidar.   For 2.4 and 2.7,
        this is 22 bytes containing 4 readings for 4 degrees.
        """
        packet_start = 0xfa
        index_offset = 0xa0
        payload_length = 20
        packet_length = 22
        speed_units_per_rpm = 64
        slices_in_rotation = 90

        #  Packet format defined here, two bytes (marker + index, then a bunch of little endian shorts
        structure_def_str = '<BB H HH HH HH HH H'

        #  Names of the unpacked bits
        tuple_def_str = 'start index speed dist0 strength0 dist1 strength1 dist2 strength2 dist3 strength3 checksum'

        #  we'll pack the readings in an array so we can iterate over them more easily
        readings = []

        # one time init for how to unpack data and tuplify it
        structure_def = struct.Struct(structure_def_str)
        tuple_def = collections.namedtuple('Packet_tuple', tuple_def_str)                
        #
        #  Construct useable internal
        #
        def __init__(self, packed_data):
                """Create a packet from the 22byte serial lidar data"""
                #
                #  Use the unpacking structure and the tuple def to get it into friendly form
                #
                unpacked_data = Packet.structure_def.unpack(packed_data)
                packet_tuple = Packet.tuple_def._make(unpacked_data)

                #
                #  Packet index represents a 4 degree range  (with high nibble offset)
                #
                self.index = Packet.decode_index(packet_tuple.index)
                index_degrees = 4*self.index

                #
                # initialize the readings for this Packet
                #
                self.readings = [ Reading(index_degrees+0, packet_tuple.dist0, packet_tuple.strength0),
                                  Reading(index_degrees+1, packet_tuple.dist1, packet_tuple.strength1),
                                  Reading(index_degrees+2, packet_tuple.dist2, packet_tuple.strength2),
                                  Reading(index_degrees+3, packet_tuple.dist3, packet_tuple.strength3) ]
                #
                # 
                #
                self.speed = packet_tuple.speed

        @staticmethod
        def decode_index(index_value):
                return index_value - Packet.index_offset

        #
        #  Decode the rpm value 
        #
        @property
        def rpm(self):
                """Speed is reported in 64ths of an RPM"""
                return int(round(self.speed / Packet.speed_units_per_rpm));

        #
        #
        #
        def __str__(self):
                """Display friendly representation of a lidar packet"""
                return "{:d} {:d} {:.2f} {:.2f} {:.2f} {:.2f}".format(self.index,
                                                                      self.rpm,
                                                                      self.readings[0].range_in_inches,
                                                                      self.readings[1].range_in_inches,
                                                                      self.readings[2].range_in_inches,
                                                                      self.readings[3].range_in_inches)
        #
        # 
        #
        def as_data(self):
                """Slice index i is for degrees 4*i, 4*i+1, 4*i+2, and 4*i+3"""
                str_data = "" 
                for i in range(4):
                        str_data = str_data + "{!s}\n".format(self.readings[i])
                return str_data                

        #
        # Checksum algorithm applies to the packet
        # Checksum function does not work.  (we ought to get it work, though)
        #
        def checksum(self,data):
                """Return True if the checksum is correct and False otherwise"""
                # group the data by word, little-endian
                data_list = []
                for t in range(10):
                        data_list.append( data[2*t] + (data[2*t+1]<<8) )
                        
                # compute the checksum on 32 bits
                chk32 = 0

                for d in data_list:
                        chk32 = (chk32 << 1) + d
                        
                # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
                checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
                checksum = checksum & 0x7FFF # truncate to 15 bits
                if (checksum == data_list[9]):
                        print("{:d} passes checksum check".format(data_list[1]))

                return int( checksum )

        def __getitem__(self, ndx):
                return self.readings[ndx]

class Laser (object):
	
	haslaser = 0
	laserport = 0
	laserlock_fd = ''
	laserlock_sz = 0
	mapnames = []
	cmdmode = 0

	def __init__(self,sd):
                """setup internal variables and associate the laser with open serial port"""
		self.results=0
		self.haslaser=0
		self.laserport=sd
        

        #
        #  Get a lidar packet for a 4 degree slice of the circle
        #  <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
        #  Slice index goes from 0 to 89.  Packet may encode it differently, though
        #
	def packet_for_slice(self,slice_index):
                """read in a packet and return the constructed packet"""
		headerpos=0
		scanline=0
		insync=0
		rpms=0	
		scandist=[]
		scanbytes=[]

                #
                # Index has 0xa in the hi nibble
                #
                packet_index = slice_index + Packet.index_offset
                
                # 22 bytes in the packet, let us read it all in there
                scanba=bytearray(Packet.packet_length)

                #
                # print("Looking for packet with index {:d}\n".format(slice_index))
		while insync == 0:
                        scanba[0] = ord(self.laserport.read(1))
			if scanba[0] == Packet.packet_start:
                                scanba[1] = ord(self.laserport.read(1))
				if scanba[1] == packet_index:
					insync=1
                #
                # We got a marker value and a matching slice value
                # Read the remaining bytes and do the checksum calculation
                #
		if insync == 1:
		        try:
                                # read the final Packet.payload_length bytes
                                charbuffer = self.laserport.read(Packet.payload_length)
                                if len(charbuffer) == Packet.payload_length:
                                        scanba[2:2+Packet.payload_length] = map(ord,charbuffer)
                                # unpack the data conformant with the structure def
                                slice_pkt = Packet(scanba)
                                        
			except IOError,e:
				print("Failed trying to read scanline {:x}".format(scanhdr))
		return slice_pkt


        #
        # Collect a complete set of packets and concatenate them into
        #
        def gather_full_rotation(self):
                """Read a rotation of lidar data and return it as a collection of packets"""
                rotation = []
                slice_index = 0

                #
                #  Loop over all of the slices 
                #
                for slice_index in range(Packet.slices_in_rotation):
                        try:
                                if self.laserport == None:
                                        # drop in a synthetic packet when there is no lidar
                                        packet_str  = "fb{:02x}0040fe002200fc014400f803660077808800abcd".format(slice_index+Packet.index_offset)
                                        packet = Packet(binascii.unhexlify(packet_str))
                                else:
	                                packet = self.packet_for_slice(slice_index)
                                rotation.append(packet)
                        except Exception as e:
                                print("Unable to get packet for data slice with index {:d}.".format(slice_index))

                #
                # If generating synthetic data, slow down for a bit
                # Sleep is related to the rotation speed in the synthetic packet.
                #
                if self.laserport == None:
                        # sleep like it took some time to get the data if we are faking it
                        elapsed_time = 60.0 / float(packet.rpm) 
                        logger.info("Sleeping for {:02f} seconds".format(elapsed_time))
                        sleep(elapsed_time)

                # return either the real or the synthetic rotation
                return rotation


#
#  A single rotation of lidar data
#
class Rotation(object):
        """
        Rotation stores the lidar packets for a single rotation.
        Rotation provides reduced, transformed, and/or sanitized
        data to the client wanting to use the data.

        The coordinate system used for the rotation is -90 to 90 degrees.

        Left to the programmer to generalize the implementation to support
        the generalization of the supported range.

        Note:  The lidar has a predermined order for gathering the data,
        so, some changes there would be necessary in order to ensure that
        the data reported in the left-to-right sweep was gathered in
        the same order.    This permits motion correction to be linearly
        applied to the data (linear interpolation applied to the full
        scan based upon the change in the range reported by the same
        heading range from scan-to-scan.
        """
        left_to_right = (-90, 91)
        full_rotation_packets = 90
        
        def __init__(self, rotation_packets):
                """Create a single rotation view of the world"""
                self.packets = rotation_packets

                if len(self.packets) != Rotation.full_rotation_packets:
                        print("Not playing with a full rotation of data, dude!")

                #
                # rotation might not have been built from 0 to 359, so
                # sort the data by the heading value.  Note that we could
                # be more efficient by packing the data in the order that we
                # want, but this is more robust.
                #
                # store as clean polar data in the -90 to 90 range
                #
                self.all_readings = sorted(list(itertools.chain(*self.packets)),
                                           key = lambda x: x.heading)
                
                view_readings = [self.all_readings[i]
                                 for i in range(*Rotation.left_to_right)
                                 if not self.all_readings[i].discard]

                #
                #  For ease of manipulation, the view data is just stored as
                #  a list of tuples (heading, range (in inches))
                #
                
                self.view_data  = [(r.heading, r.range_in_inches) for r in view_readings]

        @staticmethod
        def polar_to_cart(theta, r):
                """convert cartesian to polar data"""
                theta_r = math.radians(theta)
                return r*math.cos(theta_r), r*math.sin(theta_r)

        def polar_data(self):
                return self.view_data
        
        def cartesian_data(self):
                """Return an array of clean cartesian data points (left to right)"""
                return [Rotation.polar_to_cart(theta, r) for theta, r in self.view_data]
        
        def rpm(self):
                """report an rpm value collected in this rotation"""
                return self.packets[0].rpm

        #
        #  Not sure what ought to be iterable about this,
        #  but, for now, it can be the sanitized polar data.
        #
        def __getitem__(self, ndx):
                return self.view_data[ndx]

