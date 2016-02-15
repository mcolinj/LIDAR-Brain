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
import socket
import pdb
import numpy as np
import matplotlib.pyplot as plt
import struct
import binascii
import collections
import itertools
import math

class Reading (object):
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
        def distance(self):
                return ~(self.error_mask | self.warning_mask) & self.raw_distance

        def distance_in_inches(self):
                """Return the distance in inches.  999 if there is an error."""
                if (self.warning or self.error):
                        return 777
                else:
                        return self.distance / Reading.MM_PER_INCH
        #
        #  The formal representation of the reading contains the index (degrees)
        #  and the distance in inches.
        #
        def __str__(self):
                return "{:d},{:.2f}".format(self.heading, self.distance_in_inches())


#
#  Packet class is the one-stop shop for information about the packet
#  It knows how to read the packet off the wire and how to parse/decode
#  the contents.
#  A filter can be run over an array of Packets in order to produce a flattened data set
#  The filter can also be used to remove data with
#
class Packet (object):
        packet_start = 0xfa
        index_offset = 0xa0
        payload_length = 20
        packet_length = 22
        speed_units_per_rpm = 64

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
                unpacked_data = structure_def.unpack(packed_data)
                packet_tuple = tuple_def._make(data)

                #
                #  Packet index represents a 4 degree range  (with high nibble offset)
                #
                self.index = Packet.decode_index(packet_tuple.index)
                index_degrees = 4*index

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
                self.speed = Packet.decode_rpm(packet_tuple.speed)

        @staticmethod
        def decode_index(index_value):
                return index_value - Packet.index_offset

        #
        #  Decode the rpd value 
        #
        @staticmethod
        def decode_rpm(speed):
                """Speed is reported in 64ths of an RPM"""
                return int(round(speed / Packet.speed_units_per_rpm));

        #
        #
        #
        def __str__(self):
                """Display friendly representation of a lidar packet"""
                # 
                print("{:d} {:d} {:.2f} {:.2f} {:.2f} {:.2f}".format(self.index,
                                                                     self.speed,
                                                                     self.readings[0].strength,
                                                                     self.readings[1].strength,
                                                                     self.readings[2].strength,
                                                                     self.readings[3].strength))
        #
        # 
        #
        def print_data(self):
                """Slice index i is for degrees 4*i, 4*i+1, 4*i+2, and 4*i+3"""
                for i in range(4):
                        print("{!s}\n".format(self.readings[i]))

        #
        # Checksum algorithm applies to the packet
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

        @staticmethod
        def unit_test():
                print("OK")

# packets = [ ... ]
# flattened = itertools.chain(*packets)
# backwards = reversed(flattened)
                     
class Laser (object):
	
	haslaser = 0
	laserport = 0
	laserlock_fd = ''
	laserlock_sz = 0
	mapnames = []
	cmdmode = 0

	def __init__(self,sd):
	#set up internal variables and just return a "stub" object
	#we expect laserport to be a handle to an already open serial.Serial object
		self.results=0
		self.haslaser=0
		self.laserport=sd
        
	def ack(self):
                """send carriage return and get the prompt from the lidar unit"""
		ackbuf=0
		self.laserport.flushInput()		
		self.laserport.write("\r")
		ackbuf = self.laserport.read(3)
		if (ackbuf == "#"):
			self.laserport.flushInput()
			return 0
		else:
			return -1

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
                packet_index = slice_index + 0xa0
                
                # basic packet format string.  Used to crack 22 byte stream

                #   
                #  <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
                #  `byte 0 : <distance 7:0>`
                #  `byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>`
                #  `byte 2 : <signal strength 7:0>`
                #  `byte 3 : <signal strength 15:8>`
                
                # create a slice with named parts
                slice = slice_tuple._make(unpacked_data)

                # 22 bytes in the packet, let us read it all in there
                scanba=bytearray(Packet.packet_length)

                # 
		while insync == 0:
			nbytes=self.laserport.readinto(scanba[0:1])
			if nbytes == 1 and scanba[0] == Packet.packet_start:
				nbytes=self.laserport.readinto(scanba[1:2])
				if nbytes == 1 and scanba[1] == packet_index:
					insync=1
                #
                # We got a marker value and a matching slice value
                # Read the remaining bytes and do the checksum calculation
                #
		if insync == 1:
		        try:
                                # read the final Packet.payload_length bytes
                                nbytes=self.laserport.readinto(scanba[2:2+Packet.payload_length])
                                if nbytes == Packet.payload_length:
                                        # unpack the data conformant with the structure def
                                        slice_pkt = Packet(scanba)

			except IOError,e:
				print("Failed trying to read scanline {:x}".format(scanhdr))
		return slice_pkt


        #
        # expect to start reading at the beginning of a packet, 
        # read a single byte, and if it has the headerbyte marker, followed
        # by the specified curhdr byte, then set things to be insync and return True
        # otherwise keep reading a single character at a time forever (if we never
        # encounter the two bytes, then we just keep reading.
        # Note:  the curhdr
        #
        def read_hdr(self,curhdr):
		headerpos=0
		scanline=0
		insync=0
		self.laserport.flushInput()
                while insync == 0:
			headerbyte=ord(self.laserport.read(1))
                        #print "Expecting %x got %x" % (self.pinghdr[headerpos],headerbyte)
                        if headerbyte == 0xfa :
				scanbyte=ord(self.laserport.read(1))
				if scanbyte == curhdr:
				# print("FOUND START OF SCAN {:x} with curhdr of {:x} ".format(headerbyte,scanbyte))
					insync=1
		return True
	
	def errbits(self,errbyte):
		result=0
		if ((0x8000&(errbyte<<8))):
			result=1
		if ((0x4000&(errbyte<<8))):
			result=3
		return result

	# print out the scanned line, including the sca
	def dump_scanline(self,scanpos,scanbuf):
		print("fa {:x}:".format(scanhdr),end=" ")
		for sb in scanbuf:
			disterr=l.errbits(sb)
			if(disterr == 0):
				print("{:x} ".format(sb),end=" ")
			else:
				if(disterr == 1):
					print("<>",end=" ")
				if(disterr == 3): 
					print("><",end=" ")
		print

        #Expect that the user has already filtered out too-close/too far distance errors from the bytestream
	def bytes_distance(self,scanbytes):
		return scanbytes[0]|(scanbytes[1]<<8)

	def scanline_distance(self,scanbuf):
		distances=[]
		for d in range((len(scanbuf))-1):
			distances.append(scanbuf[d]|(scanbuf[d+1]<<8))
		return distances

def plot_rotation(packets):
        readings = itertools.chain(*packets)
        theta, radius = map(lambda r: (r.heading, r.strength), readings)
        theta = map(math.radians, theta)
        LidarView.ax.cla()
        LidarView.ax.plot(theta, radius, 'x', color='r', linewidth=3)
        pylab.show(block=False)

if __name__ == '__main__':
	
        # open up the serial port device connected to the lidar
        try: 	
		lp= serial.Serial('/dev/tty.usbserial',115200,timeout=1)
	except: 
		print("Could not open serial port for laser!")
		exit()

        # connect the port with the laser and initialize the laser object
        l = laser(lp)

        fig = pylab.figure(figsize=(9,9))
        ax = fig.add_subplot(111, projection='polar')
        ax.set_rmax(240)
        ax.grid(True)
        
        #
        #
        #
	scanrange=[]
	distdata=[]
	slice_index=0
	while 1: 
		try:
			packets[slice_index]=l.packet_for_slice(slice_index)

                        #
                        # next slice (and maybe some debugging output)
                        #
                        slice_index = (slice_index + 1) % Packet.slices_in_rotation
                        print("{:r} ".format(packets[slice_index]))
                        if slice_index == 0:
                                #plot_rotation(packets)
                                packets = []
		except IOError,e:
			print("failed to read scanline {:x}".format(scanhdr))
