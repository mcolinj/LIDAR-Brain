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

MM_PER_INCH = 25.4

class Field (object):
        """Fake field cartesian coordinates this is just a rectangle with a bump in the middle of an end"""
        # half width of field in inches
        field_width = 180
        # working depth of the field (12 feet or so) in inches
        field_depth = 144
        # tower width along the back wall (8 feet, but here a 1/2 width(
        tower_width = 48
        # 1/2 width of the tower face in inches
        tower_face  = 18
        def __init__ (self):
                self.origin = (0, 0)
                
                self.field_data = [(x, Field.field_depth)
                                   for x in range(-Field.field_width, -Field.tower_width)]
                self.field_data.extend((x, Field.field_depth+Field.tower_width+x)
                                       for x in range(-Field.tower_width, -Field.tower_face))
                self.field_data.extend((x, Field.field_depth-Field.tower_width+Field.tower_face)
                                       for x in range(-Field.tower_face, Field.tower_face))
                self.field_data.extend((x, Field.field_depth-Field.tower_width+Field.tower_face+x)
                                       for x in range(Field.tower_face, Field.tower_width))
                self.field_data.extend((x, Field.field_depth)
                                       for x in range(Field.tower_width, Field.field_width))
        def __getitem__ (self, index):
                return self.field_data[index]
        """Convert cartesian coordinates to polar (inches to degrees and inches"""
        #
        def cart_to_polar(x, y):
                return math.degrees(math.atan2(x, y)), math.sqrt(x*x+y*y)
        """translate the data points for the robot origin"""
        def translate(x, y, origin):
                return x - origin[0], y - origin[1]
        """rotate the data points in space by degrees to reflect robot orientation"""
        def rotate(r, theta, rot):
                return r, theta+rot
        #
        def createPolarField (self, origin, rotation):
                new_origin = (translate(x, y, origin) for x, y in iter(self))
                new_polar = (cart_to_polar(x, y) for x, y in new_origin])
                return rotate(r, theta, rotation) for r, theta in new_polar



class Reading (object):
        #
        # create the reading from two raw values
        #
        error_mask = 0x80
        warning_mask = 0x40
        #
       #
        #
        def __init__ (self, raw_distance, raw_strength):
                self._distance = raw_distance
                self._strength = raw_strength
                
        def error(self):
                return self.error_mask & _distance

        def warning(self):
                return self.warning_mask & _distance

        def distance(self):
                return ~(self.error_mask | self.warning_mask) & _distance

        def strength(self):
                return self._strength

        def distance_in_inches(self):
                """Return the distance in inches.  999 if there is an error."""
                if (self.warning || self.error):
                        return 999
                else:
                        return distance / MM_PER_INCH


#
#  Packet class is the one-stop shop for information about the packet
#  It knows how to read the packet off the wire and how to parse/decode
#  the contents.
#  A filter can be run over an array of Packets in order to produce a flattened data set
#  The filter can also be used to remove data with
#
class Packet (object):
        marker = 0xa0
        payload_length = 20
        structure_def_str = '<BB H HH HH HH HH H'
        tuple_def_str = 'Slice', 'start index speed dist0 strength0 dist1 strength1 dist2 strength2 dist3 strength3 checksum')
        
        readings = []

        # one time init for how to unpack data and tuplify it
        structure_def = struct.Struct(structure_def_str)
        tuple_def = collections.namedtuple(tuple_def_str)                

        def __init__(self, packed_data):
                """Create a packet from the 22byte serial lidar data"""
                unpacked_data = structure_def.unpack(packed_data)
                packet_tuple = tuple_def._make(data)

                self.index = packet_tuple.index

                # initialize the readings for this Packet
                index_degrees = 4*index
                self.readings = [ index_degrees+0, Reading(packet_tuple.dist0, packet_tuple.strength0),
                                  index_degrees+1, Reading(packet_tuple.dist1, packet_tuple.strength1),
                                  index_degrees+2, Reading(packet_tuple.dist2, packet_tuple.strength2),
                                  index_degrees+3, Reading(packet_tuple.dist3, packet_tuple.strength3) ]

                self.speed = packet_tuple.speed

        def rpm(self):
                """Speed is reported in 64ths of an RPM"""
                return self.speed / 64;

        def print_data(self):
                """Slice index i is for degrees 4*i, 4*i+1, 4*i+2, and 4*i+3"""
                for i in range(4):
                        print("{:d} ")

        #
        # Checksum algorithm runs when we find the data marker
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
        #  do the scan line.
        #  <start> <index> <speed_L> <speed_H> [Data 0] [Data 1] [Data 2] [Data 3] <checksum_L> <checksum_H>
        #
	def packet_for_slice(self,slice):
                """read in a packet and return the constructed packet"""
		headerpos=0
		scanline=0
		insync=0
		rpms=0	
		scandist=[]
		scanbytes=[]
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
                scanba=bytearray(22)

                # 
		while insync == 0:
			nbytes=self.laserport.readinto(scanba[0:1]))
			if nbytes == 1 and scanba[0] == Packet.marker:
				nbytes=self.laserport.readinfo(scanba[1:2])
				if nbytes == 1 and scanba[1] == slice:
					insync=1
                #
                # We got a marker value and a matching slice value
                # Read the remaining bytes and do the checksum calculation
                #
		if insync == 1:
			try:
                                # read the final 20 bytes
                                nbytes=self.laserport.readinto(scanba[2:22])
                                if nbytes == 20:
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

        
if __name__ == '__main__':
	
        # open up the serial port device connected to the lidar
        try: 	
		lp= serial.Serial('/dev/tty.usbserial',115200,timeout=1)
	except: 
		print("Could not open serial port for laser!")
		exit()

        # connect the port with the laser and initialize the laser object
        l = laser(lp)

        #
        #
        #
	scanrange=[]
	distdata=[]
	slice_index=0
	while 1: 
		try: 
			packets[slice_index]=l.packet_for_slice(slice_index)

                        items = []
                        for si in xrange(1,srlen,2):
				disterr=l.errbits(scanrange[si])
				if disterr == 0:
					distdata=scanrange[si-1],scanrange[si]
                                        items.append("{:04}".format(l.bytes_distance(distdata)))
                                else:
                                        items.append('0000')

                        print(', '.join(items))
	                                
			if scanhdr == 0xf9:
				scanhdr=0xa0
			else:
				scanhdr+=1 
		except IOError,e:
			print("failed to read scanline {:x}".format(scanhdr))
