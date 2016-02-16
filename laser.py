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

from multiprocessing import Process
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
import pylab
from time import sleep

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

        @property
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
                return "{:d},{:.2f}".format(self.heading, self.distance_in_inches)


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
        #  Decode the rpd value 
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
                                                                      self.readings[0].distance_in_inches,
                                                                      self.readings[1].distance_in_inches,
                                                                      self.readings[2].distance_in_inches,
                                                                      self.readings[3].distance_in_inches)
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


# could easily be a laser/lidar method, huh?
def gather_lidar_rotation(laser):
        """Read a rotation of lidar data and return it"""
        rotation = []
        slice_index = 0
        
        for slice_index in range(Packet.slices_in_rotation):
                try:
                        if lasr.laserport == None:
                                # generate synthetic packet
                                packet_str  = "fb{:02x}0040fe002200fc014400f803660077808800abcd".format(slice_index+Packet.index_offset)
                                packet = Packet(binascii.unhexlify(packet_str))
                        else:
	                        packet = laser.packet_for_slice(slice_index)
                        rotation.append(packet)
                except:
                        print("Unable to get packet for data slice with index {:d}.".format(slice_index))

        if lasr.laserport == None:
                # sleep like it took the app
                elapsed_time = 60.0 / float(packet.rpm) 
                print("Sleeping for {:02f} seconds".format(elapsed_time))
                sleep(elapsed_time)

        # return either the real or the synthetic rotation
        return rotation

def export_rotation_to_file(rotation, file_name):
        """open file_name file and write the rotation data to it"""
        text_file = open(file_name, "w")
        for p in rotation:
                text_file.write(p.as_data())
        # put rotation.str_data into file
        text_file.close()


#
#   Open up the serial port, get lidar data and write it to a file
#   every few seconds.
#
if __name__ == '__main__':

        lp = None
        
        # open up the serial port device connected to the lidar
        try: 	
		lp = serial.Serial('/dev/tty.usbserial',115200,timeout=1)
        except: 
		print("Could not open serial port for laser!")

        # connect the port with the laser and initialize the laser object
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
                        rotation = gather_lidar_rotation(lasr)
 		except IOError,e:
			print("failed to gather lidar rotation")

                #
                # Here we could read the input work queue looking for requests
                # And formulate answers to send to the requestor
                # For now, we just output a lidar rotation every 10 seconds or so
                #
                elapsed_time = (SECONDS_PER_MINUTE/float(rotation[0].rpm))
                rotation_time = rotation_time + elapsed_time
                current_time = current_time + elapsed_time
                if rotation_time > seconds_per_output:
                        # write out about every 10 seconds to output
                        file_name = "lidar{:04d}.dat".format(file_index)
                        print("Time: {:.2f} Exporting current rotation of data to {:s}".format(current_time, file_name))
                        export_rotation_to_file(rotation, file_name)
                        file_index = file_index + 1
                        rotation_time = 0
