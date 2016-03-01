from __future__ import print_function
from laser import *
import binascii
import pdb
from udp_channels import UDPChannel
from field_model import FieldModel, Robot, FakeRotation
from laser import *
from analyzer import Analyzer, r_squared
import math

def test_reading():
    """unit test of the reading object to make sure it continues to work"""
    r1 = Reading(90, 2540, 0x2000)
    assert r1.range == 2540
    assert not r1.error
    assert not r1.warning
    assert not r1.discard
    assert int(round(r1.range_in_inches)) == 100
    assert str(r1) == "90,100.00"
    
    r2 = Reading(240, 0x8123, 100)
    assert r2.error
    assert r2.discard
    assert r2.range_in_inches == 777
    
    r3 = Reading(240, 0x4000, 200)
    assert r3.warning
    assert r3.discard
    
def test_packet():
    """Create a lidar packet and make sure the packet parsing works okay"""
    p1 = Packet(binascii.unhexlify("fba50040fe002200fc014400f803660077808800abcd"))
    assert p1.index == 5
    assert p1.rpm == 256
    assert str(p1) == "5 256 10.00 20.00 40.00 777.00"
    assert p1.as_data() == "20,10.00\n21,20.00\n22,40.00\n23,777.00\n"


def test_udp_channel():
    """Create a simple two-way communication channel and make sure it sends and receives"""
    local  = UDPChannel()
    remote = UDPChannel(local_port=local.remote_port, remote_port=local.local_port)

    local_message = "Hello, Remote, I am Local."
    local.send_to(local_message)
    data, reply_address = remote.receive_from()
    assert data == local_message
    assert reply_address[0] == local.local_ip

    # Now the remote can reply, and the sender can recognize it as a reply
    remote.reply_to("ACK "+data, reply_address)

    # original sender uses special receive_reply
    data2, reply_address2 = local.receive_reply()
    assert data2 == "ACK "+local_message

    try:
        data, addr = local.receive_from()
        assert False, "Should have timed out, but did not."
    except:
        print("local.receive_from() timed out")

def test_analzyer_on_field_model():
    """
    Create a model of the field, then ask the laser to compute
    the range for sweep.  Then move the robot and ask again.
    """
    robot = Robot()
    field = FieldModel()
    tower_range = field.tower_range_from_origin()
    rotation = FakeRotation(field, robot)
    heading, sweep_range = Analyzer.range_at_heading(rotation.polar_data(), (-10,11))

    #  closest point should be dead ahead
    assert heading == 0
    assert sweep_range == tower_range

    movement = 100
    robot.move(movement)
    rotation = FakeRotation(field, robot)
    heading, sweep_range = Analyzer.range_at_heading(rotation.polar_data(), (-10,11))

    #  closest point should be dead ahead
    assert heading == 0
    assert sweep_range == (tower_range - movement)


def test_sensor_messages():
    sm1 = SensorMessage(sender='foobar', message='hi')
    sm2 = SensorMessage(sender='tester', message='howdy')

    as_string = sm1.encode_message()
    sm2 = SensorMessage.create_from_message(as_string)

    assert sm1.__dict__ == sm2.__dict__

    lrah1 = LidarRangeAtHeadingMessage()
    lrah2 = LidarRangeAtHeadingMessage()
    assert lrah2.__dict__ == lrah1.__dict__

    lrah2.range = 42
    lrah1.range = 41
    assert lrah2.__dict__ != lrah1.__dict__

    lrah1.range = 42
    assert lrah2.__dict__ == lrah1.__dict__

    lrah1.heading = 45
    lrah2.heading = 90
    assert lrah2.__dict__ != lrah1.__dict__

    lrah1.heading = 90
    assert lrah2.__dict__ == lrah1.__dict__
    assert lrah1.encode_message() == lrah2.encode_message()
    

def test_r_squared():
    #  simple 45 degree line
    data = [(1,1), (2,2), (3,3), (4,4)]
    r2 = r_squared(data)
    assert round(1000*r2) == 1000

    #  simple -45 degree line
    data = [(-1,1), (-2,2), (-3,3), (-4,4)]
    r2 = r_squared(data)
    assert round(1000*r2) == 1000

    #  horizontal line
    data = [(1,5), (2,5), (3,5), (4,5)]
    r2 = r_squared(data)
    #print r2
    #assert round(1000*r2) == 1000

    #  vertical line
    data = [(5,1), (5,2), (5,3), (5,4)]
    r2 = r_squared(data)
    #print r2
    #assert r2 == 0


    #  more interesting line
    data = [(1,1), (2,2), (3,4), (4,1), (7,8), (12,14)]
    r2 = r_squared(data)
    #print r2


    #  now let's try from the field model  (what is the r_squared on the whole thing?)
    f = FieldModel()
    rotation = FakeRotation(f)
    cart_data = rotation.cartesian_data()
    for i in range(0, len(cart_data)-4):
        slice = cart_data[i:i+4]
        r2 = r_squared(slice)
        print("{:.2f}".format(r2),end=" => ")
        print(slice)
        
    
