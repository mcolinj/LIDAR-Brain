from laser import *
import binascii
import pdb

def test_reading():
    """unit test of the reading object to make sure it continues to work"""
    r1 = Reading(90, 2540, 0x2000)
    assert r1.distance == 2540
    assert not r1.error
    assert int(round(r1.distance_in_inches)) == 100
    assert str(r1) == "90,100.00"
    r2 = Reading(240, 0x8123, 100)
    assert r2.error
    assert r2.distance_in_inches == 777
    r3 = Reading(240, 0x4000, 200)
    assert r3.warning

def test_packet():
    """Create a lidar packet and make sure the packet parsing works okay"""
    p1 = Packet(binascii.unhexlify("fba50040fe002200fc014400f803660077808800abcd"))
    assert p1.index == 5
    assert p1.rpm == 256
    assert str(p1) == "5 256 10.00 20.00 40.00 777.00"
    assert p1.as_data() == "20,10.00\n21,20.00\n22,40.00\n23,777.00\n"


def test_udp_communication_channel():
    """Create a simple two-way communication channel and make sure it sends and receives"""
    local  = UDPCommunicationChannel()
    remote = UDPCommunicationChannel(local_port=local.remote_port, remote_port=local.local_port)

    local_message = "Hello, Remote, I am Local."
    local.send_to(local_message)
    data, reply_address = remote.receive_from()
    assert data == local_message
    assert reply_address[0] == local.local_ip

    # Now the remote can reply, and the sender can recognize it as a reply
    remote.reply_to("ACK "+data, reply_address)

    # original sender uses special receive_reply
    data, reply_address = local.receive_reply()
    assert data == "ACK "+local_message
                        
