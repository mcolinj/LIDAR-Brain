from udp_channels import *

vision_ip_address = '127.0.0.1'
rio_ip_address = '127.0.0.1'

channel = UDPChannel(local_ip=rio_ip_address,
                     local_port=UDPChannel.default_remote_port,
                     remote_ip=vision_ip_address,
                     remote_port=UDPChannel.default_local_port,
                     timeout_in_seconds=30)

while 1:
    data, address = channel.receive_from()

    print data

