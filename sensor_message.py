import json

class SensorMessage(object):
    """
    Base class for sensor message
    Provides the default initialization for the common
    elements.
    """
    def __init__(self, sender, message):
        self.sender = sender
        self.message = message

    def encode_message(self):
        return json.dumps(self.__dict__)

    @classmethod
    def create_from_message(cls, message_as_string):
        contents = json.loads(message_as_string)
        return cls(contents['sender'], contents['message'])


#
#  To create a simple message type, just follow the model
#  below.
#  Use the SensorMessage as a base class, set the default values
#  for your sender and message.   Do the funky initialization.
#  Initialize the additional attributes for your message.
#
class LidarRangeAtHeadingMessage(SensorMessage):
    """
    Convenience class for sending sensor messages.
    To send a message to the robo rio, just do the following:

    range_at_heading = LidarRangeAtHeadingMessage()
    range_at_heading.range = <range>
    range_at_heading.heading = <heading>

    channel_to_rio.send_to(range_at_heading.encode_message())
    """
    def __init__(self, sender="lidar", message="range at heading"):
        super(LidarRangeAtHeadingMessage,self).__init__(sender, message)
        if message != 'range at heading':
            print("Error in factory method")
        self.range = 0
        self.heading = 0

    
class LidarPeriodicMessage(SensorMessage):
    """
    Convenience class for sending periodic messages.

    To send a message to the robo rio, just do the following:

    periodic = LidarPeriodicMessage()
    periodic.rpm = <rpm>

    channel_to_rio.send_to(range_at_heading.encode_message())
    """
    def __init__(self, name="lidar", message="periodic"):
        super(LidarPeriodicMessage,self).__init__(name, message)
        self.rpm = 0
        self.status = 'ok'

class LidarWallMessage(SensorMessage):
    """
    Convenience class for sending wall messages.

    To send a message to the robo rio, just do the following:

    wall_message = LidarWallMessage()
    wall_message.status = 'ok'
    wall_message.orientation = <degrees to turn to be parallel to wall>
    wall_message.heading = <degree heading to midpoint of wall>
    wall_message.range = <range to the midpoint>

    channel_to_rio.send_to(wall_message.encode_message())
    """
    def __init__(self, name="lidar", message="wall"):
        super(LidarWallMessage,self).__init__(name, message)
        self.orientation = 0
        self.status = 'ok'
        self.heading = 0
        self.range = 0

class RobotMessage(object):
    """
    Convenience class for receiving and cracking messages from
    the robot.
    """
    def __init__(self, message_as_string):
        self.contents = json.loads(message_as_string)
    
    def __get_item__(self, item):
        return self.contents[item]
