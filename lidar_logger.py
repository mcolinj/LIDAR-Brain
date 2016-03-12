class LidarLogger ():
    def __init__ (self, logger):
        self.logger = logger

    def log_data(self, polar_data):
        """Export lidar data to a file"""
        self.logger.info("BEGIN Full rotation of sanitized lidar data")
        for theta,radius in polar_data:
            self.logger.info("{:d}, {:.1f}\n".format(theta, radius))
        self.logger.info("END Full rotation of sanitized lidar data")

    @staticmethod
    def write_to_file(file_name, polar_data):
        with open(file_name, 'w') as f:
            for (theta, distance) in polar_data:
                f.write("{:.1f}, {:.1f}\n".format(theta, distance))

