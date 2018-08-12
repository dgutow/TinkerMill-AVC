from rplidar import RPLidar

#PORT_NAME = '/dev/ttyUSB0'

class LIDAR(object):
    def __init__(self, portname):
        self.portname = portname

    def scan(self):
        pointCloud = []
        lidar = RPLidar(self.portname)

        for i, measurement in enumerate(lidar.iter_measures()):
            for j, v in enumerate(measurement):
                if j == 0:
                    newscan = v
                if j == 1:
                    quality = v
                if j == 2:
                    angle = v
                if j == 3:
                    length = v

            # change angle to match orientation of vehicle
            angle = -1 * (angle - 90)

            pointCloud.append([angle, length])

            if i > 360:
               break
               
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

        return pointCloud
