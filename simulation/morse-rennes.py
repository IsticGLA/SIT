from morse.builder import *
from math import pi

# create a quadrobot
drone_1 = Quadrotor()
drone_1.translate(x=9.5, y=-200, z=0.5)
#drone_1.rotate(z=pi/3)

# add camera to the drone_1
camera = VideoCamera()
camera.translate(z=-0.1)
camera.rotate(y= -90 / 180 * pi)
drone_1.append(camera)
camera.add_interface("ros")

# use waypoint to control the drone_1
waypoint = RotorcraftWaypoint()
drone_1.append(waypoint)
waypoint.add_interface('ros', topic="/waypoint")

# use a pose sensor to know where the drone_1 is
pose = Pose()
drone_1.append(pose)
pose.add_interface('ros', topic="/pose")
drone_1.add_default_interface('ros')

env = Environment('simulation/map/rennes1.blend')
env.set_camera_location([109.6937, -240.998, 60])
env.set_camera_rotation([70 / 180 * 3.14, 0 / 180 * 3.14, 60 / 180 * 3.14])
env.set_camera_clip(clip_start=1, clip_end=500)

env.select_display_camera(camera)