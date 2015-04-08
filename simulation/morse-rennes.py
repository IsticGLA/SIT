from morse.builder import *

from math import pi

# create a quadrobot
drone_1 = Quadrotor()
drone_1.translate(x=-7.0, z=3.0)
drone_1.rotate(z=pi/3)

# add camera to the drone_1
camera = VideoCamera()
camera.translate(z=-0.05)
camera.rotate(x=+0.2)
drone_1.append(camera)
camera.add_interface("ros")

# use waypoint to control the drone_1
waypoint = RotorcraftWaypoint()
drone_1.append(waypoint)
waypoint.add_interface('ros', topic="/waypoint")

# use a pose sensor to know where the drone_1 is
catPose = Pose()
drone_1.append(catPose)
catPose.add_stream('ros', topic="/catposition")
drone_1.add_default_interface('ros')

env = Environment('simulation/map/rennes1.blend')
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.0470, 0, 0.7854])
env.set_camera_clip(clip_start=1, clip_end=500)

env.select_display_camera(camera)