from morse.builder import *

from math import pi

# create a quadrobot, the cat
cat = Quadrotor()
cat.translate(x=-7.0, z=3.0)
cat.rotate(z=pi/3)

# add camera to the cat
camera = VideoCamera()
camera.translate(x=0.3, z=-0.05)
camera.rotate(x=+0.2)
cat.append(camera)
camera.add_interface("ros")

# use waypoint to control the cat
waypoint = RotorcraftWaypoint()
cat.append(waypoint)
waypoint.add_interface('ros', topic="/waypoint")

# use a pose sensor to know where the cat is
catPose = Pose()
cat.append(catPose)
catPose.add_stream('ros', topic="/catposition")
cat.add_default_interface('ros')

env = Environment('test-paris.blend')
env.set_camera_location([10.0, -10.0, 10.0])
env.set_camera_rotation([1.0470, 0, 0.7854])
env.set_camera_clip(clip_start=1, clip_end=500)

env.select_display_camera(camera)
