# Karlsruhe OSM map is exported from 49.01214 (N) -> 49.00819 (S)
#                                     8.40926 (W) ->  8.41684 (E)

from morse.builder import *
from math import pi

#robot = Pioneer3DX()
#motion = MotionVWDiff()

robot = ATRV()
motion = MotionVW()

motion.translate(z=0.5)
robot.append(motion)

pose = Pose()
pose.translate(z=0.17)
robot.append(pose)

# Append an Odometry sensor
odometry = Odometry()
odometry.translate(z=0.20)
robot.append(odometry)

# Append a camera
camera = VideoCamera()
camera.rotate(z = 0*pi)
camera.rotate(y = 0.1)
camera.translate(x = 0.25, z = 0.20)
camera.properties(cam_width = 640, cam_height = 480)
camera.properties(cam_near=0.1, cam_far=500)
camera.frequency(15)
robot.append(camera)

robot.translate(z=0.5, y=10, x=-5)

#middleware = 'socket'
middleware = 'ros'

motion.add_stream(middleware)
pose.add_stream(middleware)
pose.add_service(middleware)
odometry.add_stream(middleware)
camera.add_stream(middleware)

# ***.add_interface(middleware)



uav = Quadrotor()
uav.translate(z=1.5, y=12, x=-5)

# # creates a new instance of the actuator
# waypoint = RotorcraftWaypoint()
# waypoint.properties(MaxBankAngle=0.25, HorizontalPgain=0.104*1.0*0.5,
#                     HorizontalDgain=0.139*1.1*0.5)
# # place your component at the correct location
# waypoint.translate(0, 0, 0)
# waypoint.rotate(0, 0, 0)
#
# uav.append(waypoint)

controler = RotorcraftVelocity()
uav.append(controler)

pose = Pose()
#pose.translate(z=0.17)
uav.append(pose)

# waypoint.add_overlay('ros', 'morse.middleware.ros.overlays.waypoints.WayPoint')

camera = VideoCamera()
camera.rotate(z = 0*pi)
camera.rotate(y = -0.3)
camera.translate(x = 0.05, z = -0.10)
camera.properties(cam_width = 640, cam_height = 480)
camera.properties(cam_near=0.1, cam_far=500)
camera.frequency(15)
uav.append(camera)

# define one or several communication interface, like 'socket'
#waypoint_uav.add_interface('ros')
#camera_uav.add_interface('ros')
uav.add_default_interface(middleware)

#waypoint.add_service('ros')

#env = Environment('indoors-1/indoor-1')
#env = Environment('karlsruhe_small.blend')

#env = Environment('indoors-1/boxes')
env = Environment('land-1/trees')
#env = Environment('apartment')
#env = Environment('tum_kitchen/tum_kitchen')
#env = Environment('outdoors')
#env = Environment('laas/grande_salle')
#env = Environment('water-1/deep_water')
#env = Environment('water-1/water_scene')

# env.set_camera_location([5, -5, 6])
# env.set_camera_rotation([1.047, 0, 0.785])

env.set_camera_clip(clip_start=0.1, clip_end=500)

env.select_display_camera(camera)
#env.set_physics_step_sub(5)
env.fullscreen(False)
