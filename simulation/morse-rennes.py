from morse.builder import *
from math import pi

# create a quadrobot
drone_1 = Quadrotor()
drone_1.translate(x=9.5, y=-200, z=30)
drone_2 = Quadrotor()
drone_2.translate(x=9.5, y=-205, z=30)
drone_3 = Quadrotor()
drone_3.translate(x=9.5, y=-210, z=30)
drone_4 = Quadrotor()
drone_4.translate(x=9.5, y=-215, z=30)
drone_5 = Quadrotor()
drone_5.translate(x=9.5, y=-220, z=30)
drones = [drone_1, drone_2, drone_3, drone_4, drone_5]

i = 1
for drone in drones:
    drone.name = "drone_" + str(i)
    i += 1
    camera = VideoCamera()
    camera.translate(z=-0.1)
    camera.rotate(y=-90 / 180 * pi)
    drone.append(camera)

    waypoint = RotorcraftWaypoint()
    drone.append(waypoint)

    pose = Pose()
    drone.append(pose)

    drone.add_default_interface('ros')

env = Environment('simulation/map/rennes1.blend')
env.set_camera_location([109.6937, -240.998, 60])
env.set_camera_rotation([70 / 180 * 3.14, 0 / 180 * 3.14, 60 / 180 * 3.14])
env.set_camera_clip(clip_start=1, clip_end=500)

env.select_display_camera(camera)