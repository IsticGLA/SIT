# -*- coding: utf8 -*-

from flask import Flask, jsonify, request
from flask_restful import abort
import json
import rospy
import logging
import traceback
from logging.handlers import RotatingFileHandler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import os

app = Flask(__name__)


class Drone:
    def __init__(self, label, dest_tolerance_squared):
        self.label = label
        self.dest = None
        self.position = None
        self.dest_tolerance_squared = dest_tolerance_squared
        self.path = []  # liste de dict<x,y,z>
        self.forward = True
        self.closed = False
        self.currentIndex = 0
        self.pose_sub = rospy.Subscriber(label+"/pose", PoseStamped, self.pose_callback)
        self.waypoint_pub = rospy.Publisher(label+"/waypoint", Pose, queue_size=10, latch=False)

    def pose_callback(self, pose_stamped):
        assert isinstance(pose_stamped, PoseStamped)
        pose = pose_stamped.pose
        self.position = dict([('x', pose.position.x), ('y', pose.position.y), ('z', pose.position.z)])
        if self.dest is not None:
            ex = self.dest["x"] - pose.position.x
            ey = self.dest["y"] - pose.position.y
            ez = self.dest["z"] - pose.position.z
            # verification de l'arrivée
            distance_squared = ex*ex + ey*ey + ez*ez
            if distance_squared < self.dest_tolerance_squared:
                app.logger.info("robot " + self.label + " arrived to destination")
                if len(self.path) > 1:
                    app.logger.info("robot " + self.label + " will go to next waypoint")
                    self.next_waypoint_in_path()
                else:
                    app.logger.info("robot " + self.label + " will stay there")

    def set_path(self, path, closed):
        app.logger.info("the path has " + str(len(path)) + " waypoints")
        self.path = path
        self.forward = True
        self.closed = closed
        target_index = 0
        if self.dest is not None:
            app.logger.info("finding the new waypoint closest to old destination")
            current_min_distance_squared = 10000000
            for i in range(len(path)):
                ex = self.dest["x"] - path[i]["x"]
                ey = self.dest["y"] - path[i]["y"]
                ez = self.dest["z"] - path[i]["z"]
                distance_squared = ex*ex + ey*ey + ez*ez
                if distance_squared < current_min_distance_squared:
                    current_min_distance_squared = distance_squared
                    target_index = i
        else:
            app.logger.info("drone had no destination before, going to the start of path")
        app.logger.info("going to waypoint number " + str(target_index))
        self.currentIndex = target_index
        self.goto(self.path[self.currentIndex])

    def next_waypoint_in_path(self):
        app.logger.info("setting next waypoint for robot " + self.label)
        if self.forward:
            self.currentIndex += 1
            if self.currentIndex >= len(self.path):
                if self.closed:
                    self.currentIndex = 0
                else:
                    self.currentIndex = len(self.path) - 1
                    self.forward = False
        else:
            self.currentIndex -= 1
            if self.currentIndex < 0:
                self.currentIndex = 0
                self.forward = True
        self.goto(self.path[self.currentIndex])

    def goto(self, position):
        x = position["x"]
        y = position["y"]
        z = position["z"]
        app.logger.info("setting waypoint to " + str(x) + ", " + str(y) + ", " + str(z))
        pose = Pose(position=Point(x, y, z))
        self.waypoint_pub.publish(pose)
        self.dest = position


class Controller:
    def __init__(self, nb_drones):
        self.drones = []
        for i in range(1, nb_drones + 1):
            self.drones.append(Drone("drone_" + str(i), 1))

    def set_path(self, label_drone, path, closed):
        for drone in self.drones:
            if drone.label == label_drone:
                drone.set_path(path, closed)


controller = Controller(5)


@app.route('/robot/waypoint', methods=['POST'])
def waypoint():
    global controller
    app.logger.info("received a new request on /robot/waypoint")
    if not request.json or 'x' not in request.json or 'y' not in request.json or 'z' not in request.json:
        app.logger.error("request is not json")
        abort(400)
    try:
        x = request.json['x']
        y = request.json['y']
        z = request.json['z']
        controller.set_waypoint(x, y, z)
        return jsonify({"x": x, "y": y, "z": z}), 200
    except:
        app.logger.error(traceback.format_exc())
        abort(400)


@app.route('/<path:drone_label>/path', methods=['POST'])
def set_path_for_drone(drone_label):
    global controller
    app.logger.info("received a new request on "+drone_label+"/path")
    try:
        # Get the JSON data sent from the form
        path = request.json['positions']  # list<dict<x,y,z>>
        app.logger.info("path received "+str(path))
        closed = request.json['closed']
        controller.set_path(drone_label, path, closed)
    except:
        app.logger.error(traceback.format_exc())
        abort(400)
    return "hello", 200


@app.route('/drones/info', methods=['GET'])
def get_drones_info():
    # app.logger.info("received a new request on /drones/info")
    global controller
    try:
        drones_infos = []
        for drone in controller.drones:
            info = dict([('label', drone.label), ('position', drone.position)])
            drones_infos.append(info)
        return jsonify(infos=drones_infos)
    except:
        app.logger.error(traceback.format_exc())
        abort(400)


@app.route('/hello', methods=['GET'])
def hello():
    return "hello", 200

if __name__ == '__main__':
    handler = RotatingFileHandler('flask.log', maxBytes=10000, backupCount=1)
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    app.logger.addHandler(handler)
    try:
        app.logger.info("starting ros node with ROS_IP:" + os.environ['ROS_IP'])
        rospy.init_node("node_server")
        app.logger.info("ros node started with ROS_IP:" + os.environ['ROS_IP'])
        app.run(debug=True, host='0.0.0.0', port=5000)
    except Exception as e:
        app.logger.error(traceback.format_exc())