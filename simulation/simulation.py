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

class Controller:
    def __init__(self):
        # Souscris pour écouter la position du robot
        self.pose_sub = rospy.Subscriber("/drone_1/pose", PoseStamped, self.pose_callback)
        # Publie pour setter le waypoint du robot
        self.waypoint_pub = rospy.Publisher("/drone_1/waypoint", Pose, queue_size=10, latch=False)
        self.dest=None
        self.dest_tol_squared=1 #tolérance de 1m (valeur au carré)
        self.forward = True
        self.currentIndex = 0

    def setWaypoint(self, x, y, z):
        app.logger.info("setting waypoint to " + str(x) +", "+str(y)+", "+str(z))
        pose = Pose(position=Point(x,y,z))
        self.waypoint_pub.publish(pose)
        self.dest = Point(x,y,z)

    def setPath(self, path):
        self.path = path
        self.currentIndex = 0
        self.setWaypoint(path[0]["x"], path[0]["y"], path[0]["z"])
        self.dest = path[0]
        self.forward = True

    def setClosed(self, closed):
        self.closed = closed

    def nextWaypointInPath(self):
        if self.forward:
            self.currentIndex = self.currentIndex + 1
            if self.currentIndex >= len(self.path):
                if self.closed:
                    self.currentIndex = 0
                else:
                    self.currentIndex = len(self.path) - 1
                    self.forward = False
        else:
            self.currentIndex = self.currentIndex - 1
            if self.currentIndex < 0:
                self.currentIndex = 0
                self.forward = True

        point = self.path[self.currentIndex]
        self.setWaypoint(point["x"], point["y"], point["z"])
        self.dest = point

    # appellée a chaque mise a jour de la position du robot
    def pose_callback(self, pose_stamped):
        assert isinstance(pose_stamped, PoseStamped)
        pose = pose_stamped.pose
        #app.logger.info("robot position " + str(pose.position.x) +", "+str(pose.position.y)+", "+str(pose.position.z))
        if self.dest is not None:
            #app.logger.info("dest  position " + str(self.dest["x"]) + ", "+str(self.dest["y"])+", "+str(self.dest["z"]))
            ez = self.dest["z"] - pose.position.z
            ex = self.dest["x"] - pose.position.x
            ey = self.dest["y"] - pose.position.y
            #app.logger.info("deltas " + str(ex) + ", "+ str(ey) + ", "+str(ez))
            # verification de l'arrivée
            distance_squared = ex*ex + ey*ey + ez*ez
            #app.logger.info("distance_squared : "+ str(distance_squared))
            if distance_squared < self.dest_tol_squared:
                app.logger.info("robot arrived to waypoint")
                self.nextWaypointInPath()


controller = Controller()

@app.route('/robot/waypoint', methods=['POST'])
def waypoint():
    global controller
    app.logger.info("received a new request on /robot/waypoint")
    if not request.json or not 'x' in request.json or not 'y' in request.json or not 'z' in request.json:
        app.logger.error("request is not json")
        abort(400)
    try:
        x = request.json['x']
        y = request.json['y']
        z = request.json['z']
    except Exception as e:
        app.logger.error(traceback.format_exc())
        abort(400)
    controller.setWaypoint(x, y, z)
    return jsonify({"x": x, "y": y, "z": z}), 200

@app.route('/robot/path', methods=['POST'])
def path():
    global command
    app.logger.info("received a new request on /robot/path")
    try:
        # Get the JSON data sent from the form
        path = request.json['positions'] # list<dict<x,y,z>>
        closed = request.json['closed']
        controller.setPath(path)
        controller.setClosed(closed)
    except Exception as e:
        app.logger.error(traceback.format_exc())
        abort(400)
    return "hello", 200

@app.route('/drones/info', methods=['GET'])
def dronesInfos():
    app.logger.info("received a new request on /drones/info")
    try:
        drones_infos = []
        info = dict([('label', "drone_1"), ('x', 4127), ('y', 4098)])
        drones_infos.append(info)
        return jsonify(infos = drones_infos)
    except Exception as e:
        app.logger.error(traceback.format_exc())
        abort(400)
    

@app.route('/hello', methods=['GET'])
def hello():
    return "hello", 200

if __name__ == '__main__' :
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

