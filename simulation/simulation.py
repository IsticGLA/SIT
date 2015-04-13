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

app = Flask(__name__)

class Controler:
    def __init__(self):
        # Souscris pour écouter la position du robot
        self.pose_sub = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        # Publie pour setter le waypoint du robot
        self.waypoint_pub = rospy.Publisher("/waypoint", Pose, queue_size=10, latch=False)
        self.dest_tol=1

    def setWaypoint(self, x, y, z):
        app.logger.info("setting waypoint to " + str(x) +", "+str(y)+", "+str(z))
        pose = Pose(position=Point(x,y,z))
        self.waypoint_pub.publish(pose)

    def setPath(self, path):
        self.path = path
        self.currentIndex = 0
        self.setWaypoint(path[0]["x"], path[0]["y"], path[0]["z"])

    def nextWaypointInPath(self):
        self.currentIndex = self.currentIndex+1
        if self.currentIndex >= len(self.path):
            self.currentIndex = 0
        point = self.path[self.currentIndex]
        self.setWaypoint(point["x"], point["y"], point["z"])

    # appellée a chaque mise a jour de la position du robot
    def pose_callback(self, pose_stamped):
        app.logger.info("hey")
        assert isinstance(pose_stamped, PoseStamped)
        pose = pose_stamped.pose
        
        app.logger.info("robot position " + str(pose.position.x) +", "+str(pose.position.y)+", "+str(pose.position.z))
        ez = self.dest.z - pose.position.z
        ex = self.dest.x - pose.position.x
        ey = self.dest.y - pose.position.y

        # verification de l'arrivée
        distance = sqrt(ex*ex + ey*ey + ez*ez)
        if distance < self.dest_tol:
            app.logger.info("robot arrived")

controler = Controler()

@app.route('/robot/waypoint', methods=['POST'])
def waypoint():
    global command
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
    controler.setWaypoint(x, y, z)
    return jsonify({"x": x, "y": y, "z": z}), 200

@app.route('/robot/path', methods=['POST'])
def path():
    global command
    app.logger.info("received a new request on /robot/path")
    try:
        # Get the JSON data sent from the form
        path = request.json['data'] # list<dict<x,y,z>>
        controler.setWaypoint(path[0]["x"], path[0]["y"], path[0]["z"])
    except Exception as e:
        app.logger.error(traceback.format_exc())
        abort(400)
    return "hello", 200

if __name__ == '__main__' :
    handler = RotatingFileHandler('flask.log', maxBytes=10000, backupCount=1)
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    app.logger.addHandler(handler)
    app.logger.info("starting ros node")
    rospy.init_node("flask")
    app.logger.info("ros node started")
    app.run(debug=True, host='0.0.0.0', port=5000)

