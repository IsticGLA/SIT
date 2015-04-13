from flask import Flask, jsonify, request
from flask_restful import abort
import rospy
import logging
from logging.handlers import RotatingFileHandler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class Command:
    def __init__(self):
        self.cmd = rospy.Publisher("/waypoint", Pose, queue_size=10, latch=False)

    def setWaypoint(self, x, y, z):
        pose = Pose(position=Point(x,y,z))
        self.cmd.publish(pose)

command = Command()
app = Flask(__name__)

@app.route('/robot/rotate', methods=['POST'])
def rotate() :
    if not request.json or not 'd' in request.json:
        abort(400)
    
    d = request.json['d']
    
    return jsonify({"d": d}), 201

@app.route('/robot/waypoint', methods=['POST'])
def waypoint():
    global command
    app.logger.info("received a new waypoint")
    if not request.json or not 'x' in request.json or not 'y' in request.json or not 'z' in request.json:
        app.logger.error("request is not json")
        abort(400)
    try:
        x = request.json['x']
        y = request.json['y']
        z = request.json['z']
    except:
        app.logger.error("bad request, " + str(x) +" " +str(y) +" " +str(z))
        abort(400)
    app.logger.info("setting waypoint to "+str(x)+", "+str(y)+", "+str(z))
    command.setWaypoint(x, y, z)
    return jsonify({"x": x, "y": y, "z": z}), 201

if __name__ == '__main__' :
    handler = RotatingFileHandler('flask.log', maxBytes=10000, backupCount=1)
    handler.setLevel(logging.DEBUG)
    app.logger.addHandler(handler)
    rospy.init_node("flask")
    app.run(debug=True, host='0.0.0.0', port=5000)
