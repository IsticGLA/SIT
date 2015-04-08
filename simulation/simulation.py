from flask import Flask, jsonify, request
from flask_restful import abort
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

class Command:
    def __init__(self):
        self.cmd = rospy.Publisher("/mav/waypoint", Pose, queue_size=10, latch=True)

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
    if not request.json or not 'x' in request.json or not 'y' in request.json or not 'z' in request.json:
        abort(400)
    
    x = request.json['x']
    y = request.json['y']
    z = request.json['z']
    command.setWaypoint(x, y, z)
    
    return jsonify({"x": x, "y": y, "z": z}), 201

if __name__ == '__main__' :
    rospy.init_node("flask")
    app.run(debug=True, host='0.0.0.0', port=5000)