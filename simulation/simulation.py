# -*- coding: utf8 -*-

from flask import Flask, jsonify, request
from flask_restful import abort
#import http.client
import json
import rospy
import logging
import traceback
import numpy as np
from logging.handlers import RotatingFileHandler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
import cv2
import os
import urllib
import urllib2
from urllib2 import Request, urlopen, URLError, HTTPError
from cv_bridge import CvBridge, CvBridgeError
import base64
import time, threading

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
        self.pose_sub = None
        self.waypoint_pub = rospy.Publisher(label+"/waypoint", Pose, queue_size=1, latch=True)
        self.camera_sub_image = None # pour la prise de photo au point
        self.camera_sub_video = None # pour la prise de photo régulière
        self.auto_reactivate = False
        self.bridge = CvBridge()

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
                    if self.pictureActivated:
                        self.take_picture(self.dest)
                    app.logger.info("robot " + self.label + " will go to next waypoint")
                    self.next_waypoint_in_path()
                else:
                    app.logger.info("robot " + self.label + " will stay there")
                    self.stop()
        elif self.path is not None and len(self.path) > 0:
            app.logger.warn("robot " + self.label + "has no destination")

    def take_picture(self, position):
        app.logger.info("activating picture callback")
        self.picture_position = position
        if self.camera_sub_image is not None:
            self.camera_sub_image.unregister()
        self.camera_sub_image = rospy.Subscriber(self.label+"/camera/image", Image, self.picture_callback, queue_size=1)

    def activate_video(self):
        if self.camera_sub_video is not None:
            self.camera_sub_video.unregister()
        self.camera_sub_video = rospy.Subscriber(self.label+"/camera/image", Image, self.video_callback, queue_size=1)

    def desactivate_video(self):
        if self.camera_sub_video is not None:
            self.camera_sub_video.unregister()
        self.camera_sub_video = None

    def picture_callback(self, ros_data):
        self.camera_sub_image.unregister()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            encoded_img = cv2.imencode(".jpeg", cv_image)
            data = base64.b64encode(encoded_img[1].tostring())
            self.postImage(data, False)
        except:
            app.logger.error(traceback.format_exc()) 

    def video_callback(self, ros_data):
        self.desactivate_video()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            encoded_img = cv2.imencode(".jpeg", cv_image)
            data = base64.b64encode(encoded_img[1].tostring())
            self.postImage(data, True)
        except:
            app.logger.error(traceback.format_exc())
        time.sleep(1)
        if self.auto_reactivate == True:
            self.activate_video()

    def postImage(self, image, isForVideo):
        try:
            myurl = None
            position = None
            if isForVideo == True:
                myurl = "http://37.59.58.42:8080/nivimoju/rest/image/video"
                position = self.position
            else:
                myurl = "http://37.59.58.42:8080/nivimoju/rest/image/create"
                position = self.picture_position
            body = {'base64Image':image,
                        'position': position,
                        'droneLabel': self.label}
            jsondata = json.dumps(body)
            req = urllib2.Request(myurl)
            req.add_header('Content-Type', 'application/json; charset=utf-8')
            jsondataasbytes = jsondata.encode('utf-8')   # needs to be bytes
            req.add_header('Content-Length', len(jsondataasbytes))
            response = urllib2.urlopen(req, jsondataasbytes) 
            app.logger.debug("image posted on " + myurl + " with response : " + str(response.info))
        except HTTPError as e:
            app.logger.error('The server couldn\'t fulfill the request. ')
            app.logger.error('Error code: ' + str(e.code))
        except URLError as e:
            app.logger.error('We failed to reach a server.')
            app.logger.error('Reason: ' + str(e.reason))
        except:
            app.logger.error(traceback.format_exc()) 

    def set_path(self, path, closed, pictureActivated):
        app.logger.info("the path has " + str(len(path)) + " waypoints")
        self.path = path
        self.forward = True
        self.closed = closed
        self.pictureActivated = pictureActivated
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
        if len(self.path) < 1:
            app.logger.warn("the path has no waypoint to go to : " + str(path))
        else:
            app.logger.info("going to waypoint number " + str(target_index))
            self.currentIndex = target_index
            self.goto(self.path[self.currentIndex])
        self.activate_video()
        self.auto_reactivate = True

    def next_waypoint_in_path(self):
        app.logger.info("setting next waypoint for robot " + self.label)
        if self.closed or len(self.path) <= 2:
            self.currentIndex = (self.currentIndex + 1) % len(self.path)
        else:
            if self.forward:
                self.currentIndex += 1
                if self.currentIndex >= len(self.path):
                     app.logger.info(self.label + " reached end of path, going back")
                     self.currentIndex = (len(self.path) - 2) % len(self.path)
                     self.forward = False
            else:
                self.currentIndex -= 1
                if self.currentIndex < 0:
                    app.logger.info(self.label + " reached start of path, going forward")
                    self.currentIndex = 1 % len(self.path)
                    self.forward = True
        app.logger.debug("current index " + str(self.currentIndex+1) +
                         " forward : " + str(self.forward))
        self.goto(self.path[self.currentIndex])

    def goto(self, position):
        x = position["x"]
        y = position["y"]
        z = position["z"]
        app.logger.info("publishing waypoint to point" + str(x) + ", " + str(y) + ", " + str(z))
        pose = Pose(position=Point(x, y, z))
        self.waypoint_pub.publish(pose)
        self.dest = position
        if self.pose_sub is None:
            self.pose_sub = rospy.Subscriber(self.label+"/pose", PoseStamped, self.pose_callback)

    def stop(self):
        self.__init__(self.label, self.dest_tolerance_squared)
        if self.pose_sub is not None:
            self.pose_sub.unregister()
        if self.camera_sub_image is not None:
            self.camera_sub_image.unregister()
        self.pose_sub = None
        self.camera_sub_image = None
        self.desactivate_video()
        self.auto_reactivate = False
        app.logger.info("drone " + self.label + " stopped")


class Controller:
    def __init__(self, nb_drones):
        self.drones = []
        for i in range(1, nb_drones + 1):
            self.drones.append(Drone("drone_" + str(i), 1))

    def set_path(self, label_drone, path, closed, takePictures):
        for drone in self.drones:
            if drone.label == label_drone:
                drone.set_path(path, closed, takePictures)

    def stop(self, label_drone):
        for drone in self.drones:
            if drone.label == label_drone:
                drone.stop()
                return True
        return False

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
        if len(path) < 1:
            app.logger.warn("cannot use this path")
        else:
            closed = request.json['closed']
            takePictures = request.json['takePictures']
            controller.set_path(drone_label, path, closed, takePictures)
    except:
        app.logger.error(traceback.format_exc())
        abort(400)
    return "hello", 200


@app.route('/<path:drone_label>/stop', methods=['POST'])
def stop_drone(drone_label):
    global controller
    app.logger.info("received a new request on "+drone_label+"/stop")
    try:
        # Get the JSON data sent from the form
        success = controller.stop(drone_label)
        if success:
            return "Drone " + str(drone_label) + "stopped", 200
        else:
            return "Drone " + str(drone_label) + "not stopped. Defined drones : " + str(controller.drones), 404
    except:
        app.logger.error(traceback.format_exc())
        abort(500)


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
        abort(500)


@app.route('/hello', methods=['GET'])
def hello():
    app.logger.info("hello")
    return "hello", 200

if __name__ == '__main__':
    handler = RotatingFileHandler('/sit/log/flask.log', maxBytes=10000000, backupCount=1)
    #handler = logging.StreamHandler()
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    app.logger.addHandler(handler)
    try:
        app.logger.info("starting ros node with ROS_IP:" + os.environ['ROS_IP'])
        rospy.init_node("node_server")
        app.logger.info("ros node started with ROS_IP:" + os.environ['ROS_IP'])
        app.run(debug=True, host='0.0.0.0', port=5000)
        app.logger.info("flask webservice started")
    except Exception as e:
        app.logger.error(traceback.format_exc())

