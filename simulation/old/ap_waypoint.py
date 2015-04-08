#!/usr/bin/env python
# -*- coding: utf8 -*-

import sys

from math import *

# Importe ROSPY, l'API Python de ROS
import rospy

# Importe les messages ROS qui vont être utilisés
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
#from sensor_msgs.msg import NavSatFix

# Pour les conversions d'angle si besoin
import tf.transformations

def yaw_from_quaternion(quaternion):
    assert isinstance(quaternion, Quaternion)
    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w] )
    return yaw

def saturate(val, val_min, val_max):
    if val > val_max: val=val_max
    elif val < val_min: val=val_min
    return val

def saturate_abs(val, abs_val_max):
    return saturate(val, -abs_val_max, abs_val_max)

# Classe du régulateur de cap
class APWaypoint:

  def __init__(self):
    # Publisher
    self.cmd_pub = rospy.Publisher("vel_cmd", Twist, queue_size=1, latch=False)
    self.status_pub = rospy.Publisher("status", String, queue_size=1, latch=True)
    # Subscribers
    self.pose_sub = rospy.Subscriber("pose",PoseStamped,self.pose_callback)
    self.dest_pose_sub = rospy.Subscriber("dest_pose",Pose,self.set_dest_pose)
    self.dest_point_sub = rospy.Subscriber("dest_point",Point,self.set_dest_point)
    self.dest_yaw_sub = rospy.Subscriber("dest_yaw",Float64,self.set_dest_yaw)

    # Paramètres regulateur de cap
    self.Kp_w = 4.0
    self.w_max = 3.0
    # Paramètres suivi de points
    self.Kp_xy = 2.0
    self.vxy_max = 10.0
    # Paramètres régulation d'altitude
    self.Kp_z = 1.0
    self.vz_max = 3.0
    # Destination
    self.theta = None
    self.dest = None

    self.theta_tol = 0.02
    self.dest_tol = 0.1

    self.status_pub.publish("Waiting")

  # Méthode pour définir la consigne de pose (position et cap).
  def set_dest_pose(self, consigne):
    assert isinstance(consigne, Pose)
    self.set_dest_point(consigne.position)
    self.set_dest_yaw( yaw_from_quaternion(consigne.orientation) )

  # Méthode qui définit la consigne de position.
  def set_dest_point(self, consigne):
    assert isinstance(consigne, Point)
    self.dest = consigne
    self.status_pub.publish("En route")

  # Méthode qui définit la consigne de cap.
  def set_dest_yaw(self, consigne):
    assert isinstance(consigne, Float64)
    self.theta = consigne.data


  # Fonction qui est appelée à chaque fois que la pose du robot tortue est reçue.
  def pose_callback(self, pose_stamped):
    assert isinstance(pose_stamped, PoseStamped)
    pose = pose_stamped.pose
    yaw = yaw_from_quaternion(pose.orientation)

    # Régulation du cap
    w = 0.0
    if self.theta != None:
        e_theta = self.theta - yaw
        if cos(e_theta) > 0:
           e_sin_theta = sin(e_theta)
        elif sin(e_theta) < 0:
           e_sin_theta = -1.0
        else:
           e_sin_theta = 1.0
        w = self.Kp_w * e_sin_theta #* pow(abs(e_sin_theta), 2)
        w = saturate_abs(w, self.w_max)

        if (cos(e_theta) > 0) and (abs(sin(e_theta)) < 0.02):
            w = 0.0
            self.theta = None

    # Régulation en position
    (vx, vy, vz) = (0.0, 0.0, 0.0)

    if self.dest != None:
        # Regulation en altitude
        ez = self.dest.z - pose.position.z
        vz = self.Kp_z * ez
        vz = saturate_abs(vz, self.vz_max)

        # Navigation vers le point
        ex = self.dest.x - pose.position.x
        ey = self.dest.y - pose.position.y
        d = sqrt(ex*ex + ey*ey)
        v = saturate_abs( self.Kp_xy*d, self.vxy_max )

        # consigne de vitesse dans le repere monde
        vx_0 = v * ex / d
        vy_0 = v * ey / d
        # passage dans le body-frame
        vx = vx_0*cos(-yaw) - vy_0*sin(-yaw)
        vy = vx_0*sin(-yaw) + vy_0*cos(-yaw)

        # verification de l'arrivée
        if sqrt(ex*ex + ey*ey + ez*ez) < self.dest_tol:
            (vx, vy, vz) = (0.0, 0.0, 0.0)
            self.dest = None
            self.status_pub.publish("Arrived")

    self.cmd_pub.publish(Twist(Vector3(vx,vy,vz), Vector3(0,0,-w)))


  def goto(self, x, y, z):
      self.set_dest_point(Point(x,y,z))

  def goto(self, x, y, z, theta):
      self.set_dest_point(Point(x,y,z))
      self.set_dest_yaw(theta)

# Fonction principale
def main(args):

  # Démarre le noeud ROS
  rospy.init_node('AP_waypoint')

  # Crée le pilote automatique
  ap = APWaypoint()

  # Example de log
  rospy.loginfo("Waypoint Autopilot started at %s", rospy.get_time())

  # Boucle tant que le noeud ROS n'est pas interrompu par Ctrl+C
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

# Point d'entrée
if __name__ == '__main__':
    main(sys.argv)
