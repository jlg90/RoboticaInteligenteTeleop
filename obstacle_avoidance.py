#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from ackermann_msgs.msg import AckermannDrive
import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        # Inicializa el nodo de ROS
        rospy.init_node('obstacle_avoidance')

        # Suscriptor para la nube de puntos de los obstáculos captados por el lidar
        rospy.Subscriber("/obstacles", PointCloud2, self.obstacle_callback)

        # Suscriptor para comandos de control Ackermann
        rospy.Subscriber("/blue/preorder_ackermann_cmd", AckermannDrive, self.ackermann_callback)

        # Publicador para comandos Ackermann modificados
        self.cmd_pub = rospy.Publisher("/blue/ackermann_cmd", AckermannDrive, queue_size=10)

        # Almacenar el último mensaje Ackermann recibido
        self.last_ackermann_cmd = AckermannDrive()
        
        # Flag para indicar si el robot está detenido por un obstáculo
        self.stopped_for_obstacle = False

        # Temporizador para reanudar el control después de unos segundos
        self.resume_timer = None

        # Temporizador para ignorar detección de obstáculos por un breve período
        self.ignore_obstacles_timer = None
        self.ignore_obstacles = False

    def obstacle_callback(self, msg):
        if self.ignore_obstacles:
            rospy.loginfo("Ignoring obstacle detection temporarily.")
            return

        # Procesar la nube de puntos con los obstáculos
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Asumir que el robot tiene un ancho de 1 metro y un margen de 0.5 metros a cada lado
        safety_margin = 0.01
        width = 1.4 * safety_margin

        # Filtrar puntos que están dentro de la trayectoria del robot
        obstacles_in_path = [point for point in points if -width/2 <= point[1] <= width/2 and 0 <= point[0] <= 2.0]

        # Crear un comando de AckermannDrive
        cmd = self.last_ackermann_cmd

        if obstacles_in_path:
            if not self.stopped_for_obstacle:
                rospy.loginfo("Obstacle detected! Stopping the robot.")
                cmd.speed = 0.0
                self.cmd_pub.publish(cmd)
                self.stopped_for_obstacle = True

                self.resume_timer = rospy.Timer(rospy.Duration(1), self.resume_control, oneshot=True)
        else:
            if not self.stopped_for_obstacle:
                rospy.loginfo("Path is clear. Proceeding with the original command.")
                self.cmd_pub.publish(cmd)

    def ackermann_callback(self, msg):
        # Almacena el último comando recibido
        self.last_ackermann_cmd = msg

    def resume_control(self, event):
        cmd = self.last_ackermann_cmd
        rospy.loginfo("Resuming control to the operator.")
        self.cmd_pub.publish(cmd)
        self.stopped_for_obstacle = False
        self.resume_timer = None


        self.ignore_obstacles = True
        self.ignore_obstacles_timer = rospy.Timer(rospy.Duration(3), self.stop_ignoring_obstacles, oneshot=True)

    def stop_ignoring_obstacles(self, event):
        rospy.loginfo("Re-enabling obstacle detection.")
        self.ignore_obstacles = False
        self.ignore_obstacles_timer = None

if __name__ == '__main__':
    oa = ObstacleAvoidance()
    rospy.spin()
