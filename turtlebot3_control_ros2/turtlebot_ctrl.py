#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import random

from example_interfaces.msg import String

class TurtlebotCtrl(Node):
        def __init__(self):
                super().__init__("TurtlebotCtrl")

                self.laser = LaserScan()
                self.odom = Odometry()

                self.map = np.array([   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                                                [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                                [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
                                                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                                                ])

                self.publish_cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)
                self.subscriber_odom = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
                self.subscriber_laser = self.create_subscription(LaserScan, "/scan", self.callback_laser, 10)
                self.timer = self.create_timer(0.5, self.cmd_vel_pub)

                # --- Estado para la logica de exploracion ---
                self.forward_speed = 0.1
                self.turn_speed = 0.4
                self.safe_distance = 0.65
                self.front_angle_deg = 45.0

                self.state = "EXPLORE"          # EXPLORE | AVOID
                self.turn_direction = 1.0
                self.avoid_ticks_left = 0
                self.wander_ticks_left = 0
                self.wander_bias = 0.0          # sesgo de giro suave mientras explora

        def get_front_min_and_sides(self):
                ranges = self.laser.ranges
                n = len(ranges)
                if n == 0 or self.laser.angle_increment == 0.0:
                        return float("inf"), 0.0, 0.0

                angle_min = self.laser.angle_min
                angle_increment = self.laser.angle_increment
                half_front_rad = math.radians(self.front_angle_deg)

                def normalize(a):
                        while a > math.pi:
                                a -= 2.0 * math.pi
                        while a < -math.pi:
                                a += 2.0 * math.pi
                        return a

                front = []
                left = []
                right = []
                for i, r in enumerate(ranges):
                        if math.isinf(r) or math.isnan(r) or r <= 0.0:
                                continue
                        # angulo 0 = frente del robot, independiente de si el
                        # laser barre de -pi a pi o de 0 a 2*pi
                        a = normalize(angle_min + i * angle_increment)
                        if abs(a) <= half_front_rad:
                                front.append(r)
                        elif a > 0.0:
                                left.append(r)
                        else:
                                right.append(r)

                front_min = min(front) if front else float("inf")
                left_avg = sum(left) / len(left) if left else 0.0
                right_avg = sum(right) / len(right) if right else 0.0

                return front_min, left_avg, right_avg

        def cmd_vel_pub(self):

                map_resolution = 4

                index_x = -int(self.odom.pose.pose.position.x*map_resolution)
                index_y = -int(self.odom.pose.pose.position.y*map_resolution)

                index_x += int(self.map.shape[0]/2)
                index_y += int(self.map.shape[0]/2)

                if (index_x < 1): index_x = 1
                if (index_x > self.map.shape[0]-1): index_x = self.map.shape[0]-1
                if (index_y < 1): index_y = 1
                if (index_y > self.map.shape[0]-1): index_y = self.map.shape[0]-1

                if (self.map[index_x][index_y] == 1):
                        self.map[index_x][index_y] = 2

                        self.get_logger().info("Another part reached ... percentage total reached...." + str(100*float(np.count_nonzero(self.map == 2))/(np.count_nonzero(self.map == 1) + np.count_nonzero(self.map == 2))) )
                        self.get_logger().info("Discrete Map")
                        self.get_logger().info("\n"+str(self.map))

                ########## Desenvolva seu codigo aqui ########

                front_min, left_avg, right_avg = self.get_front_min_and_sides()

                msg = TwistStamped()

                if self.state == "EXPLORE":
                        if front_min < self.safe_distance:
                                self.state = "AVOID"
                                self.avoid_ticks_left = 8  # ~4s: primero retrocede, despues gira comprometido
                                self.turn_direction = 1.0 if left_avg > right_avg else -1.0
                                self.get_logger().info(
                                        "Obstaculo detectado, esquivando hacia " +
                                        ("izquierda" if self.turn_direction > 0 else "derecha"))
                        else:
                                # Cada tanto cambiamos el sesgo de giro para no quedar
                                # siempre en el mismo sector del mapa (deambular variado).
                                if self.wander_ticks_left <= 0:
                                        self.wander_bias = random.uniform(-0.15, 0.15)
                                        self.wander_ticks_left = random.randint(16, 30)  # 8-15s aprox, tramos mas largos
                                else:
                                        self.wander_ticks_left -= 1

                                msg.twist.linear.x = self.forward_speed
                                msg.twist.angular.z = self.wander_bias

                if self.state == "AVOID":
                        if self.avoid_ticks_left > 4:
                                # Primero retrocede un poco para despegarse del obstaculo
                                msg.twist.linear.x = -0.08
                                msg.twist.angular.z = 0.0
                        else:
                                msg.twist.linear.x = 0.0
                                msg.twist.angular.z = self.turn_speed * self.turn_direction
                        self.avoid_ticks_left -= 1
                        if self.avoid_ticks_left <= 0 and front_min >= self.safe_distance:
                                self.state = "EXPLORE"

                self.publish_cmd_vel.publish(msg)

        def callback_laser(self, msg):
                self.laser = msg

        def callback_odom(self, msg):
                self.odom = msg

def main(args=None):
        rclpy.init(args=args)
        node = TurtlebotCtrl()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == "__main__":
        main()
