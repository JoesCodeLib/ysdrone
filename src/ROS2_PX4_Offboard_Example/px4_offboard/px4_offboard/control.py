#!/usr/bin/env python3
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Joe Shin"
__contact__ = "jihwanshin@yonsei.ac.kr"


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Bool
import math
import sys
import time
import datetime
import tf2_ros
import geometry_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create Publishers
        self.velocity_publisher = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)
        self.arm_publisher = self.create_publisher(Bool, '/arm_message', qos_profile)
        self.command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        
        # Create Subscribers
        self.position_subscriber = self.create_subscription(PoseStamped, '/px4_visualizer/vehicle_pose', self.position_callback, 10)
        

        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        
        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 5.0},
            {'x': 4.0, 'y': 4.0, 'z': 5.0},
            {'x': 4.0, 'y': -4.0, 'z': 5.0},
            {'x': -4.0, 'y': -4.0, 'z': 5.0},
            {'x': -4.0, 'y': 4.0, 'z': 5.0},
            {'x': 4.0, 'y': 4.0, 'z': 5.0},
            {'x': 0.0, 'y': 0.0, 'z': 5.0},
            {'x': 0.0, 'y': 0.0, 'z': 3.0},
            {'x': 0.0, 'y': 0.0, 'z': 0.0}
            # Add more waypoints as needed
        ]
        
        self.current_waypoint_index = 0
        self.position_tolerance = 0.6

        # Add a timer to print the current position every 2 seconds
        self.timer = self.create_timer(0.1, self.print_current_position)
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.arm_drone(True)  # Arm the drone
        
        self.navigate_waypoints()


    def position_callback(self, msg):
        self.current_position['x'] = msg.pose.position.x
        self.current_position['y'] = msg.pose.position.y
        self.current_position['z'] = msg.pose.position.z

        if not hasattr(self, 'initial_position_printed'):
            self.initial_position_printed = True
            self.get_logger().info(f"Initial Position: x={self.current_position['x']}, y={self.current_position['y']}, z={self.current_position['z']}")


    def arm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info(f"Drone {'armed' if arm else 'disarmed'}")

    def send_land_command(self):
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 uses timestamps in microseconds
        cmd.param1 = float(6)  # The flight mode number for LAND, based on PX4's command definition
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.target_system = 1  # Typically 1 for the main system
        cmd.target_component = 1  # Typically 1 for the autopilot
        cmd.source_system = 1  # Typically 1 for the ground control station
        cmd.source_component = 1  # Typically 1 for the ground control station
        cmd.from_external = True  # Command is from an offboard source

        self.command_publisher.publish(cmd)
        self.get_logger().info('LAND command sent')



    def print_current_position(self):
        print(f"Current Position: x={self.current_position['x']}, y={self.current_position['y']}, z={self.current_position['z']}")

    def navigate_waypoints(self):
        self.timer = self.create_timer(0.01, self.navigate_waypoint_callback)  # Create a timer to call the waypoint navigation callback

    def navigate_waypoint_callback(self):
        if self.current_waypoint_index < len(self.waypoints):
            #zero_count = 0
            target = self.waypoints[self.current_waypoint_index]
            if self.is_waypoint_reached(target):
                self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached.")
                self.current_waypoint_index += 1  # Move to next waypoint
                self.stop_drone()  # Stop the drone at the waypoint
            else:
                twist = self.calculate_velocity_command(target)
                self.velocity_publisher.publish(twist)

                print("given")
                print(f"Current command: {self.current_waypoint_index}")
                print(f"Velocity Command: linear_x={twist.linear.x}, linear_y={twist.linear.y}, linear_z={twist.linear.z}")
                
        else:
            # self.get_logger().info("Disarm service triggered.")
            # twist = self.calculate_velocity_command_land({'x': 0.0, 'y': 0.0, 'z': 3.0})
            # self.velocity_publisher.publish(twist)
            # time.sleep(5)
            # twist = self.calculate_velocity_command_land({'x': 0.0, 'y': 0.0, 'z': 0.0})
            # self.velocity_publisher.publish(twist)
            # Please work. I love you. 
            if self.is_waypoint_reached_z({'x': 0.0, 'y': 0.0, 'z': 0.0}):
                self.get_logger().info("Disarm waypoint reached.")
                self.send_land_command()
                self.send_land_command()
                self.send_land_command()
                self.send_land_command()
                self.arm_drone(False)
                self.arm_drone(False)
                # self.arm_drone(False)
                # self.arm_drone(False)
                # self.arm_drone(False)
                # self.arm_drone(False)
                # self.arm_drone(False)
                self.destroy_timer(self.timer)  # Stop the navigation timer

    def calculate_velocity_command(self, target):
        twist = Twist()
        kp = 0.2 # Proportional gain
        error_y = target['y'] - self.current_position['y']
        error_z = target['z'] - self.current_position['z']
        error_x = target['x'] - self.current_position['x']
    

        twist.linear.x = kp * error_x
        twist.linear.y = kp * -error_y #solved by JKP 2024.06.12. this is jjam difference
        twist.linear.z = kp * error_z

        # Normalize the velocities to a fixed speed
        max_speed = 1.0  # Maximum speed
        norm = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        if norm > max_speed:
            twist.linear.x = (twist.linear.x / norm) * max_speed
            twist.linear.y = (twist.linear.y / norm) * max_speed
            twist.linear.z = (twist.linear.z / norm) * max_speed

        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        print(f"Target: x={target['x']}, y={target['y']}, z={target['z']}")
        print(f"Errors: x={error_x}, y={error_y}, z={error_z}")
        print(f"Velocity Command: linear_x={twist.linear.x}, linear_y={twist.linear.y}, linear_z={twist.linear.z}")


        return twist
    
    def calculate_velocity_command_land(self, target):
        twist = Twist()
        kp = 1 # Proportional gain
        error_y = target['y'] - self.current_position['y']
        error_z = target['z'] - self.current_position['z']
        error_x = target['x'] - self.current_position['x']
    

        twist.linear.x = kp * error_x
        twist.linear.y = kp * -error_y #solved by JKP 2024.06.12. this is jjam difference
        twist.linear.z = kp * error_z

        # Normalize the velocities to a fixed speed

        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        print(f"Target: x={target['x']}, y={target['y']}, z={target['z']}")
        print(f"Errors: x={error_x}, y={error_y}, z={error_z}")
        print(f"Velocity Command: linear_x={twist.linear.x}, linear_y={twist.linear.y}, linear_z={twist.linear.z}")

        return twist
    
    def twist_zero(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        return twist

    def is_waypoint_reached(self, target):
        x_dist = abs(target['x'] - self.current_position['x'])
        y_dist = abs(target['y'] - self.current_position['y'])
        z_dist = abs(target['z'] - self.current_position['z'])

        # Check if all directional distances are within tolerance
        return (x_dist < self.position_tolerance and
                y_dist < self.position_tolerance and
                z_dist < self.position_tolerance)
        
    def is_waypoint_reached_z(self, target):
        x_dist = abs(target['x'] - self.current_position['x'])
        y_dist = abs(target['y'] - self.current_position['y'])
        z_dist = abs(target['z'] - self.current_position['z'])

        # Check waypoint only for Z
        return (z_dist < self.position_tolerance)

    def stop_drone(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        self.velocity_publisher.publish(twist)


def main(args=None):
    current_time = datetime.datetime.now()
    formatted_time = current_time.strftime("%m-%d-%H-%M-%S")
    filename = f"{formatted_time}_test.txt"
    f = open(filename, 'w')
    sys.stdout = f
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    f.close()
    sys.stdout = sys.__stdout__

if __name__ == '__main__':
    main()