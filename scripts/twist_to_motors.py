#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
from puffer_msgs.msg import DifferentialDriveCommand

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.12)
        self.rad_per_m = 1/.045
        # changed to match vel_cmd
        self.pub_motor = rospy.Publisher('vel_cmd', DifferentialDriveCommand, queue_size=10)
        #self.pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32, queue_size=10)
        rospy.Subscriber('twist', Twist, self.twistCallback)
    
        self.rate = rospy.get_param("~rate", 10)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks

        ###### main loop  ######
        while not rospy.is_shutdown():
        
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    #############################################################
    def spinOnce(self): # need to fix
    #############################################################
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
        # TODO need to convert meters to rad/s for PUFFER
        # in diff_tf, we converted encoder ticks to m/s
        # just 
        self.right = self.rad_per_m * (self.dx + self.dr * self.w / 2)
        self.left = self.rad_per_m * (self.dx - self.dr * self.w / 2)
        cmds = DifferentialDriveCommand()
        cmds.left_motor_command.header.frame_id = 'left_wheel'
        cmds.right_motor_command.header.frame_id = 'right_wheel'
        cmds.left_motor_command.motor_angular_velocity_rad_s = self.left
        cmds.right_motor_command.motor_angular_velocity_rad_s = self.right
        self.pub_motor.publish(cmds)
        # rospy.loginfo("publishing: (%d, %d)", left, right)    #debug  
        #self.pub_lmotor.publish(self.left) # old code
        #self.pub_rmotor.publish(self.right) # old code
            
        self.ticks_since_target += 1

    #############################################################
    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
