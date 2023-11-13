#! /usr/bin/python3

import rospy
from  rospkg import RosPack
import roslaunch
from std_msgs.msg import Empty
import subprocess 

class OnButton():

    def __init__(self):
        self.started_sim = False
        self.started_stack = False
        self.start_sim_cmd = False
        self.stop_sim_cmd = False
        self.start_stack_cmd = False
        self.stop_stack_cmd = False
        self.republisher = None
        # Subscribe to the "start_sim" topic
        self.start_sim_sub = rospy.Subscriber("start_sim", Empty, self.start_sim_callback)
    
        # Subscribe to the "stop_sim" topic
        self.stop_sim_sub = rospy.Subscriber("stop_sim", Empty, self.stop_sim_callback)  

        # Subscribe to the "start_stack" topic
        self.start_sim_sub = rospy.Subscriber("start_stack", Empty, self.start_stack_callback)
    
        # Subscribe to the "stop_stack" topic
        self.stop_sim_sub = rospy.Subscriber("stop_stack", Empty, self.stop_stack_callback)
        self.republisher=subprocess.Popen(["rosrun", "tf2_web_republisher", "tf2_web_republisher"]) 

    def start_rosbridge_server(self):
        # Start the rosbridge_server using subprocess
        try:
            subprocess.Popen(["roslaunch", "rosbridge_server", "rosbridge_websocket.launch"])
        except Exception as e:
            print("Error starting rosbridge_server and republisher.:", str(e))

    def stop_sim_callback(self, msg):
        # Callback function to kill swarm_sim
        rospy.loginfo("Recieved 'stop_sim' message, halting sim.")
        self.stop_sim_cmd = True
    
    def start_sim_callback(self, msg):
        # Callback function to start the swarm_sim package
        rospy.loginfo("Received 'start_sim' message, launching swarm_sim.")
        self.start_sim_cmd = True
    
    def start_stack_callback(self, msg):
        # Callback function to start the navigation stack(s)
        rospy.loginfo("Received 'start_stack' message, launching fleet bringup.")
        self.start_stack_cmd = True

    def stop_stack_callback(self, msg):
        # Callback function to stop the navigation stack(s)
        rospy.loginfo("Received 'stop_stack' message, killing fleet bringup.")
        self.stop_stack_cmd = True

def main():

    rospy.init_node("stack_controller")
    
    button = OnButton()
    rp = RosPack()
    
    # Start rosbridge server
    button.start_rosbridge_server()

    # Define roslaunch interfaces for the stacks we want to start/stop. 
    sim_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    stack_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(sim_uuid)
    roslaunch.configure_logging(stack_uuid)
    sim_launch = roslaunch.parent.ROSLaunchParent(sim_uuid, [rp.get_path('swarm_sim') + '/launch/sim.launch'])
    fleet_launch = roslaunch.parent.ROSLaunchParent(stack_uuid, [rp.get_path('fleet_bringup') + '/launch/bringup.launch'])
    
    
   
    while not rospy.is_shutdown():
    
        if not button.started_sim and button.start_sim_cmd and not button.started_stack:
            sim_launch.start()
            button.start_sim_cmd = False
            button.started_sim = True
    
        if button.started_sim and button.stop_sim_cmd:
            sim_launch.shutdown()
            button.republisher.kill()
            button.republisher=subprocess.Popen(["rosrun", "tf2_web_republisher", "tf2_web_republisher"]) 
            sim_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(sim_uuid)
            sim_launch = roslaunch.parent.ROSLaunchParent(sim_uuid, [rp.get_path('swarm_sim') + '/launch/sim.launch'])
            button.stop_sim_cmd = False
            button.started_sim = False

        if not button.started_stack and button.start_stack_cmd and not button.started_sim:
            fleet_launch.start()
            rospy.loginfo("Starting stack...")
            button.start_stack_cmd = False
            button.started_stack = True
    
        if button.started_stack and button.stop_stack_cmd:
            fleet_launch.shutdown()
            button.republisher.kill()
            button.republisher=subprocess.Popen(["rosrun", "tf2_web_republisher", "tf2_web_republisher"]) 
            stack_uuid= roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(stack_uuid)
            fleet_launch = roslaunch.parent.ROSLaunchParent(stack_uuid, [rp.get_path('fleet_bringup') + '/launch/bringup.launch'])
            button.stop_stack_cmd = False
            button.started_stack = False


if __name__ == "__main__":
    main()
