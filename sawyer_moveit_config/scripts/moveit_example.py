#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

# moveit stuff:
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_commander import MoveGroupCommander

# non ROS stuff:
import numpy as np
import random

# import set joint goals and SE(3) goals:
import joint_targets as jt
import cartesian_targets as ct


class MoveItCollisionTest( object ):
    def __init__(self):
        rospy.loginfo("Creating MoveItCollisionTest object")

        # let's create MoveIt! objects:
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

        # let's give a small delay to ensure that we have time for all
        # publishers and subscribers to be created
        rospy.sleep(3.0)

        # let's add world collision objects:
        self.world_collisions()

        # now we can set some planning and execution parameters
        self.right_arm_group.set_goal_position_tolerance(0.01)
        self.right_arm_group.set_goal_orientation_tolerance(0.01)
        self.right_arm_group.set_planning_time(5.0)
        self.right_arm_group.allow_replanning(False)
        self.right_arm_group.set_max_velocity_scaling_factor(0.3)
        self.right_arm_group.set_max_acceleration_scaling_factor(0.3)

        # create ROS interfaces:
        self.plansrv = rospy.Service("random_cartesian_pose_plan", Empty, self.random_cartesian_cb)
        self.jointsrv = rospy.Service("random_named_joint_space_plan", Empty, self.random_named_cb)
        self.circlesrv = rospy.Service("compute_cartesian_circle", Empty, self.compute_cartesian_cb)
        self.check_client = rospy.ServiceProxy("check_state_validity", GetStateValidity)
        self.ik_client = rospy.ServiceProxy("compute_ik", GetPositionIK)
        return


    def random_cartesian_cb(self, req):
        self.random_joint_space_plan(True)
        return EmptyResponse()


    def random_joint_space_plan(self, move=False):
        # RANDOM PLAN IN CARTESIAN SPACE
        good = False
        # let's also use MoveIt! to solve IK and verify that the found pose is
        # valid:
        for i in range(10):
            pstamped = self.right_arm_group.get_random_pose()
            solve, qrand = self.solve_ik(pstamped)
            if solve:
                if self.check_state(qrand):
                    self.right_arm_group.set_pose_target(pstamped)
                    good = True
                    break
        if not good:
            return
        else:
            # even though we now have a "good" IK solution, let's go ahead and
            # plan in Cartesian space just as a demo:
            self.right_arm_group.set_pose_target(pstamped)
            self.right_arm_group.plan()
            self.right_arm_group.go()
        return

    
    def check_state(self, q):
        # build request:
        req = GetStateValidityRequest()
        req.robot_state.joint_state.name = self.right_arm_group.get_active_joints()
        req.robot_state.joint_state.position = q
        resp = self.check_client(req)
        rospy.loginfo("Is q valid? %s", resp.valid)
        return resp.valid

    
    def solve_ik(self, pstamped):
        rospy.loginfo("Attempting to solve IK")
        req = GetPositionIKRequest()
        req.ik_request.group_name = "right_arm"
        req.ik_request.robot_state.joint_state.name = self.right_arm_group.get_active_joints()
        req.ik_request.robot_state.joint_state.position = [0, 0, 0, 0, 0, 0, 0]
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout = rospy.Duration(3.0)
        req.ik_request.pose_stamped = pstamped
        req.ik_request.attempts = 5
        resp = self.ik_client(req)
        q = np.zeros(len(self.right_arm_group.get_active_joints()))
        if resp.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            sol = {}
            for k,v in zip(resp.solution.joint_state.name, resp.solution.joint_state.position):
                sol[k] = v
            for i,n in enumerate(self.right_arm_group.get_active_joints()):
                q[i] = sol[n]
            rospy.loginfo("Found IK solution! q = %s", str(q.tolist()))
            return True, q
        else:
            rospy.logerr( "Could not solve IK... Error code = %d", resp.error_code.val)
            return False, q


    def world_collisions(self):
        self.scene.remove_world_object("column")
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.8
        p.pose.position.y = -0.1
        p.pose.position.z = 0.2
        self.scene.add_box("column", p, (0.1, 0.1, 0.5))
        return


    def random_named_cb(self, request):
        key = random.choice(jt.targets.keys())
        rospy.loginfo("Planning to target with key = %s", key)
        self.right_arm_group.set_joint_value_target(jt.targets[key])
        rospy.loginfo("Attempting to plan")
        plan = self.right_arm_group.plan()
        rospy.loginfo("Done with planning")
        self.right_arm_group.go()
        return EmptyResponse()


    def compute_cartesian_cb(self, request):
        # first, let's try to get to the initial pose:
        rospy.loginfo("Computing IK for the initial pose of the circle")
        pose_target = ct.targets[0]
        pstamped = PoseStamped()
        pstamped.pose = pose_target
        pstamped.header.frame_id = self.robot.get_planning_frame()
        solve, qstart = self.solve_ik(pstamped)
        if solve:
            self.right_arm_group.set_joint_value_target(qstart)
            self.right_arm_group.plan()
            self.right_arm_group.go()
        else:
            rospy.logerr("Could not solve IK for first part of the circle")
            return

        # now that we are at the start of the circle, let's see if we can construct path:
        self.right_arm_group.set_start_state_to_current_state()
        fraction = 0.0 
        max_attempts = 100 
        attempts = 0 

        # Plan the Cartesian path connecting the waypoints 
        while fraction < 1.0 and attempts < max_attempts: 
            (plan, fraction) = self.right_arm_group.compute_cartesian_path(ct.targets, 0.01, 0.0)
            # Increment the number of attempts 
            attempts += 1 
            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        # If we have a complete plan, execute the trajectory 
        if fraction == 1.0: 
            rospy.loginfo("Path computed successfully. Moving the arm.") 
            self.right_arm_group.execute(plan)
        else:
            rospy.logerr("Could not find valid cartesian path for circle")
        return EmptyResponse()



def main():
    rospy.init_node("moveit_sawyer_demonstration", log_level=rospy.INFO)
    
    try:
        colltest = MoveItCollisionTest()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__ == '__main__':
	main()
