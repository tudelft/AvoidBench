import os
import rospy
import rospkg
import sys
import math
import random
import time
import numpy as np
from enum import Enum
from ruamel.yaml import YAML
from std_msgs.msg import Empty, Float32, Bool, Header
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from rpg_quadrotor_msgs.msg import TrajectoryPoint
from avoid_msgs.msg import TaskState, Metrics, PointCloudInfo, Factor
from avoidbridge import AvoidbenchBridge, quadStateEstimate, mission_parameter, getCheckingResult
from avoidmetrics import AvoidMetrics, AvoidMission, MetricsMsg

class Mission_state(Enum):
    PREPARING = 0
    UNITYSETTING = 1
    SENTMISSION = 2
    WAITMISSION = 3
    MISSIONPROCESS = 4
    GAZEBOSETTING = 5

class AvoidManage:
    def __init__(self):
        self.unity_init_ = False
        self.unity_ready_ = False
        self.start_ok_ = False
        self.collision_state = True
        self.last_collision_state = True
        self.collision_happen = False
        self.get_seed = False
        self.get_new_mission = True
        self.if_update_map = True
        self.mission_state = Mission_state.PREPARING
        self.iter_time = []
        self.mission_id = 0
        self.trial_id = 0
        self.cv_bridge = CvBridge()
        self.loadParams()
        self.p_m = mission_parameter()
        if self.env_id == 3:
            self.cfg_ = rospkg.RosPack().get_path('avoid_manage') + '/params/task_outdoor.yaml'
        elif self.env_id == 1:
            self.cfg_ = rospkg.RosPack().get_path('avoid_manage') + '/params/task_indoor.yaml'
        self.received_state_est_ = quadStateEstimate()
        self.left_pub_ = rospy.Publisher("/rgb/left", Image, queue_size=1)
        self.right_pub_ = rospy.Publisher("/rgb/right", Image, queue_size=1)
        self.depth_pub_ = rospy.Publisher("/depth", Image, queue_size=1)
        self.pose_pub_ = rospy.Publisher("autopilot/reset_reference_state", TrajectoryPoint, queue_size=10)
        self.start_pub_ = rospy.Publisher("autopilot/start", Empty, queue_size=1)
        self.force_hover_pub_ = rospy.Publisher("autopilot/force_hover", Empty, queue_size=1)
        self.arm_pub_ = rospy.Publisher("bridge/arm", Bool, queue_size=1)
        self.goal_pub_ = rospy.Publisher("goal_point", Path, queue_size=1)
        self.task_state_pub_ = rospy.Publisher("task_state", TaskState, queue_size=10)
        self.avoid_metrics_pub_ = rospy.Publisher("metrics", Metrics, queue_size=10)
        # initialize subscriber call backs
        rospy.wait_for_service("/gazebo/set_model_state")
        self.gazebo_model_srv_ = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.state_est_sub_ = rospy.Subscriber("flight_pilot/state_estimate", Odometry, self.poseCallback, queue_size=1)
        self.mission_start_sub_ = rospy.Subscriber("mission_start", Empty, self.MissionFlagCallback, queue_size=1)
        self.iter_time_sub_ = rospy.Subscriber("iter_time", Float32, self.IterTimeCallback, queue_size=10)
        
        self.mission = AvoidMission()
        self.bridge = AvoidbenchBridge(self.cfg_)
        self.unity_init_ = True
        self.metrics = AvoidMetrics(self.cfg_)
        
        if self.save_data_mode:
            self.save_pc_sub = rospy.Subscriber("/pc_path", PointCloudInfo, self.SavePointcloudCallback)
        self.render_loop_ = rospy.Timer(rospy.Duration(0.02), self.RenderingCallback)
        self.metrics_loop_ = rospy.Timer(rospy.Duration(0.02), self.MetricsLoopCallback)
        self.task_loop_ = rospy.Timer(rospy.Duration(0.05), self.MissionProcess)
        self.mission_state_loop_ = rospy.Timer(rospy.Duration(0.008), self.MissionStateLoop)

        self.mission_state = Mission_state.UNITYSETTING


    def loadParams(self):
        self.mission_number = rospy.get_param("~flight_number", 0)
        print("mission_number: ", self.mission_number)
        self.env_id = rospy.get_param("~unity/env_idx", 0)
        self.save_data_mode = rospy.get_param("~unity/save_data_mode", False)
        return True

    def poseCallback(self, data):
        pos = np.zeros(3, dtype=np.float64)
        vel = np.zeros(3, dtype=np.float64)
        ori = np.array([0.0, 0.0, 0.0, 1.0])
        pos[0] = data.pose.pose.position.x
        pos[1] = data.pose.pose.position.y
        pos[2] = data.pose.pose.position.z
        vel[0] = data.twist.twist.linear.x
        vel[1] = data.twist.twist.linear.y
        vel[2] = data.twist.twist.linear.z
        ori[3] = data.pose.pose.orientation.w
        ori[0] = data.pose.pose.orientation.x
        ori[1] = data.pose.pose.orientation.y
        ori[2] = data.pose.pose.orientation.z
        self.received_state_est_.setStateEstimate(pos, ori, vel, data.header.stamp.to_sec())
        self.received_state_est_.transformVelocityToWorldFrame()

    def RenderingCallback(self, data):
        if self.unity_init_:
            self.unity_ready_ = self.bridge.updateUnity(self.received_state_est_)
            if self.mission_state == Mission_state.MISSIONPROCESS:
                self.collision_state = self.bridge.getQuadCollisionState()
                if not self.last_collision_state and self.collision_state:
                    self.collision_happen = True
                    print("crashed")
        timestamp = rospy.Time.now()
        header = Header(stamp=timestamp)
        header.frame_id = 'world'
        images = self.bridge.getImages()
        left_msg = self.cv_bridge.cv2_to_imgmsg(images[0], "bgr8")
        right_msg = self.cv_bridge.cv2_to_imgmsg(images[1], "bgr8")
        depth_msg = self.cv_bridge.cv2_to_imgmsg(images[2], "mono16")
        left_msg.header = header
        right_msg.header = header
        depth_msg.header = header
        self.left_pub_.publish(left_msg)
        self.right_pub_.publish(right_msg)
        self.depth_pub_.publish(depth_msg)
        self.last_collision_state = self.collision_state

    def MissionFlagCallback(self, data):
        if self.mission_state is not Mission_state.UNITYSETTING and self.mission_state is not Mission_state.GAZEBOSETTING:
            self.mission_state = Mission_state.MISSIONPROCESS

    def IterTimeCallback(self, data):
        if self.mission_state == Mission_state.MISSIONPROCESS or self.mission_state == Mission_state.SENTMISSION:
            self.iter_time.append(data.data)

    def SavePointcloudCallback(self, data):
        curr_dir = data.curr_dir
        range_ = np.zeros(3, dtype=np.float64)
        origin_ = np.zeros(3, dtype=np.float64)
        range_[0] = data.range[0]
        range_[1] = data.range[1]
        range_[2] = data.range[2]
        origin_[0] = data.origin[0]
        origin_[1] = data.origin[1]
        origin_[2] = data.origin[2]
        self.bridge.getPointCloud(curr_dir, range_, origin_)

    def MetricsLoopCallback(self, event):
        if not self.unity_ready_:
            return
        self.start_time = rospy.Time.now()
        if not self.start_ok_:
            rospy.sleep(2.0)
            if_arm_ = Bool()
            if_arm_.data = True
            take_off_ = Empty()
            while not self.bridge.finish_pc_save:
                rospy.sleep(0.2)
            self.arm_pub_.publish(if_arm_)
            self.start_pub_.publish(take_off_)
            print("send takeoff")
            self.metrics.run()
            self.start_ok_ = True
        if self.start_ok_ and self.metrics.task_finished:
            metrics_py = self.metrics.getMetricsMsg()
            metrics_msg = Metrics()
            for k in range(len(metrics_py.traversability)):
                factor_msg = Factor()
                factor_msg.optimality_factor = metrics_py.optimality_factor[k]
                factor_msg.average_goal_velocity = metrics_py.average_goal_velocity[k]
                factor_msg.mission_progress = metrics_py.mission_progress[k]
                factor_msg.processing_time = metrics_py.processing_time[k]
                factor_msg.traversability = metrics_py.traversability[k]
                factor_msg.relative_gap_size = metrics_py.relative_gap_size[k]
                factor_msg.collision_number = metrics_py.collision_number[k]
                metrics_msg.factors.append(factor_msg)
            self.avoid_metrics_pub_.publish(metrics_msg)

    def CheckCollision(self, param):
        self.bridge.finish_check = False
        self.bridge.checkCollisionState(param.m_start_point, True)
        while not self.bridge.finish_check:
            rospy.sleep(0.05)
        if getCheckingResult():
            return True
        else:
            self.bridge.finish_check = False
            self.bridge.checkCollisionState(param.m_end_point, False)
            while not self.bridge.finish_check:
                rospy.sleep(0.05)
        return getCheckingResult()

    def MissionProcess(self, event):
        if self.mission_state == Mission_state.UNITYSETTING:
            if self.get_new_mission:
                self.getMissionParam(self.mission_id)
                self.bridge.setParamFromMission(self.p_m)
                self.bridge.spawnObstacles()
                while not self.bridge.ifSceneChanged():
                    self.bridge.SpawnNewObs()
                ss = 0
                while self.CheckCollision(self.p_m):
                    ss = ss + 1
                    self.mission_state = Mission_state.UNITYSETTING
                    self.if_update_map = False
                    if ss>16:
                        self.if_update_map = True
                        return
                    self.getMissionParam(self.mission_id)
                    rospy.sleep(0.2)
                if not self.save_data_mode:
                    range_, origin_ = self.CalculateRanges()
                    self.bridge.finish_pc_save = False
                    self.bridge.getPointCloud("", range_, origin_)
                    while not self.bridge.finish_pc_save:
                        rospy.sleep(0.2)
                rospy.sleep(5.0)
            else:
                self.if_update_map = False
                self.getMissionParam(self.mission_id)
                while self.CheckCollision(self.p_m):
                    self.mission_state = Mission_state.UNITYSETTING
                    self.if_update_map = False
                    self.getMissionParam(self.mission_id)
                    rospy.sleep(0.2)
                rospy.sleep(5.0)
            self.mission = AvoidMission(self.cfg_, self.p_m, self.mission_id)
            self.resetGazebo([self.p_m.m_start_point[0], self.p_m.m_start_point[1],
                                self.p_m.m_start_point[2]], self.p_m.m_start_point[3])
            print(self.p_m.m_start_point)
            self.mission_start_time = rospy.Time.now()
            rospy.sleep(1.0)
            self.mission_state = Mission_state.SENTMISSION
            return

        if self.mission_state == Mission_state.SENTMISSION:
            if self.TakeoffSuccess():
                pub_msg = Path()
                goal = PoseStamped()
                goal.pose.position.x = self.mission.end_point[0]
                goal.pose.position.y = self.mission.end_point[1]
                goal.pose.position.z = self.mission.end_point[2]
                pub_msg.poses.append(goal)
                header = Header(stamp=rospy.Time.now())
                header.frame_id = 'world'
                pub_msg.header = header
                print("goal: ", goal.pose.position)
                self.goal_pub_.publish(pub_msg)
                rospy.sleep(0.1)
                if (rospy.Time.now() - self.mission_start_time).to_sec() > 60.0:
                    rospy.WARN("Exceed 1 min cannot start planning, pass current Trial")

        if self.mission_state == Mission_state.WAITMISSION:
            self.mission.getTrajectory(self.received_state_est_)
            if self.mission.finished and not self.save_data_mode:
                self.mission_state = Mission_state.GAZEBOSETTING
                return
            if (rospy.Time.now() - self.mission_start_time).to_sec() > 80.0:
                self.mission_state = Mission_state.GAZEBOSETTING
                return

        if self.mission_state == Mission_state.MISSIONPROCESS:
            self.mission.getTrajectory(self.received_state_est_)
            if self.collision_happen:
                # rospy.INFO("collision happened")
                self.mission.CollisionCount()
                self.collision_happen = False
            self.mission_state = Mission_state.WAITMISSION
            if self.mission.stop_flag:
                self.mission_state = Mission_state.GAZEBOSETTING

        if self.mission_state == Mission_state.GAZEBOSETTING:
            rospy.sleep(1.0)
            self.resetGazebo([0, 0, 1.2], 0)
            hover = Empty()
            self.force_hover_pub_.publish(hover)
            self.mission.cal_time = self.iter_time
            self.iter_time = []
            self.metrics.setMissions(self.mission)
            rospy.sleep(1.0)
            self.trial_id = self.trial_id + 1
            if self.trial_id == self.mission.trials:
                self.mission_id = self.mission_id + 1
                self.trial_id = 0
                self.get_new_mission = True
            else:
                self.get_new_mission = False

            if self.mission_id < self.mission_number:
                self.mission_state = Mission_state.UNITYSETTING
            else:
                self.mission_state = Mission_state.PREPARING
                self.metrics.setTaskFinishFlag(True)

    def MissionStateLoop(self, event):
        state = TaskState()
        header = Header(stamp=rospy.Time.now())
        state.header = header
        state.Mission_state = self.mission_state.value
        self.task_state_pub_.publish(state)

    def getMissionParam(self, m_id):
        cfg = YAML().load(open(self.cfg_, 'r'))
        if not self.get_seed:
            self.env_range = cfg["unity"]["range"]
            self.env_origin = cfg["unity"]["origin"]
            self.start_area = cfg["mission"]["start_area"]
            self.start_origin = cfg["mission"]["start_origin"]
            self.end_area = cfg["mission"]["end_area"]
            self.end_origin = cfg["mission"]["end_origin"]
            self.radius_area = cfg["mission"]["radius_area"]
            self.radius_origin = cfg["mission"]["radius_origin"]
            self.seed = cfg["mission"]["seed"]
            self.trials_ = cfg["mission"]["trials"]
            random.seed(self.seed)
            self.get_seed = True
        rand = random.random()
        print("rand: ", rand)

        if self.env_id == 3:
            area_id = random.randint(0, 3)
            if area_id == 0:
                self.p_m.m_start_point = [-self.start_area[0]/2.0+self.start_area[0]*rand,
                                        self.start_area[1]*rand, 2.0, 0]
                rand = random.random()
                self.p_m.m_end_point = [-self.end_area[0]/2.0+self.end_area[0]*rand,
                                    -self.end_area[1]*rand+self.end_origin[1],
                                    2.0+rand]
            if area_id == 1:
                self.p_m.m_start_point = [self.start_area[1]*rand-self.start_area[0]/2.0,
                                        self.start_area[0]*rand, 2.0, -math.pi/2.0]
                rand = random.random()
                self.p_m.m_end_point = [-self.end_area[1]*rand+self.end_area[0]/2.0,
                                    self.end_area[0]*rand, 2.0+rand]
            if area_id == 2:
                self.p_m.m_start_point = [-self.end_area[0]/2.0+self.end_area[0]*rand,
                                        -self.end_area[1]*rand+self.end_area[0],
                                        2.0, math.pi]
                rand = random.random()
                self.p_m.m_end_point = [-self.start_area[0]/2.0+self.start_area[0]*rand,
                                    self.start_area[1]*rand, 2.0+rand]
            if area_id == 3:
                self.p_m.m_start_point = [-self.end_area[1]*rand+self.end_area[0]/2.0,
                                        self.end_area[0]*rand, 2.0, math.pi/2.0]
                rand = random.random()
                self.p_m.m_end_point = [self.start_area[1]*rand-self.start_area[0]/2.0,
                                    self.start_area[0]*rand, 2.0+rand]
        elif self.env_id == 1:
            self.p_m.m_start_point = [-self.start_area[0]/2.0+self.start_area[0]*rand,
                                    self.start_area[1]*rand, 2.0, 0]
            rand = random.random()
            self.p_m.m_end_point = [-self.end_area[0]/2.0+self.end_area[0]*rand,
                                -self.end_area[1]*rand+self.end_origin[1], 2.0+rand]

        if not self.if_update_map:
            self.if_update_map = True
            return
        print("update env")
        self.p_m.trials = self.trials_
        self.p_m.m_radius = self.radius_origin + self.radius_area*rand
        self.p_m.m_seed = random.randint(0, 200)
        self.p_m.m_opacity = random.random()
        if not self.save_data_mode:
            self.p_m.m_pc_file_name = cfg["mission"]["pc_file_name"]+str(m_id)
        else:
            self.p_m.m_pc_file_name = cfg["mission"]["pc_file_name"]
            self.mission_number = cfg["mission"]["rollout"]

    def resetGazebo(self, pos, yaw):
        drone_state = SetModelStateRequest()
        cmd = TrajectoryPoint()
        drone_state.model_state.model_name = "hummingbird"
        cmd.pose.position.x = drone_state.model_state.pose.position.x = pos[0]
        cmd.pose.position.y = drone_state.model_state.pose.position.y = pos[1]
        cmd.pose.position.z = drone_state.model_state.pose.position.z = pos[2]
        cmd.heading = yaw
        quat = self.ypr2quat(yaw, 0, 0)
        drone_state.model_state.pose.orientation.w = quat[0]
        drone_state.model_state.pose.orientation.x = quat[1]
        drone_state.model_state.pose.orientation.y = quat[2]
        drone_state.model_state.pose.orientation.z = quat[3]
        drone_state.model_state.twist.linear.x = 0.0
        drone_state.model_state.twist.linear.y = 0.0
        drone_state.model_state.twist.linear.z = 0.0
        drone_state.model_state.twist.angular.x = 0.0
        drone_state.model_state.twist.angular.y = 0.0
        drone_state.model_state.twist.angular.z = 0.0
        drone_state.model_state.reference_frame = 'world'
        print(cmd.pose.position)
        result = self.gazebo_model_srv_(drone_state)
        header = Header(stamp=rospy.Time.now())
        header.frame_id = 'world'
        cmd.header = header
        self.pose_pub_.publish(cmd)

    def ypr2quat(self, yaw, pitch, roll):
        cy = math.cos(yaw*0.5)
        sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5)
        sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5)
        sr = math.sin(roll*0.5)
        quat = np.zeros(4, dtype=np.float64)
        quat[0] = cy*cp*cr + sy*sp*sr
        quat[1] = cy*cp*sr - sy*sp*cr
        quat[2] = sy*cp*sr + cy*sp*cr
        quat[3] = sy*cp*cr - cy*sp*sr
        return quat

    def CalculateRanges(self, start, end):
        range = np.zeros(3, dtype=np.float64)
        origin = np.zeros(3, dtype=np.float64)
        range[0] = math.fabs(start[0] - end[0])+16
        range[1] = math.fabs(start[1] - end[1]) + 10
        range[2] = math.fabs(start[2] - end[2]) + 8
        origin[0] = (start[0] + end[0]) / 2.0
        origin[1] = (start[1] + end[1]) / 2.0
        origin[2] = (start[2] + end[2]) / 2.0 + 2.0
        return range, origin

    def CalculateRanges(self):
        range = np.zeros(3, dtype=np.float64)
        origin = np.zeros(3, dtype=np.float64)
        range[0] = self.env_range[0]
        range[1] = self.env_range[1]
        range[2] = self.env_range[2]
        origin[0] = self.env_origin[0]
        origin[1] = self.env_origin[1]
        origin[2] = self.env_origin[2]
        return range, origin

    def TakeoffSuccess(self):
        if self.received_state_est_.position[2]>1.15 and math.fabs(self.received_state_est_.velocity[2]<0.05):
            return True
        else:
            return False

if __name__ == "__main__":
    try:
        rospy.get_master().getPid()
    except:
        print("roscore is offline, exit")
        sys.exit(-1)
    rospy.init_node('avoid_manage', anonymous=True)
    avoid_manage = AvoidManage()
    rospy.spin()
