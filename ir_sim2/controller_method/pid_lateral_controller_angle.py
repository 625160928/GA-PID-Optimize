import numpy as np
from collections import deque
import math


class PIDLateralAngleController(object):  # pylint: disable=too-few-public-methods
    """
    PIDLateralController implements lateral control using a PID.
    """

    def __init__(self,L,dt,car_steer_limit, K_P=1.0, K_D=0.0, K_I=0.0):
        """
        :param vehicle: actor to apply to local planner logic onto
        :param K_P: Proportional term
        :param K_D: Differential term
        :param K_I: Integral term
        """
        self._K_P = K_P
        self._K_D = K_D
        self._K_I = K_I
        self._e_buffer = deque(maxlen=10)
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0
        self.__pure_pursuit_long_see = 100
        self.__pure_pursuit_dis = 0.3
        self.__L = L  # 车辆轴距，单位：m
        self.__pure_pursuit_k_f = 0.2  # 前视距离系数
        self.__pure_pursuit_Lfc = 2  # 前视距离
        self.__pure_pursuit_min_r = 0.25
        self.__dt=dt
        self.error_list=[0]
        self.path_yaw_list = [0]
        self.lateral_error_list=[0]
        #  限制车轮转角
        self.__car_steer_limit = car_steer_limit
        self.__pind=0
        self.iter_max=math.pi*6
    def get_error_list(self):
        return self.error_list
    #将加速度与横向偏移变成车辆速度与角速度
    def __get_v_from_ai_di(self, car_speed, acc):
        v = car_speed + acc * self.__dt
        return v

    def set_parm(self,p,i,d):
        self._K_P=p
        self._K_D=d
        self._K_I=i
    # 搜索最临近的路点
    def __calc_target_index(self, x, y, route_x, route_y):
        dx = [x - icx for icx in route_x]
        dy = [y - icy for icy in route_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        return ind

    def __pure_pursuit_calc_target_index(self, x, y, v, route_x, route_y):
        # pure_pursuit搜索最临近的路点
        ind = self.__calc_target_index(x, y, route_x, route_y)
        # L_ = 0.0
        #
        # Lf = self.__pure_pursuit_k_f * v + self.__pure_pursuit_Lfc
        # max_count = len(route_x) / 20
        # while Lf > L_ and (ind + 1) < len(route_x):
        #     dx = route_x[ind + 1] - route_x[ind]
        #     dy = route_x[ind + 1] - route_x[ind]
        #     L_ += math.sqrt(dx ** 2 + dy ** 2)
        #     ind += 1
        #     max_count -= 1
        #     if max_count <= 0:
        #         break

        return ind

    def run_step(self, current_pose_x,current_pose_y,current_pose_theta, waypoint_x,waypoint_y,waypointyaw,speed):
        """
        Estimate the steering angle of the vehicle based on the PID equations

        :param waypoint: target waypoint
        :param current_pose: current pose of the vehicle
        :return: steering control in the range [-1, 1]
        """
        v_begin_x = current_pose_x
        v_begin_y = current_pose_y

        yaw=current_pose_theta


        v_vec = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        w_vec = np.array([waypoint_x -
                          v_begin_x, waypoint_y -
                          v_begin_y, 0.0])

        _cross = np.cross(v_vec, w_vec)
        # if abs(speed)<0.1:
        #     self.error_list.append(self.error)
        #     return 0
        lateral_error=-(v_begin_x-waypoint_x )*math.sin(waypointyaw) +(v_begin_y-waypoint_y )*math.cos(waypointyaw)

        yaw_error1=waypointyaw-yaw

        # yaw_error2=math.acos(np.clip(lateral_error/(self.__dt*(speed)), -1.0, 1.0))

        _dot=yaw_error1
        #if _cross[2] < 0:
        # _dot *= -1.0

        previous_error = self.error
        # print('waypointyaw',waypointyaw,'error ', _dot)
        self.error = _dot
        self.lateral_error_list.append(lateral_error)
        self.error_list.append(self.error*180/math.pi)
        self.path_yaw_list.append(waypointyaw*180/math.pi)
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -self.iter_max, self.iter_max)

        self.error_derivative = self.error - previous_error
        # print('error ', self.error)
        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        # print('error ang p',round(self.error*180/math.pi,2),' i ',self.error_integral,' d ',self.error_derivative ,np.clip(output, -1.0, 1.0) )
        #会返回一个范围从-1到1的数值
        return np.clip(output, -1.0, 1.0)*self.__car_steer_limit

    #x, y, theta 车辆位置
    #route_x, route_y, route_theta 路径点集
    #pind 之前追踪的目标点
    #ai 当前需要的加速度
    def control(self, x, y, theta, v, route_x, route_y, route_theta):

        ind = self.__pure_pursuit_calc_target_index(x, y, v, route_x, route_y)

        if self.__pind >= ind:
            ind = self.__pind

        if ind < len(route_x):
            tx = route_x[ind]
            ty = route_y[ind]
            tyaw=route_theta[ind]
        else:
            tx = route_x[-1]
            ty = route_y[-1]
            tyaw = route_theta[-1]
            ind = len(route_x) - 1

        w=self.run_step(x, y, theta,tx,ty,tyaw,v)
        return (w)
        # return now_car_speed, (np.float)(w) ,ind
