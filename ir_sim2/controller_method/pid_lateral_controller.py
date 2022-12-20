import numpy as np
from collections import deque
import math
class Point():
    def __init__(self):
        self.x=0
        self.y=0
def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return ((ang1 - ang2) % (2 * np.pi))
def rotation(vector, theta):
    v1_new = (vector[0] * np.cos(theta)) - (vector[1] * np.sin(theta))
    v2_new = (vector[1] * np.cos(theta)) + (vector[0] * np.sin(theta))
    z_trans = [v1_new, v2_new, vector[2]]
    line_yz = [0., 1.]
    theta2 = angle_between(line_yz, [z_trans[1], z_trans[2]])
    v1_new = (z_trans[0] * np.cos(theta2)) - (z_trans[1] * np.sin(theta2))
    v2_new = (z_trans[1] * np.cos(theta2)) + (z_trans[0] * np.sin(theta2))
    y_trans = np.array([z_trans[0], v1_new, v2_new])
    return z_trans, y_trans
class PIDLateralController(object):  # pylint: disable=too-few-public-methods
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
        self.max_i=40

        #  限制车轮转角
        self.__car_steer_limit = car_steer_limit
        self.__pind=0

    def set_parm(self,p,i,d):
        self._K_P=p
        self._K_D=d
        self._K_I=i
        
    #将加速度与横向偏移变成车辆速度与角速度
    def __get_v_from_ai_di(self, car_speed, acc):
        v = car_speed + acc * self.__dt
        return v

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
        L_ = 0.0

        Lf = self.__pure_pursuit_k_f * v + self.__pure_pursuit_Lfc
        max_count = len(route_x) / 20
        while Lf > L_ and (ind + 1) < len(route_x):
            dx = route_x[ind + 1] - route_x[ind]
            dy = route_x[ind + 1] - route_x[ind]
            L_ += math.sqrt(dx ** 2 + dy ** 2)
            ind += 1
            max_count -= 1
            if max_count <= 0:
                break

        return ind

    def run_step(self, current_pose_x,current_pose_y,current_pose_theta, waypoint_x,waypoint_y,waypoint_theta):

        dx=current_pose_x-waypoint_x
        dy=current_pose_y-waypoint_y
        cos_target_heading=math.cos(waypoint_theta)
        sin_target_heading=math.sin(waypoint_theta)
        lateral_error_m = cos_target_heading * dy - sin_target_heading * dx
        lateral_error_m *=-1
        previous_error = self.error

        # self.error = _dot
        self.error = lateral_error_m
        # restrict integral term to avoid integral windup
        self.error_integral = np.clip(self.error_integral + self.error, -1*self.max_i, self.max_i)
        self.error_derivative = self.error - previous_error

        output = self._K_P * self.error + self._K_I * self.error_integral + self._K_D * self.error_derivative
        # print('dis output  ', np.clip(output, -1.0, 1.0)*self.__car_steer_limit*180/math.pi,self.error)
        # 会返回一个范围从-1到1的数值
        return np.clip(output, -1.0, 1.0)*self.__car_steer_limit

    #x, y, theta 车辆位置
    #route_x, route_y, route_theta 路径点集
    #pind 之前追踪的目标点
    #ai 当前需要的加速度
    def control(self, x, y, theta, v, route_x, route_y, route_theta, ai):

        ind = self.__pure_pursuit_calc_target_index(x, y, v, route_x, route_y)

        if self.__pind >= ind:
            ind = self.__pind

        if ind < len(route_x):
            tx = route_x[ind]
            ty = route_y[ind]
        else:
            tx = route_x[-1]
            ty = route_y[-1]
            ind = len(route_x) - 1

        w=self.run_step(x, y, theta,tx,ty)
        # delta, ind =self.__pure_pursuit_control( x, y, theta, v, route_x, route_y,pind)

        now_car_speed= self.__get_v_from_ai_di(v, ai)
        if now_car_speed<0:
            re=-1
        else:
            re=1
        return now_car_speed, (np.float)(re*w),ind
        # return now_car_speed, (np.float)(w) ,ind
