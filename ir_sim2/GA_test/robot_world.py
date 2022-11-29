import yaml
import math
import numpy as np
import time
from ir_sim2.env import EnvBase
from ir_sim2.controller_method.pid_lateral_controller import PIDLateralController


def env1_test(env,controller,route,max_iter=3000,speed=1,show_cartoon=False):
    total_error=0
    for i in range(max_iter):
        #获取车辆当前位置
        car_state=env.robot_list[0].state
        car_position_x=car_state[0][0]
        car_position_y=car_state[1][0]
        car_position_theta_r=car_state[2][0]


        #计算车离路径的最近距离
        ind,shortest_dis = get_shortest_point(car_position_x,car_position_y,path=route)
        # print(car_position_x,car_position_y,car_position_theta_r*180/math.pi,ind,shortest_dis)

        #计算误差
        now_error=shortest_dis
        total_error+=now_error

        #计算控制
        steer_control=controller.run_step(car_position_x,car_position_y,car_position_theta_r,
                                          route[ind][0],route[ind][1])
        car_control=[[[speed],[steer_control]]]
        # print('=========')
        # print(car_control[0],type(car_control))
        # print('control',steer_control)
        # print(des_vel)

        # des_vel = env.cal_des_vel()
        # car_control=des_vel
        # print(car_control[0],type(car_control))

        #仿真控制
        env.step(car_control)
        if show_cartoon:
            env.render(0.05)

        if env.done():
            return total_error
    return total_error

def get_route1(start_point,end_point,step=0.1):
    x_arr=[]
    y_arr=[]
    theta_arr=[]
    theta_r=math.atan2(end_point[1]-start_point[1],end_point[0]-start_point[0])
    dist=math.hypot(end_point[1]-start_point[1],end_point[0]-start_point[0])
    numb=int(dist/step)
    dir_x=math.cos(theta_r)*step
    dir_y=math.sin(theta_r)*step
    #路径起点
    now_pose_x=start_point[0]
    now_pose_y=start_point[1]
    #添加起点
    x_arr.append(start_point[0])
    y_arr.append(start_point[1])
    theta_arr.append(theta_r)

    for i in range(numb-1):
        now_pose_x+=dir_x
        now_pose_y+=dir_y
        x_arr.append(now_pose_x)
        y_arr.append(now_pose_y)
        theta_arr.append(theta_r)
    return x_arr,y_arr,theta_arr

def change_path_type(route_x,route_y,route_theta_r):
    route=[]
    for i in range(len(route_y)):
        route.append([route_x[i],route_y[i],route_theta_r[i]])
    return route

def get_shortest_point(car_position_x,car_position_y,path):
    ind=0
    short_dis=math.hypot(car_position_x-path[0][0],car_position_y-path[0][1])
    for  i in range(1,len(path)):
        dis=math.hypot(car_position_x-path[i][0],car_position_y-path[i][1])
        if dis<short_dis:
            short_dis=dis
            ind=i
    return ind,short_dis

def main():
    #参数文件
    config_file='car_world.yaml'
    #车辆转向限制
    car_steer_limit=30 /180*math.pi
    #每步的时间
    dt=0.1
    show_process=False

    #pid的参数
    K_P = 0.5
    K_D = 0.1
    K_I = 0

    #设置车辆的移动速度
    car_speed=2

    #获取需要跟踪的路径
    path_x,path_y,path_theta_r=get_route1([0,20,0],[40,20,0])
    path=change_path_type(path_x,path_y,path_theta_r)

    #设置车辆起点终点
    start_point=path[0]
    end_point=path[-1]
    start_point[1]-=10

    #加载设置文件参数
    f = open(config_file, 'r', encoding='utf-8')
    cont = f.read()
    parm = yaml.load(cont,Loader=yaml.FullLoader)
    L=parm['robots']['shape'][2]

    #重新设置配置文件中车辆位置
    parm['robots']['state']=start_point+[0]
    parm['robots']['goal']=end_point
    with open(config_file, 'w') as file:
        file.write(yaml.dump(parm, allow_unicode=True))

    #设置仿真环境
    env = EnvBase(config_file)
    env.plot=show_process

    #设置pid控制器
    pid_controller=PIDLateralController(L,dt,car_steer_limit,K_P,K_D,K_I)

    start_time=time.time()
    #仿真训练
    t1_error=env1_test(env,pid_controller,route=path,speed=car_speed,show_cartoon=show_process)
    end_time=time.time()
    print('cost time ',end_time-start_time,'s')

    print('error is ',t1_error)

    env.end()

if __name__=="__main__":
    main()
    
