import yaml
import math
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

from ir_sim2.env import EnvBase
from ir_sim2.controller_method.pid_lateral_controller import PIDLateralController
from ir_sim2.controller_method.pid_lateral_controller_angle import PIDLateralAngleController


def env1_test(env, dis_controller,ang_controller, route, max_iter=3000, speed=1, end_dist=1.0, show_cartoon=False):
    steer_limit=45/180*math.pi
    total_error=0
    pose_list=[]
    route_x,route_y,route_theta_r=change_path_type3(route)
    for i in range(max_iter):
        #获取车辆当前位置
        car_state=env.robot_list[0].state
        # print(car_state)
        car_position_x=car_state[0][0]
        car_position_y=car_state[1][0]
        car_position_theta_r=car_state[2][0]
        car_speed=env.robot_list[0].vel
        # print('speed ',car_speed,)
        pose_list.append([car_position_x,car_position_y,car_position_theta_r])


        #计算车离路径的最近距离
        ind,shortest_dis = get_shortest_point(car_position_x,car_position_y,path=route)
        # print(car_position_x,car_position_y,car_position_theta_r*180/math.pi,ind,shortest_dis)

        #计算误差
        now_error=shortest_dis
        total_error+=now_error



        #计算控制
        steer_control_dis=dis_controller.run_step(car_position_x, car_position_y, car_position_theta_r,
                                              route[ind][0], route[ind][1])
        steer_control_ang=-ang_controller.run_step(car_position_x, car_position_y, car_position_theta_r,
                                              route[ind][0], route[ind][1], route[ind][2],car_speed)
        # print('=============')
        # print('dist ',steer_control_dis*180/math.pi,'  angle ',steer_control_ang*180/math.pi)

        steer_control=np.clip(steer_control_dis+steer_control_ang,-steer_limit,steer_limit)


        car_control=[[[speed],[steer_control]]]

        #仿真控制
        env.step(car_control)
        if show_cartoon:
            plt.plot(route_x,route_y,color='black')
            env.render(0.05)

        #结束判断
        goal_dis=math.hypot(route[-1][0]-car_position_x,route[-1][1]-car_position_y)
        if goal_dis<=end_dist:
            print('reach goal')
        if env.done() or goal_dis<=end_dist:
            return total_error,pose_list
    return total_error,pose_list

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

def change_path_type1(route_x,route_y,route_theta_r):
    route=[]
    for i in range(len(route_y)):
        route.append([route_x[i],route_y[i],route_theta_r[i]])
    return route

def change_path_type3(route):
    route_x, route_y, route_theta_r=[],[],[]
    for i in range(len(route)):
        route_x.append(route[i][0])
        route_y.append(route[i][1])
        route_theta_r.append(route[i][2])



    return route_x, route_y, route_theta_r

def get_shortest_point(car_position_x,car_position_y,path):
    ind=0
    short_dis=math.hypot(car_position_x-path[0][0],car_position_y-path[0][1])
    for  i in range(1,len(path)):
        dis=math.hypot(car_position_x-path[i][0],car_position_y-path[i][1])
        if dis<short_dis:
            short_dis=dis
            ind=i
    return ind,short_dis

def anylize_path_error(ori_path,real_path):
    error_list=[]
    for i in range(len(ori_path)):
        min_dis=math.hypot(ori_path[i][0]-real_path[0][0],ori_path[i][1]-real_path[0][1])
        for j in range(1,len(real_path)):
            dis=math.hypot(ori_path[i][0]-real_path[j][0],ori_path[i][1]-real_path[j][1])
            if dis<min_dis:
                min_dis=dis
        # print(min_dis)
        error_list.append(min_dis)
    # print(error_list)
    return error_list




def main():
    #参数文件
    config_file='car_world.yaml'
    #车辆转向限制
    car_steer_limit=45 /180*math.pi
    #每步的时间
    dt=0.1
    #是否显示动画
    # show_process=False
    show_process=True
    # 离终点多近算结束
    goal_dist=1

    #pid的参数
    dis_K_P = 0.02
    dis_K_D = 0.05
    dis_K_I = 0

    ang_K_P = 0.2
    ang_K_D = 0.05
    ang_K_I = 0

    #设置车辆的移动速度
    car_speed=4

    #获取需要跟踪的路径
    path_x,path_y,path_theta_r=get_route1([0,20,0],[40,20,0])
    path=change_path_type1(path_x,path_y,path_theta_r)

    #设置车辆起点终点
    start_point=path[0]
    end_point=path[-1]

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
    pid_distance_controller=PIDLateralController(L,dt,car_steer_limit,dis_K_P,dis_K_D,dis_K_I)
    pid_angle_controller=PIDLateralAngleController(L,dt,car_steer_limit,ang_K_P,ang_K_D,ang_K_I)

    start_time=time.time()
    #仿真训练
    t1_error,pose_list=env1_test(env,pid_distance_controller, pid_angle_controller,route=path,speed=car_speed,end_dist=goal_dist,show_cartoon=show_process)

    end_time=time.time()
    print('cost time ',end_time-start_time,'s')

    pose_list_x,pose_list_y,pose_list_theta_r=change_path_type3(pose_list)
    #分析数据
    error_list=anylize_path_error(ori_path=path,real_path=pose_list)
    x_arr=np.arange(1,len(error_list)+1,1)*dt

    #误差绘图
    # plt.axis([1,x_arr[-1], -1,2])
    # plt.xlabel('x[t]')
    # plt.plot(x_arr,error_list,color='red')
    # plt.plot([0,x_arr[-1]],[0,0],color='black')

    # 路线绘图
    # plt.plot(path_x,path_y)
    # plt.plot(pose_list_x,pose_list_y)

    plt.show()


    print('error is ',t1_error)

    env.end()

if __name__=="__main__":
    main()
    
