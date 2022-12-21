import yaml
import math
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from ir_sim2.GA_test.route_files import get_route_s,get_route_circle,get_route_U,get_route_dir

from ir_sim2.env import EnvBase
from ir_sim2.controller_method.pid_lateral_controller import PIDLateralController
from ir_sim2.controller_method.pid_lateral_controller_angle import PIDLateralAngleController


def env1_test(env, dis_controller,ang_controller, route, max_iter=4000, speed=1, end_dist=1.0,
              show_cartoon=False,rbf_model=None,use_route_speed=False,break_dis=10):
    steer_limit=45/180*math.pi
    total_error=0
    pose_list=[]
    route_x,route_y,route_theta_r=change_path_type3(route)
    old_ind=0
    for i in range(max_iter):
        #获取车辆当前位置
        car_state=env.robot_list[0].state
        # print(car_state)
        car_position_x=car_state[0][0]
        car_position_y=car_state[1][0]
        car_position_theta_r=car_state[2][0]
        car_speed=env.robot_list[0].vel[0][0]

        # print('speed ', car_speed)
        # print(rbf_model!=None)
        if rbf_model!=None :
            # parm=rbf_model.predict(np.asarray(car_speed).reshape(-1,1)).reshape(-1)
            parm=rbf_model.get_parm(car_speed)
            print(parm)
            dis_controller.set_parm(p = parm[0], i = parm[1], d = parm[2])
            ang_controller.set_parm(p = parm[3], i = parm[4], d = parm[5])
        
        # print('speed {}'.format(car_speed))
        pose_list.append([car_position_x,car_position_y,car_position_theta_r])


        #计算车离路径的最近距离
        ind,shortest_dis = get_shortest_point(car_position_x,car_position_y,path=route)
        if ind<old_ind:
            ind=old_ind
        else:
            old_ind=ind

        # print(car_position_x,car_position_y,car_position_theta_r*180/math.pi,ind,shortest_dis)

        #计算误差
        now_error=shortest_dis
        total_error+=now_error

        if shortest_dis>break_dis:
            # print('out route ',i)
            return total_error+10000,pose_list,max_iter*2-i



        #计算控制
        steer_control_dis=dis_controller.run_step(car_position_x, car_position_y, car_position_theta_r,
                                              route[ind][0], route[ind][1], route[ind][2])
        steer_control_ang=ang_controller.run_step(car_position_x, car_position_y, car_position_theta_r,
                                              route[ind][0], route[ind][1], route[ind][2],car_speed)
        # print('=============')
        # print('dist ',steer_control_dis*180/math.pi,'  angle ',steer_control_ang*180/math.pi)
        # steer_control_dis=0
        steer_control=np.clip(steer_control_dis+steer_control_ang,-steer_limit,steer_limit)

        # print('control ',steer_control,steer_control_dis,steer_control_ang)
        if use_route_speed==True:
            car_control=[[[route[ind][3]],[steer_control]]]
        else:
            car_control=[[[speed],[steer_control]]]

        #仿真控制
        env.step(car_control)
        if show_cartoon:
            plt.plot(route_x,route_y,color='black')
            env.render(0.05)

        #结束判断
        goal_dis=math.hypot(route[-1][0]-car_position_x,route[-1][1]-car_position_y)
        if goal_dis<=end_dist:

            # print('reach goal')
            pass
        if env.done() or goal_dis<=end_dist:
            return total_error,pose_list,i
    # print('time limit ')
    return total_error,pose_list,i

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

def change_path_type1(route_x,route_y,route_theta_r,speed_arr=None):
    route=[]
    # print(len(speed_arr),len(route_y))
    for i in range(len(route_y)):
        if speed_arr==None:
            route.append([route_x[i],route_y[i],route_theta_r[i]])
        else:

            route.append([route_x[i],route_y[i],route_theta_r[i],speed_arr[i]])
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

def test_pid_parameter(model):
    #参数文件
    config_file='car_world.yaml'
    #车辆转向限制
    car_steer_limit=45 /180*math.pi
    #每步的时间
    dt=0.1
    #是否显示动画
    show_process=False
    # show_process=True
    # 离终点多近算结束
    goal_dist=1

    #pid的参数
    dis_K_P = 0.3
    dis_K_D = 0.05
    dis_K_I = 0

    ang_K_P = 0.1
    ang_K_D = 0.05
    ang_K_I = 0

    #设置车辆的移动速度
    car_speed=4

    #获取需要跟踪的路径
    # path_x,path_y,path_theta_r=get_route1([0,20,0],[40,20,0])
    # path_x,path_y,path_theta_r,path_v=get_route_s([0,20,0],[40,20,0],speed=1)
    # path_x,path_y,path_theta_r,path_v=get_route_circle([20,20],15,speed=1)
    path_x,path_y,path_theta_r,path_v=get_route_U(30,[20,35],10,speed=4)

    path=change_path_type1(path_x,path_y,path_theta_r,speed_arr=path_v)

    #设置车辆起点终点
    start_point=path[0][0:3]
    end_point=path[-1][0:3]
    # print('===============',start_point,end_point)

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
    t1_error,pose_list,iter_times=env1_test(env,pid_distance_controller, pid_angle_controller,route=path,speed=car_speed,end_dist=goal_dist,show_cartoon=show_process,rbf_model=model,use_route_speed=True)

    end_time=time.time()
    # print('cost time ',end_time-start_time,'s')

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

    # plt.show()


    # print('error is ',t1_error)

    env.end()
    return t1_error,iter_times

def test_spead_pid_parameter(ind,car_speed,path_id,show_process=False):
    #参数文件
    config_file='car_world.yaml'
    #车辆转向限制
    car_steer_limit=45 /180*math.pi
    #每步的时间
    dt=0.1
    #是否显示动画
    # show_process=False
    # show_process=True
    # 离终点多近算结束
    goal_dist=1

    #pid的参数
    dis_K_P, dis_K_I, dis_K_D, ang_K_P, ang_K_I, ang_K_D = ind

    #设置车辆的移动速度
    # car_speed=4

    #获取需要跟踪的路径

    if path_id == 0:
        path_x, path_y, path_theta_r,path_v = get_route_dir([0, 20, 0], [40, 20, 0],speed=car_speed)
    elif path_id == 1:
        path_x, path_y, path_theta_r, path_v = get_route_s([0, 20, 0], [40, 20, 0], speed=car_speed)
    elif path_id == 2:
        path_x,path_y,path_theta_r,path_v=get_route_circle([20,20],15,speed=car_speed)
    else:
        path_x,path_y,path_theta_r,path_v=get_route_U(30,[20,35],10,speed=car_speed)


    path=change_path_type1(path_x,path_y,path_theta_r,speed_arr=path_v)

    #设置车辆起点终点
    start_point=path[0][0:3]
    end_point=path[-1][0:3]
    # print('===============',start_point,end_point)

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
    env = EnvBase(config_file,plot=show_process)

    #设置pid控制器
    pid_distance_controller=PIDLateralController(L,dt,car_steer_limit,dis_K_P,dis_K_D,dis_K_I)
    pid_angle_controller=PIDLateralAngleController(L,dt,car_steer_limit,ang_K_P,ang_K_D,ang_K_I)

    start_time=time.time()
    #仿真训练
    t1_error,pose_list,iter_times=env1_test(env,pid_distance_controller, pid_angle_controller,route=path,speed=car_speed,end_dist=goal_dist,show_cartoon=show_process,rbf_model=None,use_route_speed=False)


    end_time=time.time()
    # print('cost time ',end_time-start_time,'s')

    pose_list_x,pose_list_y,pose_list_theta_r=change_path_type3(pose_list)
    #分析数据
    # error_list=anylize_path_error(ori_path=path,real_path=pose_list)
    # x_arr=np.arange(1,len(error_list)+1,1)*dt

    #误差绘图
    # plt.axis([1,x_arr[-1], -1,2])
    # plt.xlabel('x[t]')
    # plt.plot(x_arr,error_list,color='red')
    # plt.plot([0,x_arr[-1]],[0,0],color='black')

    # 路线绘图
    # plt.plot(path_x,path_y)
    # plt.plot(pose_list_x,pose_list_y)

    # plt.show()


    # print('error is ',t1_error)

    env.end()
    return t1_error,iter_times

def test_model_in_all_env(model,control=0,speed=1.4,show_pro=False):
    #参数文件
    config_file='car_world.yaml'
    #车辆转向限制
    car_steer_limit=45 /180*math.pi
    #每步的时间
    dt=0.1
    #是否显示动画
    # show_process=False
    # show_process=True
    show_process=show_pro
    # 离终点多近算结束
    goal_dist=1
    if control==0:
        #pid的参数
        dis_K_P = model[0]
        dis_K_D = model[1]
        dis_K_I = model[2]

        ang_K_P = 0
        ang_K_D = 0
        ang_K_I = 0
    else:
        #pid的参数
        dis_K_P = 0
        dis_K_D = 0
        dis_K_I = 0

        ang_K_P = model[0]
        ang_K_D = model[1]
        ang_K_I = model[2]


    #设置车辆的移动速度
    car_speed=speed

    max_turn=np.clip(int(1500/car_speed),500,3000)
    path_arr=[]

    #获取需要跟踪的路径
    # path_x,path_y,path_theta_r,path_v=get_route_dir([0,20,0],[40,20,0],speed=car_speed)
    # path_arr.append(change_path_type1(path_x,path_y,path_theta_r,speed_arr=path_v))
    #
    # path_x,path_y,path_theta_r,path_v=get_route_s([0,20,0],[40,20,0],speed=car_speed)
    # path_arr.append(change_path_type1(path_x,path_y,path_theta_r,speed_arr=path_v))
    #
    # path_x,path_y,path_theta_r,path_v=get_route_circle([20,20],15,speed=car_speed)
    # path_arr.append(change_path_type1(path_x,path_y,path_theta_r,speed_arr=path_v))

    path_x,path_y,path_theta_r,path_v=get_route_U(30,[20,35],10,speed=car_speed)
    path_arr.append(change_path_type1(path_x,path_y,path_theta_r,speed_arr=path_v))

    total_error=0
    total_iter=0

    for path in path_arr:

        #设置车辆起点终点
        start_point=path[0][0:3]
        end_point=path[-1][0:3]

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
        env = EnvBase(config_file,plot=show_process)
        # env.plot=show_process

        #设置pid控制器
        pid_distance_controller=PIDLateralController(L,dt,car_steer_limit,dis_K_P,dis_K_D,dis_K_I)
        pid_angle_controller=PIDLateralAngleController(L,dt,car_steer_limit,ang_K_P,ang_K_D,ang_K_I)

        start_time=time.time()
        # print(model==None)
        #仿真训练
        t1_error,pose_list,iter_times=env1_test(env,pid_distance_controller, pid_angle_controller,route=path,max_iter=max_turn,
                                                speed=car_speed,end_dist=goal_dist,show_cartoon=show_process,rbf_model=None,use_route_speed=False)

        end_time=time.time()
        env.end()
        # print('cost time ',end_time-start_time,'s',iter_times)
        # print('error is ',t1_error)
        total_iter+=iter_times
        total_error+=t1_error

    return total_error,total_iter


def main():
    #参数文件
    config_file='car_world.yaml'
    #车辆转向限制
    car_steer_limit=45 /180*math.pi
    #每步的时间
    dt=0.1
    #是否显示动画
    show_process=False
    # show_process=True
    # 离终点多近算结束
    goal_dist=1

    #pid的参数
    #0.5764250986766484, 1.879224972658214, 0.8307444833606002, 4.384537513353423, 0.895612962471191, 0.27693529780814263
    dis_K_P = 0.5
    dis_K_D =  0
    dis_K_I = 0.01
    # dis_K_P = 0
    # dis_K_D =  0
    # dis_K_I = 0

    # ang_K_P =4.384537513353423
    # ang_K_D = 0.895612962471191
    # ang_K_I = 0.27693529780814263
    ang_K_P = 0
    ang_K_D = 0
    ang_K_I = 0

    #设置车辆的移动速度
    car_speed=3

    #获取需要跟踪的路径
    # path_x,path_y,path_theta_r=get_route1([0,20,0],[40,20,0])
    # path_x,path_y,path_theta_r,path_v=get_route_s([0,20,0],[40,20,0],speed=1)
    # path_x,path_y,path_theta_r,path_v=get_route_circle([20,20],15,speed=1)
    path_x,path_y,path_theta_r,path_v=get_route_U(30,[20,35],10,speed=4)

    path=change_path_type1(path_x,path_y,path_theta_r,speed_arr=path_v)

    #设置车辆起点终点
    start_point=path[0][0:3]
    end_point=path[-1][0:3]
    # print('===============',start_point,end_point)

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
    env = EnvBase(config_file,plot=show_process)

    #设置pid控制器
    pid_distance_controller=PIDLateralController(L,dt,car_steer_limit,dis_K_P,dis_K_D,dis_K_I)
    pid_angle_controller=PIDLateralAngleController(L,dt,car_steer_limit,ang_K_P,ang_K_D,ang_K_I)

    start_time=time.time()
    #仿真训练
    t1_error,pose_list,iter_times=env1_test(env,pid_distance_controller, pid_angle_controller,
                                            route=path,speed=car_speed,end_dist=goal_dist,show_cartoon=show_process,use_route_speed=False)

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
    if show_process:
        plt.show()


    print('error is ',t1_error)

    env.end()


def test_rbf_parm_pid():
    from ir_sim2.GA_test.rbf_pid import RbfPid
    v_arr=[]
    parm_arr=[]
    v_arr.append([0.2])
    parm_arr.append([6.631179399694174, 1.2519445714834974, 1.8202757779972165, 0.015133312965817103, 0.45177386351235305, 0.12510221029362378])
    v_arr.append([0.6])
    parm_arr.append([11.619609618311, 4.4225134285476475, 0.18103029135498205, 0.6716269595558078, 0.6309650005210373, 0.1932678508697221])
    v_arr.append([1.0])
    parm_arr.append([10.848542963745997, 0.4866148658867582, 0.198933858287078, 1.3194756050149214, 0.5898385353992489, 0.9955664753478234])
    v_arr.append([1.4])
    parm_arr.append([1.6117808510413938, 0.12202209424231114, 1.136392217364974, 0.40138068892082046, 1.2516250426094335, 1.4335769466315929])
    v_arr.append([1.8])
    parm_arr.append([0.8148088699150735, 1.9400366705910956, 1.6058691342502738, 0.5840434745717826, 1.9662391441900615, 2.89045910555941])
    v_arr.append([ 2.2   ])
    parm_arr.append([  9.69583037510899, 1.223886994720352, 0.6234256284619624, 1.3774660611241316, 1.624064303963203, 1.442778174326632  ])
    v_arr.append([ 2.6   ])
    parm_arr.append([ 0.42281825265616524, 1.1632779822819121, 1.717776686565698, 2.3413898157105995, 0.11518691464205888, 2.2064311690989538  ])
    v_arr.append([ 3.0   ])
    parm_arr.append([ 11.35601775334114, 2.461330496856098, 0.24621840438688603, 1.4327270239820475, 0.04757623969233293, 0.7257789003586954  ])
    v_arr.append([ 3.4   ])
    parm_arr.append([11.503980074940594, 0.8349539383719892, 0.15481947654693873, 1.5656267221851938, 0.03706243171762191, 0.9619398100703707  ])
    v_arr.append([ 3.8   ])
    parm_arr.append([ 15.551359942151784, 2.6964727487000744, 0.2958757511436185, 0.912639941984511, 1.3889910687413245, 0.5374831522290826 ])

    parmnp_arr = []
    for i in parm_arr:
        parmnp_arr.append(np.array(i))
    # for i in range(len(v_arr)):
    #     print(v_arr[i],type(v_arr[i][0]))
    #     print(parm_arr[i],type(parm_arr[i][0]))
    model=RbfPid(v_arr,parmnp_arr)


    test_model_in_all_env(model)

if __name__=="__main__":
    # test_rbf_parm_pid()
    # main()
    parm=  [1.1512398965943231, 0.7832528746335857, 1.1264852450737861]

    test_model_in_all_env(parm,1,0.5,show_pro=True)