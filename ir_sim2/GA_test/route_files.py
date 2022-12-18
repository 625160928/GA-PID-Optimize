
import math

def get_route_s(start_point,end_point,step=0.1,speed=4):
    x_arr=[]
    y_arr=[]
    theta_arr=[]
    v_arr=[]

    theta_r=math.atan2(end_point[1]-start_point[1],end_point[0]-start_point[0])
    dist=math.hypot(end_point[1]-start_point[1],end_point[0]-start_point[0])
    numb=int(dist/step)
    #添加起点
    x_arr.append(start_point[0])
    y_arr.append(start_point[1])
    theta_arr.append(theta_r)
    v_arr.append(speed/2)
    for i in range(numb-1):
        now_pose_x=start_point[0]+i*step
        now_pose_y=start_point[1]+math.sin(i*step)
        now_theta_r=math.cos(i*step)
        x_arr.append(now_pose_x)
        y_arr.append(now_pose_y)
        theta_arr.append(now_theta_r)
        v_arr.append(speed/2)
        # print(now_pose_x,now_pose_y,now_theta_r*180/math.pi)


    return x_arr,y_arr,theta_arr,v_arr

def get_route_circle(center,radiu,step=0.1,speed=4):
    x_arr=[]
    y_arr=[]
    theta_arr=[]
    v_arr=[]

    numb=int(math.pi*2*radiu/step)
    theta_r_t=step/radiu


    v=speed/2
    for i in range(numb-100):
        x=center[0]+math.cos(i*theta_r_t)*radiu
        y=center[1]+math.sin(i*theta_r_t)*radiu
        theta=i*theta_r_t+math.pi/2

        x_arr.append(x)
        y_arr.append(y)
        theta_arr.append(theta)
        v_arr.append(v)
    # print(len(x_arr))







    return x_arr,y_arr,theta_arr,v_arr

def get_route_U(dis,center,radiu,step=0.1,speed=4):
    x_arr=[]
    y_arr=[]
    theta_arr=[]
    v_arr=[]

    dec_dis=4

    #添加入场路径
    start_x=center[0]-radiu
    start_y=center[1]-dis
    if dis<dec_dis:
        numb=int(dis/step)
        for i in range(numb):
            x=start_x
            y=start_y+i*step
            theta=math.pi/2
            v=speed/2*(2-i/numb)
            x_arr.append(x)
            y_arr.append(y)
            theta_arr.append(theta)
            v_arr.append(v)
    else:
        numb=int((dis-dec_dis)/step)
        for i in range(numb):
            x=start_x
            y=start_y+i*step
            theta=math.pi/2
            v=speed
            x_arr.append(x)
            y_arr.append(y)
            theta_arr.append(theta)
            v_arr.append(v)
        numb=int((dec_dis)/step)
        for i in range(numb):
            x=start_x
            y=start_y+dis-dec_dis
            theta=math.pi/2
            v=speed/2*(2-i/numb)
            x_arr.append(x)
            y_arr.append(y)
            theta_arr.append(theta)
            v_arr.append(v)

    # 添加半圆

    numb=int(math.pi*radiu/step)
    theta_r_t=step/radiu
    for i in range(numb):
        x=center[0]+math.cos((numb-i)*theta_r_t)*radiu
        y=center[1]+math.sin((numb-i)*theta_r_t)*radiu
        theta=(numb-i)*theta_r_t+math.pi/2
        v=speed/2
        x_arr.append(x)
        y_arr.append(y)
        theta_arr.append(theta)
        v_arr.append(v)


    # 添加出场路径
    start_x=center[0]+radiu
    start_y=center[1]
    if dis<dec_dis:
        numb=int(dis/step)
        for i in range(numb):
            x=start_x
            y=start_y-i*step
            theta=-math.pi/2
            v=speed/2*(1+i/numb)
            x_arr.append(x)
            y_arr.append(y)
            theta_arr.append(theta)
            v_arr.append(v)
    else:
        numb=int((dec_dis)/step)
        for i in range(numb):
            x=start_x
            y=start_y-i*step
            theta=-math.pi/2
            v=speed/2*(1+i/numb)
            x_arr.append(x)
            y_arr.append(y)
            theta_arr.append(theta)
            v_arr.append(v)

        numb=int((dis-dec_dis)/step)
        for i in range(numb):
            x=start_x
            y=start_y-i*step-dec_dis
            theta=-math.pi/2
            v=speed
            x_arr.append(x)
            y_arr.append(y)
            theta_arr.append(theta)
            v_arr.append(v)


    # for i in range(len(v_arr)):
    #     print(i,v_arr[i])




    return x_arr,y_arr,theta_arr,v_arr


def get_route_dir(start_point,end_point,step=0.1,speed=1):
    x_arr=[]
    y_arr=[]
    theta_arr=[]
    v_arr=[]
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
    v_arr.append(speed)

    for i in range(numb-1):
        now_pose_x+=dir_x
        now_pose_y+=dir_y
        x_arr.append(now_pose_x)
        y_arr.append(now_pose_y)
        theta_arr.append(theta_r)
        v_arr.append(speed)
    return x_arr,y_arr,theta_arr,v_arr