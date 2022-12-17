
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