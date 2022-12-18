from ir_sim2.GA_test.rbf import RBFmodel
from ir_sim2.controller_method.pid_lateral_controller import PIDLateralController
from ir_sim2.controller_method.pid_lateral_controller_angle import PIDLateralAngleController

class WeightPid():
    def __init__(self,v_arr,pid_arr):
        x=[]
        y=[]
        n=len(v_arr)
        for i in range(n):
            tmp=[]
            for j in range(n):
                tmp.append(0)
            tmp[i]=1
            x.append([i])
            y.append(tmp)
        rbf_model=RBFmodel(x,y)
        rbf_model.cal_labuda()
        self.rbf=rbf_model
        self.__init_pids(pid_arr)

    def __init_pids(self,pid_arr):
        self.__pids=[]
        for i in range(len(pid_arr)):
            self.__pids.append(0)

    def get_control(self,current_speed):
        weight=self.rbf.get_solution([current_speed])

class RbfPid():
    def __init__(self, v_arr, pid_arr):
        x = v_arr
        y = pid_arr
        rbf_model = RBFmodel(x, y)
        rbf_model.cal_labuda()
        self.rbf = rbf_model


    def get_parm(self, current_speed):
        parm = self.rbf.get_solution([current_speed])
        return parm

def main():

    x = []
    y = []
    n = 6
    for i in range(n):
        tmp = []
        for j in range(n):
            tmp.append(0)
        tmp[i] = 1
        x.append([i])
        y.append(tmp)
    rbf_pid=RbfPid(x,y)
    ans=rbf_pid.get_parm(4)

    t_ans=[]
    for i in ans:
        t_ans.append(round(i,4))
    print(t_ans)

if __name__ == "__main__":
    main()
