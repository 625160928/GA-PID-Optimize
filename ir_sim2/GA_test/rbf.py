import numpy as np

class RBFmodel():
    def __init__(self,x,y,core='gaussian',parm=[1]):
        self.__data=np.array(x)
        self.__label=np.array(y)
        self.__core=core
        self.__parm=parm
        self.__table=np.zeros([len(self.__data), len(self.__data)])
        for i in range(len(self.__data)):
            for j in range(len(self.__data)):
                self.__table[i][j]=np.sqrt(sum((self.__data[i] - self.__data[j]) ** 2))

        self.__init_table()

    def __core_cal(self, d):
        if self.__core=='gaussian':
            c=self.__parm[0]
            return np.exp(-1 * c * d * d)
        print('no core function')
        return None

    def __init_table(self):
        self.__core_table=np.zeros([len(self.__data), len(self.__data)])
        # print(self.__table)
        for i in range(len(self.__data)):
            for j in range(len(self.__data)):
                d=self.__table[i][j]
                self.__core_table[i][j]=self.__core_cal(d)
        # print(self.__core_table)

    def cal_labuda(self,use_search=False):
        if use_search==False:
            re_core=np.linalg.inv(self.__core_table)

            # print(self.__label.shape,re_core.shape)
            self.__labuda=  re_core @ self.__label
            # for i in range(len(self.__labuda)):
            #     for j in range(len(self.__labuda)):
            #         print(self.__labuda[i][j],end = ' ')
            #     print()

    def get_solution(self,input_data_once):
        data=[]
        for i in range(len(self.__data)):
            # print(i,self.__data[i],input_data_once,(self.__data[i] - input_data_once))
            d=np.sqrt(sum((self.__data[i] - input_data_once) ** 2))
            data.append(self.__core_cal(d))
        data=np.array(data)
        # print(data)
        # print('data shape ',data.shape)
        # print('labuda shape ',self.__labuda.shape)
        ans=data@self.__labuda
        # print(ans.shape)
        return ans



def main():
    data_pair=[]
    x=[]
    y=[]
    for i in range(10):
        tmp=[0,0,0,0,0,0,0,0,0,0]
        tmp[i]=1
        x.append([i])
        y.append(tmp)
    rbf_model=RBFmodel(x,y)
    rbf_model.cal_labuda()
    ans=rbf_model.get_solution([8.5])

    t_ans=[]
    for i in ans:
        t_ans.append(round(i,4))
    print(t_ans)

if __name__ == "__main__":
    main()
