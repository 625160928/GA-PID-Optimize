import numpy as np
import pandas as pd
import os
from pymoo.core.problem import ElementwiseProblem
from robot_world import test_spead_pid_parameter
from sklearn.kernel_ridge import KernelRidge
from pymoo.termination import get_termination
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.optimize import minimize
from robot_world import test_pid_parameter


class PIDProblem(ElementwiseProblem):
    def __init__(self,n_var,n_obj,lb,ub):
        super().__init__(n_var=n_var,
                         n_obj=n_obj,
                         n_ieq_constr=0,
                         xl=lb,
                         xu=ub)

    def _evaluate(self, x, out, *args, **kwargs):
        X = np.arange(0.2,4,0.4).reshape(-1,1)
        y = np.asarray(x).reshape(-1,6)
        model = KernelRidge(alpha=1.0)
        model.fit(X,y)
        F1,F2=0,0
        for path_id in range(0, 4):
            for speed in range(1,5):
                f1, f2 = test_pid_parameter(model,speed,path_id ,False)
                F1+=f1
                F2+=f2
        out["F"] = [F1/F2, F2]

def GA():
    n_var = 60
    n_obj = 2
    lb = np.zeros(60)
    ub = np.ones(60)
    for i in range(20):
        ub[i*3] = 2
    pass

    problem = PIDProblem(n_var,n_obj,lb,ub)

    algorithm = NSGA2(
        pop_size=100,
        n_offsprings=10,
        sampling=FloatRandomSampling(),
        crossover=SBX(prob=0.9, eta=15),
        mutation=PM(eta=20),
        eliminate_duplicates=True
    )
    termination = get_termination("n_gen", 1000)
    res = minimize(problem,
                   algorithm,
                   termination,
                   seed=1,
                   save_history=True,
                   verbose=True)

    X = res.X
    F = res.F

    pareto_front_ans = np.concatenate((np.array(X), np.array(F)), axis=1)
    pareto_front_ans = pareto_front_ans*2
    pareto_front_ans.tofile('data2.csv', sep = ',')
    with open('./pareto_front.txt', 'w') as front:
        front.write('')
    np.savetxt('areto_front.txt', pareto_front_ans, delimiter=',')
    # with open('../output/pareto_front_{:.2}.txt'.format(car_speed), 'w') as front:
    #     for ind in pareto_front_ans:
    #         print(ind)
    #         front.write(str(ind) + '\n')

if __name__ == '__main__':
    # ind = [3.750300356647885192e+00,1.030159720710949411e+00,4.417624295582034399e-01,3.971451140049683470e+00,2.583085823514462698e-01,1.211155511068839202e-02,1.337473167006886676e+00,9.195573510150270580e-02,1.113593940872910748e+00,3.999926355765583974e+00,8.527923866231869043e-01,1.997467469903495108e+00,2.043342966467817323e+00,2.665082523224391875e-01,8.276678398709491624e-01,3.998326031727455376e+00,4.664629266073316849e-01,1.038868943003399270e+00,3.902230490939109231e+00,7.860131642508487448e-02,1.979068005716478451e+00,3.996090058628689601e+00,7.050954786719181300e-01,1.984658347973787151e+00]
    # # [3.750300356647885192e+00,1.030159720710949411e+00,4.417624295582034399e-01,3.971451140049683470e+00,2.583085823514462698e-01,1.211155511068839202e-02,1.337473167006886676e+00,9.195573510150270580e-02,1.113593940872910748e+00,3.999926355765583974e+00,8.527923866231869043e-01,1.997467469903495108e+00,2.043342966467817323e+00,2.665082523224391875e-01,8.276678398709491624e-01,3.998326031727455376e+00,4.664629266073316849e-01,1.038868943003399270e+00,3.902230490939109231e+00,7.860131642508487448e-02,1.979068005716478451e+00,3.996090058628689601e+00,7.050954786719181300e-01,1.984658347973787151e+00]
    # X = np.arange(0.5,4,1).reshape(-1,1)
    # y = np.asarray(ind).reshape(-1,6)
    # model = KernelRidge(alpha=1.0)
    # model.fit(X,y)
    # f1, f2 = test_pid_parameter(model,1,2,True)
    # print(f1)
    # print(f2)
    GA()