import numpy as np
import pandas as pd
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
        X = np.arange(0.5,4,1).reshape(-1,1)
        y = np.asarray(x).reshape(-1,6)
        model = KernelRidge(alpha=1.0)
        model.fit(X,y)
        f1, f2 = test_pid_parameter(model,False)
        out["F"] = [f1, f2]

def GA():
    n_var = 24
    n_obj = 2
    lb = np.zeros(24)
    ub = np.ones(24)
    for i in range(8):
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
    np.savetxt('areto_front.txt', pareto_front_ans, delimiter=',')
    # with open('../output/pareto_front_{:.2}.txt'.format(car_speed), 'w') as front:
    #     for ind in pareto_front_ans:
    #         print(ind)
    #         front.write(str(ind) + '\n')

if __name__ == '__main__':
#     ind = [9.404604045995696993e-01,1.599548769572369800e+00,8.437550288483947059e-02,3.515285627167905602e+00,2.531957976144185718e-01,1.594269689323443151e-01,3.005726008079294242e+00,3.370993116147930624e-01,6.503581162200320342e-01,4.300708963239069926e-01,9.755480017216335842e-01,1.872757359358949714e-01,8.782074319869852541e-01,1.189763183305670297e-01,1.202293754216288368e+00,3.988525340856445833e+00,1.825991299699351700e+00,6.016680902519394580e-02,2.366545668345299447e+00,7.032737712113035222e-02,1.094372576428940924e+00,3.282182222207676414e+00,1.450303787541600720e+00,1.126259973832459238e+00
# ]
#     X = np.arange(0.5,4,1).reshape(-1,1)
#     y = np.asarray(ind).reshape(-1,6)
#     model = KernelRidge(alpha=1.0)
#     model.fit(X,y)
#     f1, f2 = test_pid_parameter(model,True)
    GA()