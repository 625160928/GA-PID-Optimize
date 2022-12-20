import numpy as np
import pandas as pd
from pymoo.core.problem import ElementwiseProblem
from robot_world import test_spead_pid_parameter
from pymoo.termination import get_termination
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.optimize import minimize


class PIDProblem(ElementwiseProblem):
    def __init__(self,n_var,n_obj,lb,ub, car_speed):
        super().__init__(n_var=n_var,
                         n_obj=n_obj,
                         n_ieq_constr=0,
                         xl=lb,
                         xu=ub)
        self.car_speed=car_speed

    def _evaluate(self, x, out, *args, **kwargs):
        f1, f2 = test_spead_pid_parameter(x, self.car_speed)
        out["F"] = [f1, f2]

def GA():
    car_speed = 3.0
    n_var = 6
    n_obj = 2
    lb = np.array([0, 0, 0, 0, 0, 0])
    ub = np.array([2, 1, 1, 2, 1, 1])
    problem = PIDProblem(n_var,n_obj,lb,ub,car_speed)

    algorithm = NSGA2(
        pop_size=40,
        n_offsprings=10,
        sampling=FloatRandomSampling(),
        crossover=SBX(prob=0.9, eta=15),
        mutation=PM(eta=20),
        eliminate_duplicates=True
    )
    termination = get_termination("n_gen", 50)
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
    np.savetxt('../output/pareto_front_{:.2}.txt'.format(car_speed), pareto_front_ans, delimiter=',')
    # with open('../output/pareto_front_{:.2}.txt'.format(car_speed), 'w') as front:
    #     for ind in pareto_front_ans:
    #         print(ind)
    #         front.write(str(ind) + '\n')

if __name__ == '__main__':
    ind = [3.151840787176128256e+00, 1.860069071583392741e+00, 2.500092482125581550e-01, 3.968527246176385415e+00,
           1.102752442290751533e+00, 7.603249900164437136e-01]
    car_speed = 3.0
    show_process = True
    test_spead_pid_parameter(ind, car_speed, show_process)
    # GA()