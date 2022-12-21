import numpy as np
import pandas as pd
from pymoo.core.problem import ElementwiseProblem
from robot_world import test_spead_pid_parameter
from pymoo.termination import get_termination
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.algorithms.moo.sms import SMSEMOA
from pymoo.operators.crossover.sbx import SBX
from pymoo.operators.mutation.pm import PM
from pymoo.operators.sampling.rnd import FloatRandomSampling
from pymoo.optimize import minimize


class PIDProblem(ElementwiseProblem):
    def __init__(self, n_var, n_obj, lb, ub, car_speed):
        super().__init__(n_var=n_var,
                         n_obj=n_obj,
                         n_ieq_constr=0,
                         xl=lb,
                         xu=ub)
        self.car_speed = car_speed

    def _evaluate(self, x, out, *args, **kwargs):
        F1,F2=0,0
        for path_id in range(4):
            f1, f2 = test_spead_pid_parameter(x, self.car_speed,path_id)
            F1+=f1
            F2+=f2

        out["F"] = [F1, F2]


def GA():
    car_speed = 4
    n_var = 6
    n_obj = 2
    lb = np.array([0, 0, 0, 0, 0, 0])
    ub = np.array([2, 1, 1, 2, 1, 1])
    # for car_speed in np.arange(0.4, 4, 0.4):
    # car_speed_list = [0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.2, 3.6, 4]
    car_speed_list = [4]
    for car_speed in car_speed_list:
        print('car_speed',car_speed)
        problem = PIDProblem(n_var, n_obj, lb, ub, car_speed)

        algorithm = SMSEMOA(
            pop_size=40,
            n_offsprings=10,
            sampling=FloatRandomSampling(),
            crossover=SBX(prob=0.9, eta=15),
            mutation=PM(eta=20),
            eliminate_duplicates=True
        )
        termination = get_termination("n_gen", 200)
        res = minimize(problem,
                       algorithm,
                       termination,
                       seed=1,
                       save_history=True,
                       verbose=True)

        X = res.X
        F = res.F

        pareto_front_ans = np.concatenate((np.array(X), np.array(F)), axis=1)
        # pareto_front_ans = pareto_front_ans*2
        # pareto_front_ans.tofile('data2.csv', sep = ',')
        np.savetxt('../output/pareto_front_{}.txt'.format(car_speed), pareto_front_ans, delimiter=',')
    # with open('../output/pareto_front_{:.2}.txt'.format(car_speed), 'w') as front:
    #     for ind in pareto_front_ans:
    #         print(ind)
    #         front.write(str(ind) + '\n')


if __name__ == '__main__':
    ind = [3.704684427523429746e-01,2.139334815023568453e-02,7.996309266955553552e-01,1.739088131203671006e+00,2.366352920663344073e-01,3.962417872380616934e-01]
    car_speed = 4
    show_process = True
    test_spead_pid_parameter(ind, car_speed, 1,show_process)
    # GA()
