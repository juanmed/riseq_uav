import numpy as np
import quadratic_programming as qp
import cProfile
from pycallgraph import PyCallGraph
from pycallgraph import output
from pycallgraph.output import GraphvizOutput
import graphviz
import os
from multiprocessing import Process


# random_search
def random_search(order, waypoint, keyframe, current_state, init_time):
    # parameter for genetic algorithm
    entity = 10
    iteration = 1

    m = waypoint - 1
    time = np.ones((entity, m))
    for i in range(0, entity):
        time[i][:] = init_time

    sol_x = 0
    val = np.zeros(entity)

    print time
    while True:
        gene = np.zeros((entity, m))
        for i in range(0, entity):
            gene[i][:] = np.random.randint(-1, 2, m) * 0.1
        time = time + gene

        procs = []

        for i in range(0, entity):
            sol_x, val[i] = qp.qp_solution(order, waypoint, keyframe, current_state, time[i])
            #proc = Process(target=qp.qp_solution, args=(order, waypoint, keyframe, current_state, time[i]))
            #procs.append(proc)
            #proc.start()

        #for proc in procs:
        #    proc.join()
        #index = np.argmin(val)
        #print val
        #print index

        #for i in range(0, entity):
        #    time[i][:] = time[index][:]

        iteration = iteration - 1
        if iteration == 0:
            #print val[index]
            #print time[0]
            return sol_x
            #return 0

# Genetic Algorithm
def genetic_algorithm_test(m, init_time, entity, iteration):
    time = np.ones((entity, m)) * init_time / m
    gene = np.zeros((entity, m))
    for i in range(0, entity):
        gene[i] = np.random.randint(-1, 2, m) * 0.1

    while(1):
        if iteration == 0:
            break

        new_time = time + gene
        sum = np.zeros(entity)
        for i in range(0, entity):
            sum[i] = np.sum(new_time[i])

        index = np.zeros(2)
        for i in range(0, len(index)):
            index[i] = np.argmin(sum)
            sum[int(index[i])] = init_time * 10.0

        gene_best = np.array([gene[int(index[0])], gene[int(index[1])]])
        gene_best = np.reshape(gene_best, (2, m))

        gene[0] = gene_best[0]
        gene[1] = gene_best[1]

        for i in range(0, entity-2):
            for j in range(0, m):
                ran = np.random.randint(0, 2)
                mut_ran = np.random.randint(0, 10)
                if mut_ran != 0:
                    if ran == 0:
                        gene[i+2][j] = gene[0][j]
                    else:
                        gene[i + 2][j] = gene[1][j]
                else:
                    gene[i+2][j] = np.random.randint(-1, 2) * 0.1
        iteration = iteration - 1
    print gene


# TODO
    ### Function which search minimum time
    # Not Finish
    def optimal_time(self):
        sol_x = random_search(self.order, self.waypoint, self.keyframe, self.current_state, self.time_scaling)
        #dt.draw_in_plot(sol_x, self.order, self.waypoint, self.keyframe)

    # TODO
    # It would be long time to study and apply GDM
    # so for now, postpone this work later
    ''' 
    # Constrained Gradient Descent method
    def CGDmethod(self, total_time, iteration, precision):
        # h is some small number
        h = 0.0000001
        # m is segment
        m = self.waypoint -1

        # First, time is set as equal at each segment.
        time = np.ones(m) * total_time / m
        for i in range(0, iteration):
            print "hi"
            prev_time = time
            sol_x, f_before = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, prev_time)
            prev_val = f_before
            for j in range(0, m):
                g = np.ones(m) * (-1/(m-1))
                g[j] = 1
                print h*g
                print prev_time
                sol_x, f_after = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, prev_time + h*g)
                print prev_time
                gradient = (f_after - f_before)/h
                print gradient
                time[j] = time[j] - gradient
                print prev_time
            sol_x, val = qp.qp_solution(self.order, self.waypoint, self.keyframe, self.current_state, time)
            if abs(val - prev_val) < precision:
                return time, sol_x
    '''

    """
    This is python's profile to check taking time and number of time
    """
    # traj_gen.optimal_time()
    ### CProfile method
    # cProfile.run('traj_gen.optimal_time()')
    ### GraphvizOutput (Recommended) : This code shows profile Graphically
    # graphviz = output.GraphvizOutput(output_file='profile.png')
    # with PyCallGraph(output=graphviz):
    #   traj_gen.optimal_time()