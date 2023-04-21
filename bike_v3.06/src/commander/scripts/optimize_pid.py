#!/usr/bin/env python3
import rospy, time
from deap import base, creator, tools
import random
import datetime
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from std_srvs.srv import Empty
import math
from simple_pid import PID
state = Pose()
y_angle = 0
y_angular=0

pub_back = rospy.Publisher('/back_fork_position_controller/command', Float64, queue_size = 10)
# pub_front = rospy.Publisher('/Revolute_5_position_controller/command', Float64, queue_size = 10)

creator.create("FitnessMin", base.Fitness, weights=(1.0,))  
creator.create("Individual", list, fitness=creator.FitnessMin)

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    return roll_x # in radians

def get_pose(data):
    global y_angle, state,y_angular
    ind_pitch = data.name.index('motor_0419::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
    state = data.twist[ind_pitch]
    y_angular = state.angular.x

def gain_evaluation(individual):
    global y_angle, pub_back, y_angular

    Kp_p = individual[0]
    Kd_p = individual[1]

    time_interval = 0.001

    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation_client = rospy.ServiceProxy('/gazebo/reset_simulation',Empty) #rosservice call gazebo/pause_physics
    reset_simulation_client()
    y_angle = 0
    y_angular=0
    # max_angle=9999
    # target_yaw_angle = 0
    # last_error_pos = 0
    # integral_yaw_error = 0
    # yaw_displacement_sum = 0

    for i in range(5000):
        time1 = time.time()

        effort_pos = (Kp_p*y_angle + Kd_p*y_angular)
        pub_back.publish(effort_pos)

        time2 = time.time()
        interval = time2 - time1
        if abs(y_angle)>0.4:
            # integral_yaw_error = 0
            # return 9999,
            break
    
        if(interval < time_interval):
            time.sleep(time_interval - interval)
    
    # integral_yaw_error = 0
    print("fitness:",i*time_interval+interval,f" individual : {individual}")
    return i*time_interval+interval,
    # return yaw_displacement_sum,

if __name__ == '__main__':
    rospy.init_node('pid_gain_optimizer', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_pose)

    print("#####genetic optimization#####")
    toolbox = base.Toolbox()
    toolbox.register("attr_gene", random.uniform , -15, 15)
    # toolbox.register("attr_gene", random.gauss , 0, 0.2)
    toolbox.register("evaluate", gain_evaluation)
    toolbox.register("mate", tools.cxBlend, alpha=0.2)
    # toolbox.register("mate", tools.cxTwoPoint)
    # toolbox.register("mate", tools.cxPartialyMatched)
    # toolbox.register("select", tools.selTournament, tournsize=7)
    # toolbox.register("select", tools.selRoulette)
    toolbox.register("select", tools.selStochasticUniversalSampling)

    # GA parameters
    N_GEN = 30        # number of generations
    POP_SIZE = 400   # number of individuals
    CX_PB = 0.5      # crossover probability
    MUT_PB = 0.35     # mutation probability

    random.seed(datetime.datetime.now())

    points_number = 2
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=2, indpb=0.2)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_gene, points_number) #toolbox.attr_gene
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    # creating population
    pop = toolbox.population(n=POP_SIZE)

    # evaluating each individual
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit
    print("  Evaluated %i individuals" % len(pop))

    # fitness of each individual
    fits = [ind.fitness.values[0] for ind in pop]

    g = 0
    while g < N_GEN:
        g = g + 1
        print("-- Generation %i --" % g)

        offspring = toolbox.select(pop, len(pop))
        offspring = list(map(toolbox.clone, offspring))

        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CX_PB:
                toolbox.mate(child1, child2)

                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUT_PB:
                toolbox.mutate(mutant)

                del mutant.fitness.values

        invalid_ind = []
        for ind in offspring:
            if not ind.fitness.valid:
                invalid_ind.append(ind)

        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        pop[:] = offspring
        
        fits0 = [ind.fitness.values[0] for ind in pop]

        length = len(pop)
        mean = sum(fits0) / length
        sum2 = sum(x * x for x in fits0)
        std = abs(sum2 / length - mean**2)**0.5

        print("  Min %s" % min(fits0))
        print("  Max %s" % max(fits0))
        print("  Avg %s" % mean)
        print("  Std %s" % std)

    # choosing the best individual
    best_ind = tools.selBest(pop, 1)[0]
    print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))