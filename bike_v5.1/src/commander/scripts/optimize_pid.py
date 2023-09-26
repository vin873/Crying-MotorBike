#!/usr/bin/env python3
# [-3.985429523626329, 6.469176706483993, 6.770045134396865, -0.6227348917037592, 10.783451975738025, -3.6199357492280937], (4.717182151794434,)
import rospy, time
from deap import base, creator, tools
import random
import datetime
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
import math
state = Pose()
y_angle = 0
y_angular=0
y_angle_imu=0 
y_angular_imu=0
motorPos=0

pub_back = rospy.Publisher('/back_fork_effort_controller/command', Float64, queue_size = 10)
# pub_front = rospy.Publisher('/Revolute_5_position_controller/command', Float64, queue_size = 10)

creator.create("FitnessMin", base.Fitness, weights=(1.0,))  
creator.create("Individual", list, fitness=creator.FitnessMin)

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    return roll_x # in radians

def get_joint(data):
    global motorPos
    motorPos=data.position[1]

def get_pose(data):
    global y_angle, state,y_angular
    ind_pitch = data.name.index('new_wheel::base_link')
    state = data.pose[ind_pitch]
    y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
    state = data.twist[ind_pitch]
    y_angular = state.angular.x

def imucallback(msg):
    global y_angle, y_angular, y_angle_imu, y_angular_imu
    y_angle_imu = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    y_angular_imu=msg.angular_velocity.z

def constrain(variable,range0,range1):
    if variable<range0:
        variable=range0
    if variable>range1:
        variable=range1
    return variable

def gain_evaluation(individual):
    global y_angle, pub_back, y_angular, y_angle_imu, y_angular_imu
    # Kp_p = abs(individual[0])
    Kp_p = (individual[0])
    # Ki_p = (individual[1])/100
    # Kd_p = abs(individual[1])/50
    Kd_p = (individual[1])/10
    # Kp_e = (individual[0])/1.5
    # Ki_e = (individual[1])/700
    # Kd_e = (individual[2])/5
    # Kp_p = 8.85221716827368
    # Ki_p = 0
    # Kd_p = 0.673872972028931
    # Kp_e = 10
    # Ki_e = 0.6
    # Kd_e = 0.7

    time_interval = 0.001

    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation_client = rospy.ServiceProxy('/gazebo/reset_simulation',Empty) #rosservice call gazebo/pause_physics
    reset_simulation_client()
    y_angle = 0
    y_angular=0
    yaw_displacement_sum = 0
    yaw_int=0
    lastMotorError=0
    motorError=0
    motorErrorI=0
    motorErrorD=0
    targetPos=0
    motorPos=0
    for i in range(6000):
        time1 = time.time()
        # pub_back.publish(pid(y_angle))
        pub_back.publish(-Kp_p*y_angle - Kd_p*y_angular)
        # targetPos=-(Kp_p*y_angle+Ki_p*yaw_int+Kd_p*y_angular)
        # targetPos=-(Kp_p*y_angle+Kd_p*y_angular)
        motorError=targetPos-motorPos
        motorErrorI+=motorError*0.01
        motorErrorI=constrain(motorErrorI,-3.5,3.5)
        motorErrorD=(motorError-lastMotorError)/time_interval
        lastMotorError=motorError
        # pub_back.publish((Kp_e*motorError+Ki_e*motorErrorI+Kd_e*motorErrorD))
        # pub_back.publish((Kp_e*motorError+Ki_e*motorErrorI))
        # pub_back.publish((Kp_e*motorError+Kd_e*motorErrorD))
        yaw_displacement_sum+=(abs(y_angle)+abs(motorPos)*30)
        yaw_int+=y_angle*0.01
        yaw_int=constrain(yaw_int,-0.3,0.3)
        # print(yaw_int)
        time2 = time.time()
        interval = time2 - time1
        if abs(y_angle)>0.3:
            # # integral_yaw_error = 0
            print("time:",i*time_interval+interval,f" individual : {individual}")
            # print("fitness:",yaw_displacement_sum,f" individual : {individual}")
            return 0,
            break
    
        if(interval < time_interval):
            time.sleep(time_interval - interval)
    
    # integral_yaw_error = 0
    rospy.logwarn("==========================================")
    print("fitness:",yaw_displacement_sum,f" individual : {individual}")
    # print("fitness:",i*time_interval+interval,f" individual : {individual}")
    rospy.logwarn("==========================================")
    # return i*time_interval+interval,
    return 9999-yaw_displacement_sum,

if __name__ == '__main__':
    rospy.init_node('pid_gain_optimizer', anonymous=True)
    rospy.Subscriber("/gazebo/link_states", LinkStates, get_pose)
    rospy.Subscriber("/imu", Imu, imucallback)
    rospy.Subscriber("/joint_states", JointState, get_joint)

    print("#####genetic optimization#####")
    toolbox = base.Toolbox()
    toolbox.register("attr_gene", random.uniform , 0, 20)
    # toolbox.register("attr_gene", random.gauss , 0, 2)
    toolbox.register("evaluate", gain_evaluation)
    toolbox.register("mate", tools.cxBlend, alpha=0.2)
    # toolbox.register("mate", tools.cxTwoPoint)
    # toolbox.register("mate", tools.cxPartialyMatched)
    # toolbox.register("select", tools.selTournament, tournsize=7)
    # toolbox.register("select", tools.selRoulette)
    toolbox.register("select", tools.selStochasticUniversalSampling)

    # GA parameters
    N_GEN = 30        # number of generations
    POP_SIZE =  50  # number of individuals
    CX_PB = 0.5      # crossover probability
    MUT_PB = 0.3     # mutation probability

    random.seed(datetime.datetime.now())

    points_number = 2
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.1, indpb=0.2)
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
    
    # 20+ 0.56+/50
    # fitness: 11.031062207654116  individual : [-1.7895137530379641, 2.2596831862001747]
    # fitness: 10.772827886558792  individual : [1.9159329979803985, 1.6983123415545232]
    # fitness: 11.880928326612006  individual : [16.46130762241969, 13.373700891845171]
    # fitness: 7.748337194569291  individual : [19.85221716827368, 6.73872972028931]