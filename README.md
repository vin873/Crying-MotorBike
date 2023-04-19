# Crying-MotorBike

Please download this!!

    sudo apt-get install ros*controller

## 2023/01/16
1.成功下載Fusion 360

## 2023/01/17 
1.Fusion 360 to URDF

2.重灌ubuntu + roscore

3.import URDF to gazebo

4.更改transmittion、yaml

## 2023/02/11 

#### 1. gazebo launch:

    cd crying_MOTOROiD/
    source devel/setup.bash
    roslaunch Perfect_Wheel_description gazebo.launch
    
#### 2. controller launch:

    cd crying_MOTOROiD/
    source devel/setup.bash
    roslaunch Perfect_Wheel_description controller.launch
    
#### 3. imu:

    rostopic echo /imu
    
or
    
    cd crying_MOTOROiD/src/wheel/urdf
    python3 imu.py
    
#### 4. pid:
https://wiki.ros.org/rqt_reconfigure

    rosrun rqt_reconfigure rqt_reconfigure
    
#### 5. back wheel speed:
https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros

    rostopic pub -1 /back_wheel_joint_position_controller/command std_msgs/Float64 "data: 1"
    
## 2023/02/15 

#### 1. 前叉角度傾斜20

#### 2. 車輪圓角加大

#### 3. 加配重

#### 4. 更改材質，輸入轉動慣量

*問題：轉動慣量的計算原點和軸*


## 2023/02/28

#### 1. 更新轉動慣量

## 2023/03/15

#### 1. 前叉、後叉、body的材料是鋁
#### 2. 配重塊是stainless steel
#### 3. 輪子是rubber
#### 4. 本日最終版本：bibi(圖檔) bi(launch)

https://answers.gazebosim.org/question/27178/looking-for-a-stable-solution-to-reset-a-robot-with-controller-in-gazebo/


    #!/usr/bin/env python3
    import rospy, time
    from deap import base, creator, tools
    import random
    import datetime
    from std_msgs.msg import Float64
    from gazebo_msgs.msg import LinkStates
    from geometry_msgs.msg import Pose, Twist
    from sensor_msgs.msg import Imu
    from std_srvs.srv import Empty
    import math
    state = Pose()
    y_angle = 0
    y_angular=0

    pub_back = rospy.Publisher('/Revolute_2_position_controller/command', Float64, queue_size = 10)

    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))  
    creator.create("Individual", list, fitness=creator.FitnessMin)

    def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        return roll_x # in radians

    # def imucallback(msg):
    #     global y_angle, y_angular
    #     y_angle = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    #     y_angular = msg.angular_velocity.x

    def get_pose(data):
        global y_angle, state, y_angular
        ind_pitch = data.name.index('Wheel::base_link')
        state = data.pose[ind_pitch]
        y_angle = euler_from_quaternion(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w)
        state = data.twist[ind_pitch]
        y_angular = state.angular.x

    def gain_evaluation(individual):
        global y_angle, pub_back, y_angular, yaw_displacement_sum
        y_angle=0
        y_angular=0
        print(f"individual : {individual}")
        yaw_displacement_sum=0
        Kp_p = individual[0]
        # Ki_p = individual[1]/100
        Kd_p = (individual[1])/100
        time_interval = 0.001

        rospy.wait_for_service('/gazebo/reset_simulation')
        reset_simulation_client = rospy.ServiceProxy('/gazebo/reset_simulation',Empty) #rosservice call gazebo/pause_physics
        reset_simulation_client()

        for i in range(5000):
            time1 = time.time()
            effort_pos = -(Kp_p*y_angle + Kd_p*y_angular)
            pub_back.publish(effort_pos)
            time2 = time.time()
            interval = time2 - time1
            yaw_displacement_sum+=abs(y_angle)
                # return i*time_interval+interval,
            if(interval < time_interval):
                time.sleep(time_interval - interval)

            if abs(y_angle)>0.6:
            # return 9999,
                return yaw_displacement_sum/(i*time_interval+interval)*100,
        # integral_yaw_error2 = 0

        # print(f"yaw_sum : {yaw_displacement_sum}\n")  

        # return abs(max_angle),
        return yaw_displacement_sum,
        # return i*time_interval+interval,

    if __name__ == '__main__':
        rospy.init_node('pid_gain_optimizer', anonymous=True)
        rospy.Subscriber("/gazebo/link_states", LinkStates, get_pose)
        # rospy.Subscriber("/imu", Imu, imucallback)

        print("#####genetic optimization#####")
        toolbox = base.Toolbox()
        toolbox.register("attr_gene", random.uniform , -10, 10)
        # toolbox.register("attr_gene", random.gauss , 0, 0.2)
        toolbox.register("evaluate", gain_evaluation)
        # toolbox.register("mate", tools.cxBlend, alpha=0.2)
        toolbox.register("mate", tools.cxTwoPoint)
        # toolbox.register("mate", tools.cxPartialyMatched)
        # toolbox.register("select", tools.selTournament, tournsize=7)
        # toolbox.register("select", tools.selRoulette)
        toolbox.register("select", tools.selStochasticUniversalSampling)

        # GA parameters
        N_GEN = 15        # number of generations
        POP_SIZE = 50   # number of individuals
        CX_PB = 0.5      # crossover probability
        MUT_PB = 0.2     # mutation probability

        random.seed(datetime.datetime.now())

        points_number = 2
        toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
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
