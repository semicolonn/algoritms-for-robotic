#!/usr/bin/env python

import rospy
import numpy as np
import math

from math import atan2, cos, sin, radians
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


#Contantes de ganho
Kp_x = 1
Kp_theta = 1

#Constantes de atracao e repulsao
Katt = 10
Krep = 3
p_0 = 1

#Variavel pra receber os dados do laser do robo
laserMsg = None

#Variaveis pra receber a posicao do robo
x = 0.0
y = 0.0
theta = 0.0

def LaserCallback(msg):
    global laserMsg
    laserMsg = msg

def OdomCallback(msg):
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    #Calculando o valor de theta(yall) usando quaternion
    quaterion_rot = msg.pose.pose.orientation
    euler_rot = euler_from_quaternion([quaterion_rot.x, quaterion_rot.y, quaterion_rot.z, quaterion_rot.w])
    theta = euler_rot[2] #euler_rot = [roll, pitch, yall]
    

def min(vector):
    m = vector[0]
    pos = 0
    for i in range (len(vector)):
        if(m > vector[i]):
            m = vector[i]
            pos = i
    return m, pos



#Calculo da forca de atracao (Utilizando potencial quadratico)
def Att_Force(Err_x, Err_y):
    #Calcula a posicao do goal relativa ao referencial do robo
    dist = math.sqrt(Err_x**2 + Err_y**2)
    theta_vec = atan2(Err_y, Err_x)
    K = Katt * dist
    force_x = K*cos(theta_vec)
    force_y = K*sin(theta_vec)
    return force_x, force_y
 
#Calculo da forca de repulsao
def Rep_Force():
    force_x = 0
    force_y = 0
    #Para cada obstaculo dentro do range do sensor
    for i in range (len(laserMsg.ranges)):
        theta_vec = (laserMsg.angle_increment * i)
        m = laserMsg.ranges[i]
        
        if m < p_0:
            force_x += Krep * ((1/m) - (1/p_0)) * (1/(m*m))*(cos(theta_vec + theta)/m)
            force_y += Krep * ((1/m) - (1/p_0)) * (1/(m*m))*(sin(theta_vec + theta)/m)
    return force_x, force_y


F_res = np.zeros(2)
def run():
    rospy.init_node('robots_roboticamovel_2018_2', anonymous = True)
    pub1 =  rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    sub1 =  rospy.Subscriber("base_scan", LaserScan, LaserCallback)
    odom1 = rospy.Subscriber("odom", Odometry, OdomCallback)

    rate =  rospy.Rate(5) #5 hz
    cmd_vel = Twist()
    
    global laserMsg
    # goal_input = False

    goal_x = input("Digite a coordenada x do objetivo: ")
    goal_y = input("Digite a coordenada y do objetivo: ")
    while not rospy.is_shutdown():
        if laserMsg == None:
                rate.sleep()
                continue

        #getErro
        Err_x = goal_x - x
        Err_y = goal_y - y

        #getDist to goal
        dist = math.sqrt(Err_x**2 + Err_y**2)
       
        #getAttForce
        Att_x, Att_y = Att_Force(Err_x, Err_y)
        #getRepForce
        Rep_x, Rep_y = Rep_Force()

        F_res[0] = Att_x + Rep_x
        F_res[1] = Att_y + Rep_y

        angle = atan2(F_res[1], F_res[0])

        cmd_vel.linear.x = dist * cos(theta) * F_res[0] + sin(theta) * F_res[1]
        cmd_vel.angular.z = ((-sin(theta)/10) * F_res[0]) + ((cos(theta)/10) * F_res[1])

        pub1.publish(cmd_vel)
        rate.sleep()
      
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass