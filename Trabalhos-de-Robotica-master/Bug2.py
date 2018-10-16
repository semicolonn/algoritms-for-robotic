#!/usr/bin/env python

import rospy
import numpy as np
import math
from math import atan2, cos, sin
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

#Coordenadas do goal
goal_x = 5.0
goal_y = 5.0


#Contantes de ganho
Kp_x = 1
Kp_y = 1
Kp_theta = 1

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

#Se a distancia desse ponto ao goal for menor que a distancia do hit point ao goal entao esse ponto eh leave point
def LeavePoint(x,y,hitp_x, hitp_y):
    if(math.sqrt((goal_x - hitp_x)**2) + math.sqrt((goal_y - hitp_y)**2) > math.sqrt((goal_x - x)**2) + math.sqrt((goal_y - y)**2)):
        return True
    else:
        return False
def min(vector):
    m = vector[0]
    pos = 0
    for i in range (len(vector)):
        if(m > vector[i]):
            m = vector[i]
            pos = i
    return m, pos
#Matrizes do controlador
G = np.array([[Kp_x,0,0],[0,Kp_y,0],[0,0,Kp_theta]])
Err = np.zeros(3)
V = np.zeros(3)

def run():
    rospy.init_node('robots_roboticamovel_2018_2', anonymous = True)
    pub1 =  rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    sub1 =  rospy.Subscriber("base_scan", LaserScan, LaserCallback)
    odom1 = rospy.Subscriber("odom", Odometry, OdomCallback)
    
    rate =  rospy.Rate(5) #5 hz
    cmd_vel = Twist()
    
    global laserMsg

    #Estados do robo
    Follow_wall = False
    Follow_line = True

    a = 0
    b = 0
    while not rospy.is_shutdown():
        if laserMsg == None:
                rate.sleep()
                continue
        if a == 0 and b == 0:
            goal_x = input("Digite a coordenada x do objetivo: ")
            goal_y = input("Digite a coordenada y do objetivo: ")
            #Reta = ax + b = y
            a = (goal_y - y)/(goal_x - x)
            b = goal_y - (a * goal_x) 
        if(Follow_line == True):

            #Se achar um obstaculo grava o hit point
            m, pos = min(laserMsg.ranges[150:210])
            if (m< 1.0):
                hitp_x = x
                hitp_y = y
                Follow_wall = True
                Follow_line = False

            Err[0] = goal_x - x
            Err[1] = goal_y - y

            #Calcula a posicao do goal relativa ao referencial do robo
            dist = math.sqrt(Err[0]**2 + Err[1]**2)
            theta_vec = atan2(Err[1], Err[0])
            p_relative = [dist * cos(theta_vec - theta), dist * sin(theta_vec - theta)]

            Err[0] = p_relative[0]
            Err[1] = p_relative[1]
            Err[2] = theta_vec - theta

            V = G.dot(Err)

            if dist > 0.1:
                cmd_vel.angular.z = V[2]

            #Primeiro ele fica de frente para o goal depois ele segue a reta
            if(abs(cmd_vel.angular.z) < 0.01):

                cmd_vel.linear.x = V[0]
                cmd_vel.linear.y = V[1]

        if (Follow_wall == True):
            m, pos = min(laserMsg.ranges[150:210])
            if (m < 1.0):
                
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = 3.0
            
            elif laserMsg.ranges[len(laserMsg.ranges)/8] <= 0.95: #Se tiver um objeto a direita dele
                cmd_vel.linear.x = 0.65#segue reto
                cmd_vel.angular.z = 0.0
            elif laserMsg.ranges[len(laserMsg.ranges)/8] > 0.95:
                cmd_vel.linear.x = 0.1
                cmd_vel.angular.z = -1.0 #vira pra direita

            #Verifica se o ponto que o robo esta pertence a reta
            if(abs(((a * x) + b ) - y) <= 0.3):

                #Verifica se o ponto eh leave point
                if(LeavePoint(x,y,hitp_x,hitp_y) == True):
                    cmd_vel.linear.x = 0.0
                    cmd_vel.linear.y = 0.0
                    Follow_line = True
                    Follow_wall = False
                
     
        pub1.publish(cmd_vel)
        rate.sleep()
      
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass