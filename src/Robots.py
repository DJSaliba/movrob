import rospy
from math import sqrt, sin, cos
from tf.transformations import euler_from_quaternion


#TODO: Convert example into class for reuse
class DifferentialRobot():
    def callback_pose (self,data):
        self.x = data.pose.pose.position.x # posicao x do robo no mundo
        self.y = data.pose.pose.position.y # posicao y do robo no mundo
        quat = data.pose.pose.orientation
        _,_,self.theta = euler_from_quaternion(quat)

    # Rotina para a geracao da trajetoria de referencia
    def refference_trajectory (time):
        # #MUDAR PARA CIRCULO
        global x_goal , y_goal
        x_ref = x_goal
        y_ref = y_goal
        Vx_ref = 0
        Vy_ref = 0
        return (x_ref , y_ref , Vx_ref , Vy_ref )

    # Rotina para a geracao da entrada de controle
    def trajectory_controller (x_ref , y_ref , Vx_ref , Vy_ref ):
        global x_n , y_n , theta_n
        global Kp
        global Usat
        Ux = Vx_ref + Kp * (x_ref - x_n)
        Uy = Vy_ref + Kp * (y_ref - y_n)
        absU = sqrt(Ux ** 2 + Uy ** 2)
        if (absU > Usat):
            Ux = Usat * Ux / absU
            Uy = Usat * Uy / absU
        return (Ux , Uy)

    #Rotina feedback linearization
    def feedback_linearization (Ux , Uy):
        global x_n , y_n , theta_n
        global d
        VX = cos( theta_n ) * Ux + sin( theta_n ) * Uy
        WZ = (-sin( theta_n ) / d) * Ux + (cos( theta_n ) / d) * Uy
        return (VX , WZ)
