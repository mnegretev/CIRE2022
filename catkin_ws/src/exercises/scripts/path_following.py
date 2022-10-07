#!/usr/bin/env python3
#
# CONCURSO IBEROAMERICANO DE ROBOTICA ESPACIAL, PEU-UNAM, 2022
# ETAPA 04 - NAVEGACION
# SEGUIMIENTO DE RUTAS
#
# Instrucciones:
# Escriba el codigo necesario para mover al robot a lo largo de una ruta dada.
# Considere una base difenrencia. Las velocides lineal y angular maximas
# deben ser 0.3 y 0.5 respectivamente.
#

import rospy
import tf
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from custom_msgs.srv import SmoothPath, SmoothPathRequest
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

NAME = "NOMBRE DEL EQUIPO"

pub_goal_reached = None
pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    
    #
    # EJERCICIO:
    # Implemente las leyes de control vistas en las diapositivas:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # donde error_a es el error de angulo, v y w son las velocides lineal y angular,
    # y v_max, w_max, alpha and beta, son constantes de sintonizacion.
    # Almacene las variables v y w resultantes en el mensaje de tipo Twist cmd_vel
    # y devuelva esa variable. Revise la documentacion en linea del mensaje Twist.
    # Recuerde mantener el error de angulo en (-pi,pi]
    #
    
    v_max = 0.3
    w_max = 0.5
    alpha = 1.0#0.2
    beta  = 0.1#0.5
    [error_x, error_y] = [goal_x - robot_x, goal_y - robot_y]
    error_a = (math.atan2(error_y, error_x) - robot_a)%(2*math.pi)
    error_d = math.sqrt(error_x**2 + error_y**2)
    if error_a  >  math.pi:
        error_a -= 2*math.pi
    cmd_vel.linear.x  = min(v_max, error_d)*math.exp(-error_a*error_a/alpha)
    cmd_vel.angular.z = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    return cmd_vel

def follow_path(path):
    #
    # EJERCICIO:
    # Use la funcion calculate_control para mover al robot a lo largo de la ruta
    # dada como una secuencia de puntos [[x0,y0], [x1,y1], ..., [xn,yn]]
    # El publicador para el mensaje Twist ya esta declarado como 'pub_cmd_vel'
    # Puede usar los siguientes pasos para realizar el seguimiento:
    #
    # Asigne como meta local el primer punto de la ruta
    # Asigne como meta global el ultimo punto de la ruta
    # Obtenga la posicion del robot con [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    # Calcule el error global (distancia del robot a la meta global)
    # Calcule el error local (distancia del robot a la meta local)
    #
    # WHILE error global > tol and not rospy.is_shutdown() #Mantiene al programa escuchando senales como Ctrl+C
    #     Calcule las senales de control v y w y publiquelas en el mensaje respectivo
    #     loop.sleep() # Esto es importante para evitar sobre consumo de procesador
    #     Obtenga la posicion del robot
    #     Calcule el error local
    #     Si error local < 0.3 (puede cambiar este valor)
    #         Cambie la meta local al siguiente punto de la ruta
    #     Calcule error global
    # Enviar velocidad cero
    # Publique un 'True' usando el publicador pub_goal_reached
    #
    current_point = 0
    [local_xg,  local_yg ] = path[current_point]
    [global_xg, global_yg] = path[len(path)-1]
    [robot_x, robot_y, robot_a]    = get_robot_pose(listener)
    global_error = math.sqrt((global_xg-robot_x)**2 + (global_yg-robot_y)**2)
    local_error  = math.sqrt((local_xg-robot_x) **2 + (local_yg-robot_y) **2)
    
    while not rospy.is_shutdown() and global_error > 0.1:
        pub_cmd_vel.publish(calculate_control(robot_x, robot_y, robot_a, local_xg, local_yg))
        loop.sleep()
        [robot_x, robot_y, robot_a] = get_robot_pose(listener)
        local_error  = math.sqrt((local_xg-robot_x) **2 + (local_yg-robot_y) **2)
        current_point = min(current_point+1, len(path)-1) if local_error < 0.3 else current_point
        [local_xg,  local_yg ] = path[current_point]
        global_error = math.sqrt((global_xg-robot_x)**2 + (global_yg-robot_y)**2)
    pub_cmd_vel.publish(Twist())
    pub_goal_reached.publish(True)
    
def callback_global_goal(msg):
    print("Calculating path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y]))
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest(goal=PoseStamped(pose=msg.pose))
    req.start.pose.position = Point(x=robot_x, y=robot_y)
    path = rospy.ServiceProxy('/path_planning/a_star_search', GetPlan)(req).plan
    path = rospy.ServiceProxy('/path_planning/smooth_path',SmoothPath)(SmoothPathRequest(path=path)).smooth_path
    print("Following path with " + str(len(path.poses)) + " points...")
    follow_path([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    print("Global goal point reached")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def main():
    global pub_cmd_vel, pub_goal_reached, loop, listener
    print("EJERCICIO 4 - SEGUIMIENTO DE RUTAS -" + NAME)
    rospy.init_node("path_following")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    pub_goal_reached = rospy.Publisher('/navigation/goal_reached', Bool, queue_size=10)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for service for path planning...")
    rospy.wait_for_service('/path_planning/a_star_search')
    print("Service for path planning is now available.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
