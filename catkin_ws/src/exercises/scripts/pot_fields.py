#!/usr/bin/env python3
#
# CONCURSO IBEROAMERICANO DE ROBOTICA ESPACIAL, PEU-UNAM, 2022
# ETAPA 04 - NAVEGACION
# EVASION DE OBSTACULOS POR CAMPOS POTENCIALES
#
# Instrucciones:
# Complete el codigo necesario para implementar evasion de obstaculos mediante campos potenciales
# usando la tecnica de campos atractivos y repulsivos.
# Sintonice las constantes alfa y beta para obtener un movimiento suave. 
#

import rospy
import tf
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan

NAME = "NOMBRE DE EQUIPO"

listener    = None
pub_cmd_vel = None
pub_markers = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()    
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

def attraction_force(robot_x, robot_y, goal_x, goal_y):
    #
    # EJERCICIO:
    # Calcule la fuerza de atraccion, dada la posicion del robot y la posicion meta.
    # Regrese una tupla de la forma [force_x, force_y]
    # donde force_x y force_y son las componentes X y Y de la fuerza resultante
    # con respecto al mapa. 
    #
    alpha = 2.0  #Attraction constant
    force_x, force_y  = robot_x - goal_x, robot_y - goal_y
    mag = math.sqrt(force_x**2 + force_y**2)
    if mag == 0:
        return [0, 0]
    [force_x, force_y] = [alpha*force_x/mag, alpha*force_y/mag]
    return [force_x, force_y]

def rejection_force(robot_x, robot_y, robot_a, laser_readings):
    #
    # EJERCICIO:
    # Calcule la fuerza total repulsiva dada por el promedio
    # de las fuerzas repulsivas causadas por cada lectura del laser.
    # Las lecturas del laser estan en un arreglo donde cada elemento es una tupla
    # [distance, angle] ambas con respecto al robot.
    # Calcule las fuerzas repulsivas de acuerdo con las diapositivas.
    # Devuelva una tupla de la forma [force_x, force_y]
    # donde force_x y force_y son las componentes  X y Y de la
    # fuerza repulsiva resultante con respecto al mapa. 
    #
    beta = 6.0 #Rejection constant
    d0   = 1.0 #Distance of influence
    force_x, force_y = 0, 0
    for [distance, angle] in laser_readings:
        if distance < d0 and distance > 0:
            mag = beta*math.sqrt(1/distance - 1/d0)
        else:
            mag = 0
        force_x += mag*math.cos(angle + robot_a)
        force_y += mag*math.sin(angle + robot_a)
    if len(laser_readings) == 0:
        return [force_x, force_y]
    [force_x, force_y] = [force_x/len(laser_readings), force_y/len(laser_readings)]
    return [force_x, force_y]

def callback_pot_fields_goal(msg):
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y
    print("Moving to goal point " + str([goal_x, goal_y]) + " by potential fields")
    loop = rospy.Rate(20)
    global laser_readings

    #
    # EJERCICIO:
    # Mueva el robot hacia el punto meta usando campos potenciales.
    # Recuerde que el punto meta es un minimo en el campo potencial, por lo que
    # puede ser alcanzado usando el descenso del gradiente.
    # Se puede utilizar el siguiente pseudocodigo:
    #
    # Obtener la posicion del robot llamando robot_x, robot_y, robot_a = get_robot_pose(listener)
    # Calcule la distancia a la meta como math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    # WHILE distance_to_goal_point > tolerance and not rospy.is_shutdown():
    #     Calcule la fuerza de atraccion llamando [fax, fay] = attraction_force(robot_x, robot_y, goal_x, goal_y)
    #     Calcule la fuerza de repulsion llamando [frx, fry] = rejection_force (robot_x, robot_y, robot_a, laser_readings)
    #     Calcule la fuerza resultante F = Fa + Fr
    #     Calcule la siguiente posicion deseada P = [px, py] = Pr - epsilon*F
    #     Calcule la senales de control llamando msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
    #     Publique las senales de control llamando pub_cmd_vel.publish(msg_cmd_vel)
    #     Llame la funcion draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers) para dibujar los marcadores
    #
    #     Espere unos milisegundos llamando loop.sleep()
    #     Actualice la posicion del robot llamando robot_x, robot_y, robot_a = get_robot_pose(listener)
    #     Recalcule la distancia a la meta
    #  Publique velocidad cero
    #

    robot_x, robot_y, robot_a = get_robot_pose(listener)
    dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    tolerance = 0.1
    epsilon = 0.5
    while dist_to_goal > tolerance and not rospy.is_shutdown():
        afx, afy = attraction_force(robot_x, robot_y, goal_x, goal_y)
        rfx, rfy = rejection_force (robot_x, robot_y, robot_a, laser_readings)
        [fx, fy] = [afx + rfx, afy + rfy]
        [px, py] = [robot_x - epsilon*fx, robot_y - epsilon*fy]
        
        msg_cmd_vel = calculate_control(robot_x, robot_y, robot_a, px, py)
        pub_cmd_vel.publish(msg_cmd_vel)
        draw_force_markers(robot_x, robot_y, afx, afy, rfx, rfy, fx, fy, pub_markers)

        loop.sleep()
        robot_x, robot_y, robot_a = get_robot_pose(listener)
        dist_to_goal = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    pub_cmd_vel.publish(Twist())
    
    print("Goal point reached")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def callback_scan(msg):
    global laser_readings
    laser_readings = [[msg.ranges[i], msg.angle_min+i*msg.angle_increment] for i in range(len(msg.ranges))]

def draw_force_markers(robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):
    pub_markers.publish(get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))
    pub_markers.publish(get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))
    pub_markers.publish(get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))

def get_force_marker(robot_x, robot_y, force_x, force_y, color, id):
    hdr = Header(frame_id="map", stamp=rospy.Time.now())
    mrk = Marker(header=hdr, ns="pot_fields", id=id, type=Marker.ARROW, action=Marker.ADD)
    mrk.pose.orientation.w = 1
    mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
    mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
    mrk.points.append(Point(x=robot_x, y=robot_y))
    mrk.points.append(Point(x=(robot_x - force_x), y=(robot_y - force_y)))
    return mrk

def main():
    global listener, pub_cmd_vel, pub_markers
    print("EJERCICIO 5 - CAMPOS POTENCIALES - " + NAME)
    rospy.init_node("pot_fields")
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_scan)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_pot_fields_goal)
    pub_cmd_vel = rospy.Publisher('/hsrb/command_velocity', Twist,  queue_size=10)
    pub_markers = rospy.Publisher('/navigation/pot_field_markers', Marker, queue_size=10)
    listener = tf.TransformListener()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
