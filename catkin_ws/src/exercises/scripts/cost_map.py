#!/usr/bin/env python3
#
# CONCURSO IBEROAMERICANO DE ROBOTICA ESPACIAL, PEU-UNAM, 2022
# ETAPA 04 - NAVEGACION
# MAPA DE COSTO
#
# Instrucciones:
# Escriba el codigo necesario para obtener un mapa de costo
# dado un mapa de celdas de ocupacion y un radio de costo
#

import rospy
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "NOMBRE DEL EQUIPO"

def get_cost_map(static_map, cost_radius):
    if cost_radius > 20:
        cost_radius = 20
    print("Calculating cost map with " +str(cost_radius) + " cells")
    cost_map = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # EJERCICIO:
    # Escriba el codigo necesario para calcular un mapa de costo para el mapa dado.
    # Para calcular el costo, considere como ejemplo el siguiente mapa:
    # [[ 0 0 0 0 0 0]
    #  [ 0 X 0 0 0 0]
    #  [ 0 X X 0 0 0]
    #  [ 0 X X 0 0 0]
    #  [ 0 X 0 0 0 0]
    #  [ 0 0 0 X 0 0]]
    # Donde las celda ocupadas 'X' tienen un valor de 100 y las celdas libres, un valor de 0.
    # El costo es un entero que indica que tan cerca esta un obstaculo de cierta celda.
    # [[ 3 3 3 2 2 1]
    #  [ 3 X 3 3 2 1]
    #  [ 3 X X 3 2 1]
    #  [ 3 X X 3 2 2]
    #  [ 3 X 3 3 3 2]
    #  [ 3 3 3 X 3 2]]
    # El radio de costo indica el numero de celdas alrededor de los obstaculos con costos mayores que cero.
    
    for i in range(height):
        for j in range(width):
            if static_map[i,j] > 50:
                for k1 in range(-cost_radius, cost_radius+1):
                    for k2 in range(-cost_radius, cost_radius+1):
                        cost = cost_radius - max(abs(k1),abs(k2)) + 1
                        cost_map[i+k1,j+k2] = max(cost, cost_map[i+k1,j+k2])
    return cost_map

def callback_cost_map(req):
    global cost_map
    return GetMapResponse(map=cost_map)
    
def main():
    global cost_map, inflated_map
    print("EJERCICIO 2 - MAPA DE COSTO" + NAME)
    rospy.init_node("cost_maps")
    rospy.wait_for_service('/static_map')
    grid_map = rospy.ServiceProxy("/static_map", GetMap)().map
    map_info = grid_map.info
    width, height, res = map_info.width, map_info.height, map_info.resolution
    grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
    rospy.Service('/cost_map'    , GetMap, callback_cost_map)
    loop = rospy.Rate(2)
    
    cost_radius = 0.1
    while not rospy.is_shutdown():
        if rospy.has_param("/path_planning/cost_radius"):
            new_cost_radius = rospy.get_param("/path_planning/cost_radius")
        if new_cost_radius != cost_radius:
            cost_radius   = new_cost_radius
            cost_map_data = get_cost_map(grid_map, int(cost_radius/res))
            cost_map_data = numpy.ravel(numpy.reshape(cost_map_data, (width*height, 1)))
            cost_map      = OccupancyGrid(info=map_info, data=cost_map_data) 
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
