#!/usr/bin/env python3
#
# CONCURSO IBEROAMERICANO DE ROBOTICA ESPACIAL, PEU-UNAM, 2022
# ETAPA 04 - NAVEGACION
# INFLADO DE OBSTACULOS
#
# Instrucciones:
# Complete el código necesario para inflar los obstaculos dado una mapa de celdas de ocupacion
# y un numero de celdas a inflar. 
#

import rospy
import numpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest

NAME = "NOMBRE DEL EQUIPO"

def get_inflated_map(static_map, inflation_cells):
    print("Inflating map by " + str(inflation_cells) + " cells")
    inflated = numpy.copy(static_map)
    [height, width] = static_map.shape
    #
    # EJERCICIO:
    # Escriba el codigo necesario para inflar los obstaculos del mapa un radio
    # dado por 'inflation_cells' (expresado en numero de celdas)
    # El mapa esta dado en 'static_map' como un arreglo bidimensional de numpy
    # Considere como celdas ocupadas todas las celdas con un valor de ocupacion mayor que 50
    #
    for i in range(height):
        for j in range(width):
            if static_map[i,j] > 50:
                for k1 in range(-inflation_cells, inflation_cells+1):
                    for k2 in range(-inflation_cells, inflation_cells+1):
                        inflated[i+k1, j+k2] = 100
                         
    return inflated

def callback_inflated_map(req):
    global inflated_map
    return GetMapResponse(map=inflated_map)

def main():
    global cost_map, inflated_map
    print("EJERCICIO 1 - INFLADO DE OBSTACULOS" + NAME)
    rospy.init_node("map_inflation")
    rospy.wait_for_service('/static_map')
    pub_map  = rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
    grid_map = rospy.ServiceProxy("/static_map", GetMap)().map
    map_info = grid_map.info
    width, height, res = map_info.width, map_info.height, map_info.resolution
    grid_map = numpy.reshape(numpy.asarray(grid_map.data, dtype='int'), (height, width))
    rospy.Service('/inflated_map', GetMap, callback_inflated_map)
    loop = rospy.Rate(2)
    
    inflation_radius = 0.1
    while not rospy.is_shutdown():
        if rospy.has_param("/path_planning/inflation_radius"):
            new_inflation_radius = rospy.get_param("/path_planning/inflation_radius")
        if new_inflation_radius != inflation_radius:
            inflation_radius  = new_inflation_radius
            inflated_map_data = get_inflated_map(grid_map, int(inflation_radius/res))
            inflated_map_data = numpy.ravel(numpy.reshape(inflated_map_data, (width*height, 1)))
            inflated_map      = OccupancyGrid(info=map_info, data=inflated_map_data)
        pub_map.publish(callback_inflated_map(GetMapRequest()).map)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
