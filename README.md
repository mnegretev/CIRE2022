# Concurso Iberoamericano de Robótica Espacial 2022
Programa Espacial Universitario - UNAM


## Requerimientos

* Ubuntu 20.04
* ROS Noetic http://wiki.ros.org/noetic/Installation/Ubuntu

## Máquina Virtual

Se puede descargar una máquina virtual para [VirtualBox](https://www.virtualbox.org/wiki/Downloads) con todo lo necesario ya instalado de [esta dirección.](https://drive.google.com/drive/folders/1DYhmegVFEz7VA69uncpYsL8Ck0HbaIEz?usp=sharing) <br>
En esa misma carpeta hay un video con instrucciones para usar la máquina virtual. <br>
Se recomienda configurar la máquina virtual con 4 CPUs y 4GB de RAM.
Usuario: cire2022 <br>
Contraseña: cire2022

## Prueba

Abra una terminal y ejecute los siguientes comandos:

* $ cd CIRE2022
* $ git pull
* $ cd catkin_ws
* $ catkin_make
* $ source devel/setup.bash
* $ roslaunch bring_up stage02.launch

Se debería ver un RViz como el siguiente:

<img src="https://github.com/mnegretev/CIRE2022/blob/master/Media/rviz.png" alt="RViz" width="639"/>

Y un Gazebo como el siguiente:

<img src="https://github.com/mnegretev/CIRE2022/blob/master/Media/gazebo.png" alt="Gazebo" width="631"/>

## Instalación nativa

Para utilizar una instalación nativa, contactar a los organizadores. 

## Contacto
Dr. Marco Negrete<br>
Profesor Asociado C<br>
Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
[mnegretev.info](http://mnegretev.info)<br>
marco.negrete@ingenieria.unam.edu<br>
