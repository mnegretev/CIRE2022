#!/usr/bin/env python3
# license removed for brevity
import os
import rospy
import numpy as np
from gazebo_ros import gazebo_interface 
from geometry_msgs.msg import PoseStamped,Pose, Quaternion
import tf
def spawn_object(gazebo_name, name, x, y, z, yaw,roll=0.0 , pitch=0.0):
    global _path_xml, _path_model
    #_path_xml = '/home/cire2022/.gazebo/models/MODEL_NAME/model-1_4.sdf'
    #_path_model = '/home/cire2022/.gazebo/models'
    _path_xml = '/home/cire2022/CIRE2022/models/MODEL_NAME/model-1_4.sdf'
    _path_model = '/home/cire2022/CIRE2022/models'
    
    rospy.loginfo('Spawn: {0}'.format(name))
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    
    
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    initial_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    rospy.loginfo('Spawn: {0}'.format(q))

    path_xml = _path_xml.replace('MODEL_NAME', name)

    with open(path_xml, "r") as f:
        model_xml = f.read()

    model_xml = model_xml.replace('PATH_TO_MODEL', _path_model)

    gazebo_interface.spawn_sdf_model_client(gazebo_name, model_xml, rospy.get_namespace(),
                                            initial_pose, "", "/gazebo")

def spawner():
    rand_seed=np.random.randint(50)
    

    np.random.seed(rand_seed)
    _path_model = "/home/cire2022/CIRE2022/models"     
    #_path_model = "/home/cire2022/.gazebo/models"     
    objs=os.listdir(_path_model)
    objs.sort()
    
    

    num_objs=2
    x_gaz,y_gaz= 0,1.21
    obj_ix=-1


    for i in range(num_objs):
        eu_i,eu_j,eu_k = np.random.rand(3)*np.pi
        sp_x,sp_y,sp_z=x_gaz+0.3*np.random.randn(),  y_gaz+0.3*np.random.randn(),  0.71+0.1*np.random.randn()
        
        spawn_object('spawned_coords_A'+str(i),objs[obj_ix], sp_x,sp_y,sp_z,eu_i,eu_j,eu_k )
     



    num_objs=2
    x_gaz,y_gaz= -3.0,4.0
    obj_ix=0


    for i in range(num_objs):
        eu_i,eu_j,eu_k = np.random.rand(3)*np.pi
        sp_x,sp_y,sp_z=x_gaz+0.3*np.random.randn(),  y_gaz+0.3*np.random.randn(),  0.05
        
        spawn_object('spawned_coords_B'+str(i),objs[obj_ix], sp_x,sp_y,  0.05,eu_i,eu_j,eu_k )



    num_objs=2
    x_gaz,y_gaz= 3.9,5.6
    obj_ix=1


    for i in range(num_objs):
        eu_i,eu_j,eu_k = np.random.rand(3)*np.pi
        sp_x,sp_y,sp_z=x_gaz+0.3*np.random.randn(),  y_gaz+0.3*np.random.randn(),  0.05
        
        spawn_object('spawned_coords_C'+str(i),objs[obj_ix], sp_x,sp_y,  0.05,eu_i,eu_j,eu_k )
 





if __name__ == '__main__':
    try:
        spawner()
    except rospy.ROSInterruptException:
        pass
