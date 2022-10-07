/*
 * CONCURSO IBEROAMERICANO DE ROBOTICA ESPACIAL, PEU-UNAM, 2022
 * ETAPA 04 - NAVEGACION
 * LOCALIZACION MEDIATE FILTROS DE PARTICULAS
 *
 * Instrucciones:
 * Escriba el codigo necesario para implementar la localizacion mediante filtros de particulas
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"

#define NOMBRE "APELLIDO_PATERNO_APELLIDO_MATERNO"

#define LASER_DOWNSAMPLING  10
#define SENSOR_NOISE        0.1
#define RESAMPLING_NOISE    0.1
#define MOVEMENT_NOISE      0.1
#define DISTANCE_THRESHOLD  0.2
#define ANGLE_THRESHOLD     0.2

sensor_msgs::LaserScan real_sensor_info;
sensor_msgs::LaserScan real_scan;

geometry_msgs::PoseArray get_initial_distribution(int N, float min_x, float max_x, float min_y, float max_y, float min_a, float max_a)
{
    random_numbers::RandomNumberGenerator rnd;
    geometry_msgs::PoseArray particles;
    particles.poses.resize(N);
    particles.header.frame_id = "map";

    /*
     * EJERCICIO:
     * Genere un conjunto de N particulas (cada una representada por un mensaje de tipo Pose)
     * con posiciones uniformente distribuidas en el bounding box dado.
     * El conjunto de particulas debe ser un mensaje de tipo PoseArray.
     * Para generar numeros aleatorios uniformemente distribuidos, puede usar la funcion rnd.uniformReal()
     * Recuerde que la orientacion en una Pose esta representada por un cuaternion (x,y,z,w)
     * Para los angulos de Euler (roll, pitch, yaw) = (0,0,theta) el respectivo cuaternion esta
     * dado por (0,0,sin(theta/2), cos(theta/2)). 
     */
    
    for(size_t i=0; i < particles.poses.size(); i++)
    {
        particles.poses[i].position.x  = rnd.uniformReal(min_x, max_x);
        particles.poses[i].position.y  = rnd.uniformReal(min_y, max_y);
        float a = rnd.uniformReal(min_a, max_a);
        particles.poses[i].orientation.w = cos(a/2);
        particles.poses[i].orientation.z = sin(a/2);
    }
    return particles;
}

std::vector<sensor_msgs::LaserScan> simulate_particle_scans(geometry_msgs::PoseArray& particles, nav_msgs::OccupancyGrid& map)
{
    std::vector<sensor_msgs::LaserScan> simulated_scans;
    simulated_scans.resize(particles.poses.size());
    /*
     * EJERCICIO:
     * Simule una lectura del laser para cada particula dado el conjunto de particulas y un mapa.
     * Guarde las lecturas simuladas en la variable 'simulated_scans'.
     * Puede usar la funcion occupancy_grid_utils::simulateRangeScan(map, pose, info).
     * Revise documentacion en linea. 
     * http://docs.ros.org/groovy/api/occupancy_grid_utils/html/namespaceoccupancy__grid__utils.html
     * Use la variable 'real_sensor_info' (ya declarada como variable global) para la info del sensor real.
     */
    for(size_t i=0; i < particles.poses.size(); i++)
        simulated_scans[i] = *occupancy_grid_utils::simulateRangeScan(map, particles.poses[i], real_sensor_info);
    return simulated_scans;
}

std::vector<float> calculate_particle_weights(std::vector<sensor_msgs::LaserScan>& simulated_scans, sensor_msgs::LaserScan& real_scan)
{
    std::vector<float> weights;
    weights.resize(simulated_scans.size());
    /*
     * EJERCICIO:
     *
     * Para cada particula, calcule una similitud entre su lectura simulada y la lectura real. 
     * Normalice todas las similitudes (la suma de todos los valores debe ser 1.0)
     * Guarde los resultados en 'weights'.
     * NOTA IMPORTANTE 1. Las lecturas del sensor real estan SUBMUESTREADAS. 
     * Esto es, solo 1 de cada LASER_DOWNSAMPLING lecturas es considerada. 
     * Cuando se comparen las lecturas, por cada lectura simulada, se deben saltar LASER_DOWNSAMPLING lecturas reales
     * NOTA IMPORTANTE 2. Tanto las lecturas reales como las simuladas pueden tener valores infinitos, 
     * por lo que al momento de comparar primero hay que asegurarse de que ambos valores son finitos. 
     */
    
    double weights_sum = 0;
    for(size_t i=0; i < simulated_scans.size(); i++)
    {
        weights[i] = 0;
        for(size_t j=0; j < simulated_scans[i].ranges.size(); j++)
            if(real_scan.ranges[j*LASER_DOWNSAMPLING] < real_scan.range_max && simulated_scans[i].ranges[j] < real_scan.range_max)
               weights[i] += fabs(simulated_scans[i].ranges[j] - real_scan.ranges[j*LASER_DOWNSAMPLING]);
            else
               weights[i] += real_scan.range_max;
        weights[i] /= simulated_scans[i].ranges.size();
        weights[i] = exp(-weights[i]*weights[i]/SENSOR_NOISE);
        weights_sum += weights[i];
    }
    for(int i=0; i<weights.size(); i++)
        weights[i] /= weights_sum;
    return weights;
}

int random_choice(std::vector<float>& weights)
{
    random_numbers::RandomNumberGenerator rnd;
    
    /*
     * EJERCICIO:
     * Escriba un algoritmo para elegir un entero aleatorio en el rango [0, N-1], con N, el tamano de 'weights'.
     * La probabilidad de tomar el entero 'i' esta dada en el respectivo peso weights[i].
     * Devuelva el entero elegido.
     */
    
    float beta = rnd.uniformReal(0, 1);
    for(int i=0; i < weights.size(); i++)
        if(beta < weights[i])
            return i;
        else
            beta -= weights[i];
    return -1;
}

geometry_msgs::PoseArray resample_particles(geometry_msgs::PoseArray& particles, std::vector<float>& weights)
{
    random_numbers::RandomNumberGenerator rnd;
    geometry_msgs::PoseArray resampled_particles;
    resampled_particles.header.frame_id = "map";
    resampled_particles.poses.resize(particles.poses.size());
    /*
     * EJERCICIO:
     * Muestree, con reemplazo, N particulas del conjunto 'particles'.
     * La probabilidad de que la i-esima particula sea muestreada esta dada por weights[i].
     * Use la funcion random_choice para seleccionar una particula con la probabilidad correcta.
     * Agregue ruido gaussiano a para particula muestreada.
     * Use el valor RESAMPLING_NOISE como varianza de este ruido.
     * Regrese el conjunto de nuevas particulas.
     * NOTA IMPORTANTE: Recuerde que la orientacion (roll, pitch, yaw) = (0,0,theta) esta 
     * dada por el cuaternion (0,0,sin(theta/2), cos(theta/2))
     */
    for(size_t i=0; i<particles.poses.size(); i++)
    {
        int idx = random_choice(weights);
        resampled_particles.poses[i].position.x = particles.poses[idx].position.x + rnd.gaussian(0, RESAMPLING_NOISE);
        resampled_particles.poses[i].position.y = particles.poses[idx].position.y + rnd.gaussian(0, RESAMPLING_NOISE);
        float angle = atan2(particles.poses[idx].orientation.z, particles.poses[idx].orientation.w)*2;
        angle += rnd.gaussian(0, RESAMPLING_NOISE);
        resampled_particles.poses[i].orientation.w = cos(angle/2);
        resampled_particles.poses[i].orientation.z = sin(angle/2);
    }
    return resampled_particles;
}

void move_particles(geometry_msgs::PoseArray& particles, float delta_x, float delta_y, float delta_t)
{
    random_numbers::RandomNumberGenerator rnd;
    /*
     * EJERCICIO:
     *
     * Mueva cada particula un desplazamiento dado por delta_x, delta_y and delta_t.
     * El desplazamiento esta dado con respecto a la particula, es decir, para calcular la nueva posicion para
     * cada particula, se necesita rotar delta_x y delta_y, sobre el eje Z, un angulo theta_i, donde theta_i
     * es la orientacion de la particula 'i'.
     * Agregue ruido gaussiano a cada particula. Use MOVEMENT_NOISE como covarianza. 
     */
    for(size_t i=0; i < particles.poses.size(); i++)
    {
        float a = atan2(particles.poses[i].orientation.z, particles.poses[i].orientation.w)*2;
        particles.poses[i].position.x += delta_x*cos(a) - delta_y*sin(a) + rnd.gaussian(0, MOVEMENT_NOISE);
        particles.poses[i].position.y += delta_x*sin(a) + delta_y*cos(a) + rnd.gaussian(0, MOVEMENT_NOISE);
        a += delta_t + rnd.gaussian(0, MOVEMENT_NOISE);
        particles.poses[i].orientation.w = cos(a/2);
        particles.poses[i].orientation.z = sin(a/2);
    }
}

bool check_displacement(geometry_msgs::Pose2D& robot_pose, geometry_msgs::Pose2D& delta_pose)
{
    static geometry_msgs::Pose2D last_pose;
    float delta_x = robot_pose.x - last_pose.x;
    float delta_y = robot_pose.y - last_pose.y;
    float delta_a = robot_pose.theta - last_pose.theta;
    if(delta_a >  M_PI) delta_a -= 2*M_PI;
    if(delta_a < -M_PI) delta_a += 2*M_PI;
    if(sqrt(delta_x*delta_x + delta_y*delta_y) > DISTANCE_THRESHOLD || fabs(delta_a) > ANGLE_THRESHOLD)
    {
        last_pose = robot_pose;
        delta_pose.x =  delta_x*cos(robot_pose.theta) + delta_y*sin(robot_pose.theta);
        delta_pose.y = -delta_x*sin(robot_pose.theta) + delta_y*cos(robot_pose.theta);
        delta_pose.theta = delta_a;
        return true;
    }
    return false;
}

geometry_msgs::Pose2D get_robot_odometry(tf::TransformListener& listener)
{
    tf::StampedTransform t;
    geometry_msgs::Pose2D pose;
    try{
        listener.lookupTransform("odom", "base_link", ros::Time(0), t);
        pose.x = t.getOrigin().x();
        pose.y = t.getOrigin().y();
        pose.theta = atan2(t.getRotation().z(), t.getRotation().w())*2;
    }
    catch(std::exception &e){
        pose.x = 0;
        pose.y = 0;
        pose.theta = 0;
    }
    return pose;
}

geometry_msgs::Pose2D get_robot_pose_estimation(geometry_msgs::PoseArray& particles)
{
    geometry_msgs::Pose2D p;
    float z = 0;
    float w = 0;
    for(size_t i=0; i < particles.poses.size(); i++)
    {
        p.x += particles.poses[i].position.x;
        p.y += particles.poses[i].position.y;
        z   += particles.poses[i].orientation.z;
        w   += particles.poses[i].orientation.w;
        
    }
    p.x /= particles.poses.size();
    p.y /= particles.poses.size();
    z   /= particles.poses.size();
    w   /= particles.poses.size();
    p.theta = atan2(z, w)*2;
    return p;
}

void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg){real_scan = *msg;}

tf::Transform get_map_to_odom_transform(geometry_msgs::Pose2D odom, geometry_msgs::Pose2D loc)
{
    tf::Transform odom_to_base(tf::Quaternion(0,0,sin(odom.theta/2),cos(odom.theta/2)), tf::Vector3(odom.x,odom.y,0));
    tf::Transform map_to_base(tf::Quaternion(0,0,sin(loc.theta/2),cos(loc.theta/2)), tf::Vector3(loc.x, loc.y, 0));
    return map_to_base*odom_to_base.inverse();
}

int main(int argc, char** argv)
{
    std::cout << "EJERCICIO 6 - FILTROS DE PARTICULAS - " << NOMBRE << std::endl;
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n("~");
    ros::Rate loop(20);
    ros::Subscriber sub_scan      = n.subscribe("/hsrb/base_scan", 1, callback_laser_scan);
    ros::Publisher  pub_particles = n.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1); 
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::GetMap srv_get_map;

    float init_min_x = -1;
    float init_min_y = -1;
    float init_min_a = -1;
    float init_max_x = 2.0;
    float init_max_y = 2.0;
    float init_max_a = 1;
    float number_of_particles = 200;
    if(ros::param::has("~n"))
        ros::param::get("~n", number_of_particles);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", init_max_x);
    if(ros::param::has("~min_x"))
        ros::param::get("~min_x", init_min_x);
    if(ros::param::has("~min_y"))
        ros::param::get("~min_y", init_min_y);
    if(ros::param::has("~min_a"))
        ros::param::get("~min_a", init_min_a);
    if(ros::param::has("~max_x"))
        ros::param::get("~max_x", init_max_x);
    if(ros::param::has("~max_y"))
        ros::param::get("~max_y", init_max_y);
    if(ros::param::has("~max_a"))
        ros::param::get("~max_a", init_max_a);

    /*
     * IMPORTANT VARIABLES FOR THE LOCALIZATION PROCESS
     */
    geometry_msgs::PoseArray particles;                   //A set of N particles
    nav_msgs::OccupancyGrid static_map;                   //A static map
    std::vector<sensor_msgs::LaserScan> simulated_scans;  //A set of simulated laser readings, one scan per particle
    std::vector<float> particle_weights;                  //A set of weights for each particle
    geometry_msgs::Pose2D robot_odom;                     //Position estimated by the odometry
    geometry_msgs::Pose2D delta_pose;                     //Displacement since last pose estimation
    geometry_msgs::Pose2D robot_pose;                     //Estimated robot position with respect to map
    tf::Transform map_to_odom_transform;                  //Transformation from map to odom frame (which corrects odometry estimation)

    /*
     * Sentences for getting the static map, info about real lidar sensor,
     * and initialization of corresponding arrays.
     */
    ros::service::waitForService("/static_map", ros::Duration(20));
    ros::service::call("/static_map", srv_get_map);
    static_map = srv_get_map.response.map;
    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hsrb/base_scan");
    real_sensor_info = real_scan;
    real_sensor_info.angle_increment *= LASER_DOWNSAMPLING;
    std::cout << "Real Scan Info: Number of readings: " << real_scan.ranges.size() << std::endl;
    std::cout << "Min angle: " << real_scan.angle_min << std::endl;
    std::cout << "Angle increment: " << real_scan.angle_increment << std::endl;
    std::cout << "Scan downsampling: " << LASER_DOWNSAMPLING << std::endl;

    particles = get_initial_distribution(number_of_particles, init_min_x, init_max_x, init_min_y, init_max_y, init_min_a, init_max_a);
    robot_pose = get_robot_pose_estimation(particles);
    robot_odom = get_robot_odometry(listener);
    map_to_odom_transform = get_map_to_odom_transform(robot_odom, robot_pose);
    check_displacement(robot_odom, delta_pose);
    pub_particles.publish(particles);
    while(ros::ok())
    {
        robot_odom = get_robot_odometry(listener);
        if(check_displacement(robot_odom, delta_pose))
        {
            std::cout << "Displacement detected. Updating pose estimation..." << std::endl;
            
            move_particles(particles, delta_pose.x, delta_pose.y, delta_pose.theta);
            simulated_scans = simulate_particle_scans(particles, static_map);
            particle_weights = calculate_particle_weights(simulated_scans, real_scan);
            particles = resample_particles(particles, particle_weights);
            
            pub_particles.publish(particles);
            map_to_odom_transform = get_map_to_odom_transform(robot_odom, get_robot_pose_estimation(particles));
        }
        broadcaster.sendTransform(tf::StampedTransform(map_to_odom_transform, ros::Time::now(), "map", "odom"));
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
