/*
Kalaman filter as "Probabilistic robotics page: 204"
*/

#include "ros/ros.h"
#include <ros/package.h>
#include <random>
#include <iostream>
#include <Eigen/Dense>
#include "simulator/Parameters.h"
#include "simulator/simulator_parameters.h"
#include "../utilities/simulator_structures.h"
#include "simulator/simulator_base.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovariance.h"
using namespace Eigen;

const double mean = 0.0;
double stddev_land = 0.02;

std::default_random_engine generator;
std::normal_distribution<double> noise_distance(mean, stddev_land);




geometry_msgs::PoseWithCovariance  prediccion;
geometry_msgs::PoseWithCovariance  actualizacion;

visualization_msgs::Marker arrow; 
parameters params;


Matrix3d q;            // Error del proceso 
Matrix3d r;            // Error de medicion  
Matrix3d p;            // Matriz de covarianza
Matrix3d H;            // Jacobiano de la funcion H
Vector3d x_;           // Vector de estado
Matrix3d s;            // s xD
Matrix3d k;            // ganancia de Kalman
Matrix3d I;           // Identity Matrix

typedef struct LandMark_ {
        float x; //Coordenada del robot x
        float y; //Coordenada del robot y
        float r; //Distancia del landmark al robot
        float t; //Angulo del landmark respecto al robot
} LandMark;

double stddev_distance = 0.03;
double stddev_theta = 0.174533;

std::vector<LandMark> objs;

std::vector<Vector2d> position_ideal;
std::vector<Vector2d> position_real;
std::vector<Vector2d> position_kalman;

int swi;
double theta_track;
bool debug=0;

void centroidsCallback(const std_msgs::Float32MultiArray::ConstPtr& centroides)
{
	int i = 0;
	LandMark aux;
	objs.clear();

	for(std::vector<float>::const_iterator it = centroides->data.begin(); it != centroides->data.end(); ++it)
	{	
		if( i == 0 )
		{
			aux.x = *it;// +  noise_distance(generator) ;
			i++;
		}
		else if ( i == 1 )
		{
			aux.y = *it;// +  noise_distance(generator) ;
			i++;
		}
		else if ( i == 2 )
		{
			aux.r = *it ;//+  noise_distance(generator) ; /*****************Add noise ?********************/
			i ++;
		}
		else
		{
			aux.t = *it;
			if(aux.t > M_PI)
				aux.t -= 2*M_PI;
			if(aux.t < -M_PI)
				aux.t += 2*M_PI;
			objs.push_back( aux );
			i = 0;
		}	
	}
}


bool kalmanCallback(simulator::simulator_base::Request  &req ,simulator::simulator_base::Response &res)
{
	float theta;
	float dist;
	float angle_z_i_t_hat;
	Vector3d v; // Diference of measurment and predicted measurment
	Matrix3d vf;
	Vector3d z_i_t;
	Vector3d z_i_t_hat;

	
	if(req.new_simulation) // Restart variables for new estimation 
	{
		std::cout << "New simulator: " << "\n";
		r << 0.00001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001;
	    //q << 0.00001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001;
	    q << pow(stddev_distance,2), 0, 0,0, pow(stddev_distance,2), 0, 0, 0, pow(stddev_theta,2);
		p << 0,0,0,0,0,0,0,0,0;
		ros::Duration(0.1).sleep();
		x_ << params.robot_x ,params.robot_y,params.robot_theta;
		if(x_(2) > M_PI) x_(2) -= 2 * M_PI;
		if(x_(2) < M_PI) x_(2) += 2 * M_PI;
		swi = 0;
		position_ideal.clear();
		position_real.clear();
		position_kalman.clear();

		position_ideal.push_back(Vector2d(x_(0),x_(1)));
		position_real.push_back(Vector2d(x_(0),x_(1)));
		position_kalman.push_back(Vector2d(x_(0),x_(1)));
		theta_track = x_(2);

		for(int i = 0 ; i < 36; i++)
		{
			prediccion.covariance[i] = 0;
			actualizacion.covariance[i] = 0;
		}

	}

	if(swi == 0) // Prediction step
	{	
		swi = 1;

		if(debug)std::cout << "Inicio: \n" << x_ << "\n ------\n";
	    theta = x_(2) + req.theta;
		x_(0) = x_(0) + req.distance * cos(theta) ;
		x_(1) = x_(1) + req.distance * sin(theta) ;
		x_(2) = theta; 
	    
	    vf <<  	1, 0, -req.distance * sin(theta), 0, 1,  req.distance * cos(theta), 0, 0,  1;
		p = vf * p * vf.transpose() + q;
		if(debug)std::cout << "Fin prediccion x: \n" << x_ << "\n ------\n";
		if(debug)std::cout << "Fin prediccion p: \n" << p << "\n ------\n";

		theta_track += req.theta;
		position_ideal.push_back(Vector2d( position_ideal.back()(0) + req.distance * cos(theta_track) ,position_ideal.back()(1) + req.distance * sin(theta_track) ) );
		

		prediccion.pose.position.x = x_(0);
		prediccion.pose.position.y = x_(1);
		prediccion.pose.position.z = 0;

		prediccion.pose.orientation.x = 0;
		prediccion.pose.orientation.y = 0;
		prediccion.pose.orientation.z = x_(2);

		prediccion.covariance[0] = p(0,0);
		prediccion.covariance[1] = p(0,1);
		prediccion.covariance[5] = p(0,2);
		prediccion.covariance[6] = p(1,0);
		prediccion.covariance[7] = p(1,1);
		prediccion.covariance[11] = p(1,2);
		prediccion.covariance[30] = p(2,0);
		prediccion.covariance[31] = p(2,1);
		prediccion.covariance[35] = p(2,2);

		return true;
	}	
	else // Actualizacion
	{
		swi = 0;
	
		for(std::vector<LandMark>::const_iterator mj = objs.begin(); mj != objs.end(); ++mj)
		{	
			if(debug)std::cout << "Centroide x y r t : \n" << mj->x << " , " << mj->y << " , " << mj->r << " , " << mj->t   << "\n ------\n";
			
			z_i_t << mj->r , mj->t ,0; // The real measurement
			
			if(debug)std::cout << "z_i_t: \n" << z_i_t << "\n ------\n";
			
			// Calculo de las lecturas esperadas respecto a la posicion del robot obtenida en el paso de prediccion
				
				dist = pow( mj->x - x_(0),2 ) + pow( mj->y - x_(1) ,2); // Radicando de la de la distancia entre un landmark y el robot 

				// Angulo del landmark respecto al robot
				angle_z_i_t_hat  = atan2(mj->y - x_(1), mj->x - x_(0)) - x_(2);
			
				// Validacion para que el angulo obtenido siempre este entre pi y -pi
				if (angle_z_i_t_hat > M_PI)
					angle_z_i_t_hat -= 2*M_PI;
				if (angle_z_i_t_hat < -M_PI)
					angle_z_i_t_hat += 2*M_PI;

				// Vector de lecturas esperadas
				z_i_t_hat  << sqrt(dist), angle_z_i_t_hat, 0 ;

				if(debug)std::cout << "z_i_t_hat: \n" << z_i_t_hat << "\n ------\n";
			
			// fin calculo de lecturas esperadas
			
			// El jacobiano de la funcion h() con respecto al robot	
			H << -( mj->x - x_(0) ) / sqrt(dist) , -( mj->y - x_(1) ) / sqrt(dist),0,
			 	  ( mj->y - x_(1) ) / dist, -( mj->x - x_(0) ) / dist,-1,
			 	  0,0,0; // this row is zero por que  la etiqueta del landmark no depende de la posicion del robot
			
			if(debug)std::cout << "H: \n" << H << "\n ------\n";
			
			s = ( H * p * H.transpose() ) + r; // S xD
			
			if(debug)std::cout << "S: \n" << s << "\n ------\n";

			k = p * H.transpose() * s.inverse(); // Ganancia de kalman
			
			if(debug)std::cout << "K: \n"<<  k << "\n ------\n";

			v = z_i_t - z_i_t_hat;  // original menos lo esperadp_
			
			if(debug)std::cout << "V: \n"<< v << "\n ------\n";
			
			x_ = x_ + (k*v); // Actualizacion vector de estado
			
			if(debug)std::cout << "X: \n" << x_ << "\n ------\n";
			
			p = ( I - k * H ) * p; // Actualizacion matriz de covarianza
			
			if(debug)std::cout << "P: \n" << p << "\n ------\n";

		}
		
		// Validacion para que el angulo obtenido siempre este entre pi y -pi
		if (x_(2) > M_PI)
			x_(2) -= 2*M_PI;
		if (x_(2) < -M_PI)
			x_(2) += 2*M_PI;

		// Arrow for visualization in rviz
		position_real.push_back(Vector2d(params.robot_x,params.robot_y));
		position_kalman.push_back(Vector2d(x_(0),x_(1)));
		
		arrow.pose.position.x = x_(0);
		arrow.pose.position.y = x_(1);
		arrow.pose.position.z = 0;
		arrow.pose.orientation = tf::createQuaternionMsgFromYaw(x_(2));


		actualizacion.pose.position.x = x_(0);
		actualizacion.pose.position.y = x_(1);
		actualizacion.pose.position.z = 0;

		actualizacion.pose.orientation.x = 0;
		actualizacion.pose.orientation.y = 0;
		actualizacion.pose.orientation.z = x_(2);

		actualizacion.covariance[0] = p(0,0);
		actualizacion.covariance[1] = p(0,1);
		actualizacion.covariance[5] = p(0,2);
		actualizacion.covariance[6] = p(1,0);
		actualizacion.covariance[7] = p(1,1);
		actualizacion.covariance[11] = p(1,2);
		actualizacion.covariance[30] = p(2,0);
		actualizacion.covariance[31] = p(2,1);
		actualizacion.covariance[35] = p(2,2);


		if(debug)std::cout << "Prediccion Final: \n" << x_ << "\n ------\n";

		return true;
	}

}

int run=0;
void paramsCallback(const simulator::Parameters::ConstPtr& paramss)
{
	  params.robot_x             = paramss->robot_x   ;
	  params.robot_y             = paramss->robot_y   ;
	  params.robot_theta         = paramss->robot_theta   ;    
	  params.robot_radio         = paramss->robot_radio   ;    
	  params.robot_max_advance   = paramss->robot_max_advance   ;          
	  params.robot_turn_angle    = paramss->robot_turn_angle   ;         
	  params.laser_num_sensors   = paramss->laser_num_sensors   ;          
	  params.laser_origin        = paramss->laser_origin         ;     
	  params.laser_range         = paramss->laser_range   ;    
	  params.laser_value         = paramss->laser_value   ;    
	  strcpy(params.world_name ,paramss -> world_name.c_str());       
	  params.noise               = paramss->noise   ;   
	  params.run                 = paramss->run   ; 
	  params.light_x             = paramss->light_x;
	  params.light_y             = paramss->light_y;
	  params.behavior            = paramss->behavior; 

	  if( run && ! params.run   )
	  {
	  	std::cout << "BEGIN Path :: \n  Real_x Real_y Real_thetas Kalman_x Kalman_y Kalman_thetas \n";
	  	for(int i = 0; i < position_kalman.size(); ++i)
			std::cout  <<  position_real[i](0) << " " << position_real[i](1) << " " << position_real[i](2) << " "  <<   position_kalman[i](0) << " " << position_kalman[i](1) << " " << position_kalman[i](2) << " \n";
			//std::cout << position_ideal[i](0) << " " << position_ideal[i](1) << " " <<  position_real[i](0) << " " << position_real[i](1) << " "  <<   position_kalman[i](0) << " " << position_kalman[i](1)<<" \n";
		std::cout << "END Path :: \n";
	  }
	  run = params.run;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "kalman");
	ros::NodeHandle n;
	ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
	ros::ServiceServer service = n.advertiseService("simulator_kalman_serv", kalmanCallback);
	ros::Publisher pubposition = n.advertise<visualization_msgs::Marker>("/position_from_kalman", 1);		
	ros::Subscriber centroids_sub = n.subscribe("simulator_centroids_pub", 0, centroidsCallback);

	ros::Publisher prediction_pub = n.advertise<geometry_msgs::PoseWithCovariance>("/kalman_actualization", 1);
	ros::Publisher actualization_pub = n.advertise<geometry_msgs::PoseWithCovariance>("/kalman_prediction", 1);


	arrow.header.frame_id = "/map";
    arrow.header.stamp = ros::Time::now();
	arrow.ns = "position_from_kalman";
	arrow.id = 0;

	arrow.type = visualization_msgs::Marker::ARROW;
	arrow.action = visualization_msgs::Marker::ADD;
	arrow.scale.x=.10;
	arrow.scale.y=.010;
	arrow.scale.z = .010;

	arrow.color.g = 0.0f;
	arrow.color.a = 1.0;
	arrow.color.r = 1.0f;
	arrow.color.b = 0.0f;

	I << 1,0,0,0,1,0,0,0,1;
	ros::Rate loop_rate(5);
	
	while (ros::ok())
	{
		pubposition.publish(arrow);
		prediction_pub.publish(actualizacion);
		actualization_pub.publish(prediccion);
		ros::spinOnce();
		loop_rate.sleep();
	}

}