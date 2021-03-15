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

using namespace Eigen;
visualization_msgs::Marker arrow;

parameters params;
double stddev_distance = 0.005;
double stddev_theta = 0.05;
double laser = 0.015;
double stddev_angle_laser = 0.015;

Matrix3f q;
Matrix3f r;
// Matriz de covarianza
Matrix3f p;
// Jacobiano de la funcion H
Matrix3f vh;
Matrix3f sigma;
Vector3f x_;
Matrix3f s;
Matrix3f k;
Matrix3f vf;
Vector3f v;
Vector3f z;

std::vector<Vector2d> position_ideal;
std::vector<Vector2d> position_real;
std::vector<Vector2d> position_kalman;
int swi = 0;
double theta_track;
bool debug = 1;
int run = 0;


bool kalmanCallback(simulator::simulator_base::Request  &req ,simulator::simulator_base::Response &res)
{
	float theta;

	std::cout << "NNNNNNNNew simulator: " << req.new_simulation<< "\n";
	if(req.new_simulation)
	{
		r << 	0.002185 , 0, 0,
     	0, 0.021833 , 0,
     	0, 0, pow(stddev_angle_laser,2);
		/*Calcular  realmente estos valores*/	
		q << 	pow(stddev_distance,2), 0, 0,
	     		0, pow(stddev_distance,2), 0,
	     		0, 0, pow(stddev_theta,2);
		p << 0,0,0,0,0,0,0,0,0;
		vh << 1,0,0,0,1,0,0,0,1;
		sigma << 0,0,0,0,0,0,0,0,0;
		
		ros::Duration(0.1).sleep();
		x_ << params.robot_x ,params.robot_y,params.robot_theta;
		//if(x_(2) > M_PI) x_(2) -= 2 * M_PI;
		//if(x_(2) < M_PI) x_(2) += 2 * M_PI;
		swi = 0;
		position_ideal.clear();
		position_real.clear();
		position_kalman.clear();

		position_ideal.push_back(Vector2d(x_(0),x_(1)));
		position_real.push_back(Vector2d(x_(0),x_(1)));
		position_kalman.push_back(Vector2d(x_(0),x_(1)));
		theta_track = x_(2);
		std::cout << "NNNNNNNNew simulator: " << req.new_simulation<< "\n";
	}

	if( swi == 0 )
	{
		swi = 1;
		std::cout << "Antes: \n" << x_ << "\n ------\n";
		Matrix3f vf;

	    theta = x_(2) + req.theta;
		x_(0) = x_(0) + req.distance * cos(theta) ;
		x_(1) = x_(1) + req.distance * sin(theta) ;
		x_(2) = theta; 

		vf <<  	1, 0, -req.distance * sin(theta), 0, 1,  req.distance * cos(theta), 0, 0,  1;

		p = vf * p * vf.transpose() + q;
		if(debug)std::cout << "Fin prediccion x: \n" << x_ << "\n ------\n";
		theta_track += req.theta;
		position_ideal.push_back(Vector2d( position_ideal.back()(0) + req.distance * cos(theta_track) ,position_ideal.back()(1) + req.distance * sin(theta_track) ) );
		
	}
	else // Prediction
	{

		swi = 0;	
		
		// Actualizacion
		s = ( vh * p * vh.transpose() ) + r;

		k = p * vh.transpose() * s.inverse();

		v = z - x_;  // original menos lo esperadp_

		if(debug)std::cout << "Real: \n" << z << " predicho: \n" << x_ << " Res \n" << v << "  .....\n";
		if(debug)std::cout << "Justo antes \n" << x_ << "  .....\n";
		x_ = x_ + (k*v);
		if(debug)std::cout << "Justo despues \n" << x_ << "  .....\n";
		//p = p + k*s*k.transpose;
		p = p - k*vh*p;

		if(debug)std::cout << "FINAL Real: \n" << z << " predicho: \n" << x_ << " Res \n" << v << "  .....\n";
		

		// Validacion para que el angulo obtenido siempre este entre pi y -pi
		//if (x_(2) > M_PI)
		//	x_(2) -= 2*M_PI;
		//if (x_(2) < -M_PI)
		//	x_(2) += 2*M_PI;

		// Arrow for visualization in rviz
		position_real.push_back(Vector2d(params.robot_x,params.robot_y));
		position_kalman.push_back(Vector2d(x_(0),x_(1)));

		arrow.pose.position.x = x_(0);
		arrow.pose.position.y = x_(1);
		arrow.pose.position.z = 0;
		arrow.pose.orientation = tf::createQuaternionMsgFromYaw(x_(2));
		
		if(debug)std::cout << "Prediccion Final: \n" << x_ << "\n ------\n";

	}

	return true;
}


void position_from_views_Callback(const visualization_msgs::Marker::ConstPtr& pose)
{
	double roll, pitch, yaw;
	tf::Quaternion q(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	z(0) = pose->pose.position.x;
	z(1) = pose->pose.position.y;
	z(2) = yaw;

}


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
	ros::init(argc, argv, "kalman2");
	ros::NodeHandle n;
	ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
	ros::ServiceServer service = n.advertiseService("simulator_kalman_serv2", kalmanCallback);
	ros::Publisher pubposition = n.advertise<visualization_msgs::Marker>("/position_from_kalman", 1);		
	ros::Subscriber position_from_views_sub = n.subscribe("/position_from_views", 0, position_from_views_Callback);

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


	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		//pubposition.publish(arrow);
		ros::spinOnce();
		loop_rate.sleep();
	}

}