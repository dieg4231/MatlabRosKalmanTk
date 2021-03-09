#include "ros/ros.h"
#include <ros/package.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iterator>
#include <random>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32MultiArray.h"

std::default_random_engine generator;
double stddev = 0.005;
std::normal_distribution<double> noise(0, stddev);
visualization_msgs::Marker arrow;



typedef struct Vertex_ {
        float x;
        float y;
        float r;
        float t;
} Vertex;


void centroidsCallback(const std_msgs::Float32MultiArray::ConstPtr& centroides)
{

	std::vector<Vertex> objs;
	Vertex aux;
	int i = 0;
	float x = 0,y = 0,a1,a2,a3,a4,a5,a6;
	float ang_robot, ang_arrivo, ang_obj;
	float ang_robot_final = 0;

	for(std::vector<float>::const_iterator it = centroides->data.begin(); it != centroides->data.end(); ++it)
	{	
		if( i == 0 )
		{
			aux.x = *it;
			std::cout << "X: " << *it << ", ";
			i++;
		}
		else if ( i == 1 )
		{
			aux.y = *it;
			std::cout << "Y: " << *it << ", ";
			i++;
		}
		else if ( i == 2 )
		{
			aux.r = *it+ noise(generator);
			std::cout << "R: " << *it << ", "  ;
			i ++;
		}
		else
		{
			aux.t = *it;
			std::cout << "T: " << *it << "\n";
			objs.push_back( aux  );
			i = 0;
		}	
	}

	if(objs.size() > 2)
	{
		ang_robot_final= 0;
		x=y=0;
		for(i = 0; i < objs.size()-2; i++)
		{
			a1 = (objs[i+1].y - objs[i].y);
			a2 = pow(objs[i+1].r,2) - pow(objs[i+2].r,2) - pow(objs[i+1].x,2) + pow(objs[i+2].x,2) - pow(objs[i+1].y,2)  + pow(objs[i+2].y,2) ;
			a3 = objs[i+2].y - objs[i+1].y;
			a4 = pow(objs[i].r,2) - pow(objs[i+1].r,2) - pow(objs[i].x,2) + pow(objs[i+1].x,2) - pow(objs[i].y,2) + pow(objs[i+1].y,2) ;
			a5 = objs[i+2].x - objs[i+1].x;
			a6 = objs[i+1].x - objs[i].x;    
			x +=  ( (a1 * a2) - (a3 * a4) ) / ( (2 * a5 * a1) - (2 * a6 * a3) );  
			y +=  ( (a6 * a2) - (a5 * a4) ) / ( (2 * a6 * a3) - (2 * a5 * a1) );  
	
		}

		x = x / (objs.size()-2);
		y = y / (objs.size()-2);
		printf("iteraciones(%d) X: %f Y : %f \n",i,x,y );

		for(i = 0; i < objs.size(); i++)
		{
			ang_arrivo = objs[i].t; // angulo en el que esta el obstaculo con respecto al robot
			ang_obj = atan2(objs[i].y - y, objs[i].x - x); // angulo del eje x a la linea entre el robot y el objeto
			if( ang_arrivo < 0)
				ang_arrivo = 2 * M_PI + ang_arrivo;
			if( ang_obj < 0)
				ang_obj = 2 * M_PI +ang_obj;
			ang_robot = 2 * M_PI - ang_arrivo + ang_obj;
			if( ang_robot > 2 * M_PI )
				ang_robot -= 2 * M_PI;

			ang_robot_final = ang_robot;
		}
		
		

		printf("Robot thetha: %f \n",ang_robot_final  );
		if( ang_robot_final >  M_PI )
		{
				ang_robot_final -=  2 * M_PI;
				//ang_robot_final *=  -1;

		}else if( ang_robot_final <  -M_PI )
		{
				ang_robot_final +=  2 * M_PI;
				//ang_robot_final *=  -1;
		}

		printf("Robot thetha: %f \n",ang_robot_final  );

	}

	arrow.pose.position.x = x;
	arrow.pose.position.y = y;
	arrow.pose.position.z = 0;
	arrow.pose.orientation = tf::createQuaternionMsgFromYaw(ang_robot_final);
	


	
/*
Legible Y1 Y2 X2 etc
	a1 = Y2 - Y1;
	a2 = pow(R2,2) - pow(R3,2) - pow(X2,2) + pow(X3,2) - pow(Y2,2)  + pow(Y3,2) ;
	a3 = Y3 - Y2
	a4 = pow(R1,2) - pow(R2,2) - pow(X1,2) + pow(X2,2) - pow(Y1,2) + pow(Y2,2) ;
	a5 = X3 - X2;
	a6 = X2 - X1;    
	x =  ( (a1 * a2) - (a3 * a4) ) / ( (2 * a5 * a1) - (2 * a6 * a3) );  
	y =  ( (a6 * a2) - (a5 * a4) ) / ( (2 * a6 * a3) - (2 * a5 * a1) );  

Octave	
	a2 = rd(2)^2 - rd(3)^2 - l_n(1,2)^2 + l_n(1,3)^2 - l_n(2,2)^2  + l_n(2,3)^2 ;
	a3 = l_n(2,3)-l_n(2,2);
	a4 = rd(1)^2 - rd(2)^2 - l_n(1,1)^2 + l_n(1,2)^2 - l_n(2,1)^2 + l_n(2,2)^2 ;
	a5 = l_n(1,3) - l_n(1,2);
	a6 = l_n(1,2) - l_n(1,1);    
	x_z(steps) =  ( (a1 * a2) - (a3 * a4) ) / ( (2 * a5 * a1) - (2 * a6 * a3) );  
	y_z(steps) =  ( (a6 * a2) - (a5 * a4) ) / ( (2 * a6 * a3) - (2 * a5 * a1) );  
*/

	std::cout  << "\n";
}


int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "position_from_views_node");
	ros::NodeHandle n;
	ros::Subscriber params_sub = n.subscribe("simulator_centroids_pub", 0, centroidsCallback);
	ros::Publisher pubposition = n.advertise<visualization_msgs::Marker>("/position_from_views", 1);		

	arrow.header.frame_id = "/map";
    arrow.header.stamp = ros::Time::now();
	arrow.ns = "position_from_views";
	arrow.id = 0;

	arrow.type = visualization_msgs::Marker::ARROW;
	arrow.action = visualization_msgs::Marker::ADD;
	arrow.scale.x=.10;
	arrow.scale.y=.010;
	arrow.scale.z = .010;

	arrow.color.g = 1.0f;
	arrow.color.a = 1.0;
	arrow.color.r = 0.0f;
	arrow.color.b = 0.0f;

  	arrow.lifetime = ros::Duration();

    ros::Rate loop_rate(5);
	while (ros::ok())
	{
		pubposition.publish(arrow);
		ros::spinOnce();
		loop_rate.sleep();
	}
	

	return 0;
}