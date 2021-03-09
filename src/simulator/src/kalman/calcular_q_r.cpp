#include "ros/ros.h"
#include <ros/package.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <iterator>
#include "simulator/Parameters.h"
#include "simulator/simulator_parameters.h"
#include "../utilities/simulator_structures.h"
#include <random>
#define NUM_IT 100

std::default_random_engine generator;
double stddev = 0.015;
std::normal_distribution<double> noise(0, stddev);
int it = 0;

parameters params;
float x[NUM_IT],y[NUM_IT];


#include "std_msgs/Float32MultiArray.h"

typedef struct Vertex_ {
        float x;
        float y;
        float r;
} Vertex;


void centroidsCallback(const std_msgs::Float32MultiArray::ConstPtr& centroides)
{
	
	std::vector<Vertex> objs;
	Vertex aux;
	int i = 0;
	float a1,a2,a3,a4,a5,a6;

	for(std::vector<float>::const_iterator it = centroides->data.begin(); it != centroides->data.end(); ++it)
	{
		
		
		if( i == 0 )
		{
			aux.x = *it;
			//std::cout << "X: " << *it << ", ";
			i++;
		}
		else if ( i == 1 )
		{
			aux.y = *it;
			//std::cout << "Y: " << *it << ", ";
			i++;
		}
		else
		{
			aux.r = *it+ noise(generator);
			//std::cout << "R: " << *it << "\n";
			objs.push_back( aux  );
			i = 0;
		}
		
	}

	if(objs.size() > 2 )
	{
		

		for(i = 0; i < objs.size()-2; i++)
		{
			a1 = (objs[i+1].y - objs[i].y);
			a2 = pow(objs[i+1].r,2) - pow(objs[i+2].r,2) - pow(objs[i+1].x,2) + pow(objs[i+2].x,2) - pow(objs[i+1].y,2)  + pow(objs[i+2].y,2) ;
			a3 = objs[i+2].y - objs[i+1].y;
			a4 = pow(objs[i].r,2) - pow(objs[i+1].r,2) - pow(objs[i].x,2) + pow(objs[i+1].x,2) - pow(objs[i].y,2) + pow(objs[i+1].y,2) ;
			a5 = objs[i+2].x - objs[i+1].x;
			a6 = objs[i+1].x - objs[i].x;   
			if(it < NUM_IT)
			{
				x[it] =  pow(params.robot_x - ( (a1 * a2) - (a3 * a4) ) / ( (2 * a5 * a1) - (2 * a6 * a3) ),2);  
				y[it] =  pow(params.robot_y - ( (a6 * a2) - (a5 * a4) ) / ( (2 * a6 * a3) - (2 * a5 * a1) ),2);  
				it ++;
			} 
		}	
	}else
	{
		return ;
	}

	if( it == NUM_IT)
	{
		it = 0;
		for(int i = 1; i < NUM_IT; i++)
		{
			x[0] += x[i];
			y[0] += y[i];
		}
		printf("X: %f \n",x[0]/NUM_IT);
		printf("Y: %f \n",y[0]/NUM_IT );
		std::cout  << "\n";

	}
	
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

}

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "calcular_q_r");
	ros::NodeHandle n;
	ros::Subscriber centroids_sub = n.subscribe("simulator_centroids_pub", 0, centroidsCallback);
	ros::Subscriber params_sub = n.subscribe("simulator_parameters_pub", 0, paramsCallback);
    ros::Rate loop_rate(5);
	while (ros::ok())
	{
	   ros::spinOnce();
	   loop_rate.sleep();
	}
	return 0;
}