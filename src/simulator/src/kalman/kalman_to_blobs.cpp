/*
Kalaman filter as "Probabilistic robotics page: 204"
*/

#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "geometry_msgs/PoseWithCovariance.h"
#include "std_msgs/Float32MultiArray.h"

std_msgs::Float32MultiArray blob_prediction;
std_msgs::Float32MultiArray blob_actualization;

float old_pose_x_actualization = 999992;
float old_pose_x_prediction = 999992;

bool debug= true;


void predictionCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& data)
{
	double alfa;

	if(data->pose.position.x != old_pose_x_prediction)
	{
		Eigen::MatrixXcd A(2,2);

		if(debug)
		{
			printf("Actualizacion Inicia \n");
			for(int i = 0; i < 2; i++)
			{
				for(int j = 0; j < 2; j++)
				{
					A(i,j) = std::complex<double>(data->covariance[(6*i)+j],0.0);
					std::cout << A(i,j).real() << " ";
				}
				printf("\n");
			}
		}

		blob_prediction.data.clear();
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcd> es(A);

		if(debug)
		{
			for(int i = 0; i < 2; i++)
			{
				printf("\t%d: \t", i+1);
				std::cout << es.eigenvalues()[i] << " ";
			}
			printf("\n");
		}

		if(es.eigenvalues()[0] > es.eigenvalues()[1])
		{
			alfa = atan( es.eigenvectors().col(0)[0].real() / es.eigenvectors().col(0)[1].real() );
			blob_prediction.data.push_back(data->pose.position.x);
			blob_prediction.data.push_back(data->pose.position.y);
			blob_prediction.data.push_back(alfa);
			blob_prediction.data.push_back( 2 * sqrt( 5.991* es.eigenvalues()[0]) );
			blob_prediction.data.push_back( 2 * sqrt( 5.991* es.eigenvalues()[1]) );
		}
		else
		{
			alfa = atan( es.eigenvectors().col(1)[0].real() / es.eigenvectors().col(1)[1].real() );
			blob_prediction.data.push_back(data->pose.position.x);
			blob_prediction.data.push_back(data->pose.position.y);
			blob_prediction.data.push_back(alfa);
			blob_prediction.data.push_back(2 * sqrt( 5.991* es.eigenvalues()[1]) );
			blob_prediction.data.push_back(2 * sqrt( 5.991* es.eigenvalues()[0]) );
		}

		old_pose_x_prediction = data->pose.position.x;
	}

}

void actualizationCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& data)
{
	double alfa;

	if(data->pose.position.x != old_pose_x_actualization)
	{
		Eigen::MatrixXcd A(2,2);

		if(debug)
		{
			printf("Prediccion Inicia \n");
			for(int i = 0; i < 2; i++)
			{
				for(int j = 0; j < 2; j++)
				{
					A(i,j) = std::complex<double>(data->covariance[(6*i)+j],0.0);
					std::cout << A(i,j).real() << " ";
				}
				printf("\n");
			}
		}

		blob_actualization.data.clear();
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcd> es(A);

		if(debug)
		{
			printf("Eigenvalues \n");
			for(int i = 0; i < 2; i++)
			{
				printf("\t%d: \t", i+1);
				std::cout << es.eigenvalues()[i] << " ";
			}
			printf("\n");


			std::cout << es.eigenvectors() << "\n";
		}

		if(es.eigenvalues()[0] > es.eigenvalues()[1])
		{
			alfa = atan( es.eigenvectors().col(0)[1].real() / es.eigenvectors().col(0)[0].real() );
			blob_actualization.data.push_back(data->pose.position.x);
			blob_actualization.data.push_back(data->pose.position.y);
			blob_actualization.data.push_back(alfa);
			blob_actualization.data.push_back(2 * sqrt( 5.991* es.eigenvalues()[0]) );
			blob_actualization.data.push_back(2 * sqrt( 5.991* es.eigenvalues()[1]) );
		}
		else
		{
			alfa = atan( es.eigenvectors().col(1)[1].real() / es.eigenvectors().col(1)[0].real() );
			blob_actualization.data.push_back(data->pose.position.x);
			blob_actualization.data.push_back(data->pose.position.y);
			blob_actualization.data.push_back(alfa);
			blob_actualization.data.push_back(2 * sqrt( 5.991* es.eigenvalues()[1]) );
			blob_actualization.data.push_back(2 * sqrt( 5.991* es.eigenvalues()[0]) );
		}

		old_pose_x_actualization = data->pose.position.x;
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "kalman_to_blobs");
	ros::NodeHandle n;

	ros::Subscriber actualization_sub = n.subscribe("/kalman_actualization", 1, actualizationCallback);
	ros::Subscriber prediction_sub = n.subscribe("/kalman_prediction", 1, predictionCallback);

	
	ros::Publisher blob_prediction_pub = n.advertise<std_msgs::Float32MultiArray>("blob_prediction", 1);
	ros::Publisher blob_actualization_pub = n.advertise<std_msgs::Float32MultiArray>("blob_actualization", 1);
	ros::Rate loop_rate(5);
	
	while (ros::ok())
	{
		blob_prediction_pub.publish(blob_prediction);
		blob_actualization_pub.publish(blob_actualization);

		ros::spinOnce();
		loop_rate.sleep();
	}

}