#include "batch_solver.h"
#include <iostream>
#include "ros/ros.h"
#include "utility.h"
#include <visualization_msgs/Marker.h>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <geometry_msgs/Transform.h>
#include "mymsg/neighborpos.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <Eigen/Dense>
#include <signal.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <cmath>
#include "std_msgs/Float64.h"
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265358979323846

size_t N = 25;
size_t m = 2;// represent the num of neighbors for this vehicle
double Hz = 2.0;
size_t update_shift = 0;

std::mutex neig_mtx;
std::mutex init_mtx;

double x_rob;
double y_rob;
double theta_rob;
double x_fol_pos;
double y_fol_pos;
double theta_fol_pos;

double x_grip = 0;
double y_grip = 0;

int back = 0;
int co = 0;
double backspeed = -0.2;

// ******protected by neig_mtx10
std::vector<std::vector<std::vector<double>>> neig;
std::vector<std::vector<double>> pre_states(N+1,std::vector<double>(3.0,0.0));
std::vector<std::vector<double>> pre_inputs(N+1,std::vector<double>(2.0,0.0));

bool solve_success = false;
double d = 0.11;
double ts = 0.5;//1.0 / Hz;//0.3
double safety_dist = 0.15;

std::vector<std::vector<double>> obst;

std::shared_ptr<BatchSolver> bs(new BatchSolver(N,0, 0, 0, d, x_grip, y_grip, theta_rob, ts, safety_dist,obst,neig));

ros::Publisher vehicle_pub;// for visulization
ros::Publisher markerArray;
ros::Publisher control_pub;
ros::Subscriber localization_sub;
ros::Subscriber is_termination;

// Function declaration
void Initialize(ros::NodeHandle& n);
void mySigintHandler(int sig);

void UpdateVisualize();
void LocalizationCallback(const apriltag_ros::AprilTagDetectionArray& msg);


int main(int argc,char* argv[]){
	ros::init(argc,argv,"follower_vehicle");
	ros::NodeHandle n;
	ros::NodeHandle nh;
	signal(SIGINT,mySigintHandler);
   	ros::Rate loop_rate(4);

	Initialize(n);

	std::thread sim_thread(&UpdateVisualize);
	sim_thread.detach();

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	geometry_msgs::Vector3 msg_con;
	msg_con.x = 0.0;
	msg_con.y = 0.0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	return 0;
};

void mySigintHandler(int sig){
	std::cout << "Experiment stopped by user. Stopping Robot..." << std::endl;
	geometry_msgs::Vector3 msg_con;
	msg_con.x = 0;
	msg_con.y = 0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	std::cout<<"Check point mySigintHandler"<<std::endl;
	std::cout << "THE END." << std::endl;
	ros::shutdown();
}

void TerminationCallback(const std_msgs::Bool& msg){
	if(msg.data){
			geometry_msgs::Vector3 msg_con;
			msg_con.x = 0;
			msg_con.y = 0;
			msg_con.z = 0.0;
			control_pub.publish(msg_con);
			std::cout<<"THE END"<<std::endl;
			ros::shutdown();
	}
}


void Initialize(ros::NodeHandle& n){

	std::vector<std::vector<double>> neig1(N+1,std::vector<double>{10,10});//v1
	std::vector<std::vector<double>> neig2(N+1,std::vector<double>{-10,-10});//v2
	// obst.push_back(obst1);
	neig.push_back(neig1);
	neig.push_back(neig2);
	// bs->set_obst_(obst);

	bs->set_ref_states(1,1,PI);
	bs->set_initial_states(0,0,0);
	bs->set_neighbors(neig,neig_mtx);

	// std::cout<<"Check point Initialize"<<std::endl;

	vehicle_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    markerArray = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	control_pub = n.advertise<geometry_msgs::Vector3>("cs_follower",10);
	localization_sub = n.subscribe("tag_detections", 1, LocalizationCallback);
	is_termination = n.subscribe("is_Termination_Leader", 1, TerminationCallback);

};


void UpdateVisualize(){

	ros::Rate loop_rate_sim(Hz);
	// std::cout<<"Check point visualize"<<std::endl;
	ros::spinOnce();
	loop_rate_sim.sleep();

	while(ros::ok()){
		std::lock_guard<std::mutex> lk(init_mtx);
		if(1){
			ObstRviz(obst,safety_dist,markerArray);
			TrajRviz(pre_states, safety_dist,markerArray);
			VehicleRviz(x_fol_pos, y_fol_pos, theta_fol_pos, safety_dist, vehicle_pub);
			HeadingRviz(x_fol_pos, y_fol_pos, theta_fol_pos, safety_dist, vehicle_pub);
			bs->set_initial_states(x_fol_pos, y_fol_pos, theta_fol_pos);
		}

		bs->set_neighbors(neig,neig_mtx);
		bs->Solve(pre_states,pre_inputs,solve_success);

		if(solve_success){
			update_shift = 0;

			geometry_msgs::Vector3 msg_con;
			msg_con.x = pre_inputs[0][0];
			msg_con.y = pre_inputs[0][1];
			msg_con.z = 0.0;
			control_pub.publish(msg_con);



		}else{// if fail to solve, publish the shifted pre_states
			std::cout<<" fail to solve!!!!!"<<std::endl;
			exit(0);

		}

		ros::spinOnce();// send the solution ASAP after the solving
		loop_rate_sim.sleep();


	}
	std::cout << "Experiment stopped by user. Stopping Robot..." << std::endl;
	geometry_msgs::Vector3 msg_con;
	msg_con.x = 0;
	msg_con.y = 0;
	msg_con.z = 0.0;
	control_pub.publish(msg_con);
	std::cout << "THE END." << std::endl;
};


void LocalizationCallback(const apriltag_ros::AprilTagDetectionArray& msg){

	// do we have 1 detection?
	if(msg.detections.size() == 0){
		return;
	}

	for(size_t i = 0; i < msg.detections.size() ; i++){
		if(msg.detections[i].id[0] == 2){
			x_rob = msg.detections[i].pose.pose.pose.position.x;
			y_rob = -msg.detections[i].pose.pose.pose.position.y;

			double x = -msg.detections[i].pose.pose.pose.orientation.x; // Invert quaternion
			double y = -msg.detections[i].pose.pose.pose.orientation.y;
			double w =  msg.detections[i].pose.pose.pose.orientation.w;
			double z = -msg.detections[i].pose.pose.pose.orientation.z;

			theta_rob = -atan2(2.0 * ((w * z) + (x * y)), 1.0 - 2.0 * ((y*y) + (z*z))); // calc Theta

			double theta_offset = -0.0;
			double theta_fol = theta_rob + -PI/2 + theta_offset;
			double x_fol = x_rob + 0.28 * cos(theta_fol); // calc follower Cords
			double y_fol = y_rob + 0.64 * sin(theta_fol);

			std::cout<<"\nPosition - x:"<<x_fol_pos<<"\ty:"<<y_fol_pos<<"\tTheta:"<<theta_fol_pos<<std::endl;
			std::cout<<"Goal     - x:"<<x_fol<<"\ty:"<<y_fol<<"\tTheta:"<<theta_rob<<std::endl;

			bs->set_ref_states(x_fol, y_fol, theta_rob);
		}
		if(msg.detections[i].id[0] == 4){
			x_fol_pos = msg.detections[i].pose.pose.pose.position.x;
			y_fol_pos = -msg.detections[i].pose.pose.pose.position.y;

			double x = -msg.detections[i].pose.pose.pose.orientation.x; // Invert quaternion
			double y = -msg.detections[i].pose.pose.pose.orientation.y;
			double w =  msg.detections[i].pose.pose.pose.orientation.w;
			double z = -msg.detections[i].pose.pose.pose.orientation.z;

			theta_fol_pos = -atan2(2.0 * ((w * z) + (x * y)), 1.0 - 2.0 * ((y*y) + (z*z))); // calc Theta
		}
	}
};
