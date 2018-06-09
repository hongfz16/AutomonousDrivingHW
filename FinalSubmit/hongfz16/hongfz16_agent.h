#pragma once

#ifndef _HONGFZ_AGENT_
#define _HONGFZ_AGENT_

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include "pnc/simulation/vehicle_agent.h"
#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "common/utils/file/file.h"
#include "common/proto/vehicle_params.pb.h"
#include "pnc/simulation/vehicle_agent_factory.h"

using std::pair;
using std::string;
using std::cout;
using std::endl;

namespace hongfz16
{
#define PI 3.1415926535

typedef interface::geometry::Point3D Point3d;
typedef interface::geometry::Point2D Point2d;
typedef interface::geometry::Vector3d Vector3d;
typedef interface::control::ControlCommand Command;
typedef interface::perception::PerceptionObstacle Obstacle;
typedef interface::perception::PerceptionTrafficLightStatus AllTrafficLight;
typedef interface::perception::SingleTrafficLightStatus TrafficLight;
typedef interface::map::Lane Lane;

struct Params
{
	double preview_length_max;
	double preview_length_min;
	double preview_length_th_min;
	double preview_length_th_max;
	double kp;
	double theta_err_pid[3];
	double ct_err_pid[3];
	double speed_pd[2];
	double max_speed;

	Params()
	{
		kp=1;
		preview_length_max=1.5;
		preview_length_min=0;
		preview_length_th_max=10;
		preview_length_th_min=5;

		theta_err_pid[0]=16;
		theta_err_pid[1]=0;
		theta_err_pid[2]=2;
		
		ct_err_pid[0]=-1;
		ct_err_pid[1]=0;
		ct_err_pid[2]=0;

		speed_pd[0]=0.4;
		speed_pd[1]=0;

		max_speed=14;
	}
};

struct SpeedInfo
{
	double speed;
	double theta;
};

struct Err
{
	double theta_err;
	double ct_err;
};

struct Route
{
	std::vector<pair<Point2d,double> > routes;
};

struct CarInfo
{
	Point2d dest;
	Point2d start;
	bool ready;
	int refer_point_id;
	Route curr_route;
	Err last_err;
	Err int_err;
	double last_delta_speed;
	Point2d road_end_point;
	int curr_road_id;
	double last_dist_to_end;
};

class SimpleVehicleAgent : public simulation::VehicleAgent {
public:
	explicit SimpleVehicleAgent(const std::string& name) : VehicleAgent(name) {}

	virtual void Initialize(const interface::agent::AgentStatus& agent_status);

	virtual interface::control::ControlCommand RunOneIteration(
	    const interface::agent::AgentStatus& agent_status);

private:
	inline double calc_dist(const Point2d& p1, const Point2d& p2);

	inline double calc_theta(const Point2d& p1, const Point2d& p2);

	inline Point2d vec3d_to_p2d(const Vector3d& v);

	inline void clear_route();

	inline void new_route_init(const interface::agent::AgentStatus& agent_status);

	inline void init_param(const interface::agent::AgentStatus& agent_status);

	inline void my_init(const interface::agent::AgentStatus& agent_status);

	inline bool is_reach_dest(const interface::agent::AgentStatus& agent_status);

	inline SpeedInfo get_curr_speed(const interface::agent::AgentStatus& agent_status);

	inline Command control_vehicle(const interface::agent::AgentStatus& agent_status);

	inline double path_tracking(const interface::agent::AgentStatus& agent_status);

	inline double speed_control(const interface::agent::AgentStatus& agent_status);

	inline void speed_planning(const interface::agent::AgentStatus& agent_status);

	inline double get_preview_length(double cspeed);

	inline Err get_err(const interface::agent::AgentStatus& agent_status, SpeedInfo& sinfo);

	inline void update_refer_point(const interface::agent::AgentStatus& agent_status);

	inline void register_debug_info();

	inline pair<double,int> find_dist_to_route(Point2d& p);

	// inline pair<double,int> accurate_find_dist_to_route(Point2D& p, double head);

	inline double plan_next_speed(int destid,double plan_speed,const interface::agent::AgentStatus& agent_status);
	
	inline double pedestrain_plan_speed(pair<double,int>& dist_info);

	inline double car_plan_speed(pair<double,int>& dist_info);

	inline void update_road_end_point(const interface::agent::AgentStatus& agent_status);

private:
	//PLEASE CHANGE THE FILEPREFIX TO YOUR WORKING PATH BEFORE YOU RUN THIS CODE
	string fileprefix="/home/hongfz/Documents/Learn/AutomonousDrivingHW";

	CarInfo car_info;

	Params car_param;
	
	interface::map::Map mapdata;
};
}

#endif