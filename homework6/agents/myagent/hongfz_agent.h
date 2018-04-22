#pragma once

#include<iostream>
#include<fstream>
#include<vector>
#include<cmath>
#include<string>

#include "homework6/simulation/vehicle_agent.h"
#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "common/utils/file/file.h"
#include "homework6/simulation/vehicle_agent_factory.h"

#include "FindRoute.h"
#include "GetPredSucc.h"

namespace hongfz16
{
class SimpleVehicleAgent : public simulation::VehicleAgent {
public:
	explicit SimpleVehicleAgent(const std::string& name) : VehicleAgent(name) {}

	virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
	  string originmap=fileprefix+"homework6/map/grid2/map_proto.txt";
	  string processedmap=fileprefix+"homework6/agents/myagent/mymap/new_map_proto.txt";
	  //hongfz16::GetPredSucc(originmap,processedmap);
	  const interface::geometry::Point3D endp=agent_status.route_status().destination();
	  const interface::geometry::Vector3d startp=agent_status.vehicle_status().position();
	  hongfz16::FindRoute(processedmap,startp,endp,route);
	  nextdest=1;
	  totaldests=route.route_point_size();
	  old_agent_status.CopyFrom(agent_status);
	  veld=0;
	  accd=0;

	  cout<<"Finish Init!"<<endl;
	}

	virtual interface::control::ControlCommand RunOneIteration(
	    const interface::agent::AgentStatus& agent_status) {
	  interface::control::ControlCommand command;

	  if(finish)
	  	return command;

	  if(ReachNextDest(agent_status) && nextdest<totaldests)
	  {
	  	++nextdest;
	  	cout<<"nextdest"<<endl;
	  }
	  
	  if(!start_pulling_ && !velocity_reached_threshold_ && CalcVelocity(agent_status.vehicle_status().velocity())>=5.0)
	  {
	  	velocity_reached_threshold_=true;
	  	need_acc_=false;
	  	start_pulling_=false;
	  	desired_acc=0;
	  	desired_velocity=5.0;

	  	cout<<"Reach 5m/s"<<endl;
	  }

	  if(!start_pulling_ && ReachPulling(agent_status))
	  {
	  	start_pulling_=true;
	  	need_acc_=false;
	  	velocity_reached_threshold_=false;
	  	desired_acc=-comfort_acc;

	  	cout<<"Start pulling"<<endl;
	  }

	  if(velocity_reached_threshold_)
	  {
	  	double ratio=PIDControlVel(agent_status);
	  	cout<<ratio<<endl;
	  	if(ratio<0)
	  		command.set_brake_ratio(-ratio);
	  	else
	  		command.set_throttle_ratio(ratio);
	  }

	  if(start_pulling_)
	  {
	  	if(CalcVelocity(agent_status.vehicle_status().velocity())<TH)
	  		finish=true;
	  	double ratio=PIDControlAcc(agent_status);
	  	cout<<ratio<<endl;
	  	if(ratio<0)
	  		command.set_brake_ratio(-ratio);
	  	else
	  		command.set_throttle_ratio(ratio);
	  }

	  if(need_acc_)
	  {
	  	desired_acc=comfort_acc;
	  	double ratio=PIDControlAcc(agent_status);
	  	cout<<ratio<<endl;
	  	if(ratio<0)
	  		command.set_brake_ratio(-ratio);
	  	else
	  		command.set_throttle_ratio(ratio);
	  }

	  old_agent_status.CopyFrom(agent_status);
	  return command;
	}

	private:
	double CalcDistance(const interface::geometry::Vector3d& position,
	                    const interface::geometry::Point3D& destination) {
	  double sqr_sum =
	      math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
	  ;
	  return std::sqrt(sqr_sum);
	}

	double CalcVelocity(const interface::geometry::Vector3d& velocity) {
	  double sqr_sum = math::Sqr(velocity.x()) + math::Sqr(velocity.y());
	  ;
	  return std::sqrt(sqr_sum);
	}

	double GetAccX(const interface::geometry::Vector3d& acc)
	{
		return acc.x();
		//return std::sqrt(math::Sqr(acc.x())+math::Sqr(acc.y()));
	}

	double CalcDist3n2(const interface::geometry::Vector3d& position,
										 const interface::geometry::Point2D& dest)
	{
		return std::sqrt(math::Sqr(position.x()-dest.x())+math::Sqr(position.y()-dest.y()));
	}

	bool ReachNextDest(const interface::agent::AgentStatus& agent_status)
	{
		double dist=CalcDist3n2(agent_status.vehicle_status().position(),route.route_point(nextdest));
		if(dist<TH)
			return true;
		return false;
	}

	bool ReachPulling(const interface::agent::AgentStatus& agent_status)
	{
		double dist=CalcDistance(agent_status.vehicle_status().position(),agent_status.route_status().destination());
		double pulldist=CalcVelocity(agent_status.vehicle_status().velocity())/comfort_acc*2;
		if(dist<=pulldist)
			return true;
		return false;
	}

	double PIDControlVel(const interface::agent::AgentStatus& agent_status)
	{
		double p=0.1;
		double i=0.05;
		double d=0;
		double curr_vel=CalcVelocity(agent_status.vehicle_status().velocity());
		double last_vel=CalcVelocity(old_agent_status.vehicle_status().velocity());
		double pid=p*(desired_velocity-curr_vel)+i*(last_vel-curr_vel)+d*(veld+=curr_vel);
		if(pid>=1.0)
			pid=1;
		else if(pid<=-1.0)
			pid=-1;
		return pid;
	}

	double PIDControlAcc(const interface::agent::AgentStatus& agent_status)
	{
		double p=0.55;
		double i=0.2;
		double d=0;
		double curr_acc=GetAccX(agent_status.vehicle_status().acceleration_vcs());
		double last_acc=GetAccX(old_agent_status.vehicle_status().acceleration_vcs());
		double pid=p*(desired_acc-curr_acc)+i*(last_acc-curr_acc)+d*(accd+=curr_acc);
		if(pid>=1.0)
			pid=1;
		else if(pid<-1.0)
			pid=-1;
		return pid;
	}

	// Whether vehicle's current position reaches the destination
	bool start_pulling_ = false;
	// Whether vehicle's current velocity reaches 5 m/s
	bool velocity_reached_threshold_ = false;

	bool need_acc_ = true;

	bool finish=false;

	string fileprefix="/home/hongfz/Documents/Learn/AutomonousDrivingHW/";
	
	interface::route::Route route;
	
	int nextdest;
	int totaldests;

	double desired_velocity;
	double desired_acc;

	double veld;
	double accd;

	interface::agent::AgentStatus old_agent_status;

	double comfort_acc=1.5;
	double TH=1e-3;
};
}