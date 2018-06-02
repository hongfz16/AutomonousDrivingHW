#pragma once

#include<iostream>
#include<fstream>
#include<vector>
#include<cmath>
#include<string>
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

#include "FindRoute.h"
#include "GetPredSucc.h"

typedef interface::geometry::Point3D point3d;
typedef interface::geometry::Point2D point2d;
typedef interface::geometry::Vector3d vector3d;

namespace hongfz16
{
class SimpleVehicleAgent : public simulation::VehicleAgent {
public:
	explicit SimpleVehicleAgent(const std::string& name) : VehicleAgent(name) {}

	virtual void Initialize(const interface::agent::AgentStatus& agent_status) {

	  string originmap=fileprefix+"pnc/map/grid3/map_proto.txt";
	  string processedmap=fileprefix+"pnc/agents/hongfz16/mymap/new_map_proto.txt";
	  string vehicleparamstr=fileprefix+"common/data/vehicle_params/vehicle_params.txt";
	  //hongfz16::GetPredSucc(originmap,processedmap);
	  CHECK(file::ReadFileToProto(vehicleparamstr, &vehicle_params));

	  const interface::geometry::Point3D endp=agent_status.route_status().destination();
	  const interface::geometry::Vector3d startp=agent_status.vehicle_status().position();
	  hongfz16::FindRoute(processedmap,startp,endp,route);
	  nextdest=0;
	  totaldests=route.route_point_size();
	  old_agent_status.CopyFrom(agent_status);
	  veld=0;
	  accd=0;
	  steerd=0;
	  last_steer_command=pair<double,double>(0.,0.);
	  nearest1=0;
	  nearest2=1;
	  nearest1dist=CalcDist3n2(startp,route.route_point(0));
	  nearest2dist=CalcDist3n2(startp,route.route_point(1));
	  // err=0;
	  // for(int i=0;i<totaldests-1;++i)
	  // {
	  // 	//cout<<route.route_point(i).x()<<" \t "<<route.route_point(i).y()<<endl;
	  // 	//cout<<std::sqrt(math::Sqr(route.route_point(i+1).x()-route.route_point(i).x())+math::Sqr(route.route_point(i+1).y()-route.route_point(i).y()))<<endl;
	  // }
	  //string velocity_log_file=fileprefix+"homework6/result/velocity_log/velocity_log_config_3.txt";
	  //fout.open(velocity_log_file);
	}

	void clear_route()
	{
		route.clear_start_point();
		route.clear_end_point();
		route.clear_start_orientation();
		route.clear_route_point();
	}

	void new_route_init(const interface::agent::AgentStatus& agent_status) {
		point3d new_dest=agent_status.route_status().destination();
		vector3d new_start=agent_status.vehicle_status().position();
	}

	virtual interface::control::ControlCommand RunOneIteration(
	    const interface::agent::AgentStatus& agent_status) {
	  interface::control::ControlCommand command;

		PublishVariable("nextdest", to_string(nextdest));
	  //fout<<agent_status.vehicle_status().velocity().x()<<" "
	  //<<agent_status.vehicle_status().velocity().y()<<" "
	  //<<agent_status.vehicle_status().velocity().z()<<endl;

	  currerr=UpdatenGetErr(agent_status);
	  // cout<<currerr<<endl;

	  if(finish)
	  {
	  	return command;
	  }

	  if(nextdest+1<=totaldests && ReachNextDest(agent_status))
	  {
	  	nextdest+=1;
	  	//cout<<"nextdest"<<endl;
	  }

	  pair<double,double> tempcommand=PIDControlsteer(agent_status);
	  last_steer_command=tempcommand;
	  command.set_steering_angle(last_steer_command.first);
	  command.set_steering_rate(last_steer_command.second);
	  //cout<<last_steer_command.first*180./PI<<endl;
	  

	  if(!start_pulling_ && !velocity_reached_threshold_ && CalcVelocity(agent_status.vehicle_status().velocity())>=5.0)
	  {
	  	velocity_reached_threshold_=true;
	  	need_acc_=false;
	  	start_pulling_=false;
	  	desired_acc=0;
	  	desired_velocity=5.0;

	  	//cout<<"Reach 5m/s"<<endl;
	  }

	  if(!start_pulling_ && ReachPulling(agent_status))
	  {
	  	start_pulling_=true;
	  	need_acc_=false;
	  	velocity_reached_threshold_=false;
	  	desired_acc=-comfort_acc;

	  	//cout<<"Start pulling"<<endl;
	  }

	  if(velocity_reached_threshold_)
	  {
	  	double ratio=PIDControlVel(agent_status);
	  	//cout<<ratio<<endl;
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
	  	//cout<<ratio<<endl;
	  	if(ratio<0)
	  		command.set_brake_ratio(-ratio);
	  	else
	  		command.set_throttle_ratio(ratio);
	  }

	  if(need_acc_)
	  {
	  	desired_acc=comfort_acc;
	  	double ratio=PIDControlAcc(agent_status);
	  	//cout<<ratio<<endl;
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

	double UpdatenGetErr(const interface::agent::AgentStatus& agent_status)
	{
		for(int i=nearest2+1;i<route.route_point_size();++i)
		{
			double dist=CalcDist3n2(agent_status.vehicle_status().position(),route.route_point(i));
			if(dist<nearest2)
			{
				nearest1=nearest2;
				nearest1dist=nearest2dist;
				nearest2=i;
				nearest2dist=dist;
				break;
			}
		}
		double x0=agent_status.vehicle_status().position().x();
		double y0=agent_status.vehicle_status().position().y();
		double x1=route.route_point(nearest1).x();
		double y1=route.route_point(nearest1).y();
		double x2=route.route_point(nearest2).x();
		double y2=route.route_point(nearest2).y();
		double lerr=(((y1-y2)*x0+(x2-x1)*y0+x1*y2-x2*y1)/std::sqrt(math::Sqr(y1-y2)+math::Sqr(x2-x1)));
		return lerr;
	}

	bool ReachNextDest(const interface::agent::AgentStatus& agent_status)
	{
		double dist=CalcDist3n2(agent_status.vehicle_status().position(),route.route_point(nextdest));
		//cout<<dist<<endl;
		//dist=abs(agent_status.vehicle_status().position().x()-route.route_point(nextdest).x());
		//cout<<agent_status.vehicle_status().position().y()-route.route_point(nextdest).y()<<endl;
		//cout<<dist<<endl;
		if(dist<1)
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

	double CalcRho(double x1,double x2,double y1,double y2)
	{
		return 2*abs(x1*y2-x2*y1)/std::sqrt((x1*x1+y1*y1)*(x2*x2+y2*y2)*((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
	}

	double CalcDist2D(double x1,double y1)
	{
		return std::sqrt(math::Sqr(x1)+math::Sqr(y1));
	}

	double CalcAngle(double x1,double x2,double y1,double y2)
	{
		return std::acos((x1*x2+y1*y2)/(CalcDist2D(x1,y1)*CalcDist2D(x2,y2)));
	}

	pair<double,double> PIDControlsteer(const interface::agent::AgentStatus& agent_status)
	{
		if(nextdest>=totaldests-1)
			return pair<double,double>(0,0);
		double p=10;
		double i=2;
		double d=0;
		
		// double p=my_agent_pid[0];
		// double i=my_agent_pid[1];
		// double d=my_agent_pid[2];

		//->>>> Using angle as control parameter
		interface::geometry::Vector3d velocity;
		velocity.CopyFrom(agent_status.vehicle_status().velocity());
		interface::geometry::Point2D targetvec;
		targetvec.set_x(route.route_point(nextdest).x()-agent_status.vehicle_status().position().x());
		targetvec.set_y(route.route_point(nextdest).y()-agent_status.vehicle_status().position().y());
		double delta=CalcAngle(velocity.x(),targetvec.x(),velocity.y(),targetvec.y());

		//cout<<delta<<endl;

		//double delta=currerr;

		interface::geometry::Vector3d pa,po;
		interface::geometry::Point3D pb;
		po.CopyFrom(agent_status.vehicle_status().position());
		pa.set_x(route.route_point(nextdest).x());
		pa.set_y(route.route_point(nextdest).y());
		pa.set_z(0);
		pb.set_x(route.route_point(nextdest+1).x());
		pb.set_y(route.route_point(nextdest+1).y());
		pb.set_z(0);
		double ax=agent_status.vehicle_status().velocity().x();
		double ay=agent_status.vehicle_status().velocity().y();
		double bx=pa.x()-po.x();
		double by=pa.y()-po.y();
		double cross_result=ax*by-ay*bx;
		if(cross_result<0)
			delta=-delta;

		//cout<<last_steer_command.first<<endl;

		if(isnan(last_steer_delta))
			last_steer_delta=0;
		if(isnan(steerd))
			steerd=0;

		double angle=p*delta+i*(last_steer_delta-delta)+d*(steerd+=delta);
		
		last_steer_delta=delta;
		//cout<<angle<<endl;
		//cout<<angle*180/PI<<endl;
		return pair<double,double>(angle,0);
	}

	pair<double,double> Controlsteer(const interface::agent::AgentStatus& agent_status)
	{
		if(nextdest>=totaldests-1)
			return last_steer_command;
		interface::geometry::Vector3d pa,po;
		interface::geometry::Point3D pb;
		po.CopyFrom(agent_status.vehicle_status().position());
		pa.set_x(route.route_point(nextdest).x());
		pa.set_y(route.route_point(nextdest).y());
		pa.set_z(0);
		pb.set_x(route.route_point(nextdest+1).x());
		pb.set_y(route.route_point(nextdest+1).y());
		pb.set_z(0);
		
		double ax=agent_status.vehicle_status().velocity().x();
		double ay=agent_status.vehicle_status().velocity().y();
		double bx=pa.x()-po.x();
		double by=pa.y()-po.y();
		double cross_result=ax*by-ay*bx;

		Eigen::Matrix3d rotation;
		Eigen::Quaterniond quater;
		quater.x()=agent_status.vehicle_status().orientation().x();
		quater.y()=agent_status.vehicle_status().orientation().y();
		quater.z()=agent_status.vehicle_status().orientation().z();
		quater.w()=agent_status.vehicle_status().orientation().w();
		rotation=quater.matrix();
		Eigen::Vector3d va,vb,vo;
		va(0)=pa.x();va(1)=pa.y();va(2)=pa.z();
		vb(0)=pb.x();vb(1)=pb.y();vb(2)=pb.z();
		vo(0)=po.x();vo(1)=po.y();vo(2)=po.z();
		va=rotation*va;
		vb=rotation*vb;
		vo=rotation*vo;
		pa.set_x(va(0)-vo(0));pa.set_y(va(1)-vo(1));pa.set_z(va(2)-vo(2));
		pb.set_x(vb(0)-vo(0));pb.set_y(vb(1)-vo(1));pb.set_z(vb(2)-vo(2));
		
		//cout<<"----"<<endl;
		//cout<<va-vo<<endl;
		//cout<<vb-vo<<endl;

		double I=1.5;
		double K=0.2;
		double rho=CalcRho(pa.x(),pb.x(),pa.y(),pb.y());
		
		//double rho2=2*(math::Sqr(pa.y())+math::Sqr(pb.y()))/(math::Sqr(pa.x())*pa.y()+math::Sqr(pb.x())*pb.y()+math::Sqr(pa.x())*pa.x()+math::Sqr(pb.x())*pb.x());
		//if(rho-rho2<TH)
		//	cout<<"True"<<endl;

		double angle=rho*I*(vehicle_params.wheelbase()+K*math::Sqr(CalcVelocity(agent_status.vehicle_status().velocity())));

		if(cross_result<0)
			angle=-angle;

		cout<<180/PI*angle<<endl;

		return pair<double,double>(angle,0.);
	}

	// Whether vehicle's current position reaches the destination
	bool start_pulling_ = false;
	// Whether vehicle's current velocity reaches 5 m/s
	bool velocity_reached_threshold_ = false;

	bool need_acc_ = true;

	bool finish=false;
	
	interface::route::Route route;
	int nearest1;
	int nearest2;
	double nearest1dist;
	double nearest2dist;
	double currerr;
	
	int nextdest;
	int totaldests;

	double desired_velocity;
	double desired_acc;

	double veld;
	double accd;
	double steerd;

	interface::agent::AgentStatus old_agent_status;

	double comfort_acc=1.5;
	double TH=1e-3;

	pair<double,double> last_steer_command;
	double last_steer_delta;

	double PI=3.141592653;

	interface::vehicle::VehicleParams vehicle_params;

	//ofstream fout;

	//PLEASE CHANGE THE FILEPREFIX TO YOUR WORKING PATH BEFORE YOU RUN THIS CODE
	string fileprefix="/home/hongfz/Documents/Learn/AutomonousDrivingHW/";
};
}