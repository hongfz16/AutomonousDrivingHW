#include "hongfz16_agent.h"

namespace hongfz16
{

inline bool is_zero(double n)
{
	if(n<1e-3)
		return true;
	else
		return false;
}

virtual void hongfz16::SimpleVehicleAgent::Initialize(const interface::agent::AgentStatus& agent_status)
{
	init_param()
}

virtual interface::control::ControlCommand hongfz16::SimpleVehicleAgent::RunOneIteration(
	const interface::agent::AgentStatus& agent_status)
{
	interface::control::ControlCommand command;
	SpeedInfo sinfo=get_curr_speed(agent_status);
	if(is_reach_dest(agent_status) && is_zero(sinfo.speed))
	{
		car_info.ready=true;
	}
	if(agent_status.route_status().is_new_request() && car_info.ready)
	{
		my_init();
		ready=false;
	}
	if(ready)
		return command;
	command=control_vehicle(agent_status);
	return command;
}

inline double hongfz16::SimpleVehicleAgent::calc_dist(const Point2d& p1, const Point2d& p2)
{
	return sqrt(pow(p1.x()-p2.x(),2)+pow(p1.y()-p2.y(),2));
}

inline double hongfz16::SimpleVehicleAgent::calc_theta(const Point2d& p1, const Point2d& p2)
{
	return atan2(p2.y()-p1.y(),p2.x()-p1.x());
}

inline Point2d hongfz16::SimpleVehicleAgent::vec3d_to_p2d(const Vector3d& v)
{
	Point2d p;
	p.set_x(v.x());
	p.set_y(v.y());
	return p;
}

inline void hongfz16::SimpleVehicleAgent::clear_route()
{
	car_info.curr_route.routes.clear();
}

inline void hongfz16::SimpleVehicleAgent::new_route_init(const interface::agent::AgentStatus& agent_status)
{
	Point3d new_dest=agent_status.route_status().destination();
	Vector3d new_start=agent_status.vehicle_status().position();
	
	car_info.dest.set_x(new_dest.x());
	car_info.dest.set_y(new_dest.y());
	car_info.start.set_x(new_start.x());
	car_info.start.set_y(new_start.y());

  clear_route();
  interface::route::Route route;
  string processedmap=fileprefix+"pnc/agents/hongfz16/mymap/new_map_proto.txt";
  hongfz16::FindRoute(processedmap,new_start,new_dest,route);
  
  for(const Point2d& p : route.route_point())
  {
  	car_info.curr_route.routes.push_back(pair<Point2d,double>(p,0));
  }
}

inline void hongfz16::SimpleVehicleAgent::init_param(const interface::agent::AgentStatus& agent_status)
{
	car_info.refer_point_id=0;
	car_info.last_err.theta_err=0;
	car_info.last_err.ct_err=0;
	car_info.int_err.theta_err=0;
	car_info.int_err.ct_err=0;
}

inline void hongfz16::SimpleVehicleAgent::my_init(const interface::agent::AgentStatus& agent_status)
{
	new_route_init();
	init_param();
}

inline bool hongfz16::SimpleVehicleAgent::is_reach_dest(const interface::agent::AgentStatus& agent_status)
{
	if(car_info.refer_point_id==car_info.curr_route.routes.size()-1)
		return true;
	else
		return false;
}

inline SpeedInfo hongfz16::SimpleVehicleAgent::get_curr_speed(const interface::agent::AgentStatus& agent_status)
{
	SpeedInfo sinfo;
	Vector3d speed3d=agent_status.vehicle_status().velocity();
	sinfo.speed=sqrt(pow(speed3d.x(),2)+pow(speed3d.y(),2));
	sinfo.theta=atan2(speed3d.y(),speed3d.x());
	return sinfo;
}

inline Command hongfz16::SimpleVehicleAgent::control_vehicle(const interface::agent::AgentStatus& agent_status)
{
	update_refer_point(agent_status);
	Command c;
	double steer_angle=path_tracking(agent_status);
	double speed_control=speed_control(agent_status);
	c.set_steering_angle(steer_angle);
	c.set_steering_rate(0);
	if(speed_control>0)
		set_throttle_ratio(speed_control);
	else
		set_brake_ratio(abs(speed_control));
	return c;
}

double hongfz16::SimpleVehicleAgent::path_tracking(const interface::agent::AgentStatus& agent_status)
{
	SpeedInfo sinfo=get_curr_speed(agent_status);
	Err curr_err=get_err(agent_status,sinfo);
	car_info.int_err.theta_err+=curr_err.theta_err;
	car_info.int_err.ct_err+=curr_err.ct_err;
	double delta_theta_err=0;
	double delta_ct_err=0;
	if(car_info.last_err.theta_err==0)
		delta_theta_err=delta_ct_err=0;
	else
	{
		delta_theta_err=curr_err.theta_err-car_info.last_err.theta_err;
		delta_ct_err=curr_err.ct_err-car_info.last_err.ct_err;
	}
	car_info.last_err.theta_err=curr_err.theta_err;
	car_info.last_err.ct_err=curr_err.ct_err;
	return car_param.kp * (car_param.theta_err_pid[0] * curr_err.theta_err
		+ car_param.theta_err_pid[1] * car_info.int_err.theta_err
		+ car_param.theta_err_pid[2] * delta_theta_err
		+ car_param.ct_err_pid[0] * curr_err.ct_err
		+ car_param.ct_err_pid[1] * car_info.int_err.ct_err
		+ car_param.ct_err_pid[2] * delta_ct_err);
}

double hongfz16::SimpleVehicleAgent::speed_control(const interface::agent::AgentStatus& agent_status)
{

}

double hongfz16::SimpleVehicleAgent::get_preview_length(double cspeed)
{
	if(cspeed<car_param.preview_length_th_min)
		return car_param.preview_length_min;
	else if(cspeed>car_param.preview_length_th_max)
		return car_param.preview_length_max;
	else
		return (cspeed-car_param.preview_length_th_min)*(car_param.preview_length_max-car_param.preview_length_min)/
						(car_param.preview_length_th_max-car_param.preview_length_th_min)+car_param.preview_length_min;	
}

Err hongfz16::SimpleVehicleAgent::get_err(const interface::agent::AgentStatus& agent_status, const SpeedInfo& sinfo)
{
	Err err;
	Point2d preview_point;
	double preview_length=get_preview_length(sinfo.speed);
	preview_point.set_x(agent_status.vehicle_status().position().x()+preview_length*cos(sinfo.theta));
	preview_point.set_y(agent_status.vehicle_status().position().y()+preview_length*sin(sinfo.theta));
	double min_dist=1e8;
	int interest_point_id=0;
	int badcnt=0;
	for(int i=car_info.refer_point_id;i<car_info.curr_route.routes.size();++i)
	{
		double tempdist=calc_dist(car_info.curr_route.routes[i].first,preview_point);
		if(tempdist<min_dist)
		{
			min_dist=tempdist;
			interest_point_id=i;
		}
		else
		{
			badcnt++;
		}
		if(badcnt>5)
			break;
	}
	double theta=calc_theta(car_info.curr_route.routes[interest_point_id].first,car_info.curr_route.routes[interest_point_id+1].first);
	err.theta_err=theta-sinfo.theta;
	err.ct_err=min_dist;
	return err;
}

inline void update_refer_point(const interface::AgentStatus& agent_status)
{
	Point2d position;
	position.set_x(agent_status.vehicle_status().position().x());
	position.set_y(agent_status.vehicle_status().position().y());
	double currdist=calc_dist(car_info.curr_route.routes[car_info.refer_point_id].first,position);
	for(int i=car_info.refer_point_id+1;i<car_info.curr_route.routes.size();++i)
	{
		double tempdist=calc_dist(car_info.curr_route.routes[i].first,position);
		if(tempdist<currdist)
		{
			currdist=tempdist;
			car_info.refer_point_id=i;
		}
		else
		{
			break;
		}
	}
}

}