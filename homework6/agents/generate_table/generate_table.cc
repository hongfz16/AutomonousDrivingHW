#include<iostream>
#include<fstream>
#include <vector>

#include "common/proto/agent_status.pb.h"
#include "common/proto/map.pb.h"

#include "Eigen/Core"
#include "Eigen/Dense"

#include "common/utils/math/vec2d.h"
#include "common/utils/math/vec3d.h"
#include "common/proto/control.pb.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"

#include "homework6/simulation/dynamic_lib/lib_vehicle_status_model_solver.h"

namespace hongfz16{
	void init_agent_status(interface::agent::AgentStatus& agent_status);
	void set_v(interface::agent::AgentStatus& agent_status,double v);
	void set_a(interface::agent::AgentStatus& agent_status,double a);
}

using namespace std;

void hongfz16::init_agent_status(interface::agent::AgentStatus& agent_status)
{
  agent_status.mutable_vehicle_status()->mutable_position()->set_x(0);
  agent_status.mutable_vehicle_status()->mutable_position()->set_y(0);
  agent_status.mutable_vehicle_status()->mutable_position()->set_z(0);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_x(1);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_y(0);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_z(0);
  agent_status.mutable_vehicle_status()->mutable_orientation()->set_w(0);
  agent_status.mutable_route_status()->mutable_destination()->set_x(10000);
  agent_status.mutable_route_status()->mutable_destination()->set_y(0);
  agent_status.mutable_route_status()->mutable_destination()->set_z(0);
  agent_status.mutable_simulation_status()->set_is_alive(true);
}

void hongfz16::set_v(interface::agent::AgentStatus& agent_status,double v)
{
	agent_status.mutable_vehicle_status()->mutable_velocity()->set_x(v);
	agent_status.mutable_vehicle_status()->mutable_velocity()->set_y(0);
	agent_status.mutable_vehicle_status()->mutable_velocity()->set_z(0);
}

void hongfz16::set_a(interface::agent::AgentStatus& agent_status,double a)
{
	agent_status.mutable_vehicle_status()->mutable_acceleration_vcs()->set_x(a);
	agent_status.mutable_vehicle_status()->mutable_acceleration_vcs()->set_y(0);
	agent_status.mutable_vehicle_status()->mutable_acceleration_vcs()->set_z(0);
}

int main()
{
	interface::agent::AgentStatus agent_status;
	hongfz16::init_agent_status(agent_status);
	
	interface::vehicle::VehicleParams vehicle_params_;
	CHECK(file::ReadTextFileToProto("/home/hongfz/Documents/Learn/AutomonousDrivingHW/common/data/vehicle_params/vehicle_params.txt", &vehicle_params_));
	
	//std::unique_ptr<vehicle_status_model::VehicleStatusModelSolver> solver =
  //    vehicle_status_model::CreateVehicleStatusModelSolver(vehicle_params_);
  //solver->Initialize(0, agent_status.vehicle_status());

  double ratio=0;
  double cv=0;
  double ca=-10.0;

  ofstream fout1("/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework6/agents/generate_table/throttle_table.txt");
  ofstream fout2("/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework6/agents/generate_table/brake_table.txt");
  
  for(int i=0;i<10;++i)
  {
  	ratio+=0.1;
  	//ratio=1;
  	interface::control::ControlCommand command1,command2;
  	command1.set_throttle_ratio(ratio);
  	command1.set_brake_ratio(0);
  	//command2.set_brake_ratio(ratio);
  	cv=0;
  	for(int j=0;j<30;++j)
  	{
  		cv+=0.1;
			hongfz16::init_agent_status(agent_status);
			hongfz16::set_v(agent_status,cv);
			//hongfz16::set_a(agent_status,ca);
			std::unique_ptr<vehicle_status_model::VehicleStatusModelSolver> solver =
			    vehicle_status_model::CreateVehicleStatusModelSolver(vehicle_params_);
			solver->Initialize(0,agent_status.vehicle_status());
			interface::agent::VehicleStatus vs=solver->UpdateVehicleStatus(0.01,command1);
			vs=solver->UpdateVehicleStatus(0.02,command1);
			vs=solver->UpdateVehicleStatus(0.03,command1);
			vs=solver->UpdateVehicleStatus(0.04,command1);
			vs=solver->UpdateVehicleStatus(0.05,command1);
			vs=solver->UpdateVehicleStatus(0.06,command1);
			vs=solver->UpdateVehicleStatus(0.07,command1);
			vs=solver->UpdateVehicleStatus(0.08,command1);
			vs=solver->UpdateVehicleStatus(0.09,command1);
			vs=solver->UpdateVehicleStatus(0.10,command1);
			fout1<<cv<<"\t"<<ratio<<"\t"<<vs.acceleration_vcs().x()<<endl;
			//solver->Initialize(0,agent_status.vehicle_status());
			//vs=solver->UpdateVehicleStatus(0,command2);
			//fout2<<cv<<" "<<ca<<" "<<ratio<<" "<<vs.acceleration_vcs().x()<<endl;
		}
  }

  fout1.close();
  fout2.close();

  return 0;
}