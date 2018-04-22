#ifndef _FINDROUTE_H_
#define _FINDROUTE_H_

#include<iostream>
#include<string>
#include<vector>
#include<stack>
#include<queue>
#include"common/proto/route.pb.h"
#include"common/proto/map.pb.h"
#include"common/utils/file/file.h"

#include "glog/logging.h"

using namespace std;

int GetId(const string& strid)
{
	string sub=strid.substr(1,strid.size());
	return stoi(sub);
}

//Decide which lane the starting and ending point in
int FindLaneId(const interface::geometry::Point2D& testpoint, const interface::map::Map& mapdata)
{
	cout<<testpoint.x()<<" "<<testpoint.y()<<endl;
	for(const auto& lane : mapdata.lane())
	{
		interface::geometry::Point3D l0,l1,r0,r1;
		int lsize=lane.left_bound().boundary().point_size();
		int rsize=lane.right_bound().boundary().point_size();
		l0=lane.left_bound().boundary().point(0);
		l1=lane.left_bound().boundary().point(lsize-1);
		r0=lane.right_bound().boundary().point(0);
		r1=lane.right_bound().boundary().point(rsize-1);
		// vector<interface::geometry::Point3D> pvec;
		// pvec.push_back(l0);
		// pvec.push_back(l1);
		// pvec.push_back(r0);
		// pvec.push_back(r1);
		// int nCross=0;
		// for(int i=0;i<4;++i)
		// {
		// 	interface::geometry::Point3D p1,p2;
		// 	p1=pvec[i];
		// 	p2=pvec[(i+1)%4];
		// 	if(p1.y()==p2.y())
		// 		continue;
		// 	if(testpoint.y()<min(p1.y(),p2.y()))
		// 		continue;
		// 	if(testpoint.y()>max(p1.y(),p2.y()))
		// 		continue;
		// 	double xx=(testpoint.y()-p1.y())*(p2.x()-p1.x())/(p2.y()-p1.y())+p1.x();
		// 	if(xx>testpoint.x())
		// 		nCross++;
		// }
		// if(nCross%2==1)
		// {
		// 	return count;
		// }
		// ++count;

		//use simple square box to decide
		double xmin=min(min(l0.x(),l1.x()),min(r0.x(),r1.x()));
		double ymin=min(min(l0.y(),l1.y()),min(r0.y(),r1.y()));
		double xmax=max(max(l0.x(),l1.x()),max(r0.x(),r1.x()));
		double ymax=max(max(l0.y(),l1.y()),max(r0.y(),r1.y()));
		if(testpoint.x()>=xmin && testpoint.x()<=xmax && testpoint.y()>=ymin && testpoint.y()<=ymax)
			return GetId(lane.id().id());
	}
	return -1;
}

//use id(int) to find the corresponding lane
interface::map::Lane FindCorrectLane(int id,const interface::map::Map& map)
{
	for(const auto& lane : map.lane())
	{
		if(GetId(lane.id().id())==id)
			return lane;
	}
}

//inputfilename is the proto file containing starting and ending point
void FindRoute(const string& inputfilename, const string& outputfilename)
{
	//Read all the information
	interface::map::Map mapdata;

	//PLEASE USE THE RIGHT PATH
	CHECK(file::ReadFileToProto("/home/hongfz/Documents/Learn/PublicCourse/homework5/Results/Map/processed_map_proto.txt", &mapdata));
	interface::route::Route route;
	CHECK(file::ReadFileToProto(inputfilename,&route));
	interface::geometry::Point2D start_point=route.start_point(),end_point=route.end_point();
	int start_id=FindLaneId(start_point,mapdata);
	int end_id=FindLaneId(end_point,mapdata);
	if(start_id==-1 || end_id==-1)
	{
		cout<<"Point could not been find on the map"<<endl;
		return;
	}
	cout<<start_id<<" "<<end_id<<endl;
	interface::map::Lane start_lane=FindCorrectLane(start_id,mapdata);
	interface::map::Lane end_lane=FindCorrectLane(end_id,mapdata);
	int lanesize=mapdata.lane_size();
	vector<int> record;
	record.resize(lanesize+1,-1);
	queue<interface::map::Lane> ss;
	ss.push(start_lane);
	record[start_id]=0;
	bool found=false;

	//BFS
	while(!ss.empty())
	{
		//cout<<"inside while"<<endl;
		interface::map::Lane frontlane=ss.front();
		ss.pop();
		vector<int> next;
		for(const auto& id : frontlane.predecessor())
		{
		//	cout<<id.id()<<endl;
			next.push_back(GetId(id.id()));
		}
		for(const int& id : next)
		{
		//	cout<<id<<endl;
			if(record[id]!=-1)
				continue;
			const interface::map::Lane templane=FindCorrectLane(id,mapdata);
			ss.push(templane);
			record[id]=GetId(frontlane.id().id());
			if(id==end_id)
			{
				found=true;
				break;
			}
		}
		if(found)
			break;
	}

	//Reconstruct the whole route
	vector<int> routeid;
	routeid.push_back(end_id);
	while(1)
	{
		int last=routeid[routeid.size()-1];
		if(record[last]==0)
			break;
		routeid.push_back(record[last]);
	}

	//Add route points(lanes' central line points) to route
	for(int i=routeid.size()-1;i>=0;--i)
	{
		interface::map::Lane templane=FindCorrectLane(routeid[i],mapdata);
		int compflag=0;
		if(i==routeid.size()-1)
		{
			interface::geometry::Point3D flagpoint=start_lane.central_line().point(0);
			if(abs(flagpoint.x()-start_point.x())<abs(flagpoint.y()-start_point.y()))
			{
				if(flagpoint.y()<start_point.y())
					compflag=2;
				else
					compflag=-2;
			}
			else
			{
				if(flagpoint.x()<start_point.x())
					compflag=1;
				else
					compflag=-1;
			}
		}
		if(i==0)
		{
			int tempsize=end_lane.central_line().point_size();
			interface::geometry::Point3D flagpoint=end_lane.central_line().point(tempsize-1);
			if(abs(flagpoint.x()-end_point.x())<abs(flagpoint.y()-end_point.y()))
			{
				if(flagpoint.y()<end_point.y())
					compflag=2;
				else
					compflag=-2;
			}
			else
			{
				if(flagpoint.x()<end_point.x())
					compflag=1;
				else
					compflag=-1;
			}
		}
		// interface::geometry::Point2D nsp;
		// nsp.set_x(start_point.x());
		// nsp.set_y(start_point.y());
		// auto* newstartpoint=route.add_route_point();
		// newstartpoint->CopyFrom(nsp);
		for(const auto& point : templane.central_line().point())
		{
			if(i==routeid.size()-1)
			{
				if(compflag==1)
					if(point.x()<start_point.x())
						continue;
				if(compflag==-1)
					if(point.x()>start_point.x())
						continue;
				if(compflag==2)
					if(point.y()<start_point.y())
						continue;
				if(compflag==-2)
					if(point.y()>start_point.y())
						continue;
			}
			if(i==0)
			{
				if(compflag==1)
					if(point.x()<end_point.x())
						continue;
				if(compflag==-1)
					if(point.x()>end_point.x())
						continue;
				if(compflag==2)
					if(point.y()<end_point.y())
						continue;
				if(compflag==-2)
					if(point.y()>end_point.y())
						continue;
			}
			interface::geometry::Point2D p2d;
			p2d.set_x(point.x());
			p2d.set_y(point.y());
			auto* newroutepoint=route.add_route_point();
			newroutepoint->CopyFrom(p2d);
		}
		// nsp.set_x(end_point.x());
		// nsp.set_y(end_point.y());
		// auto* newendpoint=route.add_route_point();
		// newendpoint->CopyFrom(nsp);
		cout<<routeid[i]<<endl;
	}

	//Write to output file
	CHECK(file::WriteProtoToTextFile(route,outputfilename));
}


#endif //_FINDROUTE_H_