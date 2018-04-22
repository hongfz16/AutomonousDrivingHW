#ifndef _GETPREDSUCC_H_
#define _GETPREDSUCC_H_

#include<iostream>
#include<string>

#include "homework5/map/map_lib.h"
#include "common/proto/map.pb.h"
#include "common/utils/file/file.h"

#include "glog/logging.h"

using namespace std;

//Dicide two Point3D is equal
bool IsEqual(const interface::geometry::Point3D& lp,const interface::geometry::Point3D& rp)
{
	if(lp.x()==rp.x() && lp.y()==rp.y() && lp.z()==rp.z())
		return true;
	return false;
}

//Check two lanes' relationship
int CheckRelation(interface::map::Lane& ll,interface::map::Lane& rr)
//ll is rl's succ: 1
//rl is ll's succ: 2
//ll equals rl: 3
//no relation: 4
{
	interface::geometry::Point3D ll0,ll1,lr0,lr1,rl0,rl1,rr0,rr1;
	int llsize=ll.left_bound().boundary().point_size();
	int lrsize=ll.right_bound().boundary().point_size();
	int rlsize=rr.left_bound().boundary().point_size();
	int rrsize=rr.right_bound().boundary().point_size();
	ll0=ll.left_bound().boundary().point(0);
	ll1=ll.left_bound().boundary().point(llsize-1);
	lr0=ll.right_bound().boundary().point(0);
	lr1=ll.right_bound().boundary().point(lrsize-1);
	rl0=rr.left_bound().boundary().point(0);
	rl1=rr.left_bound().boundary().point(rlsize-1);
	rr0=rr.right_bound().boundary().point(0);
	rr1=rr.right_bound().boundary().point(rrsize-1);
	if(IsEqual(ll1,rl0) && IsEqual(lr1,rr0))
		return 1;
	if(IsEqual(rl1,ll0) && IsEqual(rr1,lr0))
		return 2;
	if(IsEqual(ll0,rl0) && IsEqual(ll1,rl1) && IsEqual(lr0,rr0) && IsEqual(lr1,rr1))
		return 3;
	return 4;
}

void GetPredSucc(const string& filename)
{
	interface::map::Map mapdata;

	//PLEASE CHANGE TO THE RIGHT PATH
	CHECK(file::ReadFileToProto("/home/hongfz/Documents/Learn/AutomonousDrivingHW/homework5/map/grid2/map_proto.txt", &mapdata));
	int mapsize=mapdata.lane_size();
	for(int i=0;i<mapsize;++i)
	{
		for(int j=i+1;j<mapsize;++j)
		{
			auto* llane=mapdata.mutable_lane(i);
			auto* rlane=mapdata.mutable_lane(j);
			int result=CheckRelation(*llane,*rlane);
			if(result==1)
			{
				// bool flag=false;
				// for(auto& succ : rlane.successor())
				// 	if(succ==llane.id())
				// 	{
				// 		flag=true;
				// 		break;
				// 	}
				// if(flag)
				// 	continue;
				auto* newsuccid=rlane->add_successor();
				newsuccid->CopyFrom(llane->id());
				auto* newpredid=llane->add_predecessor();
				newpredid->CopyFrom(rlane->id());
			}
			else if(result==2)
			{
				// bool flag=false;
				// for(auto& pred : rlane.predecessor())
				// 	if(pred==llane.id())
				// 	{
				// 		flag=true;
				// 		break;
				// 	}
				// if(flag)
				// 	continue;
				auto* newsuccid=llane->add_successor();
				newsuccid->CopyFrom(rlane->id());
				auto* newpredid=rlane->add_predecessor();
				newpredid->CopyFrom(llane->id());	
			}
		}
	}
	CHECK(file::WriteProtoToTextFile(mapdata,filename));
}

#endif //_GETPREDSUCC_H_