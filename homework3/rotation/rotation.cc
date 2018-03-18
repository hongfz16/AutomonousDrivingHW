// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework3/rotation/rotation.h"

using namespace std;

namespace homework3 {

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation) {
	Eigen::Vector3d rev;
	rev.x()=atan2(-rotation(0,1),rotation(0,0));
	rev.y()=atan2(rotation(0,2),sqrt(pow(rotation(0,0),2)+pow(rotation(0,1),2)));
	rev.z()=atan2(-rotation(1,2),rotation(2,2));
  	//rev=rotation.eulerAngles(0,1,2);
  	cout<<rev<<endl;
  	return rev;
}

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation) {
	Eigen::AngleAxisd rea;
	rea.angle()=acos((rotation(0,0)+rotation(1,1)+rotation(2,2)-1)/2);
	double s=sqrt(pow(rotation(2,1)-rotation(1,2),2)+pow(rotation(0,2)-rotation(2,0),2)+pow(rotation(1,0)-rotation(0,1),2));
	rea.axis().x()=(rotation(2,1)-rotation(1,2))/s;
	rea.axis().y()=(rotation(0,2)-rotation(2,0))/s;
	rea.axis().z()=(rotation(1,0)-rotation(0,1))/s;
  	//cout<<rea.axis()<<endl;
  	return rea;
}
}  // namespace homework3
