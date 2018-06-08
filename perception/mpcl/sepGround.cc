#include "sepGround.h"

Vector3d LIDARPOINT(0,0,CAR_HEIGHT);

#define CV_PI 3.141592653

inline bool doubleeq(double ld,double rd)
{
	double temp=ld-rd;
	if(temp<0)
		temp=-temp;
	if(temp<1e-3)
		return true;
	return false;
}

inline bool cmp(Vector3d& ll,Vector3d& rr)
{
	return ll(1)/ll(0) < rr(1)/rr(0);
}

inline bool anglecmp(Vector3d& ll,Vector3d& rr)
{
	double lldist=sqrt(pow(ll(0),2)+pow(ll(1),2));
	double rrdist=sqrt(pow(rr(0),2)+pow(rr(1),2));
	return ll(2)/(lldist-CAR_HEIGHT) < rr(2)/(rrdist-CAR_HEIGHT);
}

void preprocess(vector<Vector3d>& points,vector<vector<Vector3d> >& sortedpoints)
{
	sort(points.begin(),points.end(),cmp);
	vector<vector<Vector3d> > classedpoints;
	vector<Vector3d> tempvv;
	tempvv.push_back(points[0]);
	classedpoints.push_back(tempvv);
	for(size_t i=1;i<points.size();++i)
	{
		bool flag=true;
		for(size_t j=classedpoints.size();j>0;--j)
		{
			Vector3d temp=points[i];
			Vector3d classedtemp=classedpoints[j-1][0];
			if(doubleeq(atan(temp(1)/temp(0)),atan(classedtemp(1)/classedtemp(0))))
			{
				classedpoints[j-1].push_back(points[i]);
				flag=false;
				break;
			}
		}
		if(flag)
		{
			vector<Vector3d> tempv;
			tempv.push_back(points[i]);
			classedpoints.push_back(tempv);
		}
	}
	for(size_t i=0;i<classedpoints.size();++i)
	{
		vector<Vector3d> temp1;
		vector<Vector3d> temp2;
		temp1.push_back(Vector3d(0,0,0));
		temp2.push_back(Vector3d(0,0,0));
		for(size_t j=0;j<classedpoints[i].size();++j)
		{
			if(classedpoints[i][j](0)<0)
				temp1.push_back(classedpoints[i][j]);
			else
				temp2.push_back(classedpoints[i][j]);
		}
		sort(temp1.begin(),temp1.end(),anglecmp);
		sort(temp2.begin(),temp2.end(),anglecmp);
		sortedpoints.push_back(temp1);
		sortedpoints.push_back(temp2);
	}
}

inline double xyzdist(Vector3d& p1,Vector3d& p2)
{
	return sqrt(pow(p1(0)-p2(0),2)+pow(p1(1)-p2(1),2)+pow(p1(2)-p2(2),2));
}

inline double xydist(Vector3d& p1,Vector3d& p2)
{
	return sqrt(pow(p1(0)-p2(0),2)+pow(p1(1)-p2(1),2));
}

inline double angle(Vector3d& p1,Vector3d& p2)
{
	return atan((p2(2)-p1(2))/xydist(p1,p2));
}

inline double zdist(Vector3d& p1,Vector3d& p2)
{
	return p1(2)-p2(2);
}

void SeperateGroundPoints(vector<vector<Vector3d> >& sortedpoints,vector<Vector3d>& ground,vector<Vector3d>& nonground,double alphamax,double missmax,double hmin)
{
	for(int i=0;i<sortedpoints.size();++i)
	{
		vector<Vector3d>& spi=sortedpoints[i];
		int state=0;
		vector<pair<Vector3d,int> > startpoints;
		vector<pair<Vector3d,int> > thresholdpoints;
		startpoints.push_back(pair<Vector3d,int>(spi[0],0));
		for(int j=0;j<spi.size()-1;++j)
		{
			switch(state)
			{
				case 0:
				if(angle(spi[j],spi[j+1])>alphamax)
					state=1;
				if(zdist(spi[j],spi[j+1])>missmax)
					state=1;
				if(xyzdist(spi[j],LIDARPOINT)>xyzdist(spi[j+1],LIDARPOINT))
					state=1;
				break;
				case 2:
				if(zdist(spi[j],thresholdpoints[thresholdpoints.size()-1].first)<=hmin)
					state=3;
				else
					nonground.push_back(spi[j]);
				break;
			}
			if(state==1)
			{
				for(int k=j;k>=startpoints[startpoints.size()-1].second;--k)
				{
					if(spi[k](0)==0 && spi[k](1)==0 && spi[k](2)==0)
						continue;
					ground.push_back(spi[k]);
				}
				thresholdpoints.push_back(pair<Vector3d,int>(spi[j],j));
				state=2;
			}
			if(state==3)
			{
				startpoints.push_back(pair<Vector3d,int>(spi[j],j));
				state=0;
			}
		}
	}
}

// void detect_ground_points(vector<Eigen::Vector3d>& points){
//   sort(points.begin(), points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b){
//     double d1 = sqrt(a(0)*a(0)+a(1)*a(1));
//     double d2 = sqrt(b(0)*b(0)+b(1)*b(1));
//     return atan2(a(2)-2, d1) < atan2(b(2)-2, d2);
//   });

//   int N = points.size();
//   Eigen::Vector3d start_point(Eigen::Vector3d::Zero());
//   double alpha = CV_PI/4;
//   double h_min = 0.4;
//   vector<Eigen::Vector3d> ground;
//   ground.clear();

//   for (int i = 0; i < N; ++i){
//     Eigen::Vector3d pre_point(start_point);
//     while (i < N){
//       double d1 = sqrt(pre_point(0)*pre_point(0)+pre_point(1)*pre_point(1));
//       double d2 = sqrt(points[i](0)*points[i](0)+points[i](1)*points[i](1));
//       double D1 = sqrt(pre_point(0)*pre_point(0)+pre_point(1)*pre_point(1)+pre_point(2)*pre_point(2));
//       double D2 = sqrt(points[i](0)*points[i](0)+points[i](1)*points[i](1)+points[i](2)*points[i](2));
//       if ( D1 > D2 || atan2(points[i](2)-pre_point(2) ,d2-d1) > alpha || points[i](2)-pre_point(2) > h_min ){
//         int j = i+1;
//         while (j < N && points[j](2)-pre_point(2) > h_min)
//           ++j;
//         i = j;
//         if (j < N)
//           start_point = points[j];
//         break;
//       }
//       ground.push_back(points[i]);
//       pre_point = points[i++];
//     }
//   }
  
//   points.clear();
//   while (!ground.empty()){
//     points.push_back(ground.back());
//     ground.pop_back();
//   }
  
//   for (int i = 0; i < points.size(); ++i){
//     for (const Eigen::Vector3d& p: ground){
//       if (p == points[i]){
//         points.erase(points.begin()+i);
//         i--;
//         break;
//       }
//     }
//   }
// }

// void separate_ground_points(PointCloud& pointcloud){
//   //showPointCloud(pointcloud);
//   vector<Eigen::Vector3d>& points = pointcloud.points;
//   int N = points.size();
//   sort(points.begin(), points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b){
//     return atan2(a(1),a(0)) < atan2(b(1), b(0));
//   });
//   //ofstream out("sorted.txt");
//   const double eps = CV_PI/1000;
//   double low_range = -CV_PI;
  
//   vector<Eigen::Vector3d> points_; points_.clear();
//   for (int i = 0, j = 0; i < 2000; ++i, low_range += eps){
//     vector<Eigen::Vector3d> line_points;
//     line_points.clear();
//     while (j < N){
//       double angle = atan2(points[j](1), points[j](0));
//       if (angle >= low_range && angle < low_range+eps){
//         line_points.push_back(points[j++]);
//         continue;
//       }
//       break;
//     }
//     detect_ground_points(line_points);
//     points_.insert(points_.begin(), line_points.begin(), line_points.end());
//   }
//   //out.close();

//   points.clear();
//   //cout << points.size() << endl;
//   points.insert(points.begin(), points_.begin(), points_.end());
//   //showPointCloud(pointcloud);
// }

void remove_ground_points(PointCloud& pc)
{
  double PI=3.141592653;
  vector<vector<Vector3d> > sortedpoints;
  preprocess(pc.points,sortedpoints);
  vector<Vector3d> ground,nonground;
  SeperateGroundPoints(sortedpoints,ground,nonground,PI/4,0.1,0.1);
  pc.points=nonground;
}