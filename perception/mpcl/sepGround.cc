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

void remove_ground_points(PointCloud& pc)
{
  double PI=3.141592653;
  vector<vector<Vector3d> > sortedpoints;
  preprocess(pc.points,sortedpoints);
  vector<Vector3d> ground,nonground;
  SeperateGroundPoints(sortedpoints,ground,nonground,PI/4,0.1,0.1);
  pc.points=nonground;
}