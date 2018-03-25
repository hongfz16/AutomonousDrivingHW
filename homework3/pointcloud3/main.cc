#include "LIDAR2World.h"
#include "pointcloud.h"
#include "Generate3DPC.h"
#include "sepGround.h"

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

void SingleFrameProc(string path)
{
	vector<Eigen::Vector3d> grounds;
	vector<Eigen::Vector3d> nongrounds;
	for(int i=0;i<200;++i)
	{
		string filename=path+to_string(i)+string(".txt");
		PointCloud pci=ReadPointCloudFromTextFile(filename);
		Eigen::Vector3d wpoint;
		
		double PI=3.141592653;
		vector<vector<Eigen::Vector3d> > sortedpoints;
		preprocess(pci.points,sortedpoints);
		vector<Eigen::Vector3d> ground,nonground;
		SeperateGroundPoints(sortedpoints,ground,nonground,PI/4,0.1,0.1);
		
		for(int j=0;j<ground.size();++j)
		{
			LIDAR2World(ground[j],wpoint,pci.rotation,pci.translation);
			grounds.push_back(wpoint);
		}
		for(int j=0;j<nonground.size();++j)
		{
			LIDAR2World(nonground[j],wpoint,pci.rotation,pci.translation);
			nongrounds.push_back(wpoint);
		}
	}

	double meanz=0;
	for(int i=0;i<grounds.size();++i)
	{
		meanz+=grounds[i][2];
	}
	meanz/=grounds.size();
	cout<<meanz<<endl;

	Mat groundmat=ReadPointCloud(grounds);
	Mat nongroundmat=ReadPointCloud(nongrounds);
	Generate3DPCSepGround(groundmat,nongroundmat);
}

void MultiFrameProc(string path,double TH)
{
	vector<Eigen::Vector3d> grounds;
	vector<Eigen::Vector3d> nongrounds;
	for(int i=0;i<200;++i)
	{
		string filename=path+to_string(i)+string(".txt");
		PointCloud pci=ReadPointCloudFromTextFile(filename);
		Eigen::Vector3d wpoint;

		for(int i=0;i<pci.points.size();++i)
		{
			LIDAR2World(pci.points[i],wpoint,pci.rotation,pci.translation);
			if(pci.points[i][2]<TH)
				grounds.push_back(wpoint);
			else
				nongrounds.push_back(wpoint);
		}
	}
	Mat groundmat=ReadPointCloud(grounds);
	Mat nongroundmat=ReadPointCloud(nongrounds);
	Generate3DPCSepGround(groundmat,nongroundmat);
}

int main()
{
	string path("/home/hongfz/Documents/Learn/AutomonousDrivingHW/pony_data/VelodyneDevice32c/");
	
	//The naive way: using threshold to seperate ground points
	MultiFrameProc(path,-0.8);

	//The algorithm used in homework2
	//SingleFrameProc(path);

	return 0;
}