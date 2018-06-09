#include "homework4/obstacle_extract.h"
#include "perception/FeatureExtract/FeatureExtract.h"
#include "common/utils/file/file.h"
#include <fstream>
#include <string>
#include <vector>
#include <ctime>
#include <iostream>

using namespace std;

void rand_drop_out(const vector<Eigen::Vector3d>& points, vector<Eigen::Vector3d>& newp)
{
	srand((unsigned)time(nullptr));
	for(int i=0;i<points.size();++i)
	{
		double p=double(rand()%1000)/1000.0;
		if(p>0.9)
			continue;
		newp.push_back(points[i]);
	}
}

int main()
{
	string path_prefix("/home/hongfz/Documents/Learn/AutomonousDrivingHW/perception_project/");
	string sub_prefix1("20180103_nanian_103247/");
	string sub_prefix2("20171229_nanian_130255/");
	vector<string> sub;
	sub.push_back(sub_prefix1);
	sub.push_back(sub_prefix2);
	string label_path("label/VelodyneDevice32c/");
	string pointcloud_path("select/VelodyneDevice32c/");

	ofstream fout("/home/hongfz/Documents/Learn/AutomonousDrivingHW/perception/SVMTrainer/train.txt");
	// string obstacle_path("obstacle/VelodyneDevice32c/");

	// int carcount=0;
	// int othercount=0;

	for(int j=0;j<sub.size();++j)
	{
		int num=500;
		for(int i=0;i<num;++i)
		{
			cout<<i<<endl;
			PointCloud pointcloud=ReadPointCloudFromTextFile(
								  path_prefix+sub[j]+pointcloud_path+to_string(i)+string(".txt"));
			interface::object_labeling::ObjectLabels objectlabels;
			file::ReadFileToProto(path_prefix+sub[j]+label_path+to_string(i)+string(".label"),
								  &objectlabels);
			interface::perception::PerceptionObstacles obstacles;
			SingleFrameExtract(pointcloud,objectlabels,obstacles);

			for(const auto& obstacle : obstacles.obstacle())
			{
				vector<Eigen::Vector3d> points;
				if(obstacle.object_points_size()==0)
					continue;
				for(const auto& p : obstacle.object_points())
				{
					Eigen::Vector3d vp(p.x(),p.y(),p.z());
					points.push_back(vp);
				}
				string feature_str;
				if(obstacle.type()==interface::perception::ObjectType::CAR)
				{
					// carcount++;
					feature_str=ExtractTrainingFeature(points,1);
					fout<<feature_str<<endl;
				}
				else
				{
					// othercount++;
					for(int ii=0;ii<4;++ii)
					{
						vector<Eigen::Vector3d> newp;
						rand_drop_out(points,newp);
						feature_str=ExtractTrainingFeature(points,-1);
						fout<<feature_str<<endl;
					}
				}
			}
			// file::WriteProtoToTextFile(obstacles,path_prefix+obstacle_path+to_string(i)+string(".txt"));
		}
	}
	return 0;
}