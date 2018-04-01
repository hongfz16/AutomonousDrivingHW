#include "obstacle_extract.h"
#include "common/utils/file/file.h"

#include <iostream>
#include <vector>
#include <string>
#include <fstream>

using namespace std;

int main()
{
	string path_prefix("/home/hongfz/Documents/Learn/AutomonousDrivingHW/sample/");
	string label_path("label/VelodyneDevice32c/");
	string pointcloud_path("select/VelodyneDevice32c/");
	string obstacle_path("obstacle/VelodyneDevice32c/");

	int num=21;
	for(int i=0;i<num;++i)
	{
		PointCloud pointcloud=ReadPointCloudFromTextFile(
							  path_prefix+pointcloud_path+to_string(i)+string(".txt"));
		interface::object_labeling::ObjectLabels objectlabels;
		file::ReadFileToProto(path_prefix+label_path+to_string(i)+string(".label"),
							  &objectlabels);
		interface::perception::PerceptionObstacles obstacles;
		SingleFrameExtract(pointcloud,objectlabels,obstacles);
		// ofstream fout(path_prefix+obstacle_path+to_string(i)+string(".bin"),
		// 			  ios::out | ios::trunc | ios::binary);
		// if(!obstacles.SerializeToOstream(&fout))
		// {
		// 	cerr<<"Fail to serialize"<<endl;
		// 	break;
		// }
		// fout.close();
		file::WriteProtoToTextFile(obstacles,path_prefix+obstacle_path+to_string(i)+string(".txt"));
	}
	return 0;
}