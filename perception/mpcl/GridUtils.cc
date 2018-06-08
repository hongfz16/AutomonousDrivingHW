#include "perception/mpcl/GridUtils.h"

void FillGrid(vector<vector<vector<Eigen::Vector3d> > >& grid_points, 
			vector<Eigen::Vector3d>& points, double range)
{
    double grid_size=2.0*range/double(grid_points.size());
    for(int i=0;i<points.size();++i)
    {
        if(abs(points[i](0))>range || abs(points[i](1))>range)
            continue;
        int j=(range+points[i](0))/grid_size;
        int k=(range+points[i](1))/grid_size;
        grid_points[j][k].push_back(points[i]);
    }
}

bool cmp_height(Eigen::Vector3d& lp,Eigen::Vector3d& rp)
{
	return lp(2)<rp(2);
}

void remove_out_range(vector<Eigen::Vector3d>& points, int th)
{
	while(points.size()>1)
	{
		Eigen::Vector3d q_back = points.back();
		int index = points.size()-1;
		while (q_back(2) - points[index](2) < 1)
			--index;

		if (points.size()-index < th){
			points.pop_back();
			continue;
		}
		break;
	}
}

void RemoveNoisePoints(vector<vector<vector<Eigen::Vector3d> > >& grid_points)
{
    for(int i=0;i<grid_points.size();++i)
    {
        for(int j=0;j<grid_points[i].size();++j)
        {
			if(grid_points[i][j].empty())
				continue;
			sort(grid_points[i][j].begin(),grid_points[i][j].end(),cmp_height);
			remove_out_range(grid_points[i][j],15);
			reverse(grid_points[i][j].begin(),grid_points[i][j].end());
			remove_out_range(grid_points[i][j],15);
        }
    }
}

void MarkGrid(const vector<vector<vector<Eigen::Vector3d> > >& grid_points, 
            vector<vector<double> >& max_height, 
            vector<vector<double> >& min_height, 
            vector<vector<int> >& mark,
			double TH)
{
	for(int i=0;i<grid_points.size();++i)
	{
		for(int j=0;j<grid_points[i].size();++j)
		{
			int size=grid_points[i][j].size();
			if(size==0)
			{
				max_height[i][j]=min_height[i][j]=0;
				continue;
			}
			max_height[i][j]=grid_points[i][j][0](2);
			min_height[i][j]=grid_points[i][j][size-1](2);
			if(max_height[i][j]-min_height[i][j]>TH)
			{
				mark[i][j]=1;
			}
		}
	}
	for(int i=0;i<mark.size();++i)
	{
		for(int j=0;j<mark[i].size();++j)
		{
			if(mark[i][j]==0)
				continue;
			bool safe=false;
			for(int ii=max(0,i-3);ii<=i+3 && ii<mark.size();++ii)
			{
				for(int jj=max(0,j-3);jj<=j+3 && jj<mark.size();++jj)
				{
					if(safe)
						break;
					if((ii!=i || jj!=j) && mark[ii][jj]!=0)
					{
						safe=true;
						break;
					}
				}
			}
			if(!safe)
			{
				mark[i][j]=0;
			}
		}
	}
	for(int i=0;i<mark.size();++i)
	{
		for(int j=0;j<mark[i].size();++j)
		{
			if(mark[i][j]==1)
			{
			int dx[8]={0,1,0,-1,1,1,-1,-1};
			int dy[8]={1,0,-1,0,1,-1,1,-1};
			for(int k=0;k<8;++k)
			{
				if(i+dx[k]>=0 && i+dx[k]<mark.size() &&
				   j+dy[k]>=0 && j+dy[k]<mark.size() &&
				   mark[i+dx[k]][j+dy[k]]==0)
				{
					mark[i+dx[k]][j+dy[k]]=2;
				}
			}
			}
		}
	}
}

void BFS(const vector<vector<int> >& mark, 
		int start_x, int start_y,
		vector<pair<int,int> >& block,
		vector<vector<int> >& visited)
{
	stack<pair<int,int> > s;
	s.push(pair<int,int>(start_x,start_y));
	block.push_back(pair<int,int>(start_x,start_y));
	visited[start_x][start_y]=1;
	int dx[4]={0,1,0,-1};
	int dy[4]={1,0,-1,0};
	while(!s.empty())
	{
		pair<int,int> cp=s.top();
		s.pop();
		for(int k=0;k<4;++k)
		{
			if(cp.first+dx[k]>=0 && cp.first+dx[k]<mark.size() &&
			   cp.second+dy[k]>=0 && cp.second+dy[k]<mark.size() &&
			   !visited[cp.first+dx[k]][cp.second+dy[k]] &&
			   mark[cp.first+dx[k]][cp.second+dy[k]]!=0)
			{

				s.push(pair<int,int>(cp.first+dx[k],cp.second+dy[k]));
				block.push_back(pair<int,int>(cp.first+dx[k],cp.second+dy[k]));
				visited[cp.first+dx[k]][cp.second+dy[k]]=1;
			}
		}
	}
}

void FindConnectedBlocks(const vector<vector<int> >& mark,
                        vector<vector<pair<int,int> > >& blocks)
{
	vector<vector<int> > visited;
	visited.resize(mark.size());
	for(int i=0;i<visited.size();++i)
	{
		visited[i].resize(mark.size(),0);
	}
	for(int i=0;i<mark.size();++i)
	{
		for(int j=0;j<mark[i].size();++j)
		{
			if(visited[i][j] || mark[i][j]==0)
				continue;
			vector<pair<int,int> > block;
			BFS(mark,i,j,block,visited);
			blocks.push_back(block);
		}
	}
}

void FindMBR(const vector<Eigen::Vector3d>& points,
			vector<pair<double,double> >& MBR)
{
	double min_vol=1e8;
	double re_angle=0;
	double glob_x_min=1e8;
	double glob_x_max=-1e8;
	double glob_y_min=1e8;
	double glob_y_max=-1e8;
	for(int angle=0;angle<=90;angle+=5)
	{
		double rad=angle/180*PI;
		Eigen::Matrix<double,3,3> rotation;
		rotation<<cos(rad),-sin(rad),0,
				  sin(rad),cos(rad),0,
				  0,0,0;
		double x_min=1e8;
		double x_max=-1e8;
		double y_min=1e8;
		double y_max=-1e8;
		for(int i=0;i<points.size();++i)
		{
			Eigen::Vector3d temp=rotation*points[i];
			if(temp(0)<x_min)
				x_min=temp(0);
			if(temp(0)>x_max)
				x_max=temp(0);
			if(temp(1)<y_min)
				y_min=temp(1);
			if(temp(1)>y_max)
				y_max=temp(1);
		}
		double vol=(x_max-x_min)*(y_max-y_min);
		if(vol<min_vol)
		{
			min_vol=vol;
			re_angle=angle;
			Eigen::Matrix<double,2,2> rotation_back;
			rotation_back<<cos(-rad),-sin(-rad),
						   sin(-rad),cos(-rad);
			Eigen::Matrix<double,2,1> min_p,max_p;
			min_p<<x_min,y_min;
			max_p<<x_max,y_max;
			min_p=rotation_back*min_p;
			max_p=rotation_back*max_p;
			glob_x_min=min_p(0);
			glob_x_max=max_p(0);
			glob_y_min=min_p(1);
			glob_y_max=max_p(1);
		}
	}
	MBR.push_back(pair<int,int>(glob_x_min,glob_y_min));
	MBR.push_back(pair<int,int>(glob_x_min,glob_y_max));
	MBR.push_back(pair<int,int>(glob_x_max,glob_y_max));
	MBR.push_back(pair<int,int>(glob_x_max,glob_y_min));
}

void FindObstacles(const vector<vector<pair<int,int> > >& blocks,
                const vector<vector<vector<Eigen::Vector3d> > >& grid_points,
				const vector<vector<double> >& max_height,
                const vector<vector<double> >& min_height,
                vector<ObstaclePos>& positions)
{
	for(int i=0;i<blocks.size();++i)
	{
		ObstaclePos pos;
		double z_min=1e8;
		double z_max=-1e8;
		for(int j=0;j<blocks[i].size();++j)
		{
			const pair<int,int> cp=blocks[i][j];
			if(max_height[cp.first][cp.second]>z_max)
				z_max=max_height[cp.first][cp.second];
			if(min_height[cp.first][cp.second]<z_min)
				z_min=min_height[cp.first][cp.second];

			for(int k=0;k<grid_points[cp.first][cp.second].size();++k)
			{
				pos.points.push_back(grid_points[cp.first][cp.second][k]);
			}
		}
		pos.z_min=z_min;
		pos.height=z_max-z_min;
		FindMBR(pos.points,pos.MBR);
		positions.push_back(pos);
	}
}