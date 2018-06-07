#pragma once

#include "homework2/pointcloud.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stack>
using namespace std;

#ifndef PI
#define PI 3.141592653
#endif
struct ObstaclePos
{
    vector<pair<double,double> > MBR;
    double z_min;
    double height;
    vector<Eigen::Vector3d> points;
};

void FillGrid(vector<vector<vector<Eigen::Vector3d> > >& grid_points, 
                vector<Eigen::Vector3d>& points, double range);

void RemoveNoisePoints(vector<vector<vector<Eigen::Vector3d> > >& grid_points);

void MarkGrid(const vector<vector<vector<Eigen::Vector3d> > >& grid_points, 
            vector<vector<double> >& max_height, 
            vector<vector<double> >& min_height, 
            vector<vector<int> >& mark,
            double TH);

void FindConnectedBlocks(const vector<vector<int> >& mark,
                        vector<vector<pair<int,int> > >& blocks);

void FindObstacles(const vector<vector<pair<int,int> > >& blocks,
                const vector<vector<vector<Eigen::Vector3d> > >& grid_points,
                const vector<vector<double> >& max_height,
                const vector<vector<double> >& min_height,
                vector<ObstaclePos>& positions);