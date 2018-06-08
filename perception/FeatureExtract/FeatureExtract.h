#pragma once

#include "homework2/pointcloud.h"
#include "RTree.h"
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <Eigen/Eigenvalues>
using namespace std;

string ExtractFeature(const vector<Eigen::Vector3d>& points);
string ExtractTrainingFeature(const vector<Eigen::Vector3d>& points, int mclass);
