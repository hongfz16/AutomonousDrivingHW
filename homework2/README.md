# Homework2的说明
## Task1
- 使用opencv自带的直方图绘图函数`cv::calcHist()`
- 源代码位置：`DrawHistogram/`
- 运行方法：修改`main.cpp`中的点云文件路径以及存储路径前缀，运行后将会直接显示两个直方图，并且存储在`DrawHistogram/Histogram`下

## Task2
- 对于单帧的点云数据进行地面点集的分离，算法参考[A Fast Ground Segmentation Method for 3D Point Cloud](http://jips-k.org/file/down?pn=463)
- 点云的可视化使用opencv的`viz`模块，需要`QT`，`VTK`的依赖，编译opencv的时候需要打开编译`viz`模块的开关
- 分离算法接口说明：
	- `void preprocess(vector<Vector3d>& points,vector<vector<Vector3d> >& sortedpoints)`
			- 函数说明：点云数据预处理，主要是对点云数据分类，排序
			- 第一个参数：读入的点云原数据
			- 第二个参数：处理好的点云数据
	- `void SeperateGroundPoints(vector<vector<Vector3d> >& sortedpoints,vector<Vector3d>& ground,vector<Vector3d>& nonground,double alphamax,double missmax,double hmin)`
			- 函数说明：地面点集分离算法
			- 第一个参数：预处理的有序点集
			- 第二三个参数：地面点集和非地面点集
			- 后三个参数：算法中需要调整的阈值，推荐参数为`(PI/4, 0.1, 0.1)`
- 源代码位置：`PointCloud/`
- 运行方法：修改`main.cpp`中的点云文件路径，如果环境满足上述要求，直接运行将会弹出点云可视化窗口，其中地面点集为红色，其他的为绿色；若环境不满足要求，则可以按照上述要求使用算法接口进行验证；运行效果图存储在`PointCloud/Result`文件夹下

## Task3
- 使用公式`GRAY = (R*30 + G*59 + B*11 + 50) / 100`来计算灰度
- 源代码位置：`RGB2Gray/`
- 运行方法：修改`main.cpp`中的图片路径以及存储路径前缀，运行后将会显示出转化为灰度的图片，并且存储在`RGB2Gray/Converted`下

## Task4
- 