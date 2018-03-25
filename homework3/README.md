# Homework3 文档
## Task1
- 三个方向的旋转矩阵相乘的顺序很重要，通过反解旋转矩阵可以将三个方向的旋转角解出；旋转矩阵转化为四元组的过程也类似；具体公式请参考源代码；
- 源代码位置：`rotation/`

## Task2
- 简单使用`opencv`中的API`undistort()`，实现了原图片的径向畸变
- 源代码位置：`chessboard/`
- 使用说明：修改文件路径，直接运行即可

## Task3
- 通过每个点云数据文件开头的旋转矩阵(rotation)，和变换矩阵(transition)，将每个点从LIDAR本地坐标系转化为全球坐标系；地面点的分离实现了两个方法，一个是通过选取z坐标小于某个阈值的方法，另一个是第二次作业中使用的算法[A Fast Ground Segmentation Method for 3D Point Cloud](http://jips-k.org/file/down?pn=463)；详见源代码注释；
- 点云的可视化使用`viz`模块，与第二次作业相同
- 源代码位置：`pointcloud3`
- 使用说明：修改点云文件夹路径，直接运行将会使用`viz`模块将可视化的结果显示