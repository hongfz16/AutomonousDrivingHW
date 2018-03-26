# need to run with the following command:
- `bazel run -c opt --cxxopt='-std=c++14' //homework4:camera_lidar_fusion_main -- --pony_data_dir /home/hongfz/Documents/Learn/AutomonousDrivingHW/sample`
- `bazel run -c opt --cxxopt='-std=c++14' --copt='-fPIC' //homework4:pointcloud_viewer_main -- --pony_data_dir /home/hongfz/Documents/Learn/AutomonousDrivingHW/sample`