package(default_visibility = ["//visibility:public"])

cc_library(
	name="LIDAR2World",
	srcs=["LIDAR2World.cc"],
	hdrs=["LIDAR2World.h"],
	deps=[":pointcloud",],
)

cc_library(
    name = "pointcloud",
    srcs = ["pointcloud.cc"],
    hdrs = ["pointcloud.h"],
    deps = [
        "@eigen//:eigen",
        "@glog//:glog",
    ],
)

cc_library(
    name="Generate3DPC",
    srcs=["Generate3DPC.cc"],
    hdrs=["Generate3DPC.h"],
    deps=[
        "@myopencv//:opencv",
        "@eigen//:eigen",
    ]
)

cc_library(
    name="sepGround",
    srcs=["sepGround.cc"],
    hdrs=["sepGround.h"],
    deps=[":pointcloud"],
)

cc_binary(
	name="main",
	srcs=["main.cc"],
	deps=[
		":pointcloud",
		":LIDAR2World",
        ":Generate3DPC",
        ":sepGround",
	]
)