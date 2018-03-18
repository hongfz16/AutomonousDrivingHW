package(default_visibility = ["//visibility:public"])

cc_library(
    name="sepGround",
    srcs=["sepGround.cc"],
    hdrs=["sepGround.h"],
    deps=[":pointcloud"],
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
    deps=["@myopencv//:opencv"]
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        "@myopencv//:opencv",
        ":Generate3DPC",
        ":pointcloud",
        ":sepGround",
    ],
)
