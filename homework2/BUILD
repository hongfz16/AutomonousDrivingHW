package(default_visibility = ["//visibility:public"])

cc_library(
    name="sepGround",
    srcs=["sepGround.cc"],
    hdrs=["sepGround.h"],
    deps=[":pointcloud"],
)

cc_library(
    name="LaneDetect",
    srcs=["LaneDetect.cc"],
    hdrs=["LaneDetect.h"],
    deps=["@opencv//:opencv"],
)

cc_library(
    name="DrawHistogram",
    srcs=["DrawHistogram.cc"],
    hdrs=["DrawHistogram.h"],
    deps=[
        "@opencv//:opencv",
        ":pointcloud",
    ]
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
    deps=["@opencv//:opencv"]
)

cc_library(
    name="RGB2Grey",
    srcs=["RGB2Grey.cc"],
    hdrs=["RGB2Grey.h"],
    deps=["@opencv//:opencv"]
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        "@opencv//:opencv",
        ":RGB2Grey",
        ":Generate3DPC",
        ":pointcloud",
        ":DrawHistogram",
        ":LaneDetect",
        ":sepGround",
    ],
)
