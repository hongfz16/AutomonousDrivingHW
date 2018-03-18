package(default_visibility = ["//visibility:public"])

cc_library(
    name="LD_github",
    srcs=["LD_github.hpp"],
    hdrs=["LD_github.hpp"],
    deps=["@myopencv//:opencv"],
)

cc_library(
    name="LaneDetect",
    srcs=["LaneDetect.cc"],
    hdrs=["LaneDetect.h"],
    deps=["@myopencv//:opencv"],
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        "@myopencv//:opencv",
        ":LaneDetect",
        ":LD_github",
    ],
)
