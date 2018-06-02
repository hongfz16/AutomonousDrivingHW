package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hongfz16_agent",
    srcs = ["hongfz16_agent.cc"],
    hdrs = ["hongfz16_agent.h"],
    deps = [
        "//pnc/simulation:vehicle_agent",
        "//pnc/simulation:vehicle_agent_factory",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "@glog",
        ":GetPredSucc",
        ":FindRoute",
        "@eigen//:eigen",
    ],
)

cc_library(
    name="GetPredSucc",
    srcs=["GetPredSucc.cc"],
    hdrs=["GetPredSucc.h"],
    deps=[
        "//common/utils/file",
        "//pnc/map:map_lib",
        "@gflags//:gflags",
        "@glog",
    ]
)

cc_library(
    name="FindRoute",
    srcs=["FindRoute.cc"],
    hdrs=["FindRoute.h"],
    deps=[
        "//common/utils/file",
        "//pnc/map:map_lib",
        "//common/proto:cc_route_proto",
        "//common/proto:cc_geometry_proto",
        "//common/proto:cc_transform_proto",
        "@gflags//:gflags",
        "@glog",
    ]
)
