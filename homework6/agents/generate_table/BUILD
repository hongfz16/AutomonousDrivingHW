package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "generate_table",
    srcs = ["generate_table.cc"],
    deps = [
        "//homework6/simulation:vehicle_agent",
        "//homework6/simulation:vehicle_agent_factory",
        "//homework6/simulation/dynamic_lib:libdynamic",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//common/utils/file",
        "//common/utils/file:path",
        "@glog",
    ],
)