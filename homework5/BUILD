package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_visualizer_main",
    srcs = ["map_visualizer_main.cc"],
    deps = [
        "//common/utils/file",
        "//homework5/display:main_window",
        "//homework5/map:map_lib",
        "@gflags//:gflags",
        "@glog",
    ],
)


cc_library(
	name="GetPredSucc",
	srcs=["GetPredSucc.cc"],
	hdrs=["GetPredSucc.h"],
	deps=[
		"//common/utils/file",
        "//homework5/map:map_lib",
        "@gflags//:gflags",
        "@glog",
	]
)

cc_binary(
	name="GetPredSucc_main",
	srcs=["GetPredSucc_main.cc"],
	deps=[
		":GetPredSucc",
	]
)

cc_library(
    name="FindRoute",
    srcs=["FindRoute.cc"],
    hdrs=["FindRoute.h"],
    deps=[
        "//common/utils/file",
        "//homework5/display:main_window",
        "//homework5/map:map_lib",
        "@gflags//:gflags",
        "@glog",
    ]
)

cc_binary(
    name="FindRoute_main",
    srcs=["FindRoute_main.cc"],
    deps=[
        ":FindRoute",
    ]
)