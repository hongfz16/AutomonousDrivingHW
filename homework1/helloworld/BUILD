package(default_visibility = ["//visibility:public"])

cc_library(
	name="greeting",
	srcs=["greeting.cc"],
	hdrs=["greeting.h"],
)

cc_binary(
    name = "main",
    srcs = ["main.cc"],
    deps=[
    	":greeting",
    ]
)

