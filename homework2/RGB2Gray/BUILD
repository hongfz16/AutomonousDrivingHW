package(default_visibility = ["//visibility:public"])

cc_library(
    name="RGB2Grey",
    srcs=["RGB2Grey.cc"],
    hdrs=["RGB2Grey.h"],
    deps=["@myopencv//:opencv"]
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        "@myopencv//:opencv",
        ":RGB2Grey",
    ],
)
