package(default_visibility = ["//visibility:public"])

cc_library(
	name="svm_common",
	srcs=["svm_common.cc"],
	hdrs=["svm_common.h"],

)

cc_library(
	name="svm_classifier",
	srcs=["svm_classifier.cc"],
	hdrs=["svm_classifier.h"],
	deps=[
		":svm_common",
	],
)