load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_mt",
    srcs = [
        "mt_controller.cc",
        "mt_message_manager.cc",
        "mt_vehicle_factory.cc",
        "protocol/clear_error_status_1.cc",
        "protocol/current_feedback_12.cc",
        "protocol/motor_fault_status_20.cc",
        "protocol/speed_feedback_10.cc",
        "protocol/wheel_speed_feedback_11.cc",
    ],
    hdrs = [
        "mt_controller.h",
        "mt_message_manager.h",
        "mt_vehicle_factory.h",
        "protocol/clear_error_status_1.h",
        "protocol/current_feedback_12.h",
        "protocol/motor_fault_status_20.h",
        "protocol/speed_feedback_10.h",
        "protocol/wheel_speed_feedback_11.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/mt/proto:mt_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libmt_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_mt"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()
cpplint()
