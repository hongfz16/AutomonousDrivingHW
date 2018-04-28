# Homework6 Document

## Generate Table
- Method: Use `vehicle_status_model::VehicleStatusModelSolver` to simulate the car's different reactions on different status and commands; Consider velocity as the only property that have effect on acceleration; Take 0.1m/s as step of velocity, 0.1 as step of throttle/brake ratio;
- Table files: `homework6/agents/generate_table/throttle_table.txt` and `homework6/agents/generate_table/brake_table.txt`
- Table file format: Each line is one record containing velocity, throttle/brake ratio and expected acceleration.
- Source file: `homework6/agents/generate_table/generate_table.cc`
- Run with the command: `bazel run //homework6/agents/generate_table:generate_table`

## Simple agent implementation
- Method: Use PID algorithm where velocity is used as the variable to control to control the velocity when the car is expected to maintain a constant velocity; Look up table generated in task one to maintain a constant acceleration when the car is expected to accelerate; Use PID algorithm where the angle between expected direction and current velocity direction is used as the variable to control to control the steer angle when the car is expected to turn;
- Source file path: `homework6/agents/myagent`
- Run with the command same as the instruction;
- PLEASE REMEMBER TO CHANGE THE PATH IN `homework6/agent/myagent/hongfz_agent.h`(line 372) to your current working path;
- The videos will be placed in `homework6/result/videos`
- The velocity logs will be placed in `homework6/result/velocity_log`