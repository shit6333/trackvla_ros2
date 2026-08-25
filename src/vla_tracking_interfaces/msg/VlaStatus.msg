# Task and backend state published by vla_inference_node.

std_msgs/Header header

# --- task state ---------------------------------------------------------
uint8 IDLE=0
uint8 WARMING_UP=1
uint8 TRACKING=2
uint8 ERROR=3
uint8 STOPPED=4

uint8 task_state

# The instruction currently being tracked. Empty when no task is active.
string instruction

# --- backend state ------------------------------------------------------
string backend_name

# True once the backend has loaded and is able to serve inference.
bool backend_ready

# --- timing -------------------------------------------------------------
# Wall-clock seconds spent inside the most recent backend inference call.
float32 last_inference_duration

# Observation timestamp of the most recent prediction. Zero before the first
# prediction of a task.
builtin_interfaces/Time last_observation_stamp

# Predictions published since the current task started.
uint32 predictions_published

# --- errors -------------------------------------------------------------
# Empty unless task_state is ERROR.
string error_message
