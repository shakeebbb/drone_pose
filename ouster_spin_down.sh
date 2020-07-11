#!/bin/bash

nc os1-991939999414.local 7501
set_config_param auto_start_flag 0
reinitialize

sleep 5

get_sensor_info

