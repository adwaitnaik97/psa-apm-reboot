#!/bin/bash
# This configures the environment variables for the truck and this is sourced automatically by the sourcing of the terminal

# Truck ID
export AIOS_TRUCK_ID=8381
AIOS_CONFIG_PATH=$(rospack find aios_config)


# Sensor Configuration
#-----------------------------------------------------------

# ---Lidar_Calibration dir---
export AIOS_RS16_CALIB_DIR="$AIOS_CONFIG_PATH/configuration_data/$AIOS_TRUCK_ID/lidar_calibration"

# ---Lidar_Bottom_Right---
export AIOS_RS16_BOTTOM_RIGHT_ID="rs16_br"
export AIOS_RS16_BOTTOM_RIGHT_FRAME_ID="rs16_br"
export AIOS_RS16_BOTTOM_RIGHT_IP="172.20.2.105"
export AIOS_RS16_BOTTOM_RIGHT_SN="161191201429"
export AIOS_RS16_BOTTOM_RIGHT_MSOP=6705
export AIOS_RS16_BOTTOM_RIGHT_DIFOP=7705
export AIOS_RS16_BOTTOM_RIGHT_CALIB_PATH="$AIOS_RS16_CALIB_DIR/RS_$AIOS_RS16_BOTTOM_RIGHT_SN"

# ---Lidar_Top_Right---
export AIOS_RS16_TOP_RIGHT_ID="rs16_tr"
export AIOS_RS16_TOP_RIGHT_FRAME_ID="rs16_tr"
export AIOS_RS16_TOP_RIGHT_IP="172.20.2.104"
export AIOS_RS16_TOP_RIGHT_SN="161190204584"
export AIOS_RS16_TOP_RIGHT_MSOP=6704
export AIOS_RS16_TOP_RIGHT_DIFOP=7704
export AIOS_RS16_TOP_RIGHT_CALIB_PATH="$AIOS_RS16_CALIB_DIR/RS_$AIOS_RS16_TOP_RIGHT_SN"

# ---Lidar_Top_Center---
export AIOS_RS16_TOP_CENTER_ID="rs16_tc"
export AIOS_RS16_TOP_CENTER_FRAME_ID="rs16_tc"
export AIOS_RS16_TOP_CENTER_IP="172.20.2.103"
export AIOS_RS16_TOP_CENTER_SN="161190204133"
export AIOS_RS16_TOP_CENTER_MSOP=6703
export AIOS_RS16_TOP_CENTER_DIFOP=7703
export AIOS_RS16_TOP_CENTER_CALIB_PATH="$AIOS_RS16_CALIB_DIR/RS_$AIOS_RS16_TOP_CENTER_SN"

# ---Lidar_Top_Left---
export AIOS_RS16_TOP_LEFT_ID="rs16_tl"
export AIOS_RS16_TOP_LEFT_FRAME_ID="rs16_tl"
export AIOS_RS16_TOP_LEFT_IP="172.20.2.102"
export AIOS_RS16_TOP_LEFT_SN="161190205663"
export AIOS_RS16_TOP_LEFT_MSOP=6702
export AIOS_RS16_TOP_LEFT_DIFOP=7702
export AIOS_RS16_TOP_LEFT_CALIB_PATH="$AIOS_RS16_CALIB_DIR/RS_$AIOS_RS16_TOP_LEFT_SN"

# ---Lidar_Bottom_Left---
export AIOS_RS16_BOTTOM_LEFT_ID="rs16_bl"
export AIOS_RS16_BOTTOM_LEFT_FRAME_ID="rs16_bl"
export AIOS_RS16_BOTTOM_LEFT_IP="172.20.2.101"
export AIOS_RS16_BOTTOM_LEFT_SN="161190205753"
export AIOS_RS16_BOTTOM_LEFT_MSOP=6701
export AIOS_RS16_BOTTOM_LEFT_DIFOP=7701
export AIOS_RS16_BOTTOM_LEFT_CALIB_PATH="$AIOS_RS16_CALIB_DIR/RS_$AIOS_RS16_BOTTOM_LEFT_SN"

#-----------------------------------------------------------
# ---Functional_safety_sensor_(FSS)----
# ---Sick_mrs1000----
export AIOS_SICK_FSS_ID="sick_fss"
export AIOS_SICK_FSS_FRAME_ID="sick_fss"
export AIOS_SICK_FSS_IMU_FRAME_ID="sick_imu_link"
export AIOS_SICK_FSS_IP="192.168.82.178"
export AIOS_SICK_FSS_SN=""


#-----------------------------------------------------------

# ---Camera_Calibration dir---
export AIOS_BFLY_CALIB_DIR="$AIOS_CONFIG_PATH/configuration_data/$AIOS_TRUCK_ID/camera_calibration"

# ---Camera_Front_Top_Center--- how about top center???
export AIOS_BFLY_FRONT_TOP_CENTER_ID="camera_top_center_front"
export AIOS_BFLY_FRONT_TOP_CENTER_FRAME_ID="camera_top_center_front"
export AIOS_BFLY_FRONT_TOP_CENTER_SN="19120926"
export AIOS_BFLY_FRONT_TOP_CENTER_CALIB_PATH="$AIOS_BFLY_CALIB_DIR/$AIOS_BFLY_FRONT_TOP_CENTER_SN.yaml"
#export AIOS_BFLY_FTC_IP="192.168.1.110"
#export AIOS_BFLY_FTC_CALIB_STATUS=true

# ---Camera_Front_Bottom_Center--- 
export AIOS_BFLY_FRONT_BOTTOM_CENTER_ID="cam_front_bottom_centre"
export AIOS_BFLY_FRONT_BOTTOM_CENTER_FRAME_ID="cam_front_bottom_centre"
export AIOS_BFLY_FRONT_BOTTOM_CENTER_SN="19073725"
export AIOS_BFLY_FRONT_BOTTOM_CENTER_CALIB_PATH="$AIOS_BFLY_CALIB_DIR/$AIOS_BFLY_FRONT_BOTTOM_CENTER_SN.yaml"

# ---Camera_Front_Bottom_Left--- 
export AIOS_BFLY_FRONT_BOTTOM_LEFT_ID="camera_top_left_front"
export AIOS_BFLY_FRONT_BOTTOM_LEFT_FRAME_ID="camera_top_left_front"
export AIOS_BFLY_FRONT_BOTTOM_LEFT_SN="19095440"
export AIOS_BFLY_FRONT_BOTTOM_LEFT_CALIB_PATH="$AIOS_BFLY_CALIB_DIR/$AIOS_BFLY_FRONT_BOTTOM_LEFT_SN.yaml"

# ---Camera_Front_Bottom_Right--- 
export AIOS_BFLY_FRONT_BOTTOM_RIGHT_ID="camera_top_right_front"
export AIOS_BFLY_FRONT_BOTTOM_RIGHT_FRAME_ID="camera_top_right_front"
export AIOS_BFLY_FRONT_BOTTOM_RIGHT_SN="18481923"
export AIOS_BFLY_FRONT_BOTTOM_RIGHT_CALIB_PATH="$AIOS_BFLY_CALIB_DIR/$AIOS_BFLY_FRONT_BOTTOM_RIGHT_SN.yaml"

# ---Camera_Rear_Top_Left--- 
export AIOS_BFLY_REAR_TOP_LEFT_ID="cam_rear_top_left"
export AIOS_BFLY_REAR_TOP_LEFT_FRAME_ID="cam_rear_top_left"
export AIOS_BFLY_REAR_TOP_LEFT_SN="19095450"
export AIOS_BFLY_REAR_TOP_LEFT_CALIB_PATH="$AIOS_BFLY_CALIB_DIR/$AIOS_BFLY_REAR_TOP_LEFT_SN.yaml"

# ---Camera_Rear_Top_Right--- 
export AIOS_BFLY_REAR_TOP_RIGHT_ID="cam_rear_top_right"
export AIOS_BFLY_REAR_TOP_RIGHT_FRAME_ID="cam_rear_top_right"
export AIOS_BFLY_REAR_TOP_RIGHT_SN="18481922"
export AIOS_BFLY_REAR_TOP_RIGHT_CALIB_PATH="$AIOS_BFLY_CALIB_DIR/$AIOS_BFLY_REAR_TOP_RIGHT_SN.yaml"

#-----------------------------------------------------------
# ---NOVATEL_GNSS--- 
export AIOS_NOVATEL_ID="novatel"
export AIOS_NOVATEL_FRAME="novatel"
export AIOS_NOVATEL_SN="NA"
export AIOS_NOVATEL_IP="192.168.1.125"
export AIOS_NOVATEL_PORT="2000"
export AIOS_NOVATEL_DUAL_ANT="TRUE"


#-----------------------------------------------------------
# ---DELPHI_RADAR_ESR--- 
export AIOS_DELPHI_ESR_ID="esr"
export AIOS_DELPHI_ESR_FRAME_ID="esr"
export AIOS_DELPHI_ESR_SN="NA"
export AIOS_DELPHI_ESR_USE_KVASER="TRUE"
export AIOS_DELPHI_ESR_KVASER_HW_ID="56103"
export AIOS_DELPHI_ESR_KVASER_CIRCUIT_ID="0"

# ---DELPHI_RADAR_SRR2--- 
export AIOS_DELPHI_SRR_ID="srr"
export AIOS_DELPHI_SRR_LEFT_FRAME_ID="left_srr"
export AIOS_DELPHI_SRR_RIGHT_FRAME_ID="right_srr"
export AIOS_DELPHI_SRR_LEFT_SN="NA"
export AIOS_DELPHI_SRR_RIGHT_SN="NA"
export AIOS_DELPHI_SRR_USE_KVASER="TRUE"
export AIOS_DELPHI_SRR_KVASER_HW_ID="50404"
export AIOS_DELPHI_SRR_KVASER_CIRCUIT_ID="0"
#-----------------------------------------------------------





# Communication Configurations
#-----------------------------------------------------------
export AIOS_CONVERT_MPS2KMPH="3.6"
export AIOS_CONVERT_RAD2DEG="57.295779513"

# ---comm_tcp--- 
export AIOS_COMM_TCP_NAME="comm_tcp"
export AIOS_COMM_TCP_IP="192.168.1.100"
export AIOS_COMM_TCP_PORT="13000"


# ---plc_constructor--- 
export AIOS_PLC_CONSTRUCTOR_NAME="plc_constructor"
export AIOS_PLC_CONSTRUCTOR_SPEED_MPS2KMPH_REQ="TRUE"
export AIOS_PLC_CONSTRUCTOR_SPEED_RAD2DEG_REQ="TRUE"
export AIOS_PLC_CONSTRUCTOR_SPEED_RES="100"
export AIOS_PLC_CONSTRUCTOR_STEERING_OFFSET="640"
export AIOS_PLC_CONSTRUCTOR_STEERING_INVERT="TRUE"
export AIOS_PLC_CONSTRUCTOR_STEERING_RES="10"

##SPEED = REQUESTED_SPEED(mps) * MPS2KMPH(if MPS2KMPH_REQ True) * SPEED_RES
##STEER = STEERING_OFFSET + [(REQUESTED_STEER(rad) * RAD2DEG(if RAD2DEG_REQ True) * SPEED_RES) * -1( if STEERING_INVERT True)]

# ---plc_driver--- 
export AIOS_PLC_DRIVER_NAME="plc_driver"
export AIOS_PLC_DRIVER_PCAN_USB="FALSE"
export AIOS_PLC_DRIVER_PCAN_PCI="TRUE"
export AIOS_PLC_DRIVER_PCAN_DEVICE_ID="PCAN_PCIBUS1"
export AIOS_PLC_DRIVER_PCAN_BAUD="500K"













