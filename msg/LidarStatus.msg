# Reports the status of the 'AR00' message response from the lidar.
# This contains the error state and operating modes.
uint16 status

# Normal vs setting in the UAM manual.
uint16 NORMAL=0
uint16 SETTING=1
uint16 operating_mode

# AKA field set
uint16 area_number
# If the lidar is reporting an error or not.
bool error_status
# THe error code the lidar is reporting.
uint16 error_code
# Does the lidar report that it is locked out.
bool lockout_status
