Each launch file is associated with a different function. They can be run by `roslaunch goldeneye ${COMMAND} param:=value`

1. sensor/*.launch  
   These launch files launch each sensor individually for debugging purposes. The camera module also launches an image viewer.

2. all_sensor.launch
   This launches all the sensors at once

3. record.launch
   This launches the data recorder. The following parameters can be changed
   - root_path (String): The root path where you want data to be stored. This is set by default to /home/odroid/goldeneye_data
   - camera (Boolean): Collect data from the camera. Default is false. 
   - processed_image (Boolean): Collect data from the processed_image topic. Default is false. 
   - gps (Boolean): Collect data from the gps. Default is false. 
   - lidar (Boolean): Collect data from the lidar. Default is false. 
   - imu (Boolean): Collect data from the imu. Default is false. 
   - actuation (Boolean): Collect data from the motor and servos. Default is True. 

4. lane_follower.launch
   This launches the Arduino, Camera and the simple_lane_follower.py script from goldeneye. The following parameters can be changed
   - display_image (Boolean): Whether to display the raw camera image. Default is False.
   - display_processed_image (Boolean): Whether to display the processed images. Default is False.
   - publish_processed_image (Boolean): Whether to publish the processed images. Default is False.
   - debug_info (Boolean): Whether to print debug information. Default is False.
   - upperY (Integer): Bottom part of the image we want to crop. Default is 500.
   - lowerY (Integer): Top part of the image we want to crop. Default is 200.


