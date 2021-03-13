# ball chaser for ROS2 foxy(python and c++ versions)

* ball_chaser

    python version

* ball_chaser_cpp

    c++ version

* robot_interfaces
    service for drive_bot


## run with Turtlebot 3
1. 必須在 TB3 執行 bringup TB3 & camera
	1. cd ~/turtlebot3_ws
	2. Bring Up Robot
	   
       ros2 launch turtlebot3_bringup robot.launch.py
	3. Bring Up Camera, 設定camera解析度at same command
	   
       ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[640,480]
2. 取得 RGB 值
	4. 用 raspistill 取得含目標物的照片
	5. 用 range_detector.py  調整 RGB
	   
       python range_detector.py --image image.jpg --filter RGB --preview
	6. 範例:
	   
3. 以下可在 NB 或TB3任選一處執行.
	7. 設定camera解析度640*480
	   
       ros2 param set /v4l2_camera image_size [640,480]
	8. 在NB 建議先將 image 壓縮訊號轉為 raw,            process_image 再subscript raw topic, 不要直接用 image_raw topic.
	   
       ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=camera/image
	9. 執行 drive bot
	   ros2 run ball_chaser drive_bot
	4. 執行 process image
	   ros2 run process_image


## 
