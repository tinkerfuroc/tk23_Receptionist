# TTS
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run asr tts"

sleep 5

# ASR
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run asr asr_msg_publisher"
sleep 10

gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run asr list_parser"
sleep 5

# Realsense
gnome-terminal --tab -- zsh -c ". install/setup.zsh;  ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true depth_module.profile:=848x480x30 rgb_camera.profile:=848x480x30"

sleep 5

# Face Recognition
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run face_recognition_arcface receptionist_face_task"

sleep 5

# Object Detection
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run object_detection service "
sleep 10

# Rviz
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run rviz2 rviz2 -d /home/dssl/Projects/tk23_ws/src/tk23_Receptionist/config/receptionist_rviz.rviz"
sleep 10

# # After running above, run receptionist.py
# gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run tinker_receptionist receptionist.py"