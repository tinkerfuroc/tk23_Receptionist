# TTS
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run asr tts"

sleep 10

# Face Recognition
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run face_recognition_arcface receptionist_face_task"

sleep 10



# After running above, run receptionist.py
gnome-terminal --tab -- zsh -c ". install/setup.zsh; ros2 run tinker_receptionist receptionist.py"