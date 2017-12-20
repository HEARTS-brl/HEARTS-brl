# brl-hearts Competition Repository 
(Bristol Robotics Laboratory - Healthcare Engineering and Assistive Robotics Technology and Services)

"The European Robotic League (ERL) is an innovative concept for robot competitions. The ERL is composed of multiple Local Tournaments, held in different research labs across Europe, with certified test beds, and a few competitions as part of Major Tournaments, such as RoboCup. Teams participate in a minimum of 2 tournaments (Local and/or Major) per year and get scores based on their performances. A final end of year score is computed for each team, per Task Benchmark (TBM) and Functionality Benchmark (FBM), using the best two participation in tournaments, and teams are ranked based on their final score. Prizes for the top teams (per TBM and FBM where they have participated) are awarded during the next year's European Robotics Forum (ERF)."
-https://eu-robotics.net/robotics_league/erl-service/about/index.html



The code in this repository consists of ROS (indigo) packages written for the TIAGO robot. The code is divided into general skills (e.g. speech to text) and specific modules for use during the testing of the functional and task benchmarks (e.g. fbm1 or tbm2). 

_tiago ssh details_
name: pal@tiago-25c
password: pal

_webcommander_
http://tiago-25c:8080/

_remote access_
ssh pal@tiago-25c
scp [-r] pal@tiago-25c:<remote> <local>
scp -r pal@tiago-25c:/home/pal/.pal/tiago_maps/configurations/<map_name>/ /home/turtlebot/<map_name>


_for stopping the head movement_
head_manager


how to make for computers with issues with face recognition
catkin_make --pkg fbm1_object_perception fbm2_navigation hearts_camera_saver hearts_navigation hearts_test_package tbm1_getting_to_know_my_home tbm2_welcoming_visitors brl_ipcam fbm3_speech_understanding hearts_stt hearts_tts python_support_library roah_rsbb_comm_ros hearts_face_uniform hearts_webcam speech_recognition

OR


catkin_make -DCATKIN_BLACKLIST_PACKAGES="object_perception_2;fbm1_object_perception;hearts_face_uniform_reg;hearts_face_uniform"


