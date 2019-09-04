# masterproject
1:Before run the code, please make sure you have prepared all the environment needed including ROS,Tiago.

2:Clone the src folder into one of your workspace's package

3:Clone the world folder into your catkin_ws/src/tiago_simulation/tiago_gazebo.(catkin_ws is the folder you create your environment)
The Gazebo world is made by Adrian Bonus, the memeber of robot club of University of Leeds.

4:Make sure you have the model of 'hinged_door'. You can check it in the models block in the gazebo world. If you don't have, please download the models from: https://bitbucket.org/osrf/gazebo_models/downloads/ Then copy the 3d folder file to /home/username/.gazeb 

5:Make sure you have catkin_make in the catkin_ws/src

6:In the folder of your src, chomod +x lis.py test.py

7:Run Gazebo world by: roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true robot:=steel world:=office_with_door7

8:In the new terminal, please run lis.py

9:Once the callback information is being sending in the terminal window, run test.py in another new terminal window.

Tip: If you are a memeber of robot club of University of Leeds, you can use our container to run lis.py and test.py without the preparations steps. Just roslaunch custom_wolrds door_world.launch and run lis.py then test.py in other container terminal windows.

Tip2:The videos pull.mp4 and push.mp4 demonstrated the results of running code.
