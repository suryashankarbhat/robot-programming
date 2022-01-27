ROBTOTICS ASSESSMENT - COUNTING OF GRAPES AUTONOMOUSLY USING ROS PROGRAMMING

  STEP 1:
1.  use the below code to update all the softwares
Update
    
    
    
        sudo apt-get update && sudo apt-get upgrade
    


2. Install below packages required for the assessment
 

    
        sudo apt-get install \ 
            ros-melodic-topological-utils \
            ros-melodic-topological-navigation \
            ros-melodic-topological-navigation-msgs \
            ros-melodic-strands-navigation
3.  if you don't have python libraries please run the below code to install it and install opencv according to your system requirement

           
           
            pip install --user imutils
            pip install -U scikit-learn



4.First, make sure that you created a workspace and clone the required files from github https://github.com/suryashankarbhat/robot-programming/tree/main/uol_cmp9767m_tutorial or copy the files uol_cmp9767m_tutorial from CMP9767M assessment

STEP 2:

1.In this task,we will run the below code to launch the vineyard map in rviz and gazebo environment
The topological map for the demo is available in uol_cmp9767m_tutorial/maps/assessment.yaml. 
 Create a folder (named mongodb) in your user home directory. MongoDB will store all database files required to run our topological map. This step is required only once.
    Launch the simulation setup

        roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small
        
2. to launch the required files 


        roslaunch uol_cmp9767m_tutorial topo_nav.launch. 

if you work with a dockerised distribution (e.g. at home or using a remote access) please use the following line instead which will help to address      some issues with the MongoDB database: HOSTNAME=0.0.0.0 roslaunch uol_cmp9767m_tutorial topo_nav.launch.
        
you will see some warnings in the terminal where you launched topo_nav.launch saying the pointset is not found in the message_store. This is because we haven't loaded the topological map to the mongodb yet. Once you do the next step, that warning should stop.


        rosrun topological_utils load_yaml_map.py $(rospack find uol_cmp9767m_tutorial)/maps/assessment2.yaml
 
 
 This step is required only once.
       
 open the topological map visualisation config for RVIZ in uol_cmp9767m_tutorial/config/topo_nav.rviz
    

STEP 3: - 

1.to create an action client that can send goals to the robot's topological navigation action.
  In another terminal run 
  
  
    rosrun uol_cmp9767m_tutorial set_topo_nav_goal.py 
   
   it will autonomously move to the wave points

2.to count the the number of fruits in the vineyard run the below code in another terminal 


    rosrun uol_cmp9767m_tutorial grape_detection.py

      

