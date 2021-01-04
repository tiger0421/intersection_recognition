# rosInterctionRecognition
intersection recognition(ros ver.)

## Requirement
- MeCab (Python)

## Run
```
$ roslaunch intersection_recognition simple_hypothesis.launch
```

## ROS Param
- simple_hypothesis.launch
  - extended_toe_finding
    - **scan_hz** : Frequency of 2D-LiDAR  
    - **distance_thresh** : Threshold of LiDAR-beam
  - read_scenario.py
    - **scenario_path** : Path to .txt-file that a scenario is saved to

- hypothesis_simple.launch
  - toe_finding
    - **scan_hz** : Frequency of 2D-LiDAR
    - **scan_off_set** : How many adjacent point clouds to ignore
    - **epsilon1** : Threshold
    - **epsilon2** : Threshold
    - **epsilon3** : Threshold

- hypothesis.launch
  - intersection_recognition_tensorflow.p
    - **model_full_path**: Path to load Neural Network model(.h5)

### Node
![image](https://user-images.githubusercontent.com/20837922/95672217-e7d16280-0bd9-11eb-92d9-93dac311336b.png)
  - **check_scenario** : Compare scenario and hypothesis of intersection.
    - **/rotate_rad** : Specifies how much the robot should rotate with /rotate_rad (if /rotate_rad == 0, then go straight).
  - **cmd_vel_controller2** : The robot will rotate at the angle specified by /rotate_rad to publish cmd_vel.
    - **/turn_finish_flg** : Publish this flg when the robot rotates at the angle specified by /rotate_rad.
  - **/emergency_stop_flg** : When this flg is true, the robot stop the move and the calculation. 

## Movie
### On simulator (simple_hypothesis.launch)
https://www.youtube.com/watch?v=1WaGJrMmR0E

### toe-finding Ver.
https://youtu.be/fdbXFAEWZBk

### Machine Learning Ver.
https://youtu.be/1tNvuxRLBRw
