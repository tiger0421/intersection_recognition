# rosInterctionRecognition
intersection recognition(ros ver.)

## param
- hypothesis_simple.launch
  - toe_finding
    - "scan_hz" : Frequency of 2D-LiDAR
    - "scan_off_set" : How many adjacent point clouds to ignore
    - "epsilon1" : Threshold
    - "epsilon2" : Threshold
    - "epsilon3" : Threshold

- hypothesis.launch
  - intersection_recognition_tensorflow.p
    - "model_full_path": Path to load Neural Network model(.h5)

## movie
https://youtu.be/1tNvuxRLBRw

### On simulator
The robot go straight until it find 3-way junction whici can turn left.
- Test 1
a straight road
https://youtu.be/G36qR3UHfps

- Test 2
a road included 3-way junction which can turn right.
https://youtu.be/VW5J42WxkhI
