## PATH_FOLLOWER

![image](https://github.com/user-attachments/assets/6a1dc9f4-2131-40b7-966b-2ff34d0fbbbf)

### SUMMARY:
Development of an autonomous robot that could follow a path and detect different traffic signs. For this project my team and I worked side by side with Manchester Robotics, whom provided the hardware to make this project possible and helped us developed the necessary skills to accomplish our goal. 

### HARDWARE:
* Differential Mobile Robot structure
* 2 Motors with Encoders
* Jetson NVIDIA with Ubuntu 18.04 as OS
* Raspberry PI Wide Angle Camera 

### TOPICS INVOLVED:
* Kinematics
* Dynamics
* Artificial Vision
* Neural Networks

### ARTIFICIAL VISION:
The robot uses it's camera to obtain an image of it's surrounding. That image is cropped to a size where only the line to follow is seen. Then I applied a filter to blur it and reduce the noise caused by the joints of our track. Then we convert the image into binary, so that we can get a better look of our path. I then worked with that final image to find the path and measure the error of the angle. 

![image](https://github.com/user-attachments/assets/422606b6-6386-4509-bd9c-13d082ba8e52)
![image](https://github.com/user-attachments/assets/2fe77ed3-8e0f-4e77-a328-647382d890e5)


### NEURAL NETWORK
#### Description:
* 4,700 images
* Trained with Yolov5
* Optimized to ONNX
![image](https://github.com/user-attachments/assets/dd4b0946-3af4-4122-85e5-ab3f694efef6)

### Results
![image](https://github.com/user-attachments/assets/7b6bfd35-0d7d-4c6d-ba82-899c8ef851ac)

![image](https://github.com/user-attachments/assets/62ecf681-22e8-4a42-8635-808a041bc161)

[![Path_Follower](https://img.youtube.com/vi/itd_6g6dmB8/0.jpg)](https://www.youtube.com/watch?v=itd_6g6dmB8)

https://youtu.be/itd_6g6dmB8
