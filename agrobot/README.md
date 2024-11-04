![image](https://github.com/user-attachments/assets/48b98c19-6db2-419a-a9a7-83903dbc43da)

### SUMMARY
Development and implementation of an autonomous differential mobile robot capable of navigating and mapping unkown terrain, identifying and displaying the location of pests and the temperature and humidity of the environment. Also, when the user requires the robot to come back home, it is capable of planning and following an optimal route.

### KEYWORDS:
Differential Mobile Robot, Plague Detection, Route Planning, Control, Kalman Filter.  

### PROBLEMATIC:
The problematics that this project is trying to solve are:
* Agriculture generates 70% of economic activity.
* 10% of GDP comes from the agricultural sector.
* 40% of agricultural global production is lost due to pests.

### GOALS:
* Terrain Exploration
* Plague Detection
* Return to Home

### HARDWARE
* Differential Mobile Robot
* 2 Motors with Encoders
* Jetson Nano
* LiDAR
* ESP32
* USB Webcam
* DHT11 sensor (temperature & humidity)

### GMAPPING & OBSTACLE AVOIDANCE
The LiDAR and the odometry of the robot (obtained with the encoders) were used to create the environment's map in real time. Also, we developed a safe distance control, which allowed the robot to move freely around the environment avoiding any obstacles. 

### PATH:
To find the optimal route from whereever the robot is to "home" we used the RRT* algorithm. We also applied post processing algorithm to smooth the turn. 

### NEURAL NETWORK:
#### Description:
* +5000 images
* Detected plants such a Beens, Strawberries, and Tomato.
* 50 epochs
* 89% of precision
* Trained with Yolov8
* ONNX

### CONFUSION MATRIX:
![image](https://github.com/user-attachments/assets/9c85f72a-af69-4b13-a2c9-5f310b31ebfc)

### PEST MARKERS AND TEMPERATURE:
For pest detection, we used a neural network, it was trained with Yolov8 and optimized with an ONNX type file. It detects the pest with the camera, subscribes to the odometry of the robot and adds a point on the map of the
place where it was found.

Temperature and humidity are measured every 4 seconds and are shared over a TCP/IP connection from the ESP32 to the Jetson Nano, upon receiving the data, the robot checks the odometry values ​​and displays the temperature and humidity  data within the map.

### NODES CONNECTION DIAGRAM:
![image](https://github.com/user-attachments/assets/2bd7c4c1-dd85-4483-bf28-7a24372bdc36)


### Results:
[![Agrobot](https://img.youtube.com/vi/uEOCHyL1Ubg/0.jpg)](https://www.youtube.com/watch?v=uEOCHyL1Ubg)

https://www.youtube.com/watch?v=uEOCHyL1Ubg
