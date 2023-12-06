# dynamixel_controller_pkg

###### This package is based on ROS2 for controlling Robotis DYNAMIXEL motors.
###### You may need the DYNAMIXEL SDK(URL = https://github.com/ROBOTIS-GIT/DynamixelSDK.git)
```js
pip3 install dynamixel-sdk
```

## Usb access
```js
sudo chmod 777 /dev/ttyUSB0
```

## Program Description
###### DYNAMIXEL_Controller_v1.py
######     - It subscribes to a topic named motor_speed, which is used to control a single motor or multiple motors.
###### Control_Example_Single.py
######     - It publishes a message to a topic called motor_speed, which is used to test the control of a single motor.
###### Control_Example_Two.py
######     - It publishes a message to a topic called dual_motor_speed, which is used to test the control of the two motors.


## If you only have to control one motor
###### STEP 1 : Modify the program "DYNAMIXEL_Controller_1.0.py"
```js
# Old
dynamixel_controller = DualMotorController()

>>>
# New
dynamixel_controller = SingleMotorController()

```

###### STEP 2 : USE Control_Example_Single.py
```js
source install/setup.bash
python3 src/dynamixel_controller_v1/Control_Example_Single.py

# Add a new command window
source install/setup.bash
python3 src/dynamixel_controller_v1/DYNAMIXEL_Controller_v1.py

```
