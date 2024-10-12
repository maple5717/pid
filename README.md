# pid
PID controller for ROS2

Modification: 
1. disable all the nodes except for ```controller.cpp``` and ```pid.cpp```
2. PID will output only when plant feedback is received (since timer doesn't work) 
3. Disabled error filtering (but still keeps derivative filtering)

### setup BioTac
rosrun biotac_sensors biotac_pub 
### setup calibration
rosrun biotac_logger biotac_logger_v3.py 
### in another terminal, type
rostopic pub -1 /biotac_sub std_msgs/String "calibrate"

### setup tuned PID
roslaunch pid tactile_pid.launch
### run helper. In pid_helper set goal
rosrun pid pid_helper.py _input:=input _output:=output


### set robotiq rtu timeout to 0.00465
open robotiq/robotiq_modbus_rtu/src/robotic_modbus_rtu.py
self.client = ModbusSerialClient(method='rtu',port=device,stopbits=1, bytesize=8, baudrate=115200, timeout=0.00465)
