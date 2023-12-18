#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <math.h>
// #include <std_msgs/Int32MultiArray.h>
// #include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <Wire.h>
#include "FXOS8700Q_Basic.h"
#include "MadgwickAHRS.h"
#include "MS5837.h"
#include <mpu6050.hpp>
#include <geometry_msgs/Quaternion.h>
#define g 9.80665


#define ACCELEROMETER_MAGNETOMETER_ADDRESS 0x1F
#define GYROSCOPE_ADDRESS 0x68
#define UPDATE_RATE 10
#define PUBLISH_RATE 100
#define NO_OF_SAMPLES 100

float raw_a[3];
float raw_g[3];
float raw_m[3];
float cal_a[3];
float cal_g[3];
float cal_m[3];
float _ax_offset = 0.0, _ay_offset = 0.0, _az_offset = 0.0;
float _gx_offset = 0.0, _gy_offset = 0.0, _gz_offset = 0.0;
float _mx_offset = 59.94, _my_offset = 89.26, _mz_offset = 75.24;
float soft_iron_matrix[3][3]={{0.989,-0.026,-0.013},{-0.026,0.963,-0.009},{-0.013,-0.009,1.052}};
// float calliberated_m_matrix[3][1];
float cal_m_matrix[1][3];
float yaw, pitch, roll, depth;
float qx,qy,qz,qw;
int prev_time_update, prev_time_publish;

geometry_msgs::Vector3 linear_acceleration;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Vector3 magnetic_field;
geometry_msgs::Pose pose;

// std_msgs::Float32MultiArray orientation_depth_data;
// std_msgs::Float32MultiArray raw_IMU_data;
// std_msgs::Float32MultiArray calliberated_IMU_data;

ros::NodeHandle nh;
ros::Publisher pub1("linear_acceleration", &linear_acceleration);
ros::Publisher pub2("angular_velocity", &angular_velocity);
ros::Publisher pub3("magnetic_field", &magnetic_field);
ros::Publisher pub4("pose", &pose);
// ros::Publisher pub0("raw_IMU_data", &raw_IMU_data);
// ros::Publisher pub1("calliberated_IMU_data", &calliberated_IMU_data);
// ros::Publisher pub2("orientation_depth_data", &orientation_depth_data);

MPU6050 gyro;
FXOS8700QBasic Accelerometer_Magnetometer;
MS5837 Depth_Sensor;
Madgwick Filter;


void setup() 
{

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Serial.begin(9600);

  while(!Serial)
    delay(1);

  // bool status;
  // Wire.beginTransmission(0x68);
  // status=Wire.endTransmission(true);
  // if(status)
  // Serial.println("Failed");
  // else
  // Serial.println("Success");

    gyro.begin();
    gyro.setAccelerometerRange(ACCELERO_METER_RANGE_2);
    gyro.setGyroscopeRange(GYROSCOPE_RANGE_250);
    gyro.setSampleRateDivider(0);
    gyro.disableSleepMode();  

  nh.initNode();

  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.advertise(pub4);
  Accelerometer_Magnetometer = FXOS8700QBasic(1, ACCELEROMETER_MAGNETOMETER_ADDRESS, &Wire);

  Depth_Sensor.init(&Wire);
  Depth_Sensor.setFluidDensity(997);
  
  for (int  sample_no = 0; sample_no < NO_OF_SAMPLES ; sample_no++)
    {
         gyro.getSensorsReadings(raw_a[0], raw_a[1], raw_a[2], raw_g[0], raw_g[1], raw_g[2],false);
        _ax_offset += raw_a[0];
        _ay_offset += raw_a[1];
        _az_offset += raw_a[2];
        _gx_offset += raw_g[0];
        _gy_offset += raw_g[1];
        _gz_offset += raw_g[2];
        delay(50);
    }
    _ax_offset /= NO_OF_SAMPLES;
    _ay_offset /= NO_OF_SAMPLES;
    _az_offset /= NO_OF_SAMPLES;
    _gx_offset /= NO_OF_SAMPLES;
    _gy_offset /= NO_OF_SAMPLES;
    _gz_offset /= NO_OF_SAMPLES;
    _az_offset -= g;

  prev_time_update = millis();
  prev_time_publish = millis();

}

  
void loop() 
{
  if(millis() - prev_time_update >= UPDATE_RATE)
  {
    Depth_Sensor.read();
    Accelerometer_Magnetometer.updateMagData(raw_m);
    gyro.getSensorsReadings(raw_a[0], raw_a[1], raw_a[2], raw_g[0], raw_g[1], raw_g[2]);
    
    cal_a[0]=raw_a[0]-_ax_offset;
    cal_a[1]=raw_a[1]-_ay_offset;
    cal_a[2]=raw_a[2]-_az_offset;

    cal_g[0]=raw_g[0]-_gx_offset;
    cal_g[1]=raw_g[1]-_gy_offset;
    cal_g[2]=raw_g[2]-_gz_offset;

    cal_m[0]=raw_m[0]-_mx_offset;
    cal_m[1]=raw_m[1]-_my_offset;
    cal_m[2]=raw_m[2]-_mz_offset;

    cal_m[0]=cal_m[0]*soft_iron_matrix[0][0]+cal_m[1]*soft_iron_matrix[0][1]+cal_m[2]*soft_iron_matrix[0][2];
    cal_m[1]=cal_m[0]*soft_iron_matrix[1][0]+cal_m[1]*soft_iron_matrix[1][1]+cal_m[2]*soft_iron_matrix[1][2];
    cal_m[2]=cal_m[0]*soft_iron_matrix[2][0]+cal_m[1]*soft_iron_matrix[2][1]+cal_m[2]*soft_iron_matrix[2][2];
     
  

    Filter.update(cal_g[0], cal_g[1], cal_g[2], cal_a[0], cal_a[1], cal_a[2], cal_m[0], cal_m[1], cal_m[2]);
    yaw = Filter.getYaw();
    pitch = Filter.getPitch();
    roll = Filter.getRoll();
    depth = Depth_Sensor.depth();
    prev_time_update = millis();
    
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
  }

    if(millis() - prev_time_publish >= PUBLISH_RATE)
    {
    linear_acceleration.x= cal_a[0];
    linear_acceleration.y= cal_a[1];
    linear_acceleration.z= cal_a[2];

    angular_velocity.x= cal_g[0];
    angular_velocity.y= cal_g[1];
    angular_velocity.z= cal_g[2];

    magnetic_field.x= cal_m[0];
    magnetic_field.y= cal_m[1];
    magnetic_field.z= cal_m[2];

    pose.position.z = depth;
    pose.orientation.x=qx;
    pose.orientation.y=qy;
    pose.orientation.z=qz;
    pose.orientation.w=qw;

    pub1.publish(&linear_acceleration);
    pub2.publish(&angular_velocity);
    pub3.publish(&magnetic_field);
    pub4.publish(&pose);
    prev_time_publish = millis();
    }
    Serial.print("raw_gx: ");
    Serial.print(raw_g[0]);
    Serial.print("raw_gy: ");
    Serial.print(raw_g[1]);
    Serial.print("raw_gz: ");
    Serial.print(raw_g[2]);

    Serial.print("raw_ax: ");
    Serial.print(raw_a[0]);
    Serial.print("raw_ay: ");
    Serial.print(raw_a[1]);
    Serial.print("raw_az: ");
    Serial.print(raw_a[2]);

    Serial.print("raw_mx: ");
    Serial.print(raw_m[0]);
    Serial.print("raw_my: ");
    Serial.print(raw_m[1]);
    Serial.print("raw_mz: ");
    Serial.print(raw_m[2]);
    
    Serial.println(" ");

    Serial.print("cal_gx: ");
    Serial.print(cal_g[0]);
    Serial.print("cal_gy: ");
    Serial.print(cal_g[1]);
    Serial.print("cal_gz: ");
    Serial.print(cal_g[2]);

    Serial.print("cal_ax: ");
    Serial.print(cal_a[0]);
    Serial.print("cal_ay: ");
    Serial.print(cal_a[1]);
    Serial.print("cal_az: ");
    Serial.print(cal_a[2]);

    Serial.print("cal_mx: ");
    Serial.print(cal_m[0]);
    Serial.print("cal_my: ");
    Serial.print(cal_m[1]);
    Serial.print("cal_mz: ");
    Serial.print(cal_m[2]);

    Serial.println(" ");   
    
    Serial.print("yaw:");
    Serial.print(yaw);
    Serial.print("pitch:");
    Serial.print(pitch);
    Serial.print("roll:");
    Serial.print(roll);   
    Serial.println(" ");

    Serial.print("qx: ");
    Serial.print(qx);
    Serial.print("qy: ");
    Serial.print(qy);
    Serial.print("qz: ");
    Serial.print(qz);
    Serial.print("qw: ");
    Serial.println(qw);

    nh.spinOnce();
    delay(10);
  
}


