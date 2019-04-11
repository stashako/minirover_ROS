#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mini_rover/imInfo.h>
#include <serial/serial.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>

#define INVALID_VAL 9999

using std::string;

class RoverDriver
{
public:
  RoverDriver();
  serial::Serial s_port_adaControl; // mini_rover control
  serial::Serial s_port_adaCblControl; //cable management control
  serial::Serial s_port_adaSensor;
  void readSensorReadings();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void imInfoCallback(const mini_rover::imInfo::ConstPtr& imInfo);
  bool openSerialPort(serial::Serial &s, const string &port, uint32_t baudrate);
  void cableControl(string cmdStr, string msgStr);
  
  string num2cmdstr(float cmd_val);
  string chomp(string &str, const string token);
  string chompLast(string &str, const string token);

  ros::NodeHandle nh_;

  int FBwards_, LRight_, uBtn_, uBtn_val, dBtn_, dBtn_val, autoBtn_, autoBtn_val, cableCtrlBtn_, cableCtrlBtn_val;
  float kp,kd,ki,error,pre_error,integral, int_min, int_max, loadThres;
  double LR_scale, FB_scale;
  string gauageReading;
  ros::Subscriber joy_sub_, imInfo_sub;
  double pre_time;

  mini_rover::imInfo imInfo_msg;
};


RoverDriver::RoverDriver():
  FBwards_(1),
  LRight_(0),
  uBtn_(3),
  uBtn_val(0),
  dBtn_(0),
  dBtn_val(0),
  autoBtn_(5),
  autoBtn_val(0),
  cableCtrlBtn_(8),
  cableCtrlBtn_val(0),
  kp(1.0),
  kd(0.0),
  ki(10),
  error(0.0),
  pre_error(320.0),
  int_min(0.0),
  int_max(0.0),
  loadThres(15.0)
{

    //make sure the serial port is closed
  if (s_port_adaControl.isOpen())
    s_port_adaControl.close();
  if (s_port_adaCblControl.isOpen())
    s_port_adaCblControl.close();
  if (s_port_adaSensor.isOpen())
    s_port_adaSensor.close();

  //driver initialization and connection
  //obtain serial port and baudrate from the parameter server
  std::string serial_port_;
  int baudrate_;

  //get the control serial 
  if (!nh_.getParam("/rover_params/s_port_adaControl",serial_port_))
  {
    serial_port_ = "/dev/ttyUSB0";
  }
  if (!nh_.getParam("/rover_params/baudrate_adaControl",baudrate_))
  {
    baudrate_ = 115200;
  }
  //only proceed if connection is successful with the COM port
  if (openSerialPort(s_port_adaControl, serial_port_,baudrate_))
  {
    ROS_INFO("rover control serial port open, port:%s, baudrate:%d",serial_port_.c_str(),baudrate_);
  }
  else
  {
    ROS_ERROR("rover control serial port failed to open, exit!");
    ros::shutdown();
  }

  //get the cable control serial 
  if (!nh_.getParam("/rover_params/s_port_adaCblControl",serial_port_))
  {
    serial_port_ = "/dev/ttyUSB0";
  }
  if (!nh_.getParam("/rover_params/baudrate_adaCblControl",baudrate_))
  {
    baudrate_ = 115200;
  }
  //only proceed if connection is successful with the COM port
  if (openSerialPort(s_port_adaCblControl, serial_port_,baudrate_))
  {
    ROS_INFO("Cbl control serial port open, port:%s, baudrate:%d",serial_port_.c_str(),baudrate_);
  }
  else
  {
    ROS_ERROR("Cbl control serial port failed to open, exit!");
    //ros::shutdown(); cable manamgement control is optional
  }

  //get the sensor serial
  if (!nh_.getParam("/rover_params/s_port_adaSensor",serial_port_))
  {
    serial_port_ = "/dev/ttyACM0";
  }
  if (!nh_.getParam("/rover_params/baudrate_adaSensor",baudrate_))
  {
    baudrate_ = 115200;
  }
  //only proceed if connection is successful with the COM port
  if (openSerialPort(s_port_adaSensor, serial_port_,baudrate_))
  {
    ROS_INFO("sensor serial port open, port:%s, baudrate:%d",serial_port_.c_str(),baudrate_);
  }
  else
  {
    ROS_ERROR("Sensor serial port failed to open, no sensor available");
    //ros::shutdown();
  }

  //parameter setting
  nh_.param("/rover_params/axis_FBwards", FBwards_, FBwards_);
  nh_.param("/rover_params/axis_LRight", LRight_, LRight_);
  nh_.param("/rover_params/button_up", uBtn_, uBtn_);
  nh_.param("/rover_params/button_down", dBtn_, dBtn_);
  nh_.param("/rover_params/button_auto", autoBtn_, autoBtn_);
  nh_.param("/rover_params/scale_FBwards", FB_scale, FB_scale);
  nh_.param("/rover_params/scale_LRight", LR_scale, LR_scale);
  nh_.param("/rover_params/pid_kp", kp, kp);
  nh_.param("/rover_params/pid_ki", ki, ki);
  nh_.param("/rover_params/pid_kd", kd, kd);
  nh_.param("/rover_params/pid_intMin", int_min, int_min);
  nh_.param("/rover_params/pid_intMax", int_max, int_max);
  nh_.param("/rover_params/load_threshold", loadThres, loadThres);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &RoverDriver::joyCallback, this);
  imInfo_sub = nh_.subscribe<mini_rover::imInfo>("/rover_image/imInfo",1,&RoverDriver::imInfoCallback, this);

  pre_time = ros::Time::now().toSec();
}

void RoverDriver::imInfoCallback(const mini_rover::imInfo::ConstPtr& imInfo)
{ 
  imInfo_msg.x_im_midpt = imInfo->x_im_midpt;
  imInfo_msg.x_midpt_frm_lines = imInfo->x_midpt_frm_lines;
  imInfo_msg.x_im_intercept_l = imInfo->x_im_intercept_l;
  imInfo_msg.x_im_intercept_r = imInfo->x_im_intercept_r;
}

void RoverDriver::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //grab the reading of the semi-autonomous mode button
  autoBtn_val = joy->buttons[autoBtn_];
  uBtn_val    = joy->buttons[uBtn_];
  dBtn_val    = joy->buttons[dBtn_];

  // toggle the cableCtrlBtn_val if button press detected. 
  if (joy->buttons[cableCtrlBtn_] == 1 && ros::Time::now().toSec() - pre_time > 1)
  {
    pre_time = ros::Time::now().toSec();
    if (cableCtrlBtn_val == 0) 
    {
       ROS_INFO("cable control on...");
       cableCtrlBtn_val = 1;
    }
    else if (cableCtrlBtn_val == 1) 
    {
       ROS_INFO("cable control off... ");
       cableCtrlBtn_val = 0;
    }
  }


  string tmp_str = "O|";

  float FB_val = FB_scale * joy->axes[FBwards_];
  float LR_val = LR_scale * joy->axes[LRight_]; // reverse the sign with right -> pos  

  if (autoBtn_val == 0)
  {
    tmp_str.append(num2cmdstr(FB_val));
    tmp_str.append("|");
    tmp_str.append(num2cmdstr(LR_val));
    tmp_str.append("|");

    integral = 0.0;
    pre_error = imInfo_msg.x_im_midpt;
  }
  else
  {
    if(imInfo_msg.x_midpt_frm_lines == INVALID_VAL || imInfo_msg.x_im_midpt == 0)
    {
      ROS_WARN("Failed to estimate lines| %d | %d",imInfo_msg.x_midpt_frm_lines, imInfo_msg.x_im_midpt);
      tmp_str.append(num2cmdstr(0));
      tmp_str.append("|");
      tmp_str.append(num2cmdstr(0));
      tmp_str.append("|");
    }
    else
    {
      //trying out PID control
      error = imInfo_msg.x_im_midpt - imInfo_msg.x_midpt_frm_lines;
      integral += error;
      if(integral > int_max)
      {
        integral = int_max;
      }
      else if (integral < int_min)
      {
        integral = int_min;
      }
      float output = kp*error + ki*integral + kd*(pre_error - imInfo_msg.x_midpt_frm_lines);  
      ROS_INFO("|%.3f %.3f %.3f %.3f %.3f %.3f %d %d %.3f",kp,ki,kd,error,integral,pre_error - imInfo_msg.x_midpt_frm_lines, imInfo_msg.x_midpt_frm_lines,imInfo_msg.x_im_midpt,FB_val);
      //ROS_INFO("line=%d, midpt=%d, output=%f",imInfo_msg.x_midpt_frm_lines,imInfo_msg.x_im_midpt,output);
      
      //limit forward backward control by 0.2

      /*if (FB_val > 0)
      {
        tmp_str.append(num2cmdstr(0.15));
        tmp_str.append("|");

        //deal with left or right
        tmp_str.append(num2cmdstr(output));
        tmp_str.append("|");
      }
      else if (FB_val < 0)
      {
        tmp_str.append(num2cmdstr(-0.15));
        tmp_str.append("|");
      
        tmp_str.append(num2cmdstr(output));
        tmp_str.append("|");
      }
      else
      {*/
        float lim = 0.35;
        if(FB_val > lim)FB_val = lim;
        else if (FB_val < -lim)FB_val = -lim;
        if(output > lim)output = lim;
        else if (output < -lim)output = -lim;
	      tmp_str.append(num2cmdstr(FB_val));
        tmp_str.append("|");
      
        tmp_str.append(num2cmdstr(output));
        tmp_str.append("|");
      //}

      //ROS_INFO("ctrl = %f, fb=%f",output,FB_val);

      pre_error =  imInfo_msg.x_midpt_frm_lines;
    }
  }

  if(uBtn_val == 1 || dBtn_val == 1)
  {
    if(uBtn_val == 1 && dBtn_val == 0) 
    { tmp_str.append(num2cmdstr(-1));}
    else if(uBtn_val == 0 && dBtn_val == 1) 
    { tmp_str.append(num2cmdstr(1));}
    else if(uBtn_val == 1 && dBtn_val == 1)
    { tmp_str.append(num2cmdstr(0));}   
  }
  else if(cableCtrlBtn_val == 1)
  {
    float gReading = atof(gauageReading.c_str());
    if (!gauageReading.empty() && gReading >= loadThres)
      tmp_str.append(num2cmdstr(-1));
    else if (FB_val < 0 && gReading < loadThres)
      tmp_str.append(num2cmdstr(1));
    else
      tmp_str.append(num2cmdstr(0));
  }
  else
    tmp_str.append(num2cmdstr(0));
  
  cableControl(tmp_str, "joyCallback");

}

 void RoverDriver::cableControl(string cmdStr, string msgStr)
 {
    ROS_INFO("ada_crtl_str:%s",cmdStr.c_str());  

    //send to rover control
    if (s_port_adaControl.isOpen())
    {
      int n_byte = s_port_adaControl.write(cmdStr.c_str());
      if (n_byte <= 0)
        {
            msgStr.append("::rover control Failed to write data to the serial port");
            ROS_ERROR("%s",msgStr.c_str());
        }

    }else
    {
      msgStr.append(":: rover control Serial port is not open");
      ROS_ERROR("%s",msgStr.c_str());
    }

    //send to cable control
    if (s_port_adaCblControl.isOpen())
    {
      int n_byte = s_port_adaCblControl.write(cmdStr.c_str());
      if (n_byte <= 0)
        {
            msgStr.append("::cable control Failed to write data to the serial port");
            ROS_ERROR("%s",msgStr.c_str());
        }

    }else
    {
      msgStr.append(":: cable control Serial port is not open");
      ROS_ERROR("%s",msgStr.c_str());
    }
 }

void RoverDriver::readSensorReadings()
{
  string str = s_port_adaSensor.readline();
  ROS_INFO("sensorReading:%s",str.c_str());

  string s_tmp = chomp(str, "|");

  // according to RoverBox.ino 
  if(s_tmp == "C")
  {
    //process cable length and strain gauge readings.
    chomp(str,","); // return the cable length value
    gauageReading = str; // remain string is the gauge value
  }
  
}

string RoverDriver::num2cmdstr(float cmd_val)
{
  float num = 0;
  string tmp_str;
  //check for neg or pos
  if (cmd_val <0.0)
  {
    num = cmd_val*-1; //neg
    tmp_str = "-";
  }
  else
  {
    num = cmd_val;
   tmp_str = "+"; 
  }

  //take the first two digits
  num = (num + 0.005) * 100; //round to the closest
  std::stringstream ss;

  ss << std::setw(3) << std::setfill('0') << (int)num;
  return tmp_str.append(ss.str());
}

/**
 * Function to return the string before the token, exclusive of the token.
 */
string RoverDriver::chomp(string &str, const string token)
{
  size_t n_pos = string::npos;
  size_t s_pos = str.find(token);// will return string::npos if not found

  if (s_pos != n_pos)
  {
    string r_str;
    r_str = str.substr(0,s_pos);
    str.erase(0,s_pos+token.length());
    return r_str;
  }
  else
  {
    string tmp_str = str;
    str = "";

    return tmp_str;
  }
}


/**
 * Function to return the string before the last instance of token, exclusive of the token.
 */
string RoverDriver::chompLast(string &str, const string token)
{
  size_t n_pos = string::npos;
  size_t s_pos = str.find_last_of(token);// will return string::npos if not found

  if (s_pos != n_pos)
  {
    string r_str;
    r_str = str.substr(0,s_pos);
    str.erase(0,s_pos+token.length());
    return r_str;
  }
  else
  {
    string tmp_str = str;
    str = "";

    return tmp_str;
  }
}

bool RoverDriver::openSerialPort(serial::Serial &s, const string &port, uint32_t baudrate)
{
  //make sure port is not empty
  if (!port.empty())
  {
    s.setPort(port);
  }
  else
  {
    ROS_ERROR("serial port is not assigned");
    return false;
  }

  //set baudrate
  s.setBaudrate(baudrate);
  //set timeouts
  s.setTimeout(serial::Timeout::max(), 250, 0, 250, 0);

  //try to open the port
  if (!s.isOpen())
  {
    ROS_INFO("opening serial port on port %s, and baud rate %d",port.c_str(),baudrate);
    try{ 
        s.open();
    }
    catch (serial::IOException e){
        ROS_ERROR("serial::IOException:%s",e.what());
    }
  }

  if (!s.isOpen())
  {
    ROS_ERROR("serial port open failed !");
    return false;
  }else
    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rover_driver");
  RoverDriver rover_driver;

  ros::Rate r(20);
  //ros::spin();
  while(ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if(rover_driver.s_port_adaSensor.available() != 0)
        rover_driver.readSensorReadings();
  }
}
