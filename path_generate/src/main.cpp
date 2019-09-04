#include <ros/ros.h>
#include "Agent.h"
#include <iostream>
#include <fstream>
#include "Includes.h"
#include <time.h>
#include <mavros_msgs/PositionTarget.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace boost::asio;

#define random(a,b) (((double)rand()/RAND_MAX)*(b-a)+a)
#define REV_LEN 41
#define SEND_LEN 17

#define BYTE3(data)	(uint8_t)((data&0xff000000)>>24)
#define BYTE2(data)	(uint8_t)((data&0x00ff0000)>>16)
#define BYTE1(data)	(uint8_t)((data&0x0000ff00)>>8)
#define BYTE0(data)	(uint8_t)((data&0x000000ff))

#define pi 3.141592654

vector<double> waypoint[3];
int minradius = 2;
fstream f;

double ref_latitude = 34.43833621;
double ref_longitude = 108.73974429;
double ref_height = 30.0;

void InitializeAgent(AgentController*& _agent){
  _agent = new AgentController;
  //AgentController *_agent;
  AgentState _goal;
  _goal.pos = make_pair(0.0,0.0);
  _goal.theta = 0.0;
  cout<<"here zhx"<<endl;
  //set position
  //unit x, y, n*PI
  //_agent->SetState(setQuery(0.0, 0.0 ,0.0));
  //_agent->SetGoal(setQuery(10.0, 10.0, PI));
  
}
vector<double> goal[3];
void set_waypoint(double lat_add, double lon_add, double hei_add){
    // beyond the code 's reference is ref_latitude

    double input_Lat, input_Lon, input_Hei;
    input_Lat = ref_latitude + lat_add/(1000*111);
    input_Lon = ref_longitude + lon_add/(1000*(111*cos(ref_latitude/180*pi)));

    input_Hei = ref_height +hei_add;

    goal[0].push_back(input_Lat);
    goal[1].push_back(input_Lon);
    goal[2].push_back(input_Hei);
//    waypoint[0].push_back(goal[0][0]);
//    waypoint[1].push_back(goal[1][0]);
//    waypoint[2].push_back(goal[2][0]);
    
}

void set_current_waypoint(double lat_add, double lon_add, double hei_add,
                          double cur_lat, double cur_lon, double cur_hei){
    //according to the current location(GPS waypoint)
    //current GPS must be got at first
    ref_latitude = cur_lat;
    ref_longitude = cur_lon;
    ref_height = cur_hei;

    double input_Lat, input_Lon, input_Hei;
    input_Lat = ref_latitude + lat_add/(1000*111);
    input_Lon = ref_longitude + lon_add/(1000*(111*cos(ref_latitude/pi)));
    input_Hei = ref_height +hei_add;

    goal[0].push_back(input_Lat);
    goal[1].push_back(input_Lon);
    goal[2].push_back(input_Hei);
//    waypoint[0].push_back(goal[0][0]);
//    waypoint[1].push_back(goal[1][0]);
//    waypoint[2].push_back(goal[2][0]);


}


bool generate_random_start_end(int sum){
    //generate random number seed 
    AgentController* agent = NULL;
    InitializeAgent(agent);
    
    srand((int)time(0));
    vector<float> start[3],end[3];
    for(int i=0; i<sum; i++){
        start[0].push_back(random(0,30));
        start[1].push_back(random(0,30));
        start[2].push_back(random(0,3.14));
        
        end[0].push_back(random(0,30));
        end[1].push_back(random(0,30));
        end[2].push_back(random(0,3.14));
    }
    //generate path
    extern float path_length;
    vector<float> path_len;
    for(int i=0; i< start[0].size(); i++){
        agent->SetState(setQuery(start[0][i], start[1][i] , start[2][i] ));
        agent->SetGoal(setQuery(end[0][i], end[1][i] , end[2][i]));
         while(agent->Update());
        path_len.push_back(path_length);
    }
    //print result
    for(int i=0; i<path_len.size(); i++){
        cout<<path_len[i]<<" ";
        cout<<"start: "<<start[0][i]<<" "<<start[1][i]<<" "<<start[2][i]<<" ";
        cout<<"end: "<<end[0][i]<<" "<<end[1][i]<<" "<<end[2][i]<<" ";
        cout<<endl;
    }
    cout<<endl;
    //output
//    fstream f_len;
//    f_len.open("/home/zhx/catkin_ws/src/path_generate/src/length.txt");
    
//    for(int i=0; i<path_len.size(); i++){
        
//        f_len<<start[0][i]<<" "<<start[1][i]<<" "<<start[2][i]<<" ";
//        f_len<<end[0][i]<<" "<<end[1][i]<<" "<<end[2][i]<<" ";
//        f_len<<path_len[i]<<" ";
//        f_len<<endl;
//    }
    
//    f_len.close();
    
    return true;
    
}

bool analysis_Data(unsigned char feedback[REV_LEN], std_msgs::Float32MultiArray& State_data_Store){

    int feedback_int[REV_LEN];
    for(int i=0; i<REV_LEN; ++i)
        feedback_int[i] = (int)feedback[i];

    int am[15];
    am[0]=(((feedback_int[4]*256+feedback_int[5])*256+feedback_int[6])*256+feedback_int[7]);
    am[1]=(((feedback_int[8]*256+feedback_int[9])*256+feedback_int[10])*256+feedback_int[11]);
    am[2]=(((feedback_int[12]*256+feedback_int[13])*256+feedback_int[14])*256+feedback_int[15]);

    am[3]=feedback_int[16]*256+feedback_int[17];
    am[4]=feedback_int[18]*256+feedback_int[19];
    am[5]=feedback_int[20]*256+feedback_int[21];

    am[6]=feedback_int[22]*256+feedback_int[23];
    am[7]=feedback_int[24]*256+feedback_int[25];
    am[8]=feedback_int[26]*256+feedback_int[27];

    am[9]=feedback_int[28]*256+feedback_int[29];
    am[10]=feedback_int[30]*256+feedback_int[31];
    am[11]=feedback_int[32]*256+feedback_int[33];

    am[12]=feedback_int[34]*256+feedback_int[35];
    am[13]=feedback_int[36]*256+feedback_int[37];
    am[14]=feedback_int[38]*256+feedback_int[39];

    for(int l=3;l<=14;l++)
    {
        if (am[l]>32767)
        {
          am[l]=am[l]-65536;
        }
    }

    double latitude_analy = double(am[0])/10000000.0;
    double longitude_analy = double(am[1])/10000000.0;
    double height_analy = double(am[2])/100.0;

    double vel_north = double(am[3])/100.0;
    double vel_east = double(am[4])/100.0;
    double vel_ground = double(am[5])/100.0;

    double ang_roll = double(am[6])/100.0;
    double ang_pitch = double(am[7])/100.0;
    double vel_yaw = double(am[8])/10.0;

    double ang_x = double(am[9])/10.0;
    double ang_y = double(am[10])/10.0;
    double ang_z = double(am[11])/10.0;

    double acc_x = double(am[12])/100.0;
    double acc_y = double(am[13])/100.0;
    double acc_z = double(am[14])/100.0;

    //print
    cout<<"print state !"<<endl;
    cout<<"latitude_analy = "<<setprecision(11)<<latitude_analy<<endl;
    cout<<"longitude_analy = "<<setprecision(11)<<longitude_analy<<endl;
    cout<<"height_analy = "<<setprecision(11)<<height_analy<<endl;
    cout<<endl;

    cout<<"vel_north = "<<vel_north<<endl;
    cout<<"vel_east = "<<vel_east <<endl;
    cout<<"vel_ground = " << vel_ground<<endl;cout<<endl;

    cout<<"ang_roll = " <<ang_roll<<endl;
    cout<<"ang_pitch = " <<ang_pitch<<endl;
    cout<<"vel_yaw = " <<vel_yaw<<endl;cout<<endl;

    cout<<"ang_x = " <<ang_x<<endl;
    cout<<"ang_y = " <<ang_y<<endl;
    cout<<"ang_z = " <<ang_z<<endl;cout<<endl;

    cout<<"acc_x = " <<acc_x<<endl;
    cout<<"acc_y = " <<acc_y<<endl;
    cout<<"acc_z = " <<acc_z<<endl;cout<<endl;

    //convert

    State_data_Store.data.push_back(latitude_analy);
    State_data_Store.data.push_back(longitude_analy);
    State_data_Store.data.push_back(height_analy);
    State_data_Store.data.push_back(vel_north);
    State_data_Store.data.push_back(vel_east);
    State_data_Store.data.push_back(vel_ground);

    State_data_Store.data.push_back(ang_roll);
    State_data_Store.data.push_back(ang_pitch);
    State_data_Store.data.push_back(vel_yaw);

    State_data_Store.data.push_back(ang_x);
    State_data_Store.data.push_back(ang_y);
    State_data_Store.data.push_back(ang_z);

    State_data_Store.data.push_back(acc_x);
    State_data_Store.data.push_back(acc_y);
    State_data_Store.data.push_back(acc_z);

    return true;
}

string sender_Write(double latitude, double longitude, double height){

    uint8_t data_send[SEND_LEN];
    data_send[0] = (uint8_t)170;
    data_send[1] = (uint8_t)175;
    data_send[2] = (uint8_t)225;
    data_send[3] = (uint8_t)12;
    //point
    int32_t lat = latitude * 10000000.0;
    data_send[4] = BYTE3(lat);
    data_send[5] = BYTE2(lat);
    data_send[6] = BYTE1(lat);
    data_send[7] = BYTE0(lat);

    int32_t lon = longitude * 10000000.0;
    data_send[8] = BYTE3(lon);
    data_send[9] = BYTE2(lon);
    data_send[10] = BYTE1(lon);
    data_send[11] = BYTE0(lon);

    int32_t hei = height * 100.0;
    data_send[12] = BYTE3(hei);
    data_send[13] = BYTE2(hei);
    data_send[14] = BYTE1(hei);
    data_send[15] = BYTE0(hei);

    data_send[16] = 0;
    for(int i=0; i<16; i++)
        data_send[16] = data_send[16] + data_send[i];
//    cout<<"sum = "<<sum<<endl;
//    data_send[16] = sum % 256;
//    data_send[16] = data_send[16];

    string sender_;
    for(int i=0; i<SEND_LEN; ++i)
    {
        sender_+= (uint8_t)data_send[i];
//        cout<<(int)data_send[i]<<',';
    }
    cout<<endl;

    return sender_;
}

int main ( int argc , char ** argv ) 
{
  ros::init ( argc , argv , "hello" ) ;
  ros::NodeHandle nh ;
  ROS_INFO_STREAM( " Hello , ROS! " ) ;

  ros::Publisher set_pose_pub_ = nh.advertise<mavros_msgs::PositionTarget>("path_generate/set_pose", 10);
  ros::Publisher airplane_state_pub_ = nh.advertise<std_msgs::Float32MultiArray>("path_generate/Airplane_State", 10);
  srand(static_cast<unsigned int>(time(NULL)));
//  f.open("/home/nuc-fixedwing/catkin_ws/src/path_generate/src/dataint.txt");
  
//read data from flight controller
  io_service iosev;
  serial_port sp(iosev,"/dev/ttyUSB0");//定义传输的串口
  sp.set_option(serial_port::baud_rate(115200));
  sp.set_option(serial_port::flow_control());
  sp.set_option(serial_port::parity( serial_port::parity::none ));
  sp.set_option(serial_port::stop_bits( serial_port::stop_bits::one ));
  sp.set_option(serial_port::character_size(8));
//start 0xaa 0xaf 0xe1 0x0c
//170 175 225 12

  AgentController* agent = NULL;
  InitializeAgent(agent);
  
  //**********************set point********************
  // please input fixed coordinate
  set_waypoint(2000,-2000,70);
  set_waypoint(-2000,-2000,70);
//  set_waypoint(0,0,50);
//  set_waypoint(1000,2000,50);
  //update AgentStates
  //generate path

  ros::Rate loop_rate(50);

  int read_cnt = 0;
  unsigned char data_fromfc[REV_LEN];
  unsigned char data_fromfc_last[REV_LEN];
  unsigned char data_fromfc_order[REV_LEN];
  static int cnt = 0;

  double latitude = 34.43833621;
  double longitude = 108.73974429;
  double height = 30.0;


  while(ros::ok()){

      cout<<"read!"<<endl;

      //read new
      read(sp,buffer(data_fromfc));
      int record_pos;
      for(int i=0; i<REV_LEN; i++){
          if((int)data_fromfc[i] == 170 && (int)data_fromfc[i+1] == 175)
          {
              record_pos = i;
//              cout<<"position of data head = "<<i<<endl;
              break;
          }
      }
      //order it
      for(int i=record_pos; i<REV_LEN; i++){
          data_fromfc_order[i-record_pos] = data_fromfc_last[i];
      }
      for(int i=0; i<record_pos; i++){
          data_fromfc_order[REV_LEN - record_pos + i] = data_fromfc[i];
      }
      //remain last
      for(int i=0; i<REV_LEN; i++){
          data_fromfc_last[i] = data_fromfc[i];
      }
      for(int i=0; i<REV_LEN; i++){
          cout<<data_fromfc_order[i]<<" ";
      }
//      f<<endl;
    cout<<endl;

    //analysis the data
    std_msgs::Float32MultiArray State_data_Store;
    if(analysis_Data(data_fromfc_order, State_data_Store)) cout<<"analysis func ok!"<<endl;
    airplane_state_pub_.publish(State_data_Store);
//    cout<<"length = "<<State_data_Store.data.size()<<endl;

    //points update

    if(fabs(State_data_Store.data[0] - goal[0][cnt])<(0.1/111) &&
            fabs(State_data_Store.data[1] - goal[1][cnt])<(0.1/(111*cos(ref_latitude/180*pi))))
        cnt++;
    if(cnt >= goal[0].size()) cout<<"path end"<<endl;
    else{
        latitude = goal[0][cnt];
        longitude = goal[1][cnt];
        height = goal[2][cnt];
    }

    cout<<"the number of mission waypoint = "<<goal[0].size()<<endl;
    cout<<"cnt(current waypoint reached) : "<<cnt<<endl;
    cout<<"current goal waypoint in GPS = "<<setprecision(11)<<latitude<<" "<<longitude<<' '<<height<<endl;
//write
    string sender_ = sender_Write(latitude, longitude, height);
    if(!sender_.empty()) cout<<"sender is sent!"<<endl;
//    cout<<"sender_ = "<<sender_<<endl;
    write(sp,buffer(sender_,sender_.size()));
//write end
    cout<<"*************loop end*****************"<<endl;
    cout<<endl;
  }



  return 0;//everything is ok now.

  for(int i=0; i< goal[0].size()-1; i++){
    agent->SetState(setQuery(waypoint[0][waypoint[0].size()-1], waypoint[1][waypoint[0].size()-1] , waypoint[2][waypoint[0].size()-1] ));
    agent->SetGoal(setQuery(goal[0][i+1], goal[1][i+1] , goal[2][i+1]));
    while(agent->Update());
    
  }
  //print decent 
  int decent = 25;
  //if(print(decent)) cout<<"print ok!"<<endl;
  //f.close();
  // other func
  if(generate_random_start_end(100000))  cout<<"generate_random_start_end(10) run ok"<<endl;
  
  
  //close program
  delete agent;
  agent  = NULL;
  return 0;
}


