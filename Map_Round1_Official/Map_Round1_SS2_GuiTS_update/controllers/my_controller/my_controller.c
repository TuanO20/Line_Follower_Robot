#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <webots/supervisor.h>
#define TIME_STEP 8

// 8 IR ground color sensors
#define NB_GROUND_SENS 8
#define NB_LEDS 5
//toc do 24
// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];

// LEDs 
WbDeviceTag led[NB_LEDS];


// Motors
WbDeviceTag left_motor, right_motor;

/* Phần code được phép chỉnh sửa cho phù hợp */
// Định nghĩa các tín hiệu của xe
#define NOP  -1
#define MID   0
#define LEFT  1
#define RIGHT 2
#define FULL_SIGNAL 3
#define BLANK_SIGNAL 4
#define STOP_SIGNAL 5

// Điểu chỉnh tốc độ phù hợp
#define MAX_SPEED 27.5
bool bridge = 0;

// Khai báo biến cho các sensors
unsigned short threshold[NB_GROUND_SENS] = { 300 , 300 , 300 , 300 , 300 , 300 , 300 , 300 };
unsigned int filted[8] = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};

unsigned int pre_filted[8] = {0 , 0 , 0 , 0 , 0 , 0 , 0 , 0};

// Biến lưu giá trị tỉ lệ tốc độ của động cơ
double left_ratio = 0.0;
double right_ratio = 0.0;



//min = 0 , max = 10
void constrain(double *value, double min, double max) {
  if (*value > max) *value = max;
  if (*value < min) *value = min;
}

//Thí sinh không được bỏ phần đọc tín hiệu này
//Hàm đọc giá trị sensors
void ReadSensors(){
  unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
  for(int i=0; i<NB_GROUND_SENS; i++){
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
    // So sánh giá trị gs_value với threshold -> chuyển đổi sang nhị phân
    if (gs_value[i] < threshold[i])
      filted[i] = 1;
    else filted[i] = 0;
  }
}

void Drive(double left, double right)
{
    wb_robot_step(TIME_STEP);
    ReadSensors();
    printf ("\n\t\tPosition : 0b");
    for (int i = 0 ; i < 8 ; i ++)
      printf ("%u" , filted[i] );
    wb_motor_set_velocity(left_motor, left * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right * MAX_SPEED);
}

int Position ()
{
    if ( filted[1] && filted[2] && filted[3] && filted[4] && filted[5] && filted[6])
      return FULL_SIGNAL;
    else if (filted[0] && filted[1] && filted[2] && filted[3] && filted[4] && !filted[5] && !filted[6] && !filted[7])
      return NOP;
    else if (!filted[0] && !filted[1] && !filted[2] && filted[3] && filted[4] && filted[5] && filted[6] && filted[7])
      return NOP;
    if ((filted[3] && filted[4]) || (!filted[2] && filted[3] && !filted[4]) || (!filted[3] && filted[4] && !filted[5]))
      return MID;
    else if ( (filted[0] && filted[1]) || (filted[1] && filted[2]) || (filted[2] && filted[3]))
      return LEFT;
    else if ((!filted[2] && filted[3] && !filted[4]) || (!filted[1] && filted[2] && !filted[3]) || (!filted[0] && filted[1] && !filted[2]) || (filted[0] && !filted[1] && !filted[2]))
      return LEFT;
    else if ( (filted[4] && filted[5]) || (filted[5] && filted[6]) || (filted[6] && filted[7]))
      return RIGHT; 
    else if ((!filted[3] && filted[4] && !filted[5]) || (!filted[4] && filted[5] && !filted[6]) || (!filted[5] && filted[6] && !filted[7]) || (filted[7] && !filted[6] && !filted[5]))
      return RIGHT;
    else if ( !filted[0] && !filted[1] && !filted[2] && !filted[3] && !filted[4] && !filted[5] && !filted[6] && !filted[7])
      return BLANK_SIGNAL;
}

//hàm điều khiểu xe đi thẳng
void GoStraight() {
  if ((!filted[2] && filted[3] && !filted[4]))
  {
    left_ratio = 2 * 0.9;
    right_ratio = 2 * 1.136;
  }
  else if (!filted[3] && filted[4] && !filted[5])
  {
    left_ratio = 2 * 1.136;
    right_ratio = 2 * 0.9;
  }
  else if (filted[3] && filted[4])
  {
    left_ratio = 2.0;
    right_ratio = 2.0;
  }
}

void TurnLeft(){
  printf("Turn Left\n");
  if (filted[0] && filted[1])
  {
    left_ratio = 2 * 0.227;
    right_ratio = 2 * 1.81;
  }
  if (filted[1] && filted[2])
  {
    left_ratio = 2 * 0.54;
    right_ratio = 2 * 1.45;
  }
  if (filted[2] && filted[3])
  {
    left_ratio = 2 * 0.772;
    right_ratio = 2 * 1.227;
  }
  if (!filted[2] && filted[3] && !filted[4])
  {
    left_ratio = 2 * 0.9;
    right_ratio = 2 * 1.136;
  }
  if (!filted[1] && filted[2] && !filted[3])
  {
    left_ratio = 2 * 0.772;
    right_ratio = 2 * 1.18;
  }
  if (!filted[0] && filted[1] && !filted[2])
  {
    left_ratio = 2 * 0.55;
    right_ratio = 2 * 1.45;
  }
  if (filted[0] && !filted[1] && !filted[2])
  {
    left_ratio = 2 * 0.227;
    right_ratio = 2 * 1.81;
  }
}

void TurnRight(){
  printf("Turn Right\n");
  if (filted[4] && filted[5])
  {
    left_ratio = 2 * 1.227;
    right_ratio = 2 * 0.772;
  }
  if (filted[5] && filted[6])
  {
    left_ratio = 2 * 1.45;
    right_ratio = 2 * 0.54;
  }
  if (filted[6] && filted[7])
  {
    left_ratio = 2 * 1.81;
    right_ratio = 2 * 0.227;
  }
  if (!filted[3] && filted[4] && !filted[5])
  {
    left_ratio = 2 * 1.136;
    right_ratio = 2 * 0.9;
  }
  if (!filted[4] && filted[5] && !filted[6])
  {
    left_ratio = 2 * 1.18;
    right_ratio = 2 * 0.772;
  }
  if (!filted[5] && filted[6] && !filted[7])
  {
    left_ratio = 2 * 1.45;
    right_ratio = 2 * 0.55;
  }
  if (filted[7] && !filted[6] && !filted[5])
  {
    left_ratio = 2 * 1.81;
    right_ratio = 2 * 0.227;
  }
}


void TurnLeftCorner()
{
  printf("Turn Left Corner\n");
  for (int i = 0; i < 7; i++)
    Drive(0, 0);
  for (int i = 0; i < 10; i++)
  {
    Drive(-2, 1.5);
    if (Position() != BLANK_SIGNAL)
      break;
  }
  while (Position() == BLANK_SIGNAL)
    Drive(-2, 2.3);
  for (int i = 0; i < 10; i++)
    Drive(0.15, 0);
}

void TurnRightCorner()
{
  printf("Turn Right Corner\n");
  for (int i = 0; i < 7; i++)
    Drive(0, 0);
  for (int i = 0; i < 10; i++)
  {
    Drive(1.5, -2);
    if (Position() != BLANK_SIGNAL)
      break;
  }
  while (Position() == BLANK_SIGNAL)
    Drive(2.3, -2);
  for (int i = 0; i < 10; i++)
    Drive(0, 0.15);
 
}

void LostLine()
{
  printf("Lost Line\n");
  for (int i = 0; i < 2; i++)
    Drive(left_ratio, right_ratio);
  left_ratio = right_ratio = 2.0;
}

int intersection = MID;
int time = 0;
void DetectIntersection()
{
  if (filted[5] && filted[6] && filted[7])
  {
    intersection = RIGHT;
    time = 50;
  }
  else if (filted[0] && filted[1] && filted[2])
  {
    intersection = LEFT;
    time = 50;
  }
  time--;
}

//hàm dừng xe
void Stop()
{
  left_ratio =  0;
  right_ratio = 0;
}

void AutoDrive()
{
  int sum = 0;
  while (sum < 4)
  {
    wb_robot_step(TIME_STEP);
    ReadSensors();   
    printf ("\n\t\tPosition : 0b");
    for (int i = 0 ; i < 8 ; i ++)
    {
      printf ("%u" , filted[i] );
    }
    if (Position() == MID)
      GoStraight();
    else if (Position() == RIGHT)
      TurnRight();
    else if (Position() == LEFT)
      TurnLeft();
    sum = 0;
    for (int i = 0; i < 8; i++)
      sum += filted[i];
    wb_motor_set_velocity(left_motor, left_ratio * 25);
    wb_motor_set_velocity(right_motor, right_ratio * 25);
  }
  printf("Out circle\n");
  
  for (int i = 0; i < 15; i++)
    Drive(0.8, 0.8);
  for (int i = 0; i < 15; i++)
    Drive(-1.5, 1.5);
  while (!(filted[0] || filted[1] || filted[2]))
    Drive(-0.5, 0.5);
}

void setV(double left, double right)
{
    left_ratio = left;
    right_ratio = right;
}

void auto_Run(int loop_Num, double left, double right)
{
    int count = 0;
    while (count++ < loop_Num)
    {
        wb_robot_step(TIME_STEP);
        setV(left, right);

        printf("\n\tPosition : 0b");
        for (int i = 0; i < 8; i++) printf("%u", filted[i]);
        printf("\tCount=%d", count);

        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
    }
}


/*
 * This is the main program.
 */
int main() {
  
  //dùng để khai báo robot 
  //#không được bỏ
  wb_robot_init();  

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 64);

  /* initialization */
  char name[20];
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }
  
  WbDeviceTag ds = wb_robot_get_device("ds_center");
  wb_distance_sensor_enable(ds,TIME_STEP);
  double distance;

  for (int i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
  // Chương trình sẽ được lặp lại vô tận trong hàm while
    led[i] = wb_robot_get_device(name);
    wb_led_set(led[i], 1);
  }  
  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
    
  while (wb_robot_step(TIME_STEP) != -1)
  {
    ReadSensors();  
    distance = wb_distance_sensor_get_value(ds);
    //printf("distance: %f", distance); 
    if (wb_robot_get_time() < 0.5 ) 
       GoStraight();  
    // //In giá trị của cảm biến ra màn hình
    printf ("\n\t\tPosition : 0b");
    for (int i = 0 ; i < 8 ; i ++)
    {
      printf ("%u" , filted[i] );
    }
    //Điều khiển xe
    /////////////////////////////////////////////////////////////////////////////////////
        if (distance < 900 && Position() == MID && !bridge)
        {
            printf("\tBridge");

            auto_Run(30, 3.2, 3.2);
            while (1)
            {
                ReadSensors();
                wb_robot_step(TIME_STEP);

                printf("\n\tPosition : 0b");
                for (int i = 0; i < 8; i++) printf("%u", filted[i]);

                if (filted[1]) TurnLeft();
                else if (filted[6]) TurnRight();
                else setV(3, 3);

                distance = wb_distance_sensor_get_value(ds);
                if (distance < 1000) break;

                wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
                wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

            }
            bridge = 1;
            printf("\nEnd bridge");
        }
        //////////////////////////////////////////////////////////////////////////////////////////
        
    
    // dò ngã 4
    if (Position() != FULL_SIGNAL)
      DetectIntersection();
    // đi thẳng
    if (Position() == MID)
      GoStraight();
    // đi phải
    else if (Position() == RIGHT)
      TurnRight();
    // đi trái
    else if (Position() == LEFT)
      TurnLeft();
    else if (Position() == NOP)
    {
      printf("\tNOP");
      left_ratio = right_ratio = 0;
    }
    // góc trái, góc phải
    else if (Position() == BLANK_SIGNAL)
    {
      if ((pre_filted[6] && pre_filted[7]) || pre_filted[7])
      {
        TurnRightCorner();
      }
      else if ((pre_filted[0] && pre_filted[1]) || pre_filted[0])
      {
        TurnLeftCorner();
      }
      else if (pre_filted[2] || pre_filted[3] || pre_filted[4] || pre_filted[5])
      {
        LostLine();
      }
      time = 0;
    }
    // ngã 4
    else if (Position() == FULL_SIGNAL)
    {
      printf("FULL_SIGNAL\n");
      if (time < 0 || time > 30)
      {
        printf("intersection\n");
        for (int i = 0; i < 30; i++)
        {
          wb_robot_step(TIME_STEP);
          ReadSensors();
          distance = wb_distance_sensor_get_value(ds);
          wb_motor_set_velocity(left_motor, 40);
          wb_motor_set_velocity(right_motor, 40);
        }
        printf("\tp0\n");
        if (filted[0] && filted[1] && filted[2] && filted[3] && filted[4] && filted[5] && filted[6] && filted[7])
        {
          for (int i = 0; i < 40; i++)
            Drive(0, 0);
          
          for (int i = 0; i < 120; i++)
            Drive(2, 2);
          while(1)
            Drive(0, 0);
        }
        else if (distance < 1000)
        {
          printf("\tp1\n");
          for (int i = 0; i < 30; i++)
            Drive(0.62, 0.62);
          printf("\tp2\n");
          while (distance < 1000)
          {
            wb_robot_step(TIME_STEP);
            ReadSensors();
            distance = wb_distance_sensor_get_value(ds);
            printf("\t%.2f", distance);
            printf ("\n\t\tPosition : 0b");
            for (int i = 0 ; i < 8 ; i ++)
              printf ("%u" , filted[i] );
            wb_motor_set_velocity(left_motor, 0.6 * MAX_SPEED);
            wb_motor_set_velocity(right_motor, 1.8 * MAX_SPEED);
          }
          printf("\tp3\n");
          for (int i = 0; i < 30; i++)
            Drive(2.0, 2.2);
          printf("\tp4\n");
          for (int i = 0; i < 15; i++)
            Drive(2.2, 2.2);
          printf("\tp5\n");
          for (int i = 0; i < 20; i++)
            Drive(2.2, 1.5);
         
          while (Position() == BLANK_SIGNAL)
            Drive(2.2, 2.2);
          AutoDrive();
        }
        else
        {
          for (int i = 0; i < 10; i++)
            Drive(0.8, 0.8);
          ReadSensors();
          int cnt = 0;
          while ((!(!filted[2] && !filted[3] && !filted[4] && !filted[5])) && cnt <= 20)
          {
            Drive(0.65, 0.65);
            cnt++;
            printf("\t%d", cnt);
          }
          if (cnt <= 20)
          {
            while (!(filted[2] || filted[3] || filted[4] || filted[5]))
              Drive(-2, 2);
            AutoDrive();
            time = 0;
          }
        }
      }
      else if (intersection == LEFT && time <= 30)
      {
        printf("left intersection ");
        
        while (Position() == FULL_SIGNAL)
          Drive(0, 0);
        
        for (int i = 0; i < 10; i++) 
          Drive(0, 0);
        
        bool check_filted_0 = 0;
        for (int i = 0; i < 20; i++)
        {
          Drive(-2, 1);
          if (filted[0])
            check_filted_0 = 1;
        }
        if (check_filted_0 == 0)
          while (!filted[0])
            Drive(-1, 1);
        for (int i = 0; i < 10; i++)
          Drive(0.2, 0);
        
        time = 0;
      }
      
      else if (intersection == RIGHT && time <= 30)
      {
        printf("right intersection ");
        
        while (Position() == FULL_SIGNAL)
          Drive(0, 0);
          
        for (int i = 0; i < 10; i++) 
          Drive(0, 0);
          
        bool check_filted_7 = 0;
        for (int i = 0; i < 20; i++)
        {
          Drive(1, -2);
          if (filted[7])
            check_filted_7 = 1;
        }
        if (check_filted_7 == 0)
          while (!filted[7])
            Drive(1, -1);
        for (int i = 0; i < 10; i++)
          Drive(0, 0.2);
          
        time = 0;
      }
    }
    for (int i = 0; i < 8; i++)
      pre_filted[i] = filted[i];
    printf("\t%d", time);
    //Điều chỉnh tốc độ động cơ
    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
  }

  wb_robot_cleanup();
  return 0;
}