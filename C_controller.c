#include <stdio.h>
#include <unistd.h>
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
#define ROUNDABOUT 6
#define FULL_SIGNAL_LEFT 7
#define FULL_SIGNAL_RIGHT 8

// Điểu chỉnh tốc độ phù hợp
// MAX_SPEED <= 1000
#define MAX_SPEED 25

// Khai báo biến cho các sensors
int threshold = 300;
unsigned int filted[8] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
unsigned int pre_filted[8] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
unsigned int pre1_filted[8] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };


// Biến lưu giá trị tỉ lệ tốc độ của động cơ
double left_ratio = 0.0;
double right_ratio = 0.0;

//min = 0 , max = 10
void constrain(double* value, double min, double max) {
    if (*value > max) *value = max;
    if (*value < min) *value = min;
}

//Thí sinh không được bỏ phần đọc tín hiệu này
//Hàm đọc giá trị sensors
void ReadSensors() {
    unsigned short gs_value[NB_GROUND_SENS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < NB_GROUND_SENS; i++) {
        gs_value[i] = wb_distance_sensor_get_value(gs[i]);
        if (gs_value[i] < threshold)
            filted[i] = 1;
        else filted[i] = 0;
    }
}

int sum = 0, pre_Sum = 0, pre1_Sum=0;
int turn_Signal = -1, count_Straight = 0;
bool bridge = 0;
bool stable = 0, roundabout_Signal=0;

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

bool check_Curve_Left_70()
{
    if ((filted[0] && filted[1] && sum <= 3) && (pre_filted[1] && pre_filted[2] && pre_Sum<=3))
    {
        if (pre1_filted[1] && pre1_filted[2] && pre1_Sum <= 3) return 1;
    }
    return 0;
}

bool check_Curve_Right_70()
{
    if ((filted[7] && filted[6] && sum <= 3) && (pre_filted[6] && pre_filted[5] && pre_Sum <= 3))
    {
        if (pre1_filted[4] && pre1_filted[5] && pre1_Sum <= 3) return 1;
    }
    return 0;
}

void curve_Left_70()
{
    auto_Run(5, 0.25, 0.25);
    int count = 0;
    ReadSensors();
    while (!filted[2] && count++ < 20)
    {
        wb_robot_step(TIME_STEP);
        setV(-0.8, 2.3);

        printf("\tCount=%d", count);

        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
        ReadSensors();
    }
    printf("\tCurve left 70");
}

void curve_Right_70()
{
    auto_Run(5, 0.25, 0.25);
    int count = 0;
    ReadSensors();
    while (!filted[2] && count++ < 20)
    {
        wb_robot_step(TIME_STEP);
        setV(2.3, -0.8);

        printf("\tCount=%d", count);

        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
        ReadSensors();
    }
    printf("\tCurve right 70");
}

void check_Turn_Left()
{
    if ((filted[0] || filted[4]) && filted[1] && filted[2] && filted[3] && (pre_filted[0] && pre_filted[1] && pre_filted[2] && pre_filted[3] && pre_filted[4] && pre_Sum == 5))
    {
        turn_Signal = LEFT;
        printf("\tTurn signal=1");
    }
    else if ((pre_filted[0] || pre_filted[4]) && pre_filted[1] && pre_filted[2] && pre_filted[3] && (filted[0] && filted[1] && filted[2] && filted[3] && filted[4] && sum == 5))
    {
        turn_Signal = LEFT;
        printf("\tTurn signal=1");
    }
}

void check_Turn_Right()
{
    if ((filted[7] || filted[3]) && filted[4] && filted[5] && filted[6] && (pre_filted[3] && pre_filted[4] && pre_filted[5] && pre_filted[6] && pre_filted[7] && pre_Sum == 5))
    {
        turn_Signal = RIGHT;
        printf("\tTurn signal=2");
    }
    else if ((pre_filted[7] || pre_filted[3]) && pre_filted[4] && pre_filted[5] && pre_filted[6] && (filted[3] && filted[4] && filted[5] && filted[6] && filted[7] && sum == 5))
    {
        turn_Signal = RIGHT;
        printf("\tTurn signal=2");
    }
}

int Position(unsigned int A[])
{
    int mul = 1;
    sum = 0;
    for (int i = 0; i < NB_GROUND_SENS; i++)
    {
        mul *= A[i];
        sum += A[i];
    }

    if (mul == 1) return FULL_SIGNAL;
    if (sum==7 && pre_Sum==6 && ((!pre_filted[0] && !pre_filted[1]) || (!pre_filted[6] && !pre_filted[7]))) return FULL_SIGNAL;
    if (sum == 0) return BLANK_SIGNAL;

    if (A[0] && A[1] && A[2] && A[3] && A[4] && A[5] && sum >= 6) return FULL_SIGNAL_LEFT;
    if (A[7] && A[6] && A[5] && A[4] && A[3] && A[2] && sum >= 6) return FULL_SIGNAL_RIGHT;

    if (A[0] && A[1] && A[2] && A[3] && A[4] && sum == 5) return LEFT;
    if (A[3] && A[4] && A[5] && A[6] && A[7] && sum == 5) return RIGHT;

    if ((A[0] || A[2]) && A[1] && !A[3] && A[4] && A[5] && (sum == 4 || sum == 5)) return ROUNDABOUT;
    if (A[0] && A[1] && A[2] && !A[3] && A[4] && A[5] && !A[6] && !A[7]) return ROUNDABOUT;
    if ((A[0] || A[1]) && A[4] && A[5] && sum == 3) return ROUNDABOUT;
    if ((A[0] && A[1] && A[2] && A[4] && sum == 4) && (pre_filted[1] && pre_filted[2] && pre_filted[3] && pre_filted[4] && pre_Sum == 4)) return ROUNDABOUT;
    if ((A[0] || A[1]) && (A[6] || A[7]) && sum <= 4) return ROUNDABOUT;
    if ((A[0] && A[3] && A[4] && sum == 3) && (pre_filted[0] && pre_filted[1] && pre_filted[3] && pre_filted[4] && pre_Sum == 4)) return ROUNDABOUT;

    if (A[3] && A[4]) return MID;

    if ((A[0] || A[1] || A[2] || A[3]) && (A[5] + A[6] + A[7] == 0)) return LEFT;
    if ((A[4] || A[5] || A[6] || A[7]) && (A[0] + A[1] + A[2] == 0)) return RIGHT;

    return NOP;
}



//hàm điều khiểu xe đi thẳng
void GoStraight()
{
    if (!sum)
    {
        // Chống lại lực rẽ trái
        if (Position(pre_filted) == LEFT && (pre_filted[2] || pre_filted[1]) && pre_Sum <= 2)
        {
            auto_Run(8, 2.5, 1.3);
            printf("\t Stable Curve Left 1");
        }
        //Hỗ trợ lực rẽ trái
        else if (Position(pre_filted) == LEFT && pre_filted[0] && pre_Sum <= 2)
        {
            auto_Run(7, 1.1, 2.5);
            printf("\t Stable Curve Left 2");
        }
        //Chống lại lực rẽ phải
        else if (Position(pre_filted) == RIGHT && (pre_filted[5] || pre_filted[6]) && pre_Sum <= 2)
        {
            auto_Run(8, 1.3, 2.5);
            printf("\t Stable Curve Right 1");
        }
        //Hỗ trợ lực rẽ phải
        else if (Position(pre_filted) == RIGHT && pre_filted[7] && pre_Sum <= 2)
        {
            auto_Run(7, 2.5, 1.1);
            printf("\t Stable Curve Right 2");
        }
        else setV(2.2, 2.2);
        return;
    }

    if ((pre_filted[3] && pre_filted[4] && pre_Sum == 2))
    {
        if (++count_Straight > 3) stable = 1;
    }
    else count_Straight = 1;
    printf("\tGo straight");

    setV(2, 2);

}

//hàm dừng xe
void Stop()
{
    auto_Run(1, 0, 0);
}


void Turn_left()
{
    check_Turn_Left();
    if (check_Curve_Left_70()) curve_Left_70();

    if (filted[4] && filted[3] && filted[2] && filted[1] && filted[0])
    {
        setV(0.7, 0.7);
        stable = count_Straight = 0;
        
        return;
    }

    if (filted[3] && filted[2] && filted[1] && filted[0])
    {
        setV(0, 3);
        return;
    }

    if (filted[2] && filted[3] && filted[4])
    {
        setV(0.9, 2.3);
        return;
    }

    if (filted[1] && filted[2])
    {
        setV(0.4, 2.4);
        return;
    }

    if (filted[3] || filted[2])
    {
        if (!stable && count_Straight <= 3 && !filted[2])
        {
            setV(1.9, 2.2);
            printf("Done");
        }
        else setV(1.1, 2);
        count_Straight = 0;

        return;
    }

    if (filted[1])
    {
        setV(0, 2.7);
        return;
    }

    if (filted[0])
    {
        setV(0, 2.7);
        return;
    }


}


void Turn_left_corner()
{
    printf("\tTurn left corner");

    if (Position(filted) == FULL_SIGNAL || Position(filted) == ROUNDABOUT)
    {
        auto_Run(13, 0.7, 0.7);
        int count = 0;
        ReadSensors();
        while (!filted[0] && count < 27)
        {
            wb_robot_step(TIME_STEP);

            setV(-0.5, 1.65);
            printf("\tCount=%d", count++);

            wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
            wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
            ReadSensors();
        }
    }

    else
    {
        auto_Run(13, 0.3, 0.3);
        int count = 0;
        ReadSensors();
        while ((!filted[0] && !Position(filted) == MID) && count < 27)
        {
            wb_robot_step(TIME_STEP);

            printf("\n\tPosition : 0b");
            for (int i = 0; i < 8; i++) printf("%u", filted[i]);

            setV(-0.18, 1.7);

            printf("\tCount=%d", count++);

            wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
            wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

            ReadSensors();
        }
    }
    turn_Signal = 0;
}

void Turn_right()
{
    check_Turn_Right();
    if (check_Curve_Right_70()) curve_Right_70();

    if (filted[3] && filted[4] && filted[5] && filted[6] && filted[7])
    {
        setV(0.7, 0.7);
        stable = count_Straight = 0;
        return;
    }

    if (filted[4] && filted[5] && filted[6] && filted[7])
    {
        setV(3, 0);
        return;
    }

    if (filted[3] && filted[4] && filted[5])
    {
        setV(2.3, 0.9);
        return;
    }

    if (filted[5] && filted[6])
    {
        setV(2.4, 0.4);
        return;
    }


    if (filted[4] || filted[5])
    {
        if (!stable && count_Straight <= 3 && !filted[5])
        {
            setV(2.2, 1.9);
            printf("Done");
        }
        else setV(2, 1.1);
        count_Straight = 0;
        return;
    }


    if (filted[6])
    {
        setV(2.5, 0);
        return;
    }

    if (filted[7])
    {
        setV(2.7, 0);
        return;
    }
}


void Turn_right_corner()
{
    printf("\tTurn right corner");

    if (Position(filted) == FULL_SIGNAL || Position(filted) == ROUNDABOUT )
    {
        int count = 0;
        auto_Run(13, 0.7, 0.7);
        ReadSensors();
        while (!filted[7] && count++ < 27)
        {
            wb_robot_step(TIME_STEP);
            printf("\n\tPosition : 0b");

            for (int i = 0; i < 8; i++) printf("%u", filted[i]);
            setV(1.65, -0.5);
            printf("\tCount=%d", count);
            wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
            wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

            ReadSensors();
        }
    }

    else
    {
        int count = 0;
        auto_Run(13, 0.3, 0.3);
        while ((!filted[7] && !Position(filted) == MID) && count++ < 27)
        {
            wb_robot_step(TIME_STEP);

            printf("\n\tPosition : 0b");
            for (int i = 0; i < 8; i++) printf("%u", filted[i]);
            setV(1.7, -0.18);

            printf("Count=%d", count);

            wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
            wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

            ReadSensors();
        }
    }
    turn_Signal = 0;
}


void roundabout()
{
    printf("\tTurn roundabout");
    
    auto_Run(13, 0.7, 0.7);
    int count = 0;
    ReadSensors();
    while (/*(!filted[0] && Position(filted) != MID) &&*/ count < 26)
    {
        wb_robot_step(TIME_STEP);

        printf("\n\tPosition : 0b");
        for (int i = 0; i < 8; i++) printf("%u", filted[i]);

        setV(-0.3, 1.65);

        printf("\tCount=%d", count++);

        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

        ReadSensors();
    }
    roundabout_Signal = 0;
    turn_Signal = 0;
    stable = 0;
}

void inroundabout(WbDeviceTag ds, double ds_sensor)
{
    int count = 0;
    ReadSensors();
    while ((!filted[2] && !Position(filted) == MID) && count++<25)
    {
        wb_robot_step(TIME_STEP);

        printf("\n\tPosition : 0b");
        for (int i = 0; i < 8; i++) printf("%u", filted[i]);

        ds_sensor = wb_distance_sensor_get_value(ds);
        printf("\tDS=%f", ds_sensor);

        setV(0, 2);

        printf("\tCount=%d", count);

        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

        ReadSensors();
    }
    roundabout_Signal = 1;
}


void obstacle(WbDeviceTag ds, double ds_sensor)
{
    int count = 0;
    printf("\tOstacle");
    auto_Run(3, 3, 3);
    auto_Run(22, 1.1, 2.5);

    while (count++ < 33)
    {
        wb_robot_step(TIME_STEP);
        printf("\n\tPosition : 0b");
        for (int i = 0; i < 8; i++) printf("%u", filted[i]);
        setV(2.8, 2.8);

        ReadSensors();
        Position(filted);

        ds_sensor = wb_distance_sensor_get_value(ds);
        printf("\tDS=%f", ds_sensor);
        printf("\tCount=%d", count);

        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
    }

    while (!sum)
    {
        wb_robot_step(TIME_STEP);
        printf("\n\tPosition : 0b");

        for (int i = 0; i < 8; i++) printf("%u", filted[i]);
        setV(2.2, 0);

        ds_sensor = wb_distance_sensor_get_value(ds);
        printf("\tDS=%f", ds_sensor);

        ReadSensors();
        Position(filted);
        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
    }
    roundabout_Signal = 1;

}



/*
 * This is the main program.
 */
int main()
{
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

    for (int i = 0; i < NB_LEDS; i++) {
        sprintf(name, "led%d", i);
        led[i] = wb_robot_get_device(name);
        wb_led_set(led[i], 1);
    }

    char name_ds[20];
    sprintf(name_ds, "ds_center");
    WbDeviceTag ds = wb_robot_get_device(name_ds);
    wb_distance_sensor_enable(ds, TIME_STEP);

    // motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    // Chương trình sẽ được lặp lại vô tận trong hàm while
    while (wb_robot_step(TIME_STEP) != -1)
    {
        ReadSensors();
        if (wb_robot_get_time() < 0.5)
            GoStraight();

        double ds_sensor;
        ds_sensor = wb_distance_sensor_get_value(ds);


        //In giá trị của cảm biến ra màn hình
        printf("\n\tPosition : 0b");
        for (int i = 0; i < 8; i++) printf("%u", filted[i]);
        printf("\tDS=%f", ds_sensor);
        // Điều khiển xe

        int pos = Position(filted);

        //đi thẳng

        int count1 = 0;
        if (pos == ROUNDABOUT || (roundabout_Signal && turn_Signal==LEFT) ) roundabout();

        else if (ds_sensor < 900 && pos == MID && !bridge)
        {
            printf("\tBridge");
            
            auto_Run(30, 3.2, 3.2);
            while (1)
            {
                ReadSensors();
                wb_robot_step(TIME_STEP);

                printf("\n\tPosition : 0b");
                for (int i = 0; i < 8; i++) printf("%u", filted[i]);

                if (filted[1]) Turn_left();
                else if (filted[6]) Turn_right();
                else setV(3.2, 3.2);

                ds_sensor = wb_distance_sensor_get_value(ds);
                if (ds_sensor < 1000) break;

                wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
                wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
                
            }
            bridge = 1;
            printf("\nEnd bridge");
        }

        else if (pos == MID) GoStraight();
        

        else if (pos == FULL_SIGNAL)
        {
            switch (turn_Signal)
            {
            case -1:
            case 0:
                while (count1++ < 25)
                {
                    ReadSensors();

                    printf("\tCount=%d", count1);
                    wb_robot_step(TIME_STEP);

                    printf("\n\tPosition : 0b");
                    for (int i = 0; i < 8; i++) printf("%u", filted[i]);

                    ds_sensor = wb_distance_sensor_get_value(ds);
                    printf("\tDS=%f", ds_sensor);

                    setV(2, 2);

                    wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
                    wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
                }
                ReadSensors();
                pos = Position(filted);

                if (pos == FULL_SIGNAL)
                {
                    Stop();
                    wb_robot_cleanup();
                    return 0;
                }
                else
                {
                    int count = 0;
                    while (Position(filted) != BLANK_SIGNAL && count++ <10)
                    {
                        wb_robot_step(TIME_STEP);
                        setV(0.75, 0.75);

                        printf("\n\tPosition : 0b");
                        for (int i = 0; i < 8; i++) printf("%u", filted[i]);

                        ds_sensor = wb_distance_sensor_get_value(ds);
                        printf("\tDS=%f", ds_sensor);

                        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
                        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);
                        ReadSensors();
                    }

                    if (ds_sensor < 1000) obstacle(ds, ds_sensor);
                    else if (count == 11 && (filted[3] || filted[4])) GoStraight();
                    else inroundabout(ds, ds_sensor);
                }
                break;
            case 1:
                stable = 0;
                Turn_left_corner();
                break;
            case 2:
                stable = 0;
                Turn_right_corner();
                break;
            }
        }

        

        else if (pos == BLANK_SIGNAL)
        {
            switch (turn_Signal)
            {
            case -1:
            case 0:
                GoStraight();
                break;
            case 1:
                Turn_left_corner();
                break;
            case 2:
                Turn_right_corner();
                break;
            }
        }

        else if (pos == LEFT)
        {
            Turn_left();
            printf("\tTurn left");
        }
        else if (pos == RIGHT)
        {
            Turn_right();
            printf("\tTurn right");
        }
        else if (pos == FULL_SIGNAL_LEFT)
        {
            setV(1.8, 2.2);
            printf("\tFull signal left");
        }
        else if (pos == FULL_SIGNAL_RIGHT)
        {
            setV(2.2, 1.8);
            printf("\tFull signal right");
        }

        wb_motor_set_velocity(left_motor, left_ratio * MAX_SPEED);
        wb_motor_set_velocity(right_motor, right_ratio * MAX_SPEED);

        pre_Sum = pre1_Sum = 0;
        printf("\tTurn signal=%d", turn_Signal);

        for (int i = 0; i < NB_GROUND_SENS; i++)
        {
            pre1_filted[i] = pre_filted[i];
            pre1_Sum += pre_filted[i];
           

            pre_filted[i] = filted[i];
            pre_Sum += filted[i];

        }
    }
    wb_robot_cleanup();
    return 0;
}
