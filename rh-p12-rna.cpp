/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#if defined(__linux__)
#include <unistd.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <Windows.h>
#include <conio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <thread>

#include "dynamixel_sdk.h"

using namespace std;

/* ROWS */
#define ROW_MODE_CURRENT        6
#define ROW_MODE_POSITION       7

#define ROW_TORQUE_ON_OFF       10

#define ROW_CTRL_OPEN           13
#define ROW_CTRL_CLOSE          14
#define ROW_CTRL_REPEAT         15
#define ROW_CTRL_GOAL_POSITION  16

#define ROW_GOAL_PWM            19
#define ROW_GOAL_CURRENT        20
#define ROW_GOAL_VELOCITY       21
#define ROW_GOAL_POSITION       22

/* COLS */
#define COL_CHECK               5
#define COL_VALUE               23

/* CONTROL TABLE ADDRESS */
#define ADDR_OPERATING_MODE     11
#define ADDR_TORQUE_ENABLE      512
#define ADDR_GOAL_PWM           548
#define ADDR_GOAL_CURRENT       550
#define ADDR_GOAL_VELOCITY      552
#define ADDR_GOAL_POSITION      564
#define ADDR_MOVING             570

/* VALUE LIMIT */
#define MIN_POSITION            0
#define MAX_POSITION            1150

#define MIN_PWM                 0
#define MAX_PWM                 2009

#define MIN_VELOCITY            0
#define MAX_VELOCITY            2970

#define MIN_CURRENT             0
#define MAX_CURRENT             1984


#define PROTOCOL_VERSION        2.0

#define GRIPPER_ID              1
#define BAUDRATE                2000000

#if defined(__linux__)
#define DEVICE_NAME             "/dev/ttyUSB0"
#elif defined(_WIN32) || defined(_WIN64)
#define DEVICE_NAME             "COM4"
#endif

enum MODE {
  MODE_CURRENT_CTRL = 0,
  MODE_POSITION_CTRL = 5
};

enum CONTROL {
  CTRL_NONE,
  CTRL_REPEAT,
  CTRL_OPEN,
  CTRL_CLOSE,
  CTRL_POSITION
};


int g_curr_row            = ROW_MODE_POSITION;
int g_curr_col            = COL_CHECK;

MODE g_curr_mode          = MODE_POSITION_CTRL;
bool g_is_torque_on       = false;
CONTROL g_curr_control    = CTRL_NONE;


bool g_flag_goal_position = false;
bool g_flag_auto_repeat   = false;

bool g_flag_repeat_thread = false;

int g_goal_position       = 740;
int g_goal_velocity       = 2970;
int g_goal_pwm            = 2009;
int g_goal_current        = 350;

dynamixel::PacketHandler  *g_packet_handler = NULL;
dynamixel::PortHandler    *g_port_handler   = NULL;

thread *g_repeat_thread     = NULL;

int getch()
{
#if defined(__linux__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

void repeatThreadFunc(int val)
{
  const int _max_stop_count = 7;

  int       _direction      = 1;
  int       _stop_cnt       = 0;

  uint8_t   _is_moving      = 0;

  while (g_flag_repeat_thread)
  {
    if (g_packet_handler->read1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_MOVING, &_is_moving) == COMM_SUCCESS)
    {
      if (_is_moving == 1)
      {
        _stop_cnt = 0;
      }
      else if (++_stop_cnt > _max_stop_count)
      {
        if (g_curr_mode == MODE_POSITION_CTRL)
        {
          g_packet_handler->write4ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_POSITION, 
                                           (_direction < 0)? MIN_POSITION:MAX_POSITION);
        }
        else  // MODE_CURRENT_CTRL
        {
          g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT,
                                           g_goal_current * _direction);
        }
        
        _direction = (-1) * (_direction);
        _stop_cnt = 0;
      }
    }

#if defined(__linux__)
    usleep(100*1000);
#elif defined(_WIN32) || defined(_WIN64)
    Sleep(100);
#endif
  }
}

void gotoCursor(int row, int col)
{
#if defined(__linux__)
  printf("\033[%d;%dH", row+1, col+1);
#elif defined(_WIN32) || defined(_WIN64)
  COORD _pos = { col, row };
  SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), _pos);
#endif
}

void drawPage(void)
{
  uint16_t  _data16;
  uint32_t  _data32;

  //if (gPacketHandler->read4ByteTxRx(gPortHandler, GRIPPER_ID, GOAL_POSITION_ADDR, &data32) == COMM_SUCCESS)
  //  g_goal_position = data32;
  if (g_packet_handler->read4ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_VELOCITY, &_data32) == COMM_SUCCESS)
    g_goal_velocity = _data32;
  if (g_packet_handler->read2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_PWM, &_data16) == COMM_SUCCESS)
    g_goal_pwm = _data16;
  if (g_curr_mode != MODE_CURRENT_CTRL && g_packet_handler->read2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, &_data16) == COMM_SUCCESS)
    g_goal_current = _data16;

  //        0         1         2         3         4         5         6         7  
  //        012345678901234567890123456789012345678901234567890123456789012345678901
  printf(  "                                                                        \n"); // 00
  printf(  "************************************************************************\n"); //  1
  printf(  "*                         RH-P12-RN(A) Example                         *\n"); //  2
  printf(  "************************************************************************\n"); //  3
  printf(  "                                                                        \n"); //  4
  printf(  "  ++ MODE ++                                                            \n"); //  5
  printf(  "   [ %c ] (C) current control mode                                       \n", (g_curr_mode == MODE_CURRENT_CTRL)?   'V':' '); //  6
  printf(  "   [ %c ] (P) current based position control mode                        \n", (g_curr_mode == MODE_POSITION_CTRL) ? 'V':' '); //  7
  printf(  "                                                                        \n"); //  8
  printf(  "  ++ TORQUE ++                                                          \n"); //  9
  printf(  "   [ %c ] (T) torque ON / OFF                                            \n", (g_is_torque_on)?                     'V':' '); // 10
  printf(  "                                                                        \n"); //  1
  printf(  "  ++ CONTROL ++                                                         \n"); //  2
  printf(  "   [ %c ] (O) Open                                                       \n", (g_curr_control == CTRL_OPEN) ?       'V':' '); //  3
  printf(  "   [ %c ] (L) Close                                                      \n", (g_curr_control == CTRL_CLOSE)?       'V':' '); //  4
  printf(  "   [ %c ] (A) Open & Close auto repeat                                   \n", (g_curr_control == CTRL_REPEAT) ?     'V':' '); //  5
  if (g_curr_mode == MODE_POSITION_CTRL)
    printf("   [ %c ] (G) Go to goal position                                        \n", (g_curr_control == CTRL_POSITION)?    'V':' '); //  6
  else
    printf("                                                                        \n"); //  6
  printf(  "                                                                        \n"); //  7
  printf(  "  ++ PARAMETERS ++                                                      \n"); //  8
  printf(  "   goal PWM          [ %4d / %4d ]                                    \n", g_goal_pwm, MAX_PWM);                   //  9
  printf(  "   goal current      [ %4d / %4d ]                                    \n", (short)g_goal_current, MAX_CURRENT);    // 20
  if (g_curr_mode == MODE_POSITION_CTRL)
  {
    printf("   goal velocity     [ %4d / %4d ]                                    \n", g_goal_velocity, MAX_VELOCITY);         //  1
    printf("   goal position     [ %4d / %4d ]                                    \n", g_goal_position, MAX_POSITION);         //  2
  }
  printf("\n");

  gotoCursor(g_curr_row, g_curr_col);
}

void moveCursorUp()
{
  if (g_curr_row == ROW_GOAL_PWM)
    g_curr_col = COL_CHECK;

  if (g_curr_mode == MODE_CURRENT_CTRL)
  {
    if (g_curr_row == ROW_TORQUE_ON_OFF ||
      g_curr_row == ROW_CTRL_OPEN)
    {
      g_curr_row -= 3;
    }
    else if (g_curr_row == ROW_GOAL_PWM)
    {
      g_curr_row -= 4;
    }
    else if (g_curr_row != ROW_MODE_CURRENT)
    {
      g_curr_row--;
    }
  }
  else if (g_curr_mode == MODE_POSITION_CTRL)
  {
    if (g_curr_row == ROW_TORQUE_ON_OFF ||
      g_curr_row == ROW_CTRL_OPEN ||
      g_curr_row == ROW_GOAL_PWM)
    {
      g_curr_row -= 3;
    }
    else if (g_curr_row != ROW_MODE_CURRENT)
    {
      g_curr_row--;
    }
  }

  gotoCursor(g_curr_row, g_curr_col);
}

void moveCursorDown()
{
  if (g_curr_mode == MODE_CURRENT_CTRL)
  {
    if (g_curr_row == ROW_CTRL_REPEAT)
      g_curr_col = COL_VALUE;

    if (g_curr_row == ROW_MODE_POSITION ||
        g_curr_row == ROW_TORQUE_ON_OFF)
    {
      g_curr_row += 3;
    }
    else if (g_curr_row == ROW_CTRL_REPEAT)
    {
      g_curr_row += 4;
    }
    else if (g_curr_row != ROW_GOAL_CURRENT)
    {
      g_curr_row++;
    }
  }
  else if (g_curr_mode == MODE_POSITION_CTRL)
  {
    if (g_curr_row == ROW_CTRL_GOAL_POSITION)
      g_curr_col = COL_VALUE;

    if (g_curr_row == ROW_MODE_POSITION ||
        g_curr_row == ROW_TORQUE_ON_OFF ||
        g_curr_row == ROW_CTRL_GOAL_POSITION)
    {
      g_curr_row += 3;
    }
    else if (g_curr_row != ROW_GOAL_POSITION)
    {
      g_curr_row++;
    }
  }
  
  gotoCursor(g_curr_row, g_curr_col);
}

void moveCursorLeft()
{

}

void moveCursorRight()
{

}

void checkValue()
{
  if (g_curr_row == ROW_MODE_POSITION)
  {
    if (g_curr_mode != MODE_POSITION_CTRL)
    {
      // auto repeat thread stop
      if (g_curr_control == CTRL_REPEAT)
      {
        g_flag_repeat_thread = false;
        g_repeat_thread->join();
      }

      // torque off
      if (g_is_torque_on == true)
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0);

#if defined(__linux__)
      usleep(20 * 1000);
#elif defined(_WIN32) || defined(_WIN64)
      Sleep(20);
#endif

      // set mode to current based position control mode
      g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_OPERATING_MODE, MODE_POSITION_CTRL);

#if defined(__linux__)
      usleep(20 * 1000);
#elif defined(_WIN32) || defined(_WIN64)
      Sleep(20);
#endif

      // torque on
      if (g_is_torque_on == true)
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);

      // set goal current
      if ((short)g_goal_current < 0)
        g_goal_current = (-1) * g_goal_current;
      g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, g_goal_current);

      if (g_curr_control == CTRL_REPEAT)
      {
        g_flag_repeat_thread = true;
        g_repeat_thread = new thread(&repeatThreadFunc, 1);
      }

      g_curr_mode = MODE_POSITION_CTRL;

      gotoCursor(0, 0);
#if defined(__linux__)
      system("clear");
#elif defined(_WIN32) || defined(_WIN64)
      system("cls");
#endif
      drawPage();

      gotoCursor(ROW_MODE_CURRENT, g_curr_col);
      printf(" ");
      gotoCursor(g_curr_row, g_curr_col);
      printf("V");
    }
  }
  else if (g_curr_row == ROW_MODE_CURRENT)
  {
    if (g_curr_mode != MODE_CURRENT_CTRL)
    {
      // auto repeat thread stop
      if (g_curr_control == CTRL_REPEAT)
      {
        g_flag_repeat_thread = false;
        g_repeat_thread->join();
      }

      // torque off
      if (g_is_torque_on == true)
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0);

#if defined(__linux__)
      usleep(20 * 1000);
#elif defined(_WIN32) || defined(_WIN64)
      Sleep(20);
#endif

      // set mode to current control mode
      g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_OPERATING_MODE, MODE_CURRENT_CTRL);

#if defined(__linux__)
      usleep(20 * 1000);
#elif defined(_WIN32) || defined(_WIN64)
      Sleep(20);
#endif

      // torque on
      if (g_is_torque_on == true)
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);

      if (g_curr_control == CTRL_REPEAT)
      {
        g_flag_repeat_thread = true;
        g_repeat_thread = new thread(&repeatThreadFunc, 1);
      }

      g_curr_mode = MODE_CURRENT_CTRL;

      gotoCursor(0, 0);
#if defined(__linux__)
      system("clear");
#elif defined(_WIN32) || defined(_WIN64)
      system("cls");
#endif
      drawPage();

      gotoCursor(ROW_MODE_POSITION, g_curr_col);
      printf(" ");
      gotoCursor(g_curr_row, g_curr_col);
      printf("V");
    }
  }
  else if (g_curr_row == ROW_TORQUE_ON_OFF)
  {
    if (g_is_torque_on == true)
    {
      printf(" ");
      g_is_torque_on = false;
      g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0);
    }
    else
    {
      printf("V");
      g_is_torque_on = true;
      g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);
    }
  }
  else if (g_curr_row == ROW_CTRL_REPEAT)
  {
    if (g_curr_control == CTRL_REPEAT)
    {
      printf(" ");
      g_curr_control = CTRL_NONE;

      g_flag_repeat_thread = false;
      g_repeat_thread->join();
    }
    else
    {
      if (g_curr_control == CTRL_OPEN)
        gotoCursor(ROW_CTRL_OPEN, COL_CHECK);
      else if (g_curr_control == CTRL_CLOSE)
        gotoCursor(ROW_CTRL_CLOSE, COL_CHECK);
      else if (g_curr_control == CTRL_POSITION)
        gotoCursor(ROW_CTRL_GOAL_POSITION, COL_CHECK);
      // else if (gControl == NONE) GotoCursor(giRow, giCol);
      printf(" ");
      
      gotoCursor(g_curr_row, g_curr_col);
      printf("V");
      g_curr_control = CTRL_REPEAT;

      if (g_is_torque_on == false)
      {
        gotoCursor(ROW_TORQUE_ON_OFF, COL_CHECK);
        printf("V");
        g_is_torque_on = true;
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);
      }

      g_flag_repeat_thread = true;
      g_repeat_thread = new thread(&repeatThreadFunc, 1);
    }
  }
  else if (g_curr_row == ROW_CTRL_CLOSE)
  {
    if (g_curr_control == CTRL_REPEAT)
    {
      g_flag_repeat_thread = false;
      g_repeat_thread->join();
    }

    if (g_curr_control == CTRL_CLOSE)
    {
      printf(" ");
      g_curr_control = CTRL_NONE;
    }
    else
    {
      if (g_curr_control == CTRL_REPEAT)
        gotoCursor(ROW_CTRL_REPEAT, COL_CHECK);
      else if (g_curr_control == CTRL_OPEN)
        gotoCursor(ROW_CTRL_OPEN, COL_CHECK);
      else if (g_curr_control == CTRL_POSITION)
        gotoCursor(ROW_CTRL_GOAL_POSITION, COL_CHECK);
      // else if (gControl == NONE) GotoCursor(giRow, giCol);
      printf(" ");

      gotoCursor(g_curr_row, g_curr_col);
      printf("V");
      g_curr_control = CTRL_CLOSE;

      if (g_is_torque_on == false)
      {
        gotoCursor(ROW_TORQUE_ON_OFF, g_curr_col);
        printf("V");
        g_is_torque_on = true;
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);
      }

      if (g_curr_mode == MODE_POSITION_CTRL)
        g_packet_handler->write4ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_POSITION, MAX_POSITION);
      else
        g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, (g_goal_current < 0)? -g_goal_current:g_goal_current);

      gotoCursor(g_curr_row, g_curr_col);
#if defined(__linux__)
      usleep(100*1000);
#elif defined(_WIN32) || defined(_WIN64)
      Sleep(100);
#endif
      printf(" ");
      g_curr_control = CTRL_NONE;
    }
  }
  else if (g_curr_row == ROW_CTRL_OPEN)
  {
    if (g_curr_control == CTRL_REPEAT)
    {
      g_flag_repeat_thread = false;
      g_repeat_thread->join();
    }

    if (g_curr_control == CTRL_OPEN)
    {
      printf(" ");
      g_curr_control = CTRL_NONE;
    }
    else
    {
      if (g_curr_control == CTRL_REPEAT)
        gotoCursor(ROW_CTRL_REPEAT, COL_CHECK);
      else if (g_curr_control == CTRL_CLOSE)
        gotoCursor(ROW_CTRL_CLOSE, COL_CHECK);
      else if (g_curr_control == CTRL_POSITION)
        gotoCursor(ROW_CTRL_GOAL_POSITION, COL_CHECK);
      // else if (gControl == NONE) GotoCursor(giRow, giCol);
      printf(" ");

      gotoCursor(g_curr_row, g_curr_col);
      printf("V");
      g_curr_control = CTRL_OPEN;

      if (g_is_torque_on == false)
      {
        gotoCursor(ROW_TORQUE_ON_OFF, g_curr_col);
        printf("V");
        g_is_torque_on = true;
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);
      }

      if (g_curr_mode == MODE_POSITION_CTRL)
        g_packet_handler->write4ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_POSITION, MIN_POSITION);
      else
        g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, (g_goal_current < 0)? g_goal_current:-g_goal_current);

      gotoCursor(g_curr_row, g_curr_col);
#if defined(__linux__)
      usleep(100*1000);
#elif defined(_WIN32) || defined(_WIN64)
      Sleep(100);
#endif
      printf(" ");
      g_curr_control = CTRL_NONE;
    }
  }
  else if (g_curr_row == ROW_CTRL_GOAL_POSITION)
  {
    g_flag_repeat_thread = false;

    if (g_curr_control == CTRL_POSITION)
    {
      printf(" ");
      g_curr_control = CTRL_NONE;
      g_flag_goal_position = false;
    }
    else
    {
      if (g_curr_control == CTRL_REPEAT)
        gotoCursor(ROW_CTRL_REPEAT, COL_CHECK);
      else if (g_curr_control == CTRL_CLOSE)
        gotoCursor(ROW_CTRL_CLOSE, COL_CHECK);
      else if (g_curr_control == CTRL_OPEN)
        gotoCursor(ROW_CTRL_OPEN, COL_CHECK);
      // else if (gControl == NONE) GotoCursor(giRow, giCol);
      printf(" ");

      gotoCursor(g_curr_row, g_curr_col);
      printf("V");
      g_curr_control = CTRL_POSITION;

      if (g_is_torque_on == false)
      {
        gotoCursor(ROW_TORQUE_ON_OFF, g_curr_col);
        printf("V");
        g_is_torque_on = true;
        g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1);
      }

      g_packet_handler->write4ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_POSITION, g_goal_position);
      g_flag_goal_position = true;
    }
  }

  gotoCursor(g_curr_row, g_curr_col);
}

void UpDownValue(int val)
{
  if (g_curr_row == ROW_GOAL_POSITION)
  {
    g_goal_position += val;
    if (g_goal_position < MIN_POSITION)
      g_goal_position = MIN_POSITION;
    else if (g_goal_position > MAX_POSITION)
      g_goal_position = MAX_POSITION;

    if (g_flag_goal_position == true)
      g_packet_handler->write4ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_POSITION, g_goal_position);
    printf("%4d", g_goal_position);
  }
  else if (g_curr_row == ROW_GOAL_VELOCITY)
  {
    g_goal_velocity += val;
    if (g_goal_velocity < MIN_VELOCITY)
      g_goal_velocity = MIN_VELOCITY;
    else if (g_goal_velocity > MAX_VELOCITY)
      g_goal_velocity = MAX_VELOCITY;

    g_packet_handler->write4ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_VELOCITY, g_goal_velocity);
    printf("%4d", g_goal_velocity);
  }
  else if (g_curr_row == ROW_GOAL_PWM)
  {
    g_goal_pwm += val;
    if (g_goal_pwm < MIN_PWM)
      g_goal_pwm = MIN_PWM;
    else if (g_goal_pwm > MAX_PWM)
      g_goal_pwm = MAX_PWM;
    
    g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_PWM, g_goal_pwm);
    printf("%4d", g_goal_pwm);
  }
  else if (g_curr_row == ROW_GOAL_CURRENT)
  {
    g_goal_current += val;

    if (g_curr_mode == MODE_POSITION_CTRL)
    {
      if (g_goal_current < MIN_CURRENT)
        g_goal_current = MIN_CURRENT;
      else if (g_goal_current > MAX_CURRENT)
        g_goal_current = MAX_CURRENT;
    }
    else if (g_curr_mode == MODE_CURRENT_CTRL)
    {
      if (g_goal_current < -MAX_CURRENT)
        g_goal_current = -MAX_CURRENT;
      else if (g_goal_current > MAX_CURRENT)
        g_goal_current = MAX_CURRENT;
    }

    g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, g_goal_current);
    printf("%4d", (short)g_goal_current);
  }

  gotoCursor(g_curr_row, g_curr_col);
}

void Terminate()
{
  g_packet_handler->write1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0);
}


int main(int argc, char* argv[])
{
  // Initialize Packethandler2 instance
  g_packet_handler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

#if defined(__linux__)
  system("clear");
#elif defined(_WIN32) || defined(_WIN64)
  system("cls");
#endif
  
  printf(  "                                                                        \n");
  printf(  "************************************************************************\n");
  printf(  "*                         RH-P12-RN(A) Example                         *\n");
  printf(  "************************************************************************\n");

  char *devName = (char*)DEVICE_NAME;

  if (argc == 2)
    devName = argv[1];

  g_port_handler = dynamixel::PortHandler::getPortHandler(devName);

  if (g_port_handler->openPort())
  {
    printf("Succeeded to open port.\n");

    if (g_port_handler->setBaudRate(BAUDRATE))
    {
      printf("Succeeded to change the baudrate.\n");
      printf(" - Device Name : %s\n", devName);
      printf(" - Baudrate    : %d\n\n", g_port_handler->getBaudRate());
    }
    else
    {
      printf("Failed to change the baudrate.\n");
      printf("Press any key to terminate...\n");
      getch();
      return 0;
    }
  }
  else
  {
    printf("Failed to open port.\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  if (g_packet_handler->ping(g_port_handler, GRIPPER_ID) != COMM_SUCCESS)
  {
    printf("Failed to connect the gripper (ID:%d).\n", GRIPPER_ID);
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  printf("Press any key to continue...\n");
  getch();
#if defined(__linux__)
  system("clear");
#elif defined(_WIN32) || defined(_WIN64)
  system("cls");
#endif

  uint8_t _mode;
  g_packet_handler->read1ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_OPERATING_MODE, &_mode);
  g_curr_mode = (MODE)_mode;

  if (g_curr_mode == MODE_POSITION_CTRL)
    g_packet_handler->write2ByteTxRx(g_port_handler, GRIPPER_ID, ADDR_GOAL_CURRENT, g_goal_current);
  
  drawPage();

  while (true)
  {
    unsigned char ch = getch();
    //printf("%d \n", ch);

#if defined(__linux__)
    if(ch == 27)
    {
      struct termios original_ts, nowait_ts;
      
      // configure getch() to return immediately
      tcgetattr(STDIN_FILENO, &original_ts);
      nowait_ts = original_ts;
      nowait_ts.c_lflag &= ~ISIG;
      nowait_ts.c_cc[VMIN] = 0;
      nowait_ts.c_cc[VTIME] = 0;
      tcsetattr(STDIN_FILENO, TCSANOW, &nowait_ts);
      
      // short delay since slow system take some time to receive additional sequence codes
      usleep(10*1000);
      
      ch = getch();
      tcsetattr(STDIN_FILENO, TCSANOW, &original_ts);

      if(ch == 91)
      {
          ch = getch();
          if(ch == 65)      // Up arrow key
              moveCursorUp();
          else if(ch == 66) // Down arrow key
              moveCursorDown();
          else if(ch == 68) // Left arrow key
              moveCursorLeft();
          else if(ch == 67) // Right arrow key
              moveCursorRight();
      }
      else if (ch == (unsigned char)EOF)    // ESC key
      {
        Terminate();
        break;
      }
    }
#elif defined(_WIN32) || defined(_WIN64)
    if (ch == 224)
    {
      ch = getch();
      if (ch == 72)       // UP arrow key
        moveCursorUp();
      else if (ch == 80)  // DOWN arrow key
        moveCursorDown();
      else if (ch == 75)  // LEFT arrow key
        moveCursorLeft();
      else if (ch == 77)  // RIGHT arrow key
        moveCursorRight();
    }
    else if (ch == 27)    // ESC key
    {
      Terminate();
      break;
    }
#endif
    else if (ch == 32)    // SPACE key
    {
      checkValue();
    }
    else if (ch == '[')
    {
      UpDownValue(-1);
    }
    else if (ch == ']')
    {
      UpDownValue(1);
    }
    else if (ch == '{')
    {
      UpDownValue(-10);
    }
    else if (ch == '}')
    {
      UpDownValue(10);
    }

    else if (ch == 'P' || ch == 'p')
    {
      g_curr_row = ROW_MODE_POSITION;
      g_curr_col = COL_CHECK;
      gotoCursor(g_curr_row, g_curr_col);
      checkValue();
    }
    else if (ch == 'C' || ch == 'c')
    {
      g_curr_row = ROW_MODE_CURRENT;
      g_curr_col = COL_CHECK;
      gotoCursor(g_curr_row, g_curr_col);
      checkValue();
    }
    else if (ch == 'T' || ch == 't')
    {
      g_curr_row = ROW_TORQUE_ON_OFF;
      g_curr_col = COL_CHECK;
      gotoCursor(g_curr_row, g_curr_col);
      checkValue();
    }
    else if (ch == 'A' || ch == 'a')
    {
      g_curr_row = ROW_CTRL_REPEAT;
      g_curr_col = COL_CHECK;
      gotoCursor(g_curr_row, g_curr_col);
      checkValue();
    }
    else if (ch == 'L' || ch == 'l')
    {
      g_curr_row = ROW_CTRL_CLOSE;
      g_curr_col = COL_CHECK;
      gotoCursor(g_curr_row, g_curr_col);
      checkValue();
    }
    else if (ch == 'O' || ch == 'o')
    {
      g_curr_row = ROW_CTRL_OPEN;
      g_curr_col = COL_CHECK;
      gotoCursor(g_curr_row, g_curr_col);
      checkValue();
    }
    else if (ch == 'G' || ch == 'g')
    {
      if (g_curr_mode == MODE_POSITION_CTRL)
      {
        g_curr_row = ROW_CTRL_GOAL_POSITION;
        g_curr_col = COL_CHECK;
        gotoCursor(g_curr_row, g_curr_col);
        checkValue();
      }
    }
  }

  return 0;
}

