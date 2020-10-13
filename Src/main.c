/**THIS CODE TESTED FLYING
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "pid_control.cpp"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;
//***************************************define TIme varible 
int Time=0;
long int Time_count=0;
int H_Time=0;
long int H_Time_count=0;
int loop_Time=0;
int k1=0;
long int k2=0;
//************************************** end define Time varible
//**************************************define varible input RC & output pwm
#define THROTTLE_RMAX  1878
#define THROTTLE_RMIN  1080
#define K_S 0.75
#define K_S_R_MAX (K_S*(THROTTLE_RMAX-THROTTLE_RMIN))
#define K_S_R_MIN ((1-K_S)*(THROTTLE_RMAX-THROTTLE_RMIN))
#define THROTTLE_SAFE_SHUTOFF 1100
#define THROTTLE_RMID  1504

#define ROLL_RMIN  THROTTLE_RMIN
#define ROLL_RMID  1484
#define ROLL_RMAX  THROTTLE_RMAX
#define ROLL_WMIN  -30
#define ROLL_WMAX  30

#define PITCH_RMIN  THROTTLE_RMIN
#define PITCH_RMID  1489
#define PITCH_RMAX  THROTTLE_RMAX
#define PITCH_WMIN  -30
#define PITCH_WMAX  30

#define YAW_RMIN  THROTTLE_RMIN
#define YAW_RMID  1488
#define YAW_RMAX  THROTTLE_RMAX
#define YAW_WMIN  -30//30
#define YAW_WMAX  30
/////////////////////////////////////////////////////////////////////////////
int RC1=0;
int RC2,RC3,RC4,RC5,RC6,RC1H,RC2H,RC3H,RC4H,RC5H,RC6H,RC1L,RC2L,RC3L,RC4L,RC5L,RC6L,N_RC1,N_RC2,N_RC3,N_RC4,N_RC5,N_RC6,H_RC1,H_RC2,H_RC3,H_RC4,H_RC5,H_RC6;
int f1=0,f2=0,flagcheck=0,flagcheck2=0;
float K_TIMER1=0.5;//0.625;
float K_TIMER2=0.5;//0.625;
int K_f_TIMER=40000;
int ii;
double throttle;
int rx_values[6];
//**************************************end define varible input RC & output pwm

//**************************************define varible imu
UART_HandleTypeDef huart2;
int flag_imu_setup=1;
unsigned char Re_buf[30],counter=0;
int counter_calibrate_imu=0;
unsigned char sign=0;
unsigned char RX[2];
uint8_t Rig[3];
float ROLL,PITCH,YAW;
 ///////////////////////////////
 float angleX,angleY,angleZ = 0.0;
int n_inth=150;
int n_inth_zero=10;
float angleX0,angleY0,angleZ0 = 0.0;
float angleX1,angleY1,angleZ1 = 0.0;
//**************************************end difine varible imu
 //************************************define controller varible
double axissA[3];
double axissE[3];
double axissR[3];
double axissEA[3];
///////////////////////
#define R_min 950
#define R_max 2050
#define Kp_min 0.7 
#define Kp_max 1.5 
#define Ki_min 0
#define Ki_max 3
#define  Kd_min 0.1
#define Kd_max 0.5
//////////////////////////////////////////////////////////////////////////
#define ROLL_PID_KP  0.250
#define ROLL_PID_KI  0.950
#define ROLL_PID_KD  0.011
#define ROLL_PID_MIN  -200.0
#define ROLL_PID_MAX  200.0

#define PITCH_PID_KP  0.250
#define PITCH_PID_KI  0.950
#define PITCH_PID_KD  0.011
#define PITCH_PID_MIN  -200.0
#define PITCH_PID_MAX  200

#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001
#define YAW_PID_MIN  -200.0
#define YAW_PID_MAX  200.0
///////////////////////////////////////////////show
double kp,ki,kd;
/////////////////////////////////////////////////////////////////////
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;
////////////////////////////////////////////////////////////////////////////////////////////
PID roll_controller(&pid_roll_in,   &pid_roll_out,  &pid_roll_setpoint , &loop_Time,  axissEA[0],  axissEA[2],axissEA[1],DIRECT  );
PID pitch_controller(&pid_pitch_in, &pid_pitch_out, &pid_pitch_setpoint ,&loop_Time, axissEA[0],  axissEA[2],axissEA[1],DIRECT  );
PID yaw_controller(&pid_yaw_in,     &pid_yaw_out,   &pid_yaw_setpoint, &loop_Time,    axissR[0],  axissR[2],axissEA[1], DIRECT ); 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////difine motors varible
int m0, m1, m2, m3;
#define MOTOR_ZERO_LEVEL  1080
#define MOTOR_ARM_START  1500
#define MOTOR_MAX_LEVEL  1700
 //************************************ end difine contoller varible

//***************************************** define Safty varible
bool flag_Safty=false;
//****************************************** end difine Safty varible
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
//static void motortest(void);
//********************************************************helping Function
double ABS(double X)
{
  if(X>=0)
    return X;
  else return (-X);
}
////////////////////////////////////////////////////////////////////////
double map(int x,double x_min,double x_max,double X_min,double X_max)
{
  double alpha=ABS(X_max-X_min)/ABS(x_max-x_min);
  double beta=X_min-(alpha*x_min);
  return ((alpha*x)+beta);
}

//********************************************************End helping functio

//****************************************************** Time function
void LoopTime()
{
  Time=__HAL_TIM_GET_COUNTER(&htim1);
  if(Time>=H_Time)
   k1=((Time-H_Time)/2000);
  else
    k1=((Time-H_Time+K_f_TIMER)/2000);
  
   k2=( Time_count-H_Time_count)*(K_f_TIMER/2000);
  loop_Time=k1+k2;
  H_Time=Time;
  H_Time_count=Time_count;
}
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)
{
  if(htim->Instance==TIM4)
  {
  Time_count++;
  }
}
//*******************************************************End Time function

//******************************************************imu function
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void serialEvent() {
  if(flag_imu_setup==0)
  {
  
      Re_buf[counter]=RX[0];
      if(counter==0&&Re_buf[0]!=0x5A) return;          
      counter++;      
      if(counter==20)                
      {    
         counter=0;                  
         sign=1;
      }
 HAL_UART_Receive_IT(&huart3,RX,1);  
  }
  else
  {
    flag_imu_setup=0;
    HAL_UART_Receive_IT(&huart3,RX,1); 
  }
}//with intruptt
//////////////////////////////////////////////with out intrupts
/*void serialEvent() {
  
      HAL_UART_Receive(&huart2,RX,1,1);  
      Re_buf[counter]=RX[0];
      if(counter==0&&Re_buf[0]!=0x5A) return;          
      counter++;      
      if(counter==20)                
      {    
         counter=0;                  
         sign=1;
      }
}*/
////////////////////////////////////////////////////////////////////
void  imu_calibration() {
  unsigned char i=0,sum=0;
   int16_t DATA[7];
     while(counter_calibrate_imu<n_inth){
   //serialEvent();
  if(sign)
  {   
     for(i=0;i<19;i++)
      sum+=Re_buf[i]; 
     if(sum==Re_buf[i] )       
     {     
         DATA[0]=(Re_buf[4]<<8)|Re_buf[5];
         DATA[1]=(Re_buf[6]<<8)|Re_buf[7];
         DATA[2]=(Re_buf[8]<<8)|Re_buf[9];
         angleZ1=((double)((uint16_t)DATA[0])/100);
         angleY1=((double)DATA[1]/100);
         angleX1=((double)DATA[2]/100);
         counter_calibrate_imu++;
    if(counter_calibrate_imu>n_inth_zero) 
   {
    angleZ0=((double)((uint16_t)DATA[0])/100)+ angleZ0;
         angleY0=((double)DATA[1]/100)+angleY0;
         angleX0=((double)DATA[2]/100)+ angleX0;
         //**************************************************************
   }
         //*******************************
       sign=0;        
   }
  }
  }
         angleZ0=angleZ0/(n_inth-n_inth_zero);
         angleY0=angleY0/(n_inth-n_inth_zero);
         angleX0=angleX0/(n_inth-n_inth_zero);
}
//////////////////////////////////////////////////////////////////////////////////////////
double convert_angle_Z(double Z)
{
 if(ABS(Z)>=180)
{
  return (-Z/ABS(Z))*180+((Z/ABS(Z))*(ABS(Z)-180));
}
else return Z;
}
//////////////////////////////////////////////////////////////////////////////////////////
void imu_update()
{
   unsigned char i=0,sum=0;
   int16_t DATA[7];
   //serialEvent();
  if(sign)
  {   
     for(i=0;i<19;i++)
      sum+=Re_buf[i]; 
     if(sum==Re_buf[i] )        
     {     
         DATA[0]=(Re_buf[4]<<8)|Re_buf[5];
         DATA[1]=(Re_buf[6]<<8)|Re_buf[7];
         DATA[2]=(Re_buf[8]<<8)|Re_buf[9];
         PITCH= (double)DATA[2]/100;
         ROLL=(double)DATA[1]/100;
         YAW=(double)((uint16_t)DATA[0])/100;
         //YAW=convert_angle_Z(YAW);
         angleZ=convert_angle_Z(((double)((uint16_t)DATA[0])/100)- angleZ0);
         angleY=((double)DATA[1]/100)- angleY0;
         angleX=((double)DATA[2]/100)- angleX0;
         sign=0;        
   }
	}

}
///////////////////////////////////////////////////////////////////
void imu_setup() {
  //********************************************imu setup
Rig[0]=0xAA;//char Rig1[3];// "170"
Rig[1]=0x38;//char Rig2[2];//"56"
Rig[2]=0xE2;//;char Rig3[3];//"226"*/
////////////////////////////////////////////////////////////
HAL_UART_Transmit(&huart3,Rig,3,1);
 //*******************************************end imu set up
  serialEvent();
  imu_calibration();
  //************************/
}
///////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART3)
  {
    serialEvent();
  }
}

//***************************************************end imu function                                
//*************************************************** function input Rc & out put pwm
//****************************************************** function mototr
void OCR_(int Joy,int a)
{
  if(Joy==1){
__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,(a*2));
  }
  if(Joy==2){
__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,(a*2));
  }
  if(Joy==3){
__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,(a*2));
  }
  if(Joy==4){
__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,(a*2));
  }
                              
}

///////////////////////////////////////////////////////////
int motor_saturation(int a)
{
  if(a<MOTOR_ZERO_LEVEL)
  {
    return MOTOR_ZERO_LEVEL;
  }
   else if(a>MOTOR_MAX_LEVEL)
  {
    return MOTOR_MAX_LEVEL;
  }
  else
  return a;
}
/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
void update_motors(int m0, int m1, int m2, int m3)
{
  m0=motor_saturation(m0);
  m1=motor_saturation(m1);
  m2=motor_saturation(m2);
  m3=motor_saturation(m3);
  OCR_(1,m0);
  OCR_(2,m1);
  OCR_(3,m2); 
  OCR_(4,m3);
}
//////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////intrupts for read RC

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//you can not use bool variale in stm 32 hal it incompatibal with intrupt
{
TIM_IC_InitTypeDef sConfigIC;
TIM_IC_InitTypeDef sConfigIC1;
//****************************************************************************
//****************************************************************************
if(htim->Instance==TIM1){
  flagcheck=0;
  //**************************************************************************
  //Rc1
	if((f1==0)&&(flagcheck==0))//RISING NOW
        {
          N_RC1=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
          H_RC1=N_RC1;
          flagcheck=1;
          RC1L=RC1;
         sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);
           f1=1;
           HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
        }
        else if((f1==1)&&(flagcheck==0))
        {
          N_RC1=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);
          if(N_RC1>=H_RC1)
            RC1=N_RC1-H_RC1;
          else
            RC1=N_RC1-H_RC1+K_f_TIMER;
          flagcheck=1;
          RC1H=(int)(RC1*K_TIMER1);
          sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2);
  f1=2;
  HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
        }  
//**********************************************************************
        //RC2
	if((f1==2)&&(flagcheck==0))//RISING NOW
        {
          N_RC2=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
          H_RC2=N_RC2;
          flagcheck=1;
          RC2L=RC2;
         sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2);
           f1=3;
           HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
        }
        else if((f1==3)&&(flagcheck==0))
        {
           N_RC2=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_2);
            if(N_RC2>=H_RC2)
            RC2=N_RC2-H_RC2;
          else
            RC2=N_RC2-H_RC2+K_f_TIMER;
          flagcheck=1;
          RC2H=(int)(RC2*K_TIMER1);
          sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3);
  f1=4;
  HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
        }
 //**********************************************************************
     //RC3
	if((f1==4)&&(flagcheck==0))//RISING NOW
        {
          N_RC3=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
          H_RC3=N_RC3;
          flagcheck=1;
          RC3L=RC3;
         sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3);
           f1=5;
           HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
        }
        else if((f1==5)&&(flagcheck==0))
        {
        N_RC3=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_3);
        if(N_RC3>=H_RC3)
            RC3=N_RC3-H_RC3;
          else
            RC3=N_RC3-H_RC3+K_f_TIMER;
          flagcheck=1;
          RC3H=(int)(RC3*K_TIMER1);
          sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4);
  f1=6;
  HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
        }  
 //**********************************************************************
     //RC4
	if((f1==6)&&(flagcheck==0))//RISING NOW
        {
        N_RC4=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
        H_RC4=N_RC4;
          flagcheck=1;
          RC4L=RC4;
         sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4);
           f1=7;
           HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
        }
        else if((f1==7)&&(flagcheck==0))
        {
          N_RC4=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_4);
            if(N_RC4>=H_RC4)
            RC4=N_RC4-H_RC4;
          else
            RC4=N_RC4-H_RC4+K_f_TIMER;
          flagcheck=1;
          RC4H=(int)(RC4*K_TIMER1);
          sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);
  f1=0;
  HAL_TIM_IC_Stop_IT(&htim1,TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
        }           
}
//****************************************************************************************
//******************************************************************************************
if(htim->Instance==TIM4){
  flagcheck2=0;
  //**************************************************************************
  //Rc5
	if((f2==0)&&(flagcheck2==0))//RISING NOW
        {
          N_RC5=HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
          H_RC5=N_RC5;
          flagcheck2=1;
          RC5L=RC5;
         sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);
           f2=1;
           HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
        }
        else if((f2==1)&&(flagcheck2==0))
        {
          N_RC5=HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
           if(N_RC5>=H_RC5)
            RC5=N_RC5-H_RC5;
          else
            RC5=N_RC5-H_RC5+K_f_TIMER;
          flagcheck2=1;
          RC5H=(int)(RC5*K_TIMER2);
          sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
         sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);
  f2=2;
  HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
        }  
//**********************************************************************
        //RC6
	if((f2==2)&&(flagcheck2==0))//RISING NOW
        {
          N_RC6=HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2);
          H_RC6=N_RC6;
          flagcheck2=1;
          RC6L=RC6;
         sConfigIC1.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
         sConfigIC1.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC1.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC1.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC1, TIM_CHANNEL_2);
           f2=3;
           HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
        }
        else if((f2==3)&&(flagcheck2==0))
        {
          N_RC6=HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2);
           if(N_RC6>=H_RC6)
            RC6=N_RC6-H_RC6;
          else
            RC6=N_RC6-H_RC6+K_f_TIMER;
          flagcheck2=1;
          RC6H=(int)(RC6*K_TIMER2);
          sConfigIC1.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
         sConfigIC1.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC1.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC1.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC1, TIM_CHANNEL_1);
  f2=0;
  HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
        }
}
}

///////////////////////////////////////////////////////////////////////////
void READ_RX()
{
  rx_values[0]=RC1H;
  rx_values[1]=RC2H;
  rx_values[2]=RC3H;
  rx_values[3]=RC4H;
  rx_values[4]=RC5H;
  rx_values[5]=RC6H;
}
//*************************************************** end function input Rc & out put pwm

//***************************************************controller function
void pid_initialize() {
  roll_controller.SetOutputLimits(ROLL_PID_MIN,ROLL_PID_MAX);
  pitch_controller.SetOutputLimits(PITCH_PID_MIN,PITCH_PID_MAX);
  yaw_controller.SetOutputLimits(YAW_PID_MIN,YAW_PID_MAX);
  roll_controller.SetMode(AUTOMATIC);
  pitch_controller.SetMode(AUTOMATIC);
  yaw_controller.SetMode(AUTOMATIC);
  roll_controller.SetSampleTime(4);//1
  pitch_controller.SetSampleTime(4);//1
  yaw_controller.SetSampleTime(4);//1
}

void pid_update(){
  pid_roll_in = angleY;
  pid_pitch_in = angleX;
  pid_yaw_in = angleZ; 
}

void pid_compute() {
   roll_controller.SetTunings(axissEA[0],axissEA[2],axissEA[1]);
   pitch_controller.SetTunings(axissEA[0],axissEA[2],axissEA[1]);//(axissEA[0],axissEA[2],axissEA[1]);
   yaw_controller.SetTunings(axissR[0],axissR[2],axissR[1]);
   roll_controller.Compute();
   pitch_controller.Compute();
   yaw_controller.Compute();
}
//////////////////////////////////////////
///////////////////////////////////////////
void set_gain(double *axis)
{
  READ_RX();
  // Serial.println(pulseIn(A5,HIGH));
   if(rx_values[4]>=1800 && rx_values[4]!=0){
    int b=rx_values[5];//pulseIn(A5,HIGH);
       //ki
      axis[2]=map(b,R_min,R_max,Ki_min,Ki_max);
        
         flag_Safty=true;
  }
   if(rx_values[4]<1100 && rx_values[4]!=0){
    int b=rx_values[5];
  axis[0]=map(b,R_min,R_max,Kp_min,Kp_max);
   flag_Safty=false;
  }
   if((rx_values[4]>1100)&&(rx_values[4]<1800)){
    int b=rx_values[5];
    //Kd
      axis[1]=map(b,R_min,R_max,Kd_min,Kd_max);
      flag_Safty=false;
  }
}
//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void setpoint_update() {
  // here we allow +- 20 for noise and sensitivity on the RX controls...
  // ROLL rx at mid level?
  READ_RX();
  if(rx_values[0] > ROLL_RMID - 20 && rx_values[0] < ROLL_RMID + 20)
    pid_roll_setpoint = 0;
  else
  {
    if(rx_values[0]<(THROTTLE_RMIN+K_S_R_MIN))
      pid_roll_setpoint=-ROLL_WMIN;
      else if(rx_values[0]>(THROTTLE_RMIN+K_S_R_MAX))
      pid_roll_setpoint=-ROLL_WMAX;
      else
    pid_roll_setpoint =- map(rx_values[0],(THROTTLE_RMIN+K_S_R_MIN),(THROTTLE_RMIN+K_S_R_MAX),ROLL_WMIN,ROLL_WMAX);
  }
  //PITCH rx at mid level?
  if(rx_values[1] > PITCH_RMID - 20 && rx_values[1] < PITCH_RMID + 20)
    pid_pitch_setpoint = 0;
  else
  {
    if(rx_values[1]<(THROTTLE_RMIN+K_S_R_MIN))
      pid_pitch_setpoint=-PITCH_WMIN;
      else if(rx_values[1]>(THROTTLE_RMIN+K_S_R_MAX))
      pid_pitch_setpoint=-PITCH_WMAX;
      else
    pid_pitch_setpoint = -map(rx_values[1],(THROTTLE_RMIN+K_S_R_MIN),(THROTTLE_RMIN+K_S_R_MAX),PITCH_WMIN,PITCH_WMAX);
  }
  //YAW rx at mid level?
  if(rx_values[3] > YAW_RMID - 20 && rx_values[3] < YAW_RMID + 20)
    pid_yaw_setpoint = 0;
  else
  {
    if(rx_values[3]<(THROTTLE_RMIN+K_S_R_MIN))
      pid_yaw_setpoint=-YAW_WMIN;
      else if(rx_values[3]>(THROTTLE_RMIN+K_S_R_MAX))
      pid_yaw_setpoint=-YAW_WMAX;
    pid_yaw_setpoint = map(rx_values[3],(THROTTLE_RMIN+K_S_R_MIN),(THROTTLE_RMIN+K_S_R_MAX),YAW_WMIN,YAW_WMAX);
  }
}
//////////////////////////////////////////////////////////////////////////////////////
void control_update(){
  READ_RX();
  throttle=map(rx_values[2],THROTTLE_RMIN,THROTTLE_RMAX,MOTOR_ZERO_LEVEL,MOTOR_MAX_LEVEL);
  setpoint_update();
  pid_update();
      //LoopTime();
  pid_compute();
  double Tx=pid_pitch_out;
  double Ty=pid_roll_out;
  double Tz=pid_yaw_out;
  //Tz=0;
  // yaw control disabled for stabilization testing...
  if(flag_Safty==true)
  {
    m0=MOTOR_ZERO_LEVEL;
     m1=MOTOR_ZERO_LEVEL;
      m2=MOTOR_ZERO_LEVEL;
       m3=MOTOR_ZERO_LEVEL;
   OCR_(1,MOTOR_ZERO_LEVEL);
  OCR_(2,MOTOR_ZERO_LEVEL);
  OCR_(3,MOTOR_ZERO_LEVEL); 
  OCR_(4,MOTOR_ZERO_LEVEL);
  }
  else
  {
  m0 = (int)(throttle+(- Tx+Ty+Tz));
  m1 = (int)(throttle+(-Tx-Ty-Tz));
  m2 = (int)(throttle+(+Tx-Ty+Tz));
  m3 = (int)(throttle+(+Tx+Ty-Tz));
  }
  
  /*#ifdef SAFE
    if(throttle < THROTTLE_SAFE_SHUTOFF)
    {
      m0 = m1 = m2 = m3 = MOTOR_ZERO_LEVEL;
    }
  #endif*/
  
  update_motors(m0, m1, m2, m3);
}

//***************************************************End controller function
/////////////////////////////////////////////////*****test functions


//*******************************************end of that

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
//********************************************RC Setup & PWM
 //OCR
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  //****************************************************
  //RC
   HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
//********************************************End Rc setup & PWM
  
  
  //******************************************Time Setup
  HAL_TIM_Base_Start_IT(&htim1);
  //****************************************** End Time Setup
  
  //********************************************************************Setup all for programing
  //******************************************************************** End Setup all for programing
axissEA[0]=1.4;
axissEA[1]=0;
axissEA[2]=0;
axissR[0]=0.8;
axissR[1]=0.14;
axissR[2]=0;
////////////////////////////////////
//motortest();
/////////////////////////////////////
imu_setup();
pid_initialize();  
  //*********************************************************************************
  //*********************************************************************************  WHILE 1
  while (1)
  {
  //set_gain(axissR);
     kp=roll_controller.GetKp();
     kd=roll_controller.GetKd();
     ki=roll_controller.GetKi();
    //////////////////////////////////////////////////////////////////
HAL_Delay(5);
////////////////////////////////////////////////////////////////////////
    set_gain(axissEA);
    imu_update();
    LoopTime();
    control_update();
  }
  //**************************************************************************End WHILE 1
}
////////////////////////////////
/*void motortest()
{
    READ_RX();
   OCR_(1,MOTOR_ZERO_LEVEL);
  OCR_(2,MOTOR_ZERO_LEVEL);
  OCR_(3,MOTOR_ZERO_LEVEL); 
  OCR_(4,MOTOR_ZERO_LEVEL);
  while(1)
  {
  //int c=rx_values[4];
   //if(rx_values[4]>=1600 && rx_values[4]!=0){
    OCR_(3,1300);
   // HAL_DELAY(1000);
 // }
  //c=rx_values[4];
   //if(rx_values[4]<1100 && rx_values[4]!=0){
   // OCR_(2,1100);
   // HAL_DELAY(1000);
  //}
   //c=rx_values[4];
   //if((rx_values[4]>1100)&&(rx_values[4]<1800)){
   // OCR_(3,1100);
    //HAL_DELAY(3000);
   //} 
  }
}*/
///////////////////////////////////////
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 40000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 40000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
