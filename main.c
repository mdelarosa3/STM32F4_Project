
//                                               Manuel De La Rosa
//                                      Senior Project Part 1 - Board 1 STM32F4
//                                                   EECE 490B
//                                      California State Universiety, Chico
//                                                  May 10 2016

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4_discovery_lis302dl.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"


//---------------------------------General --------------------------------------
//Timing Variables
int ms_Ticks;
double Dt = 00.0;                   // time difference in milli seconds
int Dt2 = 0;
int button_t = 0;
int click = 0;
RCC_ClocksTypeDef RCC_Clocks;

void SysTick_Handler(void); 
void Delay(int d_Ticks);
int Button(void);
void Push_ButtonInit();

//--------------------------------Distance Sensor ------------------------------------

TIM_ICInitTypeDef  TIM_ICInitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
int distance = 0;
int Falling_edge = 0;

void Ultrasonic_TIM_Init(void);
void Ultrasonic_Distance_format(void);

//---------------------------------GPS Fuctions -----------------------------------------------
char GPS_message[72];
int GPScount = 0;
int GPS_ready = 0;

void GPS_USART3_Init();
void GPS_Time_fix(void);
void GPS_Knots_to_miles(void);
void GPS_Cordinates_to_google(void);
void GPS_Compass(void);

//----------------------------------BlueTooth Functions---------------------------------------
char stop = 'O';
int BlueTooth_Connected = 0;
char BT_Message_SendOut[101];

char BT_CMD  = 1;
char BT_AOK  = 2;
char BT_CONN = 3;
char BT_END  = 4;
char BT_Done = 4;
char BT_PAIR = 5;
char BT_FAIL = 9;
char BT_state = 0;

char response_rcvd = 0;
char responseID = 0, response = 0;

void BlueTooth_String_send( char str[]);
void BlueTooth_Data_send(char str[], int size);
void BlueTooth_Interrupt_Response();
char BlueTooth_Get_Response();
void Bluetooth_Protocol();
void BlueTooth_USART2_Init();

//---------------------------------------- GyroAccel ------------------------------------------
int byte_H;
int byte_L;
int nDigits = 0;

int Data_raw_GA = 0;
int GyroAccel_RawData[7];

double accel_x_scalled, accel_y_scalled, accel_z_scalled, gyro_x_scalled, gyro_y_scalled, gyro_z_scalled, temperature_scalled; 
static double accel_x_Offset, accel_y_Offset, accel_z_Offset, gyro_x_Offset, gyro_y_Offset, gyro_z_Offset;

char gyro_x_out[4], gyro_y_out[4], gyro_z_out[4], accel_x_out[5], accel_y_out[5], accel_z_out[5], temp_out[5];

double rad2degree = 57.3;              // Radian to degree conversion
double Filter_gain = 0.95; 
double angle_x_gyro = 0,angle_y_gyro=0,angle_z_gyro=0,angle_x_accel=0,angle_y_accel=0,angle_z_accel=0;
double angle_x, angle_y, angle_z = 0 ;

double Q_angleX = 0.001;
double Q_biasX = 0.003;
double R_measuredX = 0.03;
double angleX = 0; // Reset the angle
double biasX = 0; // Reset bias
double Px[2][2]; // the error covariance matrix

double Q_angleY = 0.001;
double Q_biasY = 0.003;
double R_measuredY = 0.03;
double angleY = 0; // Reset the angle
double biasY = 0; // Reset bias
double Py[2][2]; // the error covariance matrix

double Q_angleZ = 0.001;
double Q_biasZ = 0.003;
double R_measuredZ = 0.03;
double angleZ = 0; // Reset the angle
double biasZ = 0; // Reset bias
double Pz[2][2]; 

void GyroAccel_I2C2_Init();
void GyroAccel_SetUp();

void GyroAccel_Write(int addr, int register_GA, int data );
void GyroAccel_Read_register(int addr, int register_GA);
void GyroAccel_Read_Data();

void GyroAccel_OffsetCalc();
void GyroAccel_angleCalc();

void GyroAccel_Message_format();
void GyroAccel_Convert_Readings_To_String();
void GyroAccel_Float_to_char(float rundNumb, char out[], int nub_diplaied, int size);

double kalman_filterX( double accel_angle, double gyro_newRate, double dt);
double kalman_filterY( double accel_angle, double gyro_newRate, double dt);
double kalman_filterZ( double accel_angle, double gyro_newRate, double dt);

//--------------------------------------------------------------------------------------------------------




//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------

 int main(void){
  // SysTick end of count event each 1ms
  RCC_GetClocksFreq(&RCC_Clocks); 
  SysTick_Config(RCC_Clocks.HCLK_Frequency /1000);
 
  Push_ButtonInit();
  BlueTooth_USART2_Init();
  GyroAccel_I2C2_Init();
  Delay(500);
  GPS_USART3_Init();
  Ultrasonic_TIM_Init();
  
  STM_EVAL_LEDInit( LED3);      // LED Inits
  STM_EVAL_LEDInit( LED4);
  STM_EVAL_LEDInit( LED5);
  STM_EVAL_LEDInit( LED6);
  
  STM_EVAL_LEDOff(LED3);         // Orange  North   PD13
  STM_EVAL_LEDOff(LED4);         // Green   West    PD12     
  STM_EVAL_LEDOff(LED5);         // Red     East    PD14
  STM_EVAL_LEDOff(LED6);         // Blue    South   PD15
  
  Bluetooth_Protocol();   
  Delay(500);
  USART_SendData(USART2, '\n');
  BlueTooth_Data_send("Gy", 2);
  USART_SendData(USART2, '\n');
  Delay(500);
  
  GyroAccel_SetUp();
  GyroAccel_Read_Data();
  
  Dt = 0;
  while(1){
    if(Dt2 >= 60){
      Dt2 = 0;
      int timer = 131;
      GPIO_SetBits(GPIOA, GPIO_Pin_6);
      while(timer > 0){
        timer--;
      }
      GPIO_ResetBits(GPIOA, GPIO_Pin_6);
    }
  
    if(Dt >= 20){
      GyroAccel_angleCalc();
      Dt = 0;
      GyroAccel_Convert_Readings_To_String();
      GyroAccel_Message_format();
    }
    if(GPS_ready){
      GPS_ready =0;
      if( GPS_message[17]== 'A'){
     
        GPS_Time_fix();
        GPS_Cordinates_to_google();
        GPS_Knots_to_miles();
        GPS_Compass();
        
        GyroAccel_Message_format();
        Ultrasonic_Distance_format();
        
        if( Button() == 1){
          BT_Message_SendOut[100] = '1';
        }
        else{
          if(click){
            click =0;
            BT_Message_SendOut[100] = 'S';
          }
          else{
            BT_Message_SendOut[100] = '0';
          }
        }
        if (stop == 'O'){
          BlueTooth_Data_send( &BT_Message_SendOut[0], 101);
          USART_SendData(USART2, 0);  
          USART_SendData(USART2, '\n'); 
        }
      }
      else{
        if (stop == 'O'){
          BlueTooth_String_send( "No GPS Signal   ");
          USART_SendData(USART2, '\n');
        }
      }
    }
    if(GPS_message[17]== '\0'){
      if (stop == 'O'){
        BlueTooth_String_send( "GPS Not Configured ");
        USART_SendData(USART2, 0);  
        USART_SendData(USART2, '\n');
      }
    }
  }
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------





//------------------------ Disstance Sesor---------------------------------------------------
void Ultrasonic_Distance_format(void){
  distance = (Falling_edge-114.32)/223.23; // Calculate distance in cm.
  int digit = 0;
  BT_Message_SendOut[96] = (distance/100)+ ('0');
  digit = distance/100;
  BT_Message_SendOut[97] = (distance - digit*100 )/10 + ('0');
  digit = distance/10;
  BT_Message_SendOut[98] = (distance - (digit*10)) + ('0'); 
  BT_Message_SendOut[99] = 0;
}

void Ultrasonic_TIM_Init(void){
  int PrescalerValue;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // TIM4 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  // GPIOB clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  // TIM4 chennel2 configuration : PE.9 
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // TIM4_CH1 pin (PB.06) configuration 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // Connect TIM pin to AF2 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
  // Enable the TIM4 global Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseStructure.TIM_Prescaler =  0xFFFF;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period=  0xFFFFFFFF;
  TIM_TimeBaseStructure.TIM_ClockDivision= TIM_CKD_DIV4;
 // TIM_TimeBaseStructure.TIM_RepetitionCounter =
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 12000000) - 1;
  TIM_PrescalerConfig(TIM3, PrescalerValue, TIM_PSCReloadMode_Immediate);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

  // Select the TIM3 Input Trigger: TI2FP2 
  TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);

  // Select the slave Mode: Reset Mode
  TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
  TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);
  
  //TIM enable counter 
  TIM_Cmd(TIM3, ENABLE);

  // Enable the CC2 Interrupt Request
  TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
  
}

void TIM3_IRQHandler(void){

  // Clear TIM4 Capture compare interrupt pending bit 
  TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
  // Get the Input Capture value 
  if( !GPIO_ReadInputDataBit( GPIOA, GPIO_Pin_7) ){
    Falling_edge = TIM_GetCapture2(TIM3);
  }
}

//-------------------------------------------------------------------------------------------
//-------------------------- Master Bluetooth start Master ----------------------------------

void Bluetooth_Protocol(){
  //Addrss 00-06-66-65-39-E6
  Delay(300);
 
  BlueTooth_String_send("---");                         // Exit command mode
  USART_SendData(USART2, 13);
  Delay(1500);
    
  do {
    BlueTooth_String_send("$$$");                      // Enter command mode
    Delay(1000);
  } while (BlueTooth_Get_Response() != BT_CMD);
  
  BlueTooth_String_send("R,1");                         // Reboot
  USART_SendData(USART2, 13);
  Delay(1500);
  
  do {
    BlueTooth_String_send("$$$");                      // Enter command mode
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_CMD);

  do {
    BlueTooth_String_send( "SN,Bluetooth-Master");         // Name of device
    USART_SendData(USART2, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  do {
    BlueTooth_String_send("SO,Master");                      // Extended status string
    USART_SendData(USART2, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  do {
    BlueTooth_String_send("SM,0");                     // Set mode (0 = slave, 1 = master, 2 = trigger, 3 = auto, 4 = DTR, 5 = ANY)
    USART_SendData(USART2, 13);                // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  do {
    BlueTooth_String_send("SA,1");                      // Authentication (1 to enable, 0 to disable)
    USART_SendData(USART2, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  do {
    BlueTooth_String_send("SP,mikroe");                      // Security pin code (mikroe)
    USART_SendData(USART2, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);
  
  
  BlueTooth_String_send("R,1"); 
  USART_SendData(USART2, 13);
  Delay(1500);
  
  do {
    BlueTooth_String_send("$$$");                      // Enter command mode
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_CMD);
 
  BlueTooth_String_send("I,10");                       // Looks for Bluetooth devicese)
  USART_SendData(USART2, 13);                   // CR
  while (BlueTooth_Get_Response() != BT_Done);
  
  do {
    BlueTooth_String_send("SR,00066665ED0C");               // Store the address just found if there is one only device found
                                                // (if there is more than one device, you need to select one by its exact address)
    USART_SendData(USART2, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  
  BlueTooth_String_send("C,00066665ED0C");                       // Connect
  USART_SendData(USART2, 13);                   // CR
  while ( BlueTooth_Get_Response() != BT_CONN);
  Delay(2000);
  BlueTooth_Connected = 1;
}

void BlueTooth_Interrupt_Response(){
  char tmp;
  tmp = USART_ReceiveData( USART2);                         // Get received byte

  /* The responses expected from the EasyBT module:
  CMD
  AOK
  AOK
  AOK
  AOK
  AOK
  Inquiry, COD=0
  Found 1
  0006660494E3,,55AA
  Inquiry Done
  AOK
  TRYING
  MasterCONNECT*/

  // Process reception through state machine
  // We are parsing CMD<cr><lf>, AOK<cr><lf>, CONN and Done responses
  switch (BT_state) {
    case  0: {
        response = 0;                   // Clear response
        if (tmp == 'C')                 // We have 'C', it could be CMD<cr><lf>  or CONN
          BT_state = 1;                 // Expecting 'M' or 'N'
        if (tmp == 'A')                 // We have 'A', it could be AOK<cr><lf>
          BT_state = 11;                // expecting 'O'
        if (tmp == 'D')                 // We have 'D', it could be "Done"
          BT_state = 31;                // expecting 'o'
        if (tmp == 'E')                 // We have 'E', it could be "END"
          BT_state = 31;                // expecting 'N'
        if (tmp == 'N') 
          BT_state = 1;
        if (tmp == 'f') 
          BT_state = 1;
        break;                          // ...
    }
    case  1: {
        if (tmp == 'M')
          BT_state = 2;
        else if (tmp == 'O')
          BT_state = 22;
        else if (tmp == 'E') 
          BT_state = 2;
        else if (tmp == 'a') 
          BT_state = 2;
        else
          BT_state = 0;
        break;
    }
    case  2: {
        if (tmp == 'D') {
          response = BT_CMD;            // CMD
          BT_state = 40;
        }
        else if (tmp == 'W') 
          BT_state = 11;
        else if (tmp == 'i') 
          BT_state = 11;
        else
          BT_state = 0;
        break;
    }
    case 11: {
        if (tmp == 'O')
          BT_state = 12;
        else if (tmp == '_') 
          BT_state = 12;
        else if (tmp == 'l'){ 
          BT_state = 0;
          response = BT_FAIL;           // "Inquiry Done"
          response_rcvd = 1;
          responseID = response;
        }
        else
          BT_state = 0;
        break;
    }
    case 12: {
        if (tmp == 'K'){
          response = BT_AOK;
          BT_state = 40;
        }
        else if (tmp == 'P'){
          BT_state = 0;
          response = BT_PAIR;           // "Inquiry Done"
          response_rcvd = 1;
          responseID = response;
        }
        else
          BT_state = 0;
        break;
    }
    case 22: {
        if (tmp == 'N')
          BT_state = 23;
        else
          BT_state = 0;
        break;
    }
    case 23: {
        if (tmp == 'N') {
          response = BT_CONN;           // MasterCONNECT
          response_rcvd = 1;
          responseID = response;
        }
        BT_state = 0;
        break;
    }
    case 31: {
        if (tmp == 'o')
          BT_state = 32;
        else if (tmp == 'N')
          BT_state = 32;
        else
          BT_state = 0;
        break;
    }
    case 32: {
        if (tmp == 'n')
          BT_state = 33;
        else if (tmp == 'D') {
          response = BT_END;            // END
          BT_state = 40;
        }
        else
          BT_state = 0;
        break;
    }
    case 33: {
        if (tmp == 'e') {
          response = BT_Done;           // "Inquiry Done"
          response_rcvd = 1;
          responseID = response;
        }
        BT_state = 0;
        break;
    }
    case 40: {
        if (tmp == 13)
          BT_state = 41;
        else
          BT_state = 0;
        break;
    }
    case 41: {
        if (tmp == 10){
          response_rcvd = 1;
          responseID = response;
        }
          BT_state = 0;
        break;
    }
    default: {
        BT_state = 0;
        break;
    }
  }
}

char BlueTooth_Get_Response() {
  // Get BlueTooth response, if there is any
  if (response_rcvd) {
    response_rcvd = 0;
    return responseID;
  }
  else
    return 0;
}

void BlueTooth_String_send(char str[]){
  int i = 0;
  while(str[i] != '\0'){
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, str[i]);
    i++;
  }
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

void BlueTooth_Data_send(char str[], int size){
  int i = 0;
  while(i < size){
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    USART_SendData(USART2, str[i]);
    i++;
  }
  while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}


void BlueTooth_USART2_Init(){ 
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable the APB1 periph clock for USART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  // Enable the GPIOD clock, used by pins PD5, PD6
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  // Setup the GPIO pins for Tx and Rx
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;   //PD5 Tx , PD6 Rx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  // Connect PA2 and PA3 with the USART2 Alternate Function
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART2, &USART_InitStructure);

  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(USART2, ENABLE);
}

void USART2_IRQHandler(void){

  if( USART_GetITStatus(USART2, USART_IT_RXNE)){        
    if(BlueTooth_Connected == 0){
      BlueTooth_Interrupt_Response();
    }
    else{
       stop = USART_ReceiveData( USART2);
    }
  }
}

//-------------------------------------------------------------------------------------------
//------------------------ GPS  Start -------------------------------------------------------

void GPS_Compass(void){
  
  if(GPS_message[52] == ','){
    BT_Message_SendOut[44] = 'N';     // check BT_Message_SendOut[ 67 ] for N if not display no velocity
    BT_Message_SendOut[45] = ' ';
    BT_Message_SendOut[46] = ' ';
    BT_Message_SendOut[47] = ' ';
    BT_Message_SendOut[48] = ' ';
    BT_Message_SendOut[49] = ' ';
    BT_Message_SendOut[50] = 0;

  }
  else{

    BT_Message_SendOut[44] = ' ';  
    BT_Message_SendOut[45] = GPS_message[52];
    BT_Message_SendOut[46] = GPS_message[53];
    BT_Message_SendOut[47] = GPS_message[54];
    BT_Message_SendOut[48] = GPS_message[55];
    BT_Message_SendOut[49] = GPS_message[56];
    BT_Message_SendOut[50] = 0;                // 86
                      
  }
}

void GPS_Cordinates_to_google(void){
  // Latitude 
  int digits = 0;
  int digits2 = 0;
  int digits3 = 0;
  int multiplier = 8;
  BT_Message_SendOut[12] = ' '; 
  
  for(int i =0; i < 8; i++){
    if(GPS_message[21+i] != '.'){
      digits = digits + ((GPS_message[21+i]- '0')*pow(10,multiplier)); 
      multiplier--;
    }
  }
  digits = digits;
  digits = digits/6;
  multiplier = 7;
  GPS_message[21] = '.';
  for(int i = 0; i < 7; i++){
    digits2 = digits/(pow(10,multiplier));
    digits = digits-digits2*pow(10, multiplier);
    GPS_message[22+i] = digits2 + '0';
    multiplier--;
  }
  for(int i =0; i < 10; i++){
    BT_Message_SendOut[13+i]= GPS_message[19+i]; 
  }
  BT_Message_SendOut[23] = GPS_message[30]; 
  BT_Message_SendOut[24] = ' ';            
  
  // Longitude
  digits = 0;
  digits2 = 0;
  multiplier = 8;
  for(int i =0; i < 8; i++){
    if(GPS_message[35+i] != '.'){
      digits = digits + ((GPS_message[35+i]- '0')*pow(10,multiplier)); 
      multiplier--;
    }
  }
  digits = digits/6;
  multiplier = 7; 
  GPS_message[35] = '.';
  for(int i = 0; i < 7; i++){
    digits2 = digits/(pow(10,multiplier));
    digits = digits-digits2*pow(10, multiplier);
    GPS_message[36+i] = digits2 + '0';
    multiplier--;
  }
  for(int i =0; i < 11; i++){
    BT_Message_SendOut[25+i]= GPS_message[32+i];  //25
  }
  BT_Message_SendOut[36] =  GPS_message[44];      //36
  BT_Message_SendOut[37]= 0; // 0 to end the display string //44
}

void GPS_Knots_to_miles(void){

  float speed = 0;
  int speed2 =0;
  int speed3 =0;
  int i = 0;
  int i2 = 0;
  int i3 = 0;

  BT_Message_SendOut[38] = ' ';  
  while(GPS_message[46+i]!= '.'){
    i++;
  }
  i3 = i-1;
  i=i+1;
  while(i2 <= i){
    if(GPS_message[46+i2] != '.'){
      speed = speed + pow(10,i3)*(GPS_message[46+i2]-'0');
      i3--;
    }
    i2++;
  }
  if(speed > 0){  
    speed3 = speed * 1.15078;
    BT_Message_SendOut[39] = (speed3/100)+ ('0');
    speed2 = speed3/100;
    BT_Message_SendOut[40] = (speed3 - speed2*100 )/10 + ('0');
    speed2 = speed3/10;
    BT_Message_SendOut[41] = (speed3 - (speed2*10)) + ('0'); 
  }
  else{
    BT_Message_SendOut[39] = '0'; 
    BT_Message_SendOut[40] = '0';
    BT_Message_SendOut[41] = '0';
  }
  BT_Message_SendOut[42] = ' ';
  BT_Message_SendOut[43] = 0;    //66
}

void GPS_Time_fix(void){
  int digits = 0;
  GPS_message[14] = 'M';
  GPS_message[15] = '.';
  if(GPS_message[7]== '0') { 
      if ( GPS_message[8] <= '7'){
        if( GPS_message[7] == '7'){
          GPS_message[13] = 'A';
        }
        else{
          GPS_message[13] = 'P';
        }
        digits = (GPS_message[8]-'0')+12 - 7;
        GPS_message[7] = (digits/10) + '0';
        GPS_message[8] = (digits%10) + '0';
      }   
      else if (GPS_message[8] == '8'){
        GPS_message[8] = '1';
        GPS_message[13] = 'P';
      }
  }
  else if (GPS_message[7] == '1' || (GPS_message[7]== '2' && GPS_message[8]=='0')){
      if( GPS_message[7] == '1'){
        GPS_message[13] = 'A';
      }
      else{
        GPS_message[13] = 'P';
      }
      digits = ( (GPS_message[7]-'0') *10) + (GPS_message[8]-'0');
      digits = digits - 7;
      GPS_message[7] = (digits/10) + '0';
      GPS_message[8] = (digits%10) + '0';
  }
  else if (GPS_message[7] == '2'){
      GPS_message[13] = 'P';
      digits = ((GPS_message[7]-'0')*10) + (GPS_message[8] - '0');
      digits = digits - 12 - 7;
      GPS_message[7] = (digits/10) + '0';
      GPS_message[8] = (digits%10) + '0';
  }

  BT_Message_SendOut[0] = GPS_message[7];
  BT_Message_SendOut[1] = GPS_message[8];
  BT_Message_SendOut[2] = ':';
  BT_Message_SendOut[3] = GPS_message[9];
  BT_Message_SendOut[4] = GPS_message[10];
  BT_Message_SendOut[5] = ':';
  BT_Message_SendOut[6] = GPS_message[11];
  BT_Message_SendOut[7] = GPS_message[12];
  BT_Message_SendOut[8] = ' ';
  BT_Message_SendOut[9] = GPS_message[13];
  BT_Message_SendOut[10] = GPS_message[14];
  BT_Message_SendOut[11] = 0;
  
}

void GPS_USART3_Init(){  

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable the APB1 periph clock for USART2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  // Enable the GPIOA clock, used by pins PA2, PA3
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  // Setup the GPIO pins for Tx and Rx
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;   //PC-10 Tx , PC-11 Rx
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // Connect PA2 and PA3 with the USART2 Alternate Function
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

  USART_InitStructure.USART_BaudRate = 230400;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  USART_Init(USART3, &USART_InitStructure);

  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void){
 
  if( USART_GetITStatus(USART3, USART_IT_RXNE) ){
    GPS_message[GPScount] = USART_ReceiveData( USART3);
    if(GPS_message[GPScount] == '\n'){
      GPS_ready = 1;
      GPScount = 0;
    }
    else
      GPScount++;
  }
}

//-------------------------------------------------------------------------------------------
//------------------------Gyrosocope Acellerometer  Start------------------------------------

void GyroAccel_I2C2_Init(void){
  
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);     
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);    //SCL
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);    //SDA
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
        
    I2C_DeInit(I2C2);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;       
    I2C_InitStructure.I2C_OwnAddress1 = 0x4F;
//  I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
//  I2C_AnalogFilterCmd(I2C2, ENABLE);
//  I2C_InitStructure.I2C_DigitalFilter = 0x00;
//  I2C_DigitalFilterConfig(I2C2, 0x00);
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//  I2C_InitStructure.I2C_Timing = 30000; // 30k
    I2C_InitStructure.I2C_ClockSpeed = 400000;          //Less than 400kHz

    I2C_Init(I2C2, &I2C_InitStructure);  
    I2C_Cmd(I2C2, ENABLE);
}

void GyroAccel_SetUp(){
  // 0xD2 MPU Addres
  //Device address, register_GA,  data  Write
  Delay(100);
  GyroAccel_Write(0xD2, 0x6B, 0x80 );      // Reset device  Register PW R_MGMT_1= 0x6B
  Delay(100);
  GyroAccel_Write(0xD2, 0x6B, 0x00 );
  Delay(100); 
  /*GyroAccel_Write(0xD2, 0x23, 0x78 );   //FIFO_EN  for Temp, Gyro, Accel but not for slaves
  Delay(100);
  GyroAccel_Write(0xD2, 0x38, 0x10 );      //I2C_SLV0_REG
  Delay(100);
  */
 // GyroAccel_Read_register(0xD2, 0x1B);    //read Device address, register_GA
 // Delay(100);  
  //Data_raw_GA = Data_raw_GA & 0xE0;
  GyroAccel_Write(0xD2, 0x1B, 0x00 );       // GYRO_CONFIG   FS_Sel = 0:  +/-250 Degress/Second
  Delay(100);
  
  GyroAccel_Write(0xD2, 0x1C, 0x08 );       // ACCEL_CONFIG  AFS_Sel = 1: +/-4G 
  Delay(100);
  /*
  GyroAccel_Read_register(0xD2, 0x38);      //Read
  Delay(100);  
  Data_raw_GA = Data_raw_GA | 0x11;
  GyroAccel_Write(0xD2, 0x38, Data_raw_GA );
  Delay(100); 
  */
  
  GyroAccel_Write(0xD2, 0x1A, 0x00 );       // CONFIG  DLPF_CFG = 0 : Accel Bandwith 260Hz, Gyro BandW 256, Delay 0.98, Fs 8kHz
  Delay(100); 
  
  GyroAccel_OffsetCalc();
  
  GyroAccel_Write(0xD2, 0x1A, 0x06 );       // CONFIG  DLPF_CFG = 0 : Accel Bandwith 260Hz, Gyro BandW 256, Delay 0.98, Fs 8kHz
  Delay(100); 
}

void GyroAccel_Write(int addr, int register_GA, int data ){

  I2C_GenerateSTART(I2C2, ENABLE);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));           // Slave has acknowledged start condition
  I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Transmitter );          // Send MPU address
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C2, register_GA);                                      //  Send Register addres
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  I2C_SendData(I2C2, data);                                             // Send Data
  while(I2C_GetFlagStatus(I2C2,  I2C_FLAG_BUSY) == RESET);
  I2C_GenerateSTOP(I2C2, ENABLE);
}

void GyroAccel_Read_register(int addr, int register_GA){
  I2C_GenerateSTART(I2C2, ENABLE);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)); // Slave has acknowledged start condition
  I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Transmitter );
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C2,  register_GA);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
 
  I2C_GenerateSTART(I2C2, ENABLE);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C2, addr,  I2C_Direction_Receiver ); //Read
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);       // Stop
  while( !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) );
  Data_raw_GA = I2C_ReceiveData(I2C2);
 
}

void GyroAccel_Read_Data(){
  byte_H = 0;
  byte_L = 0;
  int addr = 0xD2;                       // MPU Address
  int register_GA_start = 0x3B;          // First register accel_x reads 7 regiester
  
  I2C_GenerateSTART(I2C2, ENABLE);
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));           // Slave has acknowledged start condition
  I2C_Send7bitAddress(I2C2, addr, I2C_Direction_Transmitter );          // Slave Address and The Write bit
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  I2C_SendData(I2C2,  register_GA_start);                               // Send Addres Register First(3B)
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  I2C_GenerateSTART(I2C2, ENABLE);                                      // Star signal to start reading
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
  I2C_Send7bitAddress(I2C2, addr,  I2C_Direction_Receiver );            //Read
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  I2C_AcknowledgeConfig(I2C2, ENABLE);
  
  for(int i =0; i < 7; i++){  
    while( !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    byte_H = I2C_ReceiveData(I2C2);
    while( !I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) );
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    byte_L = I2C_ReceiveData(I2C2);
    if(byte_H >= 128){                  // Conver bits to intergers
      byte_H = ~byte_H;
      byte_H = byte_H & 0x00FF;
      byte_L = ~byte_L;
      byte_L = byte_L & 0x00FF;
      GyroAccel_RawData[i] = ((byte_H << 8) | byte_L)+1;
      GyroAccel_RawData[i] = (GyroAccel_RawData[i]*(-1));       // 
    }
    else {
      GyroAccel_RawData[i] = ((byte_H << 8) | byte_L);
    
    }
  }
  I2C_AcknowledgeConfig(I2C2, DISABLE);
  I2C_GenerateSTOP(I2C2, ENABLE);       // Stop
  
  accel_x_scalled = (float)((GyroAccel_RawData[0] - accel_x_Offset)/8191.75);       // 16 bits so 2^15 - 1 = 32767   32767/(4G)  = 8191.75/G
  accel_y_scalled = (float)((GyroAccel_RawData[1] - accel_y_Offset)/8191.75);
  accel_z_scalled = (float)((GyroAccel_RawData[2] - accel_z_Offset)/8191.75 );
  
  temperature_scalled = (float)((GyroAccel_RawData[3]/340)+36.53);
  temperature_scalled = temperature_scalled*1.8+32;
  
  gyro_x_scalled  = (float)((GyroAccel_RawData[4] - gyro_x_Offset)/131.068);       // 16 bits so 2^15 - 1 = 32767      32767/(250 degrees/s) = 131.048 s/dgreess 
  gyro_y_scalled  = (float)((GyroAccel_RawData[5] - gyro_y_Offset)/131.068);
  gyro_z_scalled  = (float)((GyroAccel_RawData[6] - gyro_z_Offset)/131.068);
  
}

void GyroAccel_OffsetCalc(){
  int x, y, z, i = 0;
  int x2, y2, z2 = 0;
  
  GyroAccel_Read_Data();
  //Take the average of 1000 examples
  for(i = 0; i< 1000; i++){
    GyroAccel_Read_Data();
    x = (x+GyroAccel_RawData[0]);       // Accelerometer
    y = (y+GyroAccel_RawData[1]);
    z = (z+GyroAccel_RawData[2]);
    
    x2 = (x2+GyroAccel_RawData[4]);     // Gyroscope
    y2 = (y2+GyroAccel_RawData[5]);
    z2 = (z2+GyroAccel_RawData[6]);
    Delay(25);
  }
  accel_x_Offset = x/i;
  accel_y_Offset = y/i;
  accel_z_Offset =(z/i) - 8191.75 ;  // Z/i - (the gravity acceleration 1G) 
  
  gyro_x_Offset = x2/i;
  gyro_y_Offset = y2/i;
  gyro_z_Offset = (z2/i);
  
}

void GyroAccel_angleCalc(){
  GyroAccel_Read_Data();
  double dt = Dt/1000;
  
  //Calculate rotational agle from gravity
  angle_y_accel = atan2(-accel_x_scalled, accel_z_scalled)*rad2degree;  // we use gravity in the z axis
  angle_x_accel = atan2(accel_y_scalled, accel_z_scalled)*rad2degree;   // to calculate the tilt angle
 
  //Calculate rotational angel with a Kalman filter using angle mesurement and angles calculate(from gravity)
  angle_x = kalman_filterX( angle_x_accel, gyro_x_scalled, dt );
  angle_y = kalman_filterY( angle_y_accel, gyro_y_scalled, dt );
  // angle_z = kalman_filterZ(gyro_z_scalled, accel_z_scalled, Dt );
  
  //angle_x = angle_x + gyro_x_scalled*(dt);
  //angle_y = angle_y + gyro_y_scalled*(dt);
  angle_z = angle_z + gyro_z_scalled*(dt);      // no Kalman filter becase we dont have a compass
}

void GyroAccel_Convert_Readings_To_String() {
  // Convert accel readings
  GyroAccel_Float_to_char(accel_x_scalled, accel_x_out, 1, 5);
  GyroAccel_Float_to_char(accel_y_scalled, accel_y_out, 1, 5);
  GyroAccel_Float_to_char(accel_z_scalled, accel_z_out, 1, 5);
  // Convert temperature readings
  GyroAccel_Float_to_char(temperature_scalled, temp_out, 3, 5);
  // Convert gyro readings
  GyroAccel_Float_to_char(angle_x, gyro_x_out, 3, 4);
  GyroAccel_Float_to_char(angle_y, gyro_y_out, 3, 4);
  GyroAccel_Float_to_char(angle_z, gyro_z_out, 3, 4);
}

void GyroAccel_Float_to_char(float rundNumb, char out[], int numb_displaied, int size ){
  int i = 0;
  float digits = rundNumb;
  int digits2 = 0;
  nDigits = 2;
  int on = 0;
  int count_done = 1;
  // Specify if posity or negative 
  if(digits < 0){
    out[i] = '-';
    digits = digits*(-1);
  }
  else{
    out[i] = '+';
  }

  while(count_done){
    if( ((digits/pow(10, nDigits)) < 1.0) && (nDigits > -4)){ // Find the exponent
      nDigits--;
    }
    else {
      count_done = 0;
    }    
  } 
  // format must be 0.XX if less then 1 for acceleration
  // format must have no decimal for degrees and temprature
  for(i = 1; i < size; i++){               
    if( numb_displaied-1 > nDigits && (numb_displaied > 0 || on )) {
      out[i]= '0';
      numb_displaied--;
    }
    else if( (nDigits >= 0) || on){
        digits2 = digits/(pow(10, nDigits));
        digits = digits-digits2*pow(10, nDigits);
        out[i] = digits2 + '0';
        nDigits--;
        numb_displaied--;
    }
    else{               
        out[i] = '.';
        on = 1;
    }
  }
}

void GyroAccel_Message_format(){

  BT_Message_SendOut[51] = ' ';  
  for(int i = 0; i < 5 ; i++){
    BT_Message_SendOut[52+i] = accel_x_out[i]; 
  }
  BT_Message_SendOut[57] = ' ';
  for(int i = 0; i < 5 ; i++){
    BT_Message_SendOut[58+i] = accel_y_out[i]; 
  }
  BT_Message_SendOut[63] = ' ';
  for(int i = 0; i < 5 ; i++){
    BT_Message_SendOut[64+i] = accel_z_out[i]; 
  }
  BT_Message_SendOut[69] = ' ';
  BT_Message_SendOut[70] = 0;
  
  BT_Message_SendOut[71] = ' ';
  for(int i = 0; i < 4 ; i++){
    BT_Message_SendOut[72+i] = gyro_x_out[i]; 
  }
  BT_Message_SendOut[76] = ' ';
  for(int i = 0; i < 4 ; i++){
    BT_Message_SendOut[77+i] = gyro_y_out[i]; 
  }
  BT_Message_SendOut[81] = ' ';
  for(int i = 0; i < 4 ; i++){ 
    BT_Message_SendOut[82+i] = gyro_z_out[i]; 
  }
  BT_Message_SendOut[86] = ' ';
  BT_Message_SendOut[87] = 0;
  BT_Message_SendOut[88] = ' ';
  for(int i = 0; i < 5 ; i++){
    BT_Message_SendOut[89+i] = temp_out[i]; 
  }
  BT_Message_SendOut[94] = ' ';
  BT_Message_SendOut[95] = 0;

}

double kalman_filterX( double accel_angle, double gyro_newRate, double dt){
  
  double rate;
  double Kx[2]; // Kalman gain - This is a 2x1 vector
  double Yx; // Angle difference
  double Sx; // Estimate error
  
  // X = F*x + Bu 
  rate = (gyro_newRate - biasX);    // Angle = angle + dt(newRate - bias) 
  angleX = angleX + dt * rate;
  
  Px[0][0] = Px[0][0] + dt * ( dt*Px[1][1] - Px[0][1] -Px[1][0] + Q_angleX );
  Px[0][1] = Px[0][1] - dt * Px[1][1];
  Px[1][0] = Px[1][0] - dt * Px[1][1];
  Px[1][1] = Px[1][1] + dt * Q_biasX;
   

  Sx = Px[0][0] + R_measuredX;
  
  Kx[0] = Px[0][0]/Sx;
  Kx[1] = Px[1][0]/Sx;

  Yx =  accel_angle - angleX;
  
  angleX = angleX + Kx[0]*Yx;

  biasX = biasX+ Kx[1]*Yx;
  
  Px[0][0] = Px[0][0] - Kx[0]*Px[0][0];
  Px[0][1] = Px[0][1] - Kx[0]*Px[0][1];
  Px[1][0] = Px[1][0] - Kx[1]*Px[0][0];
  Px[1][1] = Px[1][1] - Kx[1]*Px[0][1];
  
  return angleX;
}

double kalman_filterY( double accel_angle, double gyro_newRate, double dt){
  
  double rate;
  double Ky[2]; // Kalman gain - This is a 2x1 vector
  double Yy; // Angle difference
  double Sy; // Estimate error
  
  // X = F*x + Bu 
  rate = (gyro_newRate - biasY);    // Angle = angle + dt(newRate - bias) 
  angleY = angleY + dt * rate;

  Py[0][0] = Py[0][0] + dt * ( dt*Py[1][1] - Py[0][1] -Py[1][0] + Q_angleY );
  Py[0][1] = Py[0][1] - dt * Py[1][1];
  Py[1][0] = Py[1][0] - dt * Py[1][1];
  Py[1][1] = Py[1][1] + dt * Q_biasY;
  
  Sy = Py[0][0] + R_measuredY;
  
  Ky[0] = Py[0][0]/Sy;
  Ky[1] = Py[1][0]/Sy;
  
  Yy =  accel_angle - angleY;
  
  angleY = angleY + Ky[0]*Yy;

  biasY = biasY+ Ky[1]*Yy;
  
  Py[0][0] = Py[0][0] - Ky[0]*Py[0][0];
  Py[0][1] = Py[0][1] - Ky[0]*Py[0][1];
  Py[1][0] = Py[1][0] - Ky[1]*Py[0][0];
  Py[1][1] = Py[1][1] - Ky[1]*Py[0][1];
  
  return angleY;
}

double kalman_filterZ( double accel_angle, double gyro_newRate, double dt){

  double rate;
  double Kz[2]; // Kalman gain - This is a 2x1 vector
  double Yz; // Angle difference
  double Sz; // Estimate error
  
  // X = F*x + Bu 
  rate = (gyro_newRate - biasZ);    // Angle = angle + dt(newRate - bias) 
  angleZ = angleZ + dt * rate;

  Pz[0][0] = Pz[0][0] + dt * ( dt*Pz[1][1] - Pz[0][1] -Pz[1][0] + Q_angleZ );
  Pz[0][1] = Pz[0][1] - dt * Pz[1][1];
  Pz[1][0] = Pz[1][0] - dt * Pz[1][1];
  Pz[1][1] = Pz[1][1] + dt * Q_biasZ;
  
  Sz = Pz[0][0] + R_measuredZ;
  
  Kz[0] = Pz[0][0]/Sz;
  Kz[1] = Pz[1][0]/Sz;
  
  Yz =  accel_angle - angleZ;
  
  angleZ = angleZ + Kz[0]*Yz;

  biasZ = biasZ+ Kz[1]*Yz;
  
  Pz[0][0] = Pz[0][0] - Kz[0]*Pz[0][0];
  Pz[0][1] = Pz[0][1] - Kz[0]*Pz[0][1];
  Pz[1][0] = Pz[1][0] - Kz[1]*Pz[1][0];
  Pz[1][1] = Pz[1][1] - Kz[1]*Pz[1][1];
  
  return angleZ;
}

//-------------------------------------------------------------------------------------------


// Delays number of Systicks (happens every 1 ms)
void Delay( int d_Ticks){                                              
  ms_Ticks = d_Ticks;
  while( ms_Ticks > 1);
}

void SysTick_Handler(void) {
  ms_Ticks--;
  Dt++;
  Dt2++;
  button_t++;
}


// -----------------------Push_ButonInit ----------------------------------------------------
void Push_ButtonInit(){
  GPIO_InitTypeDef   GPIO_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;
  EXTI_InitTypeDef   EXTI_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // Eanable GPIOA clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);// Enable SYSCFG clock
  
  // Configure PA0 pin as input floating 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // Enable SYSCFG clock 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  // Connect EXTI0 Line to PA0 pin 
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  // Configure EXTI0 line 
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI0 Interrupt to the lowest priority 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
}


int Button(void){ 
  int on = 0;
  on =  ( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0 ));
  return on; 
}

void EXTI0_IRQHandler(void){

  if ((EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) == SET)&&(STM_EVAL_PBGetState(BUTTON_USER) != RESET)){
    button_t = 0;
  }
  else if ((EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) == SET)){
    if( button_t < 100){
      click = 1;
    }
    else{
      click = 0;
    }
    button_t = 0;
  }
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);      // Clear the EXTI line 0 pending bit
} 

