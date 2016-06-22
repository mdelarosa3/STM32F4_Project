
//                                               Manuel De La Rosa
//                                      Senior Project Part 2 - Board 2 STM32F4
//                                                   EECE 490B
//                                      California State Universiety, Chico
//                                                  May 10 2016




// Includes ------------------------------------------------------------------
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lcd.h"
#include "stm32f4_discovery_lis302dl.h"
#include "main.h"
#include "bmp.h"
#include "dcmi_ov9655.h"
#include <cstring>
#include "stdbool.h"
#include <stdio.h>
#include "stm32f4_discovery_sdio_sd.h"
#include "stm32f4_discovery_debug.h"
#include "ff.h"

SD_Error Status = SD_OK;
FATFS filesystem;		// volume lable
FRESULT ret;			// Result code 
FIL file;                       // File object 
DIR dir;		        // Directory object
FILINFO fno;			// File information object 
UINT bw, br;
uint8_t buff[128];

/* Private function prototypes -----------------------------------------------*/
static void fault_err (FRESULT rc);
#define DCMI_DR_ADDRESS     0x50050028
#define FSMC_LCD_ADDRESS    0x60100000


RCC_ClocksTypeDef RCC_Clocks;
EXTI_InitTypeDef   EXTI_InitStructure;
uint8_t capture_Flag = DISABLE;
int ms_Ticks = 0;
int Display = 0; 

// Private function prototypes -----------------------------------------------
uint8_t DCMI_OV9655Config(void);
void DCMI_Config(void);
void I2C1_Config(void);
void LIS302DL_Reset(void);

void PushButton_Init(void);
int button(void);

void Delay( int d_Ticks);

void Display_Message_format();

//------------------------------- BlueTooth ----------------------------------------
char BT_CMD  = 1;
char BT_AOK  = 2;
char BT_CONN = 3;
char BT_END  = 4;
char BT_Done = 4;
char BT_PAIR = 5;
bool BlueT_Connected = 0;

unsigned short tmp;

char CMD_mode = 1;
char BT_state = 0;
char response_rcvd = 0;
char responseID = 0, response = 0;

char Data_Receaved[101];
char Data_Display[201];
int DReceaved_count = 0;
int Data_Ready= 0;

void BlueTooth_USART4_Init(void);
void BlueTooth_String_send(char str[]);
void BlueTooth_Interrupt_Response();
char BlueTooth_Get_Response();
void BlueTooth_Protocol();

//--------------------------------------------------------------------------------




int main(void){
  STM_EVAL_LEDInit( LED3);      // LED Inits
  STM_EVAL_LEDInit( LED4);
  STM_EVAL_LEDInit( LED5);
  STM_EVAL_LEDInit( LED6);
  
  // SysTick end of count event each 1ms
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  
  LIS302DL_Reset();
  PushButton_Init();
 
  // Initialize the LCD 
  STM32f4_Discovery_LCD_Init();
  LCD_Clear(LCD_COLOR_WHITE);
  LCD_SetTextColor(LCD_COLOR_BLUE);
  DCMI_Control_IO_Init();

  
  LCD_DisplayStringLine(LINE(2), "   Camera Init..");
		   
  // OV9655 Camera Module configuration
  if (DCMI_OV9655Config() == 0x00){
    LCD_DisplayStringLine(LINE(2), "  Ready         ");
    LCD_SetDisplayWindow(0, 0, 320, 240);
    LCD_WriteRAM_Prepare();

    // Start Image capture and Display on the LCD 
    DMA_Cmd(DMA2_Stream1, ENABLE);      // Enable DMA
    DCMI_Cmd(ENABLE);                   // Enable DCMI 
    DCMI_CaptureCmd(ENABLE);            // Capture Image   
    init_picture_count();               // Picture Count
    Delay(1000);
    DCMI_CaptureCmd(DISABLE);
    Delay(100);
    LCD_Clear(LCD_COLOR_WHITE);
    
    //Blue tooth Initilize
    LCD_DisplayStringLine(LINE(2), "  Bluetooth Paring ");
    BlueTooth_USART4_Init();
    BlueTooth_Protocol(); /// blue tooth connections
    Delay(500);
    
    LCD_Clear(LCD_COLOR_WHITE);
    LCD_SetTextColor(LCD_COLOR_RED);
    LCD_DisplayStringLine(LINE(2), "     GyroAccel  ");
    LCD_DisplayStringLine(LINE(4), "     Calibration ");
    LCD_DisplayStringLine(LINE(6), "   - Do Not Move - ");
    LCD_DisplayStringLine(LINE(8), "     Board 1 ");
    BlueTooth_USART4_Init();
    while( Data_Receaved[0] != 'G');
    while( Data_Receaved[0] == 'G' || Data_Receaved[1] == 'y');
    LCD_Clear(LCD_COLOR_WHITE);
    LCD_SetTextColor(LCD_COLOR_BLACK);
    
   // USART_SendData(UART4, 'X'); //stop bluetooth
  //  USART_Cmd(UART4, DISABLE);
    
    // mount the filesystem
    STM32f4_Discovery_Debug_Init();
    if (f_mount(0, &filesystem) != FR_OK) {
      printf("could not open filesystem \n\r");
    }
    Delay(10);

    ret = f_open(&file, "HELLO.TXT", FA_WRITE | FA_CREATE_ALWAYS);
    if (ret) {
      printf("Create a new file error\n\r");
      fault_err(ret);
    } 
    else {
      printf("Write a text data. (hello.txt)\n\r");
    }
    USART_Cmd(UART4, ENABLE);
    Data_Ready = 0;
    while (1){
    
      if(Display >= 2){
        Display = 0;
      }
 
      if ( Data_Receaved[100] == '1' && Display == 0) {
        Display = 1;
        USART_Cmd(UART4, DISABLE);
        DCMI_OV9655Config();
        LCD_SetDisplayWindow(0, 0, 320, 240);
        LCD_WriteRAM_Prepare();
        // Start Image capture and Display on the LCD 
        DMA_Cmd(DMA2_Stream1, ENABLE);      // Enable DMA
        DCMI_Cmd(ENABLE);                   // Enable DCMI 
        DCMI_CaptureCmd(ENABLE);            // Capture Image   
       /// init_picture_count();               // Picture Count
        Delay(100);
        BlueTooth_USART4_Init();
      }
      if(Data_Receaved[100] != '1'  && Display == 1){
        Display = 0;
        DCMI_CaptureCmd(DISABLE);
        Delay(100);
        LCD_Clear(LCD_COLOR_WHITE);
        LCD_SetTextColor(LCD_COLOR_BLACK);
        /*
        if (capture_Flag == ENABLE) {         // key take a photo
        DCMI_CaptureCmd(DISABLE);
        capture_Flag = DISABLE;
        Capture_Image_TO_Bmp();
        LCD_SetDisplayWindow(0, 0, 320, 240);
        LCD_WriteRAM_Prepare();
        DCMI_CaptureCmd(ENABLE);
        capture_Flag = ENABLE; 
        }
        */
        BlueTooth_USART4_Init();
        Delay(600);
      }
      else if (Data_Receaved[100] != '1'  && Display == 0){
        if(Data_Ready){
          Display_Message_format();
          LCD_SetFont(&Font16x24); // Font16x24 Font12x12  Font8x12 
          LCD_DisplayStringLine(LINE(8), &Data_Display[0] ); //Clock
          LCD_SetFont(&Font12x12);
          LCD_DisplayStringLine(LINE(1), &Data_Display[19] ); // GPS Position
          LCD_SetFont(&Font16x24);
          LCD_DisplayStringLine(LINE(6), &Data_Display[46] ); // Speed 
          LCD_SetFont(&Font12x12);
          LCD_DisplayStringLine(LINE(2), &Data_Display[64] ); // Compass
          LCD_DisplayStringLine(LINE(4), "           Accel    " );
          LCD_DisplayStringLine(LINE(5), &Data_Display[86] ); // Acceleration
          LCD_DisplayStringLine(LINE(6), "           Gyro    " );
          LCD_DisplayStringLine(LINE(7), &Data_Display[115] ); // Gyroscope
          LCD_SetFont(&Font8x12);
          LCD_DisplayStringLine(LINE(8), &Data_Display[144] ); // Temperature
          LCD_DisplayStringLine(LINE(10), &Data_Display[172] ); //Distance
          Data_Ready =0;
         // USART_SendData(UART4, 'X');
         // USART_Cmd(UART4, DISABLE);
          ret = f_write(&file,  &Data_Display[0], 203, &bw);
          if (ret) {
            printf("Write a text data to file error\n\r");
          } 
          else {
            printf("%u bytes written\n\r", bw);
          }
          Delay(50);
          if(Data_Receaved[100] == 'S'){
            printf("Close the file\n\r");
            ret = f_close(&file);
            if (ret) {
              printf("Close the hello.txt file error\n\r");			
            }
            LCD_Clear(LCD_COLOR_WHITE);
            LCD_SetTextColor(LCD_COLOR_RED);
            LCD_SetFont(&Font16x24); 
            LCD_DisplayStringLine(LINE(2), " Saving Data..");
            LCD_DisplayStringLine(LINE(4), " To SD card  ");
            Delay(10000);
            LCD_Clear(LCD_COLOR_WHITE);
          }
         // USART_Cmd(UART4, ENABLE);
         // USART_SendData(UART4, 'O');
        }
      }
    }  
  } 
  else {
    LCD_SetTextColor(LCD_COLOR_RED);
    LCD_DisplayStringLine(LINE(2), "Camera Init.. fails");    
    LCD_DisplayStringLine(LINE(4), "Check the Camera HW ");    
    LCD_DisplayStringLine(LINE(5), "  and try again ");
    while (1);      
  }
}
//--------------------------------------------------------------------------------------------






//------------------------------- Bluetooth Slave --------------------------------------------------------
void Display_Message_format(){
  // Format message so it displays in a neat way
  Data_Display[0] = ' '; // Time Display.
  Data_Display[1] = ' ';
  Data_Display[2] = ' ';
  Data_Display[3] = ' ';
  Data_Display[4] = ' ';

  for(int i = 0; i < 11; i++){
   Data_Display[5+i] = Data_Receaved[i];
  }
  Data_Display[16] =  ' ';
  Data_Display[17] =  ' ';
  Data_Display[18] =  0;
  
  for(int i = 0; i < 26; i++ ){         // Longitued and Latitude
    Data_Display[19+i]= Data_Receaved[12+i];
  }
  Data_Display[44] = ' ';
  Data_Display[45] = 0;
  
  Data_Display[46] = ' ';       // Speed Display
  Data_Display[47] = ' ';
  Data_Display[48] = ' ';
  Data_Display[49] = ' ';
  Data_Display[50] = ' ';
  Data_Display[51] = ' ';
  Data_Display[52] = ' ';
  Data_Display[53] = ' ';
  
  for (int i = 0; i < 6; i++){
    Data_Display[54+i]= Data_Receaved[38+i];
  }
  Data_Display[62] =  ' ';
  Data_Display[63] =  0;  
  
  Data_Display[64] = ' ';       // Compass
  Data_Display[65] = ' ';
  Data_Display[66] = ' ';
  Data_Display[67] = ' ';
  Data_Display[68] = ' ';
  Data_Display[69] = ' ';
  Data_Display[70] = ' ';
  Data_Display[71] = ' ';
  if( Data_Receaved[44] == 'N'){
    Data_Display[72] =  'N';  
    Data_Display[73] =  'o';  
    Data_Display[74] =  ' ';  
    Data_Display[75] =  'V';  
    Data_Display[76] =  'e';
    Data_Display[77] =  'l';  
    Data_Display[78] =  'o';
    Data_Display[79] =  'c';  
    Data_Display[80] =  'i';  
    Data_Display[81] =  't';
    Data_Display[82] =  'y';
    Data_Display[83] =  ' ';
    Data_Display[84] =  ' ';
    Data_Display[85] =  0;
  }
  else{
    Data_Display[72] = ' ';
    Data_Display[73] = ' ';
    Data_Display[74] = ' ';
    Data_Display[75] = ' ';
    for (int i = 0; i < 5; i++){
      Data_Display[76+i]= Data_Receaved[45+i];
    }
    Data_Display[81] =  ' ';
    Data_Display[82] =  ' ';
    Data_Display[83] =  ' ';
    Data_Display[84] =  ' ';
    Data_Display[85] =  0;
  }
  
  Data_Display[86] =  ' ';  //Acceleration
  Data_Display[87] =  'X';  
  Data_Display[88] =  ':';  
 
  for(int i = 0; i < 6 ; i++ ){         
    Data_Display[89+i]= Data_Receaved[52+i];
  }
  Data_Display[95] =  ' ';  
  Data_Display[96] =  'Y';  
  Data_Display[97] =  ':'; 
  for(int i = 0; i < 6 ; i++){
    Data_Display[98+i]= Data_Receaved[58+i];
  }
  Data_Display[104] =  ' ';  
  Data_Display[105] =  'Z';  
  Data_Display[106] =  ':'; 
  for(int i = 0; i < 5 ; i++){
    Data_Display[107+i]= Data_Receaved[64+i];
  }
  Data_Display[112] = ' ';
  Data_Display[113] = ' ';
  Data_Display[114] = 0;
  
  Data_Display[115] =  ' ';  //Gyroscope
  Data_Display[116] =  'X';  
  Data_Display[117] =  ':';  
 
  for(int i = 0; i < 6 ; i++ ){         
    Data_Display[118+i]= Data_Receaved[72+i];
  }
  Data_Display[124] =  ' ';  
  Data_Display[125] =  'Y';  
  Data_Display[126] =  ':'; 
  for(int i = 0; i < 6 ; i++){
    Data_Display[127+i]= Data_Receaved[77+i];
  }
  Data_Display[133] =  ' ';  
  Data_Display[134] =  'Z';  
  Data_Display[135] =  ':'; 
  for(int i = 0; i < 5 ; i++){
    Data_Display[136+i]= Data_Receaved[82+i];
  }
  Data_Display[141] = ' ';
  Data_Display[142] = ' ';
  Data_Display[143] = 0;
  
  Data_Display[144] = ' '; //Temprature
  Data_Display[145] = ' ';
  Data_Display[146] = ' ';
  Data_Display[147] = ' ';
  Data_Display[148] = ' ';
  Data_Display[149] = ' ';
  Data_Display[150] = ' ';
  Data_Display[151] = ' ';
  Data_Display[152] = ' ';
  Data_Display[153] = ' ';
  Data_Display[154] = ' ';
  Data_Display[155] = ' ';
  Data_Display[156] = 'T';
  Data_Display[157] = 'e';
  Data_Display[158] = 'm';
  Data_Display[159] = 'p';
  Data_Display[160] = ':';
  Data_Display[161] = ' ';
  
  for(int i = 0; i < 4 ; i++){
    Data_Display[162+i]= Data_Receaved[89+i];
  }
  Data_Display[166] = 161;
  Data_Display[167] = 'F';
  Data_Display[168] = ' ';
  Data_Display[169] = ' ';
  Data_Display[170] = ' ';
  Data_Display[171] = 0;
  
  Data_Display[172] = ' ';      // Distance  
  Data_Display[173] = ' ';
  Data_Display[174] = ' ';
  Data_Display[175] = ' '; 
  Data_Display[176] = ' '; 
  Data_Display[177] = ' '; 
  Data_Display[178] = ' '; 
  Data_Display[179] = ' ';
  Data_Display[180] = ' ';
  Data_Display[181] = ' ';
  Data_Display[182] = ' ';
  Data_Display[183] = 'D';
  Data_Display[184] = 'i';
  Data_Display[185] = 's';
  Data_Display[186] = 't';
  Data_Display[187] = 'a';
  Data_Display[188] = 'n';
  Data_Display[189] = 'c';
  Data_Display[190] = 'e';
  Data_Display[191] = ':';
  Data_Display[192] = ' ';
  for(int i = 0; i < 3 ; i++){
    Data_Display[193+i]= Data_Receaved[96+i];
  }
  Data_Display[196] = 'c';
  Data_Display[197] = 'm';
  Data_Display[198] = ' ';
  Data_Display[199] = 0;
  Data_Display[200] = '\n';
}

void BlueTooth_Protocol(){
  //Address 00-06-66-65-ED-0C
  Delay(300);
  BlueTooth_String_send("---");                         // Exit command Mode
  USART_SendData(UART4, 13);  
  Delay(1500);
 
  do {
    BlueTooth_String_send("$$$");                      // Enter command mode
    Delay(1000);
  } while (BlueTooth_Get_Response() != BT_CMD);

  BlueTooth_String_send("R,1"); 
  USART_SendData(UART4, 13);
  Delay(1500);

  do {
    BlueTooth_String_send("$$$");                      // Enter command mode
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_CMD);
  
  do {
    BlueTooth_String_send("SN,Bluetooth-Slave");                      // Name of device
    USART_SendData(UART4, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);
 
  do {
    BlueTooth_String_send("SO,Slave");                      // Extended status string
    USART_SendData(UART4, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  do {
    BlueTooth_String_send("SM,0");                     // Set mode (0 = slave, 1 = master, 2 = trigger, 3 = auto, 4 = DTR, 5 = ANY)
    USART_SendData(UART4, 13);                // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  do {
    BlueTooth_String_send("SA,0");                      // Authentication (1 to enable, 0 to disable)
    USART_SendData(UART4, 13);                 // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);

  do {
    BlueTooth_String_send("SP,mikroe");                      // Security pin code (mikroe)
    USART_SendData(UART4, 13);                  // CR
    Delay(500);
  } while (BlueTooth_Get_Response() != BT_AOK);
  

  BlueTooth_String_send("R,1"); 
  USART_SendData(UART4, 13);
  Delay(1500);
  
  while (BlueTooth_Get_Response() != BT_CONN);          // wait for connection
  BlueT_Connected = 1;
  
}

void BlueTooth_String_send(char str[]){
  int i = 0;
  while(str[i] != '\0'){
    while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
    USART_SendData(UART4, str[i]);
    i++;
  }
  while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
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

void BlueTooth_Interrupt_Response(){
  char tmp;
  tmp = USART_ReceiveData( UART4);                         // Get received byte

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
        if (tmp == 'E')                 // We have 'E', it could be "END"
          BT_state = 31;                // expecting 'N'
        if (tmp == 'P') 
          BT_state = 1;
        break;                          // ...
    }
    case  1: {
        if (tmp == 'M')
          BT_state = 2;
        else if (tmp == 'O')
          BT_state = 22;
        else if (tmp == 'A') 
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
        else if (tmp == 'I') 
          BT_state = 11;
        else
          BT_state = 0;
        break;
    }
    case 11: {
        if (tmp == 'O')
          BT_state = 12;
        else if (tmp == 'R') {
          response = BT_PAIR;           // "Inquiry Done"
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
        if (tmp == 'N')
          BT_state = 32;
        else
          BT_state = 0;
        break;
    }
    case 32: {
        if (tmp == 'D') {
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

void BlueTooth_USART4_Init(void){
  // Configure USART4
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4, ENABLE);
  
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
 
  // GPIO Configuration USART 4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // TX PC-10  RX PC-11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // Connect USART pins to AF  USART 4 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
 
  // USART4 Congiguration 
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure); 
  USART_Cmd(UART4, ENABLE);
  
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_Cmd(UART4, ENABLE);  
}

void UART4_IRQHandler(void){
  if(BlueT_Connected == 0){
    if( USART_GetITStatus(UART4, USART_IT_RXNE)){        
      BlueTooth_Interrupt_Response();
    }
  }
  else if ( BlueT_Connected == 1 && USART_GetITStatus(UART4, USART_IT_RXNE)){
      Data_Receaved[DReceaved_count] = USART_ReceiveData( UART4);
      if(Data_Receaved[DReceaved_count] == '\n'){
        Data_Ready = 1;
        DReceaved_count = 0;
      }
      else
        DReceaved_count++;
  }
}


//-----------------------------------------------------------------------------------------
// Configure (I2C, DCMI and DMA) to interface OV9655
// 0x00 Camera module configured correctly 
// 0xFF Camera module configuration failed
uint8_t DCMI_OV9655Config(void){
  I2C1_Config();        //I2C1 configuration for OV9655

  // Reset and check the presence of the OV9655 camera module 
  if (DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS,0x12, 0x80)){
     return (0xFF);
  }

  // OV9655 Camera size setup
  DCMI_OV9655_QVGASizeSetup();

  // Set the RGB565 mode 
  DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM7, 0x63);
  DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM15, 0x10);

  // Invert the HRef signal
  DCMI_SingleRandomWrite(OV9655_DEVICE_WRITE_ADDRESS, OV9655_COM10, 0x08);
  DCMI_Config();        //Configure DCMI to interface with the OV9655 module
  return (0x00);
}

// Configures the I2C1 used for OV9655 camera module configuration.
void I2C1_Config(void){

  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStruct;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);  // I2C1 clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // GPIOB clock enable
  
  /* Connect I2C1 pins to AF4 ************************************************/
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);  
  
  /* Configure I2C1 GPIOs *****************************************************/  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure I2C1 ***********************************************************/  
  I2C_DeInit(I2C1);             // I2C DeInit
  I2C_Cmd(I2C1, ENABLE);        // Enable the I2C peripheral
 
  // Set the I2C structure parameters 
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0xFE;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStruct.I2C_ClockSpeed = 30000;
  
  // Initialize the I2C peripheral w/ selected parameters
  I2C_Init(I2C1, &I2C_InitStruct);
}

//Configures the DCMI to interface with the OV9655 camera module.
void DCMI_Config(void){
  DCMI_InitTypeDef DCMI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef  DMA_InitStructure;
  
  // Enable DCMI GPIOs clocks 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE | 
                         RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE); 

  // Enable DCMI clock
  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_DCMI, ENABLE);

  /* Connect DCMI pins to AF13 ************************************************/
  // PCLK 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI);
  // D0-D7 
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI);
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI);
  // VSYNC 
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI);
  // HSYNC 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI);
  
  /* DCMI GPIO configuration **************************************************/
  //(D0-PC6, D1-7) 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;  
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // (D2-PE0, D3-PE1, D4-PE4) and (D6-PE5, D7-PE6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 
	                              | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* D5(PB6), VSYNC(PB7) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* PCLK(PA6) HSYNC(PA4)*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* DCMI configuration *******************************************************/ 
  DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_Continuous;
  DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
  DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
  DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
  DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
  DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
  DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b;
  DCMI_Init(&DCMI_InitStructure);

  /* Configures the DMA2 to transfer Data from DCMI to the LCD ****************/
  // Enable DMA2 clock 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  
  
  // DMA2 Stream1 Configuration  
  DMA_DeInit(DMA2_Stream1);
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = DCMI_DR_ADDRESS;	
  DMA_InitStructure.DMA_Memory0BaseAddr = FSMC_LCD_ADDRESS;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);
}

void LIS302DL_Reset(void){
  uint8_t ctrl = 0;
  
  LIS302DL_InitTypeDef  LIS302DL_InitStruct;
  LIS302DL_InterruptConfigTypeDef LIS302DL_InterruptStruct;  
  
  // Set configuration of LIS302DL
  LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
  LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
  LIS302DL_InitStruct.Axes_Enable = LIS302DL_X_ENABLE | LIS302DL_Y_ENABLE | LIS302DL_Z_ENABLE;
  LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
  LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
  LIS302DL_Init(&LIS302DL_InitStruct);
    
  // Set configuration of Internal High Pass Filter of LIS302DL
  LIS302DL_InterruptStruct.Latch_Request = LIS302DL_INTERRUPTREQUEST_LATCHED;
  LIS302DL_InterruptStruct.SingleClick_Axes = LIS302DL_CLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptStruct.DoubleClick_Axes = LIS302DL_DOUBLECLICKINTERRUPT_Z_ENABLE;
  LIS302DL_InterruptConfig(&LIS302DL_InterruptStruct);

  // Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate = 3/100 = 30ms                                                           
  Delay(30); 
  ctrl = 0xC0;          // Configure Click Window
  LIS302DL_Write(&ctrl, LIS302DL_CLICK_CTRL_REG3_ADDR, 1);
}

uint32_t LIS302DL_TIMEOUT_UserCallback(void){
  while (1) ;   //MEMS Accelerometer Timeout error occured
}
  
void PushButton_Init(void){
  
  GPIO_InitTypeDef   GPIO_InitStructure;
 // NVIC_InitTypeDef   NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // Eanable GPIOA clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);// Enable SYSCFG clock
  
  // Configure PA0 pin as input floating 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
/*
  // Connect EXTI Line0 to PA0 pin 
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

  // Configure EXTI Line0 
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI Line0 Interrupt to the lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
*/
}

int button(void){ 
  int on = 0;
  on =  ( GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0 ));
  return on; 
}

/*
void EXTI0_IRQHandler(void){
 
  ButtonPressed  = !ButtonPressed; // ButtonPressed ^ 1;
  EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);      // Clear the EXTI line 0 pending bit       
} 
*/

void Delay( int d_Ticks){               // Delays number of Systicks (happens every 1 ms)                                              
  ms_Ticks = d_Ticks;
  while( ms_Ticks > 0);
}

void SysTick_Handler(void){
  ms_Ticks--;
  if(ms_Ticks  > 100000){
    ms_Ticks = 0;
  }
}



static void fault_err (FRESULT rc){
  const char *str =
                    "OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
                    "INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
                    "INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
                    "LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
  FRESULT i;

  for (i = (FRESULT)0; i != rc && *str; i++) {
    while (*str++) ;
  }
  printf("rc=%u FR_%s\n\r", (UINT)rc, str);
  STM_EVAL_LEDOn(LED6);
  while(1);
}


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line){ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1){}
}
#endif

