/** 
* @brief Main source code for steering wheel
* @details This file is the main source code for the raspberry pi pico mounted on the steering wheel of the t24 car from the LART formula student team   
* @author Bruno Vicente
* @version 0.0.1
**/


#include <Arduino.h>
#include <mcp_can.h>
#include <hardware/uart.h>
#include "stdio.h"
//#include "pico/stdlib.h"
//#include <ardubson.h>
#include <BSONPP.h>

#define debug_led1 12
#define debug_led2 13
#define debug_led3 14



/*          --------          */

#define __LART_T24__
#define PB1_TX_13 9
#define PB0_RX_12 8




#define BSON_RPM "rpm"
#define BSON_VEHICLESPEED "vel"
#define BSON_ENGINETEMPERATURE "eng_t"
#define BSON_BATTERYVOLTAGE "bat_v"
#define SIZE_OF_BSON 40


#ifdef __LART_T14__
  #define BSON_GEARSHIFT "gear"
  #define BSON_OILPRESSURE "oil_p"
  #define BSON_OILTEMPERATURE "oil_t"
  #define BSON_DATALOGGERSTATUS "dtl_s"
  #define BSON_AFR "af_r"
  #define BSON_TCSLIP "tc_s"
  #define BSON_TCLAUNCH "tc_l"
  #undef SIZE_OF_BSON
  #define SIZE_OF_BSON 128
#endif

#ifdef __LART_T24__


  #define BSON_SOC "soc"
  #define BSON_BATTERYTEMPERATURE "bat_t"
  #define BSON_INVERTERTEMPERATURE "inv_t"
  #define BSON_POWER "pow"
  #define BSON_LAPCOUNT "lap_c"
  #define BSON_LAPTIME "lap_t"
 
  #undef SIZE_OF_BSON
 
  #define SIZE_OF_BSON 128

  
#endif 



/*          --------          */


UART Serial_uart(8,9,NC,NC);



/*  ------      UART module id's for calibration    ------  */
#define dynamics 0b00000000
#define IMU      0b00000001


/*  ------      UART variable id's     ------  */
#define speed_id  255
#define HV_batery_level_id 254
#define LV_batery_level_id 253
#define Current_Menu 252



void initial_modules_calibration();
int serial_transmit(uint8_t value);


uint8_t module_id[] = {dynamics,IMU};
void ask_calibration_values(uint8_t address,long unsigned int CAN_address_rec,long unsigned int CAN_address);

int num_modulos_ini = 2;
MCP_CAN CAN0(10); 
volatile int menu = 0;




/*  ------      Information to display      ------  */

uint8_t CAN_conected = 0;
uint8_t speed = 0;
uint8_t HV_batery_level = 0;
uint8_t LV_batery_level = 0;



void setup() 
{
  Serial_uart.begin(115200);
  pinMode(debug_led1,OUTPUT);
  pinMode(debug_led2,OUTPUT);
  pinMode(debug_led3,OUTPUT);
  Serial.begin(115200);
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK)
  {
    digitalWrite(LED2,HIGH);
  }
  digitalWrite(debug_led1,LOW);
  while(Serial.available())
  {

  }
  digitalWrite(debug_led1,HIGH);
  initial_modules_calibration();
}

void loop() 
{
  

  Serial_uart.println("Hello joao!");

}


void initial_modules_calibration()
{
  for(int i = 0; i<num_modulos_ini;i++)
  {
    uint8_t status = 0;
    do
    {
      status = serial_transmit(module_id[i]);
    } while (status == 0);
    
  }
}

int serial_transmit(uint8_t value)
{
  Serial.print(value);
  if(Serial.read() == value)
  {
    return 1;
  }
  else
  {
    return 0;
  }
  
}

void ask_calibration_values(uint8_t address,long unsigned int CAN_address_rec,long unsigned int CAN_address)
{
  uint8_t received_value_CAN,len = 0;
  unsigned char rxBuf[8],txBuff[8] = {0,0,0,0,0,0,0,0};
  long unsigned int CAN_address_filt;
  /*
  
    Some CAN code

  */
  CAN0.sendMsgBuf(CAN_address,8,txBuff);
  do
  {
    while(CAN0.checkReceive() != CAN_MSGAVAIL);
    CAN0.readMsgBuf(&CAN_address_filt, &len, rxBuf);
  } while (CAN_address_rec != CAN_address_filt);
  
  
  received_value_CAN = rxBuf[0];


  while(serial_transmit(address) == 0);
  while(serial_transmit(received_value_CAN) == 0);


}
