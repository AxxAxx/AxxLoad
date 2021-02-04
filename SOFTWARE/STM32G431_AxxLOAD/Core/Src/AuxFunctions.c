/*
 * AuxFunctions.c
 *
 *  Created on: 13 aug. 2020
 *      Author: SEAXJOH
 */


#include "AuxFunctions.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>

unsigned char ADSwrite[6];
#define ADS1115_ADDRESS 0x48
#define MCP4725_ADDRESS 0x60
int16_t voltage[4];
char buffer[10];
char _out[10];
extern bool reportStatus;
extern bool txDone;
extern uint32_t zeroTimeValue;
uint16_t mcp4725Voltage = 0;
float outputVoltageCompensationConstant = 0.15705657076;
float temperatureC;

// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3892
// the value of the 'other' resistor
#define SERIESRESISTOR 10000  


struct statusValues {
	uint32_t  timestamp;
	uint16_t  HEATSINK_Temp;
	uint16_t  setCurrent;
	uint16_t   setPower;
	uint16_t   setResistance;
	uint16_t   measuredCurrent;
	uint16_t   measuredVoltage;
	uint16_t   measuredEquivalentResistance;
	uint16_t   MOSFET1_Temp;
	uint16_t   MOSFET2_Temp;
	uint16_t   PCB_Temp;
	uint32_t   measuredPower;
	float   amperehours;
	float   watthours;
	uint16_t fanSpeed;
};

void debugPrintln(UART_HandleTypeDef *huart, char _out[]){
    txDone = false;
	HAL_UART_Transmit_IT(huart, (uint8_t *) _out, strlen(_out));
	while(!txDone);
	char newline[2] = "\r\n";
    txDone = false;
	HAL_UART_Transmit_IT(huart, (uint8_t *) newline, 2);
	while(!txDone);

}

uint16_t adc2Temperature(uint16_t adcValue, uint16_t adcResolution){

	temperatureC = (float)adcResolution / adcValue - 1;
	temperatureC = SERIESRESISTOR / temperatureC;

  temperatureC = temperatureC / THERMISTORNOMINAL;     // (R/Ro)
  temperatureC = log(temperatureC);                  // ln(R/Ro)
  temperatureC /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  temperatureC += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  temperatureC = 1.0 / temperatureC;                 // Invert
  temperatureC -= 273.15;                         // convert to C
  temperatureC *= 10;
  return temperatureC;
	}


void debugPrint(UART_HandleTypeDef *huart, char _out[]){
    txDone = false;
	HAL_UART_Transmit_IT(huart, (uint8_t *) _out, strlen(_out));
	while(!txDone);
}

void printHELP(UART_HandleTypeDef *huart, struct statusValues statusValues_1){
	  debugPrintln(huart, "|------------------------------------------|");
	  debugPrintln(huart, "|      Axel Johansson's Electronic load    |");
	  debugPrintln(huart, "|      Version 2.2 2020                    |");
	  debugPrintln(huart, "|---------- Availible Commands ------------|");
	  debugPrintln(huart, "      cc <mA> - Constant current");
	  debugPrintln(huart, "      cp <W> - Constant power(To be implemented..)");
	  debugPrintln(huart, "    cr <mOhm> - Constant resistance(To be implemented..)");
	  debugPrintln(huart, " pm <ms> <mA> - PulseMode, Set PULSE length and amplitude");
	  debugPrintln(huart, "     fs <%/A> - Fanspeed, (0-100% OR \"A\" for Automatic) - Default: A");
	  debugPrintln(huart, "      mv <mV> - Set MIN voltage - Default: 0 mV");
	  debugPrintln(huart, "     log <ms> - Interval of printing status");
	  debugPrintln(huart, "         stop - Turn current off");
	  debugPrintln(huart, "        reset - Reset charge/energy counters");
	  debugPrintln(huart, "       status - Print status");
	  debugPrintln(huart, "         help - Show this help");
	  debugPrintln(huart, " ");
	  debugPrintln(huart, "Status is printed out as:");
	  debugPrintln(huart, "Timestamp [ms]");
	  debugPrintln(huart, "Temperature - Heatsink [deg C]");
	  debugPrintln(huart, "Temperature - MosFET1 [deg C]");
	  debugPrintln(huart, "Temperature - MosFET2 [deg C]");
	  debugPrintln(huart, "Temperature - PCB [deg C]");
	  debugPrintln(huart, "Set current [mA]");
	  debugPrintln(huart, "Measured current [mA]");
	  debugPrintln(huart, "Measured Voltage [mV]");
	  debugPrintln(huart, "Measured Power [mW]");
	  debugPrintln(huart, "Measured Equivalent Resistance [OHM]");
	  debugPrintln(huart, "Discharged Amperehours [mAh]");
	  debugPrintln(huart, "Discharged Watthours [mWh]");
	  debugPrintln(huart, "Fan Speed [%]");


	  printStatus(statusValues_1, huart);
	  debugPrintln(huart, "|------------------------------------------|");
	  debugPrintln(huart, " ");
}

void BEEP(TIM_HandleTypeDef *htim){

	__HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, 5); //update pwm value
	HAL_Delay(20);
	__HAL_TIM_SetCompare(htim, TIM_CHANNEL_3, 0); //update pwm value


}
void MCP4725_write(I2C_HandleTypeDef *hi2c, uint16_t outputVoltage){

	mcp4725Voltage = outputVoltage*outputVoltageCompensationConstant;
	ADSwrite[0] = 0b01000000;
	ADSwrite[1] = mcp4725Voltage >> 4;
	ADSwrite[2] = (mcp4725Voltage & 15) << 4;

	HAL_I2C_Master_Transmit(hi2c, MCP4725_ADDRESS << 1, ADSwrite, 3, 100);
    HAL_Delay(10);

}


uint16_t stringToInt(char *string){
	uint16_t command_value = 0;
	uint8_t x=0;
	while( string[x] >= '0' && string[x] <= '9' && x < 7 ){	//check how many numbers after '$' and get message ID from that
		command_value *= 10;
		command_value += string[x] - '0';
		x++;
	}
	return command_value;
}


void setFanSpeed(UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim, struct statusValues statusValues_1){
	debugPrint(huart, "Setting Fan speed to: ");
	sprintf(buffer, "%hu", statusValues_1.fanSpeed);
	debugPrint(huart, buffer);
	debugPrintln(huart, "%");
	__HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, statusValues_1.fanSpeed*10); //update pwm value //TIM2->CCR2 = pwm;
}

void autoFanSpeed(struct statusValues statusValues_1, TIM_HandleTypeDef *htim){

	__HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, statusValues_1.fanSpeed*10);

}


void printStatus(struct statusValues statusValues_1, UART_HandleTypeDef *huart){
	debugPrint(huart, ">>");
	char buffer[15];
  		memset(&buffer, '\0', sizeof(buffer));

		//Timestamp[ms]
	  	//sprintf(buffer, "%hu", statusValues_1.timestamp);
  		sprintf(buffer, "%8.2f", statusValues_1.timestamp/1000.0);
	  	//gcvt((statusValues_1.timestamp/1000.0), 6, buffer);
		debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Temperature - Heatsink[deg C]
	  	//gcvt((statusValues_1.temperature/10.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.HEATSINK_Temp/10.0);
	  	debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Temperature - MosFET1[deg C]
	  	//gcvt((statusValues_1.temperature/10.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.MOSFET1_Temp/10.0);
	  	debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Temperature - MosFET2[deg C]
	  	//gcvt((statusValues_1.temperature/10.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.MOSFET2_Temp/10.0);
	  	debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Temperature - PCB[deg C]
	  	//gcvt((statusValues_1.temperature/10.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.PCB_Temp/10.0);
	  	debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Set current[mA]
	  	//sprintf(buffer, "%hu", statusValues_1.setCurrent);
	  	//gcvt((statusValues_1.setCurrent/1000.0), 6, buffer);
  		sprintf(buffer, "%7.2f", statusValues_1.setCurrent/1000.0);
	  	debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Measured current[mA]
	  	//sprintf(buffer, "%hu", statusValues_1.measuredCurrent);
	  	//gcvt((statusValues_1.measuredCurrent/1000.0), 6, buffer);
  		sprintf(buffer, "%7.2f", statusValues_1.measuredCurrent/1000.0);
  		debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");


		//Measured Voltage[mV]
	  	//sprintf(buffer, "%hu", statusValues_1.measuredVoltage);
	  	//gcvt((statusValues_1.measuredVoltage/1000.0), 6, buffer);
  		sprintf(buffer, "%4.2f", statusValues_1.measuredVoltage/1000.0);
	  	debugPrint(huart, buffer);
	  	memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Measured Power[mW]
		//sprintf(buffer, "%hu", statusValues_1.measuredPower);
	  	//gcvt((statusValues_1.measuredPower/1.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.measuredPower/1000.0);
  		debugPrint(huart, buffer);
		memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");




		//Measured Equivalent Resistance[mOHM]
		//sprintf(buffer, "%hu", statusValues_1.measuredPower);
	  	//gcvt((statusValues_1.measuredPower/1.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.measuredEquivalentResistance/1000.0);
  		debugPrint(huart, buffer);
		memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");







		//Amperehours[mAh]
		//sprintf(buffer, "%hu", statusValues_1.amperehours);
	  	//gcvt((statusValues_1.amperehours/1000000000.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.amperehours);
  		debugPrint(huart, buffer);
		memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");

		//Watthours[mWh]
		//sprintf(buffer, "%hu", statusValues_1.watthours);
	  	//gcvt((statusValues_1.watthours/1000000000.0), 6, buffer);
  		sprintf(buffer, "%5.2f", statusValues_1.watthours);
  		debugPrint(huart, buffer);
		memset(&buffer, '\0', sizeof(buffer));
		debugPrint(huart, "   ");


  		sprintf(buffer, "%5.0f", statusValues_1.fanSpeed*1.0);
  		debugPrint(huart, buffer);
		memset(&buffer, '\0', sizeof(buffer));
		debugPrintln(huart, "   ");




}
