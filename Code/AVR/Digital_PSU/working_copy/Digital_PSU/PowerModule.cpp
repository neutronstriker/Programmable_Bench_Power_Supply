/* 
* PowerModule.cpp
*
* Created: 8/10/2019 3:52:27 PM
* Author: neutron
*/


#include "PowerModule.h"

/*
// default constructor
PowerModule::PowerModule()
{
} //PowerModule

// default destructor
PowerModule::~PowerModule()
{
} //~PowerModule
*/

void tcaSetP0_mode_output()
{
	uint8_t data = 0;
	i2c_write(0x20,0x06,&data,1);
}
void tcaWriteP0(uint8_t data)
{
	i2c_write(0x20,0x02,&data,1);
}

void mcp4725_dac_write(uint16_t val){
	uint8_t dac_reg_upper_4_bits_as_addr =  (val>>8);  //upper 4 bits need to be sent first immediately after device address
	uint8_t dac_reg_lower_8_bits = val; //lower 8 bits sent in the third byte following upper 4 bits in fast mode write

	i2c_write(VOLTAGE_TRIMMER_DAC_MCP4725_I2C_ADDR,dac_reg_upper_4_bits_as_addr,&dac_reg_lower_8_bits,1);
}


uint16_t setVoltage_mcp4725_inverse_trf(uint16_t volts_mv){
	//y = 1.0168x - 22.3961
	return (uint16_t)(1.0168*(float)volts_mv - 22.3961);
}

void setVoltage_mcp4725(float vTrimRequired){
			#define DAC_LSB (1250.0f/4096.0f)
			
			//float dac_code_required = round(vTrimRequired/dac_lsb);
			uint16_t dac_code_required = round(vTrimRequired/DAC_LSB);
			
			mcp4725_dac_write((uint16_t)dac_code_required);
			
			
			#ifdef PRINT_DEBUG_MSG
			Serial.println();
			Serial.print(F_STR("VtrimRequired is :"));
			Serial.println(vTrimRequired);
			
			Serial.print(F_STR("DAC code is :"));
			Serial.println(dac_code_required);
			
			
			uint16_t achievedVoltage = DAC_LSB*dac_code_required;
			
			Serial.print(F_STR("Achieved Vtrim_voltage :"));
			Serial.println(achievedVoltage);
			
			Serial.print(F_STR("Achieved DCDC_Voltage :"));
			uint16_t achievedDCDCVoltage = (333750UL-(250UL*achievedVoltage))/17UL;
			Serial.println(achievedDCDCVoltage);
			Serial.println();
			#endif
			
}

uint16_t setVoltage_tpl0102_inverse_trf_2nd_order(uint16_t volts_mv){
	return (uint16_t)(1.0001*(float)volts_mv - 5.0375); //based on transfer function generated using V-shmoo_after_Trf (generated after 1st order results)
}

uint16_t setVoltage_tpl0102_inverse_trf(uint16_t volts_mv){
	/*if (volts_mv == 1700)
		return 1700;	//from analysis found that 1700 direct set value has better correlation than from reverse transfer function
	else*/
		return (uint16_t)(1.0132*(float)volts_mv - 11.37);
	//return (uint16_t)(1.0132*(float)setVoltage_tpl0102_inverse_trf_2nd_order(volts_mv) - 11.37); //based on transfer function generated using V-shmoo
	//y = 1.007002x + 21.244679
	//return (uint16_t)(1.007002*(float)volts_mv + 21.244679);
}

void setVoltage_tpl0102(float vTrimRequired){
		uint8_t pot[2];
		//pot[0] = ((float)(128*voltage_mV))/625; //first pot is used for scaling voltage to whatever value we can get; Vwl = (Vh-Vl)*(D/256) => Vwl = 1.25*(D/256) => D = (1024*Vwl)/5
		/*
		pot[0] = (256UL*voltage_mV)/625UL;
		float voltageOutPot0Wiper_mV = ((float)(625UL*pot[0]))/128UL;
		pot[1] = (256UL*voltage_mV)/voltageOutPot0Wiper_mV; //pot1 is used for adjusting the fraction error from pot 0
		*/
		
		
		uint16_t pot0Val = (vTrimRequired*256UL)/625UL;
		if (pot0Val >= 256)
		{
			pot0Val=255;
		}
		if (pot0Val == 0)
		{
			pot0Val = 1;
		}
		pot[0] = pot0Val;
		float voltageOutPot0Wiper_mV = ((float)(625UL*pot[0]))/128UL;
		pot[1] = (vTrimRequired*256UL)/voltageOutPot0Wiper_mV;
		
		//pot[1] = (256UL*voltage_mV)/((625UL*pot[0])/128UL);
		//pot[1] = 255;

		//pot[0] = (65025UL*voltage_mV)/(1250UL*127UL);	
		//pot[1] = (65025UL*voltage_mV)/(1250UL*pot[0]);
	//0.00068V is the step size required to increment or decrement voltage by 0.01V in the DCDC
	
	//this implementation of DAC is not truly 16bit as a result we loose 16bit effect after the MSB DAC has a value more than 1
	//we could actually implement a Voltage summing circuit that could how ever do that.
	
	//1. Now what is possible is that we scale the input voltage using POTA to get required voltage assuming that POTB is at its center
	//2. Then we adjust POTB to correct the fractional error by tuning it to whatever possible extent.
	
	//now there are multiple problems once the POTA taper is around 255, we are completely dependent on POTB to provide the scaling, so it essentially becomes 8bit
	//i.e. whenever the POTA moves higher and higher towards its H pin we are losing the resolution that we can achieve.
	
	//apart from that the DCDC buck 150W is a low freq 160KHz around BUCK and is not synchronous as well, so its response time is very high and when input voltage is high
	//the lower output voltage settings do not work very well. As it does not have the PWM resolution to maintain such low voltages, with high input voltage.
	
	//And since it is not synchronous it takes time to respond for output voltage change since it is unable to discharge the output cap and get lower voltages.
	
	//we can however make a 16bit DAC using two different 8bit DAC, I have made an explanation in the non-inverting summer example in everycircuit.com
	//we will need a non-inverting summing circuit such that the LSB DAC's output will scaled by full-scale value i.e. divided by 256 in this case and summed with the direct
	//output of MSB DAC. To make a proper summing arrangement we will need to buffer the LSB DAC output and SUMMING amplifier input resistance and GAIN calculated properly.
	//generally if the input resistance is same for all sum inputs then it will straight way average of all inputs multiplied by gain. If number of inputs is same as the gain
	//then it will become a simple sum. Or we can triple DAC method where two DAC's will set the lower and higher voltage inputs of the a third DAC and this will actually be used for
	//trimming.
	
	//Now a minimum load resistor drawing at least 10mA has to be put across the output as close as possible to the power module for the output to be stable
	
	//also the problem was that the measurement was shared between Load and Vbus_sense as a result the IR drop effect was also encountered in it.
	//Apart from that the Ground wire was also shared between the load and was very long cable, it should be a short wire going straight between this Vtrim header and 
	//the gnd pin close to the Gnd but away from the Power ground path, generally mostly DCDC converters will have a Analog ground plane for that.
	
	//tested a 8-bit implementation of R-2R DAC, need to build and check 16-bit results. 8-bit DAC results in R-2R_DAC_test.xlsx
	
	#ifdef PRINT_DEBUG_MSG
		//Serial.print("Voltage set is :");
		//Serial.println(current);
	
		Serial.print("VtrimRequired is :");
		Serial.println(vTrimRequired);
	
		Serial.print("Pot 0 :");
		Serial.println(pot[0]);
	
		Serial.print("Pot 1 :");
		Serial.println(pot[1]);
	
		uint16_t achievedVoltage = (1250UL*pot[0]*pot[1])/65536UL;
	
		Serial.print("Achieved Vtrim_voltage :");
		Serial.println(achievedVoltage);
		
		Serial.print("Achieved DCDC_Voltage :");
		uint16_t achievedDCDCVoltage = (333750UL-(250UL*achievedVoltage))/17UL;
		Serial.println(achievedDCDCVoltage);
		
		
	#endif
	
	i2c_write(VOLTAGE_TRIMMER_DPOT_TPL0102_100_I2C_ADDR,0,pot,2);		

}


uint8_t setVoltage(uint16_t voltage_mV)
{
	
	if (!(voltage_mV <= VOLTAGE_SET_RANGE_MAX_mV && voltage_mV >= VOLTAGE_SET_RANGE_MIN_mV))
	{
		Serial.println(F_STR("Error! Voltage Requested Out of Range"));
		return 0;
	}
		//voltage_mV = setVoltage_tpl0102_inverse_trf(voltage_mV);
		
		voltage_mV = setVoltage_mcp4725_inverse_trf(voltage_mV);
		
		//uint16_t vTrimRequired = voltage_mV;
		float vTrimRequired = 1335.0f-((float)(17UL*voltage_mV)/250UL);
		//uint16_t vTrimRequired = 1335-((17UL*voltage_mV)/250UL);
	
	//setVoltage_tpl0102(vTrimRequired);
	setVoltage_mcp4725(vTrimRequired);
	
	return 1;
}


uint8_t setCurrentLimit(uint16_t current_mA){
		uint8_t pot[2];
	
	if (!(current_mA <= CURRENT_LIMIT_RANGE_MAX_mA && current_mA >= CURRENT_LIMIT_RANGE_MIN_mA))
	{
		Serial.println(F_STR("Error! Current Limit Requested Out of Range"));
		return 0;
	}
		
		float iLimDACVout_mV = (float)current_mA/5UL;//INA250A3 800mV/A and then used a by 4 voltage divider before comparator so it is finally 200mV/A or 2mV per 10mA.
		//add a protection to not let the above value go to 0 after calculation
	
		uint16_t pot0Val = (iLimDACVout_mV*128UL)/625UL;
		if (pot0Val >= 256)
		{
			pot0Val=255;
		}
		if (pot0Val == 0)
		{
			pot0Val = 1;
		}
		pot[0] = pot0Val;
		float voltageOutPot0Wiper_mV = ((float)(625UL*pot[0]))/64UL;
		pot[1] = (iLimDACVout_mV*256UL)/voltageOutPot0Wiper_mV;
			
	#ifdef PRINT_DEBUG_MSG
		Serial.print("Current Limit set is :");
		Serial.println(current_mA);
	
		Serial.print("iLimDACVout_mV is :");
		Serial.println(iLimDACVout_mV);
	
		Serial.print("Pot 0 :");
		Serial.println(pot[0]);
	
		Serial.print("Pot 1 :");
		Serial.println(pot[1]);
	
		float achievedVoltage = (float)(2500UL*pot[0]*pot[1])/65536UL;
	
		Serial.print("Achieved iLim_DAC Voltage :");
		Serial.println(achievedVoltage);
		
		Serial.print("Achieved Current Limit :");
		float achievedCurrentLimit = (achievedVoltage*5UL);
		Serial.println(achievedCurrentLimit);
		
		
	#endif
	
	i2c_write(CURRENT_LIMIT_DPOT_TPL0102_100_I2C_ADDR,0,pot,2);		

	return 1;
}
