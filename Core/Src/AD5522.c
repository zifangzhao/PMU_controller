#include "AD5522.h"
#include "main.h"
int AD5522_init(handle_AD5522* h, SPI_HandleTypeDef* hspi)
{
	h->hspi = hspi;
	uint32_t cmd=0;
	cmd|=PMU_SYSREG_CL0|PMU_SYSREG_CL1|PMU_SYSREG_CL2|PMU_SYSREG_CL3;
	cmd|=PMU_SYSREG_GAIN0; //Sel  Output Gain to 10 (0-4.5V output)
	AD5522_SetSystemControl(h,cmd);
	
	cmd=0;
	AD5522_SetPMU(h,PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3,cmd);
	
}
int AD5522_WriteReg(handle_AD5522* h,__IO uint32_t cmd)
{
	//cmd=cmd<<3; //adjust 29bit to 32bit protocol
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,0);
	PMU_SPI_CS_DELAY
	int resp = HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd,1,1000);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
	return resp;

}

int AD5522_ReadReg(handle_AD5522* h,__IO uint32_t cmd, __IO uint32_t* rst)
{
	
	uint32_t cmd_nop=0x00FFFFFF;
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,0);
	PMU_SPI_CS_DELAY
	HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd,1,1000);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,0);
	PMU_SPI_CS_DELAY
	int resp=HAL_SPI_TransmitReceive(h->hspi,(uint8_t*)&cmd_nop,(uint8_t*)rst,1,1000);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
	//*rst=*rst>>8; //shift 29bit SPI to 24 bit readout
	*rst=*rst>>8; //shift 32bit SPI to 24 bit readout
	return resp;
}

int AD5522_SetSystemControl(handle_AD5522* h,__IO uint32_t cmd)
{
	h->reg_sys = PMU_MODE_SYSREG|cmd; 
	AD5522_WriteReg(h,cmd);
}

int AD5522_SetPMU(handle_AD5522* h,__IO uint32_t channel,__IO uint32_t cmd)
{
	if(channel!=0)
	{
		for(int i=0;i<4;i++)
		{
			if((channel&(PMU_CH_0<<i))!=0)
			{
				h->reg_pmu[i] = cmd; 
			}
		}
		AD5522_WriteReg(h,channel|cmd);
		return 0;
	}
	return -1;
}

int AD5522_SetClamp(handle_AD5522* h,__IO uint32_t channel,__IO uint16_t I_low,__IO uint16_t I_high,__IO uint16_t V_low,__IO uint16_t V_high)
{
	//Check input integraty
	if((V_low>=V_high)|(I_low>=I_high))
	{
		return -1;
	}
	//Configure SYS
	uint32_t cmd=0;
	//Enable clamp system
	if((channel&PMU_CH_0)!=0)
	{
		cmd&=~PMU_SYSREG_CL0;
	}
	if((channel&PMU_CH_1)!=0)
	{
		cmd&=~PMU_SYSREG_CL1;
	}
	if((channel&PMU_CH_2)!=0)
	{
		cmd&=~PMU_SYSREG_CL2;
	}
	if((channel&PMU_CH_3)!=0)
	{
		cmd&=~PMU_SYSREG_CL3;
	}
	cmd|=PMU_SYSREG_GAIN0; //Sel  Output Gain to 10 (0-4.5V output)
	AD5522_SetSystemControl(h,cmd);

	//configure PMU
	cmd=0;
	cmd|=PMU_PMUREG_HZI|PMU_PMUREG_MEAS_V;
	AD5522_SetPMU(h,channel,cmd);
	
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			h->reg_DAC_CLL_V[i][AD5522_DAC_REG_X1] = V_low; 
			h->reg_DAC_CLH_V[i][AD5522_DAC_REG_X1] = V_high; 
			h->reg_DAC_CLL_I[i][AD5522_DAC_REG_X1] = I_low; 
			h->reg_DAC_CLH_I[i][AD5522_DAC_REG_X1] = I_high; 
		}
	}
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_V_X1|V_low);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_V_X1|V_low);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_I_X1|I_low);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_I_X1|I_high);
	return 0;
}
int AD5522_Calibrate(handle_AD5522* h);
int AD5522_Vmeasure(handle_AD5522* h,__IO uint32_t channel,__IO uint32_t* volt);
int AD5522_StartFV_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	
}
int AD5522_StartFI_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{

}

int AD5522_StartHiZMV(handle_AD5522* h,__IO uint32_t channel)
{
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	uint32_t cmd=0;
	cmd|=PMU_PMUREG_HZI|PMU_PMUREG_MEAS_V;
	AD5522_SetPMU(h,channel,cmd);
}

int AD5522_StartFVMI(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	I_range&=0x07;
	h->i_range=I_range;
	cmd|=I_range<<15;
	AD5522_SetSystemControl(h,cmd);
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	cmd=0;
	cmd|=PMU_PMUREG_FVCI|PMU_PMUREG_MEAS_I|PMU_PMUREG_FIN|PMU_PMUREG_CL;
	AD5522_SetPMU(h,channel,cmd);
	
}
int AD5522_StartFIMV(handle_AD5522* h,uint32_t channel,uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	I_range&=0x07;
	h->i_range=I_range;
	cmd|=I_range<<15;
	
	//Configure DAC
	AD5522_SetOutputCurrent(h,channel,32768);
	//configure PMU
	cmd=0;
	cmd|=PMU_PMUREG_FICV|PMU_PMUREG_MEAS_V|PMU_PMUREG_FIN|PMU_PMUREG_CL;
	AD5522_SetPMU(h,PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3,cmd);
}

int AD5522_StartFVMV(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	I_range&=0x07;
	h->i_range=I_range;
	cmd|=I_range<<15;
	AD5522_SetSystemControl(h,cmd);
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	cmd=0;
	cmd|=PMU_PMUREG_FVCI|PMU_PMUREG_MEAS_V|PMU_PMUREG_FIN|PMU_PMUREG_CL;
	AD5522_SetPMU(h,channel,cmd);
	
}
int AD5522_StartFIMI(handle_AD5522* h,uint32_t channel,uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	cmd|=PMU_SYSREG_CL0|PMU_SYSREG_CL1|PMU_SYSREG_CL2|PMU_SYSREG_CL3;
	cmd|=PMU_SYSREG_GAIN0; //Sel  Output Gain to 10 (0-4.5V output)
	I_range&=0x07;
	h->i_range=I_range;
	cmd|=I_range<<15;
	
	//Configure DAC
	AD5522_SetOutputCurrent(h,channel,32768);
	//configure PMU
	cmd=0;
	cmd|=PMU_PMUREG_FICV|PMU_PMUREG_MEAS_I|PMU_PMUREG_FIN|PMU_PMUREG_CL;
	AD5522_SetPMU(h,PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3,cmd);
}
// X1 = (M+1)/2^n * X1 + C - 2^n-1
int AD5522_SetOutputVoltage(handle_AD5522* h,uint32_t channel,uint16_t voltage)
{
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			h->reg_DAC_FIN_V[i][AD5522_DAC_REG_X1] = voltage;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		}
	}
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_X1|voltage);
}
int AD5522_SetOutputCurrent(handle_AD5522* h,uint32_t channel,uint16_t current)
{
	uint32_t reg_base=0x08; //base for current 5uA current DAC
	reg_base=reg_base<<h->i_range; //get base addr for I dac of the selected I range
	reg_base|=PMU_MODE_DATAREG;
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			h->reg_DAC_FIN_I[i][h->i_range][AD5522_DAC_REG_X1] = current;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		}
	}
	AD5522_WriteReg(h,channel|reg_base|current);
}
