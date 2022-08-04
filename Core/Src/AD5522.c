#include "AD5522.h"
#include "main.h"
int AD5522_init(handle_AD5522* h, SPI_HandleTypeDef* hspi)
{
	h->hspi = hspi;
	uint32_t cmd=0;
	//cmd|=PMU_SYSREG_CL0|PMU_SYSREG_CL1|PMU_SYSREG_CL2|PMU_SYSREG_CL3;
	cmd|=PMU_SYSREG_GAIN0|PMU_SYSREG_TMPEN; //Sel  Output Gain to 10 (0-4.5V output)
	cmd|=PMU_SYSREG_INT10K;
	AD5522_SetSystemControl(h,cmd);
	
	cmd=0;
	AD5522_SetPMU(h,PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3,cmd);
	return 0;
}
int AD5522_WriteReg(handle_AD5522* h,__IO uint32_t cmd)
{
	//cmd=cmd<<3; //adjust 29bit to 32bit protocol
	//change endian
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,0);
	PMU_SPI_CS_DELAY
#ifdef __PMU_SPI_16B
	uint32_t cmd_this;
	uint16_t *ptr = (uint16_t*)&cmd_this;
	uint16_t *ptr1 = (uint16_t*)&cmd;
	ptr[0] = ptr1[1];
	ptr[1] = ptr1[0];
	int resp = HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd_this,2,1000);
#else
	int resp = HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd,1,1000);
#endif
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
	return resp;

}

int AD5522_ReadReg(handle_AD5522* h,__IO uint32_t cmd, __IO uint32_t* rst)
{
	
	uint32_t cmd_nop=0x00FFFFFF;
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,0);
	PMU_SPI_CS_DELAY
#ifdef __PMU_SPI_16B
	uint32_t cmd_this;
	uint16_t *ptr = (uint16_t*)&cmd_this;
	uint16_t *ptr1 = (uint16_t*)&cmd;
	ptr[0] = ptr1[1];
	ptr[1] = ptr1[0];
	HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd_this,2,1000);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,0);
	PMU_SPI_CS_DELAY
	uint32_t rst_this;
	int resp=HAL_SPI_TransmitReceive(h->hspi,(uint8_t*)&cmd_nop,(uint8_t*)&rst_this,2,1000);
	
	uint16_t *ptro = (uint16_t*)&rst_this;
	uint16_t *ptro1 = (uint16_t*)rst;
	ptro1[0] = ptro[1];
	ptro1[1] = ptro[0];
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
#else
	HAL_SPI_Transmit(h->hspi,(uint8_t*)&cmd,1,1000);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,0);
	PMU_SPI_CS_DELAY
	int resp=HAL_SPI_TransmitReceive(h->hspi,(uint8_t*)&cmd_nop,(uint8_t*)rst,1,1000);
	PMU_SPI_CS_DELAY
	HAL_GPIO_WritePin(PMU_CS_GPIO_Port,PMU_CS_Pin,1);
#endif

	//*rst=*rst>>8; //shift 29bit SPI to 24 bit readout
	*rst=*rst>>8; //shift 32bit SPI to 24 bit readout
	return resp;
}

int AD5522_SetSystemControl(handle_AD5522* h,__IO uint32_t cmd)
{
	h->reg_sys = PMU_MODE_SYSREG|cmd; 
	AD5522_WriteReg(h,cmd);
	return 0;
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

	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_HZI;
			cmd&=~PMU_PMUREG_CL;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
			h->reg_DAC_CLL_V[i][AD5522_DAC_REG_X1] = V_low; 
			h->reg_DAC_CLH_V[i][AD5522_DAC_REG_X1] = V_high; 
			h->reg_DAC_CLL_I[i][AD5522_DAC_REG_X1] = I_low; 
			h->reg_DAC_CLH_I[i][AD5522_DAC_REG_X1] = I_high; 
		}
	}
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_V_X1|V_low);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_V_X1|V_high);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLL_I_X1|I_low);
	AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_CLH_I_X1|I_high);
	
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CL;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}
int AD5522_Calibrate(handle_AD5522* h)
{
	// reset all DAC M/C registers
	for(int i=0;i<4;i++)
	{
		uint32_t channel  = (PMU_CH_0<<i);
		
		h->reg_DAC_offset[i] = 42130;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_OFFSET);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_C] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_C);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_M] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_M);
		h->reg_DAC_FIN_V[i][AD5522_DAC_REG_C] = 0;  //CH DAC_ADDRID DAC_SCALE_ID M_C_X1
		AD5522_WriteReg(h,channel|PMU_DACREG_ADDR_FIN_VOL_C);
		
		#define PMU_DACREG_ADDR_OFFSET (0x00<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_5UA_M (0x08<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_5UA_C (0x08<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_5UA_X1 (0x08<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_20UA_M (0x09<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_20UA_C (0x09<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_20UA_X1 (0x09<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_200UA_M (0x0A<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_200UA_C (0x0A<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_200UA_X1 (0x0A<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_2MA_M (0x0B<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_2MA_C (0x0B<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_2MA_X1 (0x0B<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_EXTC_M (0x0C<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_EXTC_C (0x0C<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_EXTC_X1 (0x0C<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_FIN_VOL_M (0x0D<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_FIN_VOL_C (0x0D<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_FIN_VOL_X1 (0x0D<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLL_I_M (0x14<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLL_I_C (0x14<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLL_I_X1 (0x14<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLL_V_M (0x15<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLL_V_C (0x15<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLL_V_X1 (0x15<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLH_I_M (0x1C<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLH_I_C (0x1C<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLH_I_X1 (0x1C<<16|PMU_MODE_DATAREG)
#define PMU_DACREG_ADDR_CLH_V_M (0x1D<<16|PMU_MODE_GAINREG)
#define PMU_DACREG_ADDR_CLH_V_C (0x1D<<16|PMU_MODE_OFFSETREG)
#define PMU_DACREG_ADDR_CLH_V_X1 (0x1D<<16|PMU_MODE_DATAREG)
	}
	return 0;
}
int AD5522_Vmeasure(handle_AD5522* h,__IO uint32_t channel,__IO uint32_t* volt);
int AD5522_StartFV_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	return 0;
}
int AD5522_StartFI_2CH(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	return 0;
}

int AD5522_StartHiZMV(handle_AD5522* h,__IO uint32_t channel)
{
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_HZI|PMU_PMUREG_MEAS_V;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}

int AD5522_StartFVMI(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	I_range&=0x07;
	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CH_EN|PMU_PMUREG_FVCI|PMU_PMUREG_MEAS_I|PMU_PMUREG_FIN|I_range<<15;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}
int AD5522_StartFIMV(handle_AD5522* h,uint32_t channel,uint8_t I_range)
{
	I_range&=0x07;

	//Configure DAC
	AD5522_SetOutputCurrent(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_CH_EN|PMU_PMUREG_FICV|PMU_PMUREG_MEAS_V|PMU_PMUREG_FIN|I_range<<15;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}

int AD5522_StartFVMV(handle_AD5522* h,__IO uint32_t channel,__IO uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	I_range&=0x07;
	h->i_range=I_range;

	//Configure DAC
	AD5522_SetOutputVoltage(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_FVCI|PMU_PMUREG_MEAS_V|PMU_PMUREG_FIN|I_range<<15;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
}
int AD5522_StartFIMI(handle_AD5522* h,uint32_t channel,uint8_t I_range)
{
	//Configure SYS
	uint32_t cmd=0;
	I_range&=0x07;
	h->i_range=I_range;

	
	//Configure DAC
	AD5522_SetOutputCurrent(h,channel,32768);
	//configure PMU
	for(int i=0;i<4;i++)
	{
		if((channel&(PMU_CH_0<<i))!=0)
		{
			//configure PMU
			__IO uint32_t cmd = h->reg_pmu[i];
			cmd&=~(PMU_CH_0|PMU_CH_1|PMU_CH_2|PMU_CH_3); // Clear channel selection
			cmd|=(PMU_CH_0<<i);
			cmd|=PMU_PMUREG_FICV|PMU_PMUREG_MEAS_I|PMU_PMUREG_FIN|I_range<<15;
			AD5522_SetPMU(h,PMU_CH_0<<i,cmd);
		}
	}
	return 0;
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
	return 0;
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
	return 0;
}
