
#include "iic_func.h"
#include "stm32f4xx_hal.h"

//extern I2C_HandleTypeDef hi2c2;
//extern uint16_t LCD_ADDR;

extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c1;

I2C_HandleTypeDef *pI2C_LCD;

uint8_t LCD_ADDR=0x7C; //3E //(0x27 << 1)

void rst_LCD(void)
{
	/*HAL_GPIO_WritePin(GPIOC, nRSTind_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOC, nRSTind_Pin, GPIO_PIN_RESET);
	HAL_Delay(2000);

	HAL_GPIO_WritePin(GPIOC, nRSTind_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);*/
	
}

void LCD_Init(I2C_HandleTypeDef *h,uint8_t contrast)
{
		rst_LCD();
	
		pI2C_LCD=h;
		uint8_t cmd=0;
		
	
	//0b00111001; Function Set 
		Function_Set_On(1,0);
		HAL_Delay(30);
	
		Function_Set_On(1,1);
		HAL_Delay(30);
		
	// one more time
		//ExtMode=1<<0;
    //LCD_SendCommand(cmd|bits8|numberOfLine|doubleHeightFont|ExtMode); //Function Set
	
		// Internal OSC Frequency
		Internal_OSC_Frequency(BIAS1_4,0x0C);// 1..8
	
		//Contrast set
		Contrast_set(contrast); //1..16
		
		// Power/ICON/Contrast control
		printf("Power/ICON/Contrast control\n");
		Power_ICON_Contrast();
		
		//Follower control
		Follower_control();
		printf("Follower control\n");
		
		/*// Cursor or Display Shift ;DISPLAY ON 
		cmd=1<<4; //0001xxxx
		uint8_t SC=0<<3; // Screen/Cursor select bit
		uint8_t RL=1<<2; // R/L: Right/Left
		//uint8_t =0<<1;
		//uint8_t =0<<0; //
		
		LCD_SendCommand(cmd|SC|RL);*/
		
    // Display ON/OFF control
		Display_ON(1);
		
		// CLEAR DISPLAY
		printf("CLEAR DISPLAY\n");
		LCD_SendCommand(0x01);
		
		// ENTRY MODE SET , CURSOR MOVES TO RIGHT
		printf("ENTRY MODE SET , CURSOR MOVES TO RIGHT\n");
		LCD_SendCommand(0x06);
		
		printf("Return_Home\n");
		Return_Home();
		HAL_Delay(100);
		
	
}

HAL_StatusTypeDef LCD_SendInternal(uint8_t data,uint8_t RS_) 
{
    HAL_StatusTypeDef res;
	
		//printf(". not ready 0x%x 0x%x\n",lcd_addr<<1,data);
		for(;;) 
		{
			res = HAL_I2C_IsDeviceReady(pI2C_LCD, LCD_ADDR, 1, HAL_MAX_DELAY);
			
			HAL_Delay(10);
			
			if(res == HAL_OK)
					break;
		}
		
		uint8_t RS=RS_<<6;
		uint8_t Co=0<<7;
		
		if(RS==0)
		{
			uint8_t cmd=Co|RS|0;
			uint8_t data_arr[2];

			printf("cmd=%x data=%x\n",cmd,data);
			data_arr[0]=cmd;
			data_arr[1]=data;			
			res = HAL_I2C_Master_Transmit(pI2C_LCD, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
			//res = HAL_I2C_Mem_Write(pI2C_LCD, LCD_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, data_arr, 1,HAL_MAX_DELAY);
			
			if(res!=HAL_OK)
			{
				printf(" Transmit %s\n",error_HAL(res));
				return res;
			}
	
			
			return res;
		}
		else
		{
			// RS Instruction 0 | Data 1
			uint8_t cmd=Co|RS|0;
			uint8_t data_arr[4];
			uint8_t add=0xc0+0;//add;
			data_arr[0]=0x08;//cmd;
			data_arr[1]=add;
			data_arr[2]=0x40;
			data_arr[3]=data;
			printf("cmd=%0x add=%x data=%0x\n",cmd,add,data);
			
			res = HAL_I2C_Master_Transmit(pI2C_LCD, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
			//res = HAL_I2C_Mem_Write(pI2C_LCD, LCD_ADDR, cmd,I2C_MEMADD_SIZE_8BIT, data_arr, 1 ,HAL_MAX_DELAY);
			
			if(res!=HAL_OK)
			{
				printf(" Transmit %s\n",error_HAL(res));
				return res;
			}
			
			HAL_Delay(LCD_DELAY_MS);
			
			return res;			
		}
    /*uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN; // up|0|00001000|00000100
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;
*/
    return res;
}

HAL_StatusTypeDef LCDI2C_write4bits(uint8_t cmd,uint8_t data)
{
	uint8_t data_[1];
	data_[0]=data;
	printf("nib = %x \n",data_[0]);
	
	HAL_StatusTypeDef res = HAL_I2C_Mem_Write(pI2C_LCD, LCD_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, data_, 1,HAL_MAX_DELAY);	
	if(res!=HAL_OK)
	{
		printf(" Transmit %s\n",error_HAL(res));
		return res;
	}	
	return res;
}

uint8_t ReadBusyFlagAndAddress()
{
	// It just only could write Data or Instruction to ST7032 by the IIC Interface.
	// It could not read Data or Instruction from ST7032 (except Acknowledge signal)
	
		HAL_StatusTypeDef res;
	
		printf("ReadBusyFlagAndAddress ");
	
		for(;;) 
		{
			res = HAL_I2C_IsDeviceReady(pI2C_LCD, LCD_ADDR, 1, HAL_MAX_DELAY);
			
			if(res == HAL_OK)
					break;
			else
				printf(".");
		}
		
			uint8_t RW=1;
			uint16_t lcd_addr1=LCD_ADDR|RW;
			uint8_t Co=0<<7;
			uint8_t RS=0<<6;
			
			uint8_t cmd=Co|RS|0;
			uint8_t data_arr[1];
			data_arr[0]=cmd;

			res = HAL_I2C_Master_Transmit(pI2C_LCD, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
			if(res!=HAL_OK)
				printf("Transmit !=HAL_OK");
			
	
			res = HAL_I2C_Master_Receive(pI2C_LCD, LCD_ADDR, data_arr, 1, HAL_MAX_DELAY);
			//HAL_Delay(LCD_DELAY_MS);
			if(res!=HAL_OK)
				printf("Receive !=HAL_OK");			
			printf(" %x ",data_arr[0]);
		
			return data_arr[0];
		
}																	 

HAL_StatusTypeDef LCD_SendCommand( uint8_t cmd) 
{
		printf("LCD_SendCommand 0x%0x\n",cmd);
		HAL_StatusTypeDef res;
	
		for(int i=0; i<3; i++)
		{
			res=LCD_SendInternal(cmd,0);
			if( res == HAL_OK)
				return res;
		}
    return res;
		//HAL_Delay(10);
}

void LCD_SendData(uint8_t data) 
{
		printf("LCD_SendData %c\n",data);
    LCD_SendInternal(data, 1);
		
}


void LCD_SendString(char *str) 
{
		printf("LCD_SendString %s\n",str);
	
    while(*str) 
		{
        LCD_SendData((uint8_t)(*str));
        str++;
    }
}


void I2C_Scan(char *str,I2C_HandleTypeDef *pH) 
{
    char info[] = "Scanning I2C bus...\r\n";
    //HAL_UART_Transmit(pI2C_LCD, (uint8_t*)info, strlen(info),HAL_MAX_DELAY);

    HAL_StatusTypeDef res;
		
		printf("%s pH= %x  : ",str,pH);
	
    for(uint16_t address = 0; address < 128 ; address++) 
		{
			/*if(address==0x3e)
				continue;*/
				//HAL_Delay(1);
        res = HAL_I2C_IsDeviceReady(pH, address << 1  , 1, 100);//HAL_MAX_DELAY);
			
        if(res == HAL_OK) 
				{
 						printf("device address 0x%x ",address << 1);
						/*if(address == 0x3E)
							printf("OK");*/
        }
				else 
				{
					//printf("",address);
            //HAL_UART_Transmit(pI2C_LCD, (uint8_t*)".", 1,  HAL_MAX_DELAY);
        }
    }  
		printf("\n");
    //HAL_UART_Transmit(pI2C_LCD, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
}

/*void start()
{
	SDA(0);
	SCL(0);
}
void stop()
{
	SCL(1);
	SDA(1);
	
}
void send(uint8_t byte)
{
	start();
	body(byte);
	stop();
}
void body(uint8_t byte)
{
	printf("\n");
	uint8_t vv = byte;
	for(int i=0; i<8 ;i++)
	{
		uint8_t res = 0;
		
		if(0x01 & vv)
			res=1;
		vv	= vv >> 1;		
		printf("%x",res);
		SDA(res);
		SCL(1);
		SCL(0);
	}
		
	SDA(1);
	SCL(1);
	printf("\n");
	
	if( HAL_GPIO_ReadPin(GPIOB, SDA1_Pin) == GPIO_PIN_SET)
		printf("ok");
	else
		printf("xx");
	
}

void SCL(uint8_t vv)
{
	if(vv==1)
		HAL_GPIO_WritePin(GPIOB, SCL1_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, SCL1_Pin, GPIO_PIN_RESET);
	
	HAL_Delay(1);
}

void SDA(uint8_t vv)
{
	if(vv == 1)
		HAL_GPIO_WritePin(GPIOB, SDA1_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, SDA1_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}*/

uint8_t revert(uint8_t data)
{
		uint8_t data_=0;
	
		for(int i=0; i<8;i++)	// begin to end
		{
			if((data &0x01)==0)
				data_=data_&0x01;
			else
				;
			data_=data_<<1;				
			data=data>>1;	
			//printf("data  %x\n",data);
			//printf("data_ %x\n",data_);
		}
		return data_;
}
char * error_HAL(HAL_StatusTypeDef res)
{
	if(res == HAL_ERROR)
		return "HAL_ERROR";
	else if(res == HAL_BUSY)
		return "HAL_BUSY";
	else if(res == HAL_TIMEOUT)
		return "HAL_TIMEOUT";
	else 
		return "HAL_???";
	
}

void Display_ON(uint8_t On)
{
		printf("Display ON/OFF control\n");
		uint8_t cmd=1<<3;//0x08; //00001xxx
		uint8_t D=On<<2; // entire display is turned on.
		uint8_t C=0<<1;  // cursor is turned on.
		uint8_t B=0<<0; //cursor blink is on 
		// 0C
	  LCD_SendCommand(cmd|D|C|B);
}

void Function_Set_On(uint8_t On,uint8_t ExtMode)
{
		// 4-bit mode, 2 lines, 5x7 format
		uint8_t cmd=On<<5; //0010xxxx
		uint8_t bits8=1<<4;
		uint8_t numberOfLine=1<<3;
		uint8_t doubleHeightFont=0	<<2;
		uint8_t db1=0<<1;
		uint8_t ExtMode_=ExtMode<<0;
	
		LCD_SendCommand(cmd|bits8|numberOfLine|doubleHeightFont|db1|ExtMode_);
}

void Contrast_set(uint8_t val) // 1..16
{
		uint8_t cmd=0x70; //0111xxxx
		uint8_t contrast =val & 0x0f;

	/*uint8_t C3=1<<3;
		uint8_t C2=0<<2;
		uint8_t C1=0<<1;
		uint8_t C0=0<<0;*/
		
		LCD_SendCommand(cmd|contrast); 
}

void Internal_OSC_Frequency(uint8_t bias,uint8_t freq) //1..8
{
		uint8_t cmd=0x10; //0001xxxx
		uint8_t BS=bias;//<<3; // 1/5bias
		/*uint8_t freq=val & 0x07;
		uint8_t F2=1<<2;
		uint8_t F1=0<<1;
		uint8_t F0=0<<0; //100=183Hz*/
	
		LCD_SendCommand(cmd|BS|freq); 
}

void Power_ICON_Contrast()
{
		uint8_t cmd=0x50; //0101xxxx
		uint8_t Ion=0<<3; // ICON display on/off
		uint8_t Bon=1<<2; // set booster circuit on/of
		uint8_t C5=1<<1;  //Contrast set for internal follower mode
		uint8_t C4=1<<0;
		
		LCD_SendCommand(cmd|Ion|Bon|C5|C4); 
}

void Follower_control()
{
		uint8_t cmd=0x60; //0110xxxx
		uint8_t Fon=0<3; // 
		uint8_t Rab2=1<<2; // 
		uint8_t Rab1=1<<1;  //
		uint8_t Rab0=1<<0;
		
		LCD_SendCommand(cmd|Fon|Rab2|Rab1|Rab0); 
}
void Return_Home()
{
	LCD_SendCommand(0x02); 
}

