#include "lcd.h"

/******************************************************************************
* Global functions
******************************************************************************/

u16 maxX = MAX_X;
u16 maxY = MAX_Y;


void LCD_Gpio_Init()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);

	LL_GPIO_InitTypeDef GPIO_InitStruct1 = { 0 };

	GPIO_InitStruct1.Pin			= PORT_D_PINS;
	GPIO_InitStruct1.Mode		    = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct1.Speed		    = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct1.OutputType	    = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct1.Pull		    = LL_GPIO_PULL_NO;
	
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct1);

	LL_GPIO_InitTypeDef GPIO_InitStruct2 = { 0 };

	GPIO_InitStruct2.Pin			= PORT_E_PINS;
	GPIO_InitStruct2.Mode		    = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct2.Speed		    = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct2.OutputType	    = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct2.Pull		    = LL_GPIO_PULL_NO;
					
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct2);

}

void LCD_FSMC_Init()
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
						   	 
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_FSMC);	
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin			= LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	GPIO_InitStruct.Mode		= LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed		= LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull		= LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate	= LL_GPIO_AF_12;
				
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	LL_GPIO_InitTypeDef GPIO_InitStruct1 = { 0 };
	
	GPIO_InitStruct1.Pin			= LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	GPIO_InitStruct1.Mode		= LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct1.Speed		= LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct1.OutputType	= LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct1.Pull		= LL_GPIO_PULL_NO;
	GPIO_InitStruct1.Alternate	= LL_GPIO_AF_12;
				
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct1);
	
	///////////////////////////////// 
	//	    // CS -> 1
	//	    // Reset -> 0
	//	    // RD -> 1
	//	    // RW -> 1
	
	LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_7);
	LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_4);
	LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_5);

	/* FSMC timings setup */
	
	FSMC_NORSRAM_TimingTypeDef Timing;
	
	Timing.AddressSetupTime = 2;	
	Timing.DataSetupTime = 2;
	Timing.BusTurnAroundDuration = 6;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;

	FSMC_NORSRAM_Timing_Init(FSMC_NORSRAM_DEVICE, &Timing, 0);	


	int FSMC_Bank = 0;
	/* timing structure */
	/* from datasheet:
	     address setup: 0ns
	     address hold: 0ns
	     Data setup: 5ns
	     Data hold: 5ns
	     Data access: 250ns
	     output hold: 100ns
	 */
	//FSMC_Bank1->BTCR[FSMC_Bank + 1] = FSMC_BTR1_ADDSET_1 | FSMC_BTR1_DATAST_1;

	/* Bank1 NOR/SRAM control register configuration */
	FSMC_Bank1->BTCR[FSMC_Bank] = FSMC_BCR1_MWID_0 | FSMC_BCR1_WREN | FSMC_BCR1_MBKEN;

}

void LCD_Set_Orientation(u8 orient)/*Sets orientation*/
{
	LCD_Write_Reg(0x36);
	switch (orient)
	{
	case 0: TFT_Write_Data(0x48);
	break;
	case 1: TFT_Write_Data(0x28);
	break;
	case 2: TFT_Write_Data(0x88);
	break;
	case 3: TFT_Write_Data(0xE8);
	break;
	}
	if (orient == 0 || orient == 2)
	{
		maxX = 239;
		maxY = 399;
	}
	else
	{
		maxX = 399;
		maxY = 239;
	}
}

void LCD_Init()
{	
	LCD_Gpio_Init();
	
	#ifdef LCD_FSMC	
	LCD_FSMC_Init();
	#endif // LCD_FSMC	
		

	RST_HIGH;	

	LCD_Write_Reg(0x01);          //Software Reset
	//delay_us(500000);

	//TFT_Send_Cmd(0x11);			//disp on
	//delay_us(1200000);
	
	//Memory Access Control
	LCD_Set_Orientation(ORIENT);  //выбираем ориентацию дисплея

	//COLMOD: Pixel Format Set
	LCD_Write_Reg(0x3A);  		//один пиксель будет кодироваться 16 битами
	TFT_Write_Data(0x55);
	
	//Frame Rate Control 
	LCD_Write_Reg(0xB1);
	TFT_Write_Data(0x11);
	TFT_Write_Data(0x18);  		//Frame Rate 79Hz	11 - 18
	
	
	//Power Control 1
	LCD_Write_Reg(0xC0); 		//задаём градацию серого цвета
	TFT_Write_Data(0x25);	


	//Power Control 2
	LCD_Write_Reg(0xC1); 		//настройка step up преобразователя
	TFT_Write_Data(0x11);


	//VCOM Control 1
	LCD_Write_Reg(0xC5); 		//контрастность определяется разностью VCOMH - VCOML = 5.2V
	TFT_Write_Data(0x2B); 	    //VCOMH = 3.825
	TFT_Write_Data(0x2B);   	//VCOML = -1.375
	
	//VCOM Control 2
	LCD_Write_Reg(0xC7); 		//на Vcom по сути ШИМ, а тут мы задаем offset для него
	TFT_Write_Data(0x86);       //VML=58 VMH=58
	

	//Display Function Control
	LCD_Write_Reg(0xB6);
	TFT_Write_Data(0x0A);
	TFT_Write_Data(0x82);		//восьмой бит определяет нормальный цвет кристала белый - 1, черный - 0,
	TFT_Write_Data(0x27);

	// Sleep Out
	LCD_Write_Reg(0x11);

	//delay_us(1200000);

	//Display On
	LCD_Write_Reg(0x29);
}




/* Main API functions-------------------------------------------------------*/

void LCD_Draw_Circle(uint16_t pos_x, uint16_t pos_y, uint8_t r,uint16_t color)
{
	int x = -r, y = 0, err = 2-2*r, e2;
	do 
	{
		TFT_Draw_Pixel(pos_x-x, pos_y+y,color);
		TFT_Draw_Pixel(pos_x+x, pos_y+y,color);
		TFT_Draw_Pixel(pos_x+x, pos_y-y,color);
		TFT_Draw_Pixel(pos_x-x, pos_y-y,color);
		e2 = err;
		if (e2 <= y) 
		{
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x) err += ++x*2+1;
	} while (x <= 0);
}

void LCD_Fill_Circle(uint16_t pos_x, uint16_t pos_y, uint8_t r,uint16_t color)
{
	int x = -r, y = 0, err = 2-2*r, e2;
	do 
	{

		LCD_DrawVLine(pos_x-x, pos_y-y, 2*y, color);
		LCD_DrawVLine(pos_x+x, pos_y-y, 2*y, color);

		e2 = err;
		if (e2 <= y) 
		{
			err += ++y*2+1;
			if (-x == y && e2 <= x) e2 = 0;
		}
		if (e2 > x) err += ++x*2+1;
	} while (x <= 0);

}

void LCD_Draw_Text(u16 Line, u16 Pos, u16 Color, u16 Back, char *string, u8 size) 
{
  LCD_Draw_String(Pos*FONT_X*size, Line*FONT_Y*size, Color, Back, string, size);
}
