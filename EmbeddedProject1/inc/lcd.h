#pragma once

/******************************************************************************
* Includes
******************************************************************************/

#include "main.h"
#include "font8x16.h"
#include "string.h"

/******************************************************************************
* Types
******************************************************************************/

typedef struct
{	
	u8		page;
	u16		x_pos; 
	u16		y_pos; 
	u16		color; 
	u16		back_color;	
	u8		size;

	u8		mode;
	
	char	text1[11];
	char	param1[9];
	char	param2[9];
	char	param3[9];
		
	void*	data_ptr;
	u32		data_type;
	f32		data_multiplier;
	f32		data_increment;

}LCD_Prop_typedef;



/******************************************************************************
* Defines
******************************************************************************/

#define LCD_FSMC


#define LCD_DATA 0x60080000	// для записи данных	
#define LCD_REG	 0x60000000	// для записи команд

#define FONT_X  8
#define FONT_Y  8

#define MIN_X	0
#define MIN_Y	0

#define MAX_X  239
#define MAX_Y  399

#define ORIENT	1

#define CYAN	0x07FF
#define MAGENTA 0xF81F
#define RED		0xf800
#define GREEN	0x07e0
#define BLUE	0x001f
#define BLACK	0x0000
#define YELLOW	0xffe0
#define WHITE	0xffff



extern const unsigned char simbols[][8];

enum {NONE_MODE, DATA_MODE, TEXT_MODE};
enum {TYPE_U8, TYPE_U16, TYPE_U32, TYPE_S8, TYPE_S16, TYPE_S32, TYPE_FLOAT, TYPE_DOUBLE};

#define LENGTH_8	0xff
#define LENGTH_16	0xffff
#define LENGTH_32	0xffffffff

#define RS_LOW		LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_13)	// data/cmd			FSMC_A18
#define RS_HIGH		LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_13)

#define RD_LOW		LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_4)	// read             FSMC_NOE
#define RD_HIGH		LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_4)

#define CS_LOW		LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_7)	// chip select pin  FSMC_NE1
#define CS_HIGH		LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_7)

#define WR_LOW		LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_5)	// write            FSMC_NWE
#define WR_HIGH		LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_5)

#define RST_LOW		LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_0)	// reset            PE0
#define RST_HIGH	LL_GPIO_SetOutputPin(GPIOE, LL_GPIO_PIN_0)

#define PORT_D_PINS		(LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5 | LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15)

#define PORT_E_PINS		(LL_GPIO_PIN_7 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15|LL_GPIO_PIN_0)

#ifdef LCD_FSMC
	
#endif // LCD_FSMC



/******************************************************************************
* Global functions
******************************************************************************/

void LCD_Gpio_Init();
void LCD_FSMC_Init();
void LCD_Set_Orientation(u8);/*Sets orientation*/
void LCD_Init();
void LCD_Draw_Circle(uint16_t, uint16_t ,uint8_t ,uint16_t );
void LCD_Fill_Circle(uint16_t, uint16_t ,uint8_t ,uint16_t );
void LCD_Draw_Text(u16, u16, u16 , u16 , char *, u8 ); 




/******************************************************************************
* Inline functions
******************************************************************************/

#ifdef LCD_FSMC

static inline void	LCD_Write_Reg(u8 cmd)
{
	*(volatile uint16_t *)(LCD_REG) = cmd;	
}

static inline void	TFT_Write_Data16(u16 data)
{
	*(volatile uint16_t *)(LCD_DATA) = data;
}

static inline void	TFT_Write_Data(u8 data)
{
	*(volatile uint16_t *)(LCD_DATA) = data;
}

static inline u16 LCD_ReadReg(u8 LCD_Reg) {
	/* Write 16-bit Index (then Read Reg) */
	*(volatile u16 *)(LCD_REG) = LCD_Reg;
	/* Read 16-bit Reg */
	return (*(volatile u16 *)(LCD_DATA));
}

#else


	void pin_set(GPIO_TypeDef *GPIOx, u32 pinnum, u32 Bit)
{
	if (Bit) LL_GPIO_SetOutputPin(GPIOx, pinnum);
	else	 LL_GPIO_ResetOutputPin(GPIOx, pinnum);
}

	void data_set(u16 cmd)
{	
	//	15	14	13	12	11	10	9	8	7	6	5	4	3	2	1	0		Data bits
	//	10  9	8	-	-	-	-	-	-	-	-	-	1   0   15  14		PD
	//  -	-	-	15	14	13  12  11  10  9   8   7   -   -   -	-		PE
		
	uint16_t cmd_bits[16] = {0};
		
	uint32_t port_d = 0L;
	uint32_t port_e = 0L;
	
	for (uint8_t i = 0; i < 16; i++)
	{
		cmd_bits[i] = (cmd>>i) & 1;		
	}
	
	pin_set(GPIOD, LL_GPIO_PIN_10, cmd_bits[15]);
	pin_set(GPIOD, LL_GPIO_PIN_9,  cmd_bits[14]);
	pin_set(GPIOD, LL_GPIO_PIN_8,  cmd_bits[13]);
	pin_set(GPIOD, LL_GPIO_PIN_1,  cmd_bits[3]);
	pin_set(GPIOD, LL_GPIO_PIN_0,  cmd_bits[2]);
	pin_set(GPIOD, LL_GPIO_PIN_15, cmd_bits[1]);
	pin_set(GPIOD, LL_GPIO_PIN_14, cmd_bits[0]);

	pin_set(GPIOE, LL_GPIO_PIN_15, cmd_bits[12]);
	pin_set(GPIOE, LL_GPIO_PIN_14, cmd_bits[11]);
	pin_set(GPIOE, LL_GPIO_PIN_13, cmd_bits[10]);
	pin_set(GPIOE, LL_GPIO_PIN_12, cmd_bits[9]);
	pin_set(GPIOE, LL_GPIO_PIN_11, cmd_bits[8]);
	pin_set(GPIOE, LL_GPIO_PIN_10, cmd_bits[7]);
	pin_set(GPIOE, LL_GPIO_PIN_9,  cmd_bits[6]);
	pin_set(GPIOE, LL_GPIO_PIN_8,  cmd_bits[5]);
	pin_set(GPIOE, LL_GPIO_PIN_7,  cmd_bits[4]);

	
	//GPIOD->ODR |=  (cmd_bits[15] << 10) | (cmd_bits[14] << 9)  | (cmd_bits[13] << 8)  | (cmd_bits[3] << 1)  | (cmd_bits[2] << 0)  | (cmd_bits[1] << 15)\
	//			 | (cmd_bits[0] << 14);
	//GPIOE->ODR |=  (cmd_bits[12] << 15) | (cmd_bits[11] << 14) | (cmd_bits[10] << 13) | (cmd_bits[9] << 12) | (cmd_bits[8] << 11) | (cmd_bits[7] << 10)\
	//			 | (cmd_bits[6] << 9)	| (cmd_bits[5]  << 8)  | (cmd_bits[4]  << 7);	
}

	__STATIC_INLINE void	LCD_Write_Reg(u8 cmd)
{
	RS_LOW;						//будем слать команду
	RD_HIGH;					//выставляем на ножке, отвечающей за чтение 1
	
	CS_LOW;						//активируем чип
	
	WR_LOW;						//стробируем битом записи
	data_set(cmd);
	delay_us(1);
	WR_HIGH;	

	CS_HIGH;					//деактивируем чип
	
}

	__STATIC_INLINE void	TFT_Write_Data16(u16 data)
{
	RS_HIGH;					//будем слать ДАННЫЕ
	RD_HIGH;					//выставляем на ножке, отвечающей за чтение 1
	
	CS_LOW;						//активируем чип
		
	WR_LOW; 					//стробируем битом записи
	data_set((uint16_t)data);
	delay_us(1);
	WR_HIGH;
	
	CS_HIGH;					//деактивируем чип
}

	__STATIC_INLINE void	TFT_Write_Data(u8 data)
{
	RS_HIGH; 					//будем слать ДАННЫЕ
	RD_HIGH; 					//выставляем на ножке, отвечающей за чтение 1
	
	CS_LOW; 						//активируем чип
		
	WR_LOW;  					//стробируем битом записи
	data_set(data);
	delay_us(1);
	WR_HIGH;
	
	CS_HIGH; 					//деактивируем чип
}
	
	u32 TFT_Read_Data(void)
{
	u32 data = 0;
	u32 tmpd, tmpe;
	
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;	
	GPIO_InitStruct.Mode		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull		= LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	LL_GPIO_InitTypeDef GPIO_InitStruct1 = { 0 };
	GPIO_InitStruct.Pin			= LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_13 | LL_GPIO_PIN_12 | LL_GPIO_PIN_11 | LL_GPIO_PIN_10 | LL_GPIO_PIN_9 |											LL_GPIO_PIN_8 | LL_GPIO_PIN_7;	
	GPIO_InitStruct.Mode		= LL_GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull		= LL_GPIO_PULL_DOWN;
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct1);
	
	RS_HIGH;
	WR_HIGH;

	CS_LOW;

	RD_LOW;
	delay_us(50);

	//	15	14	13	12	11	10	9	8	7	6	5	4	3	2	1	0		Data bits
	//	10  9	8	-	-	-	-	-	-	-	-	-	1   0   15  14		PD			tmpd
	//  -	-	-	15	14	13  12  11  10  9   8   7   -   -   -	-		PE			tmpe

	tmpd = LL_GPIO_ReadInputPort(GPIOD);
	tmpe = LL_GPIO_ReadInputPort(GPIOE);

//	data = (((tmpd >> 10) & 1) << 15) | (((tmpd >> 9) & 1) << 14) | (((tmpd >> 8) & 1) << 13) | (((tmpe >> 15) & 1) << 12) | (((tmpe >> 14) & 1) << 11)\
//	| (((tmpe >> 13) & 1) << 10) | (((tmpe >> 12) & 1) << 9)  | (((tmpe >> 11) & 1) << 8)  | (((tmpe >> 10) & 1) << 7)  | (((tmpe >> 9) & 1) << 6)\
//	| (((tmpe >> 8) & 1) << 5)   | (((tmpe >> 7) & 1) << 4) | (((tmpd >> 1) & 1) << 3) | (((tmpd >> 0) & 1) << 2) | (((tmpd >> 15) & 1) << 1)\
//	| (((tmpd >> 14) & 1) << 0);

	data = ((tmpd & 0x400) << 5) | ((tmpd & 0x200) << 5)	| ((tmpd & 0x100) << 5)		| ((tmpe & 0x8000)>> 3)		| ((tmpe & 0x4000)>> 3)\
		 | ((tmpe & 0x2000)>> 3) | ((tmpe & 0x1000)>> 3)	| ((tmpe & 0x800) >> 3)		| ((tmpe & 0x400) >> 3)		| ((tmpe & 0x200) >> 3)\
		 | ((tmpe & 0x100) >> 3) | ((tmpe & 0x080) >> 3)	| ((tmpd & 0x002) << 2)		| ((tmpd & 0x001) << 2)		| ((tmpd & 0x8000)>> 14)\
		 | ((tmpd & 0x4000)>> 14);
	
	RD_HIGH;

	CS_HIGH;


	LL_GPIO_InitTypeDef GPIO_InitStruct2 = { 0 };

	GPIO_InitStruct1.Pin			= LL_GPIO_PIN_0 | LL_GPIO_PIN_1 | LL_GPIO_PIN_8 | LL_GPIO_PIN_9 | LL_GPIO_PIN_10 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
	GPIO_InitStruct1.Mode		    = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct1.Speed		    = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct1.OutputType	    = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct1.Pull		    = LL_GPIO_PULL_NO;
	
	LL_GPIO_Init(GPIOD, &GPIO_InitStruct2);

	LL_GPIO_InitTypeDef GPIO_InitStruct3 = { 0 };

	GPIO_InitStruct2.Pin			= LL_GPIO_PIN_15 | LL_GPIO_PIN_14 | LL_GPIO_PIN_13 | LL_GPIO_PIN_12 | LL_GPIO_PIN_11 | LL_GPIO_PIN_10 | LL_GPIO_PIN_9 |											LL_GPIO_PIN_8 | LL_GPIO_PIN_7;
	GPIO_InitStruct2.Mode		    = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct2.Speed		    = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct2.OutputType	    = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct2.Pull		    = LL_GPIO_PULL_NO;
					
	LL_GPIO_Init(GPIOE, &GPIO_InitStruct3);
		
//	DATA_DDR_0 = 0X00; 	//порт на вход с подтяжкой к земле
//	DATA_PORT_0 = 0x00;
//	COMMAND_PORT |= (1 << lcd_dc); 		//будем читать ДАННЫЕ
//	COMMAND_PORT1 |= (1 << lcd_wr); 	 //выставляем на ножке, отвечающей за запись 1
//	
//	COMMAND_PORT &= ~(1 << lcd_cs);     //активируем чип
//	
//	COMMAND_PORT &= ~(1 << lcd_rd);     //стробируем битом чтения
//	_delay_us(5);
//	data = PINA;
//	COMMAND_PORT |= (1 << lcd_rd);
//	
//	COMMAND_PORT |= (1 << lcd_cs);     //деактивируем чип
//	DATA_DDR_0 = 0XFF; 				//порт на выход
	return data;
}

#endif // LCD_FSMC




static inline void TFT_Send_Data(u16 data)
{
	u8 data1 = data >> 8;
	u8 data2 = data & 0xff;
	TFT_Write_Data(data1);
	TFT_Write_Data(data2);	
}

/*ф-ция ограничивает координаты  рабочей области по оси Х*/
static inline void TFT_Set_Column(uint16_t start_column, uint16_t end_colunm)
{
	LCD_Write_Reg(0x2A);                                           
	TFT_Send_Data(start_column);
	TFT_Send_Data(end_colunm);
}

/*ф-ция ограничивает координаты  рабочей области по оси Y*/
static inline void TFT_Set_Page(uint16_t start_page, uint16_t end_page)
{
	LCD_Write_Reg(0x2B);                                                      
	TFT_Send_Data(start_page);
	TFT_Send_Data(end_page);
}

/*ф-ция ограничивает координаты  рабочей области*/
static inline void LCD_Set_XY(uint16_t x, uint16_t y)
{
	TFT_Set_Column(x, x);
	TFT_Set_Page(y, y);
}

/*ф-ция отрисовывает пиксель по заданным координатам*/
static inline void TFT_Draw_Pixel(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_Set_XY(x, y);
	LCD_Write_Reg(0x2c);
	TFT_Write_Data16(color);
}

static inline uint16_t constrain(uint16_t a, uint16_t b, uint16_t c)/*runtime*/
{
	if (a < b)
	{
		return b;
	}
	if (c < a)
	{
		return c;
	}
	else return a;
}




static inline void LCD_Fill_Screen(uint16_t x_left, uint16_t x_right, uint16_t y_up, uint16_t y_down, uint16_t color)
{
	unsigned long  xy = 0;
	unsigned long  i  = 0;
//	if (x_left > x_right)
//	{	
//		x_left = x_left ^ x_right;      //если координата левого края больше
//		x_right = x_left ^ x_right;     //координаты правого края они поменяются
//		x_left = x_left ^ x_right;      //местами, было x_left = 5 x_right = 3 
//										//стало x_left = 3 x_right = 5
//	}
//	if (y_up > y_down)
//	{
//		y_up = y_up ^ y_down; 			//то же самое для оси y							
//		y_down = y_up ^ y_down; 		//название этой операции
//		y_up = y_up ^ y_down; 			//"swap без временной переменной"
//	}
	//контролируем, что бы передаваемые в функцию координаты
	//входили в область допустимых значений
//	x_left = constrain(x_left, MIN_X, MAX_X);
//	x_right = constrain(x_right, MIN_X, MAX_X);
//	y_up = constrain(y_up, MIN_Y, MAX_Y);
//	y_down = constrain(y_down, MIN_Y, MAX_Y);

	xy = (x_right - x_left + 1); 		//рассчитываем количество точек
	xy = xy*(y_down - y_up + 1); 		//которое надо закрасить

	TFT_Set_Column(x_left, x_right); 	//задаём рабочую область по x
	TFT_Set_Page(y_up, y_down); 		//задаём рабочую область по y
	LCD_Write_Reg(0x2c); 				//будем писать в видео ОЗУ
	
	for(i = 0 ; i < xy ; i++)
	{
		TFT_Write_Data16(color); 		//передаём кодировку цвета
		//delay_us(1000);
	}
}

static inline void LCD_Fill_Rect(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t color)
{	
	if (length  && width)
	{
		LCD_Fill_Screen(x, x + length - 1, y, y + width - 1, color);
	}
	
}

static inline void LCD_Draw_Char(uint16_t x, uint16_t y, uint16_t color, uint16_t background, uint8_t ascii, uint8_t size)
{
	for (int i = 0; i < FONT_Y; i++)
	{
		for (uint8_t f = 0; f < FONT_X; f++)
		{
			if ((simbols[ascii - 0x20][i] >> (7 - f)) & 0x01)
			{
				LCD_Fill_Rect(x + f*size, y + i*size, size, size, color);
			}
			else
			{	
				LCD_Fill_Rect(x + f*size, y + i*size, size, size, background);
			}
		}
	}
}

static inline void LCD_Draw_String(uint16_t x, uint16_t y, uint16_t color,uint16_t phone,char *string, uint8_t size)
{
	extern u16 maxX;
        //определить конец строки очень просто если знать, что она ВСЕГДА заканчивается нулём
	while(*string)
	{      
                //проверяем не вылезем ли мы за пределы экрана при отрисовке следующего символа,
                // если да, то переходим на следующую строчку
		if((x + FONT_X) > maxX)
		{
			x = 1;
			y = y + FONT_X*size;
		}
		LCD_Draw_Char(x, y, color, phone,*string, size);//отрисовываем символ
		x += FONT_X*size;     //изменяем координату для отрисовки следующего символа
		*string++;           //увеличиваем значение указателя, чтобы он ссылался на следующий символ
	}
}

static inline void LCD_DrawHLine( uint16_t x, uint16_t y, uint16_t length,uint16_t color)
{
	TFT_Set_Column(x,x + length);
	TFT_Set_Page(y,y);
	LCD_Write_Reg(0x2c);
	for(int i=0; i<length; i++)
	TFT_Write_Data16(color);
}

static inline void LCD_DrawVLine( uint16_t x, uint16_t y, uint16_t length,uint16_t color)
{
	TFT_Set_Column(x,x);
	TFT_Set_Page(y,y+length);
	LCD_Write_Reg(0x2c);
	for(int i=0; i<length; i++)
	TFT_Write_Data16(color);
}

static inline void LCD_Draw_Rect(uint16_t x, uint16_t y, uint16_t length, uint16_t width,uint16_t color)
{
	LCD_DrawHLine(x, y, length, color);
	LCD_DrawHLine(x, y + width-1, length, color);
	LCD_DrawVLine(x, y, width,color);
	LCD_DrawVLine(x + length -1, y, width,color);
}
  