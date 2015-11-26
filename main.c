/**
*****************************************************************************
* @file    main.c
* @author  Yue Wang 12027710
* @date    03-June-2013
* @brief   Embedded system part, using stm32f10
****************************************************************************
*/

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#define QUEUE_SIZE 	64

/**
* @brief GPIO PINs for LCD
*/
#define	LCD_D4 GPIO_Pin_0
#define	LCD_D5 GPIO_Pin_1
#define LCD_D6 GPIO_Pin_2
#define LCD_D7 GPIO_Pin_3
#define LCD_EN GPIO_Pin_4
#define LCD_RS GPIO_Pin_5
#define LCD_DATA 		1
#define LCD_INSTRUCTION 0

/**
* @brief buffer for UART.
*/
typedef struct Queue
{
    char a[QUEUE_SIZE];
    int start;
    int end;
} Queue;

/**
* @brief structure for stepper motor.
*/
typedef struct wy_Stepper
{
    int direction;	//0 counter clock, 1 clockwise
    int action;	  	//0 stop         , 1 start
} wy_Stepper;

/**
* @brief structure for ultra sonic sensor.
*/
typedef struct wy_USensor
{
    int 	action;	//0 stop	  	 , 1 start
    uint16_t value;
} wy_USensor;

/**
* @brief Global variables.
*/
Queue RXQ, TXQ;
wy_Stepper Stp;
wy_USensor Usr;

/* Prototype------------------------------------------------------------------*/
/**
* @brief functions by Yue Wang.
*/
int		wy_Number_Builder(char *_p);
void 	wy_Initialize(void);
void	wy_PcCMD_Handler(char *_p);
void	wy_UButton_Handler(void);
uint16_t wy_USensor_Handler(wy_USensor *_u);

void	wy_M_Handler(char *_p);
void	wy_S_Handler(char *_p);
void	wy_G_Handler(void);
void	wy_g_Handler(char *_p);
void	wy_B_Handler(void);
void	wy_b_Handler(char *_p);
void	wy_C_Handler(void);
void	wy_c_Handler(char *_p);
void	wy_D_Handler(void);
void	wy_d_Handler(char *_p);
void	wy_R_Handler(char *_p);
void	wy_U_Handler(void);
void	wy_u_Handler(char *_p);

/**
* @brief functions by Peter.
*/
void	lcd_command(unsigned char command);
void 	lcd_putchar(unsigned char c);
static void set_RS(int mode);
static void pulse_EN(void);
static void lcd_init_8_bit(unsigned char command);

int 	canReadLine(void);
void 	readLine(char *s);
void 	writeLine(char *s);
void 	uart_putchar(char c);
void 	delay_microsec(register unsigned short n);
void 	delay_millisec(register unsigned short n);

int main(void)
{
    wy_Initialize();
    while (1)
    {
        /*	MCU -->> PC	*/
        Usr.value = wy_USensor_Handler(&Usr);
        wy_UButton_Handler();

        /*	MCU <<-- PC	*/
        if (canReadLine())
        {
            char str[80];
            readLine(str);
            wy_PcCMD_Handler(str);
        }
    }
}

/* Function definitions  ------------------------------------------------------*/
/**
* @brief handle Usensor and stepper.
*/
void SysTick_Handler(void)
{
    /*USensor*/
    if (Usr.action)
    {
        char d[4];
        d[0] = 'U';
        d[1] = ' ';
        d[2] = (Usr.value) >> 8;
        d[3] = (Usr.value) & 0x00ff;
        writeLine(d);
    }

    /*Stepper*/
    int c, j;
    static int i = 0;
    i %= 4;
    if (!Stp.action)
    {
        for (j = 0; j < 4; j++)
        {
            GPIO_WriteBit(GPIOA, 0x01 << i, 0);
        }
    }
    else
    {
        if (Stp.direction)
        {
            for (c = 1; c < 5; c++)
            {
                GPIO_WriteBit(GPIOA, 0x01 << c, ((0x01 << c) >> (i + 1)) & 0x01);
            }
        }
        else
        {
            for (c = 1; c < 5; c++)
            {
                GPIO_WriteBit(GPIOA, 0x01 << c, ((0x01 << c) >> (4 - i)) & 0x01);
            }
        }
    }
    i++;
}

/**
* @brief process string sent from PC.
*/
void wy_PcCMD_Handler(char *_p)
{
    switch (*_p)
    {
    case 'M':	wy_M_Handler(_p);	break;
    case 'G':	wy_G_Handler();	    break;
    case 'g':	wy_g_Handler(_p);	break;
    case 'B':	wy_B_Handler();	    break;
    case 'b':	wy_b_Handler(_p);	break;
    case 'S':	wy_S_Handler(_p);	break;
    case 's':	wy_S_Handler(_p);	break;
    case 'C':	wy_C_Handler();	    break;
    case 'c':	wy_c_Handler(_p);	break;
    case 'D':	wy_D_Handler();	    break;
    case 'd':	wy_d_Handler(_p);	break;
    case 'R':	wy_R_Handler(_p);	break;
    case 'U':	wy_U_Handler();	    break;
    case 'u':	wy_u_Handler(_p);	break;
    }
}

/**
* @brief toggle the running state of Usensor.
*/
void wy_U_Handler(void)
{
    static int i = 1;
    Usr.action = i;
    if (i)
    {
        SysTick_Config(SystemCoreClock / 10);
    }
    i = !i;
}

/**
* @brief toggle the running state of Usensor.
*/
void wy_u_Handler(char *_p)
{
    _p++;
    Usr.action = *_p - 48;
}
/**
* @brief control the speed of stepper by using the string from PC.
*/
void wy_R_Handler(char *_p)
{
    _p++;
    if (*_p == ' ')
    {
        _p++;
    }
    int num = wy_Number_Builder(_p);
    if (!num)
    {
        num = 1;
    }
    num = 100 / num;
    SysTick_Config(SystemCoreClock / num);
}

/**
* @brief toggle the direction of stepper.
*/
void wy_D_Handler(void)
{
    Stp.direction = !Stp.direction;
}

/**
* @brief specify the running state of stepper.
*/
void wy_c_Handler(char *_p)
{
    _p++;
    Stp.action = *_p - 48;
    SysTick_Config(SystemCoreClock / 100);
}

/**
* @brief toggle the running state of stepper.
*/
void wy_C_Handler(void)
{
    Stp.action = !Stp.action;
    SysTick_Config(SystemCoreClock / 100);
}

/**
* @brief specify the running state of Usensor.
*/
void wy_d_Handler(char *_p)
{
    _p++;
    Stp.direction = *_p - 48;
}

/**
* @brief control the position of servo by using the string from PC.
*/
void wy_S_Handler(char *_p)
{
    _p++;
    if (*_p == ' ')
    {
        _p++;
    }
    int num = wy_Number_Builder(_p);
    TIM_SetCompare3(TIM4, num * 11 + 1000);
}

/**
* @brief toggle the blue led.
*/
void wy_B_Handler(void)
{
    static int i = 0;
    i = !i;
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, i);
}

/**
* @brief specify the on off state of the blue led.
*/
void wy_b_Handler(char *_p)
{
    _p++;
    GPIO_WriteBit(GPIOC, GPIO_Pin_8, *_p - 48);
}

/**
* @brief toggle the green led.
*/
void wy_G_Handler(void)
{
    static int i = 0;
    i = !i;
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, i);
}

/**
* @brief specify the on off state of the green led.
*/
void wy_g_Handler(char *_p)
{
    _p++;
    GPIO_WriteBit(GPIOC, GPIO_Pin_9, *_p - 48);
}

/**
* @brief output the message to LCD.
*/
void wy_M_Handler(char *_p)
{
    lcd_command(0x01);
    delay_microsec(1520);
    _p++;
    if (*_p == ' ')
    {
        _p++;
    }
    while (*_p != '\0')
    {
        lcd_putchar(*_p);
        _p++;
    }
    lcd_command(0x02);
    delay_microsec(1520);
}

/**
* @brief send -100 as a char to PC request clearing the Line Edit
*/
void wy_UButton_Handler(void)
{
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
        char clr = -100;
        writeLine(&clr);
    }
}

/**
* @brief Using the status of the argument, generate the measured
* 	   distance in microsecond and return it after processing.
*/
uint16_t wy_USensor_Handler(wy_USensor *_u)
{
    uint16_t d;
    TIM_SetCompare3(TIM3, _u->action);
    if (_u->action)
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8))
        {
            TIM6->PSC = 23;   			// Set prescaler to 24 (PSC + 1)
            TIM6->ARR = 0xffff;
            TIM6->CNT = 0;    			//zero the counter
            TIM6->EGR = TIM_EGR_UG;
            TIM6->CR1 |= TIM_CR1_CEN; 	// start timer
        }
        while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8));
        TIM6->CR1 &= ~TIM_CR1_CEN; 		// stop timer

        d = TIM6->CNT;

        if (d > 60000)
        {
            d = 0;
        }

        if (((d - _u->value) > 1000) | ((d - _u->value) < -1000))
        {
            d = _u->value;
        }

        if (((d - _u->value) > 5))
        {
            d = (d + _u->value) / 2;
        }

        if (((d - _u->value) < -5))
        {
            d = (d + _u->value) / 2;
        }

        if (((d - _u->value) < 5) & ((d - _u->value) > -5))
        {
            d = _u->value;
        }

        if (d > 60000)
        {
            d = 0;
        }
        return d;
    }
    else
    {
        return _u->value;
    }
}

/**
* @brief return the integer built from the string specified.
*/
int wy_Number_Builder(char *_p)
{
    int len = 0;
    int	num = 0;
    while (*_p != '\0')
    {
        len++;
        _p++;
    }
    if (len == 0)
    {
        num = 0;
    }
    else
    {
        if (len > 2)
        {
            num = 100;
        }
        else
        {
            if (len == 1)
            {
                _p--;
                num = *_p - 48;
            }
            else
            {
                _p--;
                num = *_p - 48;
                _p--;
                num += (*_p - 48) * 10;
            }
        }
    }
    return num;
}

/**
* @brief Initialization for everything used in this programme.
*/
void wy_Initialize(void)
{
    /*RCC */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    /*GPIO*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    /*User Button*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*ECHO*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0<-->Trigger
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//PA8<-->Echo
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 240 - 1;
    TIM_TimeBaseStructure.TIM_Period = 10000 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef TIM_OCStructure;
    TIM_OCStructInit(&TIM_OCStructure);
    TIM_OCStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC3Init(TIM3, &TIM_OCStructure);

    TIM_Cmd(TIM3, ENABLE);
    TIM_SetCompare3(TIM3, 0);    //10usec PWM 0.1sec period

    Usr.action = 0;
    Usr.value = 0;

    /*STEPPER*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2
        | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    Stp.action = 0;
    Stp.direction = 1;

    SysTick_Config(SystemCoreClock / 100);

    /*LED*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /*LCD*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = LCD_D7 | LCD_D6 | LCD_D5 | LCD_D4 |
        LCD_EN | LCD_RS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_WriteBit(GPIOC, LCD_EN, Bit_RESET);
    delay_microsec(15000);

    lcd_init_8_bit(0x30);
    delay_microsec(4100);
    lcd_init_8_bit(0x30);
    delay_microsec(100);
    lcd_init_8_bit(0x30);
    lcd_init_8_bit(0x20);
    lcd_command(0x28);

    lcd_command(0x08);
    lcd_command(0x01);
    delay_microsec(1520);
    lcd_command(0x06);
    lcd_command(0x0c);

    /*PWM */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /*RXD */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  //PA9 <--->RXD
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*TXD */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	//PA10 <--->TXD
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*USART*/
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_Cmd(USART1, ENABLE);
    USART1->CR1 |= USART_CR1_RXNEIE; //enable Received data interrupt

                                     /*NVIC*/
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*TIM4*/
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 24 - 1;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /*CH3*/
    TIM_OCStructInit(&TIM_OCStructure);
    TIM_OCStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC3Init(TIM4, &TIM_OCStructure);

    TIM_Cmd(TIM4, ENABLE);
}

/**
* @brief USART handler.
*/
void USART1_IRQHandler(void)
{
    //was it a receive interrupt?
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char c = (USART1->DR & 0xff);
        if ((RXQ.end + 1) % QUEUE_SIZE != RXQ.start)
        {
            RXQ.a[RXQ.end] = c;
            RXQ.end = (RXQ.end + 1) % QUEUE_SIZE;
        }
    }
    //was it a transmit interrupt?
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if (TXQ.start != TXQ.end)
        {
            USART1->DR = TXQ.a[TXQ.start];
            TXQ.start = (TXQ.start + 1) % QUEUE_SIZE;
        }
        else
        {
            USART1->CR1 &= ~USART_CR1_TXEIE; //disable interrupt
        }
    }
}

/**
* @brief output to PUTTY.
*/
void uart_putchar(char c)
{
    if ((TXQ.end + 1) % QUEUE_SIZE != TXQ.start)
    {
        TXQ.a[TXQ.end] = c;
        TXQ.end = (TXQ.end + 1) % QUEUE_SIZE;
    }
    if (!(USART1->CR1&USART_CR1_TXEIE))
    {	//ie the tx buffer is empty
        USART1->CR1 |= USART_CR1_TXEIE;	//enable transmit interrupt
    }
}

void uart_putstring(char *s)
{
    register int i = 0;
    while (s[i] != '\0')
    {
        uart_putchar(s[i]);
        i++;
    }
}

/**
* @brief handle command by calling relevant functions.
*/
int canReadLine(void)
{
    int i = RXQ.start;
    while (i != RXQ.end)
    {
        if (RXQ.a[i] == '\n')
        {
            return 1;
        }
        i = (i + 1) % QUEUE_SIZE;
    }
    return 0;
}

/**
* @brief read the string from buffer and write into the char array specified.
*/
void readLine(char *s)
{
    int i = 0;
    s[i] = RXQ.a[RXQ.start];
    RXQ.start = (RXQ.start + 1) % QUEUE_SIZE;
    while (s[i] != '\n')
    {
        i++;
        s[i] = RXQ.a[RXQ.start];
        RXQ.start = (RXQ.start + 1) % QUEUE_SIZE;
    }
    s[i] = '\0';
}

/**
* @brief send the specified char array to UART .
*/
void writeLine(char *s)
{
    uart_putstring(s);
    uart_putchar('\n');
}

/**
* @brief by Peter.Refer to lecture note for more detail .
*/
static void set_RS(int mode)
{
    GPIO_WriteBit(GPIOC, LCD_RS, mode);
}

/**
* @brief by Peter.Refer to lecture note for more detail .
*/
static void pulse_EN(void)
{
    GPIO_WriteBit(GPIOC, LCD_EN, Bit_SET);
    delay_microsec(1);

    GPIO_WriteBit(GPIOC, LCD_EN, Bit_RESET);
    delay_microsec(1);
}

/**
* @brief by Peter.Refer to lecture note for more detail .
*/
static void lcd_init_8_bit(unsigned char command)
{
    set_RS(LCD_INSTRUCTION);
    GPIO_WriteBit(GPIOC, LCD_D4, (command >> 4) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D5, (command >> 5) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D6, (command >> 6) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D7, (command >> 7) & 0x01);
    pulse_EN();
    delay_microsec(37);
}

/**
* @brief by Peter.Refer to lecture note for more detail .
*/
void lcd_command(unsigned char command)
{
    set_RS(LCD_INSTRUCTION);
    GPIO_WriteBit(GPIOC, LCD_D4, (command >> 4) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D5, (command >> 5) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D6, (command >> 6) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D7, (command >> 7) & 0x01);
    pulse_EN();
    GPIO_WriteBit(GPIOC, LCD_D4, command & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D5, (command >> 1) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D6, (command >> 2) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D7, (command >> 3) & 0x01);
    pulse_EN();
    delay_microsec(37);
}
/**
* @brief by Peter.Refer to lecture note for more detail .
*/
void lcd_putchar(unsigned char c)
{
    set_RS(LCD_DATA);
    GPIO_WriteBit(GPIOC, LCD_D4, (c >> 4) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D5, (c >> 5) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D6, (c >> 6) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D7, (c >> 7) & 0x01);
    pulse_EN();

    GPIO_WriteBit(GPIOC, LCD_D4, c & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D5, (c >> 1) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D6, (c >> 2) & 0x01);
    GPIO_WriteBit(GPIOC, LCD_D7, (c >> 3) & 0x01);
    pulse_EN();
    delay_microsec(37);
}

/**
* @brief by Peter.Refer to lecture note for more detail .
*/
void delay_microsec(register unsigned short n)
{
    if (n > 5) n -= 5;
    else n = 1;
    TIM7->PSC = 23;
    TIM7->ARR = n;
    TIM7->CNT = 0;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->CR1 |= (TIM_CR1_OPM | TIM_CR1_CEN);
    while (TIM7->CR1 & TIM_CR1_CEN);
}
/**
* @brief by Peter.Refer to lecture note for more detail .
*/
void delay_millisec(register unsigned short n)
{
    if (n > 1) n--;
    TIM7->PSC = 23999;
    TIM7->ARR = n;
    TIM7->CNT = 0;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->CR1 |= (TIM_CR1_OPM | TIM_CR1_CEN);
    while (TIM7->CR1 & TIM_CR1_CEN);
}
