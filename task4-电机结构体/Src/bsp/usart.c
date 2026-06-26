#include "stm32f10x.h"                  // Device header
#include "usart.h"

volatile uint8_t  USART3_RxBuffer[256];  //接收不定长字符串的缓冲区
volatile uint16_t USART3_RxIndex = 0;    // 中断函数中使用 ,记录接收到的字符数量
volatile uint8_t  USART3_RxFinished = 0; // 1 表示接收到一个完整数据包
volatile uint16_t USART3_RxCount = 0;  //接收到的字符数量
/************************************************************

串口  PC10-->TX   PC11-->RX  设置部分重映射

****************************************************************/
void usart3_Init()
{
    //开启GPIOC,USART,AFIO
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //重映射
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    ///把PC10引脚初始化为复用推挽输出 把PC11引脚初始化为浮空输入
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //USART3 9600 - 8 - 0 - 0
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);

    //使能中断
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    //
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    //初始化NVIC为分组2(main.c)  抢占优先级1  响应优先级2

    //使能usart3
    USART_Cmd(USART3, ENABLE);
}


//使用usart3 发送一个字节
void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART3, Byte);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

//使用usart3 发送一个字符串
void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)
    {
        Serial_SendByte(String[i]);
    }
}


//记得使用微库
#pragma import(__use_no_semihosting)
struct __FILE       { int handle; };         // 标准库需要的支持函数
FILE __stdout;                               // FILE 在stdio.h文件中
void _sys_exit(int x) {    x = x; }          // 定义_sys_exit()以避免使用半主机模式

int fputc(int ch, FILE *f)                   // 重定义fputc函数，使printf函数输出发往fputc内重定向的UART
{
    USART_SendData(USART3,(uint8_t)ch);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);

    return ch;
}

void USART3_UartWrite(uint8_t *buf, uint8_t len)
{
    uint8_t i=0;
    for(i=0; i<len; i++)
    {
      USART_SendData(USART3,buf[i]);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    }
}

//usart3中断接收处理函数
void USART3_IRQHandler(void)
{
    // 如果是接收中断
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        // 读取数据并放入缓冲区
        uint8_t data = USART_ReceiveData(USART3);

        // 防止缓冲区溢出
        if (USART3_RxIndex < 256)
        {
            USART3_RxBuffer[USART3_RxIndex++] = data;
        }
        //清除中断标志位
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }

    //空闲中断
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        // 清除空闲中断标志
        volatile uint16_t temp = USART3->SR;
        temp = USART3->DR;
        (void)temp;//temp没用到,但是写它是为了清除硬件中断标志位

        USART3_RxCount = USART3_RxIndex;//字符数量

        USART3_RxFinished = 1;//标志位置1 , 接收到一帧数据
        USART3_RxIndex = 0;//缓冲区归零
        // 清除中断标志位
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
    }
}
