/*
 * Copyright (c) 2006-2019, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-07-24     Tanek        the first version
 * 2018-11-12     Ernest Chen  modify copyright
 */
 
#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#include "gd32f10x.h"

#define _SCB_BASE       (0xE000E010UL)
#define _SYSTICK_CTRL   (*(rt_uint32_t *)(_SCB_BASE + 0x0))
#define _SYSTICK_LOAD   (*(rt_uint32_t *)(_SCB_BASE + 0x4))
#define _SYSTICK_VAL    (*(rt_uint32_t *)(_SCB_BASE + 0x8))
#define _SYSTICK_CALIB  (*(rt_uint32_t *)(_SCB_BASE + 0xC))
#define _SYSTICK_PRI    (*(rt_uint8_t  *)(0xE000ED23UL))

// Updates the variable SystemCoreClock and must be called 
// whenever the core clock is changed during program execution.
extern void SystemCoreClockUpdate(void);

// Holds the system core clock, which is the system clock 
// frequency supplied to the SysTick timer and the processor 
// core clock.
extern uint32_t SystemCoreClock;

static uint32_t _SysTick_Config(rt_uint32_t ticks)
{
    if ((ticks - 1) > 0xFFFFFF)
    {
        return 1;
    }
    
    _SYSTICK_LOAD = ticks - 1; 
    _SYSTICK_PRI = 0xFF;
    _SYSTICK_VAL  = 0;
    _SYSTICK_CTRL = 0x07;  
    
    return 0;
}

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
#define RT_HEAP_SIZE 4096
static uint32_t rt_heap[RT_HEAP_SIZE];     // heap default size: 4K(1024 * 4)
RT_WEAK void *rt_heap_begin_get(void)
{
    return rt_heap;
}

RT_WEAK void *rt_heap_end_get(void)
{
    return rt_heap + RT_HEAP_SIZE;
}
#endif

/**
 * This function will initial your board.
 */
void rt_hw_board_init()
{
    /* System Clock Update */
    SystemCoreClockUpdate();
    
    /* System Tick Configuration */
    _SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* Call components board initial (use INIT_BOARD_EXPORT()) */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

#if defined(RT_USING_USER_MAIN) && defined(RT_USING_HEAP)
    rt_system_heap_init(rt_heap_begin_get(), rt_heap_end_get());
#endif
}

void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}


static int uart_init(void)
{
    /* enable GPIO clock 使能串口1引脚IO时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200);            //设置串口波特率
    usart_word_length_set(USART0, USART_WL_8BIT); //字长为8位数据格式
    usart_stop_bit_set(USART0, USART_STB_1BIT);   //一个停止位
    usart_parity_config(USART0, USART_PM_NONE);    //无奇偶校验位
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE); //无硬件数据流控制
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE); //无硬件数据流控制
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);        //收发模式
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);                          //使能串口0 
    
    		
    /* USART interrupt configuration */
    nvic_irq_enable(USART0_IRQn, 0, 1);             //配置串口中断优先级

    /* enable USART TBE interrupt */  
    usart_interrupt_enable(USART0, USART_INT_RBNE); //使能串口中断  : 读取数据缓冲区非空中断和溢出错误中断
    
    return 0;
}
INIT_BOARD_EXPORT(uart_init);  /* 默认选择初始化方法一：使用宏 INIT_BOARD_EXPORT 进行自动初始化 */

void rt_hw_console_output( const char *str )
{
	rt_size_t i = 0, size = 0;
    char a = '\r';
    
    size = rt_strlen(str);

    for (i = 0; i < size; i++)
    {
        if (*(str + i) == '\n')
        {
            usart_data_transmit(USART0, (uint32_t )a);
            while((usart_flag_get(USART0, USART_FLAG_TC) == RESET));
        }
        usart_data_transmit(USART0, (uint32_t)*(str + i));
        while((usart_flag_get(USART0, USART_FLAG_TC) == RESET));
    }
}

//用于finish输入
char rt_hw_console_getchar(void)
{
    /* the initial value of ch must < 0 */
    int ch = -1;

    if (usart_flag_get(USART0, USART_FLAG_RBNE) != RESET)
    {
        ch = usart_data_receive(USART0);
    }
    else
    {
        rt_thread_mdelay(10);
    }
    return ch;	
}
