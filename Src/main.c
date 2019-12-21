#include "main.h"
#include "app_rtthread.h"
#include <stdio.h>
#include <stdlib.h>

#include "display_thread.h"

extern SRAM_HandleTypeDef hsram1;


rt_thread_t display_tid;
    

int main(void)
{
    volatile rt_uint32_t *data;
    volatile rt_uint32_t read = 0;
    int count = 1;
    rt_uint32_t i = 0, err_cnt = 0;
    rt_bool_t res = RT_TRUE;
    
    rt_kprintf("main thread started.\r\n");
    while(HAL_SRAM_STATE_READY != HAL_SRAM_GetState(&hsram1));
    
    rt_kprintf("sram test.\r\n");
    rt_kprintf("write sram1.\r\n");
    data = (rt_uint32_t *)0x60000000;
    for (i=0; i<0x100000/4; i++)
    {
        data[i] = 0x60000000 + i*4;
    }
    rt_kprintf("write sram2.\r\n");
    data = (rt_uint32_t *)0x64000000;
    for (i=0; i<0x100000/4; i++)
    {
        data[i] = 0x64000000 + i*4;
    }
    //rt_thread_mdelay(2000);
#if 1
    rt_kprintf("\r\ncheck sram1.\r\n");
    data = (rt_uint32_t *)0x60000000;
    res = RT_TRUE;
    err_cnt = 0;
    for (i=0; i<0x100000/4; i++)
    {
        read = data[i];
        if (read != (0x60000000 + i*4))
        {
            //rt_kprintf("sram1 err, err_addr = 0x%08x. data = 0x%08x\r\n", 0x60000000 + i*4, read);
            res = RT_FALSE;
            err_cnt++;
        }
    }
    if (res == RT_FALSE)
    {
        rt_kprintf("check sram1 failed, err_cnt = %d.\r\n", err_cnt);
    }
    else
    {
        rt_kprintf("check sram1 done.\r\n");
    }
    //rt_thread_mdelay(2000);
#endif
#if 0
    rt_kprintf("\r\ncheck sram2.\r\n");
    data = (rt_uint32_t *)0x64000000;
    res = RT_TRUE;
    err_cnt = 0;
    for (i=0; i<0x100000/4; i++)
    {
        read = data[i];
        if (read != (0x64000000 + i*4))
        {
            //rt_kprintf("sram1 err, err_addr = 0x%08x. data = 0x%08x\r\n", 0x64000000 + i*4, read);
            res = RT_FALSE;
            err_cnt++;
        }
    }
    if (res == RT_FALSE)
    {
        rt_kprintf("check sram2 failed, err_cnt = %d.\r\n", err_cnt);
    }
    else
    {
        rt_kprintf("check sram2 done.\r\n");
    }
    //rt_thread_mdelay(2000);
#endif   

    rt_memset((void *)0x60000000, 0, 0x100000);
    
    display_tid = rt_thread_create("display", display_thread_entry, RT_NULL,
                           1024, 16, 20);
    RT_ASSERT(display_tid != RT_NULL);
    rt_thread_startup(display_tid);
    
    while (count++)
    {   
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6);
        rt_thread_mdelay(500);
    }

    return RT_EOK;
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ 

}
#endif
