#include "main.h"
#include "app_rtthread.h"

int main(void)
{
    int count = 1;
    rt_kprintf("main thread started.\r\n");
    
    while (count++)
    {
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_6);
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{ 

}
#endif
