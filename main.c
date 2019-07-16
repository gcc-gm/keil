#include <stdio.h>
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "usart.h"

int main(void)
{
    int i=1,j=500;
	
	  /* Start with board specific hardware init. */
    peripherals_init();
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	  uart_init(115200);	 //串口初始化为115200 
		printf("start up\r\n");
		
	  while(1)
	  {
			deca_sleep(j);
			printf("time is %dms\r\n",(i++)*j);
    }
   
}

