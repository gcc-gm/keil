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
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	  uart_init(115200);	 //���ڳ�ʼ��Ϊ115200 
		printf("start up\r\n");
		
	  while(1)
	  {
			deca_sleep(j);
			printf("time is %dms\r\n",(i++)*j);
    }
   
}

