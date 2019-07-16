#include "usart.h"
#include"DS_Ranging.h"
#define M 3
#define N 8
extern u16 TX_ANT_DLY;//16436
extern u16 RX_ANT_DLY;//
void filter(double *distance,double *dis);
void Setdelay();
char temp=0;
int main()
{
  double IN[3]={0},OUT[3]={0},distance[M][N],dis[3]={0}; 	
	int i,j=0,k=20; 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
	InitDS_R();
  printf("Ranging Setup:\n");//SysTick_Handler
  while (1)
	{		Setdelay();
	 		OUT[0]=get_sys_timestamp_u64()*DWT_TIME_UNITS*1000;
      for(j=0;j<N;j++)
		    for(i=0;i<M;i++)		  				
		      if(!DS_R(i,&distance[i][j],IN,temp)){	
							 --j; break;}									     				

      for(i=0;i<M;i++)
		  {
				printf("\r\nTAG %c\r\n",i+48);
				for(j=0;j<N;j++)
			  printf("%.02f  ",distance[i][j]);
			}
			
			printf("\r\n");
			OUT[1]=get_sys_timestamp_u64()*DWT_TIME_UNITS*1000;
			
			filter(distance[0],dis);
		  OUT[2]=get_sys_timestamp_u64()*DWT_TIME_UNITS*1000;			
      for(i=0;i<3;i++)
         printf("Tag %i dis=%.04f\r\n",i,dis[i]);				 
	    printf("\r\nALL_time=%.04fms,  t1=%.04fms  ,t2=%.04fms  ,t3=%dms\r\n",OUT[2]-OUT[0],OUT[2]-OUT[1],
			        OUT[1]-IN[0],k);
			
	}

}

void filter(double *data,double *dis)
{
	char i,j, k;
	double  temp[3] = {0};

	for (k = 0; k < M; ++k)
	{
		for (j = 0; j < N - 1; ++j)
		  for (i = 0; i<N - j -1; ++i)
		    if (*(data + N*k + i)>*(data + N*k + i + 1)){
			   temp[k] = *(data + N*k + i);
			   *(data + N*k + i) = *(data + N*k + i + 1);
			   *(data + N*k + i + 1) = temp[k];
		      }
		for (i = 1, dis[k] = 0; i < N - 1; ++i)
			dis[k] += *(data + N*k + i);
		dis[k] = dis[k] / (N - 2);
	}	
}


void Setdelay(){
	temp=0;
	if(USART_RX_STA&0x8000)
	{					   
		if(USART_RX_BUF[0]=='0')
		{
		--TX_ANT_DLY;
		--RX_ANT_DLY;
    temp='0';			
		}
		else
		{
		++TX_ANT_DLY;
		++RX_ANT_DLY;	
		temp='1';
		}      				
		USART_RX_STA=0;			
		}
	printf("\r\n当前TX_RX_Delay=%d\r\n",TX_ANT_DLY);	
}