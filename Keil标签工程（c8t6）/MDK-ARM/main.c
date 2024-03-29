/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), and then waits for a "response" message expected from the "DS TWR responder" example
 *           code (companion to this application). When the response is received its RX time-stamp is recorded and we send a "final" message to
 *           complete the exchange. The final message contains all the time-stamps recorded by this application, including the calculated/predicted TX
 *           time-stamp for the final message itself. The companion "DS TWR responder" example application works out the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <string.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "lcd.h"
#include "port.h"
#include "delay.h"

/* LCD屏幕上显示的应用程序名称和版本示例. */
#define APP_NAME "DS TWR INIT v1.1"

/* 测距间延迟时间，以毫秒为单位。 */
#define RNG_DELAY_MS 1000

/* 默认通信配置。我们在这里使用EVK1000的默认模式（模式3）。*/
static dwt_config_t config = {
    2,               /* 频道号码。 */
    DWT_PRF_64M,     /* 脉冲重复频率。 */
    DWT_PLEN_1024,   /* 序言长度。 */
    DWT_PAC32,       /* 序言获取块大小。仅用于RX. */
    9,               /* TX前导码。仅用于TX。 */
    9,               /* RX前导码。仅用于RX。 */
    1,               /* 使用非标准SFD（布尔） */
    DWT_BR_110K,     /* 数据速率。 */
    DWT_PHRMODE_STD, /* PHY头模式。 */
    (1025 + 64 - 32) /* SFD超时（前导码长度+ 1 + SFD长度 -  PAC大小）。仅用于RX。 */
};

/* 64 MHz PRF的默认天线延迟值。见下面的注1。 */
u16 TX_ANT_DLY=16436;//16436
u16 RX_ANT_DLY=16436;

/* 测距过程中使用的帧。见下面的注2。 */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 'H', 'T', '1' , 0x21, 0, 0};//Tap
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', 'H', 0x10, 0x02, 0, 0, 0, 0};//用于核对anchor
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE,'A', 'H', 'T', '1.', 0x23, 'j', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};//Tap最终回应
/* 消息公共部分的长度（包括功能代码，包括下面的注2）。 */
#define ALL_MSG_COMMON_LEN 10
/* 索引以访问上面定义的框架中的某些字段。 */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* 帧序列号，每次传输后递增。 */
static uint8 frame_seq_nb = 0;

/* 缓冲区存储收到的响应消息。
 * 它的大小调整为此示例代码应该处理的最长帧。 */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* 保留状态寄存器状态的副本以供参考，以便读者可以在断点处检查它。 */
static uint32 status_reg = 0;

/* UWB微秒（uus）到器件时间单位（dtu，约15.65 ps）转换因子。
 * 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* 帧之间的延迟，以UWB微秒为单位。见下面的注4。 */
/* 这是从帧传输结束到接收器使能的延迟，如DW1000等待响应功能所设定的。 */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* 这是从帧RX时间戳到TX回复时间戳的延迟，用于计算/设置DW1000的延迟TX功能。这包括
?*具有上述配置的帧长度约为2.66 ms。 */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* 接收响应超时。见下面的注5。 */
#define RESP_RX_TIMEOUT_UUS 2700

/* 帧发送/接收的时间戳，以设备时间单位表示。
 * 由于它们是40位宽，我们需要定义一个64位的int类型来处理它们。 */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;


/* 静态功能声明。 */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int main(void)
{    
	  unsigned int x=0,y=0;//SysTick_Handler
	  static int n=1;
   
	/* 从板特定硬件init开始。 */
    peripherals_init();


    /* 重置并初始化DW1000。
     * 初始化时，DW1000时钟必须暂时设置为晶振速度。初始化后，可以提高SPI速率以获得最佳效果
     * 性能。 */
    reset_DW1000(); /* 将RSTn线的特定驱动器定位到DW1000低电平一段时间。 */
    spi_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_set_rate_high();

    /* 配置DW1000。见下面的注6。 */
    dwt_configure(&config);

    /* 应用默认天线延迟值。见下面的注1。 */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* 设置预期响应的延迟和超时。见下面的注4和5。
     * 由于此示例仅处理一个始终具有相同延迟和超时的传入帧，因此可以在此处为所有值设置这些值。 */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);//2.769ms


    /* 无限循环测距。 */
    while (1)
    {
        /* 将帧数据写入DW1000并准备传输。见下面的注7。 */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* 开始传输，指示预期响应，以便在发送帧和延迟后自动启用接收
         * 由dwt_setrxaftertxdelay（）设置已经过去了。 */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);//直接发送，有等待回应

        /* 我们假设正确地实现了传输，轮询接收帧或错误/超时。见下面的注8。 */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        /*在发送轮询消息之后递增帧序列号（模256）。*/
        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len;

            /* 清除DW1000状态寄存器中发送的良好RX帧事件和TX帧。 */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* 已收到一个帧，将其读入本地缓冲区。 */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* 检查该帧是来自配套“DS TWR响应者”示例的预期响应。
             * 由于帧的序列号字段不相关，因此将其清除以简化帧的验证。*/
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* 检索轮询传输和响应接收时间戳。 */
                poll_tx_ts = get_tx_timestamp_u64();//T1
                resp_rx_ts = get_rx_timestamp_u64();//T4

                /* 计算最终消息传输时间。见下面的注9。 */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /*最终TX时间戳是我们编程的传输时间加上TX天线延迟。 */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* 在最终消息中写入所有时间戳。见下面的注10。 */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);//T1
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);//T4
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);//T5
							  
							   /*如果Tag不为0，则需要通过Anchor传回的sleep_time控制睡眠时间*/
							   if(rx_resp_msg[6]!='0'&&n)
							   { 
									 x=rx_buffer[11];
									 n=0;
								 }
                 
								 if(rx_resp_msg[12]=='0')
                    {
                      --TX_ANT_DLY;  --RX_ANT_DLY;
                    }
                   else if(rx_resp_msg[12]!=0){
                   ++TX_ANT_DLY;  ++RX_ANT_DLY;
                   } 

                /* 编写并发送最终消息。见下面的注7。 */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;//写入序列号
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);//写入最终回应帧
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);//配置发送缓存器
                dwt_starttx(DWT_START_TX_DELAYED);//延时发送，无接收

                /* 轮询DW1000直到TX帧发送事件发送。见下面的注8。 */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };

                /* 清除TXFRS事件。 */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                /* 在发送最终消息之后递增帧序列号（模256）。 */
                frame_seq_nb++;
								if(y!=x)
                {
								    y=x;
								    deca_sleep(x);
								}
								else
								   deca_sleep(95);
            }
        }
        else
        {
            /*清除DW1000状态寄存器中的RX错误事件。 */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR); 
					  n=1;
					  deca_sleep((rx_resp_msg[6]-48)*27+17);
					
        }
        /* 在测距交换之间执行延迟。 */								         				
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief 获取64位变量中的TX时间戳。
 *        /!\ 对于TX和RX，此函数假定时间戳的长度为40位！
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief 获取64位变量的RX时间戳。
 *        /!\ 对于TX和RX，此函数假定时间戳的长度为40位！
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief 使用给定值填充最终消息中的给定时间戳字段。在最终的时间戳字段中
 *        消息，最低有效字节位于较低地址。
 *
 * @param  ts_field指针在要填充的时间戳字段的第一个字节上
 *         ts时间戳值
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    110k data rate used (around 3 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
 *    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *    8 bits.
 * 10. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *     time-of-flight computation) can be handled by a 32-bit subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
