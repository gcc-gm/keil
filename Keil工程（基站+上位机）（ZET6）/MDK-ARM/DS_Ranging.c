#include"DS_Ranging.h"

u16 TX_ANT_DLY=16436;//16436
u16 RX_ANT_DLY=16436;//
char DS_R(char i,double *distance,double *IN,char temp)
{
        /*清除接收超时以开始下一个测距过程。 */
        dwt_setrxtimeout(0);//禁用超时
	 
	      /*更改目标Tag地址，实现对不同Tag的测距*/ 
	      rx_poll_msg[ALL_MSG_SOURECE_ADD]=i+48;
	      tx_resp_msg[ALL_MSG_AIM_ADD]=i+48;
	      rx_final_msg[ALL_MSG_SOURECE_ADD]=i+48;
	      tx_resp_msg[12]=temp;
	
        
        /* 立即激活接收。(能接收到完整帧、错误帧，或者或超时)*/
        dwt_rxenable(0);//
        
        /* 轮询接收帧或错误/超时。见下面的注7。*/
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };
    
				if (status_reg & SYS_STATUS_RXFCG)/*判断接收状态是否正常*/
        {
            uint32 frame_len;

            /* 在DW1000状态寄存器中清除接收到完整的RX帧事件。*/
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            /*已收到一个帧，将其读入本地缓冲区。*/
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* 检查该帧是否是“目标标签”示例发送的轮询。
             *由于帧的序列号字段不相关，因此将其清除以简化帧的验证。*/
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 resp_tx_time;

                /*检索轮询接收时间戳。*/
                poll_rx_ts = get_rx_timestamp_u64();//T2
							  IN[0]=poll_rx_ts*DWT_TIME_UNITS*1000;//T2

                /*设置响应的发送时间。见下面的注8。*/
                resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;//？
                dwt_setdelayedtrxtime(resp_tx_time);//设置延迟传输时间

                /*设置最终消息接收的预期延迟和超时。*/
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);//设置发送帧后 延迟UUS 再开启接收
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);//设置接收等待超时
							
							   /*指定当前对话Tag的睡眠时间，通过tx_resp_msg[]传输给对方*/
							   if(i!=0)
								 {
									   
									   IN[2]=IN[1];
								     tx_resp_msg[Tag_timechip_Coltrol]=(105-(char)(IN[0]-IN[1]))%100;
								 
								 }
														

                /* 编写并发送响应消息。见下面的注9。*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;//设置帧序列号
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);//响应帧写入
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0);//配置发送帧寄存器，输入传输帧长度
                dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);//延时发送，有接收回应

                /*假设正确地实现了传输，现在轮询接收预期的“最终”帧或错误/超时。
                 *见下面的注7。*/
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };
                /*在发送响应消息之后递增帧序列号（模256）。*/
                frame_seq_nb++;

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /*清除DW1000状态寄存器中发送的良好RX帧事件和TX帧。 */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                    /*已收到一个帧，将其读入本地缓冲区*/
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    /*检查该帧是否是“对话标签”示例发送的最终消息。
                     *由于在此示例中未使用帧的序列号字段，因此可将其归零以简化帧的验证。*/
                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                        uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64 tof_dtu;

                        /*检索响应传输和最终接收时间戳。*/
                        
											  resp_tx_ts = get_tx_timestamp_u64();//T3
                        final_rx_ts = get_rx_timestamp_u64();//T6
                        IN[1]=final_rx_ts*DWT_TIME_UNITS*1000;											

                        /*获取嵌入在最终消息中的时间戳。*/
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);//T1
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);//T4
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);//T5

                        /*计算飞行时间。即使时钟已经包裹，32位减法也能给出正确的答案。见下面的注10。*/
                        poll_rx_ts_32 = (uint32)poll_rx_ts;//t2
                        resp_tx_ts_32 = (uint32)resp_tx_ts;//t3
                        final_rx_ts_32 = (uint32)final_rx_ts;//t6
                        Ra = (double)(resp_rx_ts - poll_tx_ts);//Tround1 = T4 - T1
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);//Tround2 = T6 - T3
                        Da = (double)(final_tx_ts - resp_rx_ts);//Treply1 = T3 - T2
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);//Treply2 = T5 - T4
                        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        *distance = tof * SPEED_OF_LIGHT;
												
                        return 1;

                    }
                }
                else
                {
                    /*接收错误，清除DW1000状态寄存器中的RX错误事件。*/
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
        }
        else
        {
            /* 接收错误，清除DW1000状态寄存器中的RX错误事件。*/
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }  
				return 0;				
}

void InitDS_R(void)
{
    /* Start with board specific hardware init. 开始特定硬件初始化*/
    peripherals_init();

    /* Display application name on LCD. */
    //lcd_display_str(APP_NAME);

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. 
	    重置并初始化DW1000。
     *初始化时，DW1000时钟必须暂时设置为晶振速度。初始化后，可以提高SPI速率以获得最佳效果
     *表现。*/
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. 将RSTn线的特定驱动器定位到DW1000低电平一段时间。*/
    spi_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_set_rate_high();

    /* Configure DW1000. See NOTE 6 below. 配置DW1000。见下面的注6。*/
    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 1 below. 应用默认天线延迟值。见下面的注1。*/
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
          获取64位变量中的TX时间戳。
 *        /！\对于TX和RX，此函数假定时间戳的长度为40位！
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.  读取时间戳的64位值。
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
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 * @brief在64位变量中获取RX时间戳。
 *        /！\对于TX和RX，此函数假定时间戳的长度为40位！
 * @param  none
 *
 * @return  64-bit value of the read time-stamp. 读取时间戳的64位值。
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


uint64 get_sys_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readsystime(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 * @brief从最终消息中读取给定的时间戳值。在最终消息的时间戳字段中，最少
 *       有效字节位于较低地址。
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
           ts_field指针在要读取的时间戳字段的第一个字节上
 *         ts时间戳值
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
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
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
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
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 10. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/


