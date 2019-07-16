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

/* LCD��Ļ����ʾ��Ӧ�ó������ƺͰ汾ʾ��. */
#define APP_NAME "DS TWR INIT v1.1"

/* ��Χ���ӳ�ʱ�䣬�Ժ���Ϊ��λ�� */
#define RNG_DELAY_MS 1000

/* Ĭ��ͨ�����á�����������ʹ��EVK1000��Ĭ��ģʽ��ģʽ3����*/
static dwt_config_t config = {
    2,               /* Ƶ�����롣 */
    DWT_PRF_64M,     /* �����ظ�Ƶ�ʡ� */
    DWT_PLEN_1024,   /* ���Գ��ȡ� */
    DWT_PAC32,       /* ���Ի�ȡ���С��������RX. */
    9,               /* TXǰ���롣������TX�� */
    9,               /* RXǰ���롣������RX�� */
    1,               /* ʹ�÷Ǳ�׼SFD�������� */
    DWT_BR_110K,     /* �������ʡ� */
    DWT_PHRMODE_STD, /* PHYͷģʽ�� */
    (1025 + 64 - 32) /* SFD��ʱ��ǰ���볤��+ 1 + SFD���� -  PAC��С����������RX�� */
};

/* 64 MHz PRF��Ĭ�������ӳ�ֵ���������ע1�� */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* ��������ʹ�õ�֡���������ע2�� */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* ��Ϣ�������ֵĳ��ȣ��������ܴ��룬���������ע2���� */
#define ALL_MSG_COMMON_LEN 10
/* �����Է������涨��Ŀ���е�ĳЩ�ֶΡ� */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* ֡���кţ�ÿ�δ��������� */
static uint8 frame_seq_nb = 0;

/* �������洢�յ�����Ӧ��Ϣ��
 * ���Ĵ�С����Ϊ��ʾ������Ӧ�ô�����֡�� */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* ����״̬�Ĵ���״̬�ĸ����Թ��ο����Ա���߿����ڶϵ㴦������� */
static uint32 status_reg = 0;

/* UWB΢�루uus��������ʱ�䵥λ��dtu��Լ15.65 ps��ת�����ӡ�
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* ֮֡����ӳ٣���UWB΢��Ϊ��λ���������ע4�� */
/* ���Ǵ�֡���������������ʹ�ܵ��ӳ٣���DW1000�ȴ���Ӧ�������趨�ġ� */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* ���Ǵ�֡RXʱ�����TX�ظ�ʱ������ӳ٣����ڼ���/����DW1000���ӳ�TX���ܡ������
?*�����������õ�֡����ԼΪ2.66 ms�� */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* ������Ӧ��ʱ���������ע5�� */
#define RESP_RX_TIMEOUT_UUS 2700

/* ֡����/���յ�ʱ��������豸ʱ�䵥λ��ʾ��
 * ����������40λ��������Ҫ����һ��64λ��int�������������ǡ� */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* ��̬���������� */
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
    /* �Ӱ��ض�Ӳ��init��ʼ�� */
    peripherals_init();

    /* Display application name on LCD. */
    //lcd_display_str(APP_NAME);

    /* ���ò���ʼ��DW1000��
     * ��ʼ��ʱ��DW1000ʱ�ӱ�����ʱ����Ϊ�����ٶȡ���ʼ���󣬿������SPI�����Ի�����Ч��
     * ���ܡ� */
    reset_DW1000(); /* ��RSTn�ߵ��ض���������λ��DW1000�͵�ƽһ��ʱ�䡣 */
    spi_set_rate_low();
    dwt_initialise(DWT_LOADUCODE);
    spi_set_rate_high();

    /* ����DW1000���������ע6�� */
    dwt_configure(&config);

    /* Ӧ��Ĭ�������ӳ�ֵ���������ע1�� */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* ����Ԥ����Ӧ���ӳٺͳ�ʱ���������ע4��5��
     * ���ڴ�ʾ��������һ��ʼ�վ�����ͬ�ӳٺͳ�ʱ�Ĵ���֡����˿����ڴ˴�Ϊ����ֵ������Щֵ�� */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    /* ѭ����Զ������ཻ���� */
    while (1)
    {
        /* ��֡����д��DW1000��׼�����䡣�������ע7�� */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* ��ʼ���䣬ָʾԤ����Ӧ���Ա��ڷ���֡���ӳٺ��Զ����ý���
         * ��dwt_setrxaftertxdelay���������Ѿ���ȥ�ˡ� */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* ���Ǽ�����ȷ��ʵ���˴��䣬��ѯ����֡�����/��ʱ���������ע8�� */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        /*�ڷ�����ѯ��Ϣ֮�����֡���кţ�ģ256����*/
        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len;

            /* ���DW1000״̬�Ĵ����з��͵�����RX֡�¼���TX֡�� */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* ���յ�һ��֡��������뱾�ػ������� */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* ����֡���������ס�DS TWR��Ӧ�ߡ�ʾ����Ԥ����Ӧ��
             * ����֡�����к��ֶβ���أ���˽�������Լ�֡����֤��*/
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* ������ѯ�������Ӧ����ʱ����� */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* ����������Ϣ����ʱ�䡣�������ע9�� */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /*����TXʱ��������Ǳ�̵Ĵ���ʱ�����TX�����ӳ١� */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* ��������Ϣ��д������ʱ������������ע10�� */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* ��д������������Ϣ���������ע7�� */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);
                dwt_starttx(DWT_START_TX_DELAYED);

                /* ��ѯDW1000ֱ��TX֡�����¼������������ע8�� */
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };

                /* ���TXFRS�¼��� */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

                /* �ڷ���������Ϣ֮�����֡���кţ�ģ256���� */
                frame_seq_nb++;
            }
        }
        else
        {
            /*���DW1000״̬�Ĵ����е�RX�����¼��� */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }

        /* �ڲ�ཻ��֮��ִ���ӳ١� */
        deca_sleep(RNG_DELAY_MS);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief ��ȡ64λ�����е�TXʱ�����
 *        /!\ ����TX��RX���˺����ٶ�ʱ����ĳ���Ϊ40λ��
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
 * @brief ��ȡ64λ������RXʱ�����
 *        /!\ ����TX��RX���˺����ٶ�ʱ����ĳ���Ϊ40λ��
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
 * @brief ʹ�ø���ֵ���������Ϣ�еĸ���ʱ����ֶΡ������յ�ʱ����ֶ���
 *        ��Ϣ�������Ч�ֽ�λ�ڽϵ͵�ַ��
 *
 * @param  ts_fieldָ����Ҫ����ʱ����ֶεĵ�һ���ֽ���
 *         tsʱ���ֵ
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
