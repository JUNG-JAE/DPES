

#include <stdio.h>
//#include <stdlib.h>
#include <string.h>		// memcpy 등
//#include <getopt.h>
//#include <time.h>
//#include <sys/types.h>
//#include <sys/stat.h>
#include <fcntl.h> 		//  open( ... O_RDWR) 등
//#include <errno.h>
//#include <limits.h>
//#include <stdbool.h> 
#include <unistd.h>		// usleep 등
//#include <sys/time.h>
//#include <signal.h>
#include <sys/time.h>	// gettimeofday 등
#include <pthread.h>	// pthread_exit 등
//#include <sys/socket.h>
#include <termios.h>	// struct termios 등
//#include <sys/signal.h>
//#include <stdint.h>

//#define DISABLE_EXTERN_LORACOMM_H
#include "extern.h"



//===================================================================
// 
//===================================================================
static const char *loracomm_dev_port[LORACOMM_M_NUM] = {
	//"/dev/ttyUSB0", 
	(const char *)"/dev/ttyXRUSB1", 
	(const char *)"/dev/ttyXRUSB3" 
};



static int loracomm_fd[LORACOMM_M_NUM] = { 
	-1, 
	-1 
};









//===================================================================
//	CRC16 Funcs						    			 
//===================================================================

static uint16_t Generate_CRC(const unsigned char *bufPtr, int length)
{

#define CRC_16_POLYNOMIAL 	0x1021U
#define CRC_16_SEED 		0x0000
//#define CRC_16_STEP_SEED 	(~((word) CRC_16_SEED))
#define CRC_TAB_SIZE 		256

	const uint16_t Crc16_Table [CRC_TAB_SIZE] = 
	{
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
		0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
		0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
		0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
		0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
		0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
		0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
		0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
		0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
		0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
		0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
		0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
		0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
		0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
		0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
		0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
		0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
		0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
		0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
		0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
	};

	uint16_t crc_16 = CRC_16_SEED;
	
	const unsigned char *buf_ptr = bufPtr;
	int len = 0;
	
	

	for(len=length*8; len>=8; len-=8) 
	{
		crc_16 = Crc16_Table[ (crc_16 >> 8) ^ *buf_ptr ] ^ (crc_16 << 8);
		buf_ptr++;
	}
	
	if(len != 0)
	{
		uint16_t data = ((uint16_t) (*buf_ptr)) << (8);
		while(len-- != 0) 
		{
			if( ((crc_16 ^ data) & ((uint16_t)1 << 15U)) != 0U ) 
			{
				crc_16 <<= 1;
				crc_16 ^= CRC_16_POLYNOMIAL;
			}
			else 
			{
				crc_16 <<= 1;
			}
			data <<= 1;
		}
	}
	
	return (crc_16);
}


//===================================================================
//	LORACOMM TX Funcs
//===================================================================

#define LORACOMM_TXBF_SIZE	1024U

static char loracomm_txbf[LORACOMM_M_NUM][LORACOMM_TXBF_SIZE];

static uint32_t loracomm_txbf_wrp[LORACOMM_M_NUM] = {0, 0};
static uint32_t loracomm_txbf_rdp[LORACOMM_M_NUM] = {0, 0};

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
static int loracomm_txbf_read(unsigned char loracomm_num, char *ch) 
{
	int ret = -1;
	if(loracomm_txbf_wrp[loracomm_num] != loracomm_txbf_rdp[loracomm_num]) {
		*ch = loracomm_txbf[loracomm_num][loracomm_txbf_rdp[loracomm_num]];
		loracomm_txbf_rdp[loracomm_num]++;
		loracomm_txbf_rdp[loracomm_num] %= LORACOMM_TXBF_SIZE;
		ret = 0;
	}
	return ret;
}

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
static int loracomm_txbf_write(unsigned char loracomm_num, char ch) 
{
	uint32_t loracomm_txbf_wrp_tmp = (loracomm_txbf_wrp[loracomm_num]+1U) % LORACOMM_TXBF_SIZE;
	int ret = -1;
	
	if( loracomm_txbf_rdp[loracomm_num] != loracomm_txbf_wrp_tmp ) 
	{
		loracomm_txbf[loracomm_num][loracomm_txbf_wrp[loracomm_num]] = ch;
		loracomm_txbf_wrp[loracomm_num] = loracomm_txbf_wrp_tmp;
		ret = 0;
	} 
	return ret;
}


//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
static void loracomm_tx_frame(int loracomm_num, unsigned char src, unsigned char dst, 
					unsigned char cmd, const unsigned char *sub_data, int len)
{
	unsigned char tx_data[512];
	uint16_t tx_len = 0;
	int crc_len = 0;
	int crc_start_pos = 0;
	uint16_t crc;
	uint16_t i;
	
	if(loracomm_num >= (int)LORACOMM_M_NUM) {
		unused = printf("%s: loracomm_num[%d] range_error !!!\n", __func__, loracomm_num);
	}
	else {
		
		if(DB_temp->loracomm_stat[loracomm_num].error == 1U) {
			//printf("%s: loracomm_num[%d] stat_error !!!\n", __func__, loracomm_num);
		}
		else {
				
			if(dbgdisp_status[DBGDISP_LORACOMM_DUMP] == 1) {
				unused = printf("\%s: loracomm_tx[%d] Src:%02x, Dst:%02x, Cmd:%02x, Len:%02x\n",
								__func__, loracomm_num, src, dst, cmd, len);
			}

//			tx_data[tx_len] = LORACOMM_FRAME_STX;
//			tx_len++;
//			crc_start_pos++;

//			tx_data[tx_len] = LORACOMM_FRAME_STX;
//			tx_len++;
//			crc_start_pos++;

//			tx_data[tx_len] = LORACOMM_FRAME_STX;
//			tx_len++;
//			crc_start_pos++;

			tx_data[tx_len] = LORACOMM_FRAME_STX;
			tx_len++;
			crc_start_pos++;

			tx_data[tx_len] = src;
			tx_len++;
			crc_len++;
			
			tx_data[tx_len] = dst;
			tx_len++;
			crc_len++;
			
			tx_data[tx_len] = cmd;
			tx_len++;
			crc_len++;
			
			tx_data[tx_len] = (unsigned char)(((unsigned int)len   ) & 0x000000FFU);
			tx_len++;
			crc_len++;
			
			tx_data[tx_len] = (unsigned char)(((unsigned int)len>>8) & 0x000000FFU);
			tx_len++;
			crc_len++;

			for(i=0U; i<(unsigned int)len; i++) {
				tx_data[tx_len] = sub_data[i];
				tx_len++;
				crc_len++;
			}
			
			crc = Generate_CRC((unsigned char *)&tx_data[crc_start_pos], crc_len);
		//	printf("loracomm_tx_frame: crc_start_pos[%d], crc_len[%d], crc[0x%04x]\n", crc_start_pos, crc_len, crc);

			tx_data[tx_len] = (unsigned char)(crc & 0x00FFU);
			tx_len++;
			tx_data[tx_len] = (unsigned char)((unsigned char)(crc>>8) & (unsigned char)0xFFU);
			tx_len++;
			
			tx_data[tx_len] = LORACOMM_FRAME_ETX;
			tx_len++;

			for(i=0U; i<tx_len; i++) {
				unused = loracomm_txbf_write((unsigned char)loracomm_num, (char)tx_data[i]);
			}

			DB_temp->loracomm_stat[loracomm_num].tx_frame_cnt++;
		}
	}
}

//===================================================================
//	LORA RX Funcs
//===================================================================

static loracomm_rx_frame_t	loracommRxFrame[LORACOMM_M_NUM];

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
static void loracomm_rx_frame_init(unsigned char loracomm_num)
{
	if(loracomm_num >= (unsigned char)LORACOMM_M_NUM) {
		unused = printf("%s: loracomm_num[%d] range_error !!!\n", __func__, loracomm_num);
	}
	else {
		int i;
		for(i=0; i<LORACOMM_FRAME_MAX_LEN; i++) { // 256
			loracommRxFrame[loracomm_num].frame.D[i] = 0;
		}
		
		loracommRxFrame[loracomm_num].LEN = 0;
		loracommRxFrame[loracomm_num].CRC = 0;
		loracommRxFrame[loracomm_num].DataCnt = 0;
		
		loracommRxFrame[loracomm_num].State = LORACOMM_STATE_STX;
	}
}

	
//--------------------------------------------------------
//
//--------------------------------------------------------
static void loracomm_rx_frame_analyse(int loracomm_num)
{
	if(dbgdisp_status[DBGDISP_LORACOMM_DUMP] == 1) {
		unused = printf("%s[%d]: src[0x%02x],dst[0x%02x],cmd[0x%02x],len[%d],\n",
				__func__,
				loracomm_num,
				loracommRxFrame[loracomm_num].frame.F.Src,
				loracommRxFrame[loracomm_num].frame.F.Dst,
				loracommRxFrame[loracomm_num].frame.F.Cmd,
				loracommRxFrame[loracomm_num].LEN
				);
	}

	if(loracommRxFrame[loracomm_num].frame.F.Src != LORACOMM_FRAME_DST_ADDR) {
		unused = printf("%s[%d]: src[%d] error!!!, RemotePC = %u\n", __func__,
				loracomm_num, loracommRxFrame[loracomm_num].frame.F.Src, LORACOMM_FRAME_DST_ADDR);
	}

	if(loracommRxFrame[loracomm_num].frame.F.Dst != LORACOMM_FRAME_SRC_ADDR) {
		unused = printf("%s[%d]: dst[%d] error!!!, DronePilotTest = %u\n", __func__,
				loracomm_num, loracommRxFrame[loracomm_num].frame.F.Dst, LORACOMM_FRAME_SRC_ADDR);
	}

	if(loracommRxFrame[loracomm_num].frame.F.Cmd == LORACOMM_FRAME_CMD_STAT) {
		unused = printf("%s[%d]: LORACOMM_FRAME_CMD_STAT\n", __func__, loracomm_num);

	}
	else if(loracommRxFrame[loracomm_num].frame.F.Cmd == LORACOMM_FRAME_CMD_CTRL) {
		unused = printf("%s[%d]: LORACOMM_FRAME_CMD_CTRL\n", __func__, loracomm_num);
		//-----------------------------------------------
		const hbt_system_setup_t *recv_data;
#if 0
		recv_data = (hbt_system_setup_t *)loracommRxFrame[loracomm_num].frame.F.Dat;		
#else
		recv_data = (hbt_system_setup_t *)&loracommRxFrame[loracomm_num].frame.F.Dat[0];
#endif	
		
		unused = printf("%s: ...\r\n", __func__);
		
		if(recv_data->lora_pwr != DB_bkup->lora_pwr) {
			DB_bkup->lora_pwr = recv_data->lora_pwr;
		}
		
		if(recv_data->angle_init == (char)1) {
			wt61c_setting_angle_init();
		}
		
		unused = db_bkup_to_file();

		//-----------------------------------------------
	}
	else {
		unused = printf("%s[%d]: cmd[%d] error!!!\n", __func__, loracomm_num, loracommRxFrame[loracomm_num].frame.F.Cmd);
		DB_temp->loracomm_stat[loracomm_num].rx_frame_anal_err_cnt = 1;
	}

}


#if LORACOMM_SUB_THREAD_ENABLE

unsigned int loracomm_sub_rx_packet_loss_cnt = 0;

//--------------------------------------------------------
//
//--------------------------------------------------------
static void loracomm_rx_frame_analyse_sub(int loracomm_num)
{
	static unsigned int loracomm_sub_rx_pkt_cnt_prev = 0;	

	if(dbgdisp_status[DBGDISP_LORACOMM_DUMP] == 1) {
		unused = printf("%s[%d]: src[0x%02x],dst[0x%02x],cmd[0x%02x],len[%d],\n",
				__func__,
				loracomm_num,
				loracommRxFrame[loracomm_num].frame.F.Src,
				loracommRxFrame[loracomm_num].frame.F.Dst,
				loracommRxFrame[loracomm_num].frame.F.Cmd,
				loracommRxFrame[loracomm_num].LEN
				);
	}

	if(loracommRxFrame[loracomm_num].frame.F.Src != LORACOMM_FRAME_SRC_ADDR) {
		unused = printf("%s[%d]: src[%d] error!!!, DronePilotTest = %u\n", __func__,
				loracomm_num, loracommRxFrame[loracomm_num].frame.F.Src, LORACOMM_FRAME_SRC_ADDR);
	}

	if(loracommRxFrame[loracomm_num].frame.F.Dst != LORACOMM_FRAME_DST_ADDR) {
		unused = printf("%s[%d]: dst[%d] error!!!, RemotePC = %u\n", __func__,
				loracomm_num, loracommRxFrame[loracomm_num].frame.F.Dst, LORACOMM_FRAME_DST_ADDR);
	}

	if(loracommRxFrame[loracomm_num].frame.F.Cmd == LORACOMM_FRAME_CMD_STAT) {
		unused = printf("%s[%d]: LORACOMM_FRAME_CMD_STAT\n", __func__, loracomm_num);
		//-----------------------------------------------
		loracomm_heartbit_req_t loracomm_heartbit_test;

		vp_unused = memcpy(	(unsigned char *)&loracomm_heartbit_test, 
				&loracommRxFrame[loracomm_num].frame.F.Dat[0], 
				sizeof(loracomm_heartbit_req_t));
				
		if((loracomm_sub_rx_pkt_cnt_prev+1U) != loracomm_heartbit_test.pkt_cnt) {
			if((loracomm_sub_rx_pkt_cnt_prev==0U) && (loracomm_heartbit_test.pkt_cnt==0U)) {
				unused = printf("%s: ================first_cnt[%u]\n", __func__, loracomm_sub_rx_pkt_cnt_prev);
			}
			else {
				unused = printf("%s: ----------------prev_cnt[%u]\n", __func__, loracomm_sub_rx_pkt_cnt_prev);
				loracomm_sub_rx_packet_loss_cnt++;
			}
		}
		loracomm_sub_rx_pkt_cnt_prev = loracomm_heartbit_test.pkt_cnt;

		//-------------------------------------------------------------
		static unsigned int pkt_cnt_prev = 0; 
		static int first = 1;
		static unsigned int total_loss_cnt = 0;
		unsigned int loss_cnt = 0;
		
		if(first == 1) {
			first = 0;
		}
		else { 
			if(loracomm_heartbit_test.pkt_cnt > pkt_cnt_prev) {
				loss_cnt = loracomm_heartbit_test.pkt_cnt - (pkt_cnt_prev+1U);
			}
			else{
				if(loracomm_heartbit_test.pkt_cnt < pkt_cnt_prev) {
					loss_cnt = (0xffffffffU-pkt_cnt_prev) + loracomm_heartbit_test.pkt_cnt + 1U;
				}
			} 

		}	
		pkt_cnt_prev = loracomm_heartbit_test.pkt_cnt;
		
		total_loss_cnt += loss_cnt;
		
		//if(loss_cnt != 0) {
		//	plog_error("%s: loss_cnt = %d, total_loss_cnt = %d", __func__, loss_cnt, total_loss_cnt);
		//}
		
		unused = printf("%s----------------\n", __func__);
		unused = printf("pkt_cnt       = %u   // c_loss[%u], t_loss[%u]\n", loracomm_heartbit_test.pkt_cnt, loss_cnt, total_loss_cnt);
		//
		unused = printf("date_time     = %02d-%02d-%02d %02d:%02d:%02d\n",
				loracomm_heartbit_test.sys_stat.date_time.year,
				loracomm_heartbit_test.sys_stat.date_time.month,
				loracomm_heartbit_test.sys_stat.date_time.day,
				loracomm_heartbit_test.sys_stat.date_time.hour,
				loracomm_heartbit_test.sys_stat.date_time.minute,
				loracomm_heartbit_test.sys_stat.date_time.second);
		//
		unused = printf("gps.latitude  = %f\n", loracomm_heartbit_test.sys_stat.gps.latitude);
		unused = printf("gps.longitude = %f\n", loracomm_heartbit_test.sys_stat.gps.longitude);
		unused = printf("gps.altitude  = %f\n", loracomm_heartbit_test.sys_stat.gps.altitude);
		unused = printf("gps.time      = %02d-%02d-%02d %02d:%02d:%02d\n",
				loracomm_heartbit_test.sys_stat.gps.time.year,
				loracomm_heartbit_test.sys_stat.gps.time.month,
				loracomm_heartbit_test.sys_stat.gps.time.day,
				loracomm_heartbit_test.sys_stat.gps.time.hour,
				loracomm_heartbit_test.sys_stat.gps.time.minute,
				loracomm_heartbit_test.sys_stat.gps.time.second);
		unused = printf("gps.status    = %d\n", loracomm_heartbit_test.sys_stat.gps.status);
		//
		
#if 1 //2022.3.22
		unused = printf("yaw           = %f\n", loracomm_heartbit_test.sys_stat.yaw);			
#else	
		unused = printf("motion.yaw    = %f\n", loracomm_heartbit_test.sys_stat.motion.yaw);			
		unused = printf("motion.pitch  = %f\n", loracomm_heartbit_test.sys_stat.motion.pitch);
		unused = printf("motion.roll   = %f\n", loracomm_heartbit_test.sys_stat.motion.roll);
		//
		unused = printf("motion.m[0]   = %f\n", loracomm_heartbit_test.sys_stat.motion.m[0]);
		unused = printf("motion.m[1]   = %f\n", loracomm_heartbit_test.sys_stat.motion.m[1]);
		unused = printf("motion.m[2]   = %f\n", loracomm_heartbit_test.sys_stat.motion.m[2]);
		unused = printf("motion.h      = %f\n", loracomm_heartbit_test.sys_stat.motion.h);
#endif		
		//
		unused = printf("batt_status   = %d\n", loracomm_heartbit_test.sys_stat.batt_status);
		unused = printf("batt_percent  = %d\n", loracomm_heartbit_test.sys_stat.batt_percent);

		//-------------------------------------------------------------
	}
	else if(loracommRxFrame[loracomm_num].frame.F.Cmd == LORACOMM_FRAME_CMD_CTRL) {
		unused = printf("%s[%d]: LORACOMM_FRAME_CMD_CTRL\n", __func__, loracomm_num);

	}
	else {
		unused = printf("%s[%d]: cmd[%d] error!!!\n", __func__, loracomm_num, loracommRxFrame[loracomm_num].frame.F.Cmd);
		DB_temp->loracomm_stat[loracomm_num].rx_frame_anal_err_cnt = 1;
	}

}
#endif


//--------------------------------------------------------
//
//--------------------------------------------------------
static void loracomm_rx_frame_process(unsigned char loracomm_num, unsigned char ch)
{
//	printf("[%d:%02x] ", loracomm_num, ch);
	
	switch(loracommRxFrame[loracomm_num].State) 
	{
		case LORACOMM_STATE_STX:
			if(ch == LORACOMM_FRAME_STX)
			{
//	printf("stx\n");
				loracommRxFrame[loracomm_num].State = LORACOMM_STATE_SRC;
			}
			else {
				loracomm_rx_frame_init(loracomm_num);
			}
			break;
		
		case LORACOMM_STATE_SRC:
			if(ch == LORACOMM_FRAME_STX)
			{
				break;
			}
//	printf("src\n");
			loracommRxFrame[loracomm_num].frame.D[loracommRxFrame[loracomm_num].DataCnt] = ch;
			loracommRxFrame[loracomm_num].DataCnt++;
			loracommRxFrame[loracomm_num].State++;
			break;
			
		case LORACOMM_STATE_DST:
//	printf("dst\n");		
			loracommRxFrame[loracomm_num].frame.D[loracommRxFrame[loracomm_num].DataCnt] = ch;
			loracommRxFrame[loracomm_num].DataCnt++;
			loracommRxFrame[loracomm_num].State++;
			break;
			
		case LORACOMM_STATE_CMD:
//	printf("cmd\n");
			loracommRxFrame[loracomm_num].frame.D[loracommRxFrame[loracomm_num].DataCnt] = ch;
			loracommRxFrame[loracomm_num].DataCnt++;
			loracommRxFrame[loracomm_num].State++;
			break;
		
		case LORACOMM_STATE_LEN1:
//	printf("len_l\n");
			loracommRxFrame[loracomm_num].frame.D[loracommRxFrame[loracomm_num].DataCnt] = ch;
			loracommRxFrame[loracomm_num].DataCnt++;
			loracommRxFrame[loracomm_num].State++;
			break;	
			
		case LORACOMM_STATE_LEN2:
			loracommRxFrame[loracomm_num].frame.D[loracommRxFrame[loracomm_num].DataCnt] = ch;
			loracommRxFrame[loracomm_num].DataCnt++;
			
			loracommRxFrame[loracomm_num].LEN  = (unsigned short)(loracommRxFrame[loracomm_num].frame.F.Len_h)<<8;
			loracommRxFrame[loracomm_num].LEN |= (unsigned short)(loracommRxFrame[loracomm_num].frame.F.Len_l);
//	printf("len_h: %d\n", loracommRxFrame[loracomm_num].LEN);
			if(loracommRxFrame[loracomm_num].LEN == 0U) { // no data.
				loracommRxFrame[loracomm_num].State = LORACOMM_STATE_CRC1;
			}
			else {
				loracommRxFrame[loracomm_num].State++;
			}
			break;	
			
		case LORACOMM_STATE_DAT: // 6
			loracommRxFrame[loracomm_num].frame.D[loracommRxFrame[loracomm_num].DataCnt] = ch;
			loracommRxFrame[loracomm_num].DataCnt++;
//	printf("dat: %d/%d\n", loracommRxFrame[loracomm_num].DataCnt, (5 + loracommRxFrame[loracomm_num].LEN));
			if(loracommRxFrame[loracomm_num].DataCnt >= (5U + loracommRxFrame[loracomm_num].LEN) ) // src[1]+dst[1]+cmd[1]+len[2] + data[len]
			{
				loracommRxFrame[loracomm_num].State++;
			}
			break;			

		case LORACOMM_STATE_CRC1: 
//	printf("crc_l\n");		
			loracommRxFrame[loracomm_num].CRC = ch;
			loracommRxFrame[loracomm_num].State++;
			break;
			
		case LORACOMM_STATE_CRC2: 
//	printf("crc_h\n");
			loracommRxFrame[loracomm_num].CRC |= ((unsigned short)ch<<8);
			loracommRxFrame[loracomm_num].State++;
			break;
			
		case LORACOMM_STATE_ETX:
//	printf("etx\n");		
			if(ch == LORACOMM_FRAME_ETX) {
				DB_temp->loracomm_stat[loracomm_num].rx_frame_cnt++;
				// CRC: src[1]+dst[1]+cmd[1]+len[2] + data[len]
				uint16_t CRC = Generate_CRC(&loracommRxFrame[loracomm_num].frame.D[0], 5+(int)loracommRxFrame[loracomm_num].LEN);
				if(loracommRxFrame[loracomm_num].CRC != CRC) {
					plog_error("%s[%d]: Err(rcv:%x, cal:%x)",
								__func__,
								loracomm_num,
								loracommRxFrame[loracomm_num].CRC,
								CRC);
					DB_temp->loracomm_stat[loracomm_num].rx_frame_crc_err_cnt++;
				}
				else {
					if(loracomm_num == (unsigned char)LORACOMM_M_1) {
						loracomm_rx_frame_analyse((int)loracomm_num);	// main
					}
					else {
#if LORACOMM_SUB_THREAD_ENABLE
						loracomm_rx_frame_analyse_sub((int)loracomm_num); // sub 
#endif
					}
				}
			}

			loracomm_rx_frame_init(loracomm_num);
			break;
			
		default:
			//
			break;
	}		
}

static struct termios loracomm_oldtio[LORACOMM_M_NUM];

//-------------------------------------
//
//-------------------------------------
static int ttyLoraOpen(int port)
{
	const unsigned int loracomm_baud[LORACOMM_M_NUM] = { 
		0x0D, // B9600 -> 0000015 (0x0D) : bits/termios.h
		0x0D,
	};	
	
	
	struct termios loracomm_newtio[LORACOMM_M_NUM];
	int ret;
	
	ret	= open(loracomm_dev_port[port], (int)(0x02U | 0x100U | 0x800U)); // O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(ret >= 0) 
	{
		loracomm_fd[port] = ret;
		tcgetattr(loracomm_fd[port], &loracomm_oldtio[port]);
// B9600-0000015(0x0D), CS8-0000060(0x30), CLOCAL-0004000(0x800), CREAD-0000200)(0x80)
		loracomm_newtio[port].c_cflag = (loracomm_baud[port] | 0x30U | 0x800U | 0x80U);		
		//loracomm_newtio[port].c_cflag = (loracomm_baud[port] | CRTSCTS | CS8 | CLOCAL | CREAD);
		
		//loracomm_newtio[port].c_iflag = (IGNBRK | IGNPAR);
		loracomm_newtio[port].c_iflag = 0x04; //IGNPAR;	// IGNPAR - 0000004 (0x04)
		
		loracomm_newtio[port].c_oflag = 0; //CR0; //0;
		loracomm_newtio[port].c_lflag = 0;
		loracomm_newtio[port].c_cc[VMIN] = 0;
		loracomm_newtio[port].c_cc[VTIME] = 0;	
					
		tcflush(loracomm_fd[port], TCIFLUSH);
		tcsetattr(loracomm_fd[port], TCSANOW, &loracomm_newtio[port]);
		
		plog_info("%s[%d]: LORA Serial Port [%s] Open OK", __func__, port, loracomm_dev_port[port]);
	}
	else 
	{
		plog_error("%s[%d]: LORA Serial Port [%s] Open ERROR !!!", __func__, port, loracomm_dev_port[port]);
	}

	return ret;	
}


static void ttyLoraClose(int port)
{
	tcsetattr(loracomm_fd[port], TCSANOW, &loracomm_oldtio[port]);
	close(loracomm_fd[port]);
}



//=====================
// LORA COMM Thread.
//=====================

#define RXB_MAX	1024
static char rxb[LORACOMM_M_NUM][RXB_MAX];

static int E32_Init(int num)
{
	int ret = 0;

	E32_mode(num, 3);	// 11
	usleep(100000);
			
	ret = E32_ver_get(loracomm_fd[num]);
	if(ret < 0) {
		unused = printf("%s: E32_ver_get[%d] error !!!\n", __func__, num);
		ret = -1;
	}
	else {
		E32_cfg_init(loracomm_fd[num]);

		ret = E32_cfg_get(loracomm_fd[num]);
		if(ret < 0) {
			unused = printf("%s: E32_cfg_get[%d] error !!!\n", __func__, num);
			ret = -1;
		}
	}
			
	if(ret >= 0) {	
		int k=0;	
		
		E32_mode(num, 0);	// 00
		usleep(1000000);			

		for(k=0; k<10; k++) {
			ret = read(loracomm_fd[num], rxb[num], RXB_MAX); 
			if(ret <= 0) {
				break;
			}
		}
	}
	
	return ret;
}



//------------------------------------
//
//------------------------------------	
static void loracomm_to_PC(int loracomm_num)
{
	static unsigned int pkt_cnt = 0;
	static time_t curr_time; 
	const struct tm *t;
	loracomm_heartbit_req_t	loracomm_heartbit;
	
	time(&curr_time); 
	t = localtime(&curr_time);
	
#if 0 // DEBUG
	printf("\n==========> loracomm_to_PC[%d]: cnt = %d, %04d-%02d-%02d %02d:%02d:%02d\n",
			loracomm_num, pkt_cnt, t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);	
#endif
	
	loracomm_heartbit.pkt_cnt = pkt_cnt;
	pkt_cnt++;

	loracomm_heartbit.sys_stat.date_time.year   = (short)t->tm_year + 1900;
	loracomm_heartbit.sys_stat.date_time.month  = (char)t->tm_mon + 1;
	loracomm_heartbit.sys_stat.date_time.day    = (char)t->tm_mday;
	loracomm_heartbit.sys_stat.date_time.hour   = (char)t->tm_hour;
	loracomm_heartbit.sys_stat.date_time.minute = (char)t->tm_min;
	loracomm_heartbit.sys_stat.date_time.second = (char)t->tm_sec;
	
	
	if(DB_temp->gps.status == 2U) {
		vp_unused = memcpy(	(void *)&loracomm_heartbit.sys_stat.gps, 
				(void *)&DB_temp->gps, 
				sizeof(gps_t));	
	}
	else {
		loracomm_heartbit.sys_stat.gps.latitude = (float)0.0;
		loracomm_heartbit.sys_stat.gps.longitude = (float)0.0;
		loracomm_heartbit.sys_stat.gps.altitude = (float)0.0;
		loracomm_heartbit.sys_stat.gps.time.year = 0;
		loracomm_heartbit.sys_stat.gps.time.month = 0;
		loracomm_heartbit.sys_stat.gps.time.day = 0;
		loracomm_heartbit.sys_stat.gps.time.hour = 0;
		loracomm_heartbit.sys_stat.gps.time.minute = 0;
		loracomm_heartbit.sys_stat.gps.time.second = 0;
	}

#if 0
	memcpy(	(void *)&loracomm_heartbit.sys_stat.motion, 
			(void *)&DB_temp->motion, 
			sizeof(motion_t));	
#else
	

#if 1 //2022.3.22
	loracomm_heartbit.sys_stat.yaw			= DB_temp->motion.yaw;
#else
	loracomm_heartbit.sys_stat.motion.yaw	= DB_temp->motion.yaw;
	loracomm_heartbit.sys_stat.motion.pitch	= DB_temp->motion.pitch;
	loracomm_heartbit.sys_stat.motion.roll	= DB_temp->motion.roll;
	loracomm_heartbit.sys_stat.motion.m[0]	= DB_temp->motion.m[0];
	loracomm_heartbit.sys_stat.motion.m[1]	= DB_temp->motion.m[1];
	loracomm_heartbit.sys_stat.motion.m[2]	= DB_temp->motion.m[2];
	loracomm_heartbit.sys_stat.motion.h		= DB_temp->motion.h;
#endif
	
#endif	

	loracomm_heartbit.sys_stat.batt_status = DB_temp->i2c_batt.status;
	loracomm_heartbit.sys_stat.batt_percent = DB_temp->i2c_batt.percent;

	loracomm_tx_frame(	loracomm_num,
						LORACOMM_FRAME_SRC_ADDR,	// src : DPT
						LORACOMM_FRAME_DST_ADDR,	// dst : remotePC
						LORACOMM_FRAME_CMD_STAT,	// cmd : DPT(STATUS) -> remotePC
						(unsigned char *)&loracomm_heartbit,
						(int)sizeof(loracomm_heartbit_req_t) );

}



//------------------------
//
//------------------------	
void *loracomm_thread(void *arg)
{
	time_t sec;
	time_t prev_sec;
	int res;
	int i;
	int loracomm_num;
	char ch;
	int restart_cnt = 0;
	
#define LORACOMM_XXX_MS		1	
#if LORACOMM_XXX_MS // X_ms < 1_sec 					
	int lora_send_timer_start = 0;
    struct timeval ptv;
	struct timeval ctv;
	struct timeval gtv;
#endif

	vp_unused = arg;

	sleep(2);
	
	plog_info("%s: START.",  __func__);
	
	while(gAppEnd_flag == 0)
	{
		sleep(1);
		
		if(restart_cnt > 0) {
			plog_info("%s: restart_cnt = %d", __func__, restart_cnt); restart_cnt++;
		}

		loracomm_num = LORACOMM_M_1;

		plog_info("%s: loracomm_port = %d", __func__, loracomm_num);
			
		if(ttyLoraOpen(loracomm_num) < 0) {
			DB_temp->loracomm_stat[loracomm_num].error = 1;
		}
		else {
			loracomm_rx_frame_init((unsigned char)loracomm_num);

			if(isFileExsit("LORACOMM_INIT_SKIP") == 0) {
				res = E32_Init(loracomm_num);
				if(res < 0) {
					DB_temp->loracomm_stat[loracomm_num].error = 1;
				}
				else {
					DB_temp->loracomm_stat[loracomm_num].error = 0;
				}
			}

		}
		
		if(DB_temp->loracomm_stat[loracomm_num].error == 0U) 
		{
			DB_temp->loracomm_stat[loracomm_num].restart = 0;
			
			sec = time(NULL);
			prev_sec = sec;
			
			//-----------------------------------------------------------------------
			while((gAppEnd_flag == 0) && (DB_temp->loracomm_stat[loracomm_num].restart == 0U)) 
			{
				res = read(loracomm_fd[loracomm_num], rxb[loracomm_num], RXB_MAX); 
				if(res > 0) { // RX process
					for(i=0; i<res; i++) {
						loracomm_rx_frame_process((unsigned char)loracomm_num, (unsigned char)rxb[loracomm_num][i]);
					}
				}
				else { // TX process
					if(E32_aux(loracomm_num) == 1) // ???
					{
						//for(i=0; i<16; i++) 
						for(i=0; i<10; i++)  // 100byte면 (100/10)*10ms -> 한프레임 보내는데 100ms ~
						{
							if(loracomm_txbf_read((unsigned char)loracomm_num, &ch) != 0) {
								break;
							}
							//res = write(loracomm_fd[loracomm_num], &ch, 1);
							write(loracomm_fd[loracomm_num], &ch, 1);
						}
					}
				}
	
				sec = time(NULL);
				if(prev_sec != sec) 
				{
					prev_sec = sec;
					
#if LORACOMM_XXX_MS // X_ms < 1_sec
	//-----------------------------------
	// loracomm send cycle : 500ms
	//-----------------------------------
	#define LORA_SEND_TIMER_START	1		// 1,0 [2]
	#define LORA_SEND_TIMER_USEC	499000	// 500000:500ms

					if(lora_send_timer_start == 0) {
						lora_send_timer_start = LORA_SEND_TIMER_START;
						loracomm_to_PC(loracomm_num);
						gettimeofday(&ptv, NULL);						
					}
#else // 1_sec
					loracomm_to_PC(loracomm_num);
#endif					
				} // 1000ms
				
#if LORACOMM_XXX_MS // X_ms < 1_sec				
				if(lora_send_timer_start > 0) {
					gettimeofday(&ctv, NULL);
					gtv.tv_usec = ctv.tv_usec - ptv.tv_usec;
					if(gtv.tv_usec < 0) {
						gtv.tv_usec = gtv.tv_usec + 1000000;
					}
					if(gtv.tv_usec > LORA_SEND_TIMER_USEC) {
						lora_send_timer_start--;
						loracomm_to_PC(loracomm_num);
						gettimeofday(&ptv, NULL);
					}
				}
#endif				
				usleep(10000); // 10ms

			} // while
		
		}
		else {
			plog_error("%s: error!!!, check lora port(%d) ...", __func__, loracomm_num);
			break; // end
		}

		ttyLoraClose(loracomm_num);
		
	} // while

	plog_info("%s: END.",  __func__);

	pthread_exit((void *)0);
	return (void *)0;
}


#if LORACOMM_SUB_THREAD_ENABLE
//----------------------------
//
//----------------------------	
void *loracomm_sub_thread(void *arg)
{
#if 0	
	time_t sec;
	time_t prev_sec;
#endif
	int res;
	int i;
	int loracomm_num;
	char ch;
	int restart_cnt = 0;
	
	vp_unused = arg;
	
	sleep(1);
	
	plog_info("%s: START.",  __func__);
	
	while(gAppEnd_flag == 0)
	{
		sleep(1);
		
		if(restart_cnt > 0) {
			plog_info("%s: restart_cnt = %d", __func__, restart_cnt); restart_cnt++;
		}

		loracomm_num = LORACOMM_M_2; // sub 포트 사용.

		plog_info("%s: loracomm_port = %d", __func__, loracomm_num);

		if(ttyLoraOpen(loracomm_num) < 0) {
			unused = printf("%s: Serial Port [%s] Open ERROR !!!\n", __func__, loracomm_dev_port[loracomm_num]);
			DB_temp->loracomm_stat[loracomm_num].error = 1;
		}
		else {
			loracomm_rx_frame_init((unsigned char)loracomm_num);

			if(isFileExsit("LORACOMM_INIT_SKIP") == 0) {
				res = E32_Init(loracomm_num);
				if(res < 0) {
					DB_temp->loracomm_stat[loracomm_num].error = 1;
				}
				else {
					DB_temp->loracomm_stat[loracomm_num].error = 0;
				}
			}
		}
		

		if(DB_temp->loracomm_stat[loracomm_num].error == 0U) 
		{
			DB_temp->loracomm_stat[loracomm_num].restart = 0;

#if 0			
			sec = time(NULL);
			prev_sec = sec;
#endif

			while((gAppEnd_flag == 0) && (DB_temp->loracomm_stat[loracomm_num].restart == 0U)) 
			{
				res = read(loracomm_fd[loracomm_num], rxb[loracomm_num], RXB_MAX); 
				if(res > 0) { // RX process
					for(i=0; i<res; i++) {
						loracomm_rx_frame_process((unsigned char)loracomm_num, (unsigned char)rxb[loracomm_num][i]);
					}
				}
				else { // TX process
					if(E32_aux(loracomm_num) == 1) {
						for(i=0; i<16; i++) {
							if(loracomm_txbf_read((unsigned char)loracomm_num, &ch) != 0) {
								break;
							}
							//res = write(loracomm_fd[loracomm_num], &ch, 1);
							write(loracomm_fd[loracomm_num], &ch, 1);
						}
					}
				}

#if 0 // 명령 버퍼를 확인해서 있으면 그때마다 전송하는 구조...	
				sec = time(NULL);
				if(prev_sec != sec) {
					prev_sec = sec;
					
					loracomm_tx_frame(	loracomm_num,
										LORACOMM_FRAME_DST_ADDR,	// src : remotePC
										LORACOMM_FRAME_SRC_ADDR,	// dst : DPT
										LORACOMM_FRAME_CMD_CTRL,	// cmd : remotePC(CTRL) -> DPT
										(unsigned char *)&dpt_ctrl_data,
										sizeof(dpt_ctrl_data_t) );
				}
#endif				
				
				usleep(10000); // 10ms
				
			} // while
		
		}
		else {
			unused = printf("%s: error!!!, check lora port(%d) ...\n", __func__, loracomm_num);
			break; // end
		}

		ttyLoraClose(loracomm_num);
		
	} // while

	plog_info("%s: END.",  __func__);

	pthread_exit((void *)0);
	return (void *)0;
}
#endif


void init_db_temp_loracomm(void)
{
	int loracomm_num;
	for(loracomm_num = (int)LORACOMM_M_1; loracomm_num<(int)LORACOMM_M_NUM; loracomm_num++) {
		DB_temp->loracomm_stat[loracomm_num].restart = 0;	
		DB_temp->loracomm_stat[loracomm_num].error = 0;
		DB_temp->loracomm_stat[loracomm_num].tx_frame_cnt = 0;
		DB_temp->loracomm_stat[loracomm_num].rx_frame_cnt = 0;
		DB_temp->loracomm_stat[loracomm_num].rx_frame_crc_err_cnt = 0;
		DB_temp->loracomm_stat[loracomm_num].rx_frame_anal_err_cnt = 0;
	}
}

void disp_loracomm_stat(void)
{
	int loracomm_num;
	for(loracomm_num = (int)LORACOMM_M_1; loracomm_num<(int)LORACOMM_M_NUM; loracomm_num++) {
		unused = printf("loracomm[%d]: error=%d, tx_frame_cnt=%u, rx_frame_cnt=%u, rx_crc_err_cnt=%u, rx_anal_err_cnt=%u\n",
			loracomm_num,
			DB_temp->loracomm_stat[loracomm_num].error,
			DB_temp->loracomm_stat[loracomm_num].tx_frame_cnt,
			DB_temp->loracomm_stat[loracomm_num].rx_frame_cnt,
			DB_temp->loracomm_stat[loracomm_num].rx_frame_crc_err_cnt,
			DB_temp->loracomm_stat[loracomm_num].rx_frame_anal_err_cnt);
	}		
}
	


	