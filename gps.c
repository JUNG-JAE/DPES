#include <stdio.h>
#include <stdlib.h>		// atof 등
#include <unistd.h>		// read, close, sleep 등
#include <pthread.h>	// pthread_exit 등
#include <string.h>		// strlen 등
#include <fcntl.h>		// open( ... O_RDONLY) 등
#include <termios.h>	// struct termios 등 
//#include <sys/ioctl.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <assert.h>
//#include <time.h>
//#include <sys/time.h>
//#include <math.h>
//#include <stdint.h>

//#define DISABLE_EXTERN_GPS_H
#include "extern.h"



//---------------------------------------------------------------------------------
//#define GPS_PORT	"/dev/ttyXRUSB2"
#define GPS_PORT	"/dev/ttyUSB0"
//#define GPS_PORT	"/dev/ttyS0"


//#define GPS_BAUD	0x0DU	// B9600 - 0000015 (0x0D)
#define GPS_BAUD	0x1002U	// B115200 - 0010002 (0x1002)

//#define pi 3.14159265358979323846

static double ConvertGPRMCToDegree(double in);

//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------
static int gps_write(int hTTY, unsigned char *buff, int size)
{
	int res = write(hTTY, buff, size);
	if(res < 0) {
		plog_error("%s: write error", __func__); 
		return -1; 
	}
	
 	tcdrain (hTTY);
	
	if(res != size) {
		plog_warn("%s: write size[%d], return size[%d], not equal!!!", __func__, size, res);
	}
	
	return res;
} 

#if 0
static uint8_t GPGGA_OFF[16] =	{0xB5,0x62,0x06, 0x01,0x08,0x00,0xF0, 0x00,0x00,0x00,0x00,0x00,0x00,0x01, 0x00,0x24};
static uint8_t GPGLL_OFF[16] =	{0xB5,0x62,0x06, 0x01,0x08,0x00,0xF0, 0x01,0x00,0x00,0x00,0x00,0x00,0x01, 0x01,0x2B};					
static uint8_t GPGSA_OFF[16] =	{0xB5,0x62,0x06, 0x01,0x08,0x00,0xF0, 0x02,0x00,0x00,0x00,0x00,0x00,0x01, 0x02,0x32};
static uint8_t GPGSV_OFF[16] =	{0xB5,0x62,0x06, 0x01,0x08,0x00,0xF0, 0x03,0x00,0x00,0x00,0x00,0x00,0x01, 0x03,0x39};
static uint8_t GPRMC_OFF[16] =	{0xB5,0x62,0x06, 0x01,0x08,0x00,0xF0, 0x04,0x00,0x00,0x00,0x00,0x00,0x01, 0x04,0x40}; 
static uint8_t GPVTG_OFF[16] =	{0xB5,0x62,0x06, 0x01,0x08,0x00,0xF0, 0x05,0x00,0x00,0x00,0x00,0x00,0x01, 0x05,0x47};

static uint8_t GP1HZ_SET[14] =	{0xB5,0x62,0x06, 0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00, 0x01,0x39};
static uint8_t GP2HZ_SET[14] =	{0xB5,0x62,0x06, 0x08,0x06,0x00,0xF4,0x01,0x01,0x00,0x01,0x00, 0x0B,0x77};
static uint8_t GP4HZ_SET[14] =	{0xB5,0x62,0x06, 0x08,0x06,0x00,0xFA,0x00,0x01,0x00,0x01,0x00, 0xDE,0x6A};
static uint8_t GP5HZ_SET[14] =	{0xB5,0x62,0x06, 0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00, 0xDE,0x6A};
static uint8_t GP10H_SET[14] =	{0xB5,0x62,0x06, 0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00, 0x7A,0x12};

static uint8_t GPCFG_RST[21] =	{0xB5,0x62,0x06, 0x09,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x17, 0x2F,0xAE};
static uint8_t GPCFG_SAV[21] =	{0xB5,0x62,0x06, 0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x03, 0x1D,0xAB};
#endif

static void gps_setting(int gps_fd)
{
#if 0
	gps_write(gps_fd, GPCFG_RST, 21);	ms_sleep(100);
	//
//	gps_write(gps_fd, GPGGA_OFF, 16);	ms_sleep(100);
	gps_write(gps_fd, GPGLL_OFF, 16);	ms_sleep(100);
	gps_write(gps_fd, GPGSA_OFF, 16);	ms_sleep(100);
	gps_write(gps_fd, GPGSV_OFF, 16);	ms_sleep(100);
//	gps_write(gps_fd, GPRMC_OFF, 16);	ms_sleep(100);
	gps_write(gps_fd, GPVTG_OFF, 16);	ms_sleep(100);
	//

	gps_write(gps_fd, GP5HZ_SET, 14);	ms_sleep(100);
	//
	gps_write(gps_fd, GPCFG_SAV, 21);	ms_sleep(100);	
#else
	unused  = gps_fd;
#endif	
}

#define GPS_HZ	1//5

static int gps_time_convert(const char *timebuf, const char *datebuf);
//------------------------------------------------------------------------
// MAIN
//------------------------------------------------------------------------
static int GpsRead(void)
{
	const char *gps_port = GPS_PORT;
	int	gps_rd_len;
	char gps_rd_buf[256];
	
#define RCV_LENGTH	256	
	static char rcv_data[RCV_LENGTH];
	unsigned short rcv_index=0;
	
#define TOKEN_LEN	50
	char tokens[20][TOKEN_LEN];
	char tmp_tokens[10][TOKEN_LEN];
	
    int loop;
	int ChkNull=0;
	int token_cnt;
	int i;
	
	struct termios gps_oldtio;
	struct termios gps_newtio;
// O_RDWR - 00000002 (0x02), O_NOCTTY - 00000400 (0x100), O_NONBLOCK - 00004000 (0x800)	
	int gps_fd = open(gps_port, 0x02 | 0x100 | 0x800);
	if(gps_fd < 0) {
//		plog_error("%s: GPS Port [%s] Open ERROR !!!", __func__, GPS_PORT); // cppcheck: misra.py오류
		plog_error("%s: GPS Port [%s] Open ERROR !!!", __func__, gps_port);
	}
	else {
		tcgetattr(gps_fd, &gps_oldtio);			// CRTSCTS - 020000000000 (0x80000000)
		bzero(&gps_newtio, sizeof(gps_newtio)); // CS8-0000060 (0x30), CLOCAL-0004000 (0x800), CREAD-0000200) (0x80)
		//gps_newtio.c_cflag = GPS_BAUD | CRTSCTS | CS8 | CLOCAL | CREAD;	// hw flow control (USB_DGPS)
		gps_newtio.c_cflag = (unsigned int)(GPS_BAUD | 0x80000000U | 0x30U | 0x800U | 0x80U);	// hw flow control (USB_DGPS)
		//gps_newtio.c_cflag = GPS_BAUD | CS8 | CLOCAL | CREAD;
		//gps_newtio.c_iflag = (IGNBRK | IGNPAR);
		gps_newtio.c_iflag = 0x04;	// IGNPAR - 0000004 (0x04)
		gps_newtio.c_oflag = 0;
		gps_newtio.c_lflag = 0;
		gps_newtio.c_cc[VMIN] = 0;
		gps_newtio.c_cc[VTIME] = 0;	
		tcsetattr(gps_fd, TCSANOW, &gps_newtio);
		tcflush(gps_fd, TCIFLUSH);
		
	//	plog_info("%s: GPS Port [%s] Open OK", __func__, GPS_PORT); // cppcheck: misra.py오류
		plog_info("%s: GPS Port [%s] Open OK", __func__, gps_port);
		
		gps_setting(gps_fd); 
		
		vp_unused = memset(rcv_data, 0x00, RCV_LENGTH); // 꼭해주자.
		
		while(1)
		{
			gps_rd_len = read(gps_fd, gps_rd_buf, 256);
			if(gps_rd_len > 0) 
			{
				if(dbgdisp_status[DBGDISP_GPS_DUMP] == 1) 
				{
					for(loop = 0; loop < gps_rd_len; loop++) 
					{
						//printf("%c[%02x]", gps_rd_buf[loop], gps_rd_buf[loop]);
						unused = printf("%c", gps_rd_buf[loop]);
					}
				}
			}

			if(++ChkNull > 30) { // 50ms x30 => 1.5sec
				ChkNull = 0;
				DB_temp->gps.status = 0;		// no device, 0,1,2
				DB_temp->gps.latitude = (double)0.0;
				DB_temp->gps.longitude = (double)0.0;
				DB_temp->gps.altitude = (float)0.0;
			}
			
			
			if(gps_rd_len > 0) 
			{
				if(DB_temp->gps.status == 0U) 
				{ 	// no device
					gps_setting(gps_fd);
					DB_temp->gps.status = 1;		
				}
					
				ChkNull=0;
				for(loop = 0; loop < gps_rd_len; loop++)
				{
					if(gps_rd_buf[loop] == (char)0x0a) 
					{
						//printf("rcv_data[%d]: %s\n", rcv_index, rcv_data);
						
						for(i=0; i<20; i++) {
							vp_unused = memset(tokens[i],0,50);
						}
			
						token_cnt=0;

						int pos = 0;
						for(i=0; i<RCV_LENGTH; i++) {
							if(rcv_data[i] == '\0') { // NULL
								tokens[token_cnt][pos] = 0;
								if(pos > 0)	{
									token_cnt++;
								}
								break;
							}
							else if(rcv_data[i] == ',') {
								tokens[token_cnt][pos] = '\0';

								token_cnt++;
								pos = 0;
							}
							else {
								tokens[token_cnt][pos] = rcv_data[i];
								pos++;
#if 1 // 2022.3.23							
								if(pos >= (TOKEN_LEN-1))  { //  증가된 pos가 49(50-1)면  48로 변경, 넘지 않도록.
									pos--;
								}
#endif

							}
						}

#if 0 // debug
						if(token_cnt > 0) {
							for(i=0; i<token_cnt; i++) {
								printf("[%d:%s]", i, tokens[i]);
							}
							printf("\n");
						}
#endif

						if(strcmp(tokens[0], GxRMC_NAME) == 0) 
						{
							if(dbgdisp_status[DBGDISP_GPS_GPRMC] == 1) {
								unused = printf("[%d] : ", token_cnt);
								for(i=0;i<token_cnt;i++) {
									unused = printf("%s ", tokens[i]);
								}
								unused = printf("\n");
							}

							if(strcmp(tokens[2],"V") == 0) 
							{
								DB_temp->gps.status		= 1;	// no valid
								DB_temp->gps.latitude	= (double)0.0;
								DB_temp->gps.longitude 	= (double)0.0;
								DB_temp->gps.altitude 	= (float)0.0;
							}
							else { 
								if(strcmp(tokens[2],"A") == 0) 
								{
									if(token_cnt > 9)
									{
										DB_temp->gps.status		= 2;	// valid
#if 0
										DB_temp->gps.latitude	= atof(tokens[3]);
										DB_temp->gps.longitude 	= atof(tokens[5]);
#else // GPS수신(도/분 형식) -> GoogleMap(도 형식:degree)


										DB_temp->gps.latitude	= (double)ConvertGPRMCToDegree( atof(tokens[3]) );
										DB_temp->gps.longitude 	= (double)ConvertGPRMCToDegree( atof(tokens[5]) );
#endif
										vp_unused = strcpy(tmp_tokens[0], tokens[2]);  // 1 validity
										vp_unused = strcpy(tmp_tokens[1], tokens[3]);  // 2 current Latitude
										vp_unused = strcpy(tmp_tokens[2], tokens[4]);  // 3 North/South
										vp_unused = strcpy(tmp_tokens[3], tokens[5]);  // 4 current Longitude
										vp_unused = strcpy(tmp_tokens[4], tokens[6]);  // 5 East/West
										vp_unused = strcpy(tmp_tokens[5], tokens[1]);  // 6 Time Stamp
										vp_unused = strcpy(tmp_tokens[6], tokens[9]);  // 7 Date Stamp
									
										unused = gps_time_convert(tokens[1], tokens[9]);
										
									} 
									else {
										plog_error("%s: RMC, token_cnt[%d] error", __func__, token_cnt);
										for(i=0;i<token_cnt;i++) {
											plog_info("%s: token[%d] = %s", __func__, i, tokens[i]);
										}
									}
									
								}
							
							}
						}
						if(strcmp(tokens[0], GxGGA_NAME) == 0) 
						{
							if(dbgdisp_status[DBGDISP_GPS_GPGGA] == 1) 
							{
								unused = printf("(%d): ", token_cnt);
								for(i=0; i<token_cnt; i++) 
								{
									unused = printf("%s ", tokens[i]);
								}
								unused = printf("\n");
							}
							
							if(DB_temp->gps.status == 2U) 
							{ 	
								if(token_cnt > 9)
								{
									DB_temp->gps.altitude = (float)atof(tokens[9]);
									//
									if(dbgdisp_status[DBGDISP_GPS_DUMP] == 1) 
									{			//  1  2  3  4  5  6  7  8
										unused = printf("GPS:%s,%s,%s,%s,%s,%s,%s,%s\n",
													tmp_tokens[0],	// 1 validity
													tmp_tokens[1],	// 2 current Latitude
													tmp_tokens[2],	// 3 North/South
													tmp_tokens[3],	// 4 current Longitude
													tmp_tokens[4],	// 5 East/West
													tokens[9],		// 6 altitude
													tmp_tokens[5],	// 7 Time Stamp
													tmp_tokens[6]);	// 8 Date Stamp
									}
								}
								else {
									plog_error("%s: GGA, token_cnt[%d] error", __func__, token_cnt);
									for(i=0;i<token_cnt;i++) {
										plog_info("%s: token[%d] = %s", __func__, i, tokens[i]);
									}
								}
							}
						}
				
				
						

						rcv_index = 0;
						vp_unused = memset(rcv_data, 0x00, RCV_LENGTH);

					}
					else if(gps_rd_buf[loop] == (char)0x0d) {
						continue;
					}
					else 
					{
						if(rcv_index >= (unsigned short)RCV_LENGTH) {
							rcv_index = 0;
						}

						rcv_data[rcv_index] = gps_rd_buf[loop];
						rcv_index++;
					}
				}
			}

			ms_sleep(50);
			
			if(gAppEnd_flag == 1) {
				break;
			}
		}

		tcsetattr(gps_fd, TCSANOW, &gps_oldtio);
		close(gps_fd);
	}
	
    return 0;
}


//$GPRMC,024210.00,A,3734.12025,N,12649.64725,E,0.014,,210721,,,D*7F
//----------------------------------------------------------------------------------
//$GPRMC,114455.532,A,3735.0079,N,12701.6446,E,0.000000,121.61,110706,   ,*0A
//$GPRMC,111115    ,A,5144.3224,N,01922.8127,E,0.0     ,38.5  ,290112,3.7,E ,A*2B
//       1          2 3         4 5          6 7        8      9      10  11 12
//
//$GPRMC,111115,A,5144.3224,N,01922.8127,E,0.0,38.5,290112,3.7,E,A*2B
//       1      2 3         4 5          6 7   8    9      10  11 12
// 1 220516 Time Stamp
// 2 A validity - A-ok, V-invalid
// 3 5133.82 current Latitude
// 4 N North/South
// 5 00042.24 current Longitude
// 6 W East/West
// 7 173.8 Speed in knots  -> * 1.852 = Km/H
// 8 231.8 True course  -> 지원을 안함....
// 9 130694 Date Stamp
//
// --not supported
// 10 004.2 Variation
// 11 W East/West
// 12 *70 checksum
// 
//$GPRMC,065747.20,A,3734.07698,N,12649.67364,E,0.410,,270519, ,   ,A  *73
//       1         2 3          4 5           6 7    8 9      10 11 12 13
//
//       시간        유효 위도          경도          knots    degree 
//$GPRMC,114455.532,A, 3735.0079,N,12701.6446,E,0.000000,121.61,110706, ,   *0A
//       1          2  3         4 5          6 7        8      9      10 11 12
//
//$GNRMC,082832.50,A,3734.1514870,N,12649.7127869,E,0.039,,270122,,,D,V*18
//       1         2 3            4 5             6 7     89     10111213  
//	   
//-----------------------------------------------------------------------------------------
//0: Sentence ID	: $GPGGA or $GNGGA
//1: UTC			
//2: Latitude
//3: N/S Indicatior
//4: Longitude
//5: E/W Indicatior
//6: Position Fix
//7: Satellites Used
//8: HDOP
//9: Altitude
//10: Altitude Unit
//11: Geoid Separation
//12: Separation Unit
//13: DGPS Age
//14: DGPS Station ID
//15: Checksum
//
//       시간         위도         경도          계산종류 위성수 노이즈 고도(해수면) 고도2     dgps     체크섬 
//$GPGGA,114455.532,3735.0079,N,12701.6446,E,1,    03,  7.9, 48.8,M, 19.6,M  0.0,0000 *48
//0      1          2         3 4          5 6     7    8    9    10 11   12 13  14   15  
// 
//       1          2         3 4           5 6 7  8    9   10 11   1213 14 15
//$GPGGA,024209.80,3734.12024,N,12649.64725,E,2,10,1.19,63.0,M,18.1,M,,0000*63 
//  
//--- RTK ---
//$GNGGA,082832.50,3734.1514870,N,12649.7127869,E,2,12,0.68,272.651,M,18.127,M,,0000*43
//0      1         2            3 4             5 6 7  8    9       1011     1213 14 15



//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------
void *gps_thread(void *arg)
{
	vp_unused = arg;
	
	sleep(1);
	
    plog_info("%s: START.",  __func__);

    while(1)
    {
        unused = GpsRead();

		if(gAppEnd_flag == 1) {
			break;
		}

		sleep(1);
    }
	
    plog_info("%s: END.",  __func__);
	
	pthread_exit((void *)0);	
}






//---------------------------------------------------------------------------------
static int gps_time_convert(const char *timebuf, const char *datebuf)
{
	int day;
	int month;
	int year;
	int hour;
	int min;
	int sec; 
	int ret = 0;
 
	if (sscanf(timebuf, "%02d%02d%02d", &hour, &min, &sec) != 3) {
		plog_error("gps_time_convert: timebuf error !!!");
		ret = -1;
	}
	else {
 
		if (sscanf(datebuf, "%02d%02d%02d", &day, &month, &year) != 3) {
			plog_error("gps_time_convert: datebuf error !!!");
			ret = -1;
		}
		else {
		  
			year += 2000; 
		  
			if(	((day >= 1) && (day <= 31)) 
			&&	((month >= 1) && (month <= 12)) 
			&&	((year >= 2000)) 
			&&  ((hour >= 0) && (hour <= 23)) 
			&&	((min >= 0) && (min <= 59))
			&&	((sec >= 0) && (sec <= 59))	) 
			{ 
				time_t curr_time; 
				time(&curr_time); 
				struct tm *t = localtime(&curr_time); 
				
				if(dbgdisp_status[DBGDISP_GPS_TIMEUPDATE] == 1) {
					static int cnt = 0;
					if(++cnt > GPS_HZ) { // 5 Hz -> 1sec display.
						cnt = 0;
						unused = printf("Local time: %04d-%02d-%02d %02d:%02d:%02d (%s)\n",
								(t->tm_year+1900), (t->tm_mon+1), t->tm_mday,
								t->tm_hour, t->tm_min, t->tm_sec, t->tm_zone); 
						unused = printf("GPS   time: %04d-%02d-%02d %02d:%02d:%02d (UTC)\n",
								year, month, day, hour, min, sec); 
					}
				}

				DB_temp->gps.time.year		= (short)year;
				DB_temp->gps.time.month		= (char)month;
				DB_temp->gps.time.day		= (char)day;
				DB_temp->gps.time.hour		= (char)hour;
				DB_temp->gps.time.minute	= (char)min;
				DB_temp->gps.time.second	= (char)sec;
				
				if(DB_bkup->gps_time_update == 1) {
					t->tm_year	= (int)DB_temp->gps.time.year - 1900; 
					t->tm_mon	= (int)DB_temp->gps.time.month - 1; 
					t->tm_mday	= (int)DB_temp->gps.time.day; 
					t->tm_hour	= (int)DB_temp->gps.time.hour; 
					t->tm_min	= (int)DB_temp->gps.time.minute; 
					t->tm_sec	= (int)DB_temp->gps.time.second; //
			 
					time_t gps_time = mktime(t); 
					gps_time += t->tm_gmtoff; 
			 
					if(stime(&gps_time) == 0) { 
						//printf("gps_time_convert: Successfully updated local time.\n"); 
					}
					else { 
						plog_error("gps_time_convert: stime error !!!");
						ret = -1;		
					} 
				}
			} 
			else {
				plog_error("gps_time_convert: range error, gps(%04d-%02d-%02d %02d:%02d:%02d)",
								year, month, day, hour, min, sec); 
				ret = -1;
			}
		}
	}

	return ret;
}

#if 0
//---------------------------------------------------------------------------------
static double deg2rad(double deg)
{
	return (deg * pi / 180);
}

//---------------------------------------------------------------------------------
static double rad2deg(double rad)
{
	return (rad * 180 / pi);
}

//---------------------------------------------------------------------------------
static double distance(double lat1, double lon1,double lat2, double lon2, char unit)
{
	double theta,dist;
	
	theta = lon1 - lon2;
	dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
	dist = acos(dist);
	dist = rad2deg(dist);
	dist = dist * 60 * 1.1515;
	
	switch(unit) {
		case 'M' :		// Mile
			break;
		case 'K' :		// Kilo meter
			dist = dist * 1.609344;
			break;
		case 'N' :
			break;
	}
	
	return(dist);
}
#endif

//---------------------------------------------------------------------------------
static double ConvertGPRMCToDegree(double in)
{
	double ret = 0.;
	double tmpd = in;
	int tmpi = 0;
	int tmpi2 = 0;
//	GPS_stat = 2(valid), 37.568675, 126.827521, 81.892998		
	tmpi = (int)tmpd;
	tmpi2 = tmpi/100;
	ret += (double)tmpi2;
//	printf("	#1 int [%d] [%g]\n",tmpi,ret);
	tmpi2 = ((tmpi/100)*100);
	tmpd -= (double)tmpi2;
	tmpd /= 60.0;
//	printf("	#2 int [%g] [%g]\n",tmpd,ret);
	ret += tmpd;
//	printf("	#3 [%g]\n",ret);
	return(ret);
}

#if 0
//---------------------------------------------------------------------------------
static double bearingP1toP2(double P1_Lat, double P1_Lon, double P2_Lat, double P2_Lon) 
{
	// 현재 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
	double P1_Lat_radian = P1_Lat * (3.141592 / 180);
	double P1_Lon_radian = P1_Lon * (3.141592 / 180);
	
	// 목표 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
	double P2_Lat_radian = P2_Lat * (3.141592 / 180);
	double P2_Lon_radian = P2_Lon * (3.141592 / 180);
	
	// radian distance
	double radian_distance = 0;
	radian_distance = acos( sin(P1_Lat_radian) * sin(P2_Lat_radian) + 
							cos(P1_Lat_radian) * cos(P2_Lat_radian) * cos(P1_Lon_radian - P2_Lon_radian) );
	
	// 목적지 이동 방향을 구한다.(현재 좌표에서 다음 좌표로 이동하기 위해서는 방향을 설정해야 한다. 라디안값이다.
	double radian_bearing = acos( sin(P2_Lat_radian) - 
								  sin(P1_Lat_radian) * 
								   cos(radian_distance) / (cos(P1_Lat_radian) * sin(radian_distance)) ); // acos의 인수로 주어지는 x는 360분법의 각도가 아닌 radian(호도)값이다.
	
	double true_bearing = 0;
	if(sin(P2_Lon_radian - P1_Lon_radian) < 0) {
		true_bearing = radian_bearing * (180 / 3.151592);
		true_bearing = 360 - true_bearing;
	}
	else {
		true_bearing = radian_bearing * (180 / 3.141592);
	}
	
	return true_bearing;
}
#endif

//-----------------------------
//
//-----------------------------
void init_db_temp_gps(void)
{
	DB_temp->gps.latitude		= (double)0.0;
	DB_temp->gps.longitude 		= (double)0.0;
	DB_temp->gps.altitude		= (float)0.0;
	DB_temp->gps.time.year		= 0;
	DB_temp->gps.time.month		= 0;
	DB_temp->gps.time.day		= 0;
	DB_temp->gps.time.hour		= 0;
	DB_temp->gps.time.minute	= 0;
	DB_temp->gps.time.second	= 0;
	DB_temp->gps.status			= 0;
}



//=============================

/*

Rover Accuracy (m): 0.8817
Batt (99%): Voltage: 4.18V Discharging: 0.00%/hr Green
Rover Accuracy (m): 0.8794
Rover Accuracy (m): 0.8775
Batt (99%): Voltage: 4.18V Discharging: 0.00%/hr Green
Rover Accuracy (m): 0.8761
Rover Accuracy (m): 0.8749

---------------------------
//2023.11.23
>> enter후 >>
SparkFun RTK Express v3.8
** luetooth SPP broadcasting as: Express Rover-2736 **
Menu: Main
          1) Configure GNSS Receiver
2) Configure GNSS Messages
3) Configure Base
15) Configure Logging
6) Configure WiFi
7) Configure Network
1) Configure User Profiles
r) Configure Radios
s) Configure Sy1tem
f) Firmware upgrade
x) Ex1t

*/



void rtx_config_mode_exit(int rtk_cfg_fd)
{
	unused = gps_write(rtk_cfg_fd, (unsigned char *)"\n", 1);
	ms_sleep(500);
	unused = gps_write(rtk_cfg_fd, (unsigned char *)"x", 1);
	ms_sleep(500);
	unused = gps_write(rtk_cfg_fd, (unsigned char *)"\n", 1);
}


#define RTK_CFG_PORT	"/dev/ttyUSB1"

#define RTK_CFG_BAUD	0x1002U	// B115200 - 0010002 (0x1002)


//------------------------------------------------------------------------
// 
//------------------------------------------------------------------------
static int rtkCfgRead(void)
{
	const char *rtk_cfg_port = RTK_CFG_PORT; // cppcheck: misra.py오류
	char rtk_cfg_rd_buf[256];
	int	rtk_cfg_rd_len;

	#define RCV_LENGTH	256	
	static char rcv_data[RCV_LENGTH];
	unsigned short rcv_index=0;
	char	tokens[20][50];
    int loop;
	int ChkNull=0;
	int ChkNull_2=0;
	int token_cnt;
	int i;
	int ret = 0;
	
	struct termios rtk_cfg_oldtio;
	struct termios rtk_cfg_newtio;
	// O_RDWR - 00000002 (0x02), O_NOCTTY - 00000400 (0x100), O_NONBLOCK - 00004000 (0x800)
	int rtk_cfg_fd = open(rtk_cfg_port, 0x02 | 0x100 | 0x800);
	if(rtk_cfg_fd < 0) {
//		plog_error("%s: RTK_CFG Port [%s] Open ERROR !!!", __func__, RTK_CFG_PORT); // cppcheck: misra.py오류
		plog_error("%s: RTK_CFG Port [%s] Open ERROR !!!", __func__, rtk_cfg_port);
		ret =  -1;
	}
	else {

		tcgetattr(rtk_cfg_fd, &rtk_cfg_oldtio);	// CRTSCTS - 020000000000 (0x80000000)
		bzero(&rtk_cfg_newtio, sizeof(rtk_cfg_newtio)); // CS8-0000060 (0x30), CLOCAL-0004000 (0x800), CREAD-0000200) (0x80)
		rtk_cfg_newtio.c_cflag = (unsigned int)(RTK_CFG_BAUD | 0x80000000U | 0x30U | 0x800U | 0x80U);	// hw flow control (USB_DGPS)
		//rtk_cfg_newtio.c_cflag = RTK_CFG_BAUD | CS8 | CLOCAL | CREAD;
		//rtk_cfg_newtio.c_iflag = (IGNBRK | IGNPAR);
		rtk_cfg_newtio.c_iflag = 0x04;	// IGNPAR - 0000004 (0x04)
		
		rtk_cfg_newtio.c_oflag = 0;
		rtk_cfg_newtio.c_lflag = 0;
		rtk_cfg_newtio.c_cc[VMIN] = 0;
		rtk_cfg_newtio.c_cc[VTIME] = 0;	
		tcsetattr(rtk_cfg_fd, TCSANOW, &rtk_cfg_newtio);
		tcflush(rtk_cfg_fd, TCIFLUSH);
		
	//	plog_info("%s: RTK_CFG Port [%s] Open OK", __func__, RTK_CFG_PORT); // cppcheck: misra.py오류
		plog_info("%s: RTK_CFG Port [%s] Open OK", __func__, rtk_cfg_port);
	 
		vp_unused = memset(rcv_data, 0x00, RCV_LENGTH); // 꼭해주자.
		

		rtx_config_mode_exit(rtk_cfg_fd);


		while(1)
		{
			rtk_cfg_rd_len = read(rtk_cfg_fd, rtk_cfg_rd_buf, 256);
			if(rtk_cfg_rd_len > 0) 	{
				if(dbgdisp_status[DBGDISP_RTK_DUMP] == 1) { 
					for(loop = 0; loop < rtk_cfg_rd_len; loop++) {
						//printf("%c[%02x]", rtk_cfg_rd_buf[loop], rtk_cfg_rd_buf[loop]);
						unused = printf("%c", rtk_cfg_rd_buf[loop]);
					}
				}
			}

			if(++ChkNull > 200) { // 50ms x200 => 10000ms
				ChkNull = 0;
				if(DB_temp->RoverCfgRecv != (char)0) {
					DB_temp->RoverCfgRecv = 0;
					if(dbgdisp_status[DBGDISP_RTK_DUMP] == 1) {
						unused = printf("RoverCfgRecv = 0\n");
					}
					rtx_config_mode_exit(rtk_cfg_fd); // 아무것도 안들어 올때 
				}
			}

			if(++ChkNull_2 > 200) { // 50ms x200 => 10000ms
				ChkNull_2 = 0;
				if(DB_temp->RoverCfgRecv != (char)1) {
					DB_temp->RoverCfgRecv = 1;
					if(dbgdisp_status[DBGDISP_RTK_DUMP] == 1) {
						unused = printf("RoverCfgRecv = 1(a)\n");
					}
				}
			}

			if(rtk_cfg_rd_len > 0) 
			{
				ChkNull = 0;
				
				if(DB_temp->RoverCfgRecv == (char)0) {
					DB_temp->RoverCfgRecv = 1;
					if(dbgdisp_status[DBGDISP_RTK_DUMP] == 1) {
						unused = printf("RoverCfgRecv = 1(b)\n");
					}
				}
				
				for(loop = 0; loop < rtk_cfg_rd_len; loop++)
				{
					if(rtk_cfg_rd_buf[loop] == (char)0x0a) 
					{
						//printf("rcv_data[%d]: %s\n", rcv_index, rcv_data);
						
						for(i=0; i<20; i++) {
							vp_unused = memset(tokens[i],0,50);
						}
			
						token_cnt=0;

						int pos = 0;
						for(i=0; i<RCV_LENGTH; i++) {
							if(rcv_data[i] == '\0') { // NULL
								tokens[token_cnt][pos] = 0;
								if(pos > 0)	{
									token_cnt++;
								}
								break;
							}
							else if(rcv_data[i] == ' ') {
								tokens[token_cnt][pos] = '\0';
								token_cnt++;
								pos = 0;
							}
							else {
								tokens[token_cnt][pos] = rcv_data[i];
								pos++;
							}
						}

#if 0 // debug
						if(token_cnt > 0) {
							for(i=0; i<token_cnt; i++) {
								printf("[%d:%s]", i, tokens[i]);
							}
							printf("\n");
						}
#endif

						if(strcmp(tokens[0], "Rover") == 0) 
						{
#if 0 // debug
							printf("[%d] : ", token_cnt);
							for(i=0;i<token_cnt;i++) {
								printf("%s ", tokens[i]);
							}
							printf("\n");
#endif
							DB_temp->RoverAccuracy = (float)atof(tokens[3]);
							if(DB_temp->RoverCfgRecv != (char)2) {
								DB_temp->RoverCfgRecv = (char)2;
								if(dbgdisp_status[DBGDISP_RTK_DUMP] == 1) {
									unused = printf("RoverCfgRecv = 2\n");
								}
							}
							ChkNull_2 = 0;
						}
						if(strcmp(tokens[0], "Batt") == 0) 
						{
#if 0 // debug
							printf("(%d): ", token_cnt);
							for(i=0; i<token_cnt; i++) 
							{
								printf("%s ", tokens[i]);
							}
							printf("\n");
#endif
							size_t len;
							len = strlen(tokens[1]);	// 1234567
							if((len>(size_t)0) && (len<(size_t)8)) {		// (100%):
								vp_unused = memset(DB_temp->RoverBattery, 0, 8);
								//strncpy(DB_temp->RoverBattery, tokens[1], len);
								int ii;
								for(ii=0; ii<8; ii++) {
									char ch = tokens[1][1 + ii]; 
									if(ch == ')') {
										break;
									}
									DB_temp->RoverBattery[ii] = ch;
								}
							}
							
							len = strlen(tokens[3]);	// 12345
							if((len>(size_t)0) && (len<(size_t)6)) {		// 4.18V
								vp_unused = memset(DB_temp->RoverVoltage, 0, 6);
								vp_unused = strncpy(DB_temp->RoverVoltage, tokens[3], len);
							}
						}
						
						
						

						rcv_index = 0;
						vp_unused = memset(rcv_data, 0x00, RCV_LENGTH);

					}
					else if(rtk_cfg_rd_buf[loop] == (char)0x0d) {
						continue;
					}
					else 
					{
						if(rcv_index >= (unsigned short)RCV_LENGTH) {
							rcv_index = 0;
						}

						rcv_data[rcv_index] = rtk_cfg_rd_buf[loop];
						rcv_index++;
					}
				}
			}

			ms_sleep(50);
			
			if(gAppEnd_flag == 1) {
				break;
			}
		}

		tcsetattr(rtk_cfg_fd, TCSANOW, &rtk_cfg_oldtio);
		close(rtk_cfg_fd);
	}
	
    return ret;
}


//--------------------------------------------------------------------------------
//
//--------------------------------------------------------------------------------
void *rtk_cfg_thread(void *arg)
{
	vp_unused = arg;
	
	sleep(1);
	
    plog_info("%s: START.",  __func__);

    while(1)
    {
        unused = rtkCfgRead();

		if(gAppEnd_flag == 1) {
			break;
		}

		sleep(1);
    }
	
    plog_info("%s: END.",  __func__);
	
	pthread_exit((void *)0);	
}