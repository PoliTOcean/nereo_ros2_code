#include "nereo_sensors_pkg/imu_libs/WT61P.h"

static int fd;
char *i2c_dev = "/dev/i2c-1";

static volatile char s_cDataUpdate = 0;

static int i2c_read(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_read_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}
static int i2c_write(u8 addr, u8 reg, u8 *data, u32 len)
{
	if(i2c_write_data(addr>>1, reg, data, len) < 0)return 0;
	return 1;
}
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
	usleep(ucMs*1000);
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			usleep(5);
			if(s_cDataUpdate != 0)
			{
				printf("find %02X addr sensor\r\n", i);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

int WT61P_begin(char* dev, uint8_t addr){
	fd = i2c_open(dev, WT_TIMEOUT, RETRY);
	if( fd < 0){
		printf("open %s fail\n", dev);
		return -1;
	}
	else printf("open %s success\n", dev);
	WitInit(WIT_PROTOCOL_I2C, addr);
	WitI2cFuncRegister(i2c_write, i2c_read);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(Delayms);
	int iRetry = 2;
	do
	{
		s_cDataUpdate = 0;
		WitReadReg(AX, 3);
		usleep(50);
		if(s_cDataUpdate != 0)
		{
			printf("find %02X addr sensor\r\n", addr);
			iRetry--;
		}
		iRetry--;
	}while(iRetry);
	if(s_cDataUpdate == 0)
		return -1;
	printf("WT61P connected\n");
	return 0;
}

int32_t WT61P_read_angle(){
	return WitReadReg(Roll, 3);
}

int32_t WT61P_read_acc(){
	return WitReadReg(AX, 3);
}

int32_t WT61P_read_angular_vel(){
	return WitReadReg(GX, 3);
}

float WT61P_get_roll(){
	return sReg[Roll] / 32768.0f * 180.0f;
}

float WT61P_get_pitch(){
	return sReg[Pitch] / 32768.0f * 180.0f;
}

float WT61P_get_yaw(){
    return sReg[Yaw] / 32768.0f * 180.0f;
}

float WT61P_get_acc_x(){
	return sReg[AX] / 32768.0f * 2.0f;
}

float WT61P_get_acc_y(){
	return sReg[AY] / 32768.0f * 2.0f;
}

float WT61P_get_acc_z(){
	return sReg[AZ] / 32768.0f * 2.0f;
}

float WT61P_get_angular_vel_x(){
	return sReg[GX] / 32768.0f * 2000.0f;
}

float WT61P_get_angular_vel_y(){
	return sReg[GY] / 32768.0f * 2000.0f;
}

float WT61P_get_angular_vel_z(){
	return sReg[GZ] / 32768.0f * 2000.0f;
}
