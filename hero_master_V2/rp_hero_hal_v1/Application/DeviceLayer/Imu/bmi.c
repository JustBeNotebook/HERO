/**
 * @file        bmi.c
 * @author      RobotPilots@2020
 * @Version     V1.0
 * @date        16-November-2020
 * @brief       BMI270.
 */

/* Includes ------------------------------------------------------------------*/
#include "bmi.h"
#include "bmi2.h"
#include "bmi2_defs.h"
#include "bmi2_common.h"
#include "bmi270.h"
#include "bmi270_context.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
#define GRAVITY_EARTH  (9.80665f)
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!mark
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI2_GYR_RANGE_2000)) * (val);
}


/*快速求开方根的倒数*/
float inVSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/* Exported functions --------------------------------------------------------*/
int8_t  Set_gyro(struct bmi2_dev *dev)
{

    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(&config, 1, dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT2, dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {

        /* The user can change the following configuration parameters according to their requirement. */
        /* Output Data Rate. By default ODR is set as 200Hz for gyro. */
        config.cfg.gyr.odr = BMI2_GYR_ODR_1600HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config.cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the gyro configurations. */
        rslt = bmi2_set_sensor_config(&config, 1, dev);
    }

    return rslt;	

}

int8_t  Set_accel(struct bmi2_dev *bmi2_dev)
{
 /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi2_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI2_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi2_dev);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Output Data Rate. By default ODR is set as 100Hz for accel. */
        config.cfg.acc.odr = BMI2_ACC_ODR_1600HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config.cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel configurations. */
        rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);
    }

    return rslt;	
}

struct bmi2_dev bmi2_dev;
struct bmi2_sensor_data sensor_data = { 0 };
int8_t debug_bmi;
int8_t BMI_Init(void)
{
  /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print gyro data. */

    /*! Sensor initialization configuration. */
    
    /* Create an instance of sensor data structure. */
    
    /* Assign gyro sensor to variable. */
    uint8_t sens_listp;

    /* Initialize the interrupt status of gyro. */

    /* Initialize the dev structure */
    rslt = bmi2_interface_selection(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);
		
    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi2_dev);
    bmi2_error_codes_print_result(rslt);
		
    if (rslt == BMI2_OK)
    {
		sens_listp = BMI2_GYRO;		
        /* Enable the selected sensors. */
        rslt = bmi2_sensor_enable( &sens_listp, 1, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);	
		Set_gyro(&bmi2_dev);
			
		sens_listp = BMI2_ACCEL;
		/* Enable the selected sensors. */
        rslt = bmi2_sensor_enable( &sens_listp, 1, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);
		Set_accel(&bmi2_dev);
			
		sens_listp = BMI2_AUX;
        /* Enable the selected sensors. */
        rslt = bmi2_sensor_enable( &sens_listp, 1, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);
    }
    
    return rslt;
}

void BMI_Get_RawData(short *ggx,short *ggy,short *ggz,short *aax,short *aay,short *aaz)
{
	uint8_t data[12];
	int16_t buff[6];
	
	MPU_Read_all(ACCD_X_LSB,data,12);
	
	buff[0] = (int16_t)data[0] | ( (int16_t)data[1] << 8);
	buff[1] = (int16_t)data[2] | ( (int16_t)data[3] << 8);
	buff[2] = (int16_t)data[4] | ( (int16_t)data[5] << 8);
	
	buff[3] = (int16_t)data[6] | ( (int16_t)data[7] << 8);
	buff[4] = (int16_t)data[8] | ( (int16_t)data[9] << 8);
	buff[5] = (int16_t)data[10] | ( (int16_t)data[11] << 8);

	*aax = buff[0];
	*aay = buff[1];
	*aaz = buff[2];
	*ggx = buff[3];
	*ggy = buff[4];
	*ggz = buff[5];	
}

void BMI_Get_AUX(short *au1,short *au2,short *au3,short *au4)
{
	int16_t x[3],y[3],z[3],r[3];
	
	x[0] = MPU_Read_Byte(0x04) ;
	x[1] = MPU_Read_Byte(0x05)  ;
	x[2] = (int16_t)x[0] | ( (int16_t)x[1] << 8);
	
	
	y[0] = MPU_Read_Byte(0x06); 
	y[1] = MPU_Read_Byte(0x07)  ; 
	y[2] = (int16_t)y[0] | ( (int16_t)y[1] << 8 );
	
	
	z[0] = MPU_Read_Byte(0x08) ; 
	z[1] = MPU_Read_Byte(0x09) ; 
	z[2] = (int16_t)z[0] | ( (int16_t)z[1] << 8 );
	
		
	r[0] = MPU_Read_Byte(0x0a) ; 
	r[1] = MPU_Read_Byte(0x0b) ; 
	r[2] = (int16_t)z[0] | ( (int16_t)z[1] << 8 );

	*au1 = x[2];
	*au2 = y[2];
	*au3 = z[2];
	*au4 = r[2];
}

void BMI_Get_GRO(short *ggx,short *ggy,short *ggz)
{

	int16_t x[3],y[3],z[3];
	
	x[0] = MPU_Read_Byte(GYR_X_LSB) ;
	x[1] = MPU_Read_Byte(GYR_X_MSB)  ;
	x[2] = (int16_t)x[0] | ( (int16_t)x[1] << 8);
	
	
	y[0] = MPU_Read_Byte(GYR_Y_LSB); 
	y[1] = MPU_Read_Byte(GYR_Y_MSB)  ; 
	y[2] = (int16_t)y[0] | ( (int16_t)y[1] << 8 );
	
	
	z[0] = MPU_Read_Byte(GYR_Z_LSB) ; 
	z[1] = MPU_Read_Byte(GYR_Z_MSB) ; 
	z[2] = (int16_t)z[0] | ( (int16_t)z[1] << 8 );

	*ggx = x[2];
	*ggy = y[2];
	*ggz = z[2];

}

void BMI_Get_ACC(short *aax,short *aay,short *aaz)
{
	int16_t x[3],y[3],z[3];
	
	x[0] = MPU_Read_Byte(ACCD_X_LSB) ;
	x[1] = MPU_Read_Byte(ACCD_X_MSB)  ;
	x[2] = (int16_t)x[0] | ( (int16_t)x[1] << 8);
	
	
	y[0] = MPU_Read_Byte(ACCD_Y_LSB); 
	y[1] = MPU_Read_Byte(ACCD_Y_MSB); 
	y[2] = (int16_t)y[0] | ( (int16_t)y[1] << 8 );
	
	
	z[0] = MPU_Read_Byte(ACCD_Z_LSB) ; 
	z[1] = MPU_Read_Byte(ACCD_Z_MSB) ; 
	z[2] = (int16_t)z[0] | ( (int16_t)z[1] << 8 );

	*aax = x[2];
	*aay = y[2];
	*aaz = z[2];
}


/**
    @param
    @Kp
        越大表示越信任加速度，但快速晃动时，yaw轴角度可能会变化或者快速漂移。Kp越大，初始化的时候数据稳定越快。
    @Ki
        越小积分误差越小
    @halfT
        解算周期的一半，比如1ms解算1次则halfT为0.0005f
*/
float Kp = 6.0f;    
float Ki = 0.00f;
float halfT = 0.000510f;

float norm;
float vx, vy, vz;
float ex, ey, ez;
float gx,gy,gz,ax,ay,az;	 
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float exInt,eyInt,ezInt;
float q0temp,q1temp,q2temp,q3temp; 
short dead_zone = 5;
int timese;
float dead_zone_err = 0;
uint8_t BMI_Get_EulerAngle(float *pitch,float *roll,float *yaw,short *ggx,short *ggy,short *ggz,short *aax,short *aay,short *aaz)
{ 
	gx = (int16_t)(*ggx);
	gy = (int16_t)(*ggy);
	gz = (int16_t)((*ggz) /dead_zone) *dead_zone;    // 死区处理，减小yaw漂移速度
	ax =*aax;
	ay =*aay;
	az =*aaz;
	timese ++;
	if(ax * ay *az == 0)
	{
		return 0;
	}		

	gx = lsb_to_dps(gx,2000,bmi2_dev.resolution);
	gy = lsb_to_dps(gy,2000,bmi2_dev.resolution);
	gz = lsb_to_dps(gz,2000,bmi2_dev.resolution);
	
	gx = gx * 0.0174f;//陀螺仪角速度
	gy = gy * 0.0174f;
	gz = gz * 0.0174f;
	
	ax = lsb_to_mps2(ax,2,bmi2_dev.resolution);
	ay = lsb_to_mps2(ay,2,bmi2_dev.resolution);
	az = lsb_to_mps2(az,2,bmi2_dev.resolution);	

	norm = inVSqrt(ax*ax + ay*ay + az*az);
	ax = ax *norm;
	ay = ay *norm;
	az = az *norm;
	
	vx = 2*(q1*q3 - q0*q2); 
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	ex = (ay*vz - az*vy) ;   
	ey = (az*vx - ax*vz) ;
	ez = (ax*vy - ay*vx) ;
	
	exInt = exInt + ex * Ki;   
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt; 
	
	q0temp=q0;//过去时刻的四元数
	q1temp=q1;  
	q2temp=q2;  
	q3temp=q3; 
	
	//一阶龙格库塔求解四元数微分方程
	q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
	q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
	q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
	q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
	
	norm = inVSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 *norm;
	q1 = q1 * norm;
	q2 = q2 *norm;
	q3 = q3 *norm;
	
	*roll = atan2(2 * q2 * q3 + 2 * q0 * q1,q0*q0 - q1 * q1 -  q2 * q2 + q3 *q3)* 57.3f;
	*pitch = -asin( 2 * q1 * q3 -2 * q0* q2)*57.3f;
	*yaw =  atan2(2*(q1*q2 + q0*q3),q0*q0 +q1*q1-q2*q2 -q3*q3)*57.3f ;
	
	return 0;
}


void BMI_SET_Kp(float KP_set)
{
	Kp = KP_set;
}

