//
// Created by SuperChen on 2024/11/25.
//

#include "adis16467.h"
#include "drv_dwt.h"
#include "rm_config.h"
#include <drv_spi.h>
#include <math.h>
#include <rtdbg.h>
#include "drv_spi.h"
#include <rtthread.h>
#include <rtdevice.h>

#define Reserved    0x00      //0x00,0x01
#define DIAG_STAT   0x02      //0x02,0x03

#define X_GYRO_LOW   0x04      //0x04, 0x05  Output, x-axis gyroscope, low word 下面同理
#define X_GYRO_OUT   0x06     //0x06, 0x07   Output, x-axis gyroscope, high word
#define Y_GYRO_LOW   0x08     //0x08,0x09
#define Y_GYRO_OUT   0x0A     //0x0A,0x0B
#define Z_GYRO_LOW   0x0C     //0x0C,0x0D
#define Z_GYRO_OUT   0x0E     //0x0E,0x0F

#define X_ACCL_LOW   0x10    //0x10,0x11
#define X_ACCL_OUT   0x12    //0x12,0x13
#define Y_ACCL_LOW   0x14    //0x14,0x15
#define Y_ACCL_OUT   0x16    //0x16,0x17
#define Z_ACCL_LOW   0x18    //0x18,0x19
#define Z_ACCL_OUT   0x1A    //0x1A,0x1B

#define TEMP_OUT     0x1C    //0x1C,0x1D
#define TIME_STAMP   0x1E    //0x1E,0x1F Output, time stamp
//#define Reserved     0x20    //0x20,0x21,Reserved
#define DATA_CNTR    0x22    //0x22,0x23 New data counter

#define X_DELTANG_LOW 0x24   //0x24,0x25 Output, x-axis delta angle, low word
#define X_DELTANG_OUT 0x26   //0x26,0x27 Output, x-axis delta angle, high word
#define Y_DELTANG_LOW 0x28   //0x28,0x29
#define Y_DELTANG_OUT 0x2A   //0x2A,0x2B
#define Z_DELTANG_LOW 0x2C   //0x2C,0x2D
#define Z_DELTANG_OUT 0x2E   //0x2E,0X2F

#define X_DELTVEL_LOW 0x30   //0x30,0x31 Output, x-axis delta velocity, low word
#define X_DELTVEL_OUT 0x32   //0x32,0x33 Output, x-axis delta velocity, high word
#define Y_DELTVEL_LOW 0x34   //0x34,0x35
#define Y_DELTVEL_OUT 0x36   //0x37,0x38
#define Z_DELTVEL_LOW 0x38   //0x38,0x39
#define Z_DELTVEL_OUT 0x3A   //0x3A,0x3B

//#define Reserved  0X3C    //0X3C to 0X3F

#define XG_BIAS_LOW     0x40   //0x40,0x41     XG_BIAS_LOW->ZA_BIAS_HIGH,Defualt Value 0x0000 Calibration, offset, gyroscope, x-axis, low word
#define XG_BIAS_HIGH    0x42   //0x42,0x43     Calibration, offset, gyroscope, x-axis, high word
#define YG_BIAS_LOW     0x44   //0x44,0x45
#define YG_BIAS_HIGH    0x46   //0x46,0x44
#define ZG_BIAS_LOW     0x48   //0x48,0x49
#define ZG_BIAS_HIGH    0x4A   //0x4A,0x4B

#define XA_BIAS_LOW     0x4C   //0x4C,0x4D
#define XA_BIAS_HIGH    0x4E   //0x4E,0x4F
#define YA_BIAS_LOW     0x50   //0x450,0x51
#define YA_BIAS_HIGH    0x52   //0x52,0x53
#define ZA_BIAS_LOW     0x54   //0x54,0x55
#define ZA_BIAS_HIGH    0x56   //0x56,0x57

//#define Reserved    0x58   //0x58 to 0x5B

#define FILT_CTRL   0x5C   //0X5C,0X5D  Control, Bartlett window FIR filter  defualt 0x0000
#define RANG_MDL    0x5E   //0x5E,0x5F  Measurement range (model specific) identifier
#define MSC_CTRL    0x60   //0x60,0x61  Control, input/output and other miscellaneous option
#define UP_SCALE    0x62   //0x62,0x63  Control, scale factor for input clock, pulse per second (PPS) mode
#define DEC_RATE    0x64   //0x64,0x65  Control, decimation filter (output data rate)
#define NULL_CNFG   0x66   //0x66,0x67  Control, bias estimation period
#define GLOB_CMD    0x68   //0x68,0x69  Control, global commands
//#define Reserved    0x6A   //0x6A to 0x6B

#define FIRM_REV      0x6C   //0x6C,0x6D  Identification, firmware revision
#define FIRM_DM       0x6E   //0x6E,0x6F  Identification, date code, day and month
#define FIRM_Y        0x70   //0x70,0x71  Identification, date code, year
#define PROD_ID       0x72   //0x72,0x73  Identification, device number 0x4053
#define ADIS16467_ID1  0x4053  //device number 0x4053
#define SERIAL_NUM    0x74   //0x74,0x75  Identification, serial number
#define USER_SCR_1    0x76   //0x76,0x77  User Scratch Register 1
#define USER_SCR_2    0x78   //0x78,0x79  User Scratch Register 2
#define USER_SCR_3    0x7A   //0x7A,0x7B  User Scratch Register 3
#define FLSHCNT_LOW   0x7C   //0x7C,0x7D  Output, flash memory write cycle counter, lower word
#define FLSHCNT_HIGH  0x7E   //0x7E,0x7F  Output, flash memory write cycle counter, upper word

//#define SPI_DIR_READ  0x80
//#define SPI_DIR_WRITE 0x00

#define M_PI_F       3.1415926f
#define ADIS16467_ONE_G       (9.80665)   // 定义重力常数为9.80665 m/s?
//#define ADIS16467_TEMP_FACTOR 0.00294118f     //(1.0f / 340.0f)  每个原始单位对应的温度增量
//#define ADIS16467_TEMP_OFFSET 23.0f          //未测试过

//typedef struct {
//    uint8_t setbits;
//    uint8_t clearbits;
//} reg_val_t;

#define gyro_scale_factor  40*65536   //对应 1度/s
#define accel_scale_factor  1.25 / (65536 * 1000)    //1->1.25/2^16mg
#define temp_scale_factor0   10.0f     //0.1摄氏度->1
#define temp_scale_factor1   0.01220703     // 400/2^16,查阅手册可知


static rt_device_t ADIS16467_spi_dev;
struct rt_spi_device * ADIS16467_spi_dev0;
static float gyro_range_scale;
static float accel_range_scale;
static float sample_rate;

/* 通过校准得到的数据 */
static float gyro_offset[3];
//static float accel_offset[3];
static uint8_t cali_count;  // 校准次数
float ADIS16467_g_norm;        // 通过校准得出的重力加速度
float accel_scale;          // 根据标定结果校准加速度计标度因数
// 需定期校准后手动修改
#define GxOFFSET  -0.000031075f
#define GyOFFSET  -0.000007662f
#define GzOFFSET  -0.000004470f
#define gNORM 9.541743f

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void ADIS16467_gyro_rotate_to_frd(float* data)
{
    /* do nothing */
    (void)data;
}

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void ADIS16467_acc_rotate_to_frd(float* data)
{
    /* do nothing */
    (void)data;
}

static rt_err_t ADIS16467_burst_read(float accel[3],float gyro[3],float temp){

    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
    uint8_t checksum=0;
    uint8_t burst_raw[19] = {0};
//    uint8_t Burst_read_Cmd[2] = {0x00,0x68};
    static int16_t Send_Cmd=0x6800;//Burst Read指令


    rt_spi_send(ADIS16467_spi_dev0,&Send_Cmd,2);
    rt_spi_recv(ADIS16467_spi_dev0,&burst_raw,19);


    for(uint8_t i=0;i<18;i++){
        checksum += burst_raw[i];
    }if(checksum ==  burst_raw[18]){//通过和校验

        gyro[0] = (float)(burst_raw[2] << 8 | burst_raw[3]);
        gyro[1] = (float)(burst_raw[4] << 8 | burst_raw[5]);
        gyro[2] = (float)(burst_raw[6] << 8 | burst_raw[7]);

        accel[0] = (float)(burst_raw[8] << 8 | burst_raw[9]);
        accel[1] = (float)(burst_raw[10] << 8 | burst_raw[11]);
        accel[2] = (float)(burst_raw[12] << 8 | burst_raw[13]);

        temp = (float)(burst_raw[14] << 8 | burst_raw[15]);

    }


    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);

}

static rt_time_t gyro_read_raw(int32_t gyr[3])
{
    uint8_t buffer0[12];
    int16_t buffer1[6];
    spi_read_multi_reg8(ADIS16467_spi_dev, X_GYRO_LOW, (uint8_t*)buffer0, 12);
//    rt_hw_us_delay(1000);

    for(uint8_t i=0;i<6;i++){
        buffer1[i] = (buffer0[2*i+1] << 8) | buffer0[2*i];
    }

    gyr[0] = (buffer1[1] << 16) | buffer1[0];
    gyr[1] = (buffer1[3] << 16) | buffer1[2];
    gyr[2] = (buffer1[5] << 16) | buffer1[4];

    return RT_EOK;
}

static rt_err_t gyro_read_rad(float gyr[3])
{
    int32_t gyr_raw[3];

    gyro_read_raw(gyr_raw);


    gyr[0] = gyr_raw[0] / gyro_scale_factor;
    gyr[1] = gyr_raw[1] / gyro_scale_factor;
    gyr[2] = gyr_raw[2] / gyro_scale_factor;


    return RT_EOK;
}



static rt_err_t ADIS16467_Register_SET(void)
{
    uint8_t ADIS16467_id[2];
    uint16_t ADIS16467_ID0 ;
    /* init spi bus */
    rt_device_open(ADIS16467_spi_dev, RT_DEVICE_OFLAG_RDWR);
    /* read ADIS16467 id */
    spi_read_multi_reg8(ADIS16467_spi_dev, PROD_ID, (uint8_t *)ADIS16467_id,2);
    ADIS16467_ID0 = (ADIS16467_id[1] << 8 )| ADIS16467_id[0];
//    if (ADIS16467_ID0 != ADIS16467_ID1) {      //确认设备
//        LOG_W("Warning: not found ADIS16467 accel id: %02x", ADIS16467_id);
//        return RT_ERROR;
//    }


    spi_write_reg8(ADIS16467_spi_dev, GLOB_CMD, 0x80);     /* 软件重置重置设备*/
    spi_write_reg8(ADIS16467_spi_dev, (GLOB_CMD+0x01), 0x00);
    rt_hw_us_delay(200);

    spi_write_reg8(ADIS16467_spi_dev, FILT_CTRL, 0x05);     /* use filter*/
    spi_write_reg8(ADIS16467_spi_dev, (FILT_CTRL+0x01), 0x00);
    rt_hw_us_delay(200);

    spi_write_reg8(ADIS16467_spi_dev, DEC_RATE, 0x01);     /* 均值滤波器 set output rate  2000/(DEC_RATE+1) 1000Hz(now) */
    spi_write_reg8(ADIS16467_spi_dev, (DEC_RATE+0x01), 0x00);
    rt_hw_us_delay(200);

    spi_write_reg8(ADIS16467_spi_dev, NULL_CNFG, 0x0A);     /* 启动个轴accel和gyro的bias计算 */
    spi_write_reg8(ADIS16467_spi_dev, (NULL_CNFG+0x01), 0x3F);
    rt_hw_us_delay(200);

    spi_write_reg8(ADIS16467_spi_dev, USER_SCR_1, 0x05);     /* for user to store information */
    spi_write_reg8(ADIS16467_spi_dev, (USER_SCR_1+0x01), 0x00);
    rt_hw_us_delay(200);

    spi_write_reg8(ADIS16467_spi_dev, RANG_MDL, 0x07);     /* identify which model,此处为01 = ±500°/sec (ADIS16467-2BMLZ) */
    spi_write_reg8(ADIS16467_spi_dev, (RANG_MDL+0x01), 0x00);
    rt_hw_us_delay(200);

    spi_write_reg8(ADIS16467_spi_dev, GLOB_CMD, 0x07);     /* 各功能启动更新自检 更新bias_correction计算结果*/
    spi_write_reg8(ADIS16467_spi_dev, (GLOB_CMD+0x01), 0x00);
    rt_hw_us_delay(200);


    rt_hw_us_delay(200);


    return RT_EOK;
}


/**
 * @brief 读取ADIS16467的加速度计数据
 *
 * @param data 读取到的数据
 * @return rt_err_t
 */
static rt_err_t ADIS16467_gyro_read(float data[3])
{
    if (gyro_read_rad(data) != RT_EOK) {
        return -RT_ERROR;
    }
    // change to NED coordinate
    ADIS16467_gyro_rotate_to_frd(data);

    return RT_EOK;
}



static rt_time_t accel_read_raw(int32_t  acc[3])
{
    /* In case of read operations of the accelerometer part, the requested data is not sent
    immediately, but instead first a dummy byte is sent, and after this dummy byte the actual
    reqested register content is transmitted. */

    uint8_t buffer0[12];
    int16_t buffer1[6];
    spi_read_multi_reg8(ADIS16467_spi_dev, X_ACCL_LOW, (uint8_t*)buffer0, 12);
//    rt_hw_us_delay(1000);

    for(uint8_t i=0;i<6;i++){
        buffer1[i] = buffer0[2*i+1] << 8 | buffer0[2*i];
    }

//    acc[0] = (int32_t )((buffer1[1] << 16) | buffer1[0]);
//    acc[1] = (int32_t)((buffer1[3] << 16) | buffer1[2]);
//    acc[2] = (int32_t)((buffer1[5] << 16) | buffer1[4]);

    acc[0] = buffer1[1] << 16 | buffer1[0];
    acc[1] = buffer1[3] << 16 | buffer1[2];
    acc[2] = buffer1[5] << 16 | buffer1[4];


    return RT_EOK;
}


static rt_err_t accel_read_m_s2(float acc[3])
{
    int32_t  acc_raw[3];

    accel_read_raw(acc_raw);

    acc[0] = acc_raw[0] * accel_scale_factor ;
    acc[1] = acc_raw[1] * accel_scale_factor ;
    acc[2] = acc_raw[2] * accel_scale_factor ;


    return RT_EOK;
}

/**
 * @brief 读取ADIS16467的陀螺仪数据
 *
 * @param data 读取到的数据
 * @return rt_err_t
 */
static rt_err_t ADIS16467_accel_read(float data[3])
{
    if (accel_read_m_s2(data) != RT_EOK) {
        return -RT_ERROR;;
    }
    // change to NED coordinate
    ADIS16467_acc_rotate_to_frd(data);

    return RT_EOK;
}



/**
 * @brief 读取ADIS16467的温度
 *
 * @return float 温度（摄氏度）
 */
static float ADIS16467_temp_read(void)
{
    uint8_t buffer[2];
    static int16_t raw_temp;
    static float temp;

    spi_read_multi_reg8(ADIS16467_spi_dev, TEMP_OUT, (uint8_t *)buffer, 2);
    raw_temp = (int16_t)((buffer[1] << 8) | buffer[0]);

    if(raw_temp >= 0){
        temp = raw_temp / temp_scale_factor0;
    }else{
        temp = raw_temp * temp_scale_factor1;
    }

    return temp;
}

/* -------------------------------- CALIBRATE ------------------------------- */
/**
 * @brief ADIS16467 校准函数
 *
 * @note 定期或更换开发板时进行一次校准即可,校准成功后手动修改 GxOFFSET 等宏;
 *       通过在 rtconfig 中使能 ADIS16467_CALI 进行校准;
 *       在串口终端可以查看校准进度,如多次校准失败,适当调大误差范围;
 *
 * @return 校准成功将校准值写入 gyro_offset
 */
static void ADIS16467_calibrate(void){
    static float start_time;
    static uint16_t cali_times = 5000;   // 需要足够多的数据才能得到有效陀螺仪零偏校准结果
    float accel[3], gyro[3];
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;
    static float gyroDiff[3], gNormDiff;
    int32_t  acc_raw[3];

    start_time = dwt_get_time_s();
    do
    {
        if (dwt_get_time_s() - start_time > 20)
        {
            // 校准超时
/*            gyro_offset[0] = GxOFFSET;
            gyro_offset[1] = GyOFFSET;
            gyro_offset[2] = GzOFFSET;
            bmi088_g_norm = gNORM;*/
            break;
        }

        dwt_delay_s(0.005);
        // 开始时先置零，避免对数据读取造成影响
        ADIS16467_g_norm = 0;
        gyro_offset[0] = 0;
        gyro_offset[1] = 0;
        gyro_offset[2] = 0;

        for (uint16_t i = 0; i < cali_times; i++)
        {
            accel_read_raw(acc_raw);
            accel[0] = accel_scale_factor * acc_raw[0];
            accel[1] = accel_scale_factor * acc_raw[1];
            accel[2] = accel_scale_factor * acc_raw[2];
            gNormTemp = sqrtf(accel[0] * accel[0] +
                              accel[1] * accel[1] +
                              accel[2] * accel[2]);

            ADIS16467_g_norm += gNormTemp;

            gyro_read_rad(gyro);
            for(uint8_t j = 0; j < 3; j++){
                gyro_offset[j] += gyro[j];
            }

            // 记录数据极差
            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = gyro[j];
                    gyroMin[j] = gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax)
                    gNormMax = gNormTemp;
                if (gNormTemp < gNormMin)
                    gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (gyro[j] > gyroMax[j])
                        gyroMax[j] = gyro[j];
                    if (gyro[j] < gyroMin[j])
                        gyroMin[j] = gyro[j];
                }
            }

            // 数据差异过大认为收到外界干扰，需重新校准
            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];
            if (gNormDiff > 0.6f ||
                gyroDiff[0] > 1.0f ||
                gyroDiff[1] > 1.0f ||
                gyroDiff[2] > 1.0f)
                break;
            LOG_I("gyroDiff: %f",gNormDiff);
            for(uint8_t j = 0; j < 3; j++){
                LOG_D("gyroDiff%d: %f",j ,gyroDiff[j]);
            }
            dwt_delay_s(0.0005);
        }
        // 取平均值得到标定结果
        ADIS16467_g_norm /= (float)cali_times ;
        LOG_W("ADIS16467_g_norm: %f",ADIS16467_g_norm);
        for (uint8_t i = 0; i < 3; i++)
        {
            gyro_offset[i] /= (float)cali_times;
            LOG_W("gyro_offset: %f",gyro_offset[i]);
        }

        cali_count++;
    } while (gNormDiff > 0.3f ||
             fabsf(ADIS16467_g_norm - 9.8f) > 0.5f ||
             gyroDiff[0] > 1.0f ||
             gyroDiff[1] > 1.0f ||
             gyroDiff[2] > 1.0f ||
             fabsf(gyro_offset[0]) > 0.01f ||
             fabsf(gyro_offset[1]) > 0.01f ||
             fabsf(gyro_offset[2]) > 0.01f);
    // 根据标定结果校准加速度计标度因数
    accel_scale = 9.81f / ADIS16467_g_norm;
}
/* -------------------------------- CALIBRATE ------------------------------- */


/**
 * @brief 初始化ADIS16467
 *
 * @return rt_err_t
 */
static rt_err_t ADIS16467_init(void)
{
    /* Initialize ADIS16467 */
    rt_hw_spi_device_attach(SPI_ADIS16467, "ADIS16467", SPI_ADIS16467_CS);
    ADIS16467_spi_dev0 = (struct rt_spi_device *)rt_device_find("ADIS16467");
    ADIS16467_spi_dev = rt_device_find("ADIS16467");
    RT_ASSERT(ADIS16467_spi_dev != NULL);//指针不为空,找到spi4总线上挂载的ADIS16467
    /* config spi 配置spi参数*/
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */
        cfg.max_hz = 1000000;

        struct rt_spi_device* spi_device_t = (struct rt_spi_device*)ADIS16467_spi_dev;
        spi_device_t->config.data_width = cfg.data_width;
        spi_device_t->config.mode = cfg.mode & RT_SPI_MODE_MASK;
        spi_device_t->config.max_hz = cfg.max_hz;

        rt_spi_configure(spi_device_t, &cfg);
    }

    ADIS16467_Register_SET();

    gyro_offset[0] = GxOFFSET;
    gyro_offset[1] = GyOFFSET;
    gyro_offset[2] = GzOFFSET;

    ADIS16467_g_norm = gNORM;
    accel_scale = 9.81f / ADIS16467_g_norm;
    /* calibrate */
#ifdef BSP_ADIS16467_CALI
    ADIS16467_calibrate();
#endif /* BSP_ADIS16467_CALI */

    return RT_EOK;
}


struct imu_ops imu_ops = {
        .imu_init = ADIS16467_init,
        .gyro_read = ADIS16467_gyro_read,
        .accel_read = ADIS16467_accel_read,
        .burst_read = ADIS16467_burst_read,
        .temp_read = ADIS16467_temp_read,
};