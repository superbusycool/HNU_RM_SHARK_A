//
// Created by SuperChen on 2024/10/15.
//

#include "A6500.h"
#include "drv_dwt.h"
#include "rm_config.h"
#include <drv_spi.h>
#include <math.h>
#include <rtdbg.h>

//#define DBG_TAG   "imu.bmi088"
//#define DBG_LVL DBG_LOG
//#include <rtdbg.h>
//
///*********************************************************************************/
//#ifdef BIT
//#undef BIT
//#endif
//
//#define BIT(_idx) (1 << _idx)
//#define REG_VAL(_setbits, _clearbits) \
//    (reg_val_t) { .setbits = (_setbits), .clearbits = (_clearbits) }
///**********************************************************************************/    //�Ĵ�����λ�йغ���,����֮��������ع��ܼĴ������

/*�������Ĵ����е�ֵ��ʾ��������Թ����в������Բ����������ֵ���ڼ�������û�ִ�еĺ����Բ������*/
#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)

/*�������Ĵ���������������������е�ֱ��ƫ�á��ڽ��봫�����Ĵ���֮ǰ�����˼Ĵ����е�ֵ��ӵ������Ǵ�����ֵ�С�*/
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)

/*�ĸ����üĴ���˵����������ʾ*/
#define MPU6500_CONFIG              (0x1A)      // MPU-6500���üĴ���
#define MPU6500_GYRO_CONFIG         (0x1B)     // MPU-6500���������üĴ���
#define MPU6500_ACC_PWR_CONF        (0x1C)     //MPU6500_ACCEL_CONFIG


/*�͹��ʼ��ٶȼ�ODR���ƼĴ���*/
#define MPU6500_LP_ACCEL_ODR        (0x1E)

/*�˼Ĵ�������x/y/z�жϷ���ֵ*/
#define MPU6500_MOT_THR             (0x1F)

/*FIFOʹ�ܼĴ���������1���򽫶�Ӧ�����Բ���Ƶ��д��FIFO*/
#define MPU6500_FIFO_EN             (0x23)

/*IIC���豸��������������*/
#define MPU6500_I2C_MST_CTRL        (0x24)

/*IIC���豸��ؼĴ���*/
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)

/*IIC  ���豸״̬�Ĵ���*/
#define MPU6500_I2C_MST_STATUS      (0x36)
/*�����ж���ؼĴ���*/
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)

/*��14���Ĵ����洢���ٶȡ������ǡ��¶ȵ�ԭʼ����*/
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)

/*��24���Ĵ����洢IIC���豸��0��1��2��3)ͨ������IIC�ӿڣ����ⲿ��������ȡ������
�ӻ��豸4��ȡ�����ݴ����I2C_SLV4_DI�У��Ĵ���53��*/
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)

/*IIC���豸��������Ĵ���*/
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)

#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_BGW_SOFT_RST   (0x68)      //Mpu6500��λ�Ĵ���
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)

/*��Դ����Ĵ�������������MPU6500ʱ��Դ�����ƴ�����ʧ�ܵ�*/
#define MPU6500_PWR_MGMT_1         (0x6B)           //���ڿ����豸�ĵ�Դ״̬����������
#define MPU6500_ACC_PWR_CTRL          (0x6C)        //���Ƹ���������ͨ��MPU6500_PWR_MGMT_2

/*��¼д�뵽FIFO���ֽ���*/
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)

/*���ڴ�FIFO��������д����*/
#define MPU6500_FIFO_R_W            (0x74)

/*�洢һ��8λ���ݣ�������֤�豸�ı�ʾ*/
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x70

/*�������Ĵ��������������ٶȼ�����е�ֱ��ƫ�á��ڽ��봫�����Ĵ���֮ǰ�����˼Ĵ����е�ֵ��ӵ����ٶȼƴ�����ֵ�С�*/
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)

#define MPU6500_ID					(0x70)
#define MPU_IIC_ADDR				0x68


#define MPU6500_ACC_BGW_CHIPID_VALUE 0x71     //mpu6500��ACC�Ĵ�������ֵ
#define MPU6500_ACC_BGW_CHIPID       0x75    //mpu6500ID��ȡ��ַ


#define MPU6500_Gyro_BGW_CHIPID_VALUE    0x70  //mpu6500��Gyro�Ĵ�������ֵ
#define MPU6500_GRRO_BGW_CHIPID            0x75

#define MPU6500_ONE_G       (9.80665)   // ������������Ϊ9.80665 m/s?

/*�Ѳ���mpu6500_datasheet*/
#define MPU6500_ACCEL_RANGE_2G  0x00
#define MPU6500_ACCEL_RANGE_4G  0x01
#define MPU6500_ACCEL_RANGE_8G  0x10
#define MPU6500_ACCEL_RANGE_16G 0x11

/*�Ѳ���mpu6500_datasheet*/
/*�����ڲ�������(see register CONFIG)���ɿ��ƴ���������������ʵĲ����ʣ�FIFO������.�˼Ĵ���ֻ����FCHOICE=2��b11(FCHOICE_B�Ĵ���λΪ2��b00)��(0<DLPF_CFG<7)ʱ����Ч������=
�ڲ�������/(1+SMPLRT_DIV),�ڲ�������=1 kHz*/
#define MPU6500_SMPLRT_DIV      (0x19)
#define MPU6500_ACCEL_RATE_10   0x64    // 10 Hz
#define MPU6500_ACCEL_RATE_25   0x28    // 25 Hz
#define MPU6500_ACCEL_RATE_50   0x13   // 50 Hz
#define MPU6500_ACCEL_RATE_100  0x09   // 100 Hz
#define MPU6500_ACCEL_RATE_125  0x07   // 125 Hz
#define MPU6500_ACCEL_RATE_200  0x04   // 200 Hz
#define MPU6500_ACCEL_RATE_250  0x03   // 250 Hz
#define MPU6500_ACCEL_RATE_500  0x01   // 500 Hz
#define MPU6500_ACCEL_RATE_1000 0x00  // 1000 Hz

#define MPU6500_ACCEL_CONFIG       0x1C  // ���ٶȼ����üĴ���
#define MPU6500_DLPF_CFG           0x1D  // ���ֵ�ͨ�˲������üĴ���MPU6500_ACCEL_CONFIG_2

/*�Ѳ���mpu6500_datasheet*/
#define MPU6500_BW_460_HZ         0x00  // 460Hz
#define MPU6500_BW_184_HZ         0x01  // 184Hz
#define MPU6500_BW_92_HZ          0x02  // 92Hz
#define MPU6500_BW_41_HZ          0x03  // 41Hz
#define MPU6500_BW_20_HZ          0x04  // 20Hz
#define MPU6500_BW_10_HZ          0x05  // 10Hz
#define MPU6500_BW_5_HZ           0x06  // 5Hz

/*�Ѳ���mpu6500_datasheet*/
#define MPU6500_GYRO_RANGE_250_DPS  0x00  // ��250 dps
#define MPU6500_GYRO_RANGE_500_DPS  0x08  // ��500 dps
#define MPU6500_GYRO_RANGE_1000_DPS 0x10  // ��1000 dps
#define MPU6500_GYRO_RANGE_2000_DPS 0x18  // ��2000 dps

/*�Ѳ���mpu6500_datasheet*/
#define MPU6500_GYRO_BW_250      0x00  // 250Hz����
#define MPU6500_GYRO_BW_184      0x01  // 184Hz����
#define MPU6500_GYRO_BW_92       0x02  // 92Hz����
#define MPU6500_GYRO_BW_41       0x03  // 41Hz����
#define MPU6500_GYRO_BW_20       0x04  // 20Hz����
#define MPU6500_GYRO_BW_10       0x05  // 10Hz����
#define MPU6500_GYRO_BW_5        0x06  // 5Hz����

#define M_PI_F       3.1415926f
#define MPU6500_TEMP_FACTOR 0.00294118f     //(1.0f / 340.0f)  ÿ��ԭʼ��λ��Ӧ���¶�����
#define MPU6500_TEMP_OFFSET 36.53f          //δ���Թ�

typedef struct {
    uint8_t setbits;
    uint8_t clearbits;
} reg_val_t;

static rt_device_t gyro_spi_dev;
static rt_device_t accel_spi_dev;
static float gyro_range_scale;
static float accel_range_scale;
static float sample_rate;

/* ͨ��У׼�õ������� */
static float gyro_offset[3];
static uint8_t cali_count;  // У׼����
float MPU6500_g_norm;        // ͨ��У׼�ó����������ٶ�
float accel_scale;          // ���ݱ궨���У׼���ٶȼƱ������
// �趨��У׼���ֶ��޸�
#define GxOFFSET  0.00000127707f
#define GyOFFSET -0.00000808811f
#define GzOFFSET -0.00002852123f
#define gNORM 9.744925f

/* Re-implement this function to define customized rotation */
__attribute__((weak)) void MPU6500_gyro_rotate_to_frd(float* data)
{
    /* do nothing */
    (void)data;
}
static rt_err_t gyro_read_raw(int16_t gyr[3])
{
    uint8_t buffer[6];
    spi_read_multi_reg8(gyro_spi_dev, MPU6500_GYRO_XOUT_H, (uint8_t*)gyr, 6);

    gyr[0] = buffer[0] << 8 | buffer[1];
    gyr[1] = buffer[2] << 8 | buffer[3];
    gyr[2] = buffer[4] << 8 | buffer[5];

    return RT_EOK;
}

static rt_err_t gyro_read_rad(float gyr[3])
{
    int16_t gyr_raw[3];

    gyro_read_raw(gyr_raw);

    gyr[0] = gyro_range_scale * gyr_raw[0] - gyro_offset[0];
    gyr[1] = gyro_range_scale * gyr_raw[1] - gyro_offset[1];
    gyr[2] = gyro_range_scale * gyr_raw[2] - gyro_offset[2];

    return RT_EOK;
}


static rt_err_t accel_set_range(uint32_t max_g)
{
    uint8_t reg_val;

    // Ĭ�����ΧΪ16G
    if (max_g == 0) {
        max_g = 16;
    }

    // ��������ֵ������Ӧ�ļĴ���ֵ�ͱ�������
    if (max_g <= 2) {
        reg_val = MPU6500_ACCEL_RANGE_2G;
        accel_range_scale = (2 * MPU6500_ONE_G / 32768);
    } else if (max_g <= 4) {
        reg_val = MPU6500_ACCEL_RANGE_4G;
        accel_range_scale = (4 * MPU6500_ONE_G / 32768);
    } else if (max_g <= 8) {
        reg_val = MPU6500_ACCEL_RANGE_8G;
        accel_range_scale = (8 * MPU6500_ONE_G / 32768);
    } else if (max_g <= 16) {
        reg_val = MPU6500_ACCEL_RANGE_16G;
        accel_range_scale = (16 * MPU6500_ONE_G / 32768);
    } else {
        return RT_EINVAL;  // ���ز�����Ч
    }

    // ��MPU-6500�ļ��ٶȷ�Χ�Ĵ���д��ֵ
    spi_write_reg8(accel_spi_dev, MPU6500_ACC_PWR_CONF, reg_val);
    return RT_EOK;
}

static rt_err_t accel_set_sample_rate(uint32_t frequency_hz)
{
    uint8_t reg_val;

    // ��������Ƶ�����ö�Ӧ�ļĴ���ֵ
    if (frequency_hz <= 10) {
        reg_val = MPU6500_ACCEL_RATE_10;
        sample_rate = 10;
    } else if (frequency_hz <= 25) {
        reg_val = MPU6500_ACCEL_RATE_25;
        sample_rate = 25;
    } else if (frequency_hz <= 50) {
        reg_val = MPU6500_ACCEL_RATE_50;
        sample_rate = 50;
    } else if (frequency_hz <= 100) {
        reg_val = MPU6500_ACCEL_RATE_100;
        sample_rate = 100;
    } else if (frequency_hz <= 125) {
        reg_val = MPU6500_ACCEL_RATE_125;
        sample_rate = 125;
    }else if (frequency_hz <= 200) {
        reg_val = MPU6500_ACCEL_RATE_200;
        sample_rate = 200;
    }else if (frequency_hz <= 250) {
        reg_val = MPU6500_ACCEL_RATE_250;
        sample_rate = 250;
    } else if (frequency_hz <= 500) {
        reg_val = MPU6500_ACCEL_RATE_500;
        sample_rate = 500;
    } else if (frequency_hz <= 1000) {
        reg_val = MPU6500_ACCEL_RATE_1000;
        sample_rate = 1000;
    } else {
        return -RT_EINVAL; // ������Ч����
    }

    // ���ò���Ƶ��
    spi_write_reg8(accel_spi_dev, MPU6500_SMPLRT_DIV, reg_val);

    return RT_EOK;
}

static rt_err_t accel_set_bwp_odr(uint16_t dlpf_freq_hz)
{
    uint8_t dlpf_val;

    // ���������DLPFƵ�����üĴ���ֵ
    if (sample_rate <= 10) {
        dlpf_val = MPU6500_BW_460_HZ;  // ����250Hz����
    } else if (sample_rate <= 25) {
        dlpf_val = MPU6500_BW_184_HZ;  // ����184Hz����
    } else if (sample_rate <= 50) {
        dlpf_val = MPU6500_BW_92_HZ;   // ����92Hz����
    } else if (sample_rate <= 100) {
        dlpf_val = MPU6500_BW_41_HZ;   // ����41Hz����
    } else if (sample_rate <= 125) {
        dlpf_val = MPU6500_BW_20_HZ;   // ����20Hz����
    } else if (sample_rate <= 250) {
        dlpf_val = MPU6500_BW_10_HZ;   // ����10Hz����
    } else if (sample_rate <= 500) {
        dlpf_val = MPU6500_BW_5_HZ;    // ����5Hz����
    } else {
        return -RT_EINVAL; // ��Ч����
    }

    spi_write_reg8(accel_spi_dev, MPU6500_CONFIG, dlpf_val);
    return RT_EOK;
}

static rt_err_t accelerometer_init(void)
{
    uint8_t accel_id;

    /* init spi bus */
    rt_device_open(accel_spi_dev, RT_DEVICE_OFLAG_RDWR);

    /* dummy read to let accel enter SPI mode */
    spi_read_reg8(accel_spi_dev, MPU6500_ACC_BGW_CHIPID, &accel_id);
    rt_hw_us_delay(1000);
    spi_read_reg8(accel_spi_dev, MPU6500_ACC_BGW_CHIPID, &accel_id);

    /* read accel id */
    spi_read_reg8(accel_spi_dev, MPU6500_ACC_BGW_CHIPID, &accel_id);
    if (accel_id != MPU6500_ACC_BGW_CHIPID_VALUE) {
        LOG_W("Warning: not found MPU6500 accel id: %02x", accel_id);
        return RT_ERROR;
    }
    /* hardware reset */
    spi_write_reg8(accel_spi_dev, MPU6500_PWR_MGMT_1, 0x80);     /* �����豸*/
    rt_hw_us_delay(2000);
    /* soft reset */
    spi_write_reg8(accel_spi_dev, MPU6500_BGW_SOFT_RST, 0x07);     /* ��������豸*/
    rt_hw_us_delay(2000);
    /* dummy read to let accel enter SPI mode */
    spi_read_reg8(accel_spi_dev, MPU6500_ACC_BGW_CHIPID, &accel_id);
    /* enter normal mode */
    spi_write_reg8(accel_spi_dev, MPU6500_ACC_PWR_CTRL, 0x00);      /* ���� Acc & Gyro */
    rt_hw_us_delay(55000);

    /* set default range and bandwidth */
    accel_set_range(6);          /* 6g */
    accel_set_sample_rate(800);  /* 800Hz sample rate */
    accel_set_bwp_odr(280);      /* Normal BW */

    /* enter active mode */
    spi_write_reg8(accel_spi_dev, MPU6500_ACC_PWR_CONF, 0x10);    /* +-8G */
    rt_hw_us_delay(1000);

    return RT_EOK;
}


static rt_err_t gyro_set_range(unsigned max_dps)
{
    uint8_t reg_val;
    float lsb_per_dps;

    if (max_dps == 0) {
        max_dps = 2000;  // Ĭ��ֵ
    }

    if (max_dps <= 250) {
        reg_val = MPU6500_GYRO_RANGE_250_DPS;
        lsb_per_dps = 131.2;  //
    } else if (max_dps <= 500) {
        reg_val = MPU6500_GYRO_RANGE_500_DPS;
        lsb_per_dps = 65.5;   // ÿdps��LSBֵ
    } else if (max_dps <= 1000) {
        reg_val = MPU6500_GYRO_RANGE_1000_DPS;
        lsb_per_dps = 32.8;   // ÿdps��LSBֵ
    } else if (max_dps <= 2000) {
        reg_val = MPU6500_GYRO_RANGE_2000_DPS;
        lsb_per_dps = 16.4;   // ÿdps��LSBֵ
    } else {
        return RT_EINVAL;  // ������Ч����
    }

    // д�����������üĴ���
    spi_write_reg8(gyro_spi_dev, MPU6500_GYRO_CONFIG, reg_val);

    // ����ÿ��LSB����Ķ������Ի���Ϊ��λ��
    gyro_range_scale = (M_PI_F / (180.0f * lsb_per_dps));

    return RT_EOK;
}

static rt_err_t gyro_set_sample_rate(uint32_t frequency_hz)
{
    uint8_t reg_val;

    // ���������Ƶ������DLPF�Ĵ���
    if (frequency_hz <= 10) {
        reg_val = MPU6500_GYRO_BW_5;    // ����5Hz����
    } else if (frequency_hz <= 20) {
        reg_val = MPU6500_GYRO_BW_10;   // ����10Hz����
    } else if (frequency_hz <= 41) {
        reg_val = MPU6500_GYRO_BW_20;   // ����20Hz����
    } else if (frequency_hz <= 92) {
        reg_val = MPU6500_GYRO_BW_41;   // ����41Hz����
    } else if (frequency_hz <= 184) {
        reg_val = MPU6500_GYRO_BW_92;   // ����92Hz����
    } else if (frequency_hz <= 250) {
        reg_val = MPU6500_GYRO_BW_184;  // ����184Hz����
    } else if (frequency_hz <= 1000) {
        reg_val = MPU6500_GYRO_BW_250;  // ����250Hz����
    } else {
        return RT_EINVAL;  // ������Ч����
    }

    // д�����������üĴ��������ò����ʺʹ���
    spi_write_reg8(gyro_spi_dev, MPU6500_CONFIG, reg_val);

    return RT_EOK;
}

static rt_err_t gyroscope_init(void)
{
    uint8_t gyro_id;

    /* ��ʼ��SPI���� */
    rt_device_open(gyro_spi_dev, RT_DEVICE_OFLAG_RDWR);

    /* ��ȡоƬID */
    spi_read_reg8(gyro_spi_dev, MPU6500_GRRO_BGW_CHIPID, &gyro_id);
    if (gyro_id != MPU6500_Gyro_BGW_CHIPID_VALUE) {
        LOG_W("Warning: not found MPU6500 gyro id: %02x", gyro_id);
        return RT_ERROR;
    }

    /* ִ����λ */
    spi_write_reg8(gyro_spi_dev, MPU6500_BGW_SOFT_RST, 0x80); // ������λ����
    rt_hw_us_delay(35000); // �ȴ�����30ms

    gyro_set_range(2000);       /* 2000dps */
    gyro_set_sample_rate(2000); /* OSR 2000KHz, Filter BW: 230Hz */

    /* enable gyroscope */
    spi_write_reg8(gyro_spi_dev, MPU6500_ACC_PWR_CTRL, 0x00);  /* ���� Acc & Gyro */
    rt_hw_us_delay(1000);

    return RT_EOK;
}

/**
 * @brief ��ʼ��MPU6500
 *
 * @return rt_err_t
 */
static rt_err_t MPU6500_init(void)
{
    /* Initialize accelerometer */
    rt_hw_spi_device_attach(SPI_ACC, "MPU6500_a", SPI_ACC_CS);
    accel_spi_dev = rt_device_find("MPU6500_a");
    RT_ASSERT(accel_spi_dev != NULL);
    /* config spi ����spi����*/
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */
        cfg.max_hz = 7000000;

        struct rt_spi_device* spi_device_t = (struct rt_spi_device*)accel_spi_dev;
        spi_device_t->config.data_width = cfg.data_width;
        spi_device_t->config.mode = cfg.mode & RT_SPI_MODE_MASK;
        spi_device_t->config.max_hz = cfg.max_hz;

        rt_spi_configure(spi_device_t, &cfg);
    }
    /* accelerometer low-level init */
    accelerometer_init();

    /* Initialize gyroscope */
    rt_hw_spi_device_attach(SPI_GYRO, "MPU6500_g", SPI_GYRO_CS);
    gyro_spi_dev = rt_device_find("MPU6500_g");
    RT_ASSERT(gyro_spi_dev != NULL);
    /* config spi */
    {
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB; /* SPI Compatible Modes 3 */
        cfg.max_hz = 7000000;

        struct rt_spi_device* spi_device_t = (struct rt_spi_device*)gyro_spi_dev;
        spi_device_t->config.data_width = cfg.data_width;
        spi_device_t->config.mode = cfg.mode & RT_SPI_MODE_MASK;
        spi_device_t->config.max_hz = cfg.max_hz;

        rt_spi_configure(spi_device_t, &cfg);
    }
    /* gyroscope low-level init */
    gyroscope_init();
    gyro_offset[0] = GxOFFSET;
    gyro_offset[1] = GyOFFSET;
    gyro_offset[2] = GzOFFSET;
    MPU6500_g_norm = gNORM;
    accel_scale = 9.81f / MPU6500_g_norm;
    /* calibrate */
#ifdef BSP_MPU6500_CALI
    MPU6500_calibrate();
#endif /* BSP_MPU6500_CALI */

    return RT_EOK;
}

/**
 * @brief ��ȡMPU6500�ļ��ٶȼ�����
 *
 * @param data ��ȡ��������
 * @return rt_err_t
 */
static rt_err_t MPU6500_gyro_read(float data[3])
{
    if (gyro_read_rad(data) != RT_EOK) {
        return -RT_ERROR;
    }
    // change to NED coordinate
    MPU6500_gyro_rotate_to_frd(data);

    return RT_EOK;
}


static rt_err_t gyro_set_dlpf_filter(uint16_t frequency_hz)
{
    /* lpf bw is set by MPU6500_BW_ADDR */
    (void)frequency_hz;

    return RT_EOK;
}

/**
 * @brief ����MPU6500������������
 *
 * @param cfg ���ò���
 * @return rt_err_t
 */
static rt_err_t MPU6500_gyro_config(struct gyro_configure cfg)
{
    gyro_set_sample_rate(cfg.sample_rate_hz);
    gyro_set_dlpf_filter(cfg.dlpf_freq_hz);
    gyro_set_range(cfg.gyro_range_dps);

    return RT_EOK;
}

static rt_err_t accel_read_raw(int16_t acc[3])
{
    uint8_t buffer[7];

    /* In case of read operations of the accelerometer part, the requested data is not sent
    immediately, but instead first a dummy byte is sent, and after this dummy byte the actual
    reqested register content is transmitted. */
    spi_read_multi_reg8(accel_spi_dev, MPU6500_ACCEL_XOUT_H, buffer, 7);

    acc[0] = buffer[1] << 8 | buffer[2];
    acc[1] = buffer[3] << 8 | buffer[4];
    acc[2] = buffer[5] << 8 | buffer[6];

    return RT_EOK;
}


static rt_err_t accel_read_m_s2(float acc[3])
{
    int16_t acc_raw[3];

    accel_read_raw(acc_raw);

    acc[0] = accel_range_scale * acc_raw[0] * accel_scale;
    acc[1] = accel_range_scale * acc_raw[1] * accel_scale;
    acc[2] = accel_range_scale * acc_raw[2] * accel_scale;

    return RT_EOK;
}

/**
 * @brief ��ȡMPU6500������������
 *
 * @param data ��ȡ��������
 * @return rt_err_t
 */
static rt_err_t MPU6500_accel_read(float data[3])
{
    if (accel_read_m_s2(data) != RT_EOK) {
        return -RT_ERROR;;
    }
    // change to NED coordinate
    MPU6500_acc_rotate_to_frd(data);

    return RT_EOK;
}

/**
 * @brief ����MPU6500�ļ��ٶȼ�����
 *
 * @param cfg ���ò���
 * @return rt_err_t
 */
static  rt_err_t MPU6500_accel_config(struct accel_configure cfg)
{
    accel_set_sample_rate(cfg.sample_rate_hz);
    accel_set_bwp_odr(cfg.dlpf_freq_hz);
    accel_set_range(cfg.acc_range_g);

    return RT_EOK;
}

/**
 * @brief ��ȡMPU6500���¶�
 *
 * @return float �¶ȣ����϶ȣ�
 */
static float MPU6500_temp_read(void)
{
    uint8_t buffer[2];
    static int16_t raw_temp;
    static float temp;

    spi_read_multi_reg8(accel_spi_dev, MPU6500_TEMP_OUT_H, buffer, 2);
    raw_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    if (raw_temp > 1023)
    {
        raw_temp -= 2048;
    }
    temp = raw_temp * MPU6500_TEMP_FACTOR + MPU6500_TEMP_OFFSET;

    return temp;
}

/* -------------------------------- CALIBRATE ------------------------------- */
/**
 * @brief MPU6500 У׼����
 *
 * @note ���ڻ����������ʱ����һ��У׼����,У׼�ɹ����ֶ��޸� GxOFFSET �Ⱥ�;
 *       ͨ���� menuconfig ��ʹ�� MPU6500_CALI ����У׼;
 *       �ڴ����ն˿��Բ鿴У׼����,����У׼ʧ��,�ʵ�������Χ;
 *
 * @return У׼�ɹ���У׼ֵд�� gyro_offset
 */
static void MPU6500_calibrate(void){
    static float start_time;
    static uint16_t cali_times = 5000;   // ��Ҫ�㹻������ݲ��ܵõ���Ч��������ƫУ׼���
    float accel[3], gyro[3];
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;
    static float gyroDiff[3], gNormDiff;
    int16_t acc_raw[3];

    start_time = dwt_get_time_s();
    do
    {
        if (dwt_get_time_s() - start_time > 20)
        {
            // У׼��ʱ
/*            gyro_offset[0] = GxOFFSET;
            gyro_offset[1] = GyOFFSET;
            gyro_offset[2] = GzOFFSET;
            bmi088_g_norm = gNORM;*/
            break;
        }

        dwt_delay_s(0.005);
        // ��ʼʱ�����㣬��������ݶ�ȡ���Ӱ��
        MPU6500_g_norm = 0;
        gyro_offset[0] = 0;
        gyro_offset[1] = 0;
        gyro_offset[2] = 0;

        for (uint16_t i = 0; i < cali_times; i++)
        {
            accel_read_raw(acc_raw);
            accel[0] = accel_range_scale * acc_raw[0];
            accel[1] = accel_range_scale * acc_raw[1];
            accel[2] = accel_range_scale * acc_raw[2];
            gNormTemp = sqrtf(accel[0] * accel[0] +
                              accel[1] * accel[1] +
                              accel[2] * accel[2]);
            MPU6500_g_norm += gNormTemp;

            gyro_read_rad(gyro);
            for(uint8_t j = 0; j < 3; j++){
                gyro_offset[j] += gyro[j];
            }

            // ��¼���ݼ���
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

            // ���ݲ��������Ϊ�յ������ţ�������У׼
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
        // ȡƽ��ֵ�õ��궨���
        MPU6500_g_norm /= (float)cali_times;
        LOG_W("MPU6500_g_norm: %f",MPU6500_g_norm);
        for (uint8_t i = 0; i < 3; i++)
        {
            gyro_offset[i] /= (float)cali_times;
            LOG_W("gyro_offset: %f",gyro_offset[i]);
        }

        cali_count++;
    } while (gNormDiff > 0.3f ||
             fabsf(MPU6500_g_norm - 9.8f) > 0.5f ||
             gyroDiff[0] > 1.0f ||
             gyroDiff[1] > 1.0f ||
             gyroDiff[2] > 1.0f ||
             fabsf(gyro_offset[0]) > 0.01f ||
             fabsf(gyro_offset[1]) > 0.01f ||
             fabsf(gyro_offset[2]) > 0.01f);
    // ���ݱ궨���У׼���ٶȼƱ������
    accel_scale = 9.81f / MPU6500_g_norm;
}
/* -------------------------------- CALIBRATE ------------------------------- */


struct imu_ops imu_ops = {
        .imu_init = MPU6500_init,
        .gyro_read = MPU6500_gyro_read,
        .gyro_config = MPU6500_gyro_config,
        .accel_read = MPU6500_accel_read,
        .accel_config = MPU6500_accel_config,
        .temp_read = MPU6500_temp_read,
};