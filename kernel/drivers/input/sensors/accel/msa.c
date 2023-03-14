/* drivers/input/sensors/access/kxtik.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: luowei <lw@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/sensor-dev.h>

/* full scale setting - register & mask */
#define MSA_REG_SPI_I2C                 0x00
#define MSA_REG_WHO_AM_I                0x01
#define MSA_REG_ACC_X_LSB               0x02
#define MSA_REG_ACC_X_MSB               0x03
#define MSA_REG_ACC_Y_LSB               0x04
#define MSA_REG_ACC_Y_MSB               0x05
#define MSA_REG_ACC_Z_LSB               0x06
#define MSA_REG_ACC_Z_MSB               0x07 
#define MSA_REG_G_RANGE                 0x0f
#define MSA_REG_ODR_AXIS_DISABLE        0x10
#define MSA_REG_POWERMODE_BW            0x11
#define MSA_REG_SWAP_POLARITY           0x12
#define MSA_REG_FIFO_CTRL               0x14
#define MSA_REG_INTERRUPT_SETTINGS1     0x16
#define MSA_REG_INTERRUPT_SETTINGS2     0x17

#define MSA_DEVID			(0x19)
#define MSA_ACC_DISABLE		0x9E
#define MSA_ACC_ABLE		0x1E

//#define LIS3DH_RANGE			2000000

/* MSA */
#define MSA_PRECISION		10

#define MSA_ACC_ODR3			0x02  /* 3.9Hz output data rate */
#define MSA_ACC_ODR7		0x03  /* 7.81Hz output data rate */
#define MSA_ACC_ODR15	0x04  /* 15.63Hz output data rate */
#define MSA_ACC_ODR31		0x05  /* 31.25Hz output data rate */
#define MSA_ACC_ODR62		0x06  /* 62.5Hz output data rate */
#define MSA_ACC_ODR125		0x07  /* 125Hz output data rate */
#define MSA_ACC_ODR250		0x08  /* 250Hz output data rate */
#define MSA_ACC_ODR500		0x09  /* 500Hz output data rate */

struct sensor_reg_data {
	char reg;
	char data;
};

/****************operate according to sensor chip:start************/
/* odr table, hz */
struct odr_table {
	unsigned int cutoff_ms;
	unsigned int mask;
};

static struct odr_table msa_acc_odr_table[] = {
		{1,	MSA_ACC_ODR500},
		{3,	MSA_ACC_ODR250},
		{5,	MSA_ACC_ODR125},
		{10,	MSA_ACC_ODR62},
		{20,	MSA_ACC_ODR31},
		{40,	MSA_ACC_ODR15},
		{100,	MSA_ACC_ODR7},
		{1000,	MSA_ACC_ODR3},
};

static int msa_select_odr(int want)
{
	int i;
	int max_index = ARRAY_SIZE(msa_acc_odr_table);

	//printk("%s xiao ---%d--- \n",__func__,__LINE__);
	for (i = max_index - 1; i >= 0; i--) {
		if ((msa_acc_odr_table[i].cutoff_ms <= want) ||
		    (i == 0))
			break;
	}

	return msa_acc_odr_table[i].mask;
}

static int sensor_active(struct i2c_client *client, int enable, int rate/*ms*/)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	int status = 0;
	int odr_rate = 0;

	printk("%s xiao ctrl_reg:0x%x ---%d---\n",__func__,sensor->ops->ctrl_reg,__LINE__);
	if (rate == 0) {
		dev_err(&client->dev, "%s: rate == 0!!!\n", __func__);
		return -1;
	}

	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	result = msa_select_odr(odr_rate);
	sensor->ops->ctrl_data |= result;

	if (!enable) {
		status = MSA_ACC_DISABLE;
		sensor->ops->ctrl_data = status;
	} else {
		sensor->ops->init(client);
		status = MSA_ACC_ABLE;
		sensor->ops->ctrl_data = status;
	}

	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if (result)
		dev_err(&client->dev, "%s:fail to active sensor\n", __func__);
	printk("%s xiao MSA_REG_POWERMODE:0x%x END !!---%d---\n",__func__,sensor_read_reg(client, sensor->ops->ctrl_data),__LINE__);
	return result;
}

static int sensor_init(struct i2c_client *client)
{
	int result = 0;
	int i;
	int pcode = 0;

	struct sensor_reg_data reg_data[] = {
	  {MSA_REG_SPI_I2C, 0x24},
	  {MSA_REG_POWERMODE_BW, 0x1e},
	  {MSA_REG_ODR_AXIS_DISABLE, 0x07},
	  {MSA_REG_G_RANGE, 0x00},
	 };

	printk("%s xiao ---%d---\n",__func__,__LINE__);
	for (i = 0; i < (sizeof(reg_data) / sizeof(struct sensor_reg_data)); i++) {
		result = sensor_write_reg(client, reg_data[i].reg, reg_data[i].data);
		if (result) {
			dev_err(&client->dev, "%s:line=%d,i=%d,error\n", __func__, __LINE__, i);
			return result;
		}
	}
	if(0)
	{
		pcode = sensor_read_reg(client, MSA_REG_WHO_AM_I);
		printk("%s xiao pcode:%d ---%d---\n",__func__,pcode,__LINE__);	

		pcode = sensor_read_reg(client, 0x0F);
		printk("%s xiao 0x0f reg:%d ---%d---\n",__func__,pcode,__LINE__);	

		pcode = sensor_read_reg(client, 0x10);
		printk("%s xiao 0x10 reg:%d ---%d---\n",__func__,pcode,__LINE__);	

		pcode = sensor_read_reg(client, 0x11);
		printk("%s xiao 0x11 reg:%d ---%d---\n",__func__,pcode,__LINE__);	
	}
	printk("%s xiao END ---%d---\n",__func__,__LINE__);
	return result;
}

static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis)
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);
//	printk("%s xiao X:%d Y:%d Z:%d  ---%d---\n",__func__,axis->x,axis->y,axis->z,__LINE__);
	if (sensor->status_cur == SENSOR_ON) {
		/* Report acceleration sensor information */
		input_report_abs(sensor->input_dev, ABS_X, axis->x);
		input_report_abs(sensor->input_dev, ABS_Y, axis->y);
		input_report_abs(sensor->input_dev, ABS_Z, axis->z);
		input_sync(sensor->input_dev);
	}


	return 0;
}

static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
			(struct sensor_private_data *) i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = sensor->pdata;
	int ret = 0;
	short x, y, z;
	struct sensor_axis axis;
	char buffer[6] = {0};

//	printk("%s xiao ---%d---\n",__func__,__LINE__);

	if (sensor->ops->read_len < 6) {
		dev_err(&client->dev, "%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}
	memset(buffer, 0, 6);

			do {

				buffer[0] =
				    sensor_read_reg(client,
						    MSA_REG_ACC_X_LSB);
				buffer[1] =
				    sensor_read_reg(client,
						    MSA_REG_ACC_X_LSB + 1);
				buffer[2] =
				    sensor_read_reg(client,
						    MSA_REG_ACC_X_LSB + 2);
				buffer[3] =
				    sensor_read_reg(client,
						    MSA_REG_ACC_X_LSB + 3);
				buffer[4] =
				    sensor_read_reg(client,
						    MSA_REG_ACC_X_LSB + 4);
				buffer[5] =
				    sensor_read_reg(client,
						    MSA_REG_ACC_X_LSB + 5);
			} while (0);

	x = ((buffer[1] << 8) & 0xff00) + (buffer[0] & 0xFF);
	y = ((buffer[3] << 8) & 0xff00) + (buffer[2] & 0xFF);
	z = ((buffer[5] << 8) & 0xff00) + (buffer[4] & 0xFF);

	axis.x = (pdata->orientation[0]) * x + (pdata->orientation[1]) * y + (pdata->orientation[2]) * z;
	axis.y = (pdata->orientation[3]) * x + (pdata->orientation[4]) * y + (pdata->orientation[5]) * z;
	axis.z = (pdata->orientation[6]) * x + (pdata->orientation[7]) * y + (pdata->orientation[8]) * z;

	gsensor_report_value(client, &axis);

	mutex_lock(&(sensor->data_mutex));
	sensor->axis = axis;
	mutex_unlock(&(sensor->data_mutex));

	if ((sensor->pdata->irq_enable) && (sensor->ops->int_status_reg >= 0))
		sensor_read_reg(client, sensor->ops->int_status_reg);

	return ret;
}

struct sensor_operate gsensor_msa_ops = {
	.name				= "gs_msa",
	.type				= SENSOR_TYPE_ACCEL,
	.id_i2c				= ACCEL_ID_MSA,
	.read_reg				= MSA_REG_ACC_X_LSB,
	.read_len				= 6,
	.id_reg				= -1,
	.id_data 				= MSA_DEVID,
	.precision				= MSA_PRECISION,
        .ctrl_reg           = MSA_REG_POWERMODE_BW,
        .int_status_reg     = 0x00,
	.range				= {-32768, +32768},
	.trig					= (IRQF_TRIGGER_LOW | IRQF_ONESHOT),
	.active				= sensor_active,
	.init					= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_msa_ops;
}

static int __init gsensor_msa_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;

	return sensor_register_slave(type, NULL, NULL, gsensor_get_ops);
}

static void __exit gsensor_msa_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;

	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}

module_init(gsensor_msa_init);
module_exit(gsensor_msa_exit);
