/* drivers/input/sensors/access/mxc6655.c
 *
 * Copyright (C) 2012-2015 ROCKCHIP.
 * Author: <edongtech.com>
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


#define MXC6655_DEVID_ID		0x03 //chip id

#define MXC6655_RANGE			2000000

#define 	MXC6655_ID			0x0E
#define 	MXC6655_XOUT1			0x03
#define 	MXC6655_XOUT2			0x04
#define 	MXC6655_YOUT1			0x05
#define 	MXC6655_POWMODE			0x0D
#define  	MXC6655_INTEN2			0x17

#define  	MXC6655_CTRL_REG1  MXC6655_POWMODE
#define		MXC6655_RANGE_8G		0x41

#define		MXC6655_BIT_INT_EN	(1<<4)

#define     MXC6655_PRECISION       10


/* CONTROL REGISTER 1 BITS */
#define MXC6655_SUSPEND_DISABLE		0x40
#define MXC6655_SUSPEND_ENABLE		0x01



/****************operate according to sensor chip:start************/

static int turn_on;
static int sensor_active(struct i2c_client *client, int enable, int rate)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);
	int result = 0;
	turn_on = 0;
	mdelay(20);
	//int status = 0;

	//printk("NEIL+++>: mxc4005 active enter addr = 0x%x, enable = %d\n", client->addr, enable);
	sensor->ops->ctrl_data = sensor_read_reg(client, sensor->ops->ctrl_reg);
	
	//register setting according to chip datasheet
	if(enable)
	{
		sensor->ops->ctrl_data = MXC6655_SUSPEND_DISABLE;

             //   ResetIntCaliPara();
	}
	else
	{
		sensor->ops->ctrl_data = MXC6655_SUSPEND_ENABLE;
	}

	//printk("%s:reg=0x%x,reg_ctrl=0x%x,enable=%d\n", __func__, sensor->ops->ctrl_reg, sensor->ops->ctrl_data, enable);
	result = sensor_write_reg(client, sensor->ops->ctrl_reg, sensor->ops->ctrl_data);
	if(result)
		printk("%s:fail to active sensor\n", __func__);
	else
	{
		// printk("%s:enable the sensor \n", __func__);
		mdelay(300);
	}

	if (enable)
		turn_on = 1;

	
	return result;

} 

static int sensor_init(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
	    (struct sensor_private_data *) i2c_get_clientdata(client);

       
	int result = 0;
	int i = 0;
	unsigned char id_reg = MXC6655_ID;
	unsigned char id_data = 0;
	printk("client->addr: 0x%02x\n", client->addr);
	result = sensor->ops->active(client, 1, 0);
	if(result)
	{
		printk("%s:line=%d,error\n", __func__, __LINE__);
		return result;
	}
	
	sensor->status_cur = SENSOR_ON;

	for(i=0; i<3; i++)
	{
		result = sensor_rx_data(client, &id_reg, 1);
		id_data = id_reg;
		if(!result)
		break;
	}

	if(result)
	{
		printk("%s:fail to read id,result=%d\n", __func__, result);
		return result;
	}
	printk("%s(%d) id_data: %d\n", __func__, __LINE__, id_data);
	sensor->devid = id_data;
	//20181204 ,id is 2 or 3
    if(id_data==2 || id_data==3)
	{
		sensor->devid = MXC6655_DEVID_ID;
	}
    //end

	if(sensor->pdata->irq_enable)	//open interrupt
	{

		result = sensor_write_reg(client, MXC6655_INTEN2, MXC6655_BIT_INT_EN);//enable int,active high,need read INT_REL
		if(result)
		{
			printk("%s:line=%d,error\n", __func__, __LINE__);
			return result;
		}
	}

	result = sensor_write_reg(client, MXC6655_CTRL_REG1, MXC6655_RANGE_8G);
	if(result)
	{
		printk("%s:line=%d,error\n", __func__, __LINE__);
		return result;
	}

	printk("%s:%s id=0x%x\n", __func__, sensor->ops->name, id_data);
	return result;
}



static int gsensor_report_value(struct i2c_client *client, struct sensor_axis *axis) 
{
	struct sensor_private_data *sensor =
		(struct sensor_private_data *) i2c_get_clientdata(client);  
        static struct sensor_axis last_axis;

        if (axis->x == last_axis.x &&
            axis->y == last_axis.y &&
            axis->z == last_axis.z)
                axis->z += 1;
				
	/* Report acceleration sensor information */
	input_report_abs(sensor->input_dev, ABS_X, axis->x); 
	input_report_abs(sensor->input_dev, ABS_Y, axis->y);
	input_report_abs(sensor->input_dev, ABS_Z, axis->z);
	last_axis = *axis;	
	input_sync(sensor->input_dev);
	//printk("Gsensor----->x=%d  y=%d z=%d\n", axis->x, axis->y, axis->z);
 
	return 0;
}  

#define GSENSOR_MIN  10
static int sensor_report_value(struct i2c_client *client)
{
	struct sensor_private_data *sensor =
			(struct sensor_private_data *) i2c_get_clientdata(client);
    	struct sensor_platform_data *pdata = sensor->pdata;




	int ret = 0;
	int x,y,z;
	struct sensor_axis axis;
	unsigned char buffer[7] = {0};
	char value = 0;
	if (turn_on == 0)
		return -1;
			
	//printk("NEIL++>: %s:Enter\n", __func__);
	if(sensor->ops->read_len < 6)	//sensor->ops->read_len = 6
	{
		printk("%s:lenth is error,len=%d\n", __func__, sensor->ops->read_len);
		return -1;
	}

	memset(buffer, 0, 6);
	
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	do {
		*buffer = sensor->ops->read_reg;
		ret = sensor_rx_data(client, buffer, sensor->ops->read_len);
		if (ret < 0)
		return ret;
	} while (0);

	x = (s16)(buffer[0] << 8 | buffer[1]) >> 4;
	y = (s16)(buffer[2] << 8 | buffer[3]) >> 4;
	z = (s16)(buffer[4] << 8 | buffer[5]) >> 4;
//	printk("%s(%d):  memsic printf raw data 1111 = %d  %d  %d \n", __func__, __LINE__, 	x , y, 	z);
	axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;
	axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;
	
	/*
	x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
	y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;
	z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;*/
	
//   printk("%s(%d):  memsic printf raw data 2222 = %d  %d  %d \n", __func__, __LINE__, 	axis.x  , axis.y, axis.z);

	axis.x = (axis.x * 64);
	axis.y = (axis.y * 64);
	axis.z = (axis.z * 64);
//	printk("%s(%d): memsic printf raw data 333 %d  %d  %d \n", __func__, __LINE__, axis.x, axis.y, axis.z);

//	if((abs(sensor->axis.x - axis.x) > GSENSOR_MIN) || (abs(sensor->axis.y - axis.y) > GSENSOR_MIN) || (abs(sensor->axis.z - axis.z) > GSENSOR_MIN))
	{
		gsensor_report_value(client, &axis);


		mutex_lock(&(sensor->data_mutex) );
		sensor->axis = axis;
		mutex_unlock(&(sensor->data_mutex) );
	}

	if((sensor->pdata->irq_enable)&& (sensor->ops->int_status_reg >= 0)) //read sensor intterupt status register
	{
		
		value = sensor_read_reg(client, sensor->ops->int_status_reg);
		DBG("%s:sensor int status :0x%x\n", __func__, value);
	}

	return ret;
}

struct sensor_operate gsensor_mxc6655_ops = {
	.name				= "gs_mxc6655",
	.type				= SENSOR_TYPE_ACCEL,		//sensor type and it should be correct
	.id_i2c				= ACCEL_ID_MXC6655,		    //i2c id number
	.read_reg			= MXC6655_XOUT1,		    //read data
	.read_len			= 7,				        //data length
	.id_reg				= -1,			            //read device id from this register
	.id_data			= MXC6655_DEVID_ID,
	.precision			= MXC6655_PRECISION,		//10 bits
	.ctrl_reg 			= MXC6655_CTRL_REG1,		//enable or disable 
	.int_status_reg 	= SENSOR_UNKNOW_DATA,		//intterupt status register
	.range				= {-MXC6655_RANGE,MXC6655_RANGE},	//range
	.trig				= IRQF_TRIGGER_LOW|IRQF_ONESHOT,
	.active				= sensor_active,
	.init				= sensor_init,
	.report				= sensor_report_value,
};

/****************operate according to sensor chip:end************/

//function name should not be changed
static struct sensor_operate *gsensor_get_ops(void)
{
	return &gsensor_mxc6655_ops;
}


static int __init gsensor_mxc6655_init(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int result = 0;
	int type = ops->type;
	printk("NEIL+++>: gsensor_mxc6655_init enter!\n");
	result = sensor_register_slave(type, NULL, NULL, gsensor_get_ops);
	return result;
}

static void __exit gsensor_mxc6655_exit(void)
{
	struct sensor_operate *ops = gsensor_get_ops();
	int type = ops->type;
	sensor_unregister_slave(type, NULL, NULL, gsensor_get_ops);
}


module_init(gsensor_mxc6655_init);
module_exit(gsensor_mxc6655_exit);

