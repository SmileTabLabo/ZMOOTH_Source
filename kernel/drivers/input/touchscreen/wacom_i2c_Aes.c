/*
 * Wacom Penabled Driver for I2C
 *
 * Copyright (c) 2011-2019 Tatsunosuke Tobita, Wacom.
 *		<tobita.tatsunosuke@wacom.co.jp>
 * Copyright (c) 2011-2019 Martin Chen, Wacom.
 *		<martin.chen@wacom.com>, modify for G12T plus
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software
 * Foundation; either version of 2 of the License,
 * or (at your option) any later version.
 */
#include "wacom_Aes.h"
#include <linux/acpi.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#define RESET 0

//#define SHOW_REPORTDESC
#ifdef CONFIG_ACPI
	#define WACOM_HARDWARE_ID "WCOM50C1"
#endif

#define File_Test_PATH                "/data/file_test.ini"
static bool irq_enabled = true;
static int em_stop_p=0;
static unsigned int dev_version_H = 0;
static unsigned int dev_version_L = 0;
module_param_named(TP_version_H, dev_version_H, int, 0644);
module_param_named(TP_version_L, dev_version_L, int, 0644);

 int set_wacom_stop(bool state)
{
	if(em_stop_p)gpio_direction_output(em_stop_p,state);
	printk("%s set wacom_stop_gpio:%d ",__func__,gpio_get_value(em_stop_p));
	return 0;
}
EXPORT_SYMBOL_GPL(set_wacom_stop);
static ssize_t state_show(struct device *dev, struct device_attribute *attr,char *buf)
{

	switch(TP_DEBUG_ON)
	{
		case 0: *buf=48; break;
		case 1: *buf=49; break;
		case 2: *buf=50; break;
		default: *buf=48; break;
	}
	printk("%s value ACSII:0x%d (%d)\n",__func__,*buf,TP_DEBUG_ON);
	return 1;
}

static ssize_t state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	switch(*buf)
	{
		case 48: TP_DEBUG_ON=0; break;
		case 49: TP_DEBUG_ON=1; break;
		case 50: TP_DEBUG_ON=2; break;
		default: TP_DEBUG_ON=0; break;
	}
	printk("%s set %d \n",__func__,TP_DEBUG_ON);

	return count;
}
static DEVICE_ATTR(state, 0644,state_show, state_store);


static long wacom_debug_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return 0;
}


static struct file_operations wacom_debug_cdev_fops = {
	.owner= THIS_MODULE,
	.unlocked_ioctl= wacom_debug_ioctl,
};


static struct miscdevice wacom_debug_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "wacom_debug",
	.fops = &wacom_debug_cdev_fops,
};


static void parse_report_desc(struct wacom_features *features, u8 *report_desc, int report_desc_size)
{
	bool finger = false, pen = false;
	int i;
	int usage = 0, mouse = 0;

	TP_DEBUG("wacom_i2c %s ", __func__);

	for (i = 0; i < report_desc_size; i++) {
		switch (report_desc[i]) {
		case USAGE_PAGE:
			switch (report_desc[i + 1]) {
			case USAGE_PAGE_DIGITIZERS:
				usage = USAGE_PAGE_DIGITIZERS;
				if (report_desc[i + 3] == USAGE_TOUCHSCREEN || report_desc[i + 3] == USAGE_PEN) {
					mouse = 0;
					i += 4;
				}
				else
					i++;
				
				break;

			case USAGE_PAGE_DESKTOP:
				usage = USAGE_PAGE_DESKTOP;
				if (report_desc[i + 3] == USAGE_MOUSE) {
					mouse = 1;
					i += 4;
				}
				else
					i++;
				
				break;
			}
			break;

		case USAGE:
			switch (report_desc[i + 1]) {
			case USAGE_X:
				if (usage == USAGE_PAGE_DESKTOP && mouse == 0) {
					if (pen)
						features->x_max = get_unaligned_le16(&report_desc[i + 3]);
					else if (finger)
						features->x_touch = get_unaligned_le16(&report_desc[i + 3]);
					i += 4;
				}
				else if (usage == USAGE_PAGE_DIGITIZERS && mouse == 0) {
					if (pen)
						features->pressure_max = get_unaligned_le16(&report_desc[i + 3]);
					i += 4;
				}
				else
					i++;
				
				break;
				
			case USAGE_Y:
				if (usage == USAGE_PAGE_DESKTOP && mouse == 0) {
					if (pen)
						features->y_max = get_unaligned_le16(&report_desc[i + 3]);
					else if (finger)
						features->y_touch = get_unaligned_le16(&report_desc[i + 3]);
					i += 4;
				}
				else
					i++;
				
				break;
			case USAGE_FINGER:
				finger = true;
				pen = false;
				i++;
				break;

			case USAGE_STYLUS:
				pen = true;
				finger = false;
				i++;
				break;
			}
			break;
		}		
	}
}

static int wacom_get_report_descriptor(struct i2c_client *client, struct wacom_features *features,
			 HID_DESC hid_desc)
{
	int ret = -1;
	int report_desc_size = hid_desc.wReportDescLength;
	u8 cmd_reportDesc[] = {hid_desc.wReportDescRegister, 0x00};
	u8 *report_desc = NULL;
	struct i2c_msg msgs_desc[2];
	
	TP_DEBUG("wacom_i2c %s ", __func__);

	report_desc = kzalloc(sizeof(u8) * report_desc_size, GFP_KERNEL);
	if (!report_desc) {
		dev_err(&client->dev, "No memory left for this device\n");
		return -ENOMEM;
	}

	msgs_desc[0].addr = client->addr;
	msgs_desc[0].flags = 0;
	msgs_desc[0].len = sizeof(cmd_reportDesc);
	msgs_desc[0].buf = cmd_reportDesc;

	msgs_desc[1].addr = client->addr;
	msgs_desc[1].flags = I2C_M_RD;
	msgs_desc[1].len = report_desc_size;
	msgs_desc[1].buf = report_desc;
		
	ret = i2c_transfer(client->adapter, msgs_desc, ARRAY_SIZE(msgs_desc));
	if (ret < 0) {
		dev_err(&client->dev, "%s obtaining report descriptor failed\n", __func__);
		goto errReportDesc;
	}
	if (ret != ARRAY_SIZE(msgs_desc)) {
		ret = -EIO;
		goto errReportDesc;
	}

#ifdef SHOW_REPORTDESC
	int i = 0;
	for (i = 0; i < report_desc_size; i ++) {
		if( i%8 == 0)
			printk( "\n RP%d:0x ", i );
		TP_DEBUG("%02x ",  report_desc[i]);
	}
#endif

	parse_report_desc(features, report_desc, report_desc_size);
	ret = 0;

	printk( "addr: 0x%x x_max:%d, y_max:%d\n", client->addr, 
	       features->x_max, features->y_max);
	printk( "addr: 0x%x x_touch:%d, y_touch:%d\n", client->addr, 
	       features->x_touch, features->y_touch);
	printk( "pressure_max:%d, fw_version:%x\n",
	       features->pressure_max, features->fw_version);
	
errReportDesc:
	kfree(report_desc);
	report_desc = NULL;
	return ret;
}

int wacom_query_device(struct i2c_client *client, struct wacom_features *features)
{
	int ret = -1;
	u8 cmd_devDesc[] = {HID_DESC_REGISTER, 0x00};
	u8 cmd_getfeatureID04[] = {0x04,0x00,0x34,0x02,0x05,0x00};
	u8 retrive_featureID04[18] = {0};
	HID_DESC hid_descriptor = {0};		
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd_devDesc),
			.buf = cmd_devDesc,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(HID_DESC),
			.buf = (u8 *)(&hid_descriptor),
		},
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd_getfeatureID04),
			.buf = cmd_getfeatureID04,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(retrive_featureID04),
			.buf = retrive_featureID04,
		},
	};
	
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "%s obtaining query failed: %d\n", __func__, ret);
		goto errQueryDevice;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s input/output error occured;\n returned: %dbyte(s)\n", __func__, ret);
		ret = -EIO;
		goto errQueryDevice;
	}

	features->input_size = hid_descriptor.wMaxInputLength;
	features->vendorId = hid_descriptor.wVendorID;
	features->productId = hid_descriptor.wProductID;
	features->fw_version_H = get_unaligned_le16(&retrive_featureID04[13]); //Get product major version
	features->fw_version_L = retrive_featureID04[15];                      //Get product minor version
	memcpy(&features->hid_desc, &hid_descriptor, sizeof(HID_DESC));	

	printk(
		"dev_VID:0x%x, dev_PID:0x%x dev_Version:0x%x fw_version_H:%d L:%d\n",
		hid_descriptor.wVendorID, hid_descriptor.wProductID, hid_descriptor.wVersion,features->fw_version_H,features->fw_version_L );

	if(features->fw_version_H)dev_version_H=features->fw_version_H;
	if(features->fw_version_L)dev_version_L=features->fw_version_L;
	
	ret = wacom_get_report_descriptor(client, features, hid_descriptor);
	if (ret < 0)
		goto errQueryDevice;

	ret = 0;
errQueryDevice:
	return ret;
}
/*
int wacom_query_device(struct i2c_client *client, struct wacom_features *features)
{
	int ret = -1;
	u8 cmd_devDesc[] = {HID_DESC_REGISTER, 0x00};
	HID_DESC hid_descriptor = {0};		
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmd_devDesc),
			.buf = cmd_devDesc,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(HID_DESC),
			.buf = (u8 *)(&hid_descriptor),
		},
	};
	
	TP_DEBUG("wacom_i2c %s ", __func__);
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev, "%s obtaining query failed: %d\n", __func__, ret);
		goto errQueryDevice;
	}
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s input/output error occured;\n returned: %dbyte(s)\n", __func__, ret);
		ret = -EIO;
		goto errQueryDevice;
	}

	features->input_size = hid_descriptor.wMaxInputLength;
	features->vendorId = hid_descriptor.wVendorID;
	features->productId = hid_descriptor.wProductID;
	features->fw_version = hid_descriptor.wVersion;
	memcpy(&features->hid_desc, &hid_descriptor, sizeof(HID_DESC));	

	printk(
		"dev_VID:0x%x, dev_PID:0x%x dev_Version:0x%x\n",
		hid_descriptor.wVendorID, hid_descriptor.wProductID, hid_descriptor.wVersion );

	ret = wacom_get_report_descriptor(client, features, hid_descriptor);
	if (ret < 0)
		goto errQueryDevice;

	ret = 0;
errQueryDevice:
	return ret;
}*/

static void set_touch_coord(struct wacom_i2c *wac_i2c, u8 *data) {

	struct input_dev *input = wac_i2c->input_touch;
	int *finger_num = &wac_i2c->finger_num;
	bool *mt_rdy = &wac_i2c->rdy;
	int i, tsw = 0;
	int x, y, id = 0;
	struct wacom_features *features = wac_i2c->features;	

#ifdef INPUT_WITH_WIDTH
	int cw, ch;	// contact width and height
#endif
#ifdef IDX_MT_KEY
	int key0, key1, key2, key3;
#endif


	/*When data[4] has a value, then it is the first finger packet.*/
	if (data[4] > 0x00) {
		*finger_num = data[4] -1;
		*mt_rdy = false;

	TP_DEBUG("wacom_i2c input: %s %15s finger_num:%d data[4]:%d ", __func__,input->name,*finger_num,data[4]);

#ifdef IDX_MT_KEY
		// Check the key value & report here
		key0 = data[IDX_MT_KEY] & 0x01;
		key1 = data[IDX_MT_KEY] & 0x02;
		key2 = data[IDX_MT_KEY] & 0x04;
		key3 = data[IDX_MT_KEY] & 0x08;
		input_report_key(input, MY_KEY0, key0);
		input_report_key(input, MY_KEY1, key1);
		input_report_key(input, MY_KEY2, key2);
		input_report_key(input, MY_KEY3, key3);
#endif
	}

	/*One packet holds the status for two fingers.*/
	for (i = 0; i < FINGERNUM_IN_PACKET; i++) {
		tsw = data[IDX_MT_TSW + (i * TOUCH_DATA_OFFSET)] & 0x01;
		id = le16_to_cpup((__le16 *)&data[IDX_MT_ID + (i * TOUCH_DATA_OFFSET)]);
		x = le16_to_cpup((__le16 *)&data[IDX_MT_X + (i * TOUCH_DATA_OFFSET)]);
		y = le16_to_cpup((__le16 *)&data[IDX_MT_Y + (i * TOUCH_DATA_OFFSET)]);
#ifdef INPUT_WITH_WIDTH
		cw = le16_to_cpup((__le16 *)&data[IDX_MT_WIDTH + (i * TOUCH_DATA_OFFSET)]);
		ch = le16_to_cpup((__le16 *)&data[IDX_MT_HEIGHT + (i * TOUCH_DATA_OFFSET)]);
#endif
	TP_DEBUG(" wacom_i2c touch x= %d y= %d id:%d  tsw:%d ",x,y,id,tsw);

		if (tsw) {
		input_report_abs(input, ABS_MT_TOOL_TYPE, 0 );
		input_report_key(input, BTN_TOUCH, 1);
		input_report_abs(input, ABS_MT_TRACKING_ID, id);
		input_report_abs(input, ABS_MT_PRESSURE, tsw);
		input_report_abs(input, ABS_MT_POSITION_X, features->y_touch-y);
		input_report_abs(input, ABS_MT_POSITION_Y, x);
	        input_mt_sync(input);

#ifdef INPUT_WITH_WIDTH
			input_report_abs(input, ABS_MT_TOUCH_MAJOR, cw);// or ABS_MT_WIDTH_MAJOR
			input_report_abs(input, ABS_MT_TOUCH_MINOR, ch;
			TP_DEBUG(" wacom_i2c touch cw: %d ch: %d  ",cw,ch);
#endif
		}
		else  
			{
				input_report_key(input, BTN_TOUCH, 0);
				input_mt_sync(input);
			}
			
		if (*finger_num != 0) {
			(*finger_num)--;
		} 
		else {
			if (!(*mt_rdy))
				*mt_rdy = true;
			else
				*mt_rdy = false;
			break;
		}
	}
	TP_DEBUG(" wacom_i2c touch mt_rdy %d ",*mt_rdy);
	if (*mt_rdy) {
		input_sync(input);
	}

	return;
}

static void set_aes_coord(struct wacom_i2c *wac_i2c, u8 *data)
{
	struct input_dev *input = wac_i2c->input_aes;
	unsigned int x, y, pressure,x_tilt,y_tilt;
	struct wacom_features *features = wac_i2c->features;
	unsigned char tsw, f1, f2, ers, x_tilt_raw, y_tilt_raw;
	int tilt_x ,tilt_y;

	tsw = data[3] & 0x01;
	ers = data[3] & 0x04;
	f1 = data[3] & 0x02;
	f2 = data[3] & 0x10;
	x = le16_to_cpup((__le16 *)&data[4]);
	y = le16_to_cpup((__le16 *)&data[6]);
	pressure = le16_to_cpup((__le16 *)&data[8]);
	x_tilt_raw = data[18]; //(191,255) equals to (-65°,-1°), (0-65) equals to (0°-65°)
	y_tilt_raw = data[19];
	if (x_tilt_raw > PEN_TILT_MAX_G14T) // negative
		tilt_x = -(256-x_tilt_raw);
	else // positive
		tilt_x = x_tilt_raw;

	if (y_tilt_raw > PEN_TILT_MAX_G14T) // negative
		tilt_y = -(256-y_tilt_raw);
	else // positive
		tilt_y = y_tilt_raw;

	// Feb/25/2021, Shift tilt value from (-65°,65°) to all positive (0°,131°) for some Android could't explain minus value
	tilt_x = tilt_x + PEN_TILT_MAX_G14T;
	tilt_y = tilt_y + PEN_TILT_MAX_G14T;


	TP_DEBUG("wacom_i2c %s ", __func__);
	if (!wac_i2c->rdy)
		wac_i2c->tool = (data[3] & 0x0c) ? BTN_TOOL_RUBBER : BTN_TOOL_PEN;	

	wac_i2c->rdy = data[3] & 0x20;

	TP_DEBUG(" wacom_i2c x= %d y= %d pressure = %d tsw || ers:%d  x_tilt=%d y_tilt=%d ",x,y,pressure,tsw || ers,x_tilt,y_tilt);	
	input_report_key(input, BTN_TOUCH, tsw || ers);
	input_report_key(input, BTN_STYLUS, f1);
	input_report_key(input, BTN_STYLUS2, f2);
	input_report_abs(input, ABS_X, features->y_max-y);	
	input_report_abs(input, ABS_Y, x);
	input_report_abs(input, ABS_PRESSURE, pressure);
	input_report_abs(input, ABS_TILT_X, (PEN_TILT_MAX_G14T*2+1)-tilt_y);
	input_report_abs(input, ABS_TILT_Y, tilt_x);
	input_report_key(input, wac_i2c->tool, wac_i2c->rdy);
	input_sync(input);

	return;
}

static irqreturn_t wacom_i2c_irq(int irq, void *dev_id)
{
	struct wacom_i2c *wac_i2c = dev_id;

	disable_irq_nosync(irq);
	queue_work(wac_i2c->ktouch_wq, &wac_i2c->work_irq);

return IRQ_HANDLED;
}

#define RESET_CMD_SIZE	4
#if RESET
static int wacom_i2c_reset(struct i2c_client *client)
{
	int ret = -1;
	char cmd_Reset[RESET_CMD_SIZE] = {0x04, 0x00, 0x00, 0x01};
	char buf[2] = {0xff, 0xff};

	printk("wacom_i2c %s \n", __func__);

	ret = i2c_master_send(client, cmd_Reset, RESET_CMD_SIZE);
	if (ret != RESET_CMD_SIZE) {
		printk("%s: Sending reset command failed \n", __func__);
		goto errReset;
	}

	msleep(100);

	/*Confirm zero'd 2 byte data is recieved for HID over I2C spec*/
	ret = i2c_master_recv(client, buf, 2);
	if (ret != 2 || (buf[0] & 0xff) || (buf[1] & 0xff)) {
		printk("%s: Receving data failed \n", __func__);
		goto errReset;
	}

	ret = 0;
errReset:
	return ret;
}
#endif

static void wacom_i2c_enable_irq(struct i2c_client *client, bool enable)
{
#if RESET
	int ret = -1;
#endif

	printk("wacom_i2c %s \n", __func__);

	if (enable && !irq_enabled) {
		printk("wacom_i2c %s enabled irq \n ", __func__);
		enable_irq(wac_i2c->irq);

#if RESET
	ret = wacom_i2c_reset(client);
	if (ret < 0)
		dev_err(&client->dev, "%s SET_RESET failed\n", __func__);
#endif

		irq_enabled = true;
	} else if (!enable && irq_enabled) {
		printk("wacom_i2c %s disabled irq\n ", __func__);
		disable_irq(wac_i2c->irq);
		irq_enabled = false;
	}

	return;
}

static void wacom_set_inputevent (struct input_dev *input, struct i2c_client *client,
				  struct wacom_features *features, int device)
{
	TP_DEBUG("wacom_i2c %s ", __func__);

	input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;

	switch (device) {	
	case IEVENT_AES:
		__set_bit(EV_ABS, input->evbit);
		__set_bit(INPUT_PROP_DIRECT, input->propbit);
		__set_bit(EV_ABS, input->evbit);
		__set_bit(EV_KEY, input->evbit);
		__set_bit(BTN_TOUCH, input->keybit);
		__set_bit(BTN_STYLUS, input->keybit);
		__set_bit(BTN_STYLUS2, input->keybit);
		__set_bit(BTN_TOOL_PEN, input->keybit);
		__set_bit(BTN_TOOL_RUBBER, input->keybit);

		input_set_abs_params(input, ABS_X, 0, features->y_max, 0, 0);
		input_set_abs_params(input, ABS_Y, 0, features->x_max, 0, 0);
		input_set_abs_params(input, ABS_PRESSURE,0, features->pressure_max, 0, 0);
		// Feb/25/2021, Shift tilt value from (-65°,65°) to all positive (0°,131°) for some Android could't explain minus value
		input_set_abs_params(input, ABS_TILT_X, 0, (PEN_TILT_MAX_G14T*2+1), 0, 0);
		input_set_abs_params(input, ABS_TILT_Y, 0, (PEN_TILT_MAX_G14T*2+1), 0, 0);



		break;

	case IEVENT_TOUCH:
		#ifdef IDX_MT_KEY
			__set_bit(MY_KEY0, input->keybit);
			__set_bit(MY_KEY1, input->keybit);
			__set_bit(MY_KEY2, input->keybit);
			__set_bit(MY_KEY3, input->keybit);
		#endif


	        __set_bit(INPUT_PROP_DIRECT, input->propbit);
		input_set_abs_params(input, ABS_MT_POSITION_X, 0, features->y_touch, 0, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0, features->x_touch, 0, 0);
		input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
		input_set_abs_params(input, ABS_MT_TOOL_TYPE,0, MT_TOOL_MAX, 0, 0);
		input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
		break;

	default: 
		return;
	}
}


static void wacom_i2c_wq(struct work_struct *work)
{
	struct wacom_i2c *wac_i2c = container_of(work, struct wacom_i2c, work_irq);
	int error;
	u8 *data = wac_i2c->data;
	u8 i;
	TP_DEBUG(" wacom_i2c %s --%d-- !!",__func__,__LINE__);
	mutex_lock(&wac_i2c->mutex_wq);
	error = i2c_master_recv(wac_i2c->client,
				wac_i2c->data, WACOM_INPUT_REPORTSIZE);
	if (error < 0)printk("\n wacom_i2c %s read error !--%d-- !!\n",__func__,__LINE__);
	else TP_DEBUG(" wacom_i2c %s data[2]:%d --%d-- !!",__func__,data[2],__LINE__);	
	if (data[2] ==  20)
		set_aes_coord(wac_i2c, data);
	else if (data[2] ==  28)
		set_touch_coord(wac_i2c, data);
	if(TP_DEBUG_ON==2)for(i=0;i<WACOM_TOUCH_INPUTSIZE;i++)TP_DEBUG(" data[%d]:0x%x ",i,data[i]);
	schedule();
	mutex_unlock(&wac_i2c->mutex_wq);
	enable_irq(wac_i2c->irq); 
}



static int wacom_i2c_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{

	struct input_dev *input_aes, *input_touch;
	struct wacom_features features;
	int error = -1,rc;
	struct file *flp = NULL;
	u8 *refp = NULL;
	u32 ref_len = 10;
//	char file_data[];
	unsigned int irq_flags;
	unsigned int  wake_flags;
	struct device_node *np = client->dev.of_node;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}
	printk("wacom_i2c probe started(addr: %x) \n", client->addr);
	wac_i2c = kzalloc(sizeof(*wac_i2c), GFP_KERNEL);
	input_aes = input_allocate_device();
	input_touch = input_allocate_device();
	if (!wac_i2c || !input_aes || !input_touch) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	wac_i2c->features = kzalloc(sizeof(struct wacom_features), GFP_KERNEL);
	if (!wac_i2c->features) {
		printk("failed to preserve the memory");
		goto err_free_mem;
	}

	refp = kzalloc(ref_len, GFP_KERNEL);
	wac_i2c->irq_pin = of_get_named_gpio_flags(np, "em-gpio", 0, (enum of_gpio_flags *)&irq_flags);
	wac_i2c->em_stop = of_get_named_gpio_flags(np, "stop-gpio", 0, (enum of_gpio_flags *)&wake_flags);
	wac_i2c->em_rst = of_get_named_gpio_flags(np, "reset-gpio", 0, (enum of_gpio_flags *)&wake_flags);
	em_stop_p=wac_i2c->em_stop;
	if (gpio_is_valid(wac_i2c->em_stop)) {
		rc = gpio_request(wac_i2c->em_stop,"wacom Stop pin");
		if (rc != 0) {
			dev_err(&client->dev, "wacom Reset pin error\n");
			return -EIO;
		}
		else 
		{
			gpio_direction_output(wac_i2c->em_stop,1);
			printk("wacom_i2c %s Stop PIN %d request output %d \n",__func__,wac_i2c->em_stop,gpio_get_value(wac_i2c->em_stop));	
		}
	} else {
		dev_info(&client->dev, "Stop pin invalid\n");
	}


	if (gpio_is_valid(wac_i2c->em_rst)) {
		rc = gpio_request(wac_i2c->em_rst,"wacom reset pin");
		if (rc != 0) {
			dev_err(&client->dev, "wacom Reset pin error\n");
			return -EIO;
		}
		else 
		{
			gpio_direction_output(wac_i2c->em_rst,0);
			msleep(50);
			gpio_direction_output(wac_i2c->em_rst,1);
			msleep(200);
			printk("wacom_i2c %s RESET PIN %d request output %d \n",__func__,wac_i2c->em_rst,gpio_get_value(wac_i2c->em_rst));	
		}
	} else {
		dev_info(&client->dev, "Reset pin invalid\n");
	}

	
	if (gpio_is_valid(wac_i2c->irq_pin)) {
		rc = gpio_request(wac_i2c->irq_pin,"wacom irq pin");
		if (rc != 0) {
			dev_err(&client->dev, "wacom irq pin error\n");
			return -EIO;
		}
		else
		{
			gpio_direction_input(wac_i2c->irq_pin);
			printk("wacom_i2c %s PIN %dIRQ request input leve :%d \n",__func__,wac_i2c->irq_pin,gpio_get_value(wac_i2c->irq_pin));
		}
	} else {
		printk("irq pin invalid\n");
	}

	/*Check, first, whether or not there's a device corresponding to the address*/
	error = wacom_query_device(client, &features);
	if (error < 0) {
		printk("wacom_query_device read again! \n");
		msleep(100);
		error = wacom_query_device(client, &features);
		if (error < 0) {
			printk("wacom_query_device failed \n");
			goto err_exit;
		}
	}
	memcpy(wac_i2c->features, &features, sizeof(struct wacom_features));

	wac_i2c->client = client;
	wac_i2c->input_aes = input_aes;
	wac_i2c->input_touch = input_touch;

	input_aes->name = "Wacom I2C AES";
	input_aes->id.bustype = BUS_I2C;
	input_aes->id.vendor = wac_i2c->features->vendorId;
	input_aes->id.product = wac_i2c->features->productId;
	input_aes->id.version = wac_i2c->features->fw_version;
	input_aes->dev.parent = &client->dev;
	wacom_set_inputevent(input_aes, client, wac_i2c->features, IEVENT_AES);
	input_set_drvdata(input_aes, wac_i2c);
	
	input_touch->name = "Wacom I2C TOUCH";
	input_touch->id.bustype = BUS_I2C;
	input_touch->id.vendor = wac_i2c->features->vendorId;
	input_touch->id.product = wac_i2c->features->productId;
	input_touch->id.version = wac_i2c->features->fw_version;
	input_touch->dev.parent = &client->dev;
	wacom_set_inputevent(input_touch, client, wac_i2c->features, IEVENT_TOUCH);
	input_set_drvdata(input_touch, wac_i2c);


	wac_i2c->ktouch_wq = create_singlethread_workqueue("wacom");
	mutex_init(&wac_i2c->mutex_wq);
	INIT_WORK(&wac_i2c->work_irq, wacom_i2c_wq);	
	/*step8:Apply for interruption*/


	error = input_register_device(wac_i2c->input_aes);
	if (error) {
		dev_err(&client->dev, 
			"Failed to register input device, error: %d\n", error);
		goto err_free_input_aes;
	}

	error = input_register_device(wac_i2c->input_touch);
	if (error) {
		dev_err(&client->dev, 
			"Failed to register input device, error: %d\n", error);
		goto err_free_input_touch;
	}

	wac_i2c->irq = gpio_to_irq(wac_i2c->irq_pin);
	error = request_irq(wac_i2c->irq,wacom_i2c_irq,IRQF_TRIGGER_LOW, "wacom_i2c", wac_i2c);
	if (error) {
		dev_err(&client->dev,
			"Failed to enable IRQ, error: %d\n", error);
		goto err_free_mem;
	}
	i2c_set_clientdata(client, wac_i2c);

	misc_register(&wacom_debug_misc_dev);
	device_create_file(wacom_debug_misc_dev.this_device, &dev_attr_state);

		
	flp = filp_open(File_Test_PATH, O_RDONLY, 644);
	if (IS_ERR(flp))printk(" %s Open file[%s] fail ! \n", __func__,File_Test_PATH); 
	else 
	{
		error=flp->f_op->read(flp, (char *)refp, ref_len, &flp->f_pos);
		if(error < 0)printk(" %s Read file fail ! \n", __func__);
		else printk("Read File--%10s-- \n",(char *)refp);
	}

	printk("wacom_i2c %s End ! \n", __func__);
	/* Disable the IRQ, we'll enable it in wac_i2c_open() */
	return 0;

 err_free_input_touch:
	input_free_device(input_touch);

 err_free_input_aes:
	input_free_device(input_aes);
	free_irq(wac_i2c->irq, wac_i2c);

 err_free_mem:
	kfree(wac_i2c);
	wac_i2c = NULL;

 err_exit:
	em_stop_p=0;
	return error;
}

static int wacom_i2c_remove(struct i2c_client *client)
{
	struct wacom_i2c *wac_i2c = i2c_get_clientdata(client);

	printk( "%s \n", __func__);
	free_irq(wac_i2c->irq, wac_i2c);
	input_unregister_device(wac_i2c->input_aes);
	input_unregister_device(wac_i2c->input_touch);
	kfree(wac_i2c->features);
	kfree(wac_i2c);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int wacom_i2c_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	printk( "%s \n", __func__);	
	if(0)wacom_i2c_enable_irq(client, false);
	disable_irq(wac_i2c->irq);
	gpio_direction_output(wac_i2c->irq_pin,0);
	gpio_direction_output(wac_i2c->em_stop,0);
	gpio_direction_output(wac_i2c->em_rst,0);
	printk( "%s irq:%d \n", __func__,gpio_get_value(wac_i2c->irq_pin));	
	return 0;
}

static int wacom_i2c_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	printk( "%s irq:%d \n", __func__,gpio_get_value(wac_i2c->irq_pin));	
	gpio_direction_input(wac_i2c->irq_pin);
	if(0)wacom_i2c_enable_irq(client, true);
	gpio_direction_output(wac_i2c->em_stop,1);
	gpio_direction_output(wac_i2c->em_rst,1);
	msleep(50);
	enable_irq(wac_i2c->irq);
	return 0;
}
static SIMPLE_DEV_PM_OPS(wacom_i2c_pm, wacom_i2c_suspend, wacom_i2c_resume);
#endif

static const struct i2c_device_id wacom_i2c_id[] = {
	{ "WAC_I2C_TOUCH", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, wacom_i2c_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id wacom_i2c_acpi_match[] = {
	{ WACOM_HARDWARE_ID, 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, wacom_i2c_acpi_match);
#endif

static struct i2c_driver wacom_i2c_driver = {
	.driver	= {
		.name	= "wacom_i2c",
		.owner	= THIS_MODULE,
		.pm	= &wacom_i2c_pm,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(wacom_i2c_acpi_match),
#endif

	},

	.probe		= wacom_i2c_probe,
	.remove		= wacom_i2c_remove,
	.id_table	= wacom_i2c_id,
};

static int __init wacom_i2c_init(void)
{
        printk("!!!!!!!!!!!!!!!!!%s!!!!!!!!!!!!!\n", __func__);
	printk("wacom i2c driver started\n");
	return i2c_add_driver(&wacom_i2c_driver);
}

static void __exit wacom_i2c_exit(void)
{
	i2c_del_driver(&wacom_i2c_driver);
}

#if 1
module_init(wacom_i2c_init);
module_exit(wacom_i2c_exit);
#else
module_i2c_driver(wacom_i2c_driver);
#endif

MODULE_AUTHOR("Tatsunosuke Tobita <tobita.tatsunosuke@wacom.co.jp>");
MODULE_DESCRIPTION("WACOM AES I2C Device Driver");
MODULE_LICENSE("GPL");
