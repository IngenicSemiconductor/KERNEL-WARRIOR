#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/jz_cim.h>
#include "ov5640_camera.h"

#include "ov5640_init.h"
#include "ov5640-demo.h"

#define CAMERA_MODE_PREVIEW 0
#define CAMERA_MODE_CAPTURE 1
#define FPS_15 0
#define FPS_7 1
static int mode = CAMERA_MODE_CAPTURE;
static int g_width = 640;
static int g_height = 480;
struct i2c_client * g_client ;

struct workqueue_struct *ov5640_work_queue;

//---------------------------------------------------
//		i2c
//---------------------------------------------------
/**
 * ov5640_reg_read - Read a value from a register in an ov5640 sensor device
 * @client: i2c driver client structure
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an ov5640 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
int ov5640_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	u8 data[2] = {0};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data,
	};

	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		goto err;
	}
	msg.flags = I2C_M_RD;
	msg.len = 1;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		goto err;
	}

	*val = data[0];

	return 0;
err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
}

/**
 * Write a value to a register in ov5640 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
int ov5640_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { (u8)(reg >> 8), (u8)(reg & 0xff), val };
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

/**
 * Initialize a list of ov5640 registers.
 * The list of registers is terminated by the pair of values
 * @client: i2c driver client structure.
 * @reglist[]: List of address of the registers to write data.
 * Returns zero if successful, or non-zero otherwise.
 */
int ov5640_reg_writes(struct i2c_client *client, const struct ov5640_reg reglist[])
{
	int err = 0;
	int i = 0;

	while (reglist[i].reg != 0xffff) {
		err = ov5640_reg_write(client, reglist[i].reg, reglist[i].val);
	//	err = ov5640_write_reg(client, reglist[i].reg, reglist[i].val);
		if (err)
			return err;
		i++;
	}
	return 0;
}

void ov5640_late_work(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work, struct delayed_work, work);
	struct ov5640_sensor *s = container_of(dwork, struct ov5640_sensor, work);

	//download auto-focus firmware
	OV5640_af_init(s->client);

	dev_info(&s->client->dev, "%s\n", __func__);
}

//---------------------------------------------------
//		init routine
//---------------------------------------------------
int ov5640_init(struct cim_sensor *sensor_info)
{
	struct ov5640_sensor *s;
	struct i2c_client * client ;
	int ret = 0;

	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;
	g_client = client;
	mode = CAMERA_MODE_PREVIEW;
#if 1
	ret |= ov5640_reg_writes(client, ov5640_init_table);
	//ret |= ov5640_reg_writes(client, ov5640_init_regs);
	if (ret)
		goto err;

	//download auto focus firmware
	queue_delayed_work(ov5640_work_queue, &s->work, 1*HZ);
#else
	OV5640_init(client);
#endif
	dev_info(&client->dev, "ov5640 init ok\n");
	return 0;
err:
	dev_info(&client->dev, "ov5640 init fail\n");
	return ret;
}

int ov5640_preview_set(struct cim_sensor *sensor_info)                   
{                               
	struct ov5640_sensor *s;
	struct i2c_client * client ;

	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;

	dev_info(&client->dev, "%s L%d\n", __func__, __LINE__);
    	mode = CAMERA_MODE_PREVIEW;

	OV5640_preview(client);

	return 0;
} 

//---------------------------------------------------
//		auto-exposure
//---------------------------------------------------
static uint32_t stop_ae(struct i2c_client *client)
{
	return 0;
}

static uint32_t start_ae(struct i2c_client *client)
{
	return 0;
}

//---------------------------------------------------
//		preview/capture resolution
//---------------------------------------------------
int ov5640_size_switch(struct cim_sensor *sensor_info,int width,int height)
{
	struct ov5640_sensor *s;
	struct i2c_client *client;

	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;

	dev_info(&client->dev,"%s L%d: set %s size (%d, %d)\n", __func__, __LINE__, (mode == CAMERA_MODE_PREVIEW) ? "preview" : "capture", width, height);

	g_width = width;
	g_height = height;

	if (mode == CAMERA_MODE_PREVIEW) {
		if ((width == 320) && (height == 240)) {
			ov5640_reg_writes(client, ov5640_qvga_regs);
		} else if ((width == 640) && (height == 480)) {
			ov5640_reg_writes(client, ov5640_vga_regs);
		} else if ((width == 1280) && (height == 720)) {
			ov5640_reg_writes(client, ov5640_720p_regs);
		} else {
			printk("%s L%d: invalid preview size %dx%d, default vga!\n", __func__, __LINE__, width, height);
			ov5640_reg_writes(client, ov5640_vga_regs);
			g_width = 640;
			g_height = 480;
		}
	} else if (mode == CAMERA_MODE_CAPTURE) {
		if ((width == 320) && (height == 240)) {
			ov5640_reg_writes(client, ov5640_qvga_regs);
		} else if ((width == 640) && (height == 480)) {
			ov5640_reg_writes(client, ov5640_vga_regs);
		} else if ((width == 1280) && (height == 720)) {
			ov5640_reg_writes(client, ov5640_720p_regs);
		} else if ((width == 1920) && (height == 1080)) {
			ov5640_reg_writes(client, ov5640_1080p_regs);
		} else if ((width == 2592) && (height == 1944)) {
			ov5640_reg_writes(client, ov5640_5mp_regs);
		} else {
			printk("%s L%d: invalid capture size %dx%d, default 720p!\n", __func__, __LINE__, width, height);
			ov5640_reg_writes(client, ov5640_720p_regs);
			g_width = 1280;
			g_height = 720;
		}
	} else {
		printk("%s L%d: invalid mode %d\n", __func__, __LINE__, mode);
	}

	return 0;
}

int ov5640_capture_set(struct cim_sensor *sensor_info)
{
	struct ov5640_sensor *s;
	struct i2c_client *client;

	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;

	dev_info(&client->dev, "%s L%d\n", __func__, __LINE__);

	mode = CAMERA_MODE_CAPTURE;

//	OV5640_capture(client);

	return 0;
}

//---------------------------------------------------
//		auto-focus
//---------------------------------------------------
void ov5640_set_focus(struct cim_sensor *sensor_info, unsigned short arg)
{
	struct ov5640_sensor *s;
	struct i2c_client * client ;
	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;
	dev_info(&client->dev,"ov5640_set_focus\n");

	OV5640_auto_focus(client);
}

//---------------------------------------------------
//		anti-banding 50/60 Hz
//---------------------------------------------------
void ov5640_set_ab_50hz(struct i2c_client *client)
{
	OV5640_set_bandingfilter(client, 50);
}

void ov5640_set_ab_60hz(struct i2c_client *client)
{
	OV5640_set_bandingfilter(client, 60);
}

int ov5640_set_antibanding(struct cim_sensor *sensor_info,unsigned short arg)
{
	struct ov5640_sensor *s;
	struct i2c_client * client ;
	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;
	dev_info(&client->dev,"ov5640_set_antibanding");
	switch(arg)
	{
		case ANTIBANDING_AUTO :
			ov5640_set_ab_50hz(client);
			dev_info(&client->dev,"ANTIBANDING_AUTO ");
			break;
		case ANTIBANDING_50HZ :
			ov5640_set_ab_50hz(client);
			dev_info(&client->dev,"ANTIBANDING_50HZ ");
			break;
		case ANTIBANDING_60HZ :
			ov5640_set_ab_60hz(client);
			dev_info(&client->dev,"ANTIBANDING_60HZ ");
			break;
		case ANTIBANDING_OFF :
			dev_info(&client->dev,"ANTIBANDING_OFF ");
			break;
	}
	return 0;
}

//---------------------------------------------------
//		effect
//---------------------------------------------------
void ov5640_set_effect_normal(struct i2c_client *client)
{
	ov5640_reg_writes(client, ov5640_effect_normal_regs);
}

void ov5640_set_effect_yellowish(struct i2c_client *client)
{
}

void ov5640_set_effect_reddish(struct i2c_client *client)
{
}

void ov5640_set_effect_sepia(struct i2c_client *client)
{
	ov5640_reg_writes(client, ov5640_effect_sepia_regs);
}

void ov5640_set_effect_colorinv(struct i2c_client *client)
{
	ov5640_reg_writes(client, ov5640_effect_colorinv_regs);
}

void ov5640_set_effect_black_and_white (struct i2c_client *client)
{
	ov5640_reg_writes(client, ov5640_effect_grayscale_regs);
}

void ov5640_set_effect_sepiagreen(struct i2c_client *client)
{
	ov5640_reg_writes(client, ov5640_effect_sepiabluel_regs);
}

void ov5640_set_effect_sepiablue(struct i2c_client *client)
{
	ov5640_reg_writes(client, ov5640_effect_sepiabluel_regs);
}


int ov5640_set_effect(struct cim_sensor *sensor_info,unsigned short arg)
{
	struct ov5640_sensor *s;
	struct i2c_client * client ;

	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;

	dev_info(&client->dev,"ov5640_set_effect: effect 0x%08x\n", arg);
	switch(arg)
	{
		case EFFECT_NONE:
			ov5640_set_effect_normal(client);
			dev_info(&client->dev,"EFFECT_NONE");
			break;
		case EFFECT_MONO :
			ov5640_set_effect_black_and_white(client);
			dev_info(&client->dev,"EFFECT_MONO ");
			break;
		case EFFECT_NEGATIVE :
			ov5640_set_effect_colorinv(client);
			dev_info(&client->dev,"EFFECT_NEGATIVE ");
			break;
		case EFFECT_SOLARIZE ://bao guang
			dev_info(&client->dev,"EFFECT_SOLARIZE ");
			break;
		case EFFECT_SEPIA :
			ov5640_set_effect_sepia(client);
			dev_info(&client->dev,"EFFECT_SEPIA ");
			break;
		case EFFECT_POSTERIZE ://se diao fen li
			dev_info(&client->dev,"EFFECT_POSTERIZE ");
			break;
		case EFFECT_WHITEBOARD :
			dev_info(&client->dev,"EFFECT_WHITEBOARD ");
			break;
		case EFFECT_BLACKBOARD :
			dev_info(&client->dev,"EFFECT_BLACKBOARD ");
			ov5640_set_effect_black_and_white(client);
			break;
		case EFFECT_AQUA  ://qian lv se
			ov5640_set_effect_sepiagreen(client);
			dev_info(&client->dev,"EFFECT_AQUA  ");
			break;
		case EFFECT_PASTEL:
			ov5640_set_effect_reddish(client);  
			dev_info(&client->dev,"EFFECT_PASTEL");
			break;
		case EFFECT_MOSAIC:
			dev_info(&client->dev,"EFFECT_MOSAIC");
			break;
		case EFFECT_RESIZE:
			dev_info(&client->dev,"EFFECT_RESIZE");
			break;
	}
	return 0;
}

//---------------------------------------------------
//		white-balance
//---------------------------------------------------
void ov5640_set_wb_auto(struct i2c_client *client)
{       
	ov5640_reg_writes(client, ov5640_wb_auto_regs);
}

void ov5640_set_wb_cloud(struct i2c_client *client)
{
	ov5640_reg_writes(client, ov5640_wb_cloud_regs);
}

void ov5640_set_wb_daylight(struct i2c_client *client)
{              
	ov5640_reg_writes(client, ov5640_wb_daylight_regs);
}

void ov5640_set_wb_incandescence(struct i2c_client *client)
{    
	ov5640_reg_writes(client, ov5640_wb_incandescence_regs);
}

void ov5640_set_wb_fluorescent(struct i2c_client *client)
{   
	ov5640_reg_writes(client, ov5640_wb_fluorescent_regs);
}

int ov5640_set_balance(struct cim_sensor *sensor_info,unsigned short arg)
{
	struct ov5640_sensor *s;
	struct i2c_client * client ;
	s = container_of(sensor_info, struct ov5640_sensor, cs);
	client = s->client;
	
	dev_info(&client->dev,"ov5640_set_balance");
	switch(arg)
	{
		case WHITE_BALANCE_AUTO:
			ov5640_set_wb_auto(client);
			dev_info(&client->dev,"WHITE_BALANCE_AUTO");
			break;
		case WHITE_BALANCE_INCANDESCENT :
			ov5640_set_wb_incandescence(client);
			dev_info(&client->dev,"WHITE_BALANCE_INCANDESCENT ");
			break;
		case WHITE_BALANCE_FLUORESCENT ://ying guang
			ov5640_set_wb_fluorescent(client);
			dev_info(&client->dev,"WHITE_BALANCE_FLUORESCENT ");
			break;
		case WHITE_BALANCE_WARM_FLUORESCENT :
			ov5640_set_wb_fluorescent(client);
			dev_info(&client->dev,"WHITE_BALANCE_WARM_FLUORESCENT ");
			break;
		case WHITE_BALANCE_DAYLIGHT ://ri guang
			ov5640_set_wb_daylight(client);
			dev_info(&client->dev,"WHITE_BALANCE_DAYLIGHT ");
			break;
		case WHITE_BALANCE_CLOUDY_DAYLIGHT ://yin tian
			ov5640_set_wb_cloud(client);
			dev_info(&client->dev,"WHITE_BALANCE_CLOUDY_DAYLIGHT ");
			break;
		case WHITE_BALANCE_TWILIGHT :
			dev_info(&client->dev,"WHITE_BALANCE_TWILIGHT ");
			break;
		case WHITE_BALANCE_SHADE :
			dev_info(&client->dev,"WHITE_BALANCE_SHADE ");
			break;
	}

	return 0;
}

#define OV5640_DEBUG_FS
#ifdef OV5640_DEBUG_FS
#include <linux/proc_fs.h>
static int camera_write_proc(struct file *file, const char __user *buffer,
		unsigned long count, void *data)
{
	int ret;
	unsigned int flags;
	unsigned int reg;
	unsigned int val;
	char buf[32];

	if (count > 32)
		count = 32;
	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	ret = sscanf(buf,"%x: %x %x",&flags, &reg,&val);
	printk("flags=0x%x, reg = 0x%x, val =0x%x\n", flags, reg, val);
	if (flags == 0xff) {
		ov5640_write_reg(g_client,(unsigned short)reg, (unsigned char)val);
	}
	val = ov5640_read_reg(g_client, (unsigned short)reg);
	printk("================read val = 0x%x===================\n",val);

	return count;
}

static int __init init_camera_proc(void)
{
	struct proc_dir_entry *res;

	res = create_proc_entry("camera", 0666, NULL);
	if (res) {
		res->read_proc = NULL;
		res->write_proc = camera_write_proc;
		res->data = NULL;
	}
	return 0;
}
module_init(init_camera_proc);
#endif
