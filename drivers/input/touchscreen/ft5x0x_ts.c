/* 
 * drivers/input/touchscreen/ft5x0x_ts_key.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Ingenic Semiconductor Inc.
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
 *
 *	note: only support mulititouch	liaoqizhen 2010-09-01
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/tsc.h>

#include "ft5x0x_ts.h"

#if defined(CONFIG_FT5X0X_MULTITOUCH_TWO_POINT)
#define PENUP_TIMEOUT_TIMER 1
#if defined(CONFIG_TOUCHSCREEN_FT5306)
#define P2_PACKET_LEN 13  //2 point
#define P1_PACKET_LEN 7
#else
#define P2_PACKET_LEN 11  //2 point
#define P1_PACKET_LEN 5
#endif
#elif defined(CONFIG_FT5X0X_MULTITOUCH_FIVE_POINT)
#define PENUP_TIMEOUT_TIMER 1
#define P2_PACKET_LEN 29   //5 point
#define P1_PACKET_LEN 5
//#endif
#else

#define PENUP_TIMEOUT_TIMER 1
#if defined(CONFIG_TOUCHSCREEN_FT5306)
#define P2_PACKET_LEN 13  //2 point
#define P1_PACKET_LEN 7
#else
#define P2_PACKET_LEN 11  //2 point
#define P1_PACKET_LEN 5
#endif
#endif

//#define CONFIG_FT5X0X_DEBUG 


struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
#if defined(CONFIG_FT5X0X_MULTITOUCH_FIVE_POINT)
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
#endif
	u16 pen_up;
	u16	pressure;
	u8  touch_point;
};

struct ft5x0x_gpio {
	struct jztsc_pin *irq;
	struct jztsc_pin *wake;
};

struct ft5x0x_parameter {
	unsigned short		x_res;
	unsigned short		y_res;
};

struct ft5x0x_ts_data {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	struct input_dev 	*input_dev1; /* for adkey */
	struct ts_event		event;
	struct delayed_work  pen_event_work;
	//struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
	int is_suspend;
#ifdef PENUP_TIMEOUT_TIMER
	struct timer_list penup_timeout_timer;
#endif
#ifdef CONFIG_FT5X0X_DEBUG
	long int count;
#endif
	struct ft5x0x_parameter para;
	struct ft5x0x_gpio gpio;
	struct regulator *power;
};

static int ft5x0x_ts_power_on(struct ft5x0x_ts_data *ts)
{
	if (ts->power)
		return regulator_enable(ts->power);

	return 0;
}

static int ft5x0x_ts_power_off(struct ft5x0x_ts_data *ts)
{
	if (ts->power)
		return regulator_disable(ts->power);

	return 0;
}

#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TOUCH_KEY

struct touch_key_info {
	
	u16	x_min_search;
	u16	x_max_search;
	u16 y_min_search;
	u16 y_max_search;
	u16	x_min_back;
	u16	x_max_back;
	u16	y_min_back;
	u16	y_max_back;
	u16	x_min_home;
	u16	x_max_home;
	u16	y_min_home;
	u16	y_max_home;
	u16	x_min_menu;
	u16	x_max_menu;
	u16	y_min_menu;
	u16	y_max_menu;
};

static struct touch_key_info tk_info = {
#if defined(CONFIG_FT5X0X_TOUCHKEY_JB01)
	.x_min_search = 810,
	.x_max_search = 850,
	.y_min_search = 0,
	.y_max_search = 25,
	.x_min_back = 810,
	.x_max_back = 850,
	.y_min_back = 25,
	.y_max_back = 50,
	.x_min_home = 810,
	.x_max_home = 850,
	.y_min_home = 50,
	.y_max_home = 75,
	.x_min_menu = 810,
	.x_max_menu = 850,
	.y_min_menu = 75,
	.y_max_menu = 110,
#elif defined(CONFIG_FT5X0X_TOUCHKEY_INGENIC)
#if 1
	.x_min_search = 835,
	.x_max_search = 850,
	.y_min_search = 200,
	.y_max_search = 210,
	
	.x_min_back = 835,
	.x_max_back = 850,
	.y_min_back = 105,
	.y_max_back = 115,
	
	.x_min_home = 835,
	.x_max_home = 850,
	.y_min_home = 140,
	.y_max_home = 150,
	
	.x_min_menu = 835,
	.x_max_menu = 850,
	.y_min_menu = 170,
	.y_max_menu = 180,
#else	//linden-1.1 new tp 2011-12-20
	.x_min_search = 35,
	.x_max_search = 55,
	.y_min_search = 185,
	.y_max_search = 205,
	.x_min_back = 35,
	.x_max_back = 55,
	.y_min_back = 95,
	.y_max_back = 115,
	.x_min_home = 35,
	.x_max_home = 55,
	.y_min_home = 125,
	.y_max_home = 145,
	.x_min_menu = 35,
	.x_max_menu = 55,
	.y_min_menu = 155,
	.y_max_menu = 175,
#endif
#elif defined(CONFIG_FT5X0X_TOUCHKEY_706)
	.x_min_search = 0,
	.x_max_search = 0,
	.y_min_search = 0,
	.y_max_search = 0,
	.x_min_back = 1074,
	.x_max_back = 1094,
	.y_min_back = 533,
	.y_max_back = 553,
	.x_min_home = 1074,
	.x_max_home = 1094,
	.y_min_home = 458,
	.y_max_home = 478,
	.x_min_menu = 1074,
	.x_max_menu = 1094,
	.y_min_menu = 496,
	.y_max_menu = 516,
#elif defined(CONFIG_FT5X0X_TOUCHKEY_PISCES)
	.x_min_search = 65457,
	.x_max_search = 65457,
	.y_min_search = 310,
	.y_max_search = 320,
	
	.x_min_back = 65457,
	.x_max_back = 65457,
	.y_min_back = 355,
	.y_max_back = 365,
	
	.x_min_home = 65457,
	.x_max_home = 65457,
	.y_min_home = 470,
	.y_max_home = 480,
	
	.x_min_menu = 65457,
	.x_max_menu = 65457,
	.y_min_menu = 405,
	.y_max_menu = 410,
	#endif
};
static int tp_key_menu_down = 0;
static int tp_key_home_down = 0;
static int tp_key_back_down = 0;
static int tp_key_search_down = 0;

static int ft5x0x_touchket_detec(int x,int y)
{	
#ifdef CONFIG_FT5X0X_DEBUG
		printk("---ft5x0x_touchket_detec---x:%d,y:%d ----\n",x,y);
#endif
	if(x >=tk_info.x_min_back && x <= tk_info.x_max_back && y >=tk_info.y_min_back && y <= tk_info.y_max_back){
		tp_key_back_down = 1;
		return 0;
	}
	if(x >=tk_info.x_min_home && x <= tk_info.x_max_home && y >=tk_info.y_min_home && y <= tk_info.y_max_home){
		tp_key_home_down = 1;
		return 0;
	}
	if(x >=tk_info.x_min_menu && x <= tk_info.x_max_menu && y >=tk_info.y_min_menu && y <= tk_info.y_max_menu){
		tp_key_menu_down = 1;
		return 0;
	}
	if(x >=tk_info.x_min_search && x <= tk_info.x_max_search && y >=tk_info.y_min_search && y <= tk_info.y_max_search){
		tp_key_search_down = 1;
		return 0;
	}
	
	return 1;
}
static void ft5x0x_touchkey_report(struct input_dev *dev)
{
#ifdef CONFIG_FT5X0X_DEBUG
		printk("---ft5x0x_touchkey_report---%d,%d,%d,%d ----\n",tp_key_search_down,tp_key_back_down,tp_key_home_down,tp_key_menu_down);
#endif		
		if(tp_key_search_down){
			input_report_key(dev,KEY_SEARCH,1);
			input_sync(dev);
		}
		if(tp_key_back_down){
			input_report_key(dev,KEY_BACK,1);
			input_sync(dev);
		}
		if(tp_key_home_down){
			input_report_key(dev,KEY_HOME,1);
			input_sync(dev);
		}
		if(tp_key_menu_down){
			input_report_key(dev,KEY_MENU,1);
			input_sync(dev);
		}
}
static int ft5x0x_touchkey_release(struct ft5x0x_ts_data *ft5x0x_ts)
{
	if(tp_key_menu_down) {
	  tp_key_menu_down = 0;
		input_report_key(ft5x0x_ts->input_dev1,KEY_MENU,0);
		input_sync(ft5x0x_ts->input_dev1);
	}else if(tp_key_home_down) {
		tp_key_home_down = 0;
		input_report_key(ft5x0x_ts->input_dev1,KEY_HOME,0);
		input_sync(ft5x0x_ts->input_dev1);
	}else if(tp_key_search_down) {
	  tp_key_search_down = 0;
		input_report_key(ft5x0x_ts->input_dev1,KEY_SEARCH,0);
		input_sync(ft5x0x_ts->input_dev1);
	}else if(tp_key_back_down) {
	  tp_key_back_down = 0;
		input_report_key(ft5x0x_ts->input_dev1,KEY_BACK,0);
		input_sync(ft5x0x_ts->input_dev1);
	}else{
		return 1;	
	}	

	return 0;
}
#endif
static int ft5x0x_i2c_rxdata(struct ft5x0x_ts_data *ts, char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= ts->client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= ts->client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(ts->client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int ft5x0x_i2c_txdata(struct ft5x0x_ts_data *ts, char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= ts->client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	msleep(1);
	ret = i2c_transfer(ts->client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_set_reg(struct ft5x0x_ts_data *ts, u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(ts, buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}
#endif
static void ft5x0x_ts_release(struct ft5x0x_ts_data *ft5x0x_ts)
{
	int ret = 1;	
#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TOUCH_KEY	
	ret = ft5x0x_touchkey_release(ft5x0x_ts);
#endif	
	if(ret){
#ifdef CONFIG_FT5X0X_MULTITOUCH
	//input_report_abs(ft5x0x_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(ft5x0x_ts->input_dev);   //modify for 4.0 MutiTouch
#else
	input_report_abs(ft5x0x_ts->input_dev, ABS_PRESSURE, 0);
	input_report_key(ft5x0x_ts->input_dev, BTN_TOUCH, 0);
#endif
	}
	input_sync(ft5x0x_ts->input_dev);
#ifdef PENUP_TIMEOUT_TIMER
	del_timer(&(ft5x0x_ts->penup_timeout_timer));
#endif
}

static void ft5x0x_chip_reset(struct ft5x0x_ts_data *ts)
{
	if (ft5x0x_ts_power_on(ts) < 0)
		dev_err(&ts->client->dev,"power on error\n");
	msleep(20);

	set_pin_status(ts->gpio.wake, 1);
	set_pin_status(ts->gpio.wake, 0);
	msleep(20);
	set_pin_status(ts->gpio.wake, 1);
}

static int ft5x0x_read_data(struct ft5x0x_ts_data *ft5x0x_ts)
{
	struct ts_event *event = &ft5x0x_ts->event;
	u8 buf[P2_PACKET_LEN+1] = {0};
	int ret = -1;

	buf[0] = 2;
#ifdef CONFIG_FT5X0X_MULTITOUCH
	ret = ft5x0x_i2c_rxdata(ft5x0x_ts, buf, P2_PACKET_LEN);
#else
	ret = ft5x0x_i2c_rxdata(ft5x0x_ts, buf, P1_PACKET_LEN);
#endif
	if (ret < 0) {
		ft5x0x_chip_reset(ft5x0x_ts);
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	if((buf[1]==0xff) && (buf[2]==0xff) && (buf[3]==0xff) && (buf[4]==0xff)) {
		//ft5x0x_ts_release(ft5x0x_ts);
		return 1;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[0] & 0x0f;
	event->pen_up = ((buf[1] >> 6)==0x01);

	if ((event->touch_point == 0)) {
		ft5x0x_ts_release(ft5x0x_ts);
		return 1; 
	}
#if defined(CONFIG_FT5X0X_MULTITOUCH_TWO_POINT)
	if(event->touch_point >2)event->touch_point=2;
#endif
#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch (event->touch_point) {
#if defined(CONFIG_FT5X0X_MULTITOUCH_FIVE_POINT)   
		case 5:
			event->y5 = ((((u16)buf[25])<<8)&0x0f00) |buf[26];		
			event->x5 = ((((u16)buf[27])<<8)&0x0f00) |buf[28];
			#ifdef CONFIG_TSC_SWAP_XY
			tsc_swap_xy(&(event->x5),&(event->y5));
			#endif
			#ifdef CONFIG_TSC_SWAP_X
			tsc_swap_x(&(event->x5),ft5x0x_ts->para.x_res);
			#endif
			#ifdef CONFIG_TSC_SWAP_Y
			tsc_swap_y(&(event->y5),ft5x0x_ts->para.y_res);
			#endif
			
		case 4:
			event->y4 = ((((u16)buf[19])<<8)&0x0f00) |buf[20];		
			event->x4 = ((((u16)buf[21])<<8)&0x0f00) |buf[22];
			#ifdef CONFIG_TSC_SWAP_XY
			tsc_swap_xy(&(event->x4),&(event->y4));
			#endif
			#ifdef CONFIG_TSC_SWAP_X
			tsc_swap_x(&(event->x4),ft5x0x_ts->para.x_res);
			#endif
			#ifdef CONFIG_TSC_SWAP_Y
			tsc_swap_y(&(event->y4),ft5x0x_ts->para.y_res);
			#endif
			
		case 3:
			event->y3 = ((((u16)buf[13])<<8)&0x0f00) |buf[14]; 		
			event->x3 = ((((u16)buf[15])<<8)&0x0f00) |buf[16];
			#ifdef CONFIG_TSC_SWAP_XY
			tsc_swap_xy(&(event->x3),&(event->y3));
			#endif
			#ifdef CONFIG_TSC_SWAP_X
			tsc_swap_x(&(event->x3),ft5x0x_ts->para.x_res);
			#endif
			#ifdef CONFIG_TSC_SWAP_Y
			tsc_swap_y(&(event->y3),ft5x0x_ts->para.y_res);
			#endif
			
#endif
		case 2:
			event->x2 = ((((u16)buf[9])<<8)&0x0f00) |buf[10]; 		
			event->y2 = ((((u16)buf[7])<<8)&0x0f00) |buf[8];
			#ifdef CONFIG_TSC_SWAP_XY
			tsc_swap_xy(&(event->x2),&(event->y2));
			#endif
			#ifdef CONFIG_TSC_SWAP_X
			tsc_swap_x(&(event->x2),ft5x0x_ts->para.x_res);
			#endif
			#ifdef CONFIG_TSC_SWAP_Y
			tsc_swap_y(&(event->y2),ft5x0x_ts->para.y_res);
			#endif

		case 1:
			event->x1 = ((((u16)buf[3])<<8)&0x0f00) |buf[4];			
			event->y1 = ((((u16)buf[1])<<8)&0x0f00) |buf[2];
			#ifdef CONFIG_TSC_SWAP_XY
			tsc_swap_xy(&(event->x1),&(event->y1));
			#endif
			#ifdef CONFIG_TSC_SWAP_X
			tsc_swap_x(&(event->x1),ft5x0x_ts->para.x_res);
			#endif
			#ifdef CONFIG_TSC_SWAP_Y
			tsc_swap_y(&(event->y1),ft5x0x_ts->para.y_res);
			#endif
			
			break;
		default:
			return -1;
	}
#else
	if (event->touch_point == 1) {
		event->x1 = ((((s16)buf[3])<<8)&0x0f00) |buf[4];
		event->y1 =((((s16)buf[1])<<8)&0x0f00) |buf[2];
		#ifdef CONFIG_TSC_SWAP_XY
		tsc_swap_xy(&(event->x1),&(event->y1));
		#endif
		//printk("(aaaa (%d)(%d) \n", event->x1,event->y1);
	}
#endif
	event->pressure = 200;
#ifdef PENUP_TIMEOUT_TIMER
	mod_timer(&(ft5x0x_ts->penup_timeout_timer), jiffies+40);
#endif
	return 0;
}

static void ft5x0x_report_value(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;


#if defined(CONFIG_FT5406_SCREEN_1024x600)
	event->x1 = (event->x1*data->para.x_res)/800;
	event->y1 = (event->y1*data->para.y_res)/480;
	event->x2 = (event->x1*data->para.x_res)/800;
	event->y2 = (event->y1*data->para.y_res)/480;
#endif

#ifdef CONFIG_FT5X0X_MULTITOUCH
	switch(event->touch_point) {
#if defined(CONFIG_FT5X0X_MULTITOUCH_FIVE_POINT)
		case 5:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x5);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y5);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 4:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x4);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y4);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 3:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x3);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y3);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
#endif
		case 2:
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);
		case 1:	
			#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TOUCH_KEY
			if(0 == ft5x0x_touchket_detec(event->x1,event->y1)){
				ft5x0x_touchkey_report(data->input_dev1);
			}else
			#endif	
			{
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
				input_mt_sync(data->input_dev);
			}
		default:
			break;
	}
#else	/* CONFIG_FT5X0X_MULTITOUCH */
	if (event->touch_point == 1) {	
		#ifdef CONFIG_TOUCHSCREEN_FT5X0X_TOUCH_KEY
		if(0 == ft5x0x_touchket_detec(event->x1,event->y1)){
			ft5x0x_touchkey_report(data->input_dev1);
		}else
		#endif		
		{	
			input_report_abs(data->input_dev, ABS_X, event->x1);
			input_report_abs(data->input_dev, ABS_Y, event->y1);
			input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
		}
	}

#endif	/* CONFIG_FT5X0X_MULTITOUCH*/
	input_sync(data->input_dev);
	dev_dbg(&data->client->dev, "%s: 1:%d %d 2:%d %d \n", __func__,	
			event->x1, event->y1, event->x2, event->y2);
}	/*end ft5x0x_report_value*/

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	struct ft5x0x_ts_data *ts;
	ts = container_of(work, struct ft5x0x_ts_data, pen_event_work.work);

#ifdef CONFIG_FT5X0X_DEBUG
	long int count = 0;
	count = ts->count;
	printk("==ft5X06 read data start(%ld)=\n", count);
#endif

	ret = ft5x0x_read_data(ts);	
	if (ret == 0) {	
		ft5x0x_report_value(ts);
	}
#ifdef CONFIG_FT5X0X_DEBUG
	printk("==ft5306 report data end(%ld)=\n", count++);
#endif
	//msleep(1);
	enable_irq(ts->client->irq);
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
#ifdef CONFIG_FT5X0X_DEBUG
	static long int count = 0;
#endif
	disable_irq_nosync(ft5x0x_ts->client->irq);
	if(ft5x0x_ts->is_suspend == 1)
		return IRQ_HANDLED;

#ifdef CONFIG_FT5X0X_DEBUG
	ft5x0x_ts->count = count;	
	printk("==ctp int(%ld)=\n", count++);	
#endif

	//if (!work_pending(&ft5x0x_ts->pen_event_work)) {
	if (!delayed_work_pending(&ft5x0x_ts->pen_event_work)) {
		//queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
		queue_delayed_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work, msecs_to_jiffies(25));
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ts;
	ts =  container_of(handler, struct ft5x0x_ts_data, early_suspend);

#ifdef CONFIG_FT5X0X_DEBUG
	printk("==ft5x0x_ts_suspend=\n");
#endif
	ts->is_suspend = 1;
	disable_irq(ts->client->irq);
	flush_workqueue(ts->ts_workqueue);
	//if(cancel_work_sync(&ts->pen_event_work))
	if(cancel_delayed_work(&ts->pen_event_work))
		enable_irq(ts->client->irq);
	ft5x0x_set_reg(ts, FT5X0X_REG_PMODE, PMODE_HIBERNATE);	
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{

	struct ft5x0x_ts_data *ts;
	ts = container_of(handler, struct ft5x0x_ts_data, early_suspend); 
#ifdef CONFIG_FT5X0X_DEBUG
	printk("==ft5x0x_ts_resume=\n");
#endif
	ft5x0x_chip_reset(ts);	
	ts->is_suspend = 0;
	enable_irq(ts->client->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND

static void ft5x0x_gpio_init(struct ft5x0x_ts_data *ts, struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct jztsc_platform_data *pdata = dev->platform_data;

	ts->gpio.irq = &pdata->gpio[0];
	ts->gpio.wake = &pdata->gpio[1];

	if (gpio_request_one(ts->gpio.irq->num,
			     GPIOF_DIR_IN, "ft5x0x_irq")) {
		dev_err(dev, "no irq pin available\n");
		ts->gpio.irq->num = -EBUSY;
	}
	if (gpio_request_one(ts->gpio.wake->num,
			     ts->gpio.wake->enable_level
			     ? GPIOF_OUT_INIT_LOW
			     : GPIOF_OUT_INIT_HIGH,
			     "ft5x0x_shutdown")) {
		dev_err(dev, "no shutdown pin available\n");
		ts->gpio.wake->num = -EBUSY;
	}
}

static int 
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct jztsc_platform_data *pdata = client->dev.platform_data;
	int err = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	ft5x0x_ts->para.x_res = pdata->x_max;
	ft5x0x_ts->para.y_res = pdata->y_max;

	ft5x0x_ts->client = client;
	i2c_set_clientdata(client, ft5x0x_ts);

	ft5x0x_gpio_init(ft5x0x_ts, client);
	ft5x0x_ts_power_on(ft5x0x_ts);

	//INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	INIT_DELAYED_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}

	client->dev.init_name=client->name;
	ft5x0x_ts->power = regulator_get(&client->dev, "vtsc");
	if (IS_ERR(ft5x0x_ts->power)) {
		dev_warn(&client->dev, "get regulator failed\n");
	}
	ft5x0x_ts_power_on(ft5x0x_ts);

	client->irq = gpio_to_irq(ft5x0x_ts->gpio.irq->num);
	err = request_irq(client->irq, ft5x0x_ts_interrupt,
			    IRQF_TRIGGER_FALLING | IRQF_DISABLED,
			  "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;
//start add
	ft5x0x_ts->input_dev1 = input_allocate_device();
	if (!ft5x0x_ts->input_dev1) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device1\n");
		goto exit_input_dev_alloc_failed1;
	}
	
	ft5x0x_ts->input_dev1->name = "touchkey";
	set_bit(EV_KEY, ft5x0x_ts->input_dev1->evbit);
	set_bit(KEY_MENU, ft5x0x_ts->input_dev1->keybit);
	set_bit(KEY_BACK, ft5x0x_ts->input_dev1->keybit);
	set_bit(KEY_HOME, ft5x0x_ts->input_dev1->keybit);
	set_bit(KEY_SEARCH, ft5x0x_ts->input_dev1->keybit);
	err = input_register_device(ft5x0x_ts->input_dev1);
	if (err) {
		dev_err(&client->dev,
				"ft5x0x_ts_probe: failed to register input device1: %s\n",
				dev_name(&client->dev));
		goto exit_input_register_device_failed1;
	}
//end
	set_bit(INPUT_PROP_DIRECT, ft5x0x_ts->input_dev->propbit);
#ifdef CONFIG_FT5X0X_MULTITOUCH
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			ABS_MT_POSITION_X, 0, ft5x0x_ts->para.x_res, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_POSITION_Y, 0, ft5x0x_ts->para.y_res, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_TOUCH_MAJOR, 0, 250, 0, 0);
	input_set_abs_params(input_dev,
			ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, ft5x0x_ts->para.x_res, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, ft5x0x_ts->para.x_res, 0, 0);
	input_set_abs_params(input_dev,
			ABS_PRESSURE, 0, 255, 0 , 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name		= FT5X0X_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
				"ft5x0x_ts_probe: failed to register input device: %s\n",
				dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef PENUP_TIMEOUT_TIMER
	init_timer(&(ft5x0x_ts->penup_timeout_timer));
	ft5x0x_ts->penup_timeout_timer.data = (unsigned long)ft5x0x_ts;
	ft5x0x_ts->penup_timeout_timer.function  =	(void (*)(unsigned long))ft5x0x_ts_release;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	ft5x0x_chip_reset(ft5x0x_ts);

	enable_irq(client->irq);

	return 0;
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_register_device_failed1:
	input_free_device(ft5x0x_ts->input_dev1);
exit_input_dev_alloc_failed1:
exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5x0x_ts);
exit_irq_request_failed:
exit_platform_data_null:
	//cancel_work_sync(&ft5x0x_ts->pen_event_work);
	cancel_delayed_work(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	input_unregister_device(ft5x0x_ts->input_dev1);
	kfree(ft5x0x_ts);
	//cancel_work_sync(&ft5x0x_ts->pen_event_work);
	cancel_delayed_work(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);
	ft5x0x_ts_power_off(ft5x0x_ts);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ft5x0x_ts_init(void)
{
	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<hsgou@ingenic.cn>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
