/* drivers/input/keyboard/jz_remote.c
 *
 * Infrared sensor code for Ingenic JZ SoC
 *
 * Copyright(C)2012 Ingenic Semiconductor Co., LTD.
 *	http://www.ingenic.cn
 *	Sun Jiwei <jwsun@ingenic.cddn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/sched.h>

#include <linux/gpio.h>
#include <linux/input/remote.h>

#define	IR_POWER	15
#define	IR_HOME		82
#define IR_MENU		16
#define IR_BACK		83

#define IR_VOLUMEUP	(0x5c)
#define IR_VOLUMEDOWN	(0x54)

#define IR_UP		77
#define IR_DOWN		90
#define IR_LEFT		87
#define IR_RIGHT	95
#define IR_OK		91

#define	IR_HELP		86
#define IR_SEARCH	94

#define IR_NEXTSONG	81
#define	IR_PREVIOUSSONG 93
#define	IR_PLAY		85
#define	IR_PLAYPAUSE	89

#define IR_MUTE		88

#define IR_1		(0x17)
#define IR_2		(0x1b)
#define IR_3		(0x1f)
#define IR_4		(0x16)
#define IR_5		(0x1a)
#define IR_6		(0x1e)
#define IR_7		(0x15)
#define IR_8		(0x19)
#define IR_9		(0x1d)
#define IR_DOT		(0x14)
#define IR_0		(0x18)
#define IR_BACKSPACE	(0x1c)

struct jz_remote {
	struct platform_device	*pdev;
	struct timer_list	timer;
	struct input_dev	*input_dev;

	struct jz_remote_board_data	*jz_remote_board_data;

	int	irq;

	struct wake_lock work_wake_lock;

	unsigned	int	long_press_state;
	unsigned	int	long_press_count;

	unsigned	char	current_data;
	unsigned	char	keycodes[11];
	unsigned	char	key;
};

#define KEY_PHYS_NAME	"jz_remote_button/input0"

//key code tab
static unsigned char initkey_code[ ] =
{
	KEY_POWER, KEY_HOME, KEY_MENU, KEY_BACK, KEY_VOLUMEUP, KEY_VOLUMEDOWN,	\
	KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_ENTER, KEY_HELP, KEY_SEARCH,	\
	KEY_NEXTSONG, KEY_PREVIOUSSONG, KEY_PLAY, KEY_PLAYPAUSE, KEY_MUTE, KEY_1,\
	KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_DOT,	\
	KEY_BACKSPACE, KEY_WAKEUP
};

static int jz_remote_ctrl_open(struct input_dev *dev)
{
	return 0;
}

static void jz_remote_ctrl_close(struct input_dev *dev)
{
}

void jz_remote_send_key(struct jz_remote *jz_remote)
{
	uint8_t  send_key = 0;
	//printk("JZ_REMOTE Driver:The Key is %d\n",jz_remote->current_data);

	switch (jz_remote->current_data) {
#if 0
	case IR_POWER:
		send_key = KEY_POWER;
		break;
#endif
	case IR_HOME:
		send_key = KEY_HOME;
		break;
	case IR_MENU:
		send_key = KEY_MENU;
		break;
	case IR_BACK:
		send_key = KEY_BACK;
		break;
	case IR_UP:
		send_key = KEY_UP;
		break;
	case IR_DOWN:
		send_key = KEY_DOWN;
		break;

	case IR_VOLUMEUP:
		send_key = KEY_VOLUMEUP;
		break;
	case IR_VOLUMEDOWN:
		send_key = KEY_VOLUMEDOWN;
		break;

	case IR_LEFT:
		send_key = KEY_LEFT;
		break;
	case IR_RIGHT:
		send_key = KEY_RIGHT;
		break;
	case IR_OK:
		send_key = KEY_ENTER;
		break;
	case IR_HELP:
		send_key = KEY_HELP;
		break;
	case IR_SEARCH:
		send_key = KEY_SEARCH;
		break;
	case IR_NEXTSONG:
		send_key = KEY_NEXTSONG;
		break;
	case IR_PREVIOUSSONG:
		send_key = KEY_PREVIOUSSONG;
		break;
	case IR_PLAY:
		send_key = KEY_PLAY;
		break;
	case IR_PLAYPAUSE:
		send_key = KEY_PLAYPAUSE;
		break;
	case IR_MUTE:
		send_key = KEY_MUTE;
		break;
	case IR_1:
		send_key = KEY_1;
		break;
	case IR_2:
		send_key = KEY_2;
		break;
	case IR_3:
		send_key = KEY_3;
		break;
	case IR_4:
		send_key = KEY_4;
		break;
	case IR_5:
		send_key = KEY_5;
		break;
	case IR_6:
		send_key = KEY_6;
		break;
	case IR_7:
		send_key = KEY_7;
		break;
	case IR_8:
		send_key = KEY_8;
		break;
	case IR_9:
		send_key = KEY_9;
		break;
	case IR_DOT:
		send_key = KEY_DOT;
		break;
	case IR_0:
		send_key = KEY_0;
		break;
	case IR_BACKSPACE:
		send_key = KEY_BACKSPACE;
		break;
	default:
		printk("The key is not defined,key value is %d\n",jz_remote->current_data);
	}

        input_report_key(jz_remote->input_dev,send_key,1);
        input_sync(jz_remote->input_dev);
        input_report_key(jz_remote->input_dev,send_key,0);
        input_sync(jz_remote->input_dev);
}

void jz_remote_callback(unsigned long data)
{
	struct jz_remote *jz_remote = (struct jz_remote *)data;
	if(jz_remote->long_press_state == 0){
		if (gpio_get_value(jz_remote->jz_remote_board_data->gpio) == 0) {
			while(!gpio_get_value(jz_remote->jz_remote_board_data->gpio));
			jz_remote->long_press_count = 0;
			jz_remote->timer.expires  = jiffies + usecs_to_jiffies(2500);
			add_timer(&jz_remote->timer);
			jz_remote->long_press_state = 1;
		}else{
			jz_remote->long_press_state++;
			if(jz_remote->long_press_count > 30){	//RE-PRESS TIME OUT
				enable_irq(jz_remote->irq);
				jz_remote->long_press_state = 0;
				jz_remote->long_press_count = 0;
			}else{
				jz_remote->timer.expires  = jiffies + usecs_to_jiffies(2000);
				add_timer(&jz_remote->timer);
			}
		}
	}else if(jz_remote->long_press_state == 1){
		if(gpio_get_value(jz_remote->jz_remote_board_data->gpio) == 0){
			jz_remote->long_press_count = 0;
			jz_remote->timer.expires  = jiffies + msecs_to_jiffies(100);
			add_timer(&jz_remote->timer);
			jz_remote->long_press_state = 0;
#if CONFIG_JZ_REMOTE_SUPPORT_LONGPRESS
			jz_remote_send_key(jz_remote);
#endif
		}else{
			jz_remote->long_press_count++;
			if(jz_remote->long_press_count > 30){
				enable_irq(jz_remote->irq);
				jz_remote->long_press_state = 0;
				jz_remote->long_press_count = 0;
			}else{
				jz_remote->timer.expires  = jiffies + usecs_to_jiffies(2000);
				add_timer(&jz_remote->timer);
			}
		}
	}
}

void wx_ir_handler(struct jz_remote *jz_remote)
{
	int i, m , n;
	uint8_t da[4]={0,0,0,0};

	for(i = 0; i < 10; i++){
		udelay(880);
		if(gpio_get_value(jz_remote->jz_remote_board_data->gpio) == 1){
			//printk("Error: %s %s %d: i=%d\n",__FILE__,__func__,__LINE__, i);
			goto exit;
		}
	}

	while(!gpio_get_value(jz_remote->jz_remote_board_data->gpio));

	udelay(4700);

	for(m = 0; m < 4; m++){
		for(n = 0; n < 8; n++){
			while(!gpio_get_value(jz_remote->jz_remote_board_data->gpio));
			udelay(840);
			if(gpio_get_value(jz_remote->jz_remote_board_data->gpio) == 1){
				udelay(1000);
				da[m] = da[m] >> 1;
				da[m] = da[m] | 0x80;
			}else{
				da[m] = da[m] >> 1;
				da[m] = da[m] | 0x00;
			}
		}
	}

	if(da[2] != 0xff){
		jz_remote->current_data = da[2];
		jz_remote_send_key(jz_remote);
		//detect long-press and release
		{
			jz_remote->long_press_state = 0;
			jz_remote->long_press_count = 0;
			setup_timer(&jz_remote->timer, jz_remote_callback, (unsigned long)jz_remote);
			jz_remote->timer.expires  = jiffies + msecs_to_jiffies(102);//41
			add_timer(&jz_remote->timer);
			return;
		}
	}
exit:
	enable_irq(jz_remote->irq);
	jz_remote->long_press_state = 0;
}
static irqreturn_t jz_remote_irq_handler(s32 irq, void *data)
{
	struct	jz_remote *jz_remote = data;
	disable_irq_nosync(jz_remote->irq);
	wx_ir_handler(jz_remote);
	return IRQ_HANDLED;
}

static int __devinit jz_remote_probe(struct platform_device *pdev)
{
	struct jz_remote *jz_remote;
	struct input_dev *input_dev;
	int  i, ret;

	jz_remote = kzalloc(sizeof(struct jz_remote), GFP_KERNEL);
	if (!jz_remote) {
		dev_err(&pdev->dev, "failed to allocate input device for remote ctrl \n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device for remote ctrl \n");
		return -ENOMEM;
	}

	jz_remote->jz_remote_board_data = pdev->dev.platform_data;

	if (gpio_request_one(jz_remote->jz_remote_board_data->gpio,
				GPIOF_DIR_IN, "jz_remote")) {
		dev_err(&pdev->dev, "The gpio is used by other driver\n");
		jz_remote->jz_remote_board_data->gpio = -EBUSY;
		ret = -ENODEV;
		goto err_free;
	}

	jz_remote->irq = gpio_to_irq(jz_remote->jz_remote_board_data->gpio);
	if (jz_remote->irq < 0) {
		ret = jz_remote->irq;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free_gpio;
	}

	ret = request_irq(jz_remote->irq, jz_remote_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_DISABLED, "jz-remote", jz_remote);
	if (ret < 0) {
		printk(KERN_CRIT "Can't request IRQ for jz_remote__irq\n");
		return ret;
	}

	input_dev->name = pdev->name;
	input_dev->open = jz_remote_ctrl_open;
	input_dev->close = jz_remote_ctrl_close;
	input_dev->dev.parent = &pdev->dev;
	input_dev->phys = KEY_PHYS_NAME;
	input_dev->id.vendor = 0x0001;

	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;
	input_dev->keycodesize = sizeof(unsigned char);

	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], input_dev->keybit);

	clear_bit(0, input_dev->keybit);

	jz_remote->input_dev = input_dev;

	input_set_drvdata(input_dev, jz_remote);

	wake_lock_init(&jz_remote->work_wake_lock, WAKE_LOCK_SUSPEND, "jz-remote");

	input_dev->evbit[0] = BIT_MASK(EV_KEY) ;

	platform_set_drvdata(pdev, jz_remote);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register input jz_remote  device\n");
		goto err_unregister;
	}
	printk("The remote driver registe success\n");
	return 0;
err_unregister:
	input_unregister_device(jz_remote->input_dev);
	platform_set_drvdata(pdev, NULL);
err_free_gpio:
	gpio_free(jz_remote->jz_remote_board_data->gpio);
err_free:
	input_free_device(input_dev);
	kfree(jz_remote);
	return ret;
}

static int __devexit jz_remote__remove(struct platform_device *pdev)
{
	struct jz_remote *jz_remote = platform_get_drvdata(pdev);

	input_unregister_device(jz_remote->input_dev);
	input_free_device(jz_remote->input_dev);
	kfree(jz_remote);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver jz_remote_driver = {
	.probe		= jz_remote_probe,
	.remove 	= __devexit_p(jz_remote__remove),
	.driver 	= {
		.name	= "jz-remote",
		.owner	= THIS_MODULE,
	},
};

 int __init jz_remote_init(void)
{
	return platform_driver_register(&jz_remote_driver);
}

static void __exit jz_remote_exit(void)
{
	platform_driver_unregister(&jz_remote_driver);
}

module_init(jz_remote_init);
module_exit(jz_remote_exit);

MODULE_DESCRIPTION("jz_remote_irq Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sun Jiwei<jwsun@ingenic.cn>");
