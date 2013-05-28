/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	unsigned int		cur_brightness;
    unsigned int        max_brightness;
	unsigned int		suspend;
    struct mutex        pwm_lock;
    struct notifier_block nb;
	int			(*notify)(struct device *,
					  int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend bk_early_suspend;
#endif
};

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

    mutex_lock(&pb->pwm_lock);
	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

    if (pb->suspend && (brightness > pb->cur_brightness)) {
        pb->cur_brightness = brightness;
        //printk("pb->suspend, brightness =%d\n", brightness);
        mutex_unlock(&pb->pwm_lock);
        return 0;
    }

    pb->cur_brightness = brightness;
	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
        mdelay(50);
	} else {
		pwm_disable(pb->pwm);
		brightness = pb->lth_brightness +
			(brightness * (pb->period - pb->lth_brightness) / max);
		pwm_config(pb->pwm, brightness, pb->period);
		pwm_enable(pb->pwm);
	}
    mutex_unlock(&pb->pwm_lock);
	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
};

static int pwm_bl_shutdown_notify(struct notifier_block *rnb,
			   unsigned long unused2, void *unused3)
{
	struct pwm_bl_data *pb = container_of(rnb,
            struct pwm_bl_data, nb); 

    pwm_config(pb->pwm, 0, pb->period);
    pwm_disable(pb->pwm);

	return NOTIFY_DONE;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_CLEAR_PWM_OUTPUT
static void set_pwm1_gpio_function(void)
{
#define PWM1_GPIO_PXDATC (*(volatile unsigned int *)0xB0010418)
#define PWM1_GPIO_PXIMC (*(volatile unsigned int *)0xB0010428)
#define PWM1_GPIO_PXPEC (*(volatile unsigned int *)0xB0010438)
#define PWM1_GPIO_PXFUNC (*(volatile unsigned int *)0xB0010448)
	PWM1_GPIO_PXDATC = 0x2;
	PWM1_GPIO_PXIMC = 0x2;
	PWM1_GPIO_PXPEC = 0x2;
	PWM1_GPIO_PXFUNC = 0x2;
}
#endif
static void bk_e_suspend(struct early_suspend *h)
{
	struct pwm_bl_data *pb = container_of(h,
            struct pwm_bl_data, bk_early_suspend); 
    pb->suspend = 1;
    pwm_config(pb->pwm, 0, pb->period);
    pwm_disable(pb->pwm);
#ifdef CONFIG_CLEAR_PWM_OUTPUT
	gpio_direction_output(32*4+1, 0);
#endif
}

static void bk_l_resume(struct early_suspend *h)
{
    int brightness;
	struct pwm_bl_data *pb = container_of(h,
            struct pwm_bl_data, bk_early_suspend); 

    pb->suspend = 0;
#ifdef CONFIG_CLEAR_PWM_OUTPUT
	set_pwm1_gpio_function();
#endif
	mutex_lock(&pb->pwm_lock);
	pwm_disable(pb->pwm);
    brightness = pb->cur_brightness;
    brightness = pb->lth_brightness +
        (brightness * (pb->period - pb->lth_brightness) / pb->max_brightness);
    pwm_config(pb->pwm, brightness, pb->period);
    pwm_enable(pb->pwm);
    mutex_unlock(&pb->pwm_lock);
}
#endif

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
    pb->max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

    mutex_init(&pb->pwm_lock);
	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);

    pb->nb.notifier_call = pwm_bl_shutdown_notify;
	register_reboot_notifier(&pb->nb);

#ifdef CONFIG_HAS_EARLYSUSPEND
    pb->bk_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;//EARLY_SUSPEND_LEVEL_STOP_DRAWING;
    pb->bk_early_suspend.suspend = bk_e_suspend;
    pb->bk_early_suspend.resume = bk_l_resume;
    register_early_suspend(&pb->bk_early_suspend);
#endif

	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);

	unregister_reboot_notifier(&pb->nb);
    unregister_early_suspend(&pb->bk_early_suspend);

	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int __init pwm_backlight_init(void)
{
	return platform_driver_register(&pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

