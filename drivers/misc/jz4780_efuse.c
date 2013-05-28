#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>

#include <soc/base.h>
#include <mach/jz4780_efuse.h>

#define JZ_REG_EFUSE_CTRL		0x0
#define JZ_REG_EFUSE_CFG		0x4
#define JZ_REG_EFUSE_STATE		0x8
#define JZ_REG_EFUSE_DATA(n)		(0xC + (n)*4)

/* EFUSE Status Register  (OTP_STATE) */
#define JZ_EFUSE_STATE_GLOBAL_PRT	(1 << 15)
#define JZ_EFUSE_STATE_CHIPID_PRT	(1 << 14)
#define JZ_EFUSE_STATE_CUSTID_PRT	(1 << 13)
#define JZ_EFUSE_STATE_SECWR_EN		(1 << 12)
#define JZ_EFUSE_STATE_PC_PRT		(1 << 11)
#define JZ_EFUSE_STATE_HDMIKEY_PRT	(1 << 10)
#define JZ_EFUSE_STATE_SECKEY_PRT	(1 << 9)
#define JZ_EFUSE_STATE_SECBOOT_EN	(1 << 8)
#define JZ_EFUSE_STATE_HDMI_BUSY	(1 << 2)
#define JZ_EFUSE_STATE_WR_DONE		(1 << 1)
#define JZ_EFUSE_STATE_RD_DONE		(1 << 0)
/*EFUSE PROTECT BIT*/
#define JZ_EFUSE_MASK_GLOBAL_PRT	(1 << 7)
#define JZ_EFUSE_MASK_CHIPID_PRT        (1 << 6)
#define JZ_EFUSE_MASK_CUSTID_PRT        (1 << 5)
#define JZ_EFUSE_MASK_SECWR_EN		(1 << 4)
#define JZ_EFUSE_MASK_PC_PRT		(1 << 3)
#define JZ_EFUSE_MASK_HDMIKEY_PRT	(1 << 2)
#define JZ_EFUSE_MASK_SECKEY_PRT	(1 << 1)
#define JZ_EFUSE_MASK_SECBOOT_EN	(1 << 0)

static void __iomem *jz_efuse_base;
static spinlock_t jz_efuse_lock;
static int gpio_vddq_en_n;
struct timer_list vddq_protect_timer;

static uint32_t jz_efuse_reg_read(int reg)
{
	return readl(jz_efuse_base + reg);
}

static void jz_efuse_reg_write(int reg, uint32_t val)
{
	writel(val, jz_efuse_base + reg);
}

/*
static void jz_efuse_reg_write_mask(int reg, uint32_t val, uint32_t mask)
{
	uint32_t val2;

	val2 = readl(jz_efuse_base + reg);
	val2 &= ~mask;
	val2 |= val;
	writel(val2, jz_efuse_base + reg);
}
*/

static void jz_efuse_reg_set_bits(int reg, uint32_t mask)
{
	uint32_t val;

	val = readl(jz_efuse_base + reg);
	val |= mask;
	writel(val, jz_efuse_base + reg);
}

static void jz_efuse_reg_clear_bits(int reg, uint32_t mask)
{
	uint32_t val;

	val = readl(jz_efuse_base + reg);
	val &= ~mask;
	writel(val, jz_efuse_base + reg);
}

static void jz_efuse_vddq_set(unsigned long is_on)
{
	printk("JZ4780-EFUSE: vddq_set %d\n", (int)is_on);
	if (is_on) {
		mod_timer(&vddq_protect_timer, jiffies + HZ);
	}
	gpio_set_value(gpio_vddq_en_n, !is_on);
}

void jz_efuse_id_read(int is_chip_id, uint32_t *buf)
{
	int i;

	spin_lock(&jz_efuse_lock);

	if (is_chip_id)
		jz_efuse_reg_write(JZ_REG_EFUSE_CTRL, 0x8 << 21 | 15 << 16 | 1 << 0);
	else
		jz_efuse_reg_write(JZ_REG_EFUSE_CTRL, 0x18 << 21 | 15 << 16 | 1 << 0);
	while (!(jz_efuse_reg_read(JZ_REG_EFUSE_STATE) & JZ_EFUSE_STATE_RD_DONE));
	for (i = 0; i < 4; i++)
		*(buf + i) = jz_efuse_reg_read(JZ_REG_EFUSE_DATA(i));

	spin_unlock(&jz_efuse_lock);
}
EXPORT_SYMBOL_GPL(jz_efuse_id_read);

static void jz_efuse_id_write(int is_chip_id, uint32_t *buf)
{
	int i;

	if (gpio_vddq_en_n == -ENODEV) {
		printk("JZ4780-EFUSE: The VDDQ can't be opened by software!\n");
		return;
	}

	spin_lock(&jz_efuse_lock);

	for (i = 0; i < 4; i++)
		jz_efuse_reg_write(JZ_REG_EFUSE_DATA(i), *(buf + i));
	if (is_chip_id)
		jz_efuse_reg_write(JZ_REG_EFUSE_CTRL, 0x8 << 21 | 15 << 16 | 1 << 15);
	else
		jz_efuse_reg_write(JZ_REG_EFUSE_CTRL, 0x18 << 21 | 15 << 16 | 1 << 15);
	jz_efuse_vddq_set(1);
	jz_efuse_reg_set_bits(JZ_REG_EFUSE_CTRL, 1 << 1);
	while (!(jz_efuse_reg_read(JZ_REG_EFUSE_STATE) & JZ_EFUSE_STATE_WR_DONE));
	jz_efuse_vddq_set(0);
	jz_efuse_reg_clear_bits(JZ_REG_EFUSE_STATE, 1 << 15);

	spin_unlock(&jz_efuse_lock);
}

static ssize_t jz_efuse_chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t data[4];

	jz_efuse_id_read(1, data);
	return snprintf(buf, PAGE_SIZE, "%08x %08x %08x %08x\n", data[0], data[1], data[2], data[3]);
}

static ssize_t jz_efuse_user_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t data[4];

	jz_efuse_id_read(0, data);
	return snprintf(buf, PAGE_SIZE, "%08x %08x %08x %08x\n", data[0], data[1], data[2], data[3]);
}

static ssize_t jz_efuse_chip_id_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t data[4];

	sscanf (buf, "%08x %08x %08x %08x", &data[0], &data[1], &data[2], &data[3]);
	dev_info(dev, "chip id store: %08x %08x %08x %08x\n", data[0], data[1], data[2], data[3]);
	jz_efuse_id_write(1, data);
	return strnlen(buf, PAGE_SIZE);
}

static ssize_t jz_efuse_user_id_store(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t data[4];

	sscanf (buf, "%08x %08x %08x %08x", &data[0], &data[1], &data[2], &data[3]);
	dev_info(dev, "user id store: %08x %08x %08x %08x\n", data[0], data[1], data[2], data[3]);
	jz_efuse_id_write(0, data);
	return strnlen(buf, PAGE_SIZE);
}

static struct device_attribute jz_efuse_sysfs_attrs[] = {
    __ATTR(chip_id, S_IRUGO | S_IWUSR, jz_efuse_chip_id_show, jz_efuse_chip_id_store),
    __ATTR(user_id, S_IRUGO | S_IWUSR, jz_efuse_user_id_show, jz_efuse_user_id_store),
};

static int efuse_probe(struct platform_device *pdev)
{
	int ret;
	struct jz4780_efuse_platform_data *pdata;

	pdata = pdev->dev.platform_data;

        gpio_vddq_en_n = -ENODEV;

	if (!pdata)
		dev_err(&pdev->dev, "No platform data\n");
	else
		gpio_vddq_en_n = pdata->gpio_vddq_en_n;

	jz_efuse_base = ioremap(NEMC_IOBASE + 0xd0, 0x2c);
	if (!jz_efuse_base) {
		dev_err(&pdev->dev, "ioremap failed!\n");
		return -EBUSY;
	}

	if (gpio_vddq_en_n != -ENODEV) {
		ret = gpio_request(gpio_vddq_en_n, dev_name(&pdev->dev));
		if (ret) {
			dev_err(&pdev->dev, "Failed to request gpio pin: %d\n", ret);
			goto err_free_io;
		}

		ret = gpio_direction_output(gpio_vddq_en_n, 1); /* power off by default */
		if (ret) {
			dev_err(&pdev->dev, "Failed to set gpio as output: %d\n", ret);
			goto err_free_gpio;
		}
	}

	spin_lock_init(&jz_efuse_lock);
	setup_timer(&vddq_protect_timer, jz_efuse_vddq_set, 0);
	add_timer(&vddq_protect_timer);
	{
	    int i;
	    for (i = 0; i < ARRAY_SIZE(jz_efuse_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &jz_efuse_sysfs_attrs[i]);
		if (ret)
		    break;
	    }
	}

	dev_info(&pdev->dev,"ingenic efuse interface module registered.\n");

	return 0;

err_free_gpio:
	gpio_free(gpio_vddq_en_n);
err_free_io:
	iounmap(jz_efuse_base);
	return ret;
}

static int __devexit efuse_remove(struct platform_device *pdev)
{
	gpio_free(gpio_vddq_en_n);
	iounmap(jz_efuse_base);
	del_timer(&vddq_protect_timer);
	return 0;
}

static struct platform_driver efuse_driver = {
	.driver.name	= "jz4780-efuse",
	.driver.owner	= THIS_MODULE,
	.probe		= efuse_probe,
	.remove		= efuse_remove,
};

static int __init efuse_init(void)
{
	return platform_driver_register(&efuse_driver);
}

static void __exit efuse_exit(void)
{
	platform_driver_unregister(&efuse_driver);
}

module_init(efuse_init);
module_exit(efuse_exit);
MODULE_LICENSE("GPL");
