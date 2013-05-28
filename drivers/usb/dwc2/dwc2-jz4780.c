#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/usb/otg.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/jz_dwc.h>
#include <soc/base.h>
#include <soc/cpm.h>

#include "core.h"
#include "gadget.h"

#define OTG_CLK_NAME		"otg1"
#define VBUS_REG_NAME		"vbus"

#define USBRDT_VBFIL_LD_EN		25
#define USBPCR_TXPREEMPHTUNE		6
#define USBPCR_POR			22
#define USBPCR_USB_MODE			31
#define USBPCR_COMMONONN		25
#define USBPCR_VBUSVLDEXT		24
#define USBPCR_VBUSVLDEXTSEL		23
#define USBPCR_OTG_DISABLE		20
#define USBPCR_IDPULLUP_MASK		28
#define OPCR_SPENDN0			7
#define USBPCR1_USB_SEL			28
#define USBPCR1_WORD_IF0		19
#define USBPCR1_WORD_IF1		18

struct dwc2_jz4780 {
	struct platform_device  dwc2;
	struct device		*dev;

	struct clk		*clk;
	int 			dete_irq;
	struct jzdwc_pin 	*dete;

	int			id_irq;
	struct jzdwc_pin 	*id_pin;
	struct delayed_work	work;
	struct delayed_work	host_id_work;

	struct regulator 	*vbus;

	struct mutex		mutex;
	int			pullup_on;
};

struct jzdwc_pin __attribute__((weak)) dete_pin = {
	.num	      = -1,
	.enable_level = -1,
};

struct jzdwc_pin __attribute__((weak)) dwc2_id_pin = {
	.num	      = -1,
	.enable_level = -1
};

#if DWC2_DEVICE_MODE_ENABLE
static int get_pin_status(struct jzdwc_pin *pin)
{
	int val;

	if (pin->num < 0)
		return -1;
	val = gpio_get_value(pin->num);

	if (pin->enable_level == LOW_ENABLE)
		return !val;
	return val;
}
#endif

void jz4780_set_vbus(struct dwc2 *dwc, int is_on)
{
#if DWC2_HOST_MODE_ENABLE
	struct dwc2_jz4780	*jz4780;

	jz4780 = container_of(dwc->pdev, struct dwc2_jz4780, dwc2);
	if ( (jz4780->vbus == NULL) || IS_ERR(jz4780->vbus) )
		return;

	printk("set vbus %s\n", is_on ? "on" : "off");

	if (is_on) {
		if (!regulator_is_enabled(jz4780->vbus))
			regulator_enable(jz4780->vbus);
	} else {
		if (regulator_is_enabled(jz4780->vbus))
			regulator_disable(jz4780->vbus);
	}
#endif	/*DWC2_HOST_MODE_ENABLE*/

}

static inline void jz4780_usb_phy_init(struct dwc2_jz4780 *jz4780)
{
	pr_debug("init PHY\n");

	cpm_set_bit(USBPCR_POR, CPM_USBPCR);
	msleep(1);
	cpm_clear_bit(USBPCR_POR, CPM_USBPCR);
	msleep(1);
}

static inline void jz4780_usb_set_device_only_mode(void)
{
	pr_info("DWC IN DEVICE ONLY MODE\n");
	cpm_clear_bit(USBPCR_USB_MODE, CPM_USBPCR);
	cpm_clear_bit(USBPCR_OTG_DISABLE, CPM_USBPCR);
}

static inline void jz4780_usb_set_dual_mode(void)
{
	unsigned int tmp;

	pr_info("DWC IN OTG MODE\n");
	tmp = cpm_inl(CPM_USBPCR);
	tmp |= 1 << USBPCR_USB_MODE;
	tmp |= 1 << USBPCR_VBUSVLDEXT;
	tmp |= 1 << USBPCR_VBUSVLDEXTSEL;
	tmp |= 1 << USBPCR_COMMONONN;

	tmp &= ~(1 << USBPCR_OTG_DISABLE);

	cpm_outl(tmp & ~(0x03 << USBPCR_IDPULLUP_MASK), CPM_USBPCR);
}

#if DWC2_DEVICE_MODE_ENABLE
extern void dwc2_gadget_plug_change(int plugin);

static void usb_detect_work(struct work_struct *work)
{
	struct dwc2_jz4780	*jz4780;
	int			 insert;

	jz4780 = container_of(work, struct dwc2_jz4780, work.work);
	insert = get_pin_status(jz4780->dete);

	pr_info("USB %s\n", insert ? "connect" : "disconnect");
	dwc2_gadget_plug_change(insert);

	enable_irq(jz4780->dete_irq);
}

static irqreturn_t usb_detect_interrupt(int irq, void *dev_id)
{
	struct dwc2_jz4780 *jz4780 = (struct dwc2_jz4780 *)dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&jz4780->work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}
#endif

#if DWC2_HOST_MODE_ENABLE
static void usb_host_id_work(struct work_struct *work) {
	struct dwc2_jz4780	*jz4780;

	jz4780 = container_of(work, struct dwc2_jz4780, host_id_work.work);

	// printk("==============>enter %s, id pin level=%d\n",
	//	__func__, gpio_get_value(jz4780->id_pin->num));

	if (gpio_get_value(jz4780->id_pin->num) == 0) { /* host */
		cpm_set_bit(7, CPM_OPCR);
	}

	enable_irq(jz4780->id_irq);
}

static irqreturn_t usb_host_id_interrupt(int irq, void *dev_id) {
	struct dwc2_jz4780 *jz4780 = (struct dwc2_jz4780 *)dev_id;

	disable_irq_nosync(irq);
	schedule_delayed_work(&jz4780->host_id_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}
#endif	/*DWC2_HOST_MODE_ENABLE*/

static void usb_cpm_init(void) {
	unsigned int ref_clk_div = CONFIG_EXTAL_CLOCK / 24;
	unsigned int usbpcr1;

	/* select dwc otg */
	cpm_set_bit(USBPCR1_USB_SEL, CPM_USBPCR1);

	/* select utmi data bus width of port0 to 16bit/30M */
	cpm_set_bit(USBPCR1_WORD_IF0, CPM_USBPCR1);

	usbpcr1 = cpm_inl(CPM_USBPCR1);
	usbpcr1 &= ~(0x3 << 24);
	usbpcr1 |= (ref_clk_div << 24);
	cpm_outl(usbpcr1, CPM_USBPCR1);

	/* fil */
	cpm_outl(0, CPM_USBVBFIL);

	/* rdt */
	cpm_outl(0x96, CPM_USBRDT);

	/* rdt - filload_en */
	cpm_set_bit(USBRDT_VBFIL_LD_EN, CPM_USBRDT);

	/* TXRISETUNE & TXVREFTUNE. */
	//cpm_outl(0x3f, CPM_USBPCR);
	//cpm_outl(0x35, CPM_USBPCR);

	/* enable tx pre-emphasis */
	//cpm_set_bit(USBPCR_TXPREEMPHTUNE, CPM_USBPCR);

	/* OTGTUNE adjust */
	//cpm_outl(7 << 14, CPM_USBPCR);

	cpm_outl(0x8180385F, CPM_USBPCR);
}

static u64 dwc2_jz4780_dma_mask = DMA_BIT_MASK(32);

static int dwc2_jz4780_probe(struct platform_device *pdev) {
	struct platform_device		*dwc2;
	struct dwc2_jz4780		*jz4780;
	struct dwc2_platform_data	*dwc2_plat_data;
	int				 ret	      = -ENOMEM;

	jz4780 = kzalloc(sizeof(*jz4780), GFP_KERNEL);
	if (!jz4780) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto out;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we move to full device tree support this will vanish off.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &dwc2_jz4780_dma_mask;

	platform_set_drvdata(pdev, jz4780);

	dwc2 = &jz4780->dwc2;
	dwc2->name = "dwc2";
	dwc2->id = -1;
	device_initialize(&dwc2->dev);
	dma_set_coherent_mask(&dwc2->dev, pdev->dev.coherent_dma_mask);
	dwc2->dev.parent = &pdev->dev;
	dwc2->dev.dma_mask = pdev->dev.dma_mask;
	dwc2->dev.dma_parms = pdev->dev.dma_parms;

	dwc2->dev.platform_data = kzalloc(sizeof(struct dwc2_platform_data), GFP_KERNEL);
	if (!dwc2->dev.platform_data) {
		goto fail_alloc_dwc2_plat_data;
	}
	dwc2_plat_data = dwc2->dev.platform_data;

	mutex_init(&jz4780->mutex);
	jz4780->dev	= &pdev->dev;
	jz4780->dete = &dete_pin;
	jz4780->id_pin = &dwc2_id_pin;

	jz4780->clk = clk_get(NULL, OTG_CLK_NAME);
	if (IS_ERR(jz4780->clk)) {
		dev_err(&pdev->dev, "clk gate get error\n");
		goto fail_get_clk;
	}
	clk_enable(jz4780->clk);

#if DWC2_HOST_MODE_ENABLE
#ifdef CONFIG_REGULATOR
	jz4780->vbus = regulator_get(NULL, VBUS_REG_NAME);

	if (IS_ERR(jz4780->vbus)) {
		dev_err(&pdev->dev, "regulator %s get error\n", VBUS_REG_NAME);
		goto fail_get_vbus_reg;
	}
	if (regulator_is_enabled(jz4780->vbus))
		regulator_disable(jz4780->vbus);
#else
#error DWC OTG driver can NOT work without regulator!
#endif /*CONFIG_REGULATOR*/
#endif /*DWC2_HOST_MODE_ENABLE */

	jz4780->dete_irq = -1;
#if DWC2_DEVICE_MODE_ENABLE
	ret = gpio_request_one(jz4780->dete->num,
			GPIOF_DIR_IN, "usb-charger-detect");
	if (ret == 0) {
		jz4780->dete_irq = gpio_to_irq(jz4780->dete->num);
		INIT_DELAYED_WORK(&jz4780->work, usb_detect_work);

		ret = request_irq(gpio_to_irq(jz4780->dete->num),
				usb_detect_interrupt,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"usb-detect", jz4780);
		if (ret) {
			dev_err(&pdev->dev, "request usb-detect fail\n");
			goto fail_req_dete_irq;
		} else {
			disable_irq(jz4780->dete_irq);
		}
	}
#endif

	jz4780->id_irq = -1;
#if DWC2_HOST_MODE_ENABLE
	ret = gpio_request_one(jz4780->id_pin->num,
			GPIOF_DIR_IN, "usb-host-id-detect");
	if (ret == 0) {
		jz4780->id_irq = gpio_to_irq(jz4780->id_pin->num);
		INIT_DELAYED_WORK(&jz4780->host_id_work, usb_host_id_work);

		ret = request_irq(gpio_to_irq(jz4780->id_pin->num),
				usb_host_id_interrupt,
				IRQF_TRIGGER_FALLING,
				"usb-host-id", jz4780);
		if (ret) {
			jz4780->id_irq = -1;
			dev_err(&pdev->dev, "request host id interrupt fail!\n");
			goto fail_req_id_irq;
		} else {
			disable_irq(jz4780->id_irq);
		}
	} else {
		dwc2_plat_data->keep_phy_on = 1;
	}
#endif

#ifdef CONFIG_BOARD_HAS_NO_DETE_FACILITY
	if (jz4780->dete_irq < 0) {
		dwc2_plat_data->keep_phy_on = 1;
	}
#endif

	usb_cpm_init();

#if (DWC2_DEVICE_MODE_ENABLE) && !(DWC2_HOST_MODE_ENABLE)
	jz4780_usb_set_device_only_mode();
#else
	jz4780_usb_set_dual_mode();
#endif

	jz4780_usb_phy_init(jz4780);

	if (dwc2_plat_data->keep_phy_on)
		cpm_set_bit(7, CPM_OPCR);
	else
		cpm_clear_bit(7, CPM_OPCR);

	/*
	 * Close VBUS detect in DWC-OTG PHY.
	 */
	*(unsigned int*)0xb3500000 |= 0xc;

	ret = platform_device_add_resources(dwc2, pdev->resource,
					pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "couldn't add resources to dwc2 device\n");
		goto fail_add_dwc2_res;
	}

	ret = platform_device_add(dwc2);
	if (ret) {
		dev_err(&pdev->dev, "failed to register dwc2 device\n");
		goto fail_add_dwc2_dev;
	}

	if (jz4780->id_irq >= 0) {
		schedule_delayed_work(&jz4780->host_id_work, msecs_to_jiffies(10));
	}

	if (jz4780->dete_irq >= 0) {
		schedule_delayed_work(&jz4780->work, msecs_to_jiffies(10));
	}

#ifdef CONFIG_BOARD_HAS_NO_DETE_FACILITY
	if (jz4780->dete_irq < 0) {
		dwc2_gadget_plug_change(1);
	}
#endif

	return 0;

fail_add_dwc2_dev:
fail_add_dwc2_res:
#if DWC2_HOST_MODE_ENABLE
fail_req_id_irq:
#endif
#if DWC2_DEVICE_MODE_ENABLE
	free_irq(gpio_to_irq(jz4780->dete->num), jz4780);
fail_req_dete_irq:
	if (gpio_is_valid(jz4780->dete->num))
		gpio_free(jz4780->dete->num);
#endif
#if DWC2_HOST_MODE_ENABLE
#ifdef CONFIG_REGULATOR
	regulator_put(jz4780->vbus);
fail_get_vbus_reg:
#endif	/* CONFIG_REGULATOR */
#endif  /* DWC2_HOST_MODE_ENABLE */
	clk_disable(jz4780->clk);
	clk_put(jz4780->clk);

fail_get_clk:
	kfree(dwc2->dev.platform_data);

fail_alloc_dwc2_plat_data:
	kfree(jz4780);
out:
	return ret;
}

static int dwc2_jz4780_remove(struct platform_device *pdev) {
	struct dwc2_jz4780	*jz4780 = platform_get_drvdata(pdev);

	if (jz4780->dete_irq >= 0) {
		free_irq(jz4780->dete_irq, jz4780);
		gpio_free(jz4780->dete->num);
	}

	if (jz4780->id_irq >= 0) {
		free_irq(jz4780->id_irq, jz4780);
		gpio_free(jz4780->id_pin->num);
	}

#if DWC2_HOST_MODE_ENABLE
	if (!IS_ERR(jz4780->vbus))
		regulator_put(jz4780->vbus);
#endif

	clk_disable(jz4780->clk);
	clk_put(jz4780->clk);

	platform_device_unregister(&jz4780->dwc2);
	kfree(jz4780);

	return 0;
}


static struct platform_driver dwc2_jz4780_driver = {
	.probe		= dwc2_jz4780_probe,
	.remove		= dwc2_jz4780_remove,
	.driver		= {
		.name	= "jz4780-dwc2",
		.owner =  THIS_MODULE,
	},
};


static int __init dwc2_jz4780_init(void)
{
	return platform_driver_register(&dwc2_jz4780_driver);
}

static void __exit dwc2_jz4780_exit(void)
{
	platform_driver_unregister(&dwc2_jz4780_driver);
}

module_init(dwc2_jz4780_init);
module_exit(dwc2_jz4780_exit);

MODULE_ALIAS("platform:jz4780-dwc2");
MODULE_AUTHOR("Lutts Cao <slcao@ingenic.cn>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DesignWare USB2.0 JZ4780 Glue Layer");
