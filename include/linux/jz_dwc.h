#ifndef __JZ_DWC_H__
#define __JZ_DWC_H__

#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>

struct jzdwc_pin {
	short				num;
#define LOW_ENABLE			0
#define HIGH_ENABLE			1
	short 				enable_level;
};

struct dwc_jz_pri {
	struct clk		*clk;

	int 			irq;
	struct jzdwc_pin 	*dete;
	struct delayed_work	work;
	struct delayed_work	charger_delay_work;

	struct regulator 	*vbus;
	struct regulator 	*ucharger;

	spinlock_t		lock;
	struct mutex		mutex;
	void			*core_if;
	int			pullup_on;

	void			(*start)(struct dwc_jz_pri *jz_pri);
	void			(*callback)(struct dwc_jz_pri *jz_pri);
};

#endif
