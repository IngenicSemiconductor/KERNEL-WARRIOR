#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/math64.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "os/clib.h"

unsigned int nm_sleep(unsigned int seconds)
{
	msleep(seconds * 1000);
	return 0;
}

long long nd_getcurrentsec_ns(void)
{
	return sched_clock();
}

unsigned int nd_get_timestamp(void)
{
	return jiffies_to_msecs(jiffies);
}

int nm_print_message(enum nm_msg_type type, int arg)
{
	switch (type) {
	case NM_MSG_ERASE_PERSENT:
		printk("------------------> persent = %d\n", arg);
		break;
	}

	return 0;
}
