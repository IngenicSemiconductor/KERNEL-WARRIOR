#ifndef _CLIB_H_
#define _CLIB_H_

#ifdef  __KERNEL__
#include <linux/string.h>
#include <linux/math64.h>
#else
#include <string.h>
#endif

unsigned int nm_sleep(unsigned int seconds);
long long nd_getcurrentsec_ns(void);
unsigned int nd_get_timestamp(void);

enum nm_msg_type {
	NM_MSG_ERASE_PERSENT,
};

int nm_print_message(enum nm_msg_type type, int arg);

#endif /*_CLIB_H_*/
