/**
 * nanddebug.c
 **/

#include <linux/kernel.h>

#include <linux/module.h>
#include <linux/moduleparam.h>

int nm_dbg_level = 3;

module_param(nm_dbg_level,int,0644);

int __ndprint(const char *fmt, ...)
{
	va_list args;
	int ret;

	va_start(args, fmt);
	ret = vprintk(fmt, args);
	va_end(args);

	return ret;
}

void nd_dump_stack(void) {
	dump_stack();
}

static int __init nddebug_setup(char *str)
{
	get_option(&str, &nm_dbg_level);
	return 1;
}

__setup("nddebug=", nddebug_setup);


