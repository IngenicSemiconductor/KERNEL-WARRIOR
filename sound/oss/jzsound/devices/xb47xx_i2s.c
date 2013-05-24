/**
 * xb_snd_i2s.c
 *
 * jbbi <jbbi@ingenic.cn>
 *
 * 24 APR 2012
 *
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/vmalloc.h>
#include <linux/string.h>
#include <linux/sound.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <linux/switch.h>
#include <linux/dma-mapping.h>
#include <linux/soundcard.h>
#include <linux/earlysuspend.h>
#include <linux/wait.h>
#include <mach/jzdma.h>
#include <mach/jzsnd.h>
#include <soc/irq.h>
#include <soc/base.h>
#include "xb47xx_i2s.h"
#include "codecs/jz4780_codec.h"
/**
 * global variable
 **/
void volatile __iomem *volatile i2s_iomem;
static LIST_HEAD(codecs_head);
static spinlock_t i2s_irq_lock;
struct snd_switch_data switch_data;
static volatile int jz_switch_state = 0;
static struct dsp_endpoints i2s_endpoints;
static struct clk *codec_sysclk = NULL;
static volatile bool i2s_is_incall_state = false;
bool i2s_is_incall(void);

#ifdef CONFIG_JZ4780_INTERNAL_CODEC
static struct workqueue_struct *i2s_work_queue;
static struct work_struct	i2s_codec_work;
#endif

static int jz_get_hp_switch_state(void);

static struct codec_info {
	struct list_head list;
	char *name;
	unsigned long record_rate;
	unsigned long replay_rate;
	int record_codec_channel;
	int replay_codec_channel;
	int record_format;
	int replay_format;
	enum codec_mode codec_mode;
	unsigned long codec_clk;
	int (*codec_ctl)(unsigned int cmd, unsigned long arg);
	struct dsp_endpoints *dsp_endpoints;
} *cur_codec;

/*##################################################################*\
 | dump
\*##################################################################*/

static void dump_i2s_reg(void)
{
	int i;
	unsigned long reg_addr[] = {
		AICFR,AICCR,I2SCR,AICSR,I2SSR,I2SDIV
	};

	for (i = 0;i < 6; i++) {
		printk("##### aic reg0x%x,=0x%x.\n",
			(unsigned int)reg_addr[i],i2s_read_reg(reg_addr[i]));
	}
}

/*##################################################################*\
 |* suspand func
\*##################################################################*/
static int i2s_suspend(struct platform_device *, pm_message_t state);
static int i2s_resume(struct platform_device *);
static void i2s_shutdown(struct platform_device *);

/*##################################################################*\
  |* codec control
\*##################################################################*/
/**
 * register the codec
 **/

int i2s_register_codec(char *name, void *codec_ctl,unsigned long codec_clk,enum codec_mode mode)
{
	struct codec_info *tmp = vmalloc(sizeof(struct codec_info));
	if ((name != NULL) && (codec_ctl != NULL)) {
		tmp->name = name;
		tmp->codec_ctl = codec_ctl;
		tmp->codec_clk = codec_clk;
		tmp->codec_mode = mode;
		tmp->dsp_endpoints = &i2s_endpoints;
		list_add_tail(&tmp->list, &codecs_head);
		return 0;
	} else {
		return -1;
	}
}
EXPORT_SYMBOL(i2s_register_codec);

static void i2s_match_codec(char *name)
{
	struct codec_info *codec_info;
	struct list_head  *list,*tmp;

	list_for_each_safe(list,tmp,&codecs_head) {
		codec_info = container_of(list,struct codec_info,list);
		if (!strcmp(codec_info->name,name)) {
			cur_codec = codec_info;
		}
	}
}

bool i2s_is_incall(void)
{
	return i2s_is_incall_state;
}

/*##################################################################*\
|* filter opt
\*##################################################################*/
static void i2s_set_filter(int mode , uint32_t channels)
{
	struct dsp_pipe *dp = NULL;

	if (mode & CODEC_RMODE)
		dp = cur_codec->dsp_endpoints->in_endpoint;
	else
		return;

	switch(cur_codec->record_format) {
		case AFMT_U8:
			if (channels == 1) {
				dp->filter = convert_8bits_stereo2mono_signed2unsigned;
				printk("dp->filter convert_8bits_stereo2mono_signed2unsigned .\n");
			}
			else {
				dp->filter = convert_8bits_signed2unsigned;
				printk("dp->filter convert_8bits_signed2unsigned.\n");
			}
			break;
		case AFMT_S8:
			if (channels == 1) {
				dp->filter = convert_8bits_stereo2mono;
				printk("dp->filter convert_8bits_stereo2mono\n");
			}
			else {
				dp->filter = NULL;
				printk("dp->filter null\n");
			}
			break;
		case AFMT_S16_BE:
		case AFMT_S16_LE:
			if (channels == 1) {
#if 0
				dp->filter = convert_16bits_stereomix2mono;
				printk("dp->filter convert_16bits_stereomix2mono\n");
#else
				dp->filter = convert_16bits_stereo2mono;
				printk("dp->filter convert_16bits_stereo2mono\n");
#endif
			}
			else {
				dp->filter = NULL;
				printk("dp->filter null\n");
			}
			break;
		default :
			dp->filter = NULL;
			printk("AUDIO DEVICE :filter set error.\n");
	}
}

/*##################################################################*\
|* dev_ioctl
\*##################################################################*/
static int i2s_set_fmt(unsigned long *format,int mode)
{
	int ret = 0;
	int data_width = 0;

    /*
	 * The value of format reference to soundcard.
	 * AFMT_MU_LAW      0x00000001
	 * AFMT_A_LAW       0x00000002
	 * AFMT_IMA_ADPCM   0x00000004
	 * AFMT_U8			0x00000008
	 * AFMT_S16_LE      0x00000010
	 * AFMT_S16_BE      0x00000020
	 * AFMT_S8			0x00000040
	 */
	debug_print("format = %d",*format);
	switch (*format) {

	case AFMT_U8:
		data_width = 8;
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(0);
			__i2s_disable_byteswap();
			__i2s_enable_signadj();
		}
		if (mode & CODEC_RMODE) {
			__i2s_set_iss_sample_size(0);
			__i2s_disable_signadj();
		}
		break;
	case AFMT_S8:
		data_width = 8;
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(0);
			__i2s_disable_byteswap();
		}
		if (mode & CODEC_RMODE)
			__i2s_set_iss_sample_size(0);
		__i2s_disable_signadj();
		break;
	case AFMT_S16_LE:
		data_width = 16;
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(1);
			__i2s_disable_byteswap();
		}
		if (mode & CODEC_RMODE)
			__i2s_set_iss_sample_size(1);
		__i2s_disable_signadj();
		break;
	case AFMT_S16_BE:
		data_width = 16;
		if (mode & CODEC_WMODE) {
			__i2s_set_oss_sample_size(1);
			__i2s_enable_byteswap();
		}
		if (mode == CODEC_RMODE) {
			__i2s_set_iss_sample_size(1);
			*format = AFMT_S16_LE;
		}
		__i2s_disable_signadj();
		break;
	default :
		printk("I2S: there is unknown format 0x%x.\n",(unsigned int)*format);
		return -EINVAL;
	}
	if (!cur_codec)
		return -ENODEV;

	if (mode & CODEC_WMODE) {
		if ((ret = cur_codec->codec_ctl(CODEC_SET_REPLAY_DATA_WIDTH, data_width)) != 0) {
			printk("JZ I2S: CODEC ioctl error, command: CODEC_SET_REPLAY_FORMAT");
			return ret;
		}
		if (cur_codec->replay_format != *format) {
			cur_codec->replay_format = *format;
			ret |= NEED_RECONF_TRIGGER;
			ret |= NEED_RECONF_DMA;
		}
	}

	if (mode & CODEC_RMODE) {
		if ((ret = cur_codec->codec_ctl(CODEC_SET_RECORD_DATA_WIDTH, data_width)) != 0) {
			printk("JZ I2S: CODEC ioctl error, command: CODEC_SET_RECORD_FORMAT");
			return ret;
		}
		if (cur_codec->record_format != *format) {
			cur_codec->record_format = *format;
			ret |= NEED_RECONF_TRIGGER | NEED_RECONF_FILTER;
			ret |= NEED_RECONF_DMA;
		}
	}

	return ret;
}

static int i2s_set_channel(int* channel,int mode)
{
	int ret = 0;
#ifdef CONFIG_ANDROID
	int channel_save = 0;
#endif
	if (!cur_codec)
		return -ENODEV;
	debug_print("channel = %d",*channel);
	if (mode & CODEC_WMODE) {
		ret = cur_codec->codec_ctl(CODEC_SET_REPLAY_CHANNEL,(unsigned long)channel);
		if (ret < 0) {
			cur_codec->replay_codec_channel = *channel;
			return ret;
		}
		if (*channel ==2 || *channel == 4||
			*channel ==6 || *channel == 8) {
			__i2s_out_channel_select(*channel - 1);
			__i2s_disable_mono2stereo();
		} else if (*channel == 1) {
			__i2s_out_channel_select(*channel - 1);
			__i2s_enable_mono2stereo();
		} else
			return -EINVAL;
		if (cur_codec->replay_codec_channel != *channel) {
			cur_codec->replay_codec_channel = *channel;
			ret |= NEED_RECONF_FILTER;
		}
	}
	if (mode & CODEC_RMODE) {
#ifdef CONFIG_ANDROID
		{
			channel_save = *channel;
			if (*channel == 1)
				*channel = 2;
		}
#endif
		ret = cur_codec->codec_ctl(CODEC_SET_RECORD_CHANNEL,(unsigned long)channel);
		if (ret < 0)
			return ret;
		ret = 0;
#ifdef CONFIG_ANDROID
		*channel = channel_save;
#endif
		if (cur_codec->record_codec_channel != *channel) {
			cur_codec->record_codec_channel = *channel;
			ret |= NEED_RECONF_FILTER;
		}
	}
	return ret;
}

/***************************************************************\
 *  Use codec slave mode clock rate list
 *  We do not hope change EPLL,so we use 270.67M (fix) epllclk
 *  for minimum error
 *  270.67M ---	M:203 N:9 OD:1
 *	 rate	 i2sdr	 cguclk		 i2sdv.div	samplerate/error
 *	|192000	|1		|135.335M	|10			|+0.12%
 *	|96000	|3		|67.6675M	|10			|+0.12%
 *	|48000	|7		|33.83375M	|10			|-0.11%
 *	|44100	|7		|33.83375M	|11			|-0.10%
 *	|32000	|11		|22.555833M	|10			|+0.12%
 *	|24000	|15		|16.916875M	|10			|+0.12%
 *	|22050	|15		|16.916875M	|11			|-0.12%
 *	|16000	|23		|11.277916M	|10			|+0.12%
 *	|12000  |31		|8.458437M	|10			|+0.12%
 *	|11025	|31		|8.458437M	|11			|-0.10%
 *	|8000	|47		|5.523877M	|10			|+0.12%
 *	HDMI:
 *	sysclk 11.2896M (theoretical)
 *	i2sdr  23
 *	cguclk 11.277916M (practical)
 *	error  -0.10%
\***************************************************************/
static unsigned long calculate_cgu_aic_rate(unsigned long *rate)
{
	int i;
	unsigned long mrate[10] = {
		8000, 11025,16000,22050,24000,
		32000,44100,48000,96000,192000,
	};
	unsigned long mcguclk[10] = {
		5523978, 8458438, 8458438, 11277917,16916875,
		16916875,33833750,33833750,67667500,135335000,
	};
	for (i=0; i<9; i++) {
		if (*rate <= mrate[i]) {
			*rate = mrate[i];
			break;
		}
	}
	if (i >= 9) {
		*rate = 44100; /*unsupport rate use default*/
		return mcguclk[6];
	}

	return mcguclk[i];
}

static int i2s_set_rate(unsigned long *rate,int mode)
{
	int ret = 0;
	unsigned long cgu_aic_clk = 0;
	if (!cur_codec)
		return -ENODEV;
	debug_print("rate = %ld",*rate);
	if (mode & CODEC_WMODE) {
		if (cur_codec->codec_mode == CODEC_SLAVE) {
			/*************************************************************\
			|* WARING:when use codec slave mode ,                        *|
			|* EPLL must be output 270.67M clk to support all sample rate*|
			|* SYSCLK not standard over sample rate clock ,so it would   *|
			|* not be output to external codec                           *|
			\*************************************************************/
			if (strcmp(cur_codec->name,"hdmi"))
				cgu_aic_clk = calculate_cgu_aic_rate(rate);
			else
				cgu_aic_clk = cur_codec->codec_clk;
			__i2s_stop_bitclk();
			if (cur_codec->codec_clk != cgu_aic_clk || !strcmp(cur_codec->name,"hdmi")) {
				cur_codec->codec_clk = cgu_aic_clk;
				if (codec_sysclk == NULL)
					return -1;
				clk_set_rate(codec_sysclk,cur_codec->codec_clk);
				if (clk_get_rate(codec_sysclk) > cur_codec->codec_clk) {
					printk("external codec set rate fail.\n");
				}
			}
			*rate = __i2s_set_sample_rate(cur_codec->codec_clk,*rate);
			__i2s_start_bitclk();
		}
		ret = cur_codec->codec_ctl(CODEC_SET_REPLAY_RATE,(unsigned long)rate);
		cur_codec->replay_rate = *rate;
	}
	if (mode & CODEC_RMODE) {
		if (cur_codec->codec_mode == CODEC_SLAVE) {
			cgu_aic_clk = calculate_cgu_aic_rate(rate);
			if (strcmp(cur_codec->name,"hdmi"))
				return 0;
			__i2s_stop_ibitclk();
			if (cur_codec->codec_clk != cgu_aic_clk) {
				cur_codec->codec_clk = cgu_aic_clk;
				if (codec_sysclk == NULL)
					return -1;
				clk_set_rate(codec_sysclk,cur_codec->codec_clk);
				if (clk_get_rate(codec_sysclk) > cur_codec->codec_clk) {
					printk("external codec set rate fail.\n");
				}
			}
			*rate = __i2s_set_isample_rate(cur_codec->codec_clk,*rate);
			__i2s_start_ibitclk();
		}
		ret = cur_codec->codec_ctl(CODEC_SET_RECORD_RATE,(unsigned long)rate);
		cur_codec->record_rate = *rate;
	}
	return ret;
}
#define I2S_TX_FIFO_DEPTH		64
#define I2S_RX_FIFO_DEPTH		32

static int get_burst_length(unsigned long val)
{
	/* burst bytes for 1,2,4,8,16,32,64 bytes */
	int ord;

	ord = ffs(val) - 1;
	if (ord < 0)
		ord = 0;
	else if (ord > 6)
		ord = 6;

	/* if tsz == 8, set it to 4 */
	return (ord == 3 ? 4 : 1 << ord)*8;
}

static void i2s_set_trigger(int mode)
{
	int data_width = 0;
	struct dsp_pipe *dp = NULL;
	int burst_length = 0;

	if (!cur_codec)
		return;

	if (mode & CODEC_WMODE) {
		switch(cur_codec->replay_format) {
		case AFMT_S8:
		case AFMT_U8:
			data_width = 8;
			break;
		default:
		case AFMT_S16_BE:
		case AFMT_S16_LE:
			data_width = 16;
			break;
		}
		dp = cur_codec->dsp_endpoints->out_endpoint;
		burst_length = get_burst_length((int)dp->paddr|(int)dp->fragsize|
				dp->dma_config.src_maxburst|dp->dma_config.dst_maxburst);
		if (I2S_TX_FIFO_DEPTH - burst_length/data_width >= 60)
			__i2s_set_transmit_trigger(30);
		else
			__i2s_set_transmit_trigger((I2S_TX_FIFO_DEPTH - burst_length/data_width) >> 1);

	}
	if (mode &CODEC_RMODE) {
		switch(cur_codec->record_format) {
		case AFMT_S8:
		case AFMT_U8:
			data_width = 8;
			break;
		default :
		case AFMT_S16_BE:
		case AFMT_S16_LE:
			data_width = 16;
			break;
		}
		dp = cur_codec->dsp_endpoints->in_endpoint;
		burst_length = get_burst_length((int)dp->paddr|(int)dp->fragsize|
				dp->dma_config.src_maxburst|dp->dma_config.dst_maxburst);
		__i2s_set_receive_trigger(((I2S_RX_FIFO_DEPTH - burst_length/data_width) >> 1) - 1);
	}

	return;
}

static int i2s_enable(int mode)
{
	unsigned long replay_rate = 44100;
	unsigned long record_rate = 8000;
	unsigned long replay_format = 16;
	unsigned long record_format = 16;
	int replay_channel = 2;
	int record_channel = 1;
	struct dsp_pipe *dp_other = NULL;

	if (!cur_codec)
			return -ENODEV;

	cur_codec->codec_ctl(CODEC_ANTI_POP,mode);

	if (mode & CODEC_WMODE) {
		dp_other = cur_codec->dsp_endpoints->in_endpoint;
		i2s_set_fmt(&replay_format,mode);
		i2s_set_channel(&replay_channel,mode);
		i2s_set_rate(&replay_rate,mode);
	}
	if (mode & CODEC_RMODE) {
		dp_other = cur_codec->dsp_endpoints->out_endpoint;
		i2s_set_fmt(&record_format,mode);
		i2s_set_channel(&record_channel,mode);
		i2s_set_rate(&record_rate,mode);
	}
	i2s_set_trigger(mode);
	i2s_set_filter(mode,record_channel);

#ifndef CONFIG_ANDROID
	cur_codec->codec_ctl(CODEC_SET_DEFROUTE,mode);
#endif

	if (!dp_other->is_used) {
		/*avoid pop FIXME*/
		if (mode & CODEC_WMODE)
			__i2s_flush_tfifo();
		cur_codec->codec_ctl(CODEC_DAC_MUTE,1);
		__i2s_enable();
		__i2s_select_i2s();
		cur_codec->codec_ctl(CODEC_DAC_MUTE,0);
	}

	i2s_is_incall_state = false;

	return 0;
}

static int i2s_disable_channel(int mode)
{
	if (cur_codec)
		cur_codec->codec_ctl(CODEC_TURN_OFF,mode);

	if (mode & CODEC_WMODE) {
		__i2s_disable_replay();
	}
	if (mode & CODEC_RMODE) {
		__i2s_disable_record();
	}
	return 0;
}


static int i2s_dma_enable(int mode)		//CHECK
{
	int val;
	if (!cur_codec)
			return -ENODEV;
	if (mode & CODEC_WMODE) {
#if 0
		cur_codec->codec_ctl(CODEC_DAC_MUTE,1);
		__i2s_enable_replay();
		while(!__i2s_test_tur());
		__i2s_enable_transmit_dma();
#else
		cur_codec->codec_ctl(CODEC_DAC_MUTE,0);
		__i2s_flush_tfifo();
		__i2s_enable_transmit_dma();
		__i2s_enable_replay();
#endif
	}
	if (mode & CODEC_RMODE) {
		cur_codec->codec_ctl(CODEC_ADC_MUTE,1);
		__i2s_flush_rfifo();
		__i2s_enable_record();
		/* read the first sample and ignore it */
		val = __i2s_read_rfifo();
		cur_codec->codec_ctl(CODEC_ADC_MUTE,0);
		__i2s_enable_receive_dma();
	}

	return 0;
}

static int i2s_dma_disable(int mode)		//CHECK seq dma and func
{
	if (!cur_codec)
			return -ENODEV;
	if (mode & CODEC_WMODE) {
		__i2s_disable_transmit_dma();
		__i2s_disable_replay();
	}
	if (mode & CODEC_RMODE) {
		__i2s_disable_receive_dma();
		__i2s_disable_record();
	}
	return 0;
}

static int i2s_get_fmt_cap(unsigned long *fmt_cap,int mode)
{
	unsigned long i2s_fmt_cap = 0;
	if (!cur_codec)
			return -ENODEV;
	if (mode & CODEC_WMODE) {
		i2s_fmt_cap |= AFMT_S16_LE|AFMT_S16_BE|AFMT_S8|AFMT_U8;
		cur_codec->codec_ctl(CODEC_GET_REPLAY_FMT_CAP, *fmt_cap);

	}
	if (mode & CODEC_RMODE) {
		i2s_fmt_cap |= AFMT_S16_LE|AFMT_S8|AFMT_U8;
		cur_codec->codec_ctl(CODEC_GET_RECORD_FMT_CAP, *fmt_cap);
	}

	if (*fmt_cap == 0)
		*fmt_cap = i2s_fmt_cap;
	else
		*fmt_cap &= i2s_fmt_cap;

	return 0;
}


static int i2s_get_fmt(unsigned long *fmt, int mode)
{
	if (!cur_codec)
			return -ENODEV;

	if (mode & CODEC_WMODE)
		*fmt = cur_codec->replay_format;
	if (mode & CODEC_RMODE)
		*fmt = cur_codec->record_format;

	return 0;
}

static void i2s_dma_need_reconfig(int mode)
{
	struct dsp_pipe	*dp = NULL;

	if (!cur_codec)
			return;
	if (mode & CODEC_WMODE) {
		dp = cur_codec->dsp_endpoints->out_endpoint;
		dp->need_reconfig_dma = true;
	}
	if (mode & CODEC_RMODE) {
		dp = cur_codec->dsp_endpoints->in_endpoint;
		dp->need_reconfig_dma = true;
	}
	return;
}

static int i2s_set_device(unsigned long device)
{
	unsigned long tmp_rate = 44100;
  	int ret = 0;

	if (!cur_codec)
		return -1;

	/*call state operation*/
	if (*(enum snd_device_t *)device >= SND_DEVICE_CALL_START &&
            *(enum snd_device_t *)device <= SND_DEVICE_CALL_END)
		i2s_is_incall_state = true;
	else
		i2s_is_incall_state = false;

	if (*(enum snd_device_t *)device == SND_DEVICE_LOOP_TEST) {
		cur_codec->codec_ctl(CODEC_ADC_MUTE,0);
		cur_codec->codec_ctl(CODEC_DAC_MUTE,0);
		i2s_set_rate(&tmp_rate,CODEC_RWMODE);
	}

	/*hdmi operation*/
	if ((tmp_rate = cur_codec->replay_rate) == 0);
		tmp_rate = 44100;
	if ((*(enum snd_device_t *)device) == SND_DEVICE_HDMI) {
		if (strcmp(cur_codec->name,"hdmi")) {
			i2s_match_codec("hdmi");
			__i2s_stop_bitclk();
			__i2s_external_codec();
			__i2s_master_clkset();
			__i2s_start_bitclk();
			i2s_set_rate(&tmp_rate,CODEC_WMODE);
		}
	} else {
		/*restore old device*/
		if (!strcmp(cur_codec->name,"hdmi")) {
#if defined(CONFIG_JZ_INTERNAL_CODEC)
			i2s_match_codec("internal_codec");
#elif defined(CONFIG_JZ_EXTERNAL_CODEC)
			i2s_match_codec("external_codec");
#endif
			__i2s_stop_bitclk();
#if defined(CONFIG_JZ_EXTERNAL_CODEC)
			__i2s_external_codec();
#endif

			if (cur_codec->codec_mode == CODEC_MASTER) {
#if defined(CONFIG_JZ_INTERNAL_CODEC)
				__i2s_internal_codec_master();
#endif
				__i2s_slave_clkset();
				__i2s_enable_sysclk_output();
			} else if (cur_codec->codec_mode == CODEC_SLAVE) {
#if defined(CONFIG_JZ_INTERNAL_CODEC)
				__i2s_internal_codec_slave();
#endif
				__i2s_master_clkset();
				__i2s_disable_sysclk_output();
			}

			clk_set_rate(codec_sysclk,cur_codec->codec_clk);
			if (clk_get_rate(codec_sysclk) > cur_codec->codec_clk) {
				printk("codec codec set rate fail.\n");
			}
			__i2s_start_bitclk();
		}
	}

	/*route operatiom*/
	ret = cur_codec->codec_ctl(CODEC_SET_DEVICE, device);

	return ret;
}

/********************************************************\
 * dev_ioctl
\********************************************************/
static long i2s_ioctl(unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	switch (cmd) {
	case SND_DSP_ENABLE_REPLAY:
		/* enable i2s record */
		/* set i2s default record format, channels, rate */
		/* set default replay route */
		ret = i2s_enable(CODEC_WMODE);
		break;

	case SND_DSP_DISABLE_REPLAY:
		/* disable i2s replay */
		ret = i2s_disable_channel(CODEC_WMODE);
		break;

	case SND_DSP_ENABLE_RECORD:
		/* enable i2s record */
		/* set i2s default record format, channels, rate */
		/* set default record route */
		ret = i2s_enable(CODEC_RMODE);
		break;

	case SND_DSP_DISABLE_RECORD:
		/* disable i2s record */
		ret = i2s_disable_channel(CODEC_RMODE);
		break;

	case SND_DSP_ENABLE_DMA_RX:
		ret = i2s_dma_enable(CODEC_RMODE);
		break;

	case SND_DSP_DISABLE_DMA_RX:
		ret = i2s_dma_disable(CODEC_RMODE);
		break;

	case SND_DSP_ENABLE_DMA_TX:
		ret = i2s_dma_enable(CODEC_WMODE);
		break;

	case SND_DSP_DISABLE_DMA_TX:
		ret = i2s_dma_disable(CODEC_WMODE);
		break;

	case SND_DSP_SET_REPLAY_RATE:
		ret = i2s_set_rate((unsigned long *)arg,CODEC_WMODE);
		break;

	case SND_DSP_SET_RECORD_RATE:
		ret = i2s_set_rate((unsigned long *)arg,CODEC_RMODE);
		break;

	case SND_DSP_GET_REPLAY_RATE:
		if (cur_codec && cur_codec->replay_rate)
			*(unsigned long *)arg = cur_codec->replay_rate;
		ret = 0;
		break;

	case SND_DSP_GET_RECORD_RATE:
		if (cur_codec && cur_codec->record_rate)
			*(unsigned long *)arg = cur_codec->record_rate;
		ret = 0;
		break;


	case SND_DSP_SET_REPLAY_CHANNELS:
		/* set replay channels */
		ret = i2s_set_channel((int *)arg,CODEC_WMODE);
		if (ret < 0)
			break;
		//if (ret & NEED_RECONF_FILTER)
		//	i2s_set_filter(CODEC_WMODE,cur_codec->replay_codec_channel);
		ret = 0;
		break;

	case SND_DSP_SET_RECORD_CHANNELS:
		/* set record channels */
		ret = i2s_set_channel((int*)arg,CODEC_RMODE);
		if (ret < 0)
			break;
		if (ret & NEED_RECONF_FILTER)
			i2s_set_filter(CODEC_RMODE,cur_codec->record_codec_channel);
		ret = 0;
		break;

	case SND_DSP_GET_REPLAY_CHANNELS:
		if (cur_codec && cur_codec->replay_codec_channel)
			*(unsigned long *)arg = cur_codec->replay_codec_channel;
		ret = 0;
		break;

	case SND_DSP_GET_RECORD_CHANNELS:
		if (cur_codec && cur_codec->record_codec_channel)
			*(unsigned long *)arg = cur_codec->record_codec_channel;
		ret = 0;
		break;

	case SND_DSP_GET_REPLAY_FMT_CAP:
		/* return the support replay formats */
		ret = i2s_get_fmt_cap((unsigned long *)arg,CODEC_WMODE);
		break;

	case SND_DSP_GET_REPLAY_FMT:
		/* get current replay format */
		i2s_get_fmt((unsigned long *)arg,CODEC_WMODE);
		break;

	case SND_DSP_SET_REPLAY_FMT:
		/* set replay format */
		ret = i2s_set_fmt((unsigned long *)arg,CODEC_WMODE);
		if (ret < 0)
			break;
		/* if need reconfig the trigger, reconfig it */
		if (ret & NEED_RECONF_TRIGGER)
			i2s_set_trigger(CODEC_WMODE);
		/* if need reconfig the dma_slave.max_tsz, reconfig it and
		   set the dp->need_reconfig_dma as true */
		if (ret & NEED_RECONF_DMA)
			i2s_dma_need_reconfig(CODEC_WMODE);
		/* if need reconfig the filter, reconfig it */
		//if (ret & NEED_RECONF_FILTER)
		//	i2s_set_filter(CODEC_RMODE,cur_codec->replay_codec_channel);
		ret = 0;
		break;

	case SND_DSP_GET_RECORD_FMT_CAP:
		/* return the support record formats */
		ret = i2s_get_fmt_cap((unsigned long *)arg,CODEC_RMODE);
		break;

	case SND_DSP_GET_RECORD_FMT:
		/* get current record format */
		i2s_get_fmt((unsigned long *)arg,CODEC_RMODE);

		break;

	case SND_DSP_SET_RECORD_FMT:
		/* set record format */
		ret = i2s_set_fmt((unsigned long *)arg,CODEC_RMODE);
		if (ret < 0)
			break;
		/* if need reconfig the trigger, reconfig it */
		if (ret & NEED_RECONF_TRIGGER)
			i2s_set_trigger(CODEC_RMODE);
		/* if need reconfig the dma_slave.max_tsz, reconfig it and
		   set the dp->need_reconfig_dma as true */
		if (ret & NEED_RECONF_DMA)
			i2s_dma_need_reconfig(CODEC_RMODE);
		/* if need reconfig the filter, reconfig it */
		if (ret & NEED_RECONF_FILTER)
			i2s_set_filter(CODEC_RMODE,cur_codec->record_codec_channel);
		ret = 0;
		break;

	case SND_MIXER_DUMP_REG:
		dump_i2s_reg();
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_DUMP_REG,0);
		break;
	case SND_MIXER_DUMP_GPIO:
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_DUMP_GPIO,0);
		break;

	case SND_DSP_SET_STANDBY:
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_SET_STANDBY,(int)arg);
		break;

	case SND_DSP_SET_DEVICE:
		ret = i2s_set_device(arg);
		break;
	case SND_DSP_GET_HP_DETECT:
		*(int*)arg = jz_get_hp_switch_state();
		ret = 0;
		break;
	case SND_DSP_SET_RECORD_VOL:
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_SET_RECORD_VOLUME, arg);
		break;
	case SND_DSP_SET_REPLAY_VOL:
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_SET_REPLAY_VOLUME, arg);
		break;
	case SND_DSP_SET_MIC_VOL:
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_SET_MIC_VOLUME, arg);
		break;
	case SND_DSP_CLR_ROUTE:
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_CLR_ROUTE,arg);
		break;
	case SND_DSP_DEBUG:
		if (cur_codec)
			ret = cur_codec->codec_ctl(CODEC_DEBUG,arg);
		break;
	default:
		printk("SOUND_ERROR: %s(line:%d) unknown command!",
				__func__, __LINE__);
		ret = -EINVAL;
	}

	return ret;
}

/*##################################################################*\
|* functions
\*##################################################################*/

#ifdef CONFIG_JZ4780_INTERNAL_CODEC
static void i2s_codec_work_handler(struct work_struct *work)
{
	wait_event_interruptible(switch_data.wq,switch_data.hp_work.entry.next != NULL);
	cur_codec->codec_ctl(CODEC_IRQ_HANDLE,(unsigned long)(&(switch_data.hp_work)));
}
#endif

static irqreturn_t i2s_irq_handler(int irq, void *dev_id)
{
	unsigned long flags;
	irqreturn_t ret = IRQ_HANDLED;
	spin_lock_irqsave(&i2s_irq_lock,flags);
	/* check the irq source */
	/* if irq source is codec, call codec irq handler */
#ifdef CONFIG_JZ4780_INTERNAL_CODEC
	if (read_inter_codec_irq()){
		codec_irq_set_mask();
		if(!work_pending(&i2s_codec_work))
			queue_work(i2s_work_queue, &i2s_codec_work);
	}
#endif
	/* if irq source is aic, process it here */
	/*noting to do*/

	spin_unlock_irqrestore(&i2s_irq_lock,flags);

	return ret;
}

static int i2s_init_pipe(struct dsp_pipe **dp , enum dma_data_direction direction,unsigned long iobase)
{
	if (*dp != NULL || dp == NULL)
		return 0;
	*dp = vmalloc(sizeof(struct dsp_pipe));
	if (*dp == NULL) {
		return -ENOMEM;
	}

	(*dp)->dma_config.direction = direction;
	(*dp)->dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	(*dp)->dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	(*dp)->dma_type = JZDMA_REQ_I2S0;

	(*dp)->fragsize = FRAGSIZE_M;
	(*dp)->fragcnt = FRAGCNT_L;
	(*dp)->channels = 2;
	(*dp)->is_non_block = true;
	(*dp)->is_used = false;
	(*dp)->can_mmap =true;
	INIT_LIST_HEAD(&((*dp)->free_node_list));
	INIT_LIST_HEAD(&((*dp)->use_node_list));

	if (direction == DMA_TO_DEVICE) {
		(*dp)->dma_config.src_maxburst = 64;
		(*dp)->dma_config.dst_maxburst = 64;
		(*dp)->dma_config.dst_addr = iobase + AICDR;
		(*dp)->dma_config.src_addr = 0;
	} else if (direction == DMA_FROM_DEVICE) {
		(*dp)->dma_config.src_maxburst = 32;
		(*dp)->dma_config.dst_maxburst = 32;
		(*dp)->dma_config.src_addr = iobase + AICDR;
		(*dp)->dma_config.dst_addr = 0;
	} else
		return -1;

	return 0;
}

static int i2s_global_init(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *i2s_resource = NULL;
	struct dsp_pipe *i2s_pipe_out = NULL;
	struct dsp_pipe *i2s_pipe_in = NULL;
	struct clk *i2s_clk = NULL;

	i2s_resource = platform_get_resource(pdev,IORESOURCE_MEM,0);
	if (i2s_resource == NULL) {
		return -1;
	}

	/* map io address */
	if (!request_mem_region(i2s_resource->start, resource_size(i2s_resource), pdev->name)) {
		return -EBUSY;
	}
	i2s_iomem = ioremap(i2s_resource->start, resource_size(i2s_resource));
	if (!i2s_iomem) {
		ret =  -ENOMEM;
		goto __err_ioremap;
	}

	ret = i2s_init_pipe(&i2s_pipe_out,DMA_TO_DEVICE,i2s_resource->start);
	if (ret < 0) {
		goto __err_init_pipeout;
	}
	ret = i2s_init_pipe(&i2s_pipe_in,DMA_FROM_DEVICE,i2s_resource->start);
	if (ret < 0) {
		goto __err_init_pipein;
	}

	i2s_endpoints.out_endpoint = i2s_pipe_out;
	i2s_endpoints.in_endpoint = i2s_pipe_in;

	/* request aic clk */
	i2s_clk = clk_get(&pdev->dev, "aic0");
	if (IS_ERR(i2s_clk)) {
		dev_err(&pdev->dev, "aic clk_get failed\n");
		goto __err_aic_clk;
	}
	clk_enable(i2s_clk);

	spin_lock_init(&i2s_irq_lock);

#if defined(CONFIG_JZ_INTERNAL_CODEC)
	i2s_match_codec("internal_codec");
#elif defined(CONFIG_JZ_EXTERNAL_CODEC)
	i2s_match_codec("i2s_external_codec");
#else
	dev_info("WARNING: unless one codec must be select for i2s.\n");
#endif
	if (cur_codec == NULL) {
		ret = 0;
		goto __err_match_codec;
	}

	/* request irq */
	i2s_resource = platform_get_resource(pdev,IORESOURCE_IRQ,0);
	if (i2s_resource == NULL) {
		ret = -1;
		goto __err_irq;
	}

	ret = request_irq(i2s_resource->start, i2s_irq_handler,
					  IRQF_DISABLED, "aic_irq", NULL);
	if (ret < 0) {
		goto __err_irq;
	}


	__i2s_disable();
	schedule_timeout(5);
	__i2s_disable();
	__i2s_stop_bitclk();
	__i2s_stop_ibitclk();
	/*select i2s trans*/
	__aic_select_i2s();
	__i2s_select_i2s();


#if defined(CONFIG_JZ_EXTERNAL_CODEC)
	__i2s_external_codec();
#endif

	if (cur_codec->codec_mode == CODEC_MASTER) {
#if defined(CONFIG_JZ_INTERNAL_CODEC)
		__i2s_internal_codec_master();
#endif
		__i2s_slave_clkset();
		/*sysclk output*/
		__i2s_enable_sysclk_output();
	} else if (cur_codec->codec_mode == CODEC_SLAVE) {
#if defined(CONFIG_JZ_INTERNAL_CODEC)
		__i2s_internal_codec_slave();
#endif
		__i2s_master_clkset();
		__i2s_disable_sysclk_output();
	}

	/*set sysclk output for codec*/
	codec_sysclk = clk_get(&pdev->dev,"cgu_aic");
	if (IS_ERR(codec_sysclk)) {
		dev_dbg(&pdev->dev, "cgu_aic clk_get failed\n");
		goto __err_codec_clk;
	}
	/*set sysclk output for codec*/
	clk_set_rate(codec_sysclk,cur_codec->codec_clk);
	if (clk_get_rate(codec_sysclk) > cur_codec->codec_clk) {
		printk("codec interface set rate fail.\n");
		goto __err_codec_clk;
	}
	clk_enable(codec_sysclk);


	__i2s_start_bitclk();
	__i2s_start_ibitclk();

	__i2s_disable_receive_dma();
	__i2s_disable_transmit_dma();
	__i2s_disable_record();
	__i2s_disable_replay();
	__i2s_disable_loopback();
	__i2s_clear_ror();
	__i2s_clear_tur();
	__i2s_set_receive_trigger(3);
	__i2s_set_transmit_trigger(4);
	__i2s_disable_overrun_intr();
	__i2s_disable_underrun_intr();
	__i2s_disable_transmit_intr();
	__i2s_disable_receive_intr();
	__i2s_send_rfirst();
	/* play zero or last sample when underflow */
	__i2s_play_lastsample();

	__i2s_enable();
	printk("i2s init success.\n");
	return  cur_codec->codec_ctl(CODEC_INIT,0);

__err_codec_clk:
	clk_put(codec_sysclk);
__err_irq:
	clk_disable(i2s_clk);
	clk_put(i2s_clk);
__err_aic_clk:
__err_match_codec:
	vfree(i2s_pipe_in);
__err_init_pipein:
	vfree(i2s_pipe_out);
__err_init_pipeout:
	iounmap(i2s_iomem);
__err_ioremap:
	release_mem_region(i2s_resource->start,resource_size(i2s_resource));
	return ret;
}

static int i2s_init(struct platform_device *pdev)
{
	int ret = -EINVAL;

	ret = i2s_global_init(pdev);

	return ret;
}

static void i2s_shutdown(struct platform_device *pdev)
{
	/* close i2s and current codec */

	free_irq(IRQ_AIC0,NULL);
	if (cur_codec) {
		cur_codec->codec_ctl(CODEC_TURN_OFF,CODEC_RWMODE);
		cur_codec->codec_ctl(CODEC_SHUTDOWN,0);
	}
	__i2s_disable();
	return;
}

static int i2s_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (cur_codec && !i2s_is_incall())
		cur_codec->codec_ctl(CODEC_SUSPEND,0);
	__i2s_disable();
	return 0;
}

static int i2s_resume(struct platform_device *pdev)
{
	__i2s_enable();
	if (cur_codec && !i2s_is_incall())
		cur_codec->codec_ctl(CODEC_RESUME,0);
	return 0;
}

/*
 * headphone detect switch function
 *
 */
static int jz_get_hp_switch_state(void)
{
	int value = 0;
	int ret = 0;
    if (cur_codec && cur_codec->codec_ctl) {
        ret = cur_codec->codec_ctl(CODEC_GET_HP_STATE, (unsigned long)&value);
        if (ret < 0) {
            return 0;
        }
    }
	return value;
}

void *jz_set_hp_detect_type(int type,struct snd_board_gpio *hp_det,
		struct snd_board_gpio *mic_det,
		struct snd_board_gpio *mic_detect_en,
		struct snd_board_gpio *mic_select,
		int hook_active_level)
{
	switch_data.type = type;
	switch_data.hook_valid_level = hook_active_level;
	if (type == SND_SWITCH_TYPE_GPIO && hp_det != NULL) {
		switch_data.hp_gpio = hp_det->gpio;
		switch_data.hp_valid_level = hp_det->active_level;
	} else {
		switch_data.hp_gpio = -1;
	}
	if (mic_det != NULL) {
		switch_data.mic_gpio = mic_det->gpio;
		switch_data.mic_vaild_level = mic_det->active_level;
	} else {
		switch_data.mic_gpio = -1;
	}

	if (mic_detect_en != NULL) {
		switch_data.mic_detect_en_gpio = mic_detect_en->gpio;
		switch_data.mic_detect_en_level = mic_detect_en->active_level;
	} else {
		switch_data.mic_detect_en_gpio = -1;
	}

	if (mic_detect_en != NULL) {
		switch_data.mic_select_gpio = mic_select->gpio;
		switch_data.mic_select_level = mic_select->active_level;
	} else {
		switch_data.mic_select_gpio = -1;
	}

	return (&switch_data.hp_work);
}

struct snd_switch_data switch_data = {
	.sdev = {
		.name = "h2w",
	},
	.state_headset_on	=	"1",
	.state_headphone_on =   "2",
	.state_off	=	"0",
	.codec_get_sate	=	jz_get_hp_switch_state,
	.type	=	SND_SWITCH_TYPE_CODEC,
};

static struct platform_device xb47xx_i2s_switch = {
	.name	= DEV_DSP_HP_DET_NAME,
	.id		= SND_DEV_DETECT0_ID,
	.dev	= {
		.platform_data	= &switch_data,
	},
};

struct snd_dev_data i2s_data = {
	.dev_ioctl	   	= i2s_ioctl,
	.ext_data		= &i2s_endpoints,
	.minor			= SND_DEV_DSP0,
	.init			= i2s_init,
	.shutdown		= i2s_shutdown,
	.suspend		= i2s_suspend,
	.resume			= i2s_resume,
};

struct snd_dev_data snd_mixer0_data = {
	.dev_ioctl	   	= i2s_ioctl,
	.minor			= SND_DEV_MIXER0,
};

static int __init init_i2s(void)
{
	init_waitqueue_head(&switch_data.wq);

#ifdef CONFIG_JZ4780_INTERNAL_CODEC
	INIT_WORK(&i2s_codec_work, i2s_codec_work_handler);
	i2s_work_queue = create_singlethread_workqueue("i2s_codec_irq_wq");

	if (!i2s_work_queue) {
		// this can not happen, if happen, we die!
		BUG();
	}
#endif
	return platform_device_register(&xb47xx_i2s_switch);
}
module_init(init_i2s);
