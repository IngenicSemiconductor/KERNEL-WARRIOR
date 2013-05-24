/**
 * xb_snd_mixer.c
 *
 * jbbi <jbbi@ingenic.cn>
 *
 * 24 APR 2012
 *
 */

#include "xb_snd_mixer.h"
#include "xb_snd_dsp.h"
#include <linux/soundcard.h>
#include <linux/gpio.h>
/*###########################################################*\
 * interfacees
\*###########################################################*/
/********************************************************\
 * llseek
\********************************************************/
loff_t xb_snd_mixer_llseek(struct file *file,
						   loff_t offset,
						   int origin,
						   struct snd_dev_data *ddata)
{
	return 0;
}

/********************************************************\
 * read
\********************************************************/
ssize_t xb_snd_mixer_read(struct file *file,
						  char __user *buffer,
						  size_t count,
						  loff_t *ppos,
						  struct snd_dev_data *ddata)
{
	return -1;
}

/********************************************************\
 * write
\********************************************************/
static void print_format(unsigned long fmt)
{
	switch(fmt) {
		case AFMT_U8:
			printk("AFMT_U8.\n");
			break;
		case AFMT_S8:
			printk("AFMT_S8.\n");
			break;
		case AFMT_S16_LE:
			printk("AFMT_S16_LE.\n");
			break;
		case AFMT_S16_BE:
			printk("AFMT_S16_BE.\n");
			break;
		default :
			printk("unknown format.\n");
	}
}

ssize_t xb_snd_mixer_write(struct file *file,
		const char __user *buffer,
		size_t count,
		loff_t *ppos,
		struct snd_dev_data *ddata)
{
	char buf_byte;
	char buf_vol[3];
	int volume = 0;
	unsigned long fmt_in = 0;
	unsigned long fmt_out = 0;
	unsigned int channels_in = 0;
	unsigned int  channels_out = 0;
	unsigned long rate_in = 0;
	unsigned long rate_out = 0;
	unsigned hp_state = 0;
	unsigned long devices = 0;
	//int mode = 3;
	if (copy_from_user((void *)&buf_byte, buffer, 1)) {
		printk("JZ MIX: copy_from_user failed !\n");
		return -EFAULT;
	}

	switch (buf_byte) {
		case '1' :
			printk(" \"1\" command :print codec and aic register.\n");
			ddata->dev_ioctl(SND_MIXER_DUMP_REG,0);
			break;
		case '2':
			printk(" \"2\" command :print audio hp and speaker gpio state.\n");
			ddata->dev_ioctl(SND_MIXER_DUMP_GPIO,0);
			break;
		case '3':
			printk(" \"3\" command :print current format channels and rate.\n");
			ddata->dev_ioctl(SND_DSP_GET_RECORD_FMT, (unsigned long)&fmt_in);
			ddata->dev_ioctl(SND_DSP_GET_REPLAY_FMT, (unsigned long)&fmt_out);
			printk("record format : ");
			print_format(fmt_in);
			printk("replay format : ");
			print_format(fmt_out);
			ddata->dev_ioctl(SND_DSP_GET_RECORD_CHANNELS,(unsigned long)&channels_in);
			ddata->dev_ioctl(SND_DSP_GET_REPLAY_CHANNELS,(unsigned long)&channels_out);
			printk("record channels : %d.\n", channels_in);
			printk("replay channels : %d.\n", channels_out);
			ddata->dev_ioctl(SND_DSP_GET_RECORD_RATE,(unsigned long)&rate_in);
			ddata->dev_ioctl(SND_DSP_GET_REPLAY_RATE,(unsigned long)&rate_out);
			printk("record samplerate : %ld.\n", rate_in);
			printk("replay samplerate : %ld.\n", rate_out);
			break;
		case '4':
			printk(" \"4\" command:print headphone detect state.\n");
			ddata->dev_ioctl(SND_DSP_GET_HP_DETECT,(unsigned long)&hp_state);
			printk("headphone state : %d.\n ",hp_state);
			break;
		case '5':
			printk(" \"5\" set headphone route.\n");
			devices = SND_DEVICE_HEADSET;
			ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);
			break;
		case '6':
			printk(" \"6\" set speaker route.\n");
			devices = SND_DEVICE_SPEAKER;
			ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);
			break;
		case '7':
			printk(" \"7\" set loop test route for phone.\n");
			devices = SND_DEVICE_LOOP_TEST;
			ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);
			break;
		case 'c':
			/*printk(" \"c\" clear rount 1 CODEC_RMODE ,2 CODEC_WMODE ,3 CODEC_RWMODE\n");
			if(copy_from_user((void *)buf_vol, buffer, 2)) {
				return -EFAULT;
			}
			mode = (buf_vol[1] - '0');
			printk("audio clear %s route.\n",mode == CODEC_RMODE ? "read" :
					mode == CODEC_WMODE ? "write" : "all");
			mode = 3;
			ddata->dev_ioctl(SND_DSP_SET_MIC_VOL,(unsigned long)mode);*/
			break;
		case 'm':
			printk(" \"m\" set mic volume 0 ~ 20db\n");
			if(copy_from_user((void *)buf_vol, buffer, 3)) {
				return -EFAULT;
			}
			volume = (buf_vol[1] - '0') * 10 + (buf_vol[2] - '0');
			printk("mic volume is %d.\n",volume);
			ddata->dev_ioctl(SND_DSP_SET_MIC_VOL,(unsigned long)&volume);
			printk("mic volume is %d.\n",volume);
			break;
		case 'v':
			printk(" \"v\" set record adc  volume 0 ~ 23db\n");
			if(copy_from_user((void *)buf_vol, buffer, 3)) {
				return -EFAULT;
			}
			volume = (buf_vol[1] - '0') * 10 + (buf_vol[2] - '0');
			ddata->dev_ioctl(SND_DSP_SET_RECORD_VOL,(unsigned long)&volume);
			printk("record volume is %d.\n",volume);
			break;
		case 'd':
			printk("\"d\" misc debug\n");
			if(copy_from_user((void *)buf_vol, buffer, 2)) {
				printk("audio misc debug default opreation\n");
				ddata->dev_ioctl(SND_DSP_DEBUG,0);
			} else {
				printk("audio misc debug interface NO %c\n",buf_vol[1]);
				ddata->dev_ioctl(SND_DSP_DEBUG,(unsigned long)buf_vol[1]);
			}
			break;
		default:
			printk("undefine debug interface \"%c\".\n", buf_byte);
			printk(" \"1\" command :print codec and aic register.\n");
			printk(" \"2\" command :print audio hp and speaker gpio state.\n");
			printk(" \"3\" command :print current format channels and rate.\n");
			printk(" \"4\" command:print headphone detect state.\n");
			printk(" \"5\" set headphone route.\n");
			printk(" \"6\" set speaker route.\n");
			printk(" \"7\" set loop test route for phone.\n");
			printk(" \"c\" clear rount 1 CODEC_RMODE ,2 CODEC_WMODE ,3 CODEC_RWMODE\n");
			printk(" \"m[db]\" set mic volume (db = 0 ~ 20)\n");
			printk(" \"v[db]\" set record adc volume (db = 0 ~ 23)\n");
			printk(" \"d[debug interface No]\" misc debug\n");
	}
	return count;
}

/********************************************************\
 * poll
\********************************************************/
unsigned int xb_snd_mixer_poll(struct file *file,
							   poll_table *wait,
							   struct snd_dev_data *ddata)
{
	return 0;
}

/********************************************************\
 * ioctl
\********************************************************/
long xb_snd_mixer_ioctl(struct file *file,
						unsigned int cmd,
						unsigned long arg,
						struct snd_dev_data *ddata)
{
	unsigned long devices = 0;
	int ret = 0;

	switch (cmd) {

	case SOUND_MIXER_WRITE_VOLUME:{

		int vol = 0;
		if (get_user(vol, (int*)arg)){
			return -EFAULT;
		}
		vol = vol & 0xff;
		if (ddata->dev_ioctl) {
			ret = (int)ddata->dev_ioctl(SND_DSP_SET_REPLAY_VOL, (unsigned long)&vol);
			if (ret < 0)
			   break;
		}
		ret = put_user(vol, (int *)arg);
		break;
	}
	case SOUND_MIXER_READ_VOLUME: {

		int vol = -1;
		if (ddata->dev_ioctl) {
			ret = (int)ddata->dev_ioctl(SND_DSP_SET_REPLAY_VOL, (unsigned long)&vol);
		}
		vol = vol << 8 | vol;
		ret = put_user(vol, (int *)arg);
		break;
	}

	case SOUND_MIXER_READ_DEVMASK: {
		int devmsk = (1 << SOUND_MIXER_VOLUME);
		ret = put_user(devmsk, (int *)arg);
		break;
	}

	case SNDCTL_MIXER_LOOP_TEST_ON:{
		printk("set loop test route for phone.\n");
		devices = SND_DEVICE_LOOP_TEST;
		ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);

		break;
	}
	case SNDCTL_MIXER_LOOP_TEST_OFF:{
		printk("close loop test route for phone.\n");
		devices = SND_DEVICE_SPEAKER;
		ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);

		break;
	}
	case SOUND_MIXER_WRITE_LINE: {
		devices = SND_DEVICE_HEADSET_MIC,
		ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);
		devices = SND_DEVICE_HEADSET,
		ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);
		break;
	}
	case SOUND_MIXER_WRITE_MIC:	 {
		int gain = 0;
		if (get_user(gain, (int*)arg)){
			return -EFAULT;
		}
		devices = SND_DEVICE_BUILDIN_MIC,
		ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);
		devices = SND_DEVICE_SPEAKER,
		ddata->dev_ioctl(SND_DSP_SET_DEVICE,(unsigned long)&devices);
		ddata->dev_ioctl(SND_DSP_SET_MIC_VOL,(unsigned long)&gain);
		break;
	}

		//case SNDCTL_MIX_DESCRIPTION:
		/* OSS 4.x: get description text for a mixer control */
		//break;
		//case SNDCTL_MIX_ENUMINFO:
		/* OSS 4.x: get choice list for a MIXT_ENUM control */
		//break;
		//case SNDCTL_MIX_EXTINFO:
		/* OSS 4.x: get a mixer extension descriptor */
		//break;
		//case SNDCTL_MIX_NREXT:
		/* OSS 4.x: get number of mixer extension descriptor records */
		//break;
		//case SNDCTL_MIX_NRMIX:
		/* OSS 4.x: get number of mixer devices in the system */
		//break;
		//case SNDCTL_MIX_READ:
		/* OSS 4.x: read the current value of a mixer control */
		//break;
		//case SNDCTL_MIX_WRITE:
		/* OSS 4.x: change value of a mixer control */
		//break;
	}
	return 0;
}

/********************************************************\
 * mmap
\********************************************************/
int xb_snd_mixer_mmap(struct file *file,
		struct vm_area_struct *vma,
		struct snd_dev_data *ddata)
{
	return -1;
}

/********************************************************\
 * open
\********************************************************/
int xb_snd_mixer_open(struct inode *inode,
		struct file *file,
		struct snd_dev_data *ddata)
{
	return 0;
}

/********************************************************\
 * release
\********************************************************/
int xb_snd_mixer_release(struct inode *inode,
		struct file *file,
		struct snd_dev_data *ddata)
{
	return 0;
}

/********************************************************\
 * probe
\********************************************************/
int xb_snd_mixer_probe(struct snd_dev_data *ddata)
{
	return 0;
}
