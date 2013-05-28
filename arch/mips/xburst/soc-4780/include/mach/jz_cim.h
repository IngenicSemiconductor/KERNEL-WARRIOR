#ifndef __JZ_CIM_H__
#define __JZ_CIM_H__

#define CIMIO_SHUTDOWN				0x01		// stop preview
#define CIMIO_START_PREVIEW			0x02
#define CIMIO_START_CAPTURE			0x03
#define CIMIO_GET_FRAME				0x04
#define CIMIO_GET_SENSORINFO		0x05
#define CIMIO_GET_VAR				0x06
#define CIMIO_GET_CAPTURE_PARAM			0x07
#define CIMIO_GET_PREVIEW_PARAM			0x08		// get preview size and format
#define CIMIO_GET_SUPPORT_PSIZE			0x09
#define CIMIO_SET_PARAM				0x0a
#define CIMIO_SET_PREVIEW_MEM			0x0b		// alloc mem for buffers by app
#define CIMIO_SET_CAPTURE_MEM			0x0c		// alloc mem for buffers by app
#define CIMIO_SELECT_SENSOR			0x0d
#define CIMIO_DO_FOCUS				0x0e
#define CIMIO_AF_INIT				0x0f
#define CIMIO_SET_PREVIEW_SIZE			0x10
#define CIMIO_SET_CAPTURE_SIZE			0x11
#define CIMIO_GET_SUPPORT_CSIZE			0x12
#define CIMIO_SET_VIDEO_MODE			0x13
#define CIMIO_STOP_PREVIEW			0x14
#define CIMIO_SET_TLB_BASE			0x15
#define CIMIO_GET_SENSOR_COUNT			0x16
#define CIMIO_SET_PREVIEW_FMT		0x17
#define CIMIO_SET_CAPTURE_FMT		0x18

//cim output format, set according system/core/include/system/graphics.h
#define HAL_PIXEL_FORMAT_YV12				0x32315659
#define HAL_PIXEL_FORMAT_JZ_YUV_420_P		0x47700001
#define HAL_PIXEL_FORMAT_JZ_YUV_420_B		0x47700002
#define HAL_PIXEL_FORMAT_YCbCr_422_I        0x14

#define CIM_CSC_YUV420B				0x1
#define CIM_CSC_YUV420P				0x2
#define CIM_BYPASS_YUV422I			0X3

//cim config for sensor YUYV output order
#define SENSOR_OUTPUT_FORMAT_Y0UY1V 		0		//11 22 33 44
#define SENSOR_OUTPUT_FORMAT_UY1VY0 		1		//22 33 44 11
#define SENSOR_OUTPUT_FORMAT_Y1VY0U 		2		//33 44 11 22
#define SENSOR_OUTPUT_FORMAT_VY0UY1 		3		//44 11 22 33
#define SENSOR_OUTPUT_FORMAT_VY1UY0 		4		//44 33 22 11
#define SENSOR_OUTPUT_FORMAT_Y1UY0V 		5		//33 22 11 44
#define SENSOR_OUTPUT_FORMAT_UY0VY1 		6		//22 11 44 33
#define SENSOR_OUTPUT_FORMAT_Y0VY1U 		7		//11 44 33 22

//camera param cmd
#define CPCMD_SET_BALANCE			(0x1 << (16 + 3))
#define CPCMD_SET_EFFECT			(0x1 << (16 + 4))
#define CPCMD_SET_ANTIBANDING			(0x1 << (16 + 5))
#define CPCMD_SET_FLASH_MODE			(0x1 << (16 + 6))
#define CPCMD_SET_SCENE_MODE			(0x1 << (16 + 7))
#define CPCMD_SET_FOCUS_MODE			(0x1 << (16 + 9))
#define CPCMD_SET_FPS				(0x1 << (16 + 10))
#define CPCMD_SET_NIGHTSHOT_MODE		(0x1 << (16 + 11))
#define CPCMD_SET_LUMA_ADAPTATION       	(0x1 << (16 + 12))
#define CPCMD_SET_BRIGHTNESS			(0x1 << (16 + 13))	//add for VT app 
#define CPCMD_SET_CONTRAST			(0x1 << (16 + 14))	//add for VT app 

// Values for white balance settings.
#define WHITE_BALANCE_AUTO			0x1 << 0
#define WHITE_BALANCE_INCANDESCENT 		0x1 << 1
#define WHITE_BALANCE_FLUORESCENT 		0x1 << 2
#define WHITE_BALANCE_WARM_FLUORESCENT 		0x1 << 3
#define WHITE_BALANCE_DAYLIGHT 			0x1 << 4
#define WHITE_BALANCE_CLOUDY_DAYLIGHT 		0x1 << 5
#define WHITE_BALANCE_TWILIGHT 			0x1 << 6
#define WHITE_BALANCE_SHADE 			0x1 << 7
#define WHITE_BALANCE_TUNGSTEN 			0x1 << 8
		
// Values for effect settings.
#define EFFECT_NONE				0x1 << 0
#define EFFECT_MONO 				0x1 << 1
#define EFFECT_NEGATIVE 			0x1 << 2
#define EFFECT_SOLARIZE 			0x1 << 3
#define EFFECT_SEPIA 				0x1 << 4
#define EFFECT_POSTERIZE 			0x1 << 5
#define EFFECT_WHITEBOARD 			0x1 << 6
#define EFFECT_BLACKBOARD			0x1 << 7	
#define EFFECT_AQUA 				0x1 << 8
#define EFFECT_PASTEL				0x1 << 9
#define EFFECT_MOSAIC				0x1 << 10
#define EFFECT_RESIZE				0x1 << 11

// Values for antibanding settings.
#define ANTIBANDING_AUTO 			0x1 << 0
#define ANTIBANDING_50HZ 			0x1 << 1
#define ANTIBANDING_60HZ 			0x1 << 2
#define ANTIBANDING_OFF 			0x1 << 3

// Values for flash mode settings.
#define FLASH_MODE_OFF				0x1 << 0
#define FLASH_MODE_AUTO 			0x1 << 1
#define FLASH_MODE_ON 				0x1 << 2
#define FLASH_MODE_RED_EYE 			0x1 << 3
#define FLASH_MODE_TORCH 		        0x1 << 4	

// Values for scene mode settings.
#define SCENE_MODE_AUTO 			0x1 << 0
#define SCENE_MODE_ACTION 			0x1 << 1
#define SCENE_MODE_PORTRAIT   			0x1 << 2
#define SCENE_MODE_LANDSCAPE  			0x1 << 3
#define SCENE_MODE_NIGHT     			0x1 << 4
#define SCENE_MODE_NIGHT_PORTRAIT   		0x1 << 5
#define SCENE_MODE_THEATRE  			0x1 << 6
#define SCENE_MODE_BEACH   			0x1 << 7
#define SCENE_MODE_SNOW    			0x1 << 8
#define SCENE_MODE_SUNSET    			0x1 << 9
#define SCENE_MODE_STEADYPHOTO   		0x1 << 10
#define SCENE_MODE_FIREWORKS    		0x1 << 11
#define SCENE_MODE_SPORTS    			0x1 << 12
#define SCENE_MODE_PARTY   			0x1 << 13
#define SCENE_MODE_CANDLELIGHT 			0x1 << 14

// Values for focus mode settings.
#define FOCUS_MODE_FIXED 			0x1 << 0
#define FOCUS_MODE_AUTO				0x1 << 1	 
#define FOCUS_MODE_INFINITY 			0x1 << 2
#define FOCUS_MODE_MACRO 			0x1 << 3


// Values for fps mode settings.
#define FPS_MODE_10 			0x1<<0
#define FPS_MODE_15				0x1<<1	 
#define FPS_MODE_20				0x1<<2
#define FPS_MODE_25 			0x1<<3
#define FPS_MODE_30				0x1<<4


struct frm_size {
	unsigned int w;
	unsigned int h;
};

struct mode_bit_map {
	unsigned short balance;
	unsigned short effect;
	unsigned short antibanding;
	unsigned short flash_mode;
	unsigned short scene_mode;
	unsigned short focus_mode;
	unsigned short fps;
};


struct sensor_info 
{
	unsigned int 		sensor_id;
	char 			name[32];
	int facing;
	int orientation;

    unsigned int 		prev_resolution_nr;
	unsigned int 		cap_resolution_nr;
	 
	struct mode_bit_map modes;
};


struct cim_sensor {
	int 	id;
	char  name[16];
	struct list_head	list;

        int facing;
        int orientation;
	int cim_cfg;
	struct mode_bit_map modes;	//indicate sensor can support modes
	struct mode_bit_map para;	//indicate currnet parameter value

	bool first_used;

	struct frm_size	*preview_size;
	struct frm_size	*capture_size;
	int prev_resolution_nr;
	int cap_resolution_nr;
	int cap_wait_frame;

	int (*probe)(struct cim_sensor *data);
	int (*init)(struct cim_sensor *data);
	int (*power_on)(struct cim_sensor *data);
	int (*shutdown)(struct cim_sensor *data);
	int (*reset)(struct cim_sensor *data);

	int (*af_init)(struct cim_sensor *data);
	int (*start_af)(struct cim_sensor *data);
	int (*stop_af)(struct cim_sensor *data);
	void (*read_all_regs)(struct cim_sensor *data);

	int (*set_preivew_mode)(struct cim_sensor *data);
	int (*set_capture_mode)(struct cim_sensor *data);
	int (*set_video_mode)(struct cim_sensor *data);

	int (*set_resolution)(struct cim_sensor *data,int width,int height);
	int (*set_balance)(struct cim_sensor *data,unsigned short arg);
	int (*set_effect)(struct cim_sensor *data,unsigned short arg);
	int (*set_antibanding)(struct cim_sensor *data,unsigned short arg);
	int (*set_flash_mode)(struct cim_sensor *data,unsigned short arg);
	int (*set_scene_mode)(struct cim_sensor *data,unsigned short arg);
	int (*set_focus_mode)(struct cim_sensor *data,unsigned short arg);
	int (*set_fps)(struct cim_sensor *data,unsigned short arg);
	int (*set_nightshot)(struct cim_sensor *data,unsigned short arg);
	int (*set_luma_adaption)(struct cim_sensor *data,unsigned short arg);
	int (*set_brightness)(struct cim_sensor *data,unsigned short arg);
	int (*set_contrast)(struct cim_sensor *data,unsigned short arg);

	int (*fill_buffer)(struct cim_sensor *data,char *buf);
	
	void *private;
};

struct jz_cim_platform_data {
	void (*power_on)(void);
	void (*power_off)(void);
};

typedef struct CameraYUVMeta {
	int32_t index;
	int32_t width;
	int32_t height;
	int32_t yPhy;
	int32_t uPhy;
	int32_t vPhy;
	int32_t yAddr;
	int32_t uAddr;
	int32_t vAddr;
	int32_t yStride;
	int32_t uStride;
	int32_t vStride;
	int32_t count;
	int32_t format;
}CameraYUVMeta;

#define CAMERA_FACING_FRONT  1
#define CAMERA_FACING_BACK  0

extern int camera_sensor_register(struct cim_sensor *desc);

#endif
