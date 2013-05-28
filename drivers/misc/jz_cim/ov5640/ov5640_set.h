#ifndef __OV5640_SET_H__
#define __OV5640_SET_H__


int ov5640_preview_set(struct cim_sensor *sensor_info);
int ov5640_capture_set(struct cim_sensor *sensor_info);
int ov5640_size_switch(struct cim_sensor *sensor_info,int width,int height);

int ov5640_init(struct cim_sensor *sensor_info);
int ov5640_set_effect(struct cim_sensor *sensor_info,unsigned short arg);
int ov5640_set_antibanding(struct cim_sensor *sensor_info,unsigned short arg);
int ov5640_set_balance(struct cim_sensor *sensor_info,unsigned short arg);
void ov5640_set_focus(struct cim_sensor *sensor_info, unsigned short arg);

extern struct workqueue_struct *ov5640_work_queue;
extern void ov5640_late_work(struct work_struct *work);

#endif

