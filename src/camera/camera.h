#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stdint.h>
#include <stdbool.h>

int camera_init();

int camera_set_video_capturing(bool capture);

int camera_set_image_capturing(bool capture);

bool camera_is_video_capturing();

bool camera_is_image_capturing();

void camera_set_image_capture_interval(float sec);

float camera_get_image_capture_interval();

void camera_set_image_capture_number(int n);

int camera_get_image_capture_number();

void camera_set_status_interval_msec(int msec);

int camera_get_status_interval_msec();

//----- Information

const char *camera_get_vendor_name();

const char *camera_get_model_name();

const char *camera_get_url();

uint32_t camera_get_version();

uint32_t camera_get_capability();

#endif // _CAMERA_H_
