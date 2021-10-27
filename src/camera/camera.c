#include "camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include "util/logger.h"
#include "util/debug.h"

#include "mavlink/c_library_v2/common/mavlink.h"

#include <linux/videodev2.h>

#define SIZE_240P "428x240"
#define SIZE_360P "640x360"
#define SIZE_480P "720x480"
#define SIZE_720P "1280x720"
#define SIZE_1080P "1920x1080"

#define CMD_FFMPEG_CAPTURE "ffmpeg -i /dev/video0 -f h264 pipe: 2> /dev/null"
#define CMD_FFMPEG_STREAM "ffmpeg -i pipe: -f rtsp rtsp://localhost:8554/mystream 2> /dev/null"

pthread_t _camera_thread; // Camera thread.
pthread_mutex_t _camera_mutex; // Camera mutex.

struct v4l2_capability _capability; // V4L2 Capability.

FILE *capture; // popen ffmpeg capture process.
FILE *stream; // popen ffmpeg rtsp stream process.
FILE *video_file = NULL; // popen ffmpeg saving process of video.
FILE *picture_file = NULL; // popen ffmpeg saving process of picture.

bool _video_capturing; // True if recording video else false.
bool _image_capturing; // True if shooting else false.
float _image_capture_interval; // Interval between capturing.
int _n_images_to_take; // Number of image to take, set through MAVLink command.
int _status_interval; // msec interval to send MAVLink camera capture status.

void *camera_handler(void *arg);
void camera_atexit();

int camera_init() {
    pthread_mutex_init(&_camera_mutex, NULL);
    
    int fd = open("/dev/video0", O_RDWR);
    if (ioctl(fd, VIDIOC_QUERYCAP, &_capability) != 0) {
        LOG_ERROR("Failed to query capability.\n");
        close(fd);
        return -1;
    }
    close(fd);
    _video_capturing = false;
    _image_capturing = false;
    _image_capture_interval = 0;
    _n_images_to_take = false;
    _status_interval = 0;

    atexit(camera_atexit);

    return pthread_create(&_camera_thread, NULL, camera_handler, NULL);
}

int camera_set_video_capturing(bool capture) {
    if (_video_capturing == capture) {
        return -1;
    }
    pthread_mutex_lock(&_camera_mutex);
    _video_capturing = capture;
    if (_video_capturing) {
        // Open recording process.
    } else {
        // Close recording process.
    }

    pthread_mutex_unlock(&_camera_mutex);
}

int camera_set_image_capturing(bool capture) {
    if (_image_capturing == capture) {
        return -1;
    }
    pthread_mutex_lock(&_camera_mutex);
    _image_capturing = capture;
    if (_image_capturing) {
        // Open taking process.
    } else {
        // Close taking process.
    }

    pthread_mutex_unlock(&_camera_mutex);
}

/**
 * @brief Check if camera is capturing video.
 * 
 * @return True if capturing video else false.
 */
bool camera_is_video_capturing() {
    return _video_capturing;
}

/**
 * @brief Check if camera is capturing image.
 * 
 * @return True if capturing image else false.
 */
bool camera_is_image_capturing() {
    return _image_capturing;
}

/**
 * @brief Set interval between capturing images.
 * 
 * @param sec 
 *      Second.
 */
void camera_set_image_capture_interval(float sec) {
    _image_capture_interval = sec;
}

/**
 * @brief Get interval between capturing images.
 * 
 * @return Second.
 */
float camera_get_image_capture_interval() {
    return _image_capture_interval;
}

/**
 * @brief Set number of images to take, 0 if take forever until CMD_IMAGE_STOP_CAPTURING.
 * 
 * @param n 
 *      Number of images to take.
 */
void camera_set_image_capture_number(int n) {
    if (n == 0) {
        _n_images_to_take = -1;
    } else {
        _n_images_to_take = n;
    }
}

/**
 * @brief Get the number of images to take.
 * 
 * @return Number of images to take.
 */
int camera_get_image_capture_number() {
    return _n_images_to_take;
}

/**
 * @brief Set the interval between camera_capture_status.
 * 
 * @param msec 
 *      millisecond.
 */
void camera_set_status_interval_msec(int msec) {
    _status_interval = msec;
}

//----- Information

/**
 * @brief Get the interval between camera_capture_status.
 * 
 * @return millisecond.
 */
int camera_get_status_interval_msec() {
    return _status_interval;
}

/**
 * @brief Get the camera vendor name.
 * 
 * @return Vendor name.
 */
const char *camera_get_vendor_name() {
    return _capability.bus_info;
}

/**
 * @brief Get the camera model name.
 * 
 * @return Model name.
 */
const char *camera_get_model_name() {
    return _capability.card;
}

/**
 * @brief Get the link of RTSP stream.
 * 
 * @return The link of RTSP stream.
 */
const char *camera_get_url() {
    return "rtsp://192.168.0.12:8554/mystream";
}

/**
 * @brief Get the version of camera firmware.
 * 
 * @return Version of camera firmware.
 */
uint32_t camera_get_version() {
    return _capability.version;
}

/**
 * @brief Get the MAVLink capability of camera.
 * 
 * @return Capability.
 */
uint32_t camera_get_capability() {
    return CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
        CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
        (_capability.capabilities & V4L2_CAP_VIDEO_CAPTURE ? CAMERA_CAP_FLAGS_CAPTURE_IMAGE : 0) |
        CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
        CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM;
}

/**
 * @brief Camera capture/stream thread.
 * 
 * @param arg 
 * @return NULL
 */
void *camera_handler(void *arg) {
    pthread_detach(pthread_self());
    capture = popen(CMD_FFMPEG_CAPTURE, "r");
    stream = popen(CMD_FFMPEG_STREAM, "w");

    char buf[1024];
    int cnt;
    while (1) {
        pthread_mutex_lock(&_camera_mutex);
        
        cnt = fread(buf, 1, 1024, capture);

        // Publish to stream.
        fwrite(buf, 1, cnt, stream);
        
        pthread_mutex_unlock(&_camera_mutex);
        
    }

    pthread_exit(NULL);
}

/**
 * @brief Camera at exit.
 * 
 */
void camera_atexit() {
    LOG("Closing camera.\n");
    pclose(capture);
    pclose(stream);

    if (video_file != NULL) {
        pclose(video_file);
    }
    if (picture_file != NULL) {
        pclose(picture_file);
    }
}