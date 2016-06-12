/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 *
 *
 * Modified by Felipe Bombardelli <felipebombardelli@gmail.com>
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */



/*=====================================- HEADER -======================================*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <libfreenect.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sys/stat.h>
#include <sys/file.h>

#include <pthread.h>
#include <math.h>
#include <signal.h>



typedef struct {
	freenect_context *ctx;
	freenect_device  *dev;

	uint8_t *depth_mid, *depth_front;
	uint8_t *rgb_back, *rgb_mid, *rgb_front;

	uint16_t t_gamma[2048];
	pthread_mutex_t mutex;
} Kinect;

Kinect G_kt;




/*-------------------------------------------------------------------------------------*/



/*====================================- CALLBACK -=====================================*/

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp){
	int i;
	uint16_t *depth = (uint16_t*)v_depth;

	pthread_mutex_lock(&G_kt.mutex);

	FILE* fd = fopen("depth","a+");
	assert( fd );
	flock( fileno(fd), LOCK_EX );
	ftruncate( fileno(fd), 0 );
	fprintf(fd, "class=Mat:Cv\n");
	fprintf(fd, "cols=640\n");
	fprintf(fd, "rows=480\n");
	fprintf(fd, "step=%d\n", 640*2);
	fprintf(fd, "type=%d\n", 3);       // CV_16S
	fprintf(fd, "data=<%d|", 640*480*2);
	fwrite(depth, sizeof(uint16_t), 640*480, fd);
	flock( fileno(fd), LOCK_UN );
	fclose(fd);

	pthread_mutex_unlock(&G_kt.mutex);
}

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp){
	//static unsigned char out_rgb[640*480*3];

	pthread_mutex_lock(&G_kt.mutex);

	// swap buffers
	assert (G_kt.rgb_back == rgb);
	G_kt.rgb_back = G_kt.rgb_mid;
	freenect_set_video_buffer(dev, G_kt.rgb_back);
	G_kt.rgb_mid = (uint8_t*)rgb;


	/*int i;
	for (i=0; i<640*480*3; i++){
		out_rgb[i] = G_kt.rgb_mid[i];
	}*/

	


	FILE* fd = fopen("image","a+");
	assert( fd );
	flock( fileno(fd), LOCK_EX );
	ftruncate( fileno(fd), 0 );
	fprintf(fd, "class=Mat:Cv\n");
	fprintf(fd, "cols=640\n");
	fprintf(fd, "rows=480\n");
	fprintf(fd, "step=%d\n", 640*3);
	fprintf(fd, "type=%d\n", 16);
	fprintf(fd, "data=<%d|", 640*480*3);
	fwrite( rgb, sizeof(uint8_t), 640*480*3, fd);
	flock( fileno(fd), LOCK_UN );
	fclose(fd);

	pthread_mutex_unlock(&G_kt.mutex);
}

/*-------------------------------------------------------------------------------------*/



/*=====================================- KINECT -======================================*/


int kinect_init(Kinect* kt){
	pthread_mutex_init(&kt->mutex, NULL);

	kt->depth_mid   = (uint8_t*)malloc(640*480*3);
	kt->depth_front = (uint8_t*)malloc(640*480*3);
	kt->rgb_back    = (uint8_t*)malloc(640*480*3);
	kt->rgb_mid     = (uint8_t*)malloc(640*480*3);
	kt->rgb_front   = (uint8_t*)malloc(640*480*3);


	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3) * 6;
		kt->t_gamma[i] = v*6*256;
	}



	if (freenect_init(&kt->ctx, NULL) < 0) {
		fprintf(stderr, "freenect_init() failed\n");
		return 1;
	}



	freenect_set_log_level(kt->ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(kt->ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));



	int nr_devices = freenect_num_devices (kt->ctx);
	fprintf(stderr, "Number of devices found: %d\n", nr_devices);



	int user_device_number = 0;

	if (nr_devices < 1) {
		fprintf(stderr, "No devices detected\n");
		freenect_shutdown(kt->ctx);
		return 1;
	}



	if (freenect_open_device(kt->ctx, &kt->dev, user_device_number) < 0) {
		fprintf(stderr, "Could not open device\n");
		freenect_shutdown(kt->ctx);
		return 1;
	}


	int freenect_angle = 0;
	freenect_set_tilt_degs(kt->dev,freenect_angle);
	freenect_set_led(kt->dev,LED_RED);
	freenect_set_depth_callback(kt->dev, depth_cb);
	freenect_set_video_callback(kt->dev, rgb_cb);
	freenect_set_video_mode(kt->dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
	freenect_set_depth_mode(kt->dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
	freenect_set_video_buffer(kt->dev, kt->rgb_back);

	freenect_start_depth(kt->dev);
	freenect_start_video(kt->dev);


	return 0;

}


void kinect_finish(Kinect* kt){
	pthread_mutex_unlock(&G_kt.mutex);
	freenect_stop_depth(kt->dev);
	freenect_stop_video(kt->dev);
	freenect_close_device(kt->dev);
	freenect_shutdown(kt->ctx);
}


void kinect_read(Kinect* kt){
	freenect_update_tilt_state(kt->dev);
}


/*-------------------------------------------------------------------------------------*/




/*======================================- MAIN -=======================================*/

void intHandler(int dummy){
	kinect_finish(&G_kt);
	fprintf(stderr,"Kinnnect Finalizing\n");
	sleep(1);
	exit(0);
}


int main(int argc, char **argv){

	int error = kinect_init(&G_kt);
	if ( error ){
		printf("Error\n");
		return 1;
	}

	
	//signal(SIGQUIT, intHandler);
	signal(SIGINT, intHandler);
	//signal(SIGHUP, intHandler);

	while ( !feof(stdin) ){
		kinect_read(&G_kt);
		printf("#end\n");
		fflush(stdout);
		usleep(20000);
	}
	kinect_finish(&G_kt);
	return 0;
}


/*-------------------------------------------------------------------------------------*/
