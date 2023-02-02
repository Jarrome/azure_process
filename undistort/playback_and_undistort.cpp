#include <stdio.h>
#include <iostream>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include "depthir_to_color_and_undist.h"

#include <turbojpeg.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

typedef struct
{
    char *filename;
    k4a_playback_t handle;
    k4a_record_configuration_t record_config;
    k4a_capture_t capture;
} recording_t;

static void print_capture_info(recording_t *file)
{
    k4a_image_t images[3];
    images[0] = k4a_capture_get_color_image(file->capture);
    images[1] = k4a_capture_get_depth_image(file->capture);
    images[2] = k4a_capture_get_ir_image(file->capture);

    printf("%-32s", file->filename);
    for (int i = 0; i < 3; i++)
    {
        if (images[i] != NULL)
        {
            uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
            printf("  %7ju usec", timestamp);
            k4a_image_release(images[i]);
            images[i] = NULL;
        }
        else
        {
            printf("  %12s", "");
        }
    }
    printf("\n");
}

bool MJPG2BGRA(const k4a_image_t &mjpg_im, k4a_image_t &bgra_im){

	k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
		     k4a_image_get_width_pixels(mjpg_im),
		     k4a_image_get_height_pixels(mjpg_im),
		     k4a_image_get_width_pixels(mjpg_im) * 4 * (int)sizeof(uint8_t),
		     &bgra_im);
	tjhandle m_decompressor;
	m_decompressor = tjInitDecompress();
	const int decompressStatus = tjDecompress2(m_decompressor,
			k4a_image_get_buffer(mjpg_im),
			static_cast<unsigned long>(k4a_image_get_size(mjpg_im)),
			k4a_image_get_buffer(bgra_im),
			k4a_image_get_width_pixels(mjpg_im),
			0,
			k4a_image_get_height_pixels(mjpg_im),
			TJPF_BGRA,
			TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
	(void) tjDestroy(m_decompressor);
	return true;

}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("Usage: playback_external_sync.exe <master.mkv>\n");
        return 1;
    }

    size_t file_count = 1; 
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

pinhole_t pinhole;
k4a_image_t depth_image = NULL;
k4a_image_t compressed_color_image = NULL;
k4a_image_t color_image;

k4a_image_t ir_image = NULL;

k4a_image_t transformed_depth_image = NULL;
k4a_image_t transformed_ir_image = NULL;

k4a_image_t undistorted_color = NULL;
k4a_image_t undistorted_depth = NULL;
k4a_image_t undistorted_ir = NULL;



k4a_transformation_t transformation = NULL;
k4a_image_t custom_ir_image = NULL;
k4a_image_t custom_depth_image = NULL;

int ir_image_width_pixels;// = k4a_image_get_width_pixels(ir_image);
int ir_image_height_pixels;// = k4a_image_get_height_pixels(ir_image);
int ir_image_stride_bytes;// = k4a_image_get_stride_bytes(ir_image);
uint8_t *ir_image_buffer;// = k4a_image_get_buffer(ir_image);
int depth_image_width_pixels;// = k4a_image_get_width_pixels(ir_image);
int depth_image_height_pixels;// = k4a_image_get_height_pixels(ir_image);
int depth_image_stride_bytes;// = k4a_image_get_stride_bytes(ir_image);
uint8_t *depth_image_buffer;// = k4a_image_get_buffer(ir_image);
interpolation_t interpolation_type = INTERPOLATION_NEARESTNEIGHBOR;
/*
interpolation_t depth_interpolation = INTERPOLATION_NEARESTNEIGHBOR;
interpolation_t color_interpolation = INTERPOLATION_BILINEAR;
interpolation_t IR_interpolation = INTERPOLATION_BILINEAR_DEPTH;
*/

    k4a_image_t lut = NULL;






    // Allocate memory to store the state of N recordings.
    recording_t *files =  (recording_t*) malloc (sizeof(recording_t) * file_count);
    if (files == NULL)
    {
        printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(recording_t) * file_count);
        return 1;
    }
    memset(files, 0, sizeof(recording_t) * file_count);

    // Open each recording file and validate they were recorded in master/subordinate mode.
files[0].filename = argv[1];

result = k4a_playback_open(files[0].filename, &files[0].handle);
if (result != K4A_RESULT_SUCCEEDED)
{
	printf("Failed to open file: %s\n", files[0].filename);
	return 1;
}

result = k4a_playback_get_record_configuration(files[0].handle, &files[0].record_config);
if (result != K4A_RESULT_SUCCEEDED)
{
    printf("Failed to get record configuration for file: %s\n", files[0].filename);
    return 1;
}
k4a_stream_result_t stream_result;
// preload 20 frames for fear has depth but color is NULL
for(int i=0; i<2; i++){
	std::cout << i;
stream_result = k4a_playback_get_next_capture(files[0].handle, &files[0].capture);
}

// calibration
k4a_calibration_t calibration;
if (K4A_RESULT_SUCCEEDED !=
k4a_playback_get_calibration(files[0].handle, &calibration))
{
printf("Failed to get calibration\n");
return 1;
}

// Generate a pinhole model for depth camera
pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_COLOR);
k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16,
	     pinhole.width,
	     pinhole.height,
	     pinhole.width * (int)sizeof(coordinate_t),
	     &lut);


create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_COLOR, &pinhole, lut, interpolation_type);


for(int i=0; i<4000;i++){
    depth_image = k4a_capture_get_depth_image(files[0].capture);
    compressed_color_image = k4a_capture_get_color_image(files[0].capture);
	//decompress mjpg to bgra
    MJPG2BGRA(compressed_color_image, color_image);
	

    ir_image = k4a_capture_get_ir_image(files[0].capture);

ir_image_width_pixels = k4a_image_get_width_pixels(ir_image);
ir_image_height_pixels = k4a_image_get_height_pixels(ir_image);
ir_image_stride_bytes = k4a_image_get_stride_bytes(ir_image);
ir_image_buffer = k4a_image_get_buffer(ir_image);
if (K4A_RESULT_SUCCEEDED != 
            k4a_image_create_from_buffer(K4A_IMAGE_FORMAT_CUSTOM16,
                                         ir_image_width_pixels,
                                         ir_image_height_pixels,
                                         ir_image_stride_bytes,
                                         ir_image_buffer,
                                         ir_image_height_pixels * ir_image_stride_bytes,
                                         [](void *_buffer, void *context) {
                                            delete[](uint8_t *) _buffer;
                                            (void)context;
                                         },
                                         NULL,
                                         &custom_ir_image))
	{
	    printf("Failed to create custom ir image\n");
	    return false;
	}
	// transform
	k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
		     pinhole.width,
		     pinhole.height,
			pinhole.width * (int)sizeof(uint16_t),
		     &transformed_depth_image);

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16,
		     pinhole.width,
		     pinhole.height,
			pinhole.width * (int)sizeof(uint16_t),
		     &transformed_ir_image);

	// transform to color coordinate
	//
	transformation = k4a_transformation_create(&calibration);


	if (K4A_RESULT_SUCCEEDED !=
		k4a_transformation_depth_image_to_color_camera_custom(transformation,
		depth_image,
		custom_ir_image,
		transformed_depth_image,
		transformed_ir_image,
		K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST,
		0))
	{
	printf("Failed to compute transformed depth and custom image\n");
	return false;
	}


	using namespace std::chrono_literals;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// undistort
	k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			     pinhole.width,
			     pinhole.height,
			     pinhole.width *4* (int)sizeof(uint8_t),
			     &undistorted_color);
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16,
			     pinhole.width,
			     pinhole.height,
			     pinhole.width * (int)sizeof(uint16_t),
			     &undistorted_depth);
	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16,
			     pinhole.width,
			     pinhole.height,
			     pinhole.width * (int)sizeof(uint16_t),
			     &undistorted_ir);

	remap_color(color_image, lut, undistorted_color, interpolation_type);
	remap(transformed_depth_image, lut, undistorted_depth, interpolation_type);
	remap(transformed_ir_image, lut, undistorted_ir, interpolation_type);


	//save image
	cv::Mat depth_cv_(k4a_image_get_height_pixels(undistorted_depth),k4a_image_get_width_pixels(undistorted_depth),
			CV_16UC1,
			(void*)k4a_image_get_buffer(undistorted_depth),
			cv::Mat::AUTO_STEP);
	cv::Mat ir_cv_(k4a_image_get_height_pixels(undistorted_ir),k4a_image_get_width_pixels(undistorted_ir),
			CV_16UC1,
			(void*)k4a_image_get_buffer(undistorted_ir),
			cv::Mat::AUTO_STEP);
	cv::Mat color_cv_(k4a_image_get_height_pixels(undistorted_color),k4a_image_get_width_pixels(undistorted_color),
			CV_8UC4,
			(void*)k4a_image_get_buffer(undistorted_color),
			cv::Mat::AUTO_STEP);
	/*
		cv::imshow("depth", depth_cv_);
		cv::imshow("color", color_cv_);
		cv::imshow("ir", ir_cv_);
		cv::waitKey();
	*/

	cv::imwrite("depth.jpg", depth_cv_);
	cv::imwrite("color.jpg", color_cv_);
	cv::imwrite("ir.jpg", ir_cv_);

	depth_cv_.release();
	color_cv_.release();
	ir_cv_.release();



	print_capture_info(files);
	stream_result = k4a_playback_get_next_capture(files[0].handle, &files[0].capture);
}




return 0;







}


