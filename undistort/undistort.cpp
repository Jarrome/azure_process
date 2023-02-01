#include "depthir_to_color_and_undist.h"
int main(int argc, char **argv)
{
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    k4a_capture_t capture = NULL;
    std::string file_name;
    uint32_t device_count = 0;
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t depth_image = NULL;
k4a_image_t color_image = NULL;
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

    k4a_image_t lut = NULL;
    k4a_image_t undistorted = NULL;
    interpolation_t interpolation_type = INTERPOLATION_NEARESTNEIGHBOR;
    pinhole_t pinhole;
    std::cout << argc << std::endl;
    if (argc != 3)
    {
        printf("undistort.exe <interpolation type> <output file>\n");
        printf("interpolation type: \n");
        printf("    - 0: NearestNeighbor\n");
        printf("    - 1: Bilinear\n");
        printf("    - 2: Bilinear with invalidation\n");
        printf("e.g. undistort.exe 2 undistorted_depth.csv\n");
        returnCode = 2;
        goto Exit;
    }

    interpolation_type = (interpolation_t)(std::stoi(argv[1]));
    file_name = argv[2];

    device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
        printf("No K4A devices found\n");
        return 0;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
        goto Exit;
    }

    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P; 
    //config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;


    k4a_calibration_t calibration;
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        goto Exit;
    }


    // Generate a pinhole model for depth camera
    pinhole = create_pinhole_from_xy_range(&calibration, K4A_CALIBRATION_TYPE_COLOR);
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM16,
                     pinhole.width,
                     pinhole.height,
                     pinhole.width * (int)sizeof(coordinate_t),
                     &lut);


    create_undistortion_lut(&calibration, K4A_CALIBRATION_TYPE_COLOR, &pinhole, lut, interpolation_type);

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
        goto Exit;
    }

    // Get a capture
    // capture 3 times because first several color is null
    for(int i=0; i<20; i++){
	    switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
	    {
	    case K4A_WAIT_RESULT_SUCCEEDED:
		break;
	    case K4A_WAIT_RESULT_TIMEOUT:
		printf("Timed out waiting for a capture\n");
		goto Exit;
	    case K4A_WAIT_RESULT_FAILED:
		printf("Failed to read a capture\n");
		goto Exit;
	    }
	    std::cout<< i;
    }

    // Get images
    //
    //
    // 
    depth_image = k4a_capture_get_depth_image(capture);
    color_image = k4a_capture_get_color_image(capture);
    ir_image = k4a_capture_get_ir_image(capture);



    if (depth_image == 0)
    {
        printf("Failed to get depth image from capture\n");
        goto Exit;
    }

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

std::cout <<"3"<<std::flush;

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
std::cout <<"4"<<std::flush;



    remap_color(color_image, lut, undistorted_color, interpolation_type);
    remap(transformed_depth_image, lut, undistorted_depth, interpolation_type);
    remap(transformed_ir_image, lut, undistorted_ir, interpolation_type);
std::cout <<"5"<<std::flush;

    //write_csv_file(file_name.c_str(), undistorted);

    k4a_image_release(depth_image);
    k4a_capture_release(capture);
    k4a_image_release(lut);
    //k4a_image_release(undistorted);

    returnCode = 0;
Exit:
    if (device != NULL)
    {
        k4a_device_close(device);
    }

    return returnCode;
}
