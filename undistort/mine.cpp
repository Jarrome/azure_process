// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <algorithm>

#define INVALID INT32_MIN

#include <chrono>
#include <thread>

typedef struct _pinhole_t
{
    float px;
    float py;
    float fx;
    float fy;

    int width;
    int height;
} pinhole_t;

typedef struct _coordinate_t
{
    int x;
    int y;
    float weight[4];
} coordinate_t;

typedef enum
{
    INTERPOLATION_NEARESTNEIGHBOR, /**< Nearest neighbor interpolation */
    INTERPOLATION_BILINEAR,        /**< Bilinear interpolation */
    INTERPOLATION_BILINEAR_DEPTH   /**< Bilinear interpolation with invalidation when neighbor contain invalid
                                                 data with value 0 */
} interpolation_t;

// Compute a conservative bounding box on the unit plane in which all the points have valid projections
static void compute_xy_range(const k4a_calibration_t *calibration,
                             const k4a_calibration_type_t camera,
                             const int width,
                             const int height,
                             float &x_min,
                             float &x_max,
                             float &y_min,
                             float &y_max)
{
    // Step outward from the centre point until we find the bounds of valid projection
    const float step_u = 0.25f;
    const float step_v = 0.25f;
    const float min_u = 0;
    const float min_v = 0;
    const float max_u = (float)width - 1;
    const float max_v = (float)height - 1;
    const float center_u = 0.5f * width;
    const float center_v = 0.5f * height;

    int valid;
    k4a_float2_t p;
    k4a_float3_t ray;

    // search x_min
    for (float uv[2] = { center_u, center_v }; uv[0] >= min_u; uv[0] -= step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_min = ray.xyz.x;
    }

    // search x_max
    for (float uv[2] = { center_u, center_v }; uv[0] <= max_u; uv[0] += step_u)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        x_max = ray.xyz.x;
    }

    // search y_min
    for (float uv[2] = { center_u, center_v }; uv[1] >= min_v; uv[1] -= step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_min = ray.xyz.y;
    }

    // search y_max
    for (float uv[2] = { center_u, center_v }; uv[1] <= max_v; uv[1] += step_v)
    {
        p.xy.x = uv[0];
        p.xy.y = uv[1];
        k4a_calibration_2d_to_3d(calibration, &p, 1.f, camera, camera, &ray, &valid);

        if (!valid)
        {
            break;
        }
        y_max = ray.xyz.y;
    }
}

static pinhole_t create_pinhole_from_xy_range(const k4a_calibration_t *calibration, const k4a_calibration_type_t camera)
{
    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        width = calibration->color_camera_calibration.resolution_width;
        height = calibration->color_camera_calibration.resolution_height;
    }

    float x_min = 0, x_max = 0, y_min = 0, y_max = 0;
    compute_xy_range(calibration, camera, width, height, x_min, x_max, y_min, y_max);

    pinhole_t pinhole;

    float fx = 1.f / (x_max - x_min);
    float fy = 1.f / (y_max - y_min);
    float px = -x_min * fx;
    float py = -y_min * fy;

    pinhole.fx = fx * width;
    pinhole.fy = fy * height;
    pinhole.px = px * width;
    pinhole.py = py * height;
    pinhole.width = width;
    pinhole.height = height;

    return pinhole;
}

static void create_undistortion_lut(const k4a_calibration_t *calibration,
                                    const k4a_calibration_type_t camera,
                                    const pinhole_t *pinhole,
                                    k4a_image_t lut,
                                    interpolation_t type)
{
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    k4a_float3_t ray;
    ray.xyz.z = 1.f;

    int src_width = calibration->depth_camera_calibration.resolution_width;
    int src_height = calibration->depth_camera_calibration.resolution_height;
    if (camera == K4A_CALIBRATION_TYPE_COLOR)
    {
        src_width = calibration->color_camera_calibration.resolution_width;
        src_height = calibration->color_camera_calibration.resolution_height;
    }

    for (int y = 0, idx = 0; y < pinhole->height; y++)
    {
        ray.xyz.y = ((float)y - pinhole->py) / pinhole->fy;

        for (int x = 0; x < pinhole->width; x++, idx++)
        {
            ray.xyz.x = ((float)x - pinhole->px) / pinhole->fx;

            k4a_float2_t distorted;
            int valid;
            k4a_calibration_3d_to_2d(calibration, &ray, camera, camera, &distorted, &valid);

            coordinate_t src;
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                // Remapping via nearest neighbor interpolation
                src.x = (int)floorf(distorted.xy.x + 0.5f);
                src.y = (int)floorf(distorted.xy.y + 0.5f);
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                // Remapping via bilinear interpolation
                src.x = (int)floorf(distorted.xy.x);
                src.y = (int)floorf(distorted.xy.y);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }

            if (valid && src.x >= 0 && src.x < src_width && src.y >= 0 && src.y < src_height)
            {
                lut_data[idx] = src;

                if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // Compute the floating point weights, using the distance from projected point src to the
                    // image coordinate of the upper left neighbor
                    float w_x = distorted.xy.x - src.x;
                    float w_y = distorted.xy.y - src.y;
                    float w0 = (1.f - w_x) * (1.f - w_y);
                    float w1 = w_x * (1.f - w_y);
                    float w2 = (1.f - w_x) * w_y;
                    float w3 = w_x * w_y;

                    // Fill into lut
                    lut_data[idx].weight[0] = w0;
                    lut_data[idx].weight[1] = w1;
                    lut_data[idx].weight[2] = w2;
                    lut_data[idx].weight[3] = w3;
                }
            }
            else
            {
                lut_data[idx].x = INVALID;
                lut_data[idx].y = INVALID;
            }
        }
    }
}

static void remap_color(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
	std::cout << "1" << std::flush;
    int src_width = k4a_image_get_width_pixels(src);
std::cout << "1" << std::flush;

    int dst_width = k4a_image_get_width_pixels(dst);
std::cout << "1" << std::flush;

    int dst_height = k4a_image_get_height_pixels(dst);
std::cout << "1" << std::flush;


    uint32_t *src_data = (uint32_t *)(void *)k4a_image_get_buffer(src);
    uint32_t *dst_data = (uint32_t *)(void *)k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint32_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint32_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),
                                               std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                                               std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint32_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                                         neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                                         0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}


static void remap(const k4a_image_t src, const k4a_image_t lut, k4a_image_t dst, interpolation_t type)
{
    int src_width = k4a_image_get_width_pixels(src);
    int dst_width = k4a_image_get_width_pixels(dst);
    int dst_height = k4a_image_get_height_pixels(dst);

    uint16_t *src_data = (uint16_t *)(void *)k4a_image_get_buffer(src);
    uint16_t *dst_data = (uint16_t *)(void *)k4a_image_get_buffer(dst);
    coordinate_t *lut_data = (coordinate_t *)(void *)k4a_image_get_buffer(lut);

    memset(dst_data, 0, (size_t)dst_width * (size_t)dst_height * sizeof(uint16_t));

    for (int i = 0; i < dst_width * dst_height; i++)
    {
        if (lut_data[i].x != INVALID && lut_data[i].y != INVALID)
        {
            if (type == INTERPOLATION_NEARESTNEIGHBOR)
            {
                dst_data[i] = src_data[lut_data[i].y * src_width + lut_data[i].x];
            }
            else if (type == INTERPOLATION_BILINEAR || type == INTERPOLATION_BILINEAR_DEPTH)
            {
                const uint16_t neighbors[4]{ src_data[lut_data[i].y * src_width + lut_data[i].x],
                                             src_data[lut_data[i].y * src_width + lut_data[i].x + 1],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x],
                                             src_data[(lut_data[i].y + 1) * src_width + lut_data[i].x + 1] };

                if (type == INTERPOLATION_BILINEAR_DEPTH)
                {
                    // If the image contains invalid data, e.g. depth image contains value 0, ignore the bilinear
                    // interpolation for current target pixel if one of the neighbors contains invalid data to avoid
                    // introduce noise on the edge. If the image is color or ir images, user should use
                    // INTERPOLATION_BILINEAR
                    if (neighbors[0] == 0 || neighbors[1] == 0 || neighbors[2] == 0 || neighbors[3] == 0)
                    {
                        continue;
                    }

                    // Ignore interpolation at large depth discontinuity without disrupting slanted surface
                    // Skip interpolation threshold is estimated based on the following logic:
                    // - angle between two pixels is: theta = 0.234375 degree (120 degree / 512) in binning resolution
                    // mode
                    // - distance between two pixels at same depth approximately is: A ~= sin(theta) * depth
                    // - distance between two pixels at highly slanted surface (e.g. alpha = 85 degree) is: B = A /
                    // cos(alpha)
                    // - skip_interpolation_ratio ~= sin(theta) / cos(alpha)
                    // We use B as the threshold that to skip interpolation if the depth difference in the triangle is
                    // larger than B. This is a conservative threshold to estimate largest distance on a highly slanted
                    // surface at given depth, in reality, given distortion, distance, resolution difference, B can be
                    // smaller
                    const float skip_interpolation_ratio = 0.04693441759f;
                    float depth_min = std::min(std::min(neighbors[0], neighbors[1]),
                                               std::min(neighbors[2], neighbors[3]));
                    float depth_max = std::max(std::max(neighbors[0], neighbors[1]),
                                               std::max(neighbors[2], neighbors[3]));
                    float depth_delta = depth_max - depth_min;
                    float skip_interpolation_threshold = skip_interpolation_ratio * depth_min;
                    if (depth_delta > skip_interpolation_threshold)
                    {
                        continue;
                    }
                }

                dst_data[i] = (uint16_t)(neighbors[0] * lut_data[i].weight[0] + neighbors[1] * lut_data[i].weight[1] +
                                         neighbors[2] * lut_data[i].weight[2] + neighbors[3] * lut_data[i].weight[3] +
                                         0.5f);
            }
            else
            {
                printf("Unexpected interpolation type!\n");
                exit(-1);
            }
        }
    }
}

static void write_csv_file(const char *file_name, const k4a_image_t src)
{
    FILE *fp = 0;
#ifdef _MSC_VER
    fopen_s(&fp, file_name, "w");
#else
    fp = fopen(file_name, "w");
#endif

    int width = k4a_image_get_width_pixels(src);
    int height = k4a_image_get_height_pixels(src);
    uint16_t *src_data = (uint16_t *)(void *)k4a_image_get_buffer(src);

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            fprintf(fp, "%d", src_data[y * width + x]);
            if (x < width - 1)
            {
                fprintf(fp, ",");
            }
        }
        fprintf(fp, "\n");
    }

    fclose(fp);
}

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
