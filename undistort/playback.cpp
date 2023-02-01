#include <stdio.h>
#include <iostream>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

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


int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("Usage: playback_external_sync.exe <master.mkv>\n");
        return 1;
    }

    size_t file_count = 1; 
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

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
for(int i=0; i<20; i++){
stream_result = k4a_playback_get_next_capture(files[0].handle, &files[0].capture);
}

//
for(int i=0; i<4000;i++){
	print_capture_info(files);
	stream_result = k4a_playback_get_next_capture(files[0].handle, &files[0].capture);
}
}


