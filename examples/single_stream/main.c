// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <stdio.h>
#include <stdlib.h>
#include <k4a/k4a.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>
#include <k4ainternal/common.h>
#include <cJSON.h>
#include <locale.h>

#define PORT 8080
#define MAXLINE 1024

#define SIZEOF(a) sizeof(a) / sizeof(*a)

int server_fd, new_socket, valread;
struct sockaddr_in address;
struct timeval tv; // for timestamps
int opt = 1;
int addrlen = sizeof(address);

int INTRINSIC_BYTE_SIZE = 60;
int EXTRINSIC_BYTE_SIZE = 48;
int CAMERA_CALIB_BUFFER_SIZE = 120;
int *buffer, *pBuffer, word, nibblesWritten;
int frameNum;

typedef struct ROI
{
    int _x;
    int _y;
    int _w;
    int _h;
} Roi;

typedef struct FrameProperties
{
    int width;
    int height;
    float metric_radius;
} FrameProps;

unsigned char *serialize_int(unsigned char *buffer, int value)
{
    /* Write big-endian int value into buffer; assumes 32-bit int and 8-bit char. */
    buffer[0] = value >> 24;
    buffer[1] = value >> 16;
    buffer[2] = value >> 8;
    buffer[3] = value;
    return buffer + 4;
}

unsigned char *serialize_float(unsigned char *buffer, float value)
{
    buffer[0] = ((long)value & 0xff000000) >> 24;
    buffer[1] = ((long)value & 0x00ff0000) >> 16;
    buffer[2] = ((long)value & 0x0000ff00) >> 8;
    buffer[3] = ((long)value & 0x000000ff);
    return buffer + 4;
}

/* In order to send struct over tcp, we must serialize the struct as a byte stream */
unsigned char *serialize_frame_props(unsigned char *buffer, FrameProps *value)
{
    buffer = serialize_int(buffer, value->width);
    buffer = serialize_int(buffer, value->height);
    buffer = serialize_float(buffer, value->metric_radius);
    return buffer;
}

void ConfigureTCP()
{
    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
}

void EncodeVLE(int value)
{
    do
    {
        int nibble = value & 0x7; // lower 3 bits
        if (value >>= 3)
            nibble |= 0x8;
        word <<= 4;
        word |= nibble;
        if (++nibblesWritten == 8)
        {
            *pBuffer++ = word;
            // printf("Point %d \n", *pBuffer);
            nibblesWritten = 0;
            word = 0;
        }
    } while (value);
}

int DecodeVLE()
{
    unsigned int nibble;
    int value = 0, bits = 29;
    do
    {
        if (!nibblesWritten)
        {
            word = *pBuffer++; //   load word
            nibblesWritten = 8;
        }
        nibble = word & 0xf0000000;
        value |= (nibble << 1) >> bits;
        word <<= 4;
        nibblesWritten--;
        bits -= 3;
    } while (nibble & 0x80000000);
    return value;
}

int PixelOverlapROI(int p_x, int p_y, Roi roi[], int count)
{
    int result = 0;
    for (int i = 0; i < count; i++)
    {
        // printf("ROI X %d, ROI Y %d\n", roi[i]._x, roi[i]._y);
        if (p_x >= roi[i]._x && p_x <= (roi[i]._x + roi[i]._w))
        { // if within bounds
            if (p_y >= roi[i]._y && p_y <= (roi[i]._y + roi[i]._h))
            {
                result = 1;
                // printf("Pixel %d, %d is in bounds\n", p_x, p_y);
            }
        }
    }
    return result;
}

int CompressRVL(short *input,
                char *output,
                int numPixels,
                k4a_image_t depth_img,
                Roi roi[],
                size_t roi_size,
                cJSON *depth_frame_json)
{
    // size_t passed_array_size = sizeof(*roi) / sizeof(Roi);
    buffer = pBuffer = (int *)output;
    nibblesWritten = 0;
    short *end = input + numPixels;
    short previous = 0;

    /*printf(" | Depth16 res:%4dx%4d stride:%5d\n",
           k4a_image_get_height_pixels(depth_img),
           k4a_image_get_width_pixels(depth_img),
           k4a_image_get_stride_bytes(depth_img));*/
    printf("roi %p, roi size %p, depth img %ld\n", &roi, &depth_img, roi_size);
    /*int _w = k4a_image_get_width_pixels(depth_img);
    for (int j = 0; j < k4a_image_get_height_pixels(depth_img); j++)
    {
        for (int i = 0; i < k4a_image_get_width_pixels(depth_img); i++)
        {
            if (PixelOverlapROI(i, j, roi, roi_size) == 0)
            {
                *(input + (j * _w + i)) = 0;
                // printf("Addr %d\n", (j * k4a_image_get_width_pixels(depth_img) + i));
            }
        }
    }*/
    int totalZeros = 0;
    int totalOnes = 0;
    while (input != end)
    {
        int zeros = 0, nonzeros = 0;
        for (; (input != end) && !*input; input++, zeros++)
            ;
        EncodeVLE(zeros); // number of zeros
        for (short *p = input; (p != end) && *p++; nonzeros++)
            ;
        EncodeVLE(nonzeros); // number of nonzeros
        for (int i = 0; i < nonzeros; i++)
        {
            short current = *input++;
            int delta = current - previous;
            int positive = (delta << 1) ^ (delta >> 31);
            EncodeVLE(positive); // nonzero value
            previous = current;
        }
        totalZeros += zeros;
        totalOnes += nonzeros;
    }
    // printf("Num Zeros %d, Num Ones %d\n", totalZeros, totalOnes);
    if (nibblesWritten)
    { // last few values
        *pBuffer++ = word << 4 * (8 - nibblesWritten);
    }
    cJSON *rvl_num_zeros = NULL;
    cJSON *rvl_num_ones = NULL;
    rvl_num_zeros = cJSON_CreateNumber(totalZeros);
    rvl_num_ones = cJSON_CreateNumber(totalOnes);
    cJSON_AddItemToObject(depth_frame_json, "total_zeros", rvl_num_zeros);
    cJSON_AddItemToObject(depth_frame_json, "total_ones", rvl_num_ones);
    int result = (int)((char *)pBuffer - (char *)buffer); // num bytes
    return result;
}

void DecompressRVL(char *input, short *output, int numPixels)
{
    buffer = pBuffer = (int *)input;
    nibblesWritten = 0;
    short current, previous = 0;
    int numPixelsToDecode = numPixels;
    while (numPixelsToDecode)
    {
        int zeros = DecodeVLE(); // number of zeros
        numPixelsToDecode -= zeros;
        for (; zeros; zeros--)
            *output++ = 0;
        int nonzeros = DecodeVLE(); // number of nonzeros
        numPixelsToDecode -= nonzeros;
        for (; nonzeros; nonzeros--)
        {
            int positive = DecodeVLE(); // nonzero value
            int delta = (positive >> 1) ^ -(positive & 1);
            current = previous + delta;
            *output++ = current;
            previous = current;
        }
    }
}

void print_calibration_properties(k4a_calibration_camera_t calib)
{
    printf("resolution width: %d\nresolution height: %d\n", calib.resolution_width, calib.resolution_height);
    printf("max fov of camera (metric radius): %f\n", calib.metric_radius);
    printf("principal point x: %f\nprincipal point y: %f\n",
           calib.intrinsics.parameters.param.cx,
           calib.intrinsics.parameters.param.cy);
    printf("focal length x: %f\nfocal length y: %f\n",
           calib.intrinsics.parameters.param.fx,
           calib.intrinsics.parameters.param.fy);
    printf("radial distortion coefficients:\n");
    printf("k1: %f\nk2: %f\nk3: %f\nk4: %f\nk5: %f\nk6: %f\n",
           calib.intrinsics.parameters.param.k1,
           calib.intrinsics.parameters.param.k2,
           calib.intrinsics.parameters.param.k3,
           calib.intrinsics.parameters.param.k4,
           calib.intrinsics.parameters.param.k5,
           calib.intrinsics.parameters.param.k6);
    printf("center of distortion in Z=1 plane, x: %f\ncenter of distortion in Z=1 plane, y: %f\n",
           calib.intrinsics.parameters.param.codx,
           calib.intrinsics.parameters.param.cody);
    printf("tangential distortion coefficient x: %f\ntangential distortion coefficient y: %f\n",
           calib.intrinsics.parameters.param.p1,
           calib.intrinsics.parameters.param.p2);
    printf("metric radius: %f\n", calib.intrinsics.parameters.param.metric_radius);
}

void get_calibration_data(k4a_calibration_t *calibration,
                          float *kinect_intrinsics,
                          float *kinect_extrinsics,
                          FrameProps *frame_props,
                          int color_flag)
{

    k4a_calibration_camera_t calib = (color_flag == 0) ? (*calibration).color_camera_calibration :
                                                         (*calibration).depth_camera_calibration;
    k4a_calibration_extrinsics_t calib_extrinsics =
        (*calibration).extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
    // printf("\n===== Device %d: %d =====\n", (int)deviceIndex, get_serial(device));
    // print_calibration_properties(calib);

    // Declare memory space for intrinsics, and extrinsics write values to memory
    float intrinsics[] = { calib.intrinsics.parameters.param.cx,           calib.intrinsics.parameters.param.cy,
                           calib.intrinsics.parameters.param.fx,           calib.intrinsics.parameters.param.fy,
                           calib.intrinsics.parameters.param.k1,           calib.intrinsics.parameters.param.k2,
                           calib.intrinsics.parameters.param.k3,           calib.intrinsics.parameters.param.k4,
                           calib.intrinsics.parameters.param.k5,           calib.intrinsics.parameters.param.k6,
                           calib.intrinsics.parameters.param.codx,         calib.intrinsics.parameters.param.cody,
                           calib.intrinsics.parameters.param.p1,           calib.intrinsics.parameters.param.p2,
                           calib.intrinsics.parameters.param.metric_radius };
    float extrinsics[] = { calib_extrinsics.rotation[0],    calib_extrinsics.rotation[1],
                           calib_extrinsics.rotation[2],    calib_extrinsics.rotation[3],
                           calib_extrinsics.rotation[4],    calib_extrinsics.rotation[5],
                           calib_extrinsics.rotation[6],    calib_extrinsics.rotation[7],
                           calib_extrinsics.rotation[8],    calib_extrinsics.translation[0],
                           calib_extrinsics.translation[1], calib_extrinsics.translation[2] };
    (*frame_props).width = calib.resolution_width;
    (*frame_props).height = calib.resolution_height;
    (*frame_props).metric_radius = calib.metric_radius;
    printf("size of calib %ld\t, size of kinect intrinsics %ld",
           sizeof(calib.intrinsics.parameters),
           sizeof(intrinsics));
    for (size_t i = 0; i < (size_t)(INTRINSIC_BYTE_SIZE / sizeof(float)); i++)
    {
        kinect_intrinsics[i] = intrinsics[i];
    }
    for (size_t i = 0; i < (size_t)(EXTRINSIC_BYTE_SIZE / sizeof(float)); i++)
    {
        kinect_extrinsics[i] = extrinsics[i];
    }
}

int main(int argc, char **argv)
{
    frameNum = 0;
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    int captureFrameCount;
    k4a_capture_t capture = NULL;

    if (argc < 2)
    {
        printf("%s FRAMECOUNT\n", argv[0]);
        printf("Capture FRAMECOUNT color and depth frames from the device using the separate get frame APIs\n");
        returnCode = 2;
        goto Exit;
    }

    captureFrameCount = atoi(argv[1]);
    printf("Capturing %d frames\n", captureFrameCount);

    uint32_t device_count = k4a_device_get_installed_count();

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

    k4a_calibration_t calibration;

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start device\n");
        goto Exit;
    }
    // ConfigureUDP();
    ConfigureTCP();

    // get calibration
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("\nFailed to get calibration\n");
        goto Exit;
    }

    float *dcam_kinect_intrinsics = (float *)malloc(INTRINSIC_BYTE_SIZE);
    float *dcam_kinect_extrinsics = (float *)malloc(EXTRINSIC_BYTE_SIZE);
    float *ccam_kinect_intrinsics = (float *)malloc(INTRINSIC_BYTE_SIZE);
    float *ccam_kinect_extrinsics = (float *)malloc(EXTRINSIC_BYTE_SIZE);
    FrameProps frame_props;
    get_calibration_data(&calibration, ccam_kinect_intrinsics, ccam_kinect_extrinsics, &frame_props, 0);
    get_calibration_data(&calibration, dcam_kinect_intrinsics, dcam_kinect_extrinsics, &frame_props, 1);

    /*printf("\nKinect intrinsics %ld, Kinect Extrinsics %ld, Frame Properties %ld",
           sizeof(dcam_kinect_intrinsics),
           sizeof(dcam_kinect_extrinsics),
           sizeof(frame_props));*/
    printf("\nFrame properties %d, %d, %f\n", frame_props.width, frame_props.height, frame_props.metric_radius);
    // send camera intrinsics and extrinsics first
    unsigned char buffer[sizeof(frame_props)];
    serialize_frame_props(buffer, &frame_props);
    printf("\nof size %ld\n", sizeof(frame_props));
    // int camera_calib_props_size = sizeof(frame_props) + INTRINSIC_BYTE_SIZE + EXTRINSIC_BYTE_SIZE;
    int camera_calib_props_size = 120;
    printf("Camera calib prop size %d", camera_calib_props_size);
    char *camera_calib_buffer = (char *)malloc(camera_calib_props_size);
    memcpy(camera_calib_buffer, dcam_kinect_extrinsics, EXTRINSIC_BYTE_SIZE);
    memcpy(camera_calib_buffer + EXTRINSIC_BYTE_SIZE, dcam_kinect_intrinsics, INTRINSIC_BYTE_SIZE);
    // memcpy(camera_calib_buffer+sizeof(dcam_kinect_extrinsics)/sizeof(float) +
    // sizeof(dcam_kinect_intrinsics)/sizeof(float), &frame_props.width, 4); memcpy(camera_calib_buffer,
    // &frame_props.height, 4); memcpy(camera_calib_buffer, &frame_props.metric_radius, 4);
    // memcpy(camera_calib_buffer + 108, &buffer,
    //        12); // copy camera calib props first
    memcpy(camera_calib_buffer + 108, &frame_props.width, 4);
    memcpy(camera_calib_buffer + 112, &frame_props.height, 4);
    memcpy(camera_calib_buffer + 116, &frame_props.metric_radius, 4);
    send(new_socket, camera_calib_buffer, camera_calib_props_size, 0);
    free(camera_calib_buffer);
    // send(new_socket, buffer, sizeof(frame_props), 0);
    // send(new_socket, dcam_kinect_intrinsics, sizeof(float) * 15, 0);
    // send(new_socket, dcam_kinect_extrinsics, sizeof(float) * 12, 0);

    // initialize json
    char *json_file_str = NULL;
    cJSON *monitor = cJSON_CreateObject(); // file container
    cJSON *measurements = NULL;
    cJSON *rois_json = NULL;

    rois_json = cJSON_CreateArray();
    if (rois_json == NULL)
    {
        printf("Error creating json array\n");
        goto Exit;
    }
    measurements = cJSON_CreateArray();
    if (measurements == NULL)
    {
        printf("Error creating json array\n");
        goto Exit;
    }
    // Add main objects to json file
    cJSON_AddItemToObject(monitor, "rois", rois_json);
    cJSON_AddItemToObject(monitor, "measurements", measurements); // add json array

    if (monitor == NULL)
    {
        printf("Error creating json file\n");
        goto Exit;
    }

    // define rois
    Roi roi[2];
    Roi roi_a = (Roi){ ._x = 100, ._y = 100, ._w = 100, ._h = 100 };
    Roi roi_b = (Roi){ ._x = 300, ._y = 200, ._w = 200, ._h = 100 };
    roi[0] = roi_a;
    roi[1] = roi_b;

    for (size_t index_roi = 0; index_roi < sizeof(roi) / sizeof(roi[0]); index_roi++)
    {
        printf("Roi Index %ld\n", index_roi);
        cJSON *roi_instance = cJSON_CreateObject(); // file container
        cJSON *roi_x = cJSON_CreateNumber(roi[index_roi]._x);
        cJSON *roi_y = cJSON_CreateNumber(roi[index_roi]._y);
        cJSON *roi_w = cJSON_CreateNumber(roi[index_roi]._w);
        cJSON *roi_h = cJSON_CreateNumber(roi[index_roi]._h);

        cJSON_AddItemToObject(roi_instance, "roi_x", roi_x);
        cJSON_AddItemToObject(roi_instance, "roi_y", roi_y);
        cJSON_AddItemToObject(roi_instance, "roi_w", roi_w);
        cJSON_AddItemToObject(roi_instance, "roi_h", roi_h);
        cJSON_AddItemToArray(rois_json, roi_instance);
    }

    while (captureFrameCount--)
    {
        k4a_image_t color_image;
        k4a_image_t depth_image;
        k4a_image_t image;

        cJSON *capture_obj = cJSON_CreateObject();
        cJSON *capture_ts = NULL;
        // Get a depth frame
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            gettimeofday(&tv, NULL);
            unsigned long long capture_ms = (unsigned long long)(tv.tv_sec) * 1000 +
                                            (unsigned long long)(tv.tv_usec) / 1000;
            capture_ts = cJSON_CreateNumber(capture_ms);
            cJSON_AddItemToObject(capture_obj, "capture_ts", capture_ts);
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            goto Exit;
        }
        cJSON_AddItemToArray(measurements, capture_obj);
        // printf("%llu\n", milliSecondsSinceEpoch);
        // Probe for a color image
        color_image = k4a_capture_get_color_image(capture);
        // measure time it takes to receive color frame
        gettimeofday(&tv, NULL);
        unsigned long long milliSecondsSinceEpoch = (unsigned long long)(tv.tv_sec) * 1000 +
                                                    (unsigned long long)(tv.tv_usec) / 1000;
        cJSON *color_frame_ts = cJSON_CreateNumber(milliSecondsSinceEpoch);

        uint32_t *input_color_img = (uint32_t *)k4a_image_get_buffer(color_image);
        int32_t total_frame_size_color = (int32_t)k4a_image_get_size(color_image);

        if (color_image)
        {
            /*printf(" | Color res:%4dx%4d stride:%5d ",
                   k4a_image_get_height_pixels(color_image),
                   k4a_image_get_width_pixels(color_image),
                   k4a_image_get_stride_bytes(color_image));*/
            cJSON *color_frame_json = cJSON_CreateObject();
            cJSON_AddItemToArray(measurements, color_frame_json);
            // blackout non-roi regions
            int num_color_zeros = 0;
            clock_t t_color;
            t_color = clock();
            /*int color_img_stride = k4a_image_get_width_pixels(color_image);
            for (int j = 0; j < k4a_image_get_height_pixels(color_image); j++)
            {
                for (int i = 0; i < k4a_image_get_width_pixels(color_image); i++)
                {
                    // printf("Pixel %d\n", (j * color_img_stride + i));
                    if (PixelOverlapROI(i, j, roi, SIZEOF(roi)) == 0)
                    {
                        *(input_color_img + (j * color_img_stride + i)) = (uint32_t)0;
                        num_color_zeros++;
                    }
                }
            }*/
            t_color = clock() - t_color; // timer difference
            double time_taken_color = ((double)t_color) / CLOCKS_PER_SEC;
            cJSON *roi_color_latency = cJSON_CreateNumber(time_taken_color);
            cJSON *num_color_zeros_JSON = cJSON_CreateNumber(num_color_zeros);
            cJSON *color_frame_size = cJSON_CreateNumber(total_frame_size_color);
            cJSON_AddItemToObject(color_frame_json, "is_depth_frame", cJSON_CreateNumber(0));
            cJSON_AddItemToObject(color_frame_json, "roi_color_latency", roi_color_latency);
            cJSON_AddItemToObject(color_frame_json, "num_color_zeros_JSON", num_color_zeros_JSON);
            cJSON_AddItemToObject(color_frame_json, "color_frame_size", color_frame_size);
            cJSON_AddItemToObject(color_frame_json, "color_frame_ts", color_frame_ts);
            // send frame size followed by frame data
            // send(new_socket, &total_frame_size_color, sizeof(total_frame_size_color), 0);
            /*printf("\nColor frame addr size %ld, color frame byte size %d\n",
                   sizeof(total_frame_size_color),
                   total_frame_size_color);*/

            // char *color_buff_index= color_output_buffer;
            /*if (send(new_socket, input_color_img, total_frame_size_color, 0) < 0)
            {
                printf("\nError sending data");
            }*/
            char *color_img_pkt = (char *)malloc(total_frame_size_color + sizeof(int32_t));
            // printf("\nTotal frame size allocated is %ld\n", (total_frame_size_color + sizeof(uint32_t)));
            memcpy(color_img_pkt, &total_frame_size_color, sizeof(int32_t));
            memcpy(color_img_pkt + sizeof(int32_t), input_color_img, k4a_image_get_size(color_image));

            // send(new_socket, input_color_img, total_frame_size_color, 0);
            send(new_socket, color_img_pkt, total_frame_size_color + sizeof(int32_t), 0);
            free(color_img_pkt);

            char color_path[37]; // save to disk
            snprintf(color_path, sizeof(color_path), "/home/meteor/Desktop/color%d.bytes", frameNum);
            // printf("\nAttempt to write file to %s\n", path);
            FILE *color_file = fopen(color_path, "wb");
            fwrite(input_color_img,
                   sizeof(int32_t),
                   k4a_image_get_height_pixels(color_image) * k4a_image_get_width_pixels(color_image),
                   color_file);
            fclose(color_file);

            k4a_image_release(color_image);
        }
        else
        {
            printf(" | Color None                       ");
        }

        // probe for a IR16 image
        image = k4a_capture_get_ir_image(capture);
        if (image != NULL)
        {
            /*printf(" | Ir16 res:%4dx%4d stride:%5d ",
                   k4a_image_get_height_pixels(image),
                   k4a_image_get_width_pixels(image),
                   k4a_image_get_stride_bytes(image));*/
            k4a_image_release(image);
        }
        else
        {
            printf(" | Ir16 None                       ");
        }
        // measure time it takes for depth frame since acquisition
        gettimeofday(&tv, NULL);
        milliSecondsSinceEpoch = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
        cJSON *depth_frame_ts = cJSON_CreateNumber(milliSecondsSinceEpoch);
        // Probe for a depth16 image
        depth_image = k4a_capture_get_depth_image(capture);
        short *input_img = (short *)(void *)k4a_image_get_buffer(depth_image);
        // initialize byte array
        char output_buffer[k4a_image_get_height_pixels(depth_image) * k4a_image_get_width_pixels(depth_image)];

        if (depth_image != NULL)
        {
            /*printf(" | Depth16 res:%4dx%4d stride:%5d\n",
                   k4a_image_get_height_pixels(depth_image),
                   k4a_image_get_width_pixels(depth_image),
                   k4a_image_get_stride_bytes(depth_image));*/

            cJSON *rvl_latency = NULL;
            cJSON *json_frame_num = NULL;
            cJSON *depth_frame_size = NULL;
            cJSON *total_delta_rvl = NULL;
            cJSON *is_depth = cJSON_CreateNumber(1);
            cJSON *depth_frame_json = cJSON_CreateObject();
            if (depth_frame_json == NULL)
            {
                printf("Error creating json file\n");
                goto Exit;
            }
            cJSON_AddItemToArray(measurements, depth_frame_json); // add object instance to json array
            // start latency timer
            clock_t t;
            t = clock();
            int diff_bytes = CompressRVL(input_img,
                                         output_buffer,
                                         k4a_image_get_height_pixels(depth_image) *
                                             k4a_image_get_width_pixels(depth_image),
                                         depth_image,
                                         roi,
                                         SIZEOF(roi),
                                         depth_frame_json);
            // end latency timer
            t = clock() - t; // timer difference
            double time_taken = ((double)t) / CLOCKS_PER_SEC;
            rvl_latency = cJSON_CreateNumber(time_taken);
            // char output_json_fn[36]; // save to disk
            // snprintf(output_json_fn, sizeof(output_json_fn), "/home/meteor/Desktop/outputJSON.json");
            int32_t total_frame_size = (int32_t)sizeof(output_buffer);
            // printf("\nDifference in bytes after RVL:%d \n", diff_bytes);
            printf("Size of output buffer for transmission: %d, size from k4a: %ld, size of size_t %ld\n",
                   total_frame_size,
                   k4a_image_get_size(depth_image),
                   sizeof(size_t));
            // send(new_socket, &total_frame_size, sizeof(int32_t), 0);
            // printf("Size of header message %ld", sizeof(header_msg));

            // char *d_buff_index = output_buffer;
            // printf("\nCreated memory reference to buffer at %p\n, size of size_t %ld\n", d_buff_index,
            // sizeof(size_t));* /

            char *depth_img_pkt = (char *)malloc(total_frame_size + sizeof(int32_t));
            printf("\nTotal frame size allocated is %ld\n", (total_frame_size + sizeof(int32_t)));
            memcpy(depth_img_pkt, &total_frame_size, sizeof(int32_t));
            memcpy(depth_img_pkt + sizeof(int32_t), output_buffer, total_frame_size);
            int32_t parse_frame_size_d = *((int32_t *)depth_img_pkt);
            printf("\n Parsed frame size is %d\n", parse_frame_size_d);
            send(new_socket, depth_img_pkt, total_frame_size + sizeof(int32_t), 0);

            // send(new_socket, output_buffer, total_frame_size, 0);
            free(depth_img_pkt);

            char path[37]; // save to disk
            snprintf(path, sizeof(path), "/home/meteor/Desktop/depth%d.bytes", frameNum);
            // printf("\nAttempt to write file to %s\n", path);
            FILE *file = fopen(path, "wb");
            fwrite(input_img,
                   sizeof(short),
                   k4a_image_get_height_pixels(depth_image) * k4a_image_get_width_pixels(depth_image),
                   file);
            fclose(file);

            depth_frame_size = cJSON_CreateNumber(total_frame_size);
            total_delta_rvl = cJSON_CreateNumber(diff_bytes);
            json_frame_num = cJSON_CreateNumber(frameNum);
            cJSON_AddItemToObject(depth_frame_json, "is_depth_frame", is_depth);
            cJSON_AddItemToObject(depth_frame_json, "depth_frame_size", depth_frame_size);
            cJSON_AddItemToObject(depth_frame_json, "total_delta_rvl", total_delta_rvl);
            cJSON_AddItemToObject(depth_frame_json, "rvl_latency", rvl_latency);
            cJSON_AddItemToObject(depth_frame_json, "frame_num", json_frame_num);
            cJSON_AddItemToObject(depth_frame_json, "depth_frame_ts", depth_frame_ts);
            k4a_image_release(depth_image);
            frameNum++;
            printf("Finish depth frame processes\n");
        }
        else
        {
            printf(" | Depth16 None\n");
        }

        // release capture
        k4a_capture_release(capture);
        fflush(stdout);
    }

    // write json measurements to disk
    json_file_str = cJSON_Print(monitor);
    if (json_file_str == NULL)
    {
        printf("Failed to create JSON file\n");
        goto Exit;
    }
    FILE *outputJSON = fopen("./outputJSON.json", "wb");
    fprintf(outputJSON, "%s\n", json_file_str);
    fclose(outputJSON);

    returnCode = 0;
Exit:
    if (device != NULL)
    {
        k4a_device_close(device);
    }

    return returnCode;
}
