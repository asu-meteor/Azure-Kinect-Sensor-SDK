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
#include <netdb.h>
#include "rvl_compress.h"

#define SA struct sockaddr
#define PORT 11001
#define MAXLINE 1024

#define SIZEOF(a) sizeof(a) / sizeof(*a)

int FRAME_SIZE_HEADER_MSG;

int connect_to_server()
{
    int sockfd; // connfd;
    struct sockaddr_in servaddr;

    // socket create and verification
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
        printf("socket creation failed...\n");
        exit(0);
    }
    else
        printf("Socket successfully created..\n");
    bzero(&servaddr, sizeof(servaddr));

    // assign IP, PORT
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = inet_addr("192.168.0.91");
    servaddr.sin_port = htons(PORT);

    // connect the client socket to server socket
    if (connect(sockfd, (SA *)&servaddr, sizeof(servaddr)) != 0)
    {
        printf("connection with the server failed...\n");
        exit(0);
    }
    else
    {
        printf("connected to the server..\n");
        // open data comm link here
    }
    return sockfd;
}

int main(int argc, char **argv)
{
    int returnCode = 1;
    k4a_device_t device = NULL;
    const int32_t TIMEOUT_IN_MS = 1000;
    int captureFrameCount;
    int frame_number = 0;
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
    // start server connection
    int sockfd = connect_to_server();

    // get calibration
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("\nFailed to get calibration\n");
        goto Exit;
    }

    // char *hello = "hello from client";
    // send(sockfd, hello, strlen(hello), 0);

    while (captureFrameCount--)
    {
        k4a_image_t color_image;
        k4a_image_t depth_image;
        // char *color_img_pkt;
        // char *depth_img_pkt;
        char *tcp_packet;
        int32_t total_frame_size_color = 0;
        int32_t total_frame_size_depth = 0;
        // Get a depth frame
        switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            goto Exit;
        }

        color_image = k4a_capture_get_color_image(capture);
        depth_image = k4a_capture_get_depth_image(capture);

        if (color_image && depth_image)
        {
            uint32_t *input_color_img = (uint32_t *)k4a_image_get_buffer(color_image);
            total_frame_size_color = (int32_t)k4a_image_get_size(color_image);
            // color_img_pkt = (char *)malloc(total_frame_size_color + sizeof(int32_t));
            // memcpy(color_img_pkt, &total_frame_size_color, sizeof(int32_t));
            // memcpy(color_img_pkt + sizeof(int32_t), input_color_img, k4a_image_get_size(color_image));

            // depth image
            short *input_img = (short *)(void *)k4a_image_get_buffer(depth_image);
            // initialize byte array
            char output_buffer[k4a_image_get_height_pixels(depth_image) * k4a_image_get_width_pixels(depth_image)];
            total_frame_size_depth = (int32_t)sizeof(output_buffer);
            // depth_img_pkt = (char *)malloc(total_frame_size_depth + sizeof(int32_t));
            int diff_bytes = CompressRVL(input_img,
                                         output_buffer,
                                         k4a_image_get_height_pixels(depth_image) *
                                             k4a_image_get_width_pixels(depth_image));
            printf("Diff bytes %d, total frame size %d\n", diff_bytes, total_frame_size_depth);
            // memcpy(depth_img_pkt, &total_frame_size_depth, sizeof(int32_t));
            // memcpy(depth_img_pkt + sizeof(int32_t), output_buffer, total_frame_size_depth);
            printf("Consturcting tcp packet\n");
            // two frame headers, frame number, color wxh, depth wxh, along with color+depth frame data
            tcp_packet = (char *)malloc(total_frame_size_color + total_frame_size_depth + (sizeof(int32_t) * 7));

            int32_t c_width = k4a_image_get_width_pixels(color_image);
            int32_t c_height = k4a_image_get_height_pixels(color_image);
            int32_t d_width = k4a_image_get_width_pixels(depth_image);
            int32_t d_height = k4a_image_get_height_pixels(depth_image);
            printf("Frame number %d\n", frame_number);
            memcpy(tcp_packet, &frame_number, sizeof(int32_t));
            memcpy(tcp_packet + sizeof(int32_t), &c_width, sizeof(int32_t));
            memcpy(tcp_packet + (sizeof(int32_t) * 2), &c_height, sizeof(int32_t));
            memcpy(tcp_packet + (sizeof(int32_t) * 3), &total_frame_size_color, sizeof(int32_t));
            memcpy(tcp_packet + (sizeof(int32_t) * 4), input_color_img, total_frame_size_color);

            memcpy(tcp_packet + total_frame_size_color + (sizeof(int32_t) * 4), &frame_number, sizeof(int32_t));
            memcpy(tcp_packet + total_frame_size_color + (sizeof(int32_t) * 5), &d_width, sizeof(int32_t));
            memcpy(tcp_packet + total_frame_size_color + (sizeof(int32_t) * 6), &d_height, sizeof(int32_t));
            memcpy(tcp_packet + total_frame_size_color + (sizeof(int32_t) * 7),
                   &total_frame_size_depth,
                   sizeof(int32_t));
            memcpy(tcp_packet + total_frame_size_color + (sizeof(int32_t) * 8), output_buffer, total_frame_size_depth);
            // send(new_socket, input_color_img, total_frame_size_color, 0);
            size_t total_tcp_pkt_size = total_frame_size_color + total_frame_size_depth + (sizeof(int32_t) * 8);
            send(sockfd, tcp_packet, total_tcp_pkt_size, 0);
            free(tcp_packet);
        }
        k4a_image_release(color_image);
        k4a_image_release(depth_image);
        // release memory and capture
        // free(depth_img_pkt);
        k4a_capture_release(capture);
        frame_number++;
    }
    // printf("%d\n", sockfd);
    // close the socket
    close(sockfd);
Exit:
    if (device != NULL)
    {
        k4a_device_close(device);
    }
    return returnCode;
}
