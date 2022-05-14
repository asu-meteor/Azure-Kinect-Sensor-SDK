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
#define PORT 11000
#define MAXLINE 1024

#define SIZEOF(a) sizeof(a) / sizeof(*a)

void func(int sockfd)
{
    char buff[80];
    int n;
    for (;;)
    {
        bzero(buff, sizeof(buff));
        printf("Enter the string : ");
        n = 0;
        while ((buff[n++] = getchar()) != '\n')
            ;
        // write(sockfd, buff, sizeof(buff));
        send(sockfd, &buff, sizeof(buff), 0);
        // bzero(buff, sizeof(buff));
        // // read(sockfd, buff, sizeof(buff));
        // printf("From Server : %s", buff);
        // if ((strncmp(buff, "exit", 4)) == 0)
        // {
        //     printf("Client Exit...\n");
        //     break;
        // }
    }
}

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

        if (color_image)
        {
            uint32_t *input_color_img = (uint32_t *)k4a_image_get_buffer(color_image);
            int32_t total_frame_size_color = (int32_t)k4a_image_get_size(color_image);
            char *color_img_pkt = (char *)malloc(total_frame_size_color + sizeof(int32_t));
            memcpy(color_img_pkt, &total_frame_size_color, sizeof(int32_t));
            memcpy(color_img_pkt + sizeof(int32_t), input_color_img, k4a_image_get_size(color_image));

            // send(new_socket, input_color_img, total_frame_size_color, 0);
            send(sockfd, color_img_pkt, total_frame_size_color + sizeof(int32_t), 0);
            free(color_img_pkt);
            k4a_image_release(color_image);
        }
        k4a_capture_release(capture);
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
