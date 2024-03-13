#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "common.h"
#include "opencv.h"

using namespace cv;

int read_png_frames(void * buffer, const char* filename, int expected_buffer_size) 
{
    // Read the image
    Mat image = imread(filename);
    if (image.empty()) {
        PRINT_ERROR("Could not open or find the image %s.\n", filename);
        return FAILED;
    }
    
    // Convert the image to a buffer
    Mat flattened_image = image.reshape(1, 1);
    int buffer_size = flattened_image.total() * flattened_image.elemSize();

    if (true == IMAGE_FOLDER_IMR_DEBUG && true == g_customize.Image_Folder_Enable) 
    {
        Conv_RGB2YUYV((unsigned char*)flattened_image.data, (unsigned char*)buffer, g_customize.Frame_Width, g_customize.Frame_Height);
        return SUCCESS;
    } 
    
    if (buffer_size == expected_buffer_size) 
    {
        std::memcpy(buffer, flattened_image.data, buffer_size);
    } 
    else 
    {
        PRINT_ERROR("Buffer size mismatch: expected %d, got %d.\n", expected_buffer_size, buffer_size);
        return FAILED;
    }

    return SUCCESS;
}

//Used for debug only
int write_image(void * buffer, const char* filename, int height, int width) 
{
    if (buffer == nullptr || filename == nullptr) {
        PRINT_ERROR("Buffer is empty.\n");
        return FAILED;
    }

    if (height <= 0 || width <= 0) {
        PRINT_ERROR("Invalid Dimensions.\n");
        return FAILED;
    }

    Mat image(height, width, CV_8UC3, buffer);
    imwrite(filename, image);
    return SUCCESS;
}


void Conv_RGB2YUYV(unsigned char * bgr, unsigned char * yuyv, int width, int height)
{
    int z = 0;
    int x;
    int yline;

    for (yline = 0; yline < height; yline++)
    {
        for (x = 0; x < width; x++) 
        {
            int r = *bgr++;
            int g = *bgr++;
            int b = *bgr++;

            // Convert RGB to YUV
            int y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
            int u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
            int v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;

            // Clamp YUV values
            y = (y > 255) ? 255 : ((y < 0) ? 0 : y);
            u = (u > 255) ? 255 : ((u < 0) ? 0 : u);
            v = (v > 255) ? 255 : ((v < 0) ? 0 : v);

            // Pack YUV values into YUYV format
            if (z == 0) {
                yuyv[0] = (unsigned char)y;
                yuyv[1] = (unsigned char)u;
            } else {
                yuyv[2] = (unsigned char)y;
                yuyv[3] = (unsigned char)v;
                yuyv += 4;
            }

            z = !z; // Toggle z value
        }
    }
}