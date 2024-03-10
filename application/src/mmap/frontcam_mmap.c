#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
#include "common.h"
#include "frontcam_mmap.h"


int mmap_image_init() 
{   
    //gp_vin_out_buffer
    //g_customize, g_frame_width, g_frame_height, BPP_*, get_buffer_size() are all defined in the common.h or mmap.h file
    size_t size = get_buffer_size();
    mode_t old_umask = umask(0);
    mmap_file = open("image_buffer_out.dat", O_RDWR | O_CREAT | O_TRUNC, (mode_t)0666);
    umask(old_umask);

    if (mmap_file == -1) {
        PRINT_ERROR("Failed to open or create mmap file 'image_buffer_out.dat'\n");
        return FAILED; // 1
    }

    if (ftruncate(mmap_file, size) == -1) {
        PRINT_ERROR("Failed to set the size of mmap file 'image_buffer_out.dat'\n");
        close(mmap_file);
        mmap_file = -1;
        return FAILED; // 1
    }
    
    mapped_buffer_out = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mmap_file, 0);
    
    if (mapped_buffer_out == MAP_FAILED) {
        PRINT_ERROR("Failed to map mmap buffer to file 'image_buffer_out.dat'\n");
        close(mmap_file);
        mmap_file = -1;
        return FAILED; // 1
    }
    
    return SUCCESS; // 0
}

int mmap_deinit() {
    size_t size = get_buffer_size();
    
    if (munmap(mapped_buffer_out, size) != 0) {
        PRINT_ERROR("Error unmapping file 'image_buffer_out.dat'\n");
        close(mmap_file);
        return FAILED; // 1
    }
    
    if (mmap_file != -1) {
        if (close(mmap_file) != 0) {
            PRINT_ERROR("Failed to close mmap file 'image_buffer_out.dat'\n");
            return FAILED; // 1
        }
        mmap_file = -1;
    }
    
    return SUCCESS; // 0
}

int mmap_copy() {
    size_t size = get_buffer_size();
    if (gp_vin_out_buffer == NULL || mapped_buffer_out == NULL) {
        PRINT_ERROR("gp_vin_out_buffer or mapped_buffer_out is NULL\n");
        return FAILED; // 1
    }
    memcpy((void*)mapped_buffer_out, gp_vin_out_buffer, size);
    return SUCCESS; // 0
}