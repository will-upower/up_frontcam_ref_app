/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only
 * intended for use with Renesas products. No other uses are authorized. This
 * software is owned by Renesas Electronics Corporation and is protected under
 * all applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT
 * LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
 * TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS
 * ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE
 * FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR
 * ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE
 * BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software
 * and to discontinue the availability of this software. By using this software,
 * you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2022-2023 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : common.h
* Version      : 1.0.0
* Description  : header File
***********************************************************************************************************************/

/***********************************************************************************************************************
* History : DD.MM.YYYY  Version  Description
 *          06.03.2023  1.0.0    First Release
***********************************************************************************************************************/

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/
#ifndef COMMON_H_
#define COMMON_H_

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include <sys/time.h>
#include <stdio.h>

#include "rcar-xos/osal/r_osal.h"
#include "customize.h"
# include "rcar-xos/impfw/r_impfw.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/
#define OSAL_RESOURCE_ID      (0xf000U)
#define MUTEX_ID_NO1          (OSAL_RESOURCE_ID + 0U)
#define MUTEX_ID_NO2          (OSAL_RESOURCE_ID + 1U)
#define MUTEX_ID_NO3          ( OSAL_RESOURCE_ID + 2U )
#define MUTEX_ID_NO4          ( OSAL_RESOURCE_ID + 3U )
#define MUTEX_ID_NO5          ( OSAL_RESOURCE_ID + 4U )
#define MUTEX_ID_NO6          ( OSAL_RESOURCE_ID + 5U )
#define MUTEX_ID_NO7          ( OSAL_RESOURCE_ID + 6U )

#define MUTEX_ID_NO8          ( OSAL_RESOURCE_ID + 7U )
#define MUTEX_ID_NO9          ( OSAL_RESOURCE_ID + 8U )

#define FRAME_WIDTH           (1280)
#define FRAME_HEIGHT          (720)
#define DISPLAY_WIDTH         (1920)
#define DISPLAY_HEIGHT        (1080)
#define MULTIPLANE_SCREEN_WIDTH      (960)
#define MULTIPLANE_SCREEN_HEIGHT     (540)
#define LOG_SCREEN_WIDTH      (640)
#define LOG_SCREEN_HEIGHT     (1080)
#define GRAPH_HEIGHT          (360)
#define TIMEOUT_MS            (10000)                         // 10000 milisecond
#define TIMEOUT_2_S           (2000) 
#define TIMEOUT_5_S           (5000) 
#define TIMEOUT_1_S           (1000)
#define TIMEOUT_1MS_SLEEP     (1)                             // 1 milisecond
#define TIMEOUT_5MS_SLEEP     (5)
#define TIMEOUT_10MS_SLEEP    (10)                            // 10 milisecond
#define TIMEOUT_15MS_SLEEP    (15)                            // 15 milisecond
#define TIMEOUT_20MS_SLEEP    (20)                            // 20 milisecond
#define TIMEOUT_25MS_SLEEP    (25)                            // 25 milisecond
#define TIMEOUT_50MS_SLEEP    (50)
#define TIMEOUT_MS_WAIT       (5)
#define DATA_LEN_64           (64)
#define DATA_LEN_128          (128)
#define DATA_LEN_256          (256)
#define BPP_YUV               (2)
#define BPP_RGB               (3)
#define BPP_Y10               (4)
#define BPP_Y                 (1)
#define NUM_CNN_CHANNELS      (3)
#define SUCCESS               (0)
#define FAILED                (1)
#define DISABLE               (0)
#define IMAGE_LIST            ("List.txt")
#define OUTPUT_BUFFER         ("Output_Buffer")
#define INVALID               (-1)
#define IMPDEMO_OK            (0)
#define IMPDEMO_NG            (-1)
#define YUYV                  (0)
#define UYVY                  (1)
#define RGB                   (2)
#define Y10                   (4)

#if (RCAR_V4H)
#define CNN_CYCLE_COUNT_MAX   (800000)                      /* CNN Load Cycle count max value for V4H SoC*/
#else
#define CNN_CYCLE_COUNT_MAX   (533000)                      /* CNN Load Cycle count max value for V3H2 SoC*/
#endif

/* message queue IDs */
#define MQ_ID_NO1               OSAL_RESOURCE_ID + 0U
#define MQ_ID_NO2               OSAL_RESOURCE_ID + 1U

/* message queue config */
#define MAX_NUM_OF_MSG 1;
#define MSG_SIZE       4;

#define N_DEVS_MAX            (2)
/* condition variable IDs */
#define COND_ID_NO1             OSAL_RESOURCE_ID + 0U
#define COND_ID_NO2             OSAL_RESOURCE_ID + 1U
#define COND_ID_NO3             OSAL_RESOURCE_ID + 2U

#define PRINT_ERROR(...)      { printf("ERROR: %s (%d): ", __func__, __LINE__); printf(__VA_ARGS__);}
#define PRINT_WARNING(...)    { printf("WARNING: "); printf(__VA_ARGS__);}
#define PRINT_INFO(...)       { printf("INFO: "); printf(__VA_ARGS__);}
#define DEBUG_PRINT(...)      { if (true == g_customize.Debug_Enable ) { printf("DEBUG: %s(%d):", __func__, __LINE__); printf(__VA_ARGS__);} }
#define OSAL_SAMPLE_ERR(...)  {printf("error: %s(%d):", __func__, __LINE__); printf(__VA_ARGS__);}
#define IMPFW                  (1)
#define AI_SYNC_ENABLE         (1)
#define NUM_OUT_BUFF           (4)
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define AI_BUF_WIDTH    (256)
#define AI_BUF_HEIGHT   (256)
#define MAX_LEN_IMG_BUFF         (256)

/*Semantic Segmentation*/
#define SEM_SEG_IMG_WIDTH       256
#define SEM_SEG_IMG_HEIGHT      256
#define SEM_SEG_IMG_CHANNEL     3
#define SEM_SEG_OUTPUT_LEN      (SEM_SEG_IMG_WIDTH * SEM_SEG_IMG_HEIGHT * SEM_SEG_IMG_CHANNEL)

/*Pose Estimation*/
#define POSE_EST_IMG_WIDTH       224
#define POSE_EST_IMG_HEIGHT      224
#define POSE_EST_IMG_CHANNEL     3
#define POSE_EST_OUTPUT_LEN      (POSE_EST_IMG_WIDTH * POSE_EST_IMG_HEIGHT * POSE_EST_IMG_CHANNEL)

#define POSE_EST_INFERENCE_WIDTH    28
#define POSE_EST_INFERENCE_HEIGHT   28
#define POSE_EST_HEATMAP_CHANNEL    19
#define POSE_EST_PAF_CHANNEL        38
#define POSE_EST_CHANNEL             1


#define OBJ_DET_OUTPUT_LEN      (6300 * 32)
#define OBJ_DET_OUT_COLOUMN     (6300)
#define OBJ_DET_OUT_ROW         (32)

//new
#define MMAP_OUT "mmap_frontcam_out.dat"
#define MMAP_IN "mmap_frontcam_in.dat"
/**********************************************************************************************************************
 Typedef definitions
 *********************************************************************************************************************/
typedef struct st_cnn_channel_details
{
    void *       p_virt;
    size_t       width;
    size_t       height;
    unsigned int bytesPerPixel;
    unsigned int channel;

}st_cnn_channel_details_t;

typedef struct st_attr_inithandels
{
    impfw_attr_handle_t  impfw_imp0attr;
}st_attr_inithandels_t;

typedef struct
{
    int status;
    int is_enable;
}st_modulestatus_t;

typedef struct 
{
    st_modulestatus_t imr_ldc;
    st_modulestatus_t imr_rs;
    st_modulestatus_t isp;
    st_modulestatus_t vin;
    st_modulestatus_t vout;
    st_modulestatus_t cdnn;
}st_systemstatus_t;

typedef enum e_fc_module
{
    eVIN = 0,
    eIMR_LDC,
    eIMR_RS,
    eAI,
    eFC_DRAW,
    eVOUT,
}e_fc_module_t;

typedef enum e_fc_sem_seg
{
    eLANE = 1,
    eROAD,
    eNOROAD,
}e_fc_sem_seg_t;

typedef enum{
    IMR_RS_CH0 = 0,
    IMR_RS_CH1,
    IMR_RS_CH2,
    IMR_RS_CH3
}model_name;

struct cordinate
{
    float x;
    float y;
};

struct bbox_cor
{
    struct cordinate start;
    struct cordinate end;
    uint32_t cls;
};

typedef struct
{
    uint32_t bbox_arr_count;
    struct   bbox_cor bbox_arr[30];
    
}st_obj_det;

extern struct bbox_cor  _bbox_cor[30];
extern int detObjCount;

/**********************************************************************************************************************
 Exported global variables and functions
 *********************************************************************************************************************/
extern st_systemstatus_t g_fcStatus;
extern int64_t g_frame_width;
extern int64_t g_frame_height;
extern int g_det_accuracy;
extern unsigned char * gp_vin_out_buffer;
extern unsigned char * gp_high_res_buffer;
extern unsigned char * gp_imr_ldc_buffer;
extern unsigned char * gp_imr_rs_buffer_ch0;
extern unsigned char * gp_imr_rs_buffer_ch1;
extern unsigned char * gp_imr_rs_buffer_ch2;
extern unsigned char * gp_imr_rs_buffer_ch3;
extern unsigned char * gp_imr_rs_buffer_ch4;
extern unsigned char * gp_opencv_buffer;
extern unsigned char * gp_graph_buffer;
extern unsigned char * gp_log_buffer;
extern unsigned char * gp_ai_rgb_buffer;
extern unsigned char * gp_isp_in;
extern unsigned char * gp_imr_ldc_in;
extern unsigned char * gp_imr_res_in;
extern unsigned char * gp_cnn_in;
extern unsigned char * gp_opencv_in;
extern unsigned char * gp_yuv2rgb_in;
extern unsigned char * gp_vout_in;
extern unsigned char * gp_isp_buffer;
extern unsigned char * gp_image_buffer;
extern unsigned char * gp_in_buff_y;
extern unsigned char * gp_in_buff_uv;
extern unsigned char * gp_buf_yuv;
extern unsigned char * mul_plane_buffer_1;
extern unsigned char * mul_plane_buffer_2;
extern unsigned char * mul_plane_buffer_3;
extern unsigned char * mul_plane_buffer_4;
extern void * gp_isp_out_y;
extern void * gp_isp_out_uv;
extern void * gp_isp_out_rgb;
extern osal_memory_manager_handle_t g_handle_osalmmngr;
extern st_osal_mmngr_config_t g_osal_mmngr_config;
extern osal_axi_bus_id_t s_imp_dev_axi_bus_id;
extern osal_mutex_handle_t g_mtx_handle;
extern st_cnn_channel_details_t g_CnnChannelDetails[NUM_CNN_CHANNELS];
extern int64_t g_cnn_load_cnt;
extern unsigned int g_exe_time;

extern osal_mutex_handle_t g_mtx_handle_opencv;
extern osal_mutex_handle_t g_mtx_handle_vin_out;
extern osal_mutex_handle_t g_mtx_handle_vout;
extern osal_mutex_handle_t g_mtx_handle_imrrs;
extern osal_mutex_handle_t g_mtx_handle_imrldc;
extern osal_mutex_handle_t g_mtx_handle_isp;
extern osal_mutex_handle_t g_mtx_handle_yuv2rgb;
extern osal_mutex_handle_t g_mtx_handle_cnn;
extern osal_mutex_handle_t g_mtx_handle_imrldc_out;
extern osal_mutex_handle_t g_mtx_handle_imrrs_out;

extern osal_mutex_handle_t * gp_mtx_handle_vin;
extern osal_mutex_handle_t * gp_mtx_handle_opencv;
extern osal_mutex_handle_t * gp_mtx_handle_imr_ldc;
extern osal_mutex_handle_t * gp_mtx_handle_imr_rs;
extern osal_mutex_handle_t * gp_mtx_handle_isp;
extern osal_mutex_handle_t * gp_mtx_handle_vout;
extern osal_mutex_handle_t * gp_mtx_handle_yuv2rgb;
extern osal_mutex_handle_t * gp_mtx_handle_cnn;
extern osal_mutex_handle_t * gp_mtx_handle_imr_ldc_out;

extern osal_mutex_handle_t handle_osalmutex;
extern osal_mutex_handle_t handle_osalmutex_0;
extern osal_mutex_handle_t handle_osalmutex_1;
extern osal_mutex_handle_t handle_osalmutex_2;
extern osal_mutex_handle_t handle_osalmutex_3;

extern osal_cond_handle_t g_vin_cond_handle;
extern osal_cond_handle_t g_imr_rs_cond_handle;
extern osal_cond_handle_t g_imr_ldc_cond_handle;

extern osal_cond_handle_t * gp_vin_cond_handle;
extern osal_cond_handle_t * gp_imr_rs_cond_handle;
extern osal_cond_handle_t * gp_imr_ldc_cond_handle;
extern osal_cond_handle_t * gp_opencv_cond_handle;
extern osal_cond_handle_t * gp_vout_cond_handle;

extern int32_t g_imr_in_done;
extern int32_t g_imr_out_done;
extern int32_t g_ai_done;
extern int32_t g_vout_done;
extern double  g_cpu_usage;
extern bool    g_load_flag;
extern unsigned plane_count;
extern uint8_t g_vout_pix_fmt;
extern bool g_display_od_flag;
extern bool g_display_pos_flag;
extern bool g_display_ssg_flag;
extern bool g_display_depth_flag;
extern bool g_display_tsd_flag;

extern st_osal_mq_config_t g_mq_config_aiactivity;
extern osal_mq_handle_t    g_mq_handle_aiactivity; 
extern st_osal_mq_config_t g_mq_config_imgread;
extern osal_mq_handle_t    g_mq_handle_imgread; 

extern uint8_t model_num;
extern uint32_t g_hs_width;
extern uint32_t g_hs_height;

extern void Conv_YUYV2RGB (unsigned char * yuyv, unsigned char * bgr, int width, int height);
extern int g_fps[5];
extern void fpsCount(int dev);

/* VIN */
extern int R_VIN_Initilize (int VIN_Device,
                     int Frame_Width,
                     int Frame_Height,
                     int VIN_Offset_X,
                     int VIN_Offset_Y,
                     int VIN_Capture_Format,
                     int VIN_Req_Buffer_Num,
                     int Debug_Enable);
extern int R_VIN_Execute(int VIN_Capture_Format, int VIN_Device);
extern int R_VIN_Copy_Data (int VIN_Capture_Format, unsigned char *vin_out_buf, int VIN_Device);
extern int R_VIN_DeInitialize (int VIN_Device);
extern int R_Create_Image_List(char * Frame_Folder_Name);
/* ISP */
extern int R_ISP_Initialize ();
extern int R_ISP_Execute ();
extern int R_ISP_DeInitialize ();
/* IMR */
extern int R_IMR_Init ();
extern int R_IMR_SetupLDC ();
extern int R_IMR_SetupResize (void);
extern int R_IMR_ExecuteLDC (void);
extern int R_IMR_ExecuteResize (void);
extern int R_IMR_AllocImageLDC (void);
extern int R_IMR_AllocImageResize (void);
extern int init_mmgr (void);
extern int R_IMR_Deinit (void);
extern int imrdrv_wait_ch2();
extern int imrdrv_wait_ch3();
/* color conv */
extern int y_uv2yuyv (char * pdst, char * py, char * puv, int width, int height);
extern int conv_raw10_yuv8(void *p, char * pdst, int size);
extern void y_uv2yuyv_8(uint8_t *pdst, uint8_t *py, uint8_t *puv, int width, int height);
/* VOUT */
extern int vout_init ();
extern int execute ();
extern int64_t vout_deinit ();
extern int get_syncstatus(e_fc_module_t module, int flow);
extern int set_syncstatus(e_fc_module_t module, int flow);

/* OSAL */
extern int deinit_mmgr ();

#ifdef __cplusplus
extern "C"
{
#endif
int f_opencv_execute ();
int create_opencv_window();
void self_destruct_ocv();
int R_FC_SyncStart(e_fc_module_t module, osal_mutex_handle_t *ptr_mtx, osal_cond_handle_t *ptr_cond, int flow);
int R_FC_SyncEnd(e_fc_module_t module, osal_mutex_handle_t *ptr_mtx, osal_cond_handle_t *ptr_cond, int flow);

#ifdef __cplusplus
}
#endif

extern unsigned char sem_seg_array[];
extern float pe_array_heatmaps[];
extern float pe_array_pafs[];
extern uint32_t g_output_stride;

extern uint32_t g_output_buf_hwaddr_uv; 

extern bool g_is_thread_exit;

#endif /* COMMON_H_ */
