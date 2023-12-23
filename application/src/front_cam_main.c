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
* File Name    : front_cam_main.c
* Version      : 1.0.0
* Description  : Main Application File
***********************************************************************************************************************/

/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 31.05.2023 1.00     First Release
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h> // MISRA
#include "rcar-xos/rcar_xos_config.h"
#include "common.h"
#include "customize.h"
#include "cpuload.h"
#if(CDNN)
#include "rcar-xos/ai_lib/ai_lib.h"
#include "cdnn_main.h"
#endif

/**********************************************************************************************************************
 Exported global variables and functions
 *********************************************************************************************************************/
unsigned char * gp_isp_in          = NULL;
unsigned char * gp_imr_ldc_in      = NULL;
unsigned char * gp_imr_res_in      = NULL;
unsigned char * gp_cnn_in          = NULL;
unsigned char * gp_opencv_in       = NULL;
unsigned char * gp_vout_in         = NULL;
unsigned char * gp_yuv2rgb_in      = NULL;
unsigned char * gp_vin_out_buffer  = NULL;
unsigned char * gp_high_res_buffer = NULL;
unsigned char * gp_isp_buffer      = NULL;
unsigned char * gp_image_buffer    = NULL;
unsigned char * gp_imr_ldc_buffer  = NULL;
unsigned char * gp_imr_rs_buffer_ch0   = NULL;
unsigned char * gp_imr_rs_buffer_ch1  = NULL;
unsigned char * gp_imr_rs_buffer_ch2  = NULL;
unsigned char * gp_imr_rs_buffer_ch3  = NULL;
unsigned char * gp_imr_rs_buffer_ch4  = NULL;
unsigned char * gp_opencv_buffer   = NULL;
unsigned char * gp_ai_rgb_buffer   = NULL;
unsigned char * gp_graph_buffer    = NULL;
unsigned char * gp_log_buffer      = NULL;
unsigned char * mul_plane_buffer_1 = NULL;
unsigned char * mul_plane_buffer_2 = NULL;
unsigned char * mul_plane_buffer_3 = NULL;
unsigned char * mul_plane_buffer_4 = NULL;

unsigned char * gp_in_buff_y;
unsigned char * gp_in_buff_uv;
unsigned char * gp_buf_yuv;

int     g_det_accuracy              = 0;
int64_t g_frame_width               = 0;
int64_t g_frame_height              = 0;
uint32_t g_hs_width                 = 0;
uint32_t g_hs_height                = 0;
uint8_t g_vout_pix_fmt              = 0;
char buffer[DATA_LEN_256];

uint32_t us_VOUT                    = 0;

unsigned int g_exe_time             = 0;
int64_t      g_cnn_load_cnt         = 0;


bool g_is_thread_exit               = false;
bool g_load_flag                    = false;
bool g_display_od_flag              = true;
bool g_display_ssg_flag             = true;
bool g_display_pos_flag             = true;
bool g_display_tsd_flag             = true;
bool g_display_depth_flag           = false;

st_customize_t g_customize;
double g_cpu_usage;

osal_mutex_handle_t g_mtx_handle  = OSAL_MUTEX_HANDLE_INVALID;

/*Message Queue Params*/
st_osal_mq_config_t g_mq_config_aiactivity;
osal_mq_handle_t    g_mq_handle_aiactivity = OSAL_MQ_HANDLE_INVALID;

st_osal_mq_config_t g_mq_config_imgread;
osal_mq_handle_t    g_mq_handle_imgread = OSAL_MQ_HANDLE_INVALID;

void * gp_isp_out_y;
void * gp_isp_out_uv;
void * gp_isp_out_rgb;

st_systemstatus_t g_fcStatus;

osal_mutex_handle_t g_mtx_handle_opencv       = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_vin_out      = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_vout         = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_imrrs        = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_imrldc       = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_isp          = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_yuv2rgb      = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_cnn          = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_imrldc_out   = OSAL_MUTEX_HANDLE_INVALID;
osal_mutex_handle_t g_mtx_handle_imrrs_out    = OSAL_MUTEX_HANDLE_INVALID;

osal_mutex_handle_t * gp_mtx_handle_vin         = NULL;
osal_mutex_handle_t * gp_mtx_handle_opencv      = NULL;
osal_mutex_handle_t * gp_mtx_handle_imr_ldc     = NULL;
osal_mutex_handle_t * gp_mtx_handle_imr_rs      = NULL;
osal_mutex_handle_t * gp_mtx_handle_isp         = NULL;
osal_mutex_handle_t * gp_mtx_handle_vout        = NULL;
osal_mutex_handle_t * gp_mtx_handle_cnn         = NULL;
osal_mutex_handle_t * gp_mtx_handle_yuv2rgb     = NULL;
osal_mutex_handle_t * gp_mtx_handle_imr_ldc_out = NULL;

osal_cond_handle_t g_vin_cond_handle            = OSAL_COND_HANDLE_INVALID;
osal_cond_handle_t g_imr_rs_cond_handle         = OSAL_COND_HANDLE_INVALID;
osal_cond_handle_t g_imr_ldc_cond_handle        = OSAL_COND_HANDLE_INVALID;

osal_cond_handle_t * gp_vin_cond_handle        = NULL;
osal_cond_handle_t * gp_imr_rs_cond_handle     = NULL;
osal_cond_handle_t * gp_imr_ldc_cond_handle    = NULL;
osal_cond_handle_t * gp_opencv_cond_handle     = NULL;
osal_cond_handle_t * gp_vout_cond_handle       = NULL;

int32_t g_imr_in_done     = 0;
int32_t g_imr_out_done    = 0;
int32_t g_ai_done         = 0;
int32_t g_vout_done       = 0;

int g_fps[5];
extern int R_VIN_Start_Capturing (int VIN_Device);
/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/
static int64_t R_FC_SystemInit ();
static int64_t buffer_map ();
static int64_t Vin_Buffer_Alloc();
static int64_t Isp_Buffer_Alloc();
static int64_t Imr_Buffer_Alloc();
static void ISP_inputcustom();
static void IMR_LDC_inputcustom();
static void IMR_Resize_inputcustom();
static void Color_Conversion_bufferCustom();
static void opencv_inputcustom();
static void vout_inputcustom();
static int64_t syncflow_enable(e_fc_module_t module);
static int64_t syncflow_disable(e_fc_module_t module);
static void    sigint_handler (int signum);
static void    FcModuleInitFlags ();
static inline unsigned long uSecElapsed (struct timeval *t2, struct timeval *t1);
static int st_r_vin_execute_main (void);
static int Opencv_buffer_alloc();

/**********************************************************************************************************************
 Function Declarations
*********************************************************************************************************************/
int64_t R_Init_Modules ();
int64_t R_Deinit_Modules ();
int64_t R_Capture_Task ();
int64_t R_VOUT_Task ();
int64_t R_CTRL_Task ();
int64_t R_Inference_Task ();
int64_t R_IMR_Task ();
int64_t R_CPULOAD_Task ();
int     f_opencv_execute ();
int     R_PipelineParamValidate();
int64_t R_Create_Mutex();
int64_t R_Mutex_Map();
int get_syncstatus (e_fc_module_t module, int flow);
int set_syncstatus (e_fc_module_t module, int flow);
int (* fp_syncstart) (e_fc_module_t , osal_mutex_handle_t *, osal_cond_handle_t *, int);
int (* fp_syncend) (e_fc_module_t , osal_mutex_handle_t *, osal_cond_handle_t *, int);
int R_FC_SyncStart (e_fc_module_t module, osal_mutex_handle_t *ptr_mtx, osal_cond_handle_t *ptr_cond, int flow);
int R_FC_SyncEnd (e_fc_module_t module, osal_mutex_handle_t *ptr_mtx, osal_cond_handle_t *ptr_cond, int flow);

/******************************************************************************************************************//**
/* Function Name : main */
/******************************************************************************************************************//**
 * @brief       main function
                [Covers: BPAP_FC_V4H_AD045][Covers: BPAP_FC_V4H_AD001][Covers: BPAP_FC_V4H_AD002]
                [Covers: BPAP_FC_V4H_AD007][Covers: BPAP_FC_V4H_AD009][Covers: BPAP_FC_V4H_AD055]
                [Covers: BPAP_FC_V4H_AD011][Covers: BPAP_FC_V4H_AD012][Covers: BPAP_FC_V4H_AD015]
                [Covers: BPAP_FC_V4H_AD017][Covers: BPAP_FC_V4H_AD024][Covers: BPAP_FC_V4H_AD026]
                [Covers: BPAP_FC_V4H_AD027][Covers: BPAP_FC_V4H_AD033][Covers: BPAP_FC_V4H_AD034]
                [Covers: BPAP_FC_V4H_AD047][Covers: BPAP_FC_V4H_AD058]
 * @param[in]   argc                        unused  
 * @param[in]   argv                        unused
 * @param[out]  none
 * @retval      true                        success
 * @retval      false                       fail
***********************************************************************************************************************/
int main(int argc, char * argv[]) 
{
    (void)argc; /* unused */
    (void)argv; /* unused */
    int ret;
    fp_syncstart = R_FC_SyncStart;
    fp_syncend = R_FC_SyncEnd;
    FcModuleInitFlags();                            /* Initialize flags */

    do 
    {
        signal(SIGINT, sigint_handler);
        ret = R_CustomizeLoad(&g_customize, FC_CustomizeFile);

        if (FAILED == ret)
        {
            PRINT_INFO("Cannot find a customize file. customization paramters are used default values\n");
            R_CustomizeInit(&g_customize);           /* Initialize customization parameters */
        }
#if (CDNN)
        if ((true == g_customize.CDNN_Load_Enable) || (true == g_customize.CPU_Load_Enable))     /* Graph Display */
#else
        if (true == g_customize.CPU_Load_Enable)     /* Graph Display */
#endif
        {
            g_customize.Frame_Width         = FRAME_WIDTH;
            g_customize.Frame_Height        = FRAME_HEIGHT;
            g_customize.VOUT_Pos_X          = 0;
            g_customize.VOUT_Pos_Y          = 0;
            g_customize.VOUT_Display_Width  = DISPLAY_WIDTH;
            g_customize.VOUT_Display_Height = DISPLAY_HEIGHT;
        }

        if (true == g_customize.MMAP_Contingency_enable)  /* Disable native CDNN/CNN Functionality */
        {
            g_customize.CDNN_Enable = 0;
            g_customize.OBJ_DET_Enable = 0;
            g_customize.SEM_SEG_Enable = 0;
            g_customize.POSE_EST_Enable = 0;
        }

        R_CustomizePrint(&g_customize);            /* Print customization parameters */

        ret = R_CustomizeValidate(&g_customize);   /* Customization parameter validation */
        if (FAILED == ret)
        {
            PRINT_ERROR("Failed R_CustomizeValidate \n");
            break;
        }

        ret = R_PipelineParamValidate();   /* Customization parameter validation */
        if (ret != SUCCESS)
        {
            PRINT_WARNING("Change the customization parameters in front_cam_customize.config file and \
re-run the application\n FC App terminating...\n ");
            break;
        }



        if (true == g_customize.Image_Folder_Enable)     /* Enabled to read image from folder */
        {
            ret = R_Create_Image_List(g_customize.Frame_Folder_Name);                 /* image list creation */
            if (ret != SUCCESS)
            {
                PRINT_ERROR("Failed R_Create_Image_List\n");
                break;
            }
        }

        ret = R_FC_SystemInit();                        /* System initialization */

        if (FAILED == ret)
        {
            PRINT_ERROR("R_FC_SystemInit failed\n");
            break;
        }

        ret = buffer_map();                              /* Mapping IN/OUT buffers */

        if (FAILED == ret)
        {
            PRINT_ERROR("buffer_map failed\n");
            break;
        }
        
        e_osal_return_t osal_ret;

        e_osal_return_t ret_osal;
        osal_ret = R_OSAL_Initialize();                  /* OSAL Initialize */

        if (OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("OSAL Initialization failed with error %d\n", osal_ret);
            break;
        }

        ret = init_mmgr();                              /* Memory manager initialisation */

        if (ret)
        {
            ret_osal = R_OSAL_Deinitialize();           /* OSAL deinitialize if failed */
            if (OSAL_RETURN_OK != ret_osal)
            {
                PRINT_ERROR("Failed R_OSAL_Deinitialize ret=%d\n", ret_osal);
            }
            break;
        }

        ret = R_Init_Modules();                         /* Initialize modules */

        if (FAILED == ret)
        {
       
            R_Deinit_Modules();                         /* Deinitialize modules if failed */
            PRINT_ERROR("Failed R_Init_Modules ret\n");
            break;
        }
    
        ret = R_Create_Mutex();                         /* Create mutex */

        if (FAILED == ret)
        {
       
            R_Deinit_Modules();                         /* Deinitialize modules if failed */
            PRINT_ERROR("Failed R_Create_Mutex ret\n");
            break;
        }

        ret = R_Mutex_Map();                            /* Mutex mapping */

        if (FAILED == ret)
        {
       
            R_Deinit_Modules();                         /* Deinitialize modules if failed */
            PRINT_ERROR("Failed R_Mutex_Map ret\n");
            break;
        }

        /* Condition handle create */
        /* vin condition handle create */
        osal_ret = R_OSAL_ThsyncCondCreate((osal_cond_id_t)COND_ID_NO1, &g_vin_cond_handle);

        if (OSAL_RETURN_OK != osal_ret)
        {
            R_Deinit_Modules();
            PRINT_ERROR("condition creation was failed, osal_ret = %d\n", osal_ret);
            break;
        }
        /* IMR LDC condition handle create */
        osal_ret = R_OSAL_ThsyncCondCreate((osal_cond_id_t)COND_ID_NO2, &g_imr_ldc_cond_handle);

        if (OSAL_RETURN_OK != osal_ret)
        {
            R_Deinit_Modules();
            PRINT_ERROR("condition creation was failed, osal_ret = %d\n", osal_ret);
            break;
        }
        /* IMR resize condition handle create */
        osal_ret = R_OSAL_ThsyncCondCreate((osal_cond_id_t)COND_ID_NO3, &g_imr_rs_cond_handle);

        if (OSAL_RETURN_OK != osal_ret)
        {
            R_Deinit_Modules();
            PRINT_ERROR("condition creation was failed, osal_ret = %d\n", osal_ret);
            break;
        }

        /*Configuration for the Message Queue for driver activity */
        osal_ret = R_OSAL_MqInitializeMqConfigSt(&g_mq_config_aiactivity);

        if (OSAL_RETURN_OK != osal_ret)
        {
            R_Deinit_Modules();
            PRINT_ERROR("Message Queue Initialization failed for driver activity with error %d\n", osal_ret);
            break;
        }
        else
        {
            g_mq_config_aiactivity.max_num_msg = MAX_NUM_OF_MSG;
            g_mq_config_aiactivity.msg_size = MSG_SIZE;
            osal_ret = R_OSAL_MqCreate(&g_mq_config_aiactivity, (osal_mq_id_t)MQ_ID_NO2, &g_mq_handle_aiactivity);
            if (OSAL_RETURN_OK != osal_ret)
            {
                R_Deinit_Modules();
                PRINT_ERROR("message queue creation was failed, osal_ret = %d\n", osal_ret);
                break;
            }
        }

        if (true == g_customize.Image_Folder_Enable) /* Configure message queue when image read from folder enabled */
        {
            osal_ret = R_OSAL_MqInitializeMqConfigSt(&g_mq_config_imgread);
            
            if (OSAL_RETURN_OK != osal_ret)
            {
                R_Deinit_Modules();
                PRINT_ERROR("Message Queue Initialization failed for image read from folder with error %d\n", osal_ret);
                break;
            }
            else
            {
                g_mq_config_imgread.max_num_msg = MAX_NUM_OF_MSG;                     /* maximum number of messages */
                g_mq_config_imgread.msg_size = MSG_SIZE;
                osal_ret = R_OSAL_MqCreate(&g_mq_config_imgread, (osal_mq_id_t)MQ_ID_NO1, &g_mq_handle_imgread);
                
                if (OSAL_RETURN_OK != osal_ret)
                {
                    R_Deinit_Modules();
                    PRINT_ERROR("message queue creation was failed, osal_ret = %d\n", osal_ret);
                    break;
                }
            }
        }
        // Addendum: opencv window:
        if (0 == g_customize.VOUT_Enable) {
            create_opencv_window();
            atexit(self_destruct_ocv);
        }
        /* Create Capture thread */
        osal_thread_handle_t    capture_thrd_hndl         = OSAL_THREAD_HANDLE_INVALID;
        int64_t                 thrd_return_value         = -1;
        st_osal_thread_config_t capture_thrd_cfg;
        capture_thrd_cfg.func       = R_Capture_Task;
        capture_thrd_cfg.priority   = OSAL_THREAD_PRIORITY_TYPE1;
        capture_thrd_cfg.stack_size = 0x2000;
        capture_thrd_cfg.task_name  = "R_Capture_Task";
        capture_thrd_cfg.userarg    = NULL;

        /* Create IMR thread */
        osal_thread_handle_t    imr_thrd_hndl         = OSAL_THREAD_HANDLE_INVALID;
        int64_t                 imr_thrd_return_value = -1;
        st_osal_thread_config_t imr_thrd_cfg;
        imr_thrd_cfg.func       = R_IMR_Task;
        imr_thrd_cfg.priority   = OSAL_THREAD_PRIORITY_TYPE0;
        imr_thrd_cfg.stack_size = 0x2000;
        imr_thrd_cfg.task_name  = "R_IMR_Task";
        imr_thrd_cfg.userarg    = NULL;
        
        /* Create CNN thread */
        osal_thread_handle_t    ai_thrd_hndl         = OSAL_THREAD_HANDLE_INVALID;
        int64_t                 ai_thrd_return_value = -1;
        st_osal_thread_config_t ai_thrd_cfg;
        ai_thrd_cfg.func       = R_Inference_Task;
        ai_thrd_cfg.priority   = OSAL_THREAD_PRIORITY_TYPE0;
        ai_thrd_cfg.stack_size = 0x200000;
        ai_thrd_cfg.task_name  = "R_Inference_Task";
        ai_thrd_cfg.userarg    = NULL;

        /* Create Vout thread */
        osal_thread_handle_t    vout_thrd_hndl         = OSAL_THREAD_HANDLE_INVALID;
        int64_t                 vout_thrd_return_value = -1;
        st_osal_thread_config_t vout_thrd_cfg;
        vout_thrd_cfg.func       = R_VOUT_Task;
        vout_thrd_cfg.priority   = OSAL_THREAD_PRIORITY_TYPE0;
        vout_thrd_cfg.stack_size = 0x2000;
        vout_thrd_cfg.task_name  = "R_VOUT_Task";
        vout_thrd_cfg.userarg    = NULL;

        /* Create control thread */
        osal_thread_handle_t    ctrl_thrd_hndl         = OSAL_THREAD_HANDLE_INVALID;
        int64_t                 ctrl_thrd_return_value = -1;
        st_osal_thread_config_t ctrl_thrd_cfg;
        ctrl_thrd_cfg.func       = R_CTRL_Task;
        ctrl_thrd_cfg.priority   = OSAL_THREAD_PRIORITY_TYPE0;
        ctrl_thrd_cfg.stack_size = 0x2000;
        ctrl_thrd_cfg.task_name  = "R_CTRL_Task";
        ctrl_thrd_cfg.userarg    = NULL;

        /* Create CPU thread */
        osal_thread_handle_t    cpu_thrd_hndl         = OSAL_THREAD_HANDLE_INVALID;
        int64_t                 cpu_thrd_return_value = -1;
        st_osal_thread_config_t cpu_thrd_cfg;
        cpu_thrd_cfg.func       = R_CPULOAD_Task;
        cpu_thrd_cfg.priority   = OSAL_THREAD_PRIORITY_TYPE0;
        cpu_thrd_cfg.stack_size = 0x2000;
        cpu_thrd_cfg.task_name  = "R_CPULOAD_Task";
        cpu_thrd_cfg.userarg    = NULL;

        /* START_THREAD: Thread starts */

        /* Start control Thread */
        osal_ret = R_OSAL_ThreadCreate(&ctrl_thrd_cfg, 0xf003, &ctrl_thrd_hndl);
        if (OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("OSAL Control thread creation failed with error %d\n", osal_ret);
            R_Deinit_Modules();
            break;
        }

        /* Start Capture Thread */
        osal_ret = R_OSAL_ThreadCreate(&capture_thrd_cfg, 0xf000, &capture_thrd_hndl);
        if (OSAL_RETURN_OK != osal_ret)
        {
            g_is_thread_exit = true;
            PRINT_ERROR("OSAL capture thread creation failed with error %d\n", osal_ret);
            R_Deinit_Modules();
            break;
        }

        /* Start IMR Thread */
        if(true == g_customize.IMR_LDC || true == g_customize.IMR_Resize)
        {
            osal_ret = R_OSAL_ThreadCreate(&imr_thrd_cfg, 0xf004, &imr_thrd_hndl);
            if (OSAL_RETURN_OK != osal_ret)
            {
                PRINT_ERROR("OSAL IMR thread creation failed with error %d\n", osal_ret);
                R_Deinit_Modules();
                break;
            }
        }

#if(CDNN)
        if((true == g_customize.CDNN_Enable) || (true == g_customize.MMAP_Contingency_enable))
        {
            osal_ret = R_OSAL_ThreadCreate(&ai_thrd_cfg, 0xf001, &ai_thrd_hndl);
            if (OSAL_RETURN_OK != osal_ret)
            {
                g_is_thread_exit = true;
                PRINT_ERROR("OSAL inference thread creation failed with error %d\n", osal_ret);
                R_Deinit_Modules();
                break;
            }
        }
#endif
        /* Start CPU Thread */
#if (RCAR_V4H) && (CDNN)
        if ((true == g_customize.CPU_Load_Enable) || (true == g_customize.CDNN_Load_Enable))
#else
        if (true == g_customize.CPU_Load_Enable)
#endif
        {
            osal_ret = R_OSAL_ThreadCreate(&cpu_thrd_cfg, 0xf005, &cpu_thrd_hndl);
            if (OSAL_RETURN_OK != osal_ret)
            {
                g_is_thread_exit = true;
                PRINT_ERROR("OSAL cpu thread creation failed with error %d\n", osal_ret);
                R_Deinit_Modules();
                break;
            }
        }
        /* Start vout Thread */
        osal_ret = R_OSAL_ThreadCreate(&vout_thrd_cfg, 0xf002, &vout_thrd_hndl);
        if (OSAL_RETURN_OK != osal_ret)
        {
            g_is_thread_exit = true;
            PRINT_ERROR("OSAL vout thread creation failed with error %d\n", osal_ret);
            R_Deinit_Modules();
            break;
        }
        /* wait until capture thread */
        osal_ret = R_OSAL_ThreadJoin(capture_thrd_hndl, &thrd_return_value);
        if (OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("OSAL capture thread join failed with error %d\n", osal_ret);
            R_Deinit_Modules();
            break;
        }

        if(true == g_customize.IMR_LDC || true == g_customize.IMR_Resize)
        {
            osal_ret = R_OSAL_ThreadJoin(imr_thrd_hndl, &imr_thrd_return_value);
            if (OSAL_RETURN_OK != osal_ret)
            {
                PRINT_ERROR("OSAL imr thread join failed with error %d\n", osal_ret);
                R_Deinit_Modules();
                break;
            }
        }

        /*Wait until Inference Thread*/
#if (CDNN)
        if((true == g_customize.CDNN_Enable) || (true == g_customize.MMAP_Contingency_enable))

        {
            osal_ret = R_OSAL_ThreadJoin(ai_thrd_hndl, &ai_thrd_return_value);
            if (OSAL_RETURN_OK != osal_ret)
            {
                PRINT_ERROR("OSAL inference thread join failed with error %d\n", osal_ret);
                R_Deinit_Modules();
                break;
            }
        }
#endif

        /*Wait until CPU Thread*/
#if (CDNN)
        if ((true == g_customize.CPU_Load_Enable) || (true == g_customize.CDNN_Load_Enable))
#else
        if (true == g_customize.CPU_Load_Enable) 
#endif
        {
            osal_ret = R_OSAL_ThreadJoin(cpu_thrd_hndl, &cpu_thrd_return_value);
            if (OSAL_RETURN_OK != osal_ret)
            {
                PRINT_ERROR("OSAL cpu thread join failed with error %d\n", osal_ret);
                R_Deinit_Modules();
                break;
            }
        }

        /* wait until VOUT thread finished */
        osal_ret = R_OSAL_ThreadJoin(vout_thrd_hndl, &vout_thrd_return_value);
        if (OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("OSAL vout thread join failed with error %d\n", osal_ret);
            R_Deinit_Modules();
            break;
        }

        printf("Please press Enter to Exit \n");
        /* wait until control thread finished */
        osal_ret = R_OSAL_ThreadJoin(ctrl_thrd_hndl, &ctrl_thrd_return_value);
        if (OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("OSAL control thread join failed with error %d\n", osal_ret);
            R_Deinit_Modules();
            break;
        }

        /* De-Initialize */
        R_Deinit_Modules();
    }
    while (0);
    printf("FC App terminated successfully.\n");

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function main
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_Capture_Task */
/******************************************************************************************************************//**
 * @brief       Capture thread
                [Covers: BPAP_FC_V4H_AD005][Covers: BPAP_FC_V4H_AD015][Covers: BPAP_FC_V4H_AD016]
                [Covers: BPAP_FC_V4H_AD020][Covers: BPAP_FC_V4H_AD024][Covers: BPAP_FC_V4H_AD027]
                [Covers: BPAP_FC_V4H_AD035][Covers: BPAP_FC_V4H_AD036][Covers: BPAP_FC_V4H_AD040]
                [Covers: BPAP_FC_V4H_AD046][Covers: BPAP_FC_V4H_AD047][Covers: BPAP_FC_V4H_AD048]
                [Covers: BPAP_FC_V4H_AD008]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success
 * @retval      false           fail
***********************************************************************************************************************/
int64_t R_Capture_Task()
{
    int ret = INVALID;
    char image_name_buf[MAX_LEN_IMG_BUFF];
    char fname[MAX_LEN_IMG_BUFF];

    FILE * fp_list  = NULL;
    FILE * fp_Image = NULL;

    static int image_read_send_flag = 0;

    struct timeval  mod_starttime;
    struct timeval  mod_endtime;

    if (true == g_customize.Image_Folder_Enable)            /* Image read from folder enabled */
    {
        fp_list = fopen(IMAGE_LIST, "r");                    /* IMAGE_LIST file open */
        if (fp_list == NULL)
        {
            PRINT_ERROR("Could not open file\n");
            g_is_thread_exit = true;
            return FAILED;
        }
    }

    while (!g_is_thread_exit)
    {
        if (true == g_customize.VIN_Enable)                 /* Camera capture enabled */
        {
            ret = st_r_vin_execute_main();                  /* Execution of VIN */

            if (FAILED == ret)
            {
                PRINT_ERROR("Failed R_VIN_Execute \n");
                g_is_thread_exit = true;
                return FAILED;
            }
        }
        else
        {
            if (true == g_customize.Image_Folder_Enable)       /* Image read from folder enabled */
            {
                if (fgets(fname, MAX_LEN_IMG_BUFF, fp_list))
                {
                    /* sending message to image read message queue */
                    e_osal_return_t osal_ret = R_OSAL_MqSendForTimePeriod(g_mq_handle_imgread, TIMEOUT_MS, 
                                                 (void *)&image_read_send_flag, g_mq_config_imgread.msg_size);
                    if (OSAL_RETURN_OK != osal_ret)
                    {
                        PRINT_ERROR("sending message to image read MQ was failed, osal_ret = %d\n", osal_ret);
                    }
                    sscanf(fname, "%s", buffer);
                    sprintf(image_name_buf, "%s/%s", g_customize.Frame_Folder_Name, buffer);
                    fp_Image = fopen(image_name_buf, "rb");
                    if(fp_Image == NULL)
                    {
                        PRINT_ERROR("Could not open file\n");
                        g_is_thread_exit = true;
                        return FAILED;
                    }
                    R_FC_SyncStart(eVIN, &g_mtx_handle_vin_out, &g_vin_cond_handle, 1);
                    fread(gp_vin_out_buffer, sizeof(unsigned char), g_frame_height * g_frame_width * BPP_YUV, fp_Image);
                    R_FC_SyncEnd(eVIN, &g_mtx_handle_vin_out, &g_vin_cond_handle, 1);
                    fclose(fp_Image);
                }
                else
                {
                    fclose(fp_list);
                    printf("Read Image from folder completed\n");
                    if (remove(IMAGE_LIST) == 0)                          /* Remove the image list */
                    {
                        printf("The file is deleted successfully\n");
                    } else 
                    {
                        PRINT_ERROR("The file is not deleted\n");
                    }
                    g_is_thread_exit = true;
                    return SUCCESS;
                }

                R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_25MS_SLEEP);             /* Thread sleep */
            }
            else if(false == g_customize.ISP_Enable)
            {
                R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_50MS_SLEEP);
            }
        }


         if (true == g_customize.ISP_Enable)
         {
             ret = R_ISP_Execute();
             if (FAILED == ret)
             {
                 PRINT_ERROR("Failed R_ISP_Execute \n");
                 g_is_thread_exit = true;
                 return FAILED;
             }

             if (true == g_customize.ISP_RAW_OUT_Format)
             {
                 memcpy((void *)gp_isp_buffer, (void *)gp_isp_out_rgb , (size_t) g_frame_width * g_frame_height * BPP_RGB);
             }
             else
             {
 #if (!RCAR_V4H)
                 R_FC_SyncStart(eVIN, &g_mtx_handle_vin_out, &g_vin_cond_handle, 1);
                 ret = y_uv2yuyv(gp_isp_buffer, (char *)gp_isp_out_y, (char *)gp_isp_out_uv, g_frame_width, g_frame_height);
                 if (FAILED == ret)
                 {
                     PRINT_ERROR("Failed y_uv2yuyv Conversion \n");
                     g_is_thread_exit = true;
                     return FAILED;
                 }
                 R_FC_SyncEnd(eVIN, &g_mtx_handle_vin_out, &g_vin_cond_handle, 1);
 #endif
             }
         }
    }

    return SUCCESS;
}

/**********************************************************************************************************************
 End of function R_Capture_Task
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_IMR_Task */
/******************************************************************************************************************//**
 * @brief       IMR Thread
                [Covers: BPAP_FC_V4H_AD011][Covers: BPAP_FC_V4H_AD017][Covers: BPAP_FC_V4H_AD023]
                [Covers: BPAP_FC_V4H_AD024][Covers: BPAP_FC_V4H_AD028][Covers: BPAP_FC_V4H_AD037]
                [Covers: BPAP_FC_V4H_AD049]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success
 * @retval      false           fail
 *********************************************************************************************************************/
int64_t R_IMR_Task()
{
    int ret;

    R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_50MS_SLEEP);
    while (!g_is_thread_exit)
    {

        if (true == g_customize.IMR_LDC)
        {
            ret = R_IMR_ExecuteLDC();
            if (FAILED == ret)
            {
                PRINT_ERROR("Failed R_IMR_ExecuteLDC \n");
                g_is_thread_exit = true;
                return FAILED;
            }
        }

        if (true == g_customize.IMR_Resize)
        {
            ret = R_IMR_ExecuteResize();                  /* Execution of IMR Resize */
            if (FAILED == ret)
            {
                PRINT_ERROR("Failed R_IMR_ExecuteResize \n");
                g_is_thread_exit = true;
                return FAILED;
            }
        }
    }
    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_IMR_Task
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_CPULOAD_Task */
/******************************************************************************************************************//**
 * @brief       CPU Thread
                [Covers: BPAP_FC_V4H_AD030]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success
 * @retval      false           fail
 *********************************************************************************************************************/
int64_t R_CPULOAD_Task()
{
    int ret;
    int pid_temp = getpid();
    st_cpuload load_0;
    st_cpuload load_1;

    R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_50MS_SLEEP);
    
    if (true == g_customize.CPU_Load_Enable)                        /* If CPU Load is enabled */
    {
        while (!g_is_thread_exit)
        {
            R_CPU_Getstats(&load_0, &pid_temp);                     /* Get status of load 0 */
            R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_1_S);
            R_CPU_Getstats(&load_1, &pid_temp);                     /* Get status of load 1 */
            g_cpu_usage = R_CPU_CalculateLoad(&load_0, &load_1);    /* Calculate CPU load */
            DEBUG_PRINT("CPU: %lf%%\n", g_cpu_usage);               /* Print CPU load */
            g_load_flag = true;
        }
    }
    else                                                            /* If CPU Load is disabled */
    {
        while (!g_is_thread_exit)
        {
            R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_1_S);
            g_load_flag = true;
        }
    }
    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_CPULOAD_Task
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_VOUT_Task */
/******************************************************************************************************************//**
 * @brief       Display Thread
                [Covers: BPAP_FC_V4H_AD020][Covers: BPAP_FC_V4H_AD021][Covers: BPAP_FC_V4H_AD024]
                [Covers: BPAP_FC_V4H_AD031][Covers: BPAP_FC_V4H_AD039][Covers: BPAP_FC_V4H_AD043]
                [Covers: BPAP_FC_V4H_AD051][Covers: BPAP_FC_V4H_AD052]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success
 * @retval      false           fail
 *********************************************************************************************************************/
int64_t R_VOUT_Task()
{   
    int ret = INVALID;
    char folder[] = OUTPUT_BUFFER;
    char image_name[64];
    int size = (g_frame_width * g_frame_height) * BPP_RGB;
    static int image_read_receive_flag = 0;
    e_osal_return_t osal_ret = OSAL_RETURN_OK;
    bool is_queue_empty = false;
 
    R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_50MS_SLEEP);
    while (!g_is_thread_exit)
    {   
            ret = f_opencv_execute();                   /* OpenCV execute */
            if (FAILED == ret)
            {
                g_is_thread_exit = true;
                PRINT_ERROR("Failed f_opencv_execute \n");
                return FAILED;
            }

            if (0 != g_customize.Proc_Time)              /* If processing time is enabled */
            {
                fpsCount(0);                             /* Get FPS */
            }
            if (true == g_customize.VOUT_Enable)         /* if VOUT is enabled */
            {
                ret = execute();                         /* VOUT execution */
                if (FAILED == ret)
                {
                    g_is_thread_exit = true;
                    PRINT_ERROR("Failed Vout execute \n");
                    return FAILED;
                }
            }
            if(false == g_customize.VIN_Enable)          /* if VIN is enabled */
            {
                if (true == g_customize.Image_Folder_Enable)
                {
                    FILE * fp = NULL;
                    mkdir(folder, 0777);
                    sprintf(image_name, "%s/%s_out", folder, buffer);
                    fp = fopen(image_name, "wb");
                    fwrite(gp_opencv_buffer, sizeof(unsigned char), (size_t)size, fp);
                    fclose(fp);
                }
            }

            if (true == g_customize.Image_Folder_Enable)
            {
                 /*Receiving message to image read MQ*/
                e_osal_return_t osal_ret = R_OSAL_MqReceiveForTimePeriod(g_mq_handle_imgread, TIMEOUT_MS, 
                                           (void *)&image_read_receive_flag, g_mq_config_imgread.msg_size);     
                if (OSAL_RETURN_OK != osal_ret)
                {
                    PRINT_ERROR("receiving message to image read MQ was failed, osal_ret = %d\n", osal_ret);
                }
            }

        }

    is_queue_empty = false;
    R_OSAL_ThreadSleepForTimePeriod((osal_milli_sec_t)TIMEOUT_50MS_SLEEP);

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_VOUT_Task
 *********************************************************************************************************************/

/*********************************************************************************************************************/ 
/* Function Name :R_Inference_Task  */
/******************************************************************************************************************//**
 * @brief       Inference thread
                [Covers: BPAP_FC_V4H_AD004][Covers: BPAP_FC_V4H_AD005][Covers: BPAP_FC_V4H_AD006]
                [Covers: BPAP_FC_V4H_AD014][Covers: BPAP_FC_V4H_AD018][Covers: BPAP_FC_V4H_AD057]
                [Covers: BPAP_FC_V4H_AD024][Covers: BPAP_FC_V4H_AD025][Covers: BPAP_FC_V4H_AD029]
                [Covers: BPAP_FC_V4H_AD038][Covers: BPAP_FC_V4H_AD050]
 * @param[in]   none
 * @param[out]  none
 * @retval      true           success(0)
 * @retval      false           fail(1)
***********************************************************************************************************************/
int64_t R_Inference_Task()
{ 
    static int counter = 0;
    static int start_time=0;

#if(CDNN)
    R_CDNN_Execute();
#endif
    return 0;
}
/**********************************************************************************************************************
 End of function R_Inference_Task
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_CTRL_Task */
/******************************************************************************************************************//**
 * @brief       User control thread
                [Covers: BPAP_FC_V4H_AD032]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)
 *********************************************************************************************************************/
int64_t R_CTRL_Task()
{
    char user_in[DATA_LEN_64]="";
    
    while (!g_is_thread_exit)
    {

        fgets(user_in, sizeof(user_in), stdin);

        if (strncmp(user_in, "x", 1) == 0)                  /* user input is "x" : exit */
        {
            g_is_thread_exit = true;
            if (g_is_thread_exit)
            {
                break;
            }
        }
        else
        {
            /* Do Nothing */
        }
    }
    return 0;
}
/**********************************************************************************************************************
 End of function R_CTRL_Task
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_Init_Modules */
/******************************************************************************************************************//**
 * @brief       Module Initialization
                [Covers: BPAP_FC_V4H_AD009][Covers: BPAP_FC_V4H_AD020]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success
 * @retval      false           fail
 *********************************************************************************************************************/
int64_t R_Init_Modules()
{
    int ret = INVALID;
    int32_t  ret_sample      = IMPDEMO_OK;

    
    /* Initialize all modules */
    if(true == g_customize.VIN_Enable)                  /* If VIN enabled */
    {
        ret = R_VIN_Initilize(g_customize.VIN_Device, 
                     g_customize.Frame_Width, g_customize.Frame_Height, g_customize.VIN_Offset_X, 
                     g_customize.VIN_Offset_Y, g_customize.VIN_Capture_Format, g_customize.VIN_Req_Buffer_Num, 
                     g_customize.Debug_Enable);  /* Initialize VIN */
        if (FAILED == ret)
        {
            g_fcStatus.vin.status = FAILED;
            PRINT_ERROR("Failed R_VIN_Initilize \n");
            return FAILED;
        }

        ret = R_VIN_Start_Capturing(g_customize.VIN_Device);                                  /* Start capturing frames */
        if (SUCCESS != ret)
        {
            PRINT_ERROR("R_VIN_Start_Capturing failed\n");
            return FAILED;
        }
        g_fcStatus.vin.status = SUCCESS;
    }

    if(true == g_customize.IMR_LDC || true == g_customize.IMR_Resize)
    {
        ret = R_IMR_Init();
        if (FAILED == ret)
        {
            g_fcStatus.imr_ldc.status = FAILED;
            g_fcStatus.imr_rs.status  = FAILED;
            PRINT_ERROR("Failed R_IMR_Init \n");
            return FAILED;
        }

    }

     if (true == g_customize.ISP_Enable)
     {
         ret = R_ISP_Initialize();                    /* ISP initialization */
         if (FAILED == ret)
         {
             g_fcStatus.isp.status = FAILED;
             PRINT_ERROR("Failed ISP_init \n");
             return FAILED;
         }
     }

    if (true == g_customize.IMR_LDC)                   /* If IMR lens distortion correction enabled */
    {
        ret = R_IMR_SetupLDC();                        /* Set up IMR LDC */
        if (FAILED == ret)
        {
            PRINT_ERROR("Failed R_IMR_SetupLDC \n");
            return FAILED;
        }
    }

    if (true == g_customize.IMR_Resize)                /* If IMR resize enabled */
    {
        ret = R_IMR_SetupResize();                     /* Set up IMR resize */
        if (FAILED == ret)
        {
            PRINT_ERROR("Failed R_IMR_SetupResize \n");
            return FAILED;
        }
    }

    if (true == g_customize.VOUT_Enable)               /* If VOUT is enabled */
    {
        ret = vout_init();                             /* Initialize VOUT */

        if (FAILED == ret)
        {
            g_fcStatus.vout.status = FAILED;
            PRINT_ERROR("Failed vout_init \n");
            return FAILED;
        }
        g_fcStatus.vout.status = SUCCESS;
    }
    else
    {
        g_fcStatus.vout.status = FAILED;
        printf("VOUT Disabled : Enable from config file\n");
    }

    set_syncstatus(eFC_DRAW, 0);
    return SUCCESS;
}

/**********************************************************************************************************************
 End of function R_Init_Modules
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_Deinit_Modules */
/******************************************************************************************************************//**
 * @brief       Module deinitialization
                [Covers: BPAP_FC_V4H_AD020]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)
 *********************************************************************************************************************/
int64_t R_Deinit_Modules()
{
    int ret                     = INVALID;
    int32_t  ret_sample         = IMPDEMO_OK;
    e_osal_return_t osal_ret    = OSAL_RETURN_OK;
    bool isEmpty                = false;

    R_OSAL_ThsyncCondDestroy(g_vin_cond_handle);
    R_OSAL_ThsyncCondDestroy(g_imr_ldc_cond_handle);
    R_OSAL_ThsyncCondDestroy(g_imr_rs_cond_handle);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_opencv);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_vout);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_imrrs);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_imrldc);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_isp);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_vin_out);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_yuv2rgb);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_imrldc_out);
    R_OSAL_ThsyncMutexDestroy(g_mtx_handle_imrrs_out);

    osal_ret = R_OSAL_MqIsEmpty(g_mq_handle_aiactivity, &isEmpty);
    if(OSAL_RETURN_OK != osal_ret)
    {
        // Notifying that R_OSAL_MqIsEmpty() failed.
        // Not returning from here as resource de-allocation required to perform.
        PRINT_ERROR("R_OSAL_MqIsEmpty failed, osal_ret = %d\n", osal_ret);
    }

    if(false == isEmpty)
    {
        osal_ret = R_OSAL_MqReset(g_mq_handle_aiactivity);
        if(OSAL_RETURN_OK != osal_ret)
        {
            // Notifying that R_OSAL_MqReset() failed.
            // Not returning from here as resource de-allocation required to perform.
            PRINT_ERROR("R_OSAL_MqReset failed, osal_ret = %d\n", osal_ret);
        }
    }

    osal_ret = R_OSAL_MqDelete(g_mq_handle_aiactivity);
    if(OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_MqDelete, osal_ret = %d\n", osal_ret);
    }

    free(gp_vin_out_buffer);

    if (SUCCESS == g_fcStatus.vin.status) 
    {
        R_VIN_DeInitialize(g_customize.VIN_Device);                            /* Deinitialize VIN */
    }


     if (true == g_customize.ISP_Enable )
     {
         ret = R_ISP_DeInitialize();
         if (FAILED == ret)
         {
             PRINT_ERROR("Failed R_ISP_DeInitialize \n");
             return FAILED;
         }
     }
    free(gp_isp_buffer);
    free(gp_image_buffer);
    free(gp_imr_ldc_buffer);
    free(gp_imr_rs_buffer_ch0);
    free(gp_imr_rs_buffer_ch1);
    free(gp_imr_rs_buffer_ch2);
    free(gp_imr_rs_buffer_ch3);
    free(gp_imr_rs_buffer_ch4);
    free(gp_ai_rgb_buffer);

    if(true == g_customize.IMR_LDC || true == g_customize.IMR_Resize)
    {
        ret = R_IMR_Deinit();
        if (FAILED == ret)
        {
            PRINT_ERROR("Failed IMR DeInitialize \n");
            return FAILED;
        }
    }

    free(gp_opencv_buffer);

    if (SUCCESS == g_fcStatus.vout.status)
    {
        ret = vout_deinit();                                    /* Deinitialize VOUT */
        if (FAILED == ret)
        {
            PRINT_ERROR("Failed vout DeInitialize \n");
            return FAILED;
        }
    }
    ret = deinit_mmgr();                                        /* Deinitialize memory manager */
    if (FAILED == ret)
    {
        PRINT_ERROR("Failed memory manager DeInitialize \n");
        return FAILED;
    }
    ret = R_OSAL_Deinitialize();                                /* Deinitialize OSAL */
    if (FAILED == ret)
    {
        PRINT_ERROR("Failed OSAL DeInitialize \n");
        return FAILED;
    }

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_Deinit_Modules
 *********************************************************************************************************************/
/**********************************************************************************************************************
/* Function Name : R_FC_SystemInit */
/******************************************************************************************************************//**
 * @brief       FC system Initialisation
                [Covers: BPAP_FC_V4H_AD009]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)
 *********************************************************************************************************************/
static int64_t R_FC_SystemInit()
{
    int ret = INVALID;
    g_frame_width = g_customize.Frame_Width;                        /* Set frame width and height */
    g_frame_height = g_customize.Frame_Height;


   switch (g_customize.VOUT_Display_Format)
    {
    /*setting vout pixel format*/
    case 0:
        g_vout_pix_fmt = BPP_YUV;
        break;
    case 1:
        g_vout_pix_fmt = BPP_YUV;
        break;
    case 2:
        g_vout_pix_fmt = BPP_RGB;
        break;
    
    default:
        g_vout_pix_fmt = 0;
        break;
    }

    /* memory allocation for VIN buffer */
    ret = Vin_Buffer_Alloc();
    if(FAILED == ret)
    {
        DEBUG_PRINT("Failed to allocate vin_buffer \n");
        return ret;
    }

    /* memory allocation for ISP buffers */
    ret = Isp_Buffer_Alloc();
    if(FAILED == ret)
    {
        DEBUG_PRINT("Failed to allocate isp buffer \n");
        return ret;
    }

    /* memory allocation for IMR buffers */
    ret = Imr_Buffer_Alloc();
    if(FAILED == ret)
    {
        DEBUG_PRINT("Failed to allocate isp buffer \n");
        return ret;
    }

    /* buffer allocation for OpenCV buffers */
    ret = Opencv_buffer_alloc();
    if(FAILED == ret)
    {
        DEBUG_PRINT("Failed to allocate opencv buffer \n");
        return ret;
    }

    return ret;
}
/**********************************************************************************************************************
 End of function R_FC_SystemInit
 *********************************************************************************************************************/
/**********************************************************************************************************************
/* Function Name : R_PipelineParamValidate */
/******************************************************************************************************************//**
 * @brief       Pipeline Validation
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)
 *********************************************************************************************************************/
int R_PipelineParamValidate()
{

#ifdef NDEBUG
    if (true == g_customize.Debug_Enable)
    {
        PRINT_WARNING("Currently the build has taken in release mode. So the configuration parameter Debug_Enable = 1\
 is an invalid configuration\n");
        return FAILED;
    }
#endif
    if ((true == g_customize.VIN_Enable) && (true == g_customize.Image_Folder_Enable))
    {
        PRINT_WARNING("Configuration parameters VIN_Enable and Image_Folder_Enable are enabled at same time, \
which is an invalid configuration for the application \n");
        return FAILED;
    }
    
#if(CDNN)
     else if ((false == g_customize.IMR_Resize) && (true == g_customize.CDNN_Enable))
     {
        PRINT_WARNING("Configuration parameters IMR_Resize is disabled and CDNN_Enable is enabled at same time, \
which is an invalid configuration for the application \n");
        return FAILED;
     }
#endif
    else if ((true == g_customize.VIN_Enable) && (true == g_customize.ISP_Enable))
    {
        PRINT_WARNING("Currently FC App not supporting raw image from camera. So the configuration VIN_Enable=1 and \
ISP_Enable=1 are invalid configuration for the application \n");
        return FAILED;
    }

    else if((g_customize.Frame_Width * 2) % MAX_LEN_IMG_BUFF != 0)
    {
        PRINT_WARNING("Please give a valid stride value(Frame Width * BPP) as multiple of 256 to run the pipeline \n");
        return FAILED;
    }
    else if((true == g_customize.Image_Folder_Enable) && (true == g_customize.ISP_Enable) )
    {
        PRINT_WARNING("Currently FC App only support to read single ISP buffer. Application pipeline shall not \
                         handle ISP buffers from image folder\n");
        return FAILED;
    }

    return SUCCESS;
}

/**********************************************************************************************************************
 End of function R_PipelineParamValidate
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : buffer_map */
/******************************************************************************************************************//**
 * @brief       Customization
                [Covers: BPAP_FC_V4H_AD053]
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)
 *********************************************************************************************************************/
static int64_t buffer_map()
{
    if (false == g_customize.VIN_Enable)    /* VIN disabled */
    {
        if (false == g_customize.Image_Folder_Enable)
        {
            printf("[%s]\n", g_customize.Frame_File_Name);
            FILE * buf_fp = NULL;
            buf_fp = fopen(g_customize.Frame_File_Name, "rb");
            if (NULL == buf_fp)
            {
                PRINT_ERROR("Input Frame not found [%s]\n", g_customize.Frame_File_Name);
                 return FAILED;
            }

            if (true == g_customize.ISP_Enable)                                 /* ISP enabled */
            {
#if (!RCAR_V4H)
                fread(gp_image_buffer, sizeof(unsigned char), g_frame_height * g_frame_width * BPP_Y, buf_fp);
#else
                fread(gp_image_buffer, sizeof(unsigned char), (1296 * 784 * 2), buf_fp);
#endif
            }
            else
            {
                fread(gp_vin_out_buffer, sizeof(unsigned char), g_frame_height * g_frame_width * BPP_YUV, buf_fp);
            }

            fclose(buf_fp);
        }
    }

    /* ISP Input Customization */
    ISP_inputcustom();
   
    /* IMR LDC Input Customization */
    IMR_LDC_inputcustom();
 
    /* IMR Resize Input Customization */
    IMR_Resize_inputcustom();

    /*gp_yuv2rgb_in buffer setting */
    Color_Conversion_bufferCustom();

    /* OpenCV Input Customization */
    opencv_inputcustom();

    /* Vout Input Customization */
    vout_inputcustom(); 

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function buffer_map 
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : sigint_handler */
/******************************************************************************************************************//**
 * @brief       thread exit
 * @param[in]   signum
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
static void sigint_handler(int signum) 
{
   g_is_thread_exit = true;
   signal(SIGINT, sigint_handler);
}
/***********************************************************************************************************************
* End of function sigint_handler
***********************************************************************************************************************/
/**********************************************************************************************************************
/* Function Name : FcModuleInitFlags*/
/******************************************************************************************************************//**
 * @brief       Module init Flags
 * @param[in]   none
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
static void FcModuleInitFlags()
{
    g_fcStatus.vin.status = INVALID;
    g_fcStatus.imr_ldc.status = INVALID;
    g_fcStatus.imr_rs.status = INVALID;
    g_fcStatus.isp.status = INVALID;
    g_fcStatus.vout.status = INVALID;
}
/***********************************************************************************************************************
* End of function FcModuleInitFlags
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_Create_Mutex */
/******************************************************************************************************************//**
 * @brief       Create Mutex
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)     
 *********************************************************************************************************************/
int64_t R_Create_Mutex()
{
    e_osal_return_t osal_ret;
                                                                              /* Opencv mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO1, &g_mtx_handle_opencv);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate opencv %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* Vout mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO2, &g_mtx_handle_vout);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate vout %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* IMR resize mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO3, &g_mtx_handle_imrrs);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate imrrs %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* IMR LDC mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO4, &g_mtx_handle_imrldc);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate imrldc %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* ISP mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO5, &g_mtx_handle_isp);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate isp %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* Vin_out mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO6, &g_mtx_handle_vin_out);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate vin_out %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* YUV to RGB mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO7, &g_mtx_handle_yuv2rgb);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate yuv2rgb %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* IMR LDC out mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO8, &g_mtx_handle_imrldc_out);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate imrldc_out %d\n", osal_ret);
        return FAILED;
    }
                                                                              /* IMR resize out mutex handle create */
    osal_ret= R_OSAL_ThsyncMutexCreate((osal_mutex_id_t)MUTEX_ID_NO9, &g_mtx_handle_imrrs_out);
    
    if (OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("Failed R_OSAL_ThsyncMutexCreate imrrs out %d\n", osal_ret);
        return FAILED;
    }
    

    return SUCCESS;
}
/***********************************************************************************************************************
* End of function R_Create_Mutex
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : R_Mutex_Map */
/******************************************************************************************************************//**
 * @brief       Mapping of Mutex
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)     
 *********************************************************************************************************************/
int64_t R_Mutex_Map()
{
    /*Vin Customization */
    gp_mtx_handle_vin  = &g_mtx_handle_vin_out;
    gp_vin_cond_handle = &g_vin_cond_handle;

    /* IMR LDC Input Customization */
    gp_mtx_handle_imr_ldc = &g_mtx_handle_vin_out;
    gp_imr_ldc_cond_handle = &g_vin_cond_handle;

    /*yuv2rgb */
    if (true == g_customize.IMR_Resize )
    {
        gp_mtx_handle_yuv2rgb = &g_mtx_handle_imrrs;
    }
    else if (true == g_customize.IMR_LDC )
    {
        gp_mtx_handle_yuv2rgb = &g_mtx_handle_imrldc;
    }
    else 
    {
        gp_mtx_handle_yuv2rgb = &g_mtx_handle_vin_out;
    }

    /* IMR Resize Input Customization */

    gp_mtx_handle_imr_rs = &g_mtx_handle_vin_out;
    gp_imr_rs_cond_handle = &g_vin_cond_handle;

    /* OpenCV Input Customization */
    if (true == g_customize.IMR_LDC )
    {
        gp_mtx_handle_opencv  = &g_mtx_handle_imrldc;
        gp_opencv_cond_handle = &g_imr_ldc_cond_handle;
    }
    else
    {
        gp_mtx_handle_opencv  = &g_mtx_handle_vin_out;
        gp_opencv_cond_handle = &g_vin_cond_handle;
    }

    if (true == g_customize.IMR_LDC )
    {
        gp_mtx_handle_vout  = &g_mtx_handle_imrldc;
    }
    else if (true == g_customize.ISP_Enable )
    {
        gp_mtx_handle_vout = &g_mtx_handle_vin_out;
    }
    else if (true == g_customize.VIN_Enable )
    {
        gp_mtx_handle_vout  = &g_mtx_handle_vin_out;
    }
    else
    {
        gp_mtx_handle_vout  = &g_mtx_handle_opencv;
    }

    return SUCCESS;
}
/***********************************************************************************************************************
* End of function R_Mutex_Map
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : get_syncstatus */
/******************************************************************************************************************//**
 * @brief       To get the sync status
                [Covers: BPAP_FC_V4H_AD009]
 * @param[in]   module              defining FC modules
 * @param[in]   flow                defines the direction of buffer copy of thread(0-input 1-output)
 * @param[out]  none
 * @retval      status  
 *********************************************************************************************************************/
int get_syncstatus(e_fc_module_t module, int flow)
{
    int status = INVALID;
    if (flow == 1)
    {
        switch(module)
        {
        case eVIN:
            if (true == g_customize.IMR_LDC )
            {
                status = g_imr_in_done;
            }
            else if (true == g_customize.IMR_Resize)
            {
                status = g_imr_in_done && g_vout_done;
            }
            else
            {
                status = g_vout_done;
            }
            break;

        case eIMR_LDC:
            status = g_vout_done;
            break;

        case eIMR_RS:
            #if(CDNN)
            if((true == g_customize.CDNN_Enable) || (true == g_customize.MMAP_Contingency_enable))
            {
                status = g_ai_done;
            } 
            #endif
            break;
        }
    }
    else
    {
        switch(module)
        {
            case eIMR_LDC:
                    status = g_imr_in_done;
                break;
            case eFC_DRAW:
                    status = g_vout_done;
                break;
            case eIMR_RS:
                    status = g_imr_in_done;
                    break;
            case eAI:
                    status = g_ai_done;
                    break;
        }
    }

    return status;
}
/***********************************************************************************************************************
* End of function get_syncstatus
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name : set_syncstatus */
/******************************************************************************************************************//**
 * @brief       To set the sync status
                [Covers: BPAP_FC_V4H_AD009]
 * @param[in]   module          defining FC modules
 * @param[in]   flow            defines the direction of buffer copy of thread(0-input 1-output)
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)     
 *********************************************************************************************************************/
int set_syncstatus(e_fc_module_t module, int flow)
{
    int ret;
    if (flow == 1)
    {
        ret =    syncflow_enable(module);
        if(FAILED == ret)
        {
        DEBUG_PRINT("Failed to enable sync status\n");
        return ret;
        }
    }
    else
    {
        ret =    syncflow_disable(module);
        if(FAILED == ret)
        {
        DEBUG_PRINT("Failed to disable sync status\n");
        return ret;
        }
    }
    return SUCCESS;
}
/***********************************************************************************************************************
* End of function set_syncstatus
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  R_FC_SyncStart*/
/******************************************************************************************************************//**
 * @brief       sync start 
                [Covers: BPAP_FC_V4H_AD009]
 * @param[in]   module          defining FC modules
 * @param[in]   flow            defines the direction of buffer copy of thread(0-input 1-output)
 * @param[in]   ptr_mtx         mutex handle for corresponding module
 * @param[in]   ptr_cond        condition handle for corresponding thread
 * @retval      true            success(0)
 * @retval      false           fail(1)     
 *********************************************************************************************************************/
int R_FC_SyncStart(e_fc_module_t module, osal_mutex_handle_t *ptr_mtx, osal_cond_handle_t *ptr_cond, int flow)
{
    /*The function R_FC_SyncStart is used to start the synchronization process in thread.
      A R_OSAL_ThsyncMutexLockForTimePeriod function is used to lock a mutex assigned to handle with timeout for 
      specified time.
      The get_syncstatus function will check the  buffer copy status of the  thread according to the flow. 
      A condition wait function(R_OSAL_ThsyncCondWaitForTimePeriod) is used to blocks the calling thread on the 
      condition handle only if status equals flow.*/

    int sts = 0;
    e_osal_return_t osal_ret = R_OSAL_ThsyncMutexLockForTimePeriod(*ptr_mtx, (osal_milli_sec_t)TIMEOUT_MS);
    
    if (OSAL_RETURN_OK != osal_ret && g_is_thread_exit == false)
    {
        printf("mutex lock failed, osal_ret = %d module = %d \n", osal_ret, module);
    }
    sts = get_syncstatus(module, flow);
    while (sts == flow)
    {
        osal_ret = R_OSAL_ThsyncCondWaitForTimePeriod(*ptr_cond, *ptr_mtx, (osal_milli_sec_t)TIMEOUT_5_S);
        
        if (OSAL_RETURN_OK != osal_ret)
        {   
            if (g_is_thread_exit == false)
            {
                printf(" condition wait failed, osal_ret = %d module = %d direction %d\n", osal_ret, module, flow);
            }
            break;
        }
        sts = get_syncstatus(module, flow);
    }
    return SUCCESS;
}
/***********************************************************************************************************************
* End of function R_FC_SyncStart
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  R_FC_SyncEnd*/
/******************************************************************************************************************//**
 * @brief       sync stop
                [Covers: BPAP_FC_V4H_AD009]
 * @param[in]   module          defining fc modules
 * @param[in]   flow            defines the direction of buffer copy of thread(0-input 1-output)
 * @param[in]   ptr_mtx         mutex handle for corresponding module
 * @param[in]   ptr_cond        condition handle for corresponding thread
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)     
 *********************************************************************************************************************/
int R_FC_SyncEnd(e_fc_module_t module, osal_mutex_handle_t *ptr_mtx, osal_cond_handle_t *ptr_cond, int flow)
{
    /* The function R_FC_SyncEnd is used for end the synchronization process in thread.Here a set_syncstatus function is
       used to set buffer copy status of the thread according to the flow.
       A R_OSAL_ThsyncCondBroadcast function is used to unblocks all thread that's waiting on the condition handle.
       The R_OSAL_ThsyncMutexUnlock function is used to unlock mutex assigned to handle.*/

    set_syncstatus(module, flow);

    e_osal_return_t osal_ret = R_OSAL_ThsyncCondBroadcast(*ptr_cond);

    if (OSAL_RETURN_OK != osal_ret)
    {
        printf(" condition Signal failed, osal_ret = %d module = %d \n", osal_ret, module);
    }

    osal_ret = R_OSAL_ThsyncMutexUnlock(*ptr_mtx);

    if (OSAL_RETURN_OK != osal_ret && g_is_thread_exit == false)
    {
        printf("mutex unlock failed, osal_ret = %d module = %d \n", osal_ret, module);
    }
    return SUCCESS;
}
/***********************************************************************************************************************
* End of function R_FC_SyncEnd
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  uSecElapsed*/
/******************************************************************************************************************//**
 * @brief       To get elapsed time in us
                [Covers: BPAP_FC_V4H_AD041][Covers: BPAP_FC_V4H_AD042][Covers: BPAP_FC_V4H_AD056]
 * @param[in]   t1        time value structure pointer
 * @param[in]   t2        time value structure pointer
 * @param[out]  none
 * @retval      elapsed time
 *********************************************************************************************************************/
static inline unsigned long uSecElapsed(struct timeval *t2, struct timeval *t1)
{
        return (t2->tv_sec - t1->tv_sec) * 1000000 + t2->tv_usec - t1->tv_usec;
}
/***********************************************************************************************************************
* End of function uSecElapsed
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  fpsCount*/
/******************************************************************************************************************//**
 * @brief       To get the fps count
                [Covers: BPAP_FC_V4H_AD041][Covers: BPAP_FC_V4H_AD042]
 * @param[in]   dev
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
void fpsCount(int dev)
{
    static unsigned frames[N_DEVS_MAX];
    static struct timeval frame_time[N_DEVS_MAX];
    static unsigned long usec[N_DEVS_MAX];
    struct timeval t;

    gettimeofday(&t, NULL);
    usec[dev] += frames[dev]++ ? uSecElapsed(&t, &frame_time[dev]) : 0;
    frame_time[dev] = t;
    if (usec[dev] >= 1000000) 
    {
        g_fps[dev] = ((unsigned long long)frames[dev] * 10000000 + usec[dev] - 1) / usec[dev];
        fprintf(stderr, " FPS: %3u.%1u\n", g_fps[dev] / 10, g_fps[dev] % 10);
        usec[dev] = 0;
        frames[dev] = 0;
    }
}
/***********************************************************************************************************************
* End of function fpsCount
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  Vin_Buffer_Alloc */
/******************************************************************************************************************//**
 * @brief       Memory allocation for Vin buffer
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 *********************************************************************************************************************/
static int64_t Vin_Buffer_Alloc()
{
    if (YUYV == g_customize.VIN_Capture_Format)                     /* YUYV capture format */
    {
        gp_vin_out_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_YUV); /* vin buffer allocation */
    }
    else if (RGB == g_customize.VIN_Capture_Format)                 /* RGB capture format */
    {
        gp_vin_out_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_RGB); /* vin buffer allocation */
    }
    else if (Y10 == g_customize.VIN_Capture_Format)                 /* Y10 capture format */
    {
        gp_vin_out_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_Y10); /* vin buffer allocation */
    }
    else                                                            /* vin buffer allocation for other formats */
    {
        gp_vin_out_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_YUV); 
    }

    if (NULL == gp_vin_out_buffer)
    {
        DEBUG_PRINT("Failed to allocate vin_buffer \n");
        return FAILED;
    }
    
    return SUCCESS;
}

/***********************************************************************************************************************
* End of function Vin_Buffer_Alloc
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  Isp_Buffer_Alloc */
/******************************************************************************************************************//**
 * @brief       Memory allocation for isp buffers
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 *********************************************************************************************************************/
static int64_t Isp_Buffer_Alloc()
{
    if (true == g_customize.ISP_RAW_OUT_Format)                     /* ISP raw format enabled */
    {
        gp_isp_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_RGB); /* isp buffer allocation */
    }
    else
    {
        gp_isp_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_YUV);
    }
    if (NULL == gp_isp_buffer)
    {
        DEBUG_PRINT("Failed to allocate isp_buffer Buffer \n");
        return FAILED;
    }

    if (true == g_customize.ISP_Enable)                             /* ISP enabled */
    {
        gp_image_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_RGB); /* image buffer allocation */

        if (NULL == gp_image_buffer)
        {
            DEBUG_PRINT("Failed to allocate gp_image_buffer Buffer \n");
            return FAILED;
        }
    }

    return SUCCESS;
}

/***********************************************************************************************************************
* End of function Isp_Buffer_Alloc
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  Imr_Buffer_Alloc */
/******************************************************************************************************************//**
 * @brief       Memory allocation for imr buffers
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 *********************************************************************************************************************/
static int64_t Imr_Buffer_Alloc()
{

    /* ai_rgb buffer allocation */
    gp_ai_rgb_buffer = (char *)malloc(g_customize.IMR_Resize_Width_Ch_0 * g_customize.IMR_Resize_Height_Ch_0 * BPP_RGB); 
    if (NULL == gp_ai_rgb_buffer)
    {
        DEBUG_PRINT("Failed to allocate gp_ai_rgb_buffer Buffer \n");
        return FAILED;
    }

    gp_imr_ldc_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_YUV); /* imr_ldc buffer allocation */
    if (NULL == gp_imr_ldc_buffer)
    {
        DEBUG_PRINT("Failed to allocate gp_imr_ldc_buffer Buffer \n");
        return FAILED;
    }

    if (g_customize.IMR_Ch_0_Enable)
    {
        gp_imr_rs_buffer_ch0 = (char *)malloc(g_customize.IMR_Resize_Width_Ch_0 * g_customize.IMR_Resize_Height_Ch_0 * 
		BPP_YUV); /* imr_rs buffer allocation */
        if (NULL == gp_imr_rs_buffer_ch0)
        {
            DEBUG_PRINT("Failed to allocate gp_imr_rs_buffer_ch0 Buffer \n");
            return FAILED;
        }
    }

    if (g_customize.IMR_Ch_1_Enable)
    {
        gp_imr_rs_buffer_ch1 = (char *)malloc(g_customize.IMR_Resize_Width_Ch_1 * g_customize.IMR_Resize_Height_Ch_1 * 
		BPP_YUV); /* imr_rs buffer allocation */
        if (NULL == gp_imr_rs_buffer_ch1)
        {
            DEBUG_PRINT("Failed to allocate gp_imr_rs_buffer_ch1 Buffer \n");
            return FAILED;
        }
    }

    if (g_customize.IMR_Ch_2_Enable)
    {
        gp_imr_rs_buffer_ch2 = (char *)malloc(g_customize.IMR_Resize_Width_Ch_2 * g_customize.IMR_Resize_Height_Ch_2 * 
		BPP_YUV); /* imr_rs buffer allocation */
        if (NULL == gp_imr_rs_buffer_ch2)
        {
            DEBUG_PRINT("Failed to allocate gp_imr_rs_buffer_ch2 Buffer \n");
            return FAILED;
        }
    }

    if (g_customize.IMR_Ch_3_Enable)
    {
        gp_imr_rs_buffer_ch3 = (char *)malloc(g_customize.IMR_Resize_Width_Ch_3 * g_customize.IMR_Resize_Height_Ch_3 * 
		BPP_YUV); /* imr_rs buffer allocation */
        if (NULL == gp_imr_rs_buffer_ch3)
        {
            DEBUG_PRINT("Failed to allocate gp_imr_rs_buffer_ch3 Buffer \n");
            return FAILED;
        }
    }

    if (g_customize.IMR_Ch_4_Enable)
    {
        gp_imr_rs_buffer_ch4 = (char *)malloc(g_customize.IMR_Resize_Width_Ch_4 * g_customize.IMR_Resize_Height_Ch_4 * 
		BPP_YUV); /* imr_rs buffer allocation */
        if (NULL == gp_imr_rs_buffer_ch4)
        {
            DEBUG_PRINT("Failed to allocate gp_imr_rs_buffer_ch4 Buffer \n");
            return FAILED;
        }
    }

    return SUCCESS;
}

/***********************************************************************************************************************
* End of function Imr_Buffer_Alloc
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  ISP_inputcustom*/
/******************************************************************************************************************//**
 * @brief       ISP Input Customization        
 * @param[in]   none
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
static void ISP_inputcustom()
{
    /* ISP Input Customization */
    if (true == g_customize.VIN_Enable)
    {
        gp_isp_in = gp_vin_out_buffer;
    }
    else
    {
        gp_isp_in = gp_image_buffer;
    }
}

/***********************************************************************************************************************
* End of function ISP_inputcustom
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  IMR_LDC_inputcustom*/
/******************************************************************************************************************//**
 * @brief       IMR LDC Input Customization           
 * @param[in]   none
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
static void IMR_LDC_inputcustom()
{
    /* IMR LDC Input Customization */
    if (true == g_customize.ISP_Enable)
    {

        gp_imr_ldc_in = gp_isp_buffer;
    }
    else if (true == g_customize.VIN_Enable)
    {
        gp_imr_ldc_in = gp_vin_out_buffer;
    }
    else
    {
        gp_imr_ldc_in = gp_vin_out_buffer; 
    }
}
/***********************************************************************************************************************
* End of function IMR_LDC_inputcustom
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  IMR_Resize_inputcustom */
/******************************************************************************************************************//**
 * @brief       IMR Resize Input Customization             
 * @param[in]   none
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
static void IMR_Resize_inputcustom()
{
    /* IMR Resize Input Customization */
    if (true == g_customize.IMR_LDC)
    {
        gp_imr_res_in = gp_imr_ldc_buffer;
    }
    else if (true == g_customize.ISP_Enable)
    {
        gp_imr_res_in = gp_isp_buffer;
    }
    else if (true == g_customize.VIN_Enable)
    {
        gp_imr_res_in = gp_vin_out_buffer;
    }
    else
    {
        gp_imr_res_in = gp_vin_out_buffer;
    }

}
/***********************************************************************************************************************
* End of function IMR_Resize_inputcustom
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  Color_Conversion_bufferCustom */
/******************************************************************************************************************//**
 * @brief       colour conversion buffer customization            
 * @param[in]   none
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/    
static void Color_Conversion_bufferCustom()
{   
    /*gp_yuv2rgb_in buffer setting */
    if (true == g_customize.IMR_Resize)
    {
        gp_yuv2rgb_in = gp_imr_rs_buffer_ch0;
    }
    else if (true == g_customize.IMR_LDC)
    {
        gp_yuv2rgb_in = gp_imr_ldc_buffer;
    }
    else if (true == g_customize.ISP_Enable)
    {
        gp_yuv2rgb_in = gp_isp_buffer;
    }
    else 
    {
        gp_yuv2rgb_in = gp_vin_out_buffer;
    }

    gp_cnn_in = gp_ai_rgb_buffer;

}
/***********************************************************************************************************************
* End of function Color_Conversion_bufferCustom
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  opencv_inputcustom */
/******************************************************************************************************************//**
 * @brief       OpenCV Input Customization             
 * @param[in]   none
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
static void opencv_inputcustom()
{
    /* OpenCV Input Customization */
    if (true == g_customize.IMR_LDC)
    {
        gp_opencv_in = gp_imr_ldc_buffer;
    }
    else if (true == g_customize.VIN_Enable)
    {
        gp_opencv_in = gp_vin_out_buffer;
    }
    else if (true == g_customize.ISP_Enable)
    {
        gp_opencv_in = gp_isp_buffer;
    }
    else
    {
        gp_opencv_in = gp_vin_out_buffer;
    }
}
/***********************************************************************************************************************
* End of function opencv_inputcustom
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  vout_inputcustom */
/******************************************************************************************************************//**
 * @brief       Vout Input Customization            
 * @param[in]   none
 * @param[out]  none
 * @retval      none
 *********************************************************************************************************************/
static void vout_inputcustom()
{
    /* Vout Input Customization */
    if (true == g_customize.VOUT_Enable)
    {
        gp_vout_in = gp_opencv_buffer;
    }
}
/***********************************************************************************************************************
* End of function vout_inputcustom
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  syncflow_enable */
/******************************************************************************************************************//**
 * @brief       To enable sync flow
                [Covers: BPAP_FC_V4H_AD009]
 * @param[in]   module
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 *********************************************************************************************************************/
static int64_t syncflow_enable(e_fc_module_t module)
{ 
    switch(module)
    {
        case eVIN:
            if (true == g_customize.IMR_LDC )
            {
                g_imr_in_done = 1;
            }
            else if (true == g_customize.IMR_Resize)
            {
                g_imr_in_done = 1;
                g_vout_done = 1;                    // TODO: Need to fix once Opencv enable/ disable is implemented
            }
            else
            {
                g_vout_done = 1;                    // TODO: Need to fix once Opencv enable/ disable is implemented
            }
            break;

        case eIMR_LDC:
            if (true == g_customize.IMR_Resize)
            {
                    g_vout_done = 1;
            }
            else
            {
                    g_vout_done = 0;
            }
            break;

        case eIMR_RS:
            #if(CDNN)
            if((true == g_customize.CDNN_Enable) || (true == g_customize.MMAP_Contingency_enable))
            {
                g_ai_done = 1;
            }
            #endif
            break;
    }

    return SUCCESS;
}
/***********************************************************************************************************************
* End of function syncflow_enable
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  syncflow_disable */
/******************************************************************************************************************//**
 * @brief       To disable sync flow
                [Covers: BPAP_FC_V4H_AD009]
 * @param[in]   module
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 *********************************************************************************************************************/
static int64_t syncflow_disable(e_fc_module_t module)
{
    switch(module)
    {
        case eIMR_LDC:
            if (true == g_customize.VIN_Enable || true == g_customize.ISP_Enable || 
                true == g_customize.Image_Folder_Enable)
            {
                g_imr_in_done = 0;
            }
            else
            {
                g_imr_in_done = 1;
            }
            break;
        case eIMR_RS:
            if (true == g_customize.VIN_Enable || true == g_customize.ISP_Enable || 
                true == g_customize.Image_Folder_Enable)
            {
                g_imr_in_done = 0;
            }
            else
            {
                g_imr_in_done = 1;
            }
            break;
        case eAI:
                g_ai_done = 0;
            break;
        case eFC_DRAW:
                if (false == g_customize.VIN_Enable && false == g_customize.IMR_LDC && 
                false == g_customize.ISP_Enable && false == g_customize.Image_Folder_Enable)
                {
                    g_vout_done = 1;
                }
                else
                {
                    g_vout_done = 0;
                }
                
            break;
    }

    return SUCCESS;
}
/***********************************************************************************************************************
* End of function syncflow_disable
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  st_r_vin_execute_main */
/******************************************************************************************************************//**
 * @brief       camera capture execution main
                [Covers: BPAP_FC_V4H_AD015][Covers: BPAP_FC_V4H_AD035]
 * @param[in]   module
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 *********************************************************************************************************************/
static int st_r_vin_execute_main (void)
{
    int ret = FAILED;

    ret = R_VIN_Execute (g_customize.VIN_Capture_Format, g_customize.VIN_Device);
    if (FAILED == ret)
    {
        PRINT_ERROR("Failed R_VIN_Execute \n");
        return ret;
    }
    
    if(true == g_customize.ISP_Enable)
    {
        ret = R_VIN_Copy_Data (g_customize.VIN_Capture_Format, gp_vin_out_buffer, g_customize.VIN_Device);
    }
    else
    {
        R_FC_SyncStart(eVIN, gp_mtx_handle_vin, gp_vin_cond_handle, 1);
        
        ret = R_VIN_Copy_Data (g_customize.VIN_Capture_Format, gp_vin_out_buffer, g_customize.VIN_Device);
        
        R_FC_SyncEnd(eVIN, gp_mtx_handle_vin, gp_vin_cond_handle, 1);
    }
    
    return ret;
}

/***********************************************************************************************************************
* End of function st_r_vin_execute_main
***********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  Opencv_buffer_alloc */
/******************************************************************************************************************//**
 * @brief       To allocate memory for opencv buffers
                
 * @param[in]   none
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 *********************************************************************************************************************/
static int Opencv_buffer_alloc()
{
    gp_opencv_buffer = (char *)malloc(g_frame_width * g_frame_height * BPP_RGB);  /* opencv buffer allocation */
    if (NULL == gp_opencv_buffer)
    {
        DEBUG_PRINT("Failed to allocate gp_opencv_buffer Buffer \n");
        return FAILED;
    }

    gp_graph_buffer = (char *)malloc(FRAME_WIDTH * GRAPH_HEIGHT * BPP_RGB);                      /* graph buffer allocation */
    if (NULL == gp_graph_buffer)
    {
        DEBUG_PRINT("Failed to allocate gp_graph_buffer Buffer \n");
        return FAILED;
    }

    gp_log_buffer = (char *)malloc(LOG_SCREEN_WIDTH * LOG_SCREEN_HEIGHT * BPP_RGB);                        /* log buffer allocation */
    if (NULL == gp_log_buffer)
    {
        DEBUG_PRINT("Failed to allocate gp_log_buffer Buffer \n");
        return FAILED;
    }

    mul_plane_buffer_1 = (char *)malloc(MULTIPLANE_SCREEN_WIDTH * MULTIPLANE_SCREEN_HEIGHT * BPP_RGB);
    if (NULL == mul_plane_buffer_1)
    {
        DEBUG_PRINT("Failed to allocate mul_plane_buffer_1 Buffer \n");
        return FAILED;
    }

    mul_plane_buffer_2 = (char *)malloc(MULTIPLANE_SCREEN_WIDTH * MULTIPLANE_SCREEN_HEIGHT * BPP_RGB);
    if (NULL == mul_plane_buffer_2)
    {
        DEBUG_PRINT("Failed to allocate mul_plane_buffer_2 Buffer \n");
        return FAILED;
    }

    mul_plane_buffer_3 = (char *)malloc(MULTIPLANE_SCREEN_WIDTH * MULTIPLANE_SCREEN_HEIGHT * BPP_RGB);
    if (NULL == mul_plane_buffer_3)
    {
        DEBUG_PRINT("Failed to allocate mul_plane_buffer_3 Buffer \n");
        return FAILED;
    }

    mul_plane_buffer_4 = (char *)malloc(MULTIPLANE_SCREEN_WIDTH * MULTIPLANE_SCREEN_HEIGHT * BPP_RGB);
    if (NULL == mul_plane_buffer_4)
    {
        DEBUG_PRINT("Failed to allocate mul_plane_buffer_4 Buffer \n");
        return FAILED;
    }

    return SUCCESS;
}
/***********************************************************************************************************************
* End of function Opencv_buffer_alloc
***********************************************************************************************************************/
