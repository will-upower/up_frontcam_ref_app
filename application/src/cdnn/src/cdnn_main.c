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

/**********************************************************************************************************************
 * File Name    : cdnn_main.c
 * Version      : 0.1.0
 * Description  : cdnn functions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 * History : DD.MM.YYYY Version  Description
 *         : 06.03.2023 0.1.0     Alpha Release
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/* Local includes */
#include "common.h"
#include "cdnn_main.h"
#include "buffer_configuration.h"
#include "object_detection.h"

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables
 *********************************************************************************************************************/

float* address;
float*** reshape_array;
float*** temparr2;
float* temparr3;
float**** final_array;
float od_post_out[OBJ_DET_OUT_COLOUMN][OBJ_DET_OUT_ROW];
float objectness;
int (* fp_syncstart) ();
int (* fp_syncend) ();

bool R_FC_Pre_post(e_ai_pre_post_t inf_work, const int8_t* data);
bool (* fp_inf_pre_post) (e_ai_pre_post_t,  const int8_t* data);
char ai_final_buffer[SEM_SEG_IMG_WIDTH * SEM_SEG_IMG_HEIGHT * SEM_SEG_IMG_CHANNEL];

unsigned char ai_buffer[SEM_SEG_IMG_WIDTH][SEM_SEG_IMG_HEIGHT][SEM_SEG_IMG_CHANNEL];
float out_array[SEM_SEG_IMG_CHANNEL][SEM_SEG_IMG_WIDTH][SEM_SEG_IMG_HEIGHT];
float ai_out_buffer_final[SEM_SEG_IMG_WIDTH][SEM_SEG_IMG_HEIGHT][SEM_SEG_IMG_CHANNEL];
uint8_t post_proc_arr[SEM_SEG_IMG_WIDTH * SEM_SEG_IMG_HEIGHT];
float max_value = 0;
int count = 0;
float sem_output[SEM_SEG_IMG_CHANNEL * SEM_SEG_IMG_WIDTH * SEM_SEG_IMG_HEIGHT];
float ai_out_buffer[SEM_SEG_IMG_WIDTH * SEM_SEG_IMG_HEIGHT * SEM_SEG_IMG_CHANNEL] = {0};

unsigned char ai_buffer_od[OBJ_WIDTH][OBJ_HEIGHT][INF_INPUT_CHANNELS];
char ai_final_buffer_od[OBJ_WIDTH * OBJ_HEIGHT * INF_INPUT_CHANNELS];

unsigned char ai_buffer_pe[POSE_EST_IMG_WIDTH][POSE_EST_IMG_HEIGHT][POSE_EST_IMG_CHANNEL];
char ai_final_buffer_pe[POSE_EST_IMG_WIDTH * POSE_EST_IMG_HEIGHT * POSE_EST_IMG_CHANNEL];
int network_number;
/**********************************************************************************************************************
 Static variables and functions 
 *********************************************************************************************************************/
static int R_FC_SyncStartWrapperAI();
static int R_FC_SyncEndWrapperAI();
static void softmax(float *x, int n);
char * gp_ai_final_buffer;
void Conv_YUYV2RGB(unsigned char * yuyv, unsigned char * bgr, int width, int height);
static unsigned char * get_imr_resize_buffer (int channel);
/**********************************************************************************************************************
 Function Declarations
*********************************************************************************************************************/
st_network_t s_network_info_h[1] = {
                    #include "fc_v4h2_objdet_netinfo.h"
                    }; 

st_network_t s_network_info_h_ss[1] = {
                    #include "fc_v4h2_semseg_netinfo.h"
                    }; 

st_network_t s_network_info_h_od[1] = {
                    #include "fc_v4h2_objdet_netinfo.h"
                    }; 

st_network_t s_network_info_h_pe[1] = {
                    #include "fc_v4h2_poseest_netinfo.h"
                    };

st_network_t s_network_info_h_ssod[2] = {
                    #include "fc_v4h2_semseg_netinfo.h"
                    #include "fc_v4h2_objdet_netinfo.h"
                    };

st_network_t s_network_info_h_sspe[2] = {
                    #include "fc_v4h2_semseg_netinfo.h"
                    #include "fc_v4h2_poseest_netinfo.h"
                    };

st_network_t s_network_info_h_odpe[2] = {
                    #include "fc_v4h2_objdet_netinfo.h"
                    #include "fc_v4h2_poseest_netinfo.h"
                    };

st_network_t s_network_info_h_ssodpe[3] = {
                    #include "fc_v4h2_semseg_netinfo.h"
                    #include "fc_v4h2_objdet_netinfo.h"
                    #include "fc_v4h2_poseest_netinfo.h"
                    };

/**********************************************************************************************************************
/* Function Name :  R_CDNN_Execute */
/******************************************************************************************************************//**
 * @brief       CDNN execution function
                [Covers: BPAP_FC_V4H_AD004][Covers: BPAP_FC_V4H_AD005][Covers: BPAP_FC_V4H_AD006]
                [Covers: BPAP_FC_V4H_AD014][Covers: BPAP_FC_V4H_AD018][Covers: BPAP_FC_V4H_AD057]
                [Covers: BPAP_FC_V4H_AD025][Covers: BPAP_FC_V4H_AD038][Covers: BPAP_FC_V4H_AD050]
                [Covers: BPAP_FC_V4H_AD003][Covers: BPAP_FC_V4H_AD013]
 * @param[in]   None
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)
 ***********************************************************************************************************************/
int R_CDNN_Execute()
{
    fp_syncstart = R_FC_SyncStartWrapperAI;
    fp_syncend = R_FC_SyncEndWrapperAI;
    fp_inf_pre_post = R_FC_Pre_post;
    e_usecore_t target_usecore = IMPFWDEMO_CNN_ONLY;

    if(true == g_customize.CDNN_Enable)
    {
        R_OSAL_ThreadSleepForTimePeriod ((osal_milli_sec_t)TIMEOUT_50MS_SLEEP);
        params_to_cdnn(AI_SYNC_ENABLE, get_imr_resize_buffer(g_sem_seg_map_ch), 
            gp_ai_rgb_buffer, g_customize.IMR_Resize_Width_Ch_0, g_customize.IMR_Resize_Height_Ch_0, &g_is_thread_exit, 
            s_imp_dev_axi_bus_id, fp_syncstart, fp_syncend, g_customize.Proc_Time, 
            g_fps, &g_cnn_load_cnt, &g_exe_time, g_customize.CDNN_Load_Enable, fp_inf_pre_post, gp_ai_final_buffer);

        if(g_customize.SEM_SEG_Enable == 1 && g_customize.OBJ_DET_Enable == 1 && g_customize.POSE_EST_Enable == 1)
        {
            
            network_number = 0;

            st_meminfo_t s_meminfo_h = {
                .input_tensor =
                {
                    #include "fc_v4h2_semseg_input.h"
                    #include "fc_v4h2_objdet_input.h"
                    #include "fc_v4h2_poseest_input.h"
                    },
                    .inttensor           = {0},
                    .orig_inttensor_addr = 0xd0000000,
            };

            load_model_info(3, target_usecore, s_meminfo_h, s_network_info_h_ssodpe);

        }
        else if(g_customize.SEM_SEG_Enable == 1 && g_customize.OBJ_DET_Enable == 1)
        {

            network_number = 0;

            st_meminfo_t s_meminfo_h = {
                .input_tensor =
                {
                    #include "fc_v4h2_semseg_input.h"
                    #include "fc_v4h2_objdet_input.h"
                    },
                    .inttensor           = {0},
                    .orig_inttensor_addr = 0xd0000000,
            };

            load_model_info(2, target_usecore, s_meminfo_h, s_network_info_h_ssod);

        }
        else if(g_customize.SEM_SEG_Enable == 1 && g_customize.POSE_EST_Enable == 1)
        {

            network_number = 0;

            st_meminfo_t s_meminfo_h = {
                .input_tensor =
                {
                    #include "fc_v4h2_semseg_input.h"
                    #include "fc_v4h2_poseest_input.h"
                    },
                    .inttensor           = {0},
                    .orig_inttensor_addr = 0xd0000000,
            };

            load_model_info(2, target_usecore, s_meminfo_h, s_network_info_h_sspe);

        }
        else if(g_customize.OBJ_DET_Enable == 1 && g_customize.POSE_EST_Enable == 1)
        {

            network_number = 0;

            st_meminfo_t s_meminfo_h = {
                .input_tensor =
                {
                    #include "fc_v4h2_objdet_input.h"
                    #include "fc_v4h2_poseest_input.h"
                    },
                    .inttensor           = {0},
                    .orig_inttensor_addr = 0xd0000000,
            };

            load_model_info(2, target_usecore, s_meminfo_h, s_network_info_h_odpe);

        }
        else if (g_customize.SEM_SEG_Enable == 1)
        {
            network_number = 1;
            st_meminfo_t s_meminfo_h = {
                .input_tensor =
                {
                    #include "fc_v4h2_semseg_input.h"
                    },
                    .inttensor           = {0},
                    .orig_inttensor_addr = 0xd0000000,
            };

            load_model_info(1, target_usecore, s_meminfo_h, s_network_info_h_ss);

        }
        else if (g_customize.OBJ_DET_Enable == 1)
        {
            network_number = 0;
            st_meminfo_t s_meminfo_h = {
                .input_tensor =
                {
                    #include "fc_v4h2_objdet_input.h"
                    },
                    .inttensor           = {0},
                    .orig_inttensor_addr = 0xd0000000,
            };

            load_model_info(1, target_usecore, s_meminfo_h, s_network_info_h_od);

        }
        else if (g_customize.POSE_EST_Enable == 1)
        {
            network_number = 0;
            st_meminfo_t s_meminfo_h = {
                .input_tensor =
                {
                    #include "fc_v4h2_poseest_input.h"
                    },
                    .inttensor           = {0},
                    .orig_inttensor_addr = 0xd0000000,
            };

            load_model_info(1, target_usecore, s_meminfo_h, s_network_info_h_pe);

        }

        cdnn_ai_main();

    }

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_CDNN_Execute
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  inferencePreprocess */
/******************************************************************************************************************//**
 * @brief       Inference Pre process steps
                [Covers: BPAP_FC_V4H_AD004][Covers: BPAP_FC_V4H_AD005][Covers: BPAP_FC_V4H_AD014]
 * @param[in]   None
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 ***********************************************************************************************************************/
void inferencePreprocess_ss()
{
    /*preprocess of semantic segmentation*/
       
    int32_t ret_sample = IMPDEMO_OK;
    size_t  ret;

    /*convert to 3D Array*/
    int cnt = 0;
    for (uint32_t y = 0; y < g_customize.SEM_SEG_Width ; y++)
    {
        for (uint32_t x = 0; x < g_customize.SEM_SEG_Height; x++) 
        {
            for (uint32_t c = 0; c < SEM_SEG_IMG_CHANNEL; c++)
            {               
                ai_buffer[y][x][c] = gp_ai_rgb_buffer[cnt];   
                cnt++;
            }
        }
    }

    /* transpose */
    cnt = 0;
    for (int k = 0; k < SEM_SEG_IMG_CHANNEL; k++)
    {
        for (int i = 0; i < g_customize.SEM_SEG_Width; i++)
        {
            for (int j = 0; j < g_customize.SEM_SEG_Height; j++)
            {       
                ai_final_buffer[cnt] = ai_buffer[i][j][k];
                cnt++;
            }
        }
    }

    /* Normalize the values by dividng 2 */
    for(int i = 0; i < SEM_SEG_OUTPUT_LEN ; i++)
    {
        ai_final_buffer[i] = (ai_final_buffer[i] / 2);
    }
    gp_ai_final_buffer = ai_final_buffer;
    return ret_sample;
}
/**********************************************************************************************************************
 End of function inferencePreprocess
 *********************************************************************************************************************/


/**********************************************************************************************************************
/* Function Name :  inferencePreprocess_od */
/******************************************************************************************************************//**
 * @brief       Inference Pre process steps
 * @param[in]   None
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1) 
 ***********************************************************************************************************************/
void inferencePreprocess_od()
{
    /*preprocess of object detection*/
    int32_t ret_sample = IMPDEMO_OK;
    size_t  ret;

    /*convert to 3D Array*/
    int cnt = 0;
    for (uint32_t x = 0; x < OBJ_WIDTH; x++)
    {
        for (uint32_t y = 0; y < OBJ_HEIGHT; y++) 
        {
            for (uint32_t c = 0; c < INF_INPUT_CHANNELS; c++)
            {               
                ai_buffer_od[x][y][c] = gp_ai_rgb_buffer[cnt];
                cnt++;
            }
        }
    }

    /* transpose */
    cnt = 0;
    for (int k = 0; k < INF_INPUT_CHANNELS; k++)
    {
        for (int i = 0; i < OBJ_WIDTH; i++)
        {
            for (int j = 0; j < OBJ_HEIGHT; j++)
            {       
                ai_final_buffer_od[cnt] = ai_buffer_od[i][j][k];
                cnt++;
            }
        }
    }

    /* Normalize the values by dividng 2 */
    for(int i = 0; i < (OBJ_WIDTH * OBJ_HEIGHT * INF_INPUT_CHANNELS); i++)
    {
        ai_final_buffer_od[i] = (ai_final_buffer_od[i] / 2);
    }
    gp_ai_final_buffer = ai_final_buffer_od;
    return ret_sample;
}

/**********************************************************************************************************************
/* Function Name :  inferencePreprocess_pe */
/******************************************************************************************************************//**
 * @brief       Inference Pre process steps pose estimation
                [Covers: BPAP_FC_V4H_AD059]
 * @param[in]   None
 * @param[out]  none
 * @retval      true            success(0)
 * @retval      false           fail(1)
 ***********************************************************************************************************************/
void inferencePreprocess_pe()
{
    /*preprocess of Pose Estimation*/

    int32_t ret_sample = IMPDEMO_OK;
    size_t  ret;
    int cnt = 0;

    /*convert to 3D Array*/  
    for (uint32_t x = 0; x < g_customize.POSE_EST_Width; x++)
    {
        for (uint32_t y = 0; y < g_customize.POSE_EST_Height; y++)
        {
            for (uint32_t c = 0; c < POSE_EST_IMG_CHANNEL; c++)
            {
                ai_buffer_pe[x][y][c] = gp_ai_rgb_buffer[cnt];
                cnt++;
            }
        }
    }

    /* transpose */
    cnt = 0;
    for (int k = 0; k < POSE_EST_IMG_CHANNEL; k++)
    {
        for (int i = 0; i < g_customize.POSE_EST_Width; i++)
        {
            for (int j = 0; j < g_customize.POSE_EST_Height; j++)
            {
                ai_final_buffer_pe[cnt] = ai_buffer_pe[i][j][k];
                cnt++;
            }
        }
    }

    /* Normalize the values by dividng 2 */
    for(int i = 0; i < POSE_EST_OUTPUT_LEN; i++)
    {
        ai_final_buffer_pe[i] = (ai_final_buffer_pe[i] / 2);
    }
    gp_ai_final_buffer = ai_final_buffer_pe;
    
    return ret_sample;
}
/**********************************************************************************************************************
 End of function inferencePreprocess
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  inferencePostprocess_ss */
/******************************************************************************************************************//**
 * @brief       inferance post process steps
                [Covers: BPAP_FC_V4H_AD004][Covers: BPAP_FC_V4H_AD005][Covers: BPAP_FC_V4H_AD004]
                [Covers: BPAP_FC_V4H_AD014]
 * @param[in]   data contain inference output 
 * @param[out]  none
 * @retval      None
 ***********************************************************************************************************************/
void inferencePostprocess_ss(signed char * data)
{
    uint32_t det_que =0;

    /* Divide with scaled value(10.038) */
    for(int i= 0 ; i< SEM_SEG_OUTPUT_LEN ;i++)
    {
        sem_output[i] = (data[i]) / (float)(10.038);
    }

    /* Store model output to 3d array */
    count = 0;
    for (int i = 0; i < SEM_SEG_IMG_CHANNEL; i++)
    {
        for (int j = 0; j < g_customize.SEM_SEG_Width; j++)
        {
            for (int k = 0; k < g_customize.SEM_SEG_Height; k++)
            {      
                out_array[i][j][k] = sem_output[count];
                count++;
            }
        }
    }

    /* transpose */
    count = 0;
    for (int j = 0; j < g_customize.SEM_SEG_Width; j++)
    {
        for (int k = 0; k < g_customize.SEM_SEG_Height; k++)
        {
            for (int i = 0; i < SEM_SEG_IMG_CHANNEL; i++)
            {
                ai_out_buffer[count] = out_array[i][j][k];
                count++;
            }
        }
    }

    /* softmax function */
    softmax(ai_out_buffer, SEM_SEG_OUTPUT_LEN);

    /* Store model output to 3d array */
    count = 0;
    for (int i = 0; i < g_customize.SEM_SEG_Width; i++)
    {
        for (int j = 0; j < g_customize.SEM_SEG_Height; j++)
        {
            for (int k = 0; k < SEM_SEG_IMG_CHANNEL; k++)
            {      
                ai_out_buffer_final[i][j][k] = ai_out_buffer[count];
                count++;
            }
        }
    }

    /* Find the maximum values */
    count = 0;
    for (int i = 0; i < g_customize.SEM_SEG_Width; i++)
    {
        for (int j = 0; j < g_customize.SEM_SEG_Height; j++)
        {
            for (int k = 0; k < SEM_SEG_IMG_CHANNEL; k++)
            {
                if (ai_out_buffer_final[i][j][k] > max_value)
                {
                    max_value = ai_out_buffer_final[i][j][k];
                    if(k == 2)
                    {
                        post_proc_arr[count] = eNOROAD;
                    }
                    if(k == 1)
                    {
                        post_proc_arr[count] = eROAD;
                    }
                    if(k == 0)
                    {
                        post_proc_arr[count] = eLANE;
                    }
                }
            }

            max_value = 0.0;
            count ++;  
        }
    }

    memcpy(sem_seg_array, post_proc_arr, sizeof(post_proc_arr));

    if (g_customize.OBJ_DET_Enable == 0 || g_customize.POSE_EST_Enable == 0)
    {
        e_osal_return_t osal_ret = R_OSAL_MqSendForTimePeriod(g_mq_handle_aiactivity, TIMEOUT_MS, (void *)&det_que, 
                                                                g_mq_config_aiactivity.msg_size);
        if(OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("sending message to detection que MQ was failed, osal_ret = %d\n", osal_ret);
        }
    }

}
/**********************************************************************************************************************
 End of function inferencePostprocess
 *********************************************************************************************************************/

float sigmoid(float x)
{
    return (float)(1.0 / (1.0 + exp(-x)));
}
/**********************************************************************************************************************
/* Function Name :  inferencePostprocess_od */
/******************************************************************************************************************//**
 * @brief       inferance post process steps
                [Covers: BPAP_FC_V4H_AD003][Covers: BPAP_FC_V4H_AD013]
 * @param[in]   data contain inference output 
 * @param[out]  none
 * @retval      None
 ***********************************************************************************************************************/
void inferencePostprocess_od(signed char * data)
{
    uint32_t det_que = 0;
    int      no_obj  = 0;

    float scale_values[] = {6.121841020459118F, 6.086807820233777F, 6.477121153846503F};

    typedef struct {
        const float anchors[6];
    }_anchors;

    _anchors anch[] = { 
                            {{ 116, 90, 156, 198, 373, 326 }},
                            {{ 30, 61, 62, 45, 59, 119 }},
                            {{ 10, 13, 16, 30, 33, 23 }}
                      };

    for (int n = 0; n < (int)s_network_info_h[network_number].output_num; n++)
    {
        int cnt = 0;

        int grid_h = s_network_info_h[network_number].out_cf[n].height;
        int grid_w = s_network_info_h[network_number].out_cf[n].width;

        reshape_array = (float***)malloc(s_network_info_h[network_number].out_cf[n].ch * sizeof(float**));
        for (int w = 0; w < s_network_info_h[network_number].out_cf[n].ch; w++)
        {
            reshape_array[w] = (float**)malloc(s_network_info_h[network_number].out_cf[n].height * sizeof(float*));
     
            for (int r = 0; r < s_network_info_h[network_number].out_cf[n].height; r++)
            {
                reshape_array[w][r] = (float*)malloc(s_network_info_h[network_number].out_cf[n].width * sizeof(float));
            }
        }

        temparr2 = (float***)malloc(s_network_info_h[network_number].out_cf[n].height * sizeof(float**));
        for (int w = 0; w < s_network_info_h[network_number].out_cf[n].height; w++)
        {
            temparr2[w] = (float**)malloc(s_network_info_h[network_number].out_cf[n].width * sizeof(float*));
     
            for (int r = 0; r < s_network_info_h[network_number].out_cf[n].width; r++)
            {
                temparr2[w][r] = (float*)malloc(s_network_info_h[network_number].out_cf[n].ch * sizeof(float));
            }
        }

        temparr3 = (float *)malloc(sizeof(float) * (s_network_info_h[network_number].out_cf[n].ch_stride * 
                   s_network_info_h[network_number].out_cf[n].ch));

        final_array = (float ****)malloc(grid_h * sizeof(float ***));
        for (int w = 0; w < grid_h; w++)
        {
            final_array[w] = (float ***)malloc(grid_w * sizeof(float **));
            for (int r = 0; r < grid_w; r++)
            {
                final_array[w][r] = (float **)malloc(NUM_BB * sizeof(float *));
                for (int s = 0; s < NUM_BB; s++)
                {
                    final_array[w][r][s] = (float *)malloc((NUM_CLASS + 5) * sizeof(float));
                }
            }
        }

        /* Scaling with scale values */
        address = (float *)malloc(sizeof(float) * (s_network_info_h[network_number].out_cf[n].ch_stride * 
                  s_network_info_h[network_number].out_cf[n].ch));
        for(int i = 0; i < (int)(s_network_info_h[network_number].out_cf[n].ch_stride * 
            s_network_info_h[network_number].out_cf[n].ch); i++)
        {
            address[i] = data[i] / scale_values[n];
        }
        
        /*Reshaping each of three outputs to 3D arrays of required sizes*/
        cnt = 0;
        for(int a = 0; a < s_network_info_h[network_number].out_cf[n].ch; a++)
        {
            for(int c = 0; c < s_network_info_h[network_number].out_cf[n].height; c++)
            {
                for(int d = 0; d < s_network_info_h[network_number].out_cf[n].width; d++)
                {
                    reshape_array[a][c][d] = address[cnt];
                    cnt++;
                }
            }
        }
		
		/*Taking transpose of each 3D array as (1, 2, 0)*/
        for(int i = 0; i < s_network_info_h[network_number].out_cf[n].height; i++)
        {
            for(int j = 0; j < s_network_info_h[network_number].out_cf[n].width; j++)
            {
                for(int k = 0; k < s_network_info_h[network_number].out_cf[n].ch; k++)
                {
                    temparr2[i][j][k] = reshape_array[k][i][j];
                }
            }
        }

        /*Converting 3D array to a 1D array*/
        cnt = 0;
        for(int i = 0; i < s_network_info_h[network_number].out_cf[n].height; i++)
        {
            for(int j = 0; j < s_network_info_h[network_number].out_cf[n].width; j++)
            {
                for(int k = 0; k < s_network_info_h[network_number].out_cf[n].ch; k++)
                {
                    temparr3[cnt] = temparr2[i][j][k];
                    cnt++;
                }
            }
        }

        //4-D array
        cnt = 0;
        for(int i = 0; i < grid_h; i++)
        {
            for(int j = 0; j < grid_w; j++)
            {
                for(int k = 0; k < NUM_BB; k++)
                {
                    for(int l = 0; l <  (NUM_CLASS + 5); l++)
                    {
                        final_array[i][j][k][l] = temparr3[cnt];
                        if( (l < 2) || (l > 3))
                        {
                            final_array[i][j][k][l] = sigmoid(final_array[i][j][k][l]);
                        }
                        if(l > 4)
                        {
                            final_array[i][j][k][l] = final_array[i][j][k][l] * final_array[i][j][k][4];
                            if(final_array[i][j][k][l] < TH_PROB)
                            {
                                final_array[i][j][k][l] = 0;
                            }
                        }
                        cnt++;
                    }
                }
            }
        }


        for(int i = 0; i < (grid_w * grid_h); i++)
        {
            int row = i / grid_w;
            int col = i % grid_w;
            for(int b = 0; b < NUM_BB; b++)
            {
                objectness = final_array[row][col][b][4];

                final_array[row][col][b][0] = (final_array[row][col][b][0] + (float)col) / (float)grid_w;
                final_array[row][col][b][1] = (final_array[row][col][b][1] + (float)row) / (float)grid_h;
                final_array[row][col][b][2] = (float)(anch[n].anchors[2 * b + 0] * 
                                                    exp(final_array[row][col][b][2]) / OBJ_WIDTH);
                final_array[row][col][b][3] = (float)(anch[n].anchors[2 * b + 1] * 
                                                    exp(final_array[row][col][b][3]) / OBJ_HEIGHT);
                final_array[row][col][b][0] = (final_array[row][col][b][0] - (OBJ_WIDTH - WIDTH_ADJUST_OBJ) / 2. /
                                               OBJ_WIDTH) / ((float) WIDTH_ADJUST_OBJ / OBJ_WIDTH);
                final_array[row][col][b][1] = (final_array[row][col][b][1] - (OBJ_HEIGHT - HEIGHT_ADJUST_OBJ) / 2. / 
                                               OBJ_HEIGHT) / ((float) HEIGHT_ADJUST_OBJ / OBJ_HEIGHT);
                final_array[row][col][b][2] *= (float) (OBJ_WIDTH / WIDTH_ADJUST_OBJ);
                final_array[row][col][b][3] *= (float) (OBJ_HEIGHT / HEIGHT_ADJUST_OBJ);

                final_array[row][col][b][0]  = round(final_array[row][col][b][0]  * g_customize.Frame_Width);
                final_array[row][col][b][1]  = round(final_array[row][col][b][1]  * g_customize.Frame_Height);
                final_array[row][col][b][2]  = round(final_array[row][col][b][2]  * g_customize.Frame_Width);
                final_array[row][col][b][3]  = round(final_array[row][col][b][3]  * g_customize.Frame_Height);

                for(int j = 0; j < (NUM_CLASS + 5); j++)
                { 
                    od_post_out[no_obj][j] = final_array[row][col][b][j];                    
                }
                no_obj++;
            }
        }
            data = (int8_t *)data + (s_network_info_h[network_number].out_cf[n].ch_stride * 
                    s_network_info_h[network_number].out_cf[n].ch);
            for(int w = 0; w < s_network_info_h[network_number].out_cf[n].ch; w++) 
            {
                for(int r = 0; r < s_network_info_h[network_number].out_cf[n].height; r++) 
                {
                    free(reshape_array[w][r]);
                }
                free(reshape_array[w]);
            }
            free(reshape_array);

            for(int w = 0; w < s_network_info_h[network_number].out_cf[n].height; w++) 
            {
                for(int r = 0; r < s_network_info_h[network_number].out_cf[n].width; r++) 
                {
                    free(temparr2[w][r]);
                }
                free(temparr2[w]);
            }
            free(temparr2);

            free(temparr3);

            for(int w = 0; w < grid_h; w++) 
            {
                for(int r = 0; r < grid_w; r++) 
                {
                    for(int s = 0; s < NUM_BB; s++) 
                    {
                        free(final_array[w][r][s]);
                    }
                    free(final_array[w][r]);
                }
                free(final_array[w]);
            }
            free(final_array);

            free(address);
        }

    obj_det(od_post_out);



    if (g_customize.POSE_EST_Enable == 0)
    {
        e_osal_return_t osal_ret = R_OSAL_MqSendForTimePeriod(g_mq_handle_aiactivity, TIMEOUT_MS, (void *)&det_que, 
                                                                g_mq_config_aiactivity.msg_size);
        if(OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("sending message to detection que MQ was failed, osal_ret = %d\n", osal_ret);
        }
    }

}

/**********************************************************************************************************************
 End of function inferencePostprocess
 *********************************************************************************************************************/

/**********************************************************************************************************************
/* Function Name :  inferencePostprocess_pe */
/******************************************************************************************************************//**
 * @brief       inference post process pose estimation
                [Covers: BPAP_FC_V4H_AD059]
 * @param[in]   data contain inference output 
 * @param[out]  none
 * @retval      None
 ***********************************************************************************************************************/
void inferencePostprocess_pe(signed char * data)
{
    uint32_t det_que = 0;
    int      no_obj  = 0;
    int      cnt     = 0;

    float scale_values[] = {126.30460827144651F, 117.72530575258872F};

    for (int n = 0; n < (int)s_network_info_h_pe[network_number].output_num; n++)
    {
        cnt = s_network_info_h_pe[network_number].out_cf[0].ch_stride * 
            s_network_info_h_pe[network_number].out_cf[0].ch;

        for(int i = 0; i < (int)(s_network_info_h_pe[network_number].out_cf[n].ch_stride *
            s_network_info_h_pe[network_number].out_cf[n].ch); i++)
        {
            if (0 == n)
            {
                pe_array_pafs[i] = data[i] / scale_values[n];
            }

            if (1 == n)
            {
                pe_array_heatmaps[i] = data[cnt+i] / scale_values[n];
            }
        }
    }




    e_osal_return_t osal_ret = R_OSAL_MqSendForTimePeriod(g_mq_handle_aiactivity, TIMEOUT_MS, (void *)&det_que, 
                                                            g_mq_config_aiactivity.msg_size);
    if(OSAL_RETURN_OK != osal_ret)
    {
        PRINT_ERROR("sending message to detection queue MQ was failed, osal_ret = %d\n", osal_ret);
    }

}

/**********************************************************************************************************************
 End of function inferencePostprocess_pe
 *********************************************************************************************************************/

 /**********************************************************************************************************************
/* Function Name :  R_FC_Pre_post */
/******************************************************************************************************************//**
 * @brief       wrapper function for preprocess and post process
                this function is calling from ai_lib to perform pre and post process
                [Covers: BPAP_FC_V4H_AD004][Covers: BPAP_FC_V4H_AD005][Covers: BPAP_FC_V4H_AD014]
                [Covers: BPAP_FC_V4H_AD003][Covers: BPAP_FC_V4H_AD013]
 * @param[in]   inf_work enum to decide pre or post process to perform
 * @param[in / out ]   data  data to set/get fromt or to ai_lib
 * @retval      true
 * @retval      false
 ***********************************************************************************************************************/
bool R_FC_Pre_post(e_ai_pre_post_t inf_work, const int8_t* data)
{
    bool retVal = true;
    static int R_FC_Pre_cnt  = 0 ;
    static int R_FC_Post_cnt = 0 ;
    if(g_customize.SEM_SEG_Enable == 0 && g_customize.OBJ_DET_Enable == 1)
    {
        if(2 != R_FC_Pre_cnt && 2 != R_FC_Post_cnt)
        {
            R_FC_Pre_cnt  = 1;
            R_FC_Post_cnt = 1;
        }
    }

    do
    {
        switch(inf_work)
            {
                case eInfPreProcess:
                    if (R_FC_Pre_cnt == 0 && g_customize.SEM_SEG_Enable == 1)
                    {
                        R_FC_SyncStart(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);   
                        Conv_YUYV2RGB(get_imr_resize_buffer(g_sem_seg_map_ch), gp_ai_rgb_buffer, g_customize.IMR_Resize_Width_Ch_0, 
                                    g_customize.IMR_Resize_Height_Ch_0);
                        inferencePreprocess_ss();
                        if (g_customize.OBJ_DET_Enable == 1 || g_customize.POSE_EST_Enable == 1)
                        {
                            R_FC_Pre_cnt = 1;
                        }
                        else
                        {
                            R_FC_SyncEnd(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);
                        }
                    }
                    else if (R_FC_Pre_cnt == 1 && g_customize.OBJ_DET_Enable == 1)
                    {
                        if (g_customize.SEM_SEG_Enable == 0)
                        {
                            R_FC_SyncStart(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);
                        }
                        Conv_YUYV2RGB(get_imr_resize_buffer(g_obj_det_map_ch), gp_ai_rgb_buffer, OBJ_WIDTH, OBJ_HEIGHT);
                        
                        inferencePreprocess_od();
                        if (g_customize.SEM_SEG_Enable == 1 && g_customize.POSE_EST_Enable == 0)
                        {
                            R_FC_Pre_cnt = 0;
                        }
                        else if(g_customize.POSE_EST_Enable == 1)
                        {
                            R_FC_Pre_cnt = 2;
                        }
                        if (g_customize.POSE_EST_Enable == 0)
                        {
                            R_FC_SyncEnd(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);
                        }
                    }
                    else if (g_customize.POSE_EST_Enable == 1)
                    {

                        if ((g_customize.SEM_SEG_Enable == 0) && (g_customize.OBJ_DET_Enable == 0))
                        {   
                            R_FC_SyncStart(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);
                        }
                        Conv_YUYV2RGB(get_imr_resize_buffer(g_pose_est_map_ch), gp_ai_rgb_buffer, POSE_EST_IMG_WIDTH, POSE_EST_IMG_HEIGHT);
                        inferencePreprocess_pe();
                        if (g_customize.SEM_SEG_Enable == 1)
                        {
                            R_FC_Pre_cnt = 0;
                        }
                        else if (g_customize.SEM_SEG_Enable == 0 && g_customize.OBJ_DET_Enable == 1)
                        {
                            R_FC_Pre_cnt = 1;
                        }

                        R_FC_SyncEnd(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);
                    }
                    
                    if (0 != g_customize.Proc_Time)                           /* If processing time is enabled */
                    {
                        fpsCount(1);                                           /* Calculate inference FPS */
                    }
                    break;

                case eInfPostProcess:
                    if (R_FC_Post_cnt ==0 && g_customize.SEM_SEG_Enable == 1)
                    {
                        inferencePostprocess_ss((signed char*)data);
                        if (g_customize.OBJ_DET_Enable == 1 || g_customize.POSE_EST_Enable == 1)
                        {
                            R_FC_Post_cnt = 1;
                        }
                    }
                    else if (R_FC_Post_cnt ==1 && g_customize.OBJ_DET_Enable == 1)
                    {
                        inferencePostprocess_od((signed char*)data);
                        if (g_customize.SEM_SEG_Enable == 1 && g_customize.POSE_EST_Enable == 0)
                        {
                            R_FC_Post_cnt = 0;
                        }
                        else if(g_customize.POSE_EST_Enable == 1)
                        {

                            R_FC_Post_cnt = 2;
                        }
                        
                    }
                    else if (g_customize.POSE_EST_Enable == 1)
                    {
                        inferencePostprocess_pe((signed char*)data);
                        if (g_customize.SEM_SEG_Enable == 1)
                        {
                            R_FC_Post_cnt = 0;
                        }
                        else if(g_customize.SEM_SEG_Enable == 0 && g_customize.OBJ_DET_Enable == 1 )
                        {
                            R_FC_Post_cnt = 1;
                        }
                    }
                    break;

                default:
                    printf("\r\n Wrong inference action");
                    retVal = false;
                    break;
        }
    } while (0);

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_FC_Pre_post
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/* Function Name : get_imr_resize_buffer */
/**********************************************************************************************************************
 * @brief       Get the resize buffer correspoding to the IMR channel mapped to the AI model
 * @param[in]   channel: AI model mapped IMR channel
 * @param[out]  None
 * @retval      Pointer to the resize buffer
***********************************************************************************************************************/
static unsigned char * get_imr_resize_buffer (int channel)
{
    unsigned char * rs_buffer;

    switch (channel)
    {
        case 0:
            rs_buffer = gp_imr_rs_buffer_ch0;
            break;
        case 1:
            rs_buffer = gp_imr_rs_buffer_ch1;
            break;
        case 2:
            rs_buffer = gp_imr_rs_buffer_ch2;
            break;
        case 3:
            rs_buffer = gp_imr_rs_buffer_ch3;
            break;
        case 4:
            rs_buffer = gp_imr_rs_buffer_ch4;
            break;
        default:
            rs_buffer = gp_imr_rs_buffer_ch0;
            break;
    }

    return rs_buffer;
}
/**********************************************************************************************************************
 End of function get_imr_resize_buffer
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/* Function Name : R_FC_SyncStartWrapperAI */
/**********************************************************************************************************************
 * @brief       wrapper to synchronize with ai library
 * @param[in]   none
 * @param[out]  none
 * @retval      none
***********************************************************************************************************************/
static int R_FC_SyncStartWrapperAI()
{
    R_FC_SyncStart(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);       

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_FC_SyncStartWrapperAI
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/* Function Name : R_FC_SyncEndWrapperAI */
/**********************************************************************************************************************
 * @brief       wrapper to synchronize with ai library
 * @param[in]   none
 * @param[out]  none
 * @retval      none
***********************************************************************************************************************/
static int R_FC_SyncEndWrapperAI()
{
    R_FC_SyncEnd(eAI, &g_mtx_handle_imrrs_out, &g_imr_rs_cond_handle, 0);

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function R_FC_SyncEndWrapperAI
 *********************************************************************************************************************/
/***********************************************************************************************************************
* Function Name: softmax function
                 [Covers: BPAP_FC_V4H_AD004][Covers: BPAP_FC_V4H_AD005][Covers: BPAP_FC_V4H_AD014]
* Description  : Used to represent a probability distribution over a set of possible outcomes. 
* Arguments    : float *x 
*                int n
* Return Value : None
***********************************************************************************************************************/
static void softmax(float *x, int n) 
{
    float max = x[0];
    for (int i = 1; i < n; i++) 
    {
        if (x[i] > max) 
        {
            max = x[i];
        }
    }

    for (int i = 0; i < n; i++)
    {
        x[i] = (float)exp(x[i] - max);
    }

    float sum = 0;
    for (int i = 0; i < n; i++) 
    {
        sum += x[i];
    }

    for (int i = 0; i < n; i++)
    {
        x[i] /= sum;
    }
}
/**********************************************************************************************************************
 * End of function softmax function
 ********************************************************************************************************************/
void Conv_YUYV2RGB(unsigned char * yuyv, unsigned char * bgr, int width, int height)
{
    int z = 0;
    int x;
    int yline;

    for (yline = 0; yline < height; yline++)
    {
        for (x = 0; x < width; x++) 
        {
            int r;
            int g;
            int b;
            int y;
            int u;
            int v;
            if (!z)
            {
                y = yuyv[0] << 8;
            }
            else 
            {
                y = yuyv[2] << 8;
            }
            u = yuyv[1] - 128;                                                  /* U component calculation */
            v = yuyv[3] - 128;                                                  /* V component calculation */
                            
            r = (y + (359 * v)) >> 8;                                           /* Red component calculation */
            g = ((y - (88 * u)) - (183 * v)) >> 8;                              /* Green component calculation */
            b = (y + (454 * u)) >> 8;                                           /* Blue component calculation */
                                                                                /* Combining r, g, b components */
            *(bgr ++) = (unsigned char)((r > 255) ? 255 : ((r < 0) ? 0 : r));
            *(bgr ++) = (unsigned char)((g > 255) ? 255 : ((g < 0) ? 0 : g));
            *(bgr ++) = (unsigned char)((b > 255) ? 255 : ((b < 0) ? 0 : b));

            if (z++)
            {
                z = 0;
                yuyv += 4;
            }
        }
    }
}


//New
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
int R_CDNN_Execute_MMAP() {
    //gp_ai_rgb_buffer, g_customize.IMR_Resize_Width_Ch_0 * g_customize.IMR_Resize_Height_Ch_0 * BPP_RGB
    
    //Do preprocess, yuv2rgb, resize with imr, etc...
    Conv_YUYV2RGB(get_imr_resize_buffer(g_sem_seg_map_ch), gp_ai_rgb_buffer, g_customize.IMR_Resize_Width_Ch_0, g_customize.IMR_Resize_Height_Ch_0);
    //inferencePreprocess_ss();

    //MMAP out the preprocessed image
    int file = open(MMAP_OUT, O_RDWR | O_CREAT | O_TRUNC, (mode_t)0600);
    //lock file
    size_t size = g_customize.IMR_Resize_Width_Ch_0 * g_customize.IMR_Resize_Height_Ch_0 * BPP_RGB;
    if (file == -1) {
        perror("Error opening file");
        return(FAILED);
    }
    if (ftruncate(file, size) == -1) {
        perror("Error truncating file");
        close(file);
        return(FAILED);
    }
    unsigned char *mapped_buffer_out = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, file, 0);
    if (mapped_buffer_out == MAP_FAILED) {
        perror("Error mapping file to memory");
        close(file);
        return(FAILED);
    }
    memcpy(mapped_buffer_out, gp_ai_rgb_buffer, size);
    if (munmap(mapped_buffer_out, size) == -1) {
        perror("Error unmapping file");
        close(file);
        return(FAILED);
    }
    if (munmap(mapped_buffer_out, size) == -1) {
        perror("Error unmapping file");
        return FAILED;
    }
    //unlock file
    close(file);

    //External program mmaps the data in and runs inference. Then it mmaps the data to MMAP_IN



    return SUCCESS;
}