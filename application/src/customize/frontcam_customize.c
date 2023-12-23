/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2022 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : front_cam_customize.c
* Version      : 1.0
* Description  : Customize File
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : Version DD.MM.YYYY Description
*         : 1.00   27.09.2022 First Release 
***********************************************************************************************************************/

/*======================================================================================================================
Includes <System Includes> , "Project Includes"
======================================================================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "common.h"
#include "customize.h"

/**********************************************************************************************************************
 Global variables
 *********************************************************************************************************************/
int g_obj_det_map_ch = -1;
int g_sem_seg_map_ch = -1;
int g_pose_est_map_ch = -1;

/**********************************************************************************************************************
 Private (static) variables and functions
 *********************************************************************************************************************/
char * getstr(char *buffer, char *tag_name, char *dest_buffer);
int CustomizeRangeCheck (char * var, int val, int min, int max);

static int ImrCustomParamCheck (st_customize_t *custom_param);
static void map_imr_ch_to_ai_model (st_customize_t *custom_param, int ch, int ch_width, int ch_height);
static void enable_imr_channel (st_customize_t *custom_param, int channel);

/******************************************************************************************************************/
/* Function Name : getstr */
/******************************************************************************************************************//**
 * @brief       Getting string
                [Covers: BPAP_FC_V4H_AD033]
 * @param[in]   buffer     Buffer of length
 * @param[in]   tag_name     Tag Name
 * @param[out]  dest_buffer
 * @retval      dest_buffer
 ***********************************************************************************************************************/
char * getstr(char *buffer, char *tag_name, char *dest_buffer)
{
    char  tmp[256];
    char  * temp_po;
    int   len;

    len = strlen(tag_name);                                                 /* tag name length */

    if (strncmp(buffer, tag_name, len) == 0)                                /* check for tag */
    {
        buffer = &buffer[len];

        while ((*buffer) == ' ')
        {
            buffer++;                                                       /* skip space */
        }

        if ((*buffer == '\0') || (*buffer == '\n'))
        {
            goto END_OF_FUNC;
        }

        temp_po = tmp;

        if ((*buffer) == '"')
        {
            buffer++;
            while (((*buffer) != '"') && ((*buffer) != '\0') && ((*buffer) != '\n'))
            {
                *temp_po++ = *buffer++;
            }
            if ((*buffer) != '"')
            {
                goto END_OF_FUNC;
            }
        } 
        else
        {
            while (((*buffer) != ' ') && ((*buffer) != '\0') && ((*buffer) != '\n'))
            {
                *temp_po++ = *buffer++;
            }
        }
        *temp_po = '\0';
        strcpy(dest_buffer, tmp);
    }

END_OF_FUNC:

    return (dest_buffer);
}
/**********************************************************************************************************************
 End of function  getstr
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/* Function Name : R_CustomizeInit */
/**********************************************************************************************************************
 * @brief       default customization value
                [Covers: BPAP_FC_V4H_AD033]
 * @param[in]   none
 * @param[out]  custom_param      Customization pointer
 * @retval      none
 ***********************************************************************************************************************/
void R_CustomizeInit(st_customize_t *custom_param)
{
    memset(custom_param, 0, sizeof(st_customize_t)); 

    /* VIN */
    custom_param->VIN_Enable                  = 0;
    custom_param->VIN_Device                  = 1;
    custom_param->VIN_Offset_X                = 0;
    custom_param->VIN_Offset_Y                = 0;
    custom_param->VIN_Req_Buffer_Num          = 4;
    custom_param->VIN_Capture_Format          = 3;
    custom_param->Max_Camera_Width            = 1920;
    custom_param->Max_Camera_Height           = 1020;

    /* Image */
    custom_param->Frame_Width                 = 1280;
    custom_param->Frame_Height                = 720;


    /* ISP */
    custom_param->ISP_Enable                  = 0;
    custom_param->ISP_Channel                 = 0;
    custom_param->ISP_RAW_IN_Format           = 1;
    custom_param->ISP_RAW_OUT_Format          = 0;

    /* IMR */
    custom_param->IMR_Channel                 = 0;
    custom_param->IMR_LDC                     = 0;
    custom_param->IMR_LDC_Params_k1           = 0.000023f;
    custom_param->IMR_LDC_Params_k2           = 0;
    custom_param->IMR_LDC_Params_k3           = 0;
    custom_param->IMR_LDC_Params_p1           = 0;
    custom_param->IMR_LDC_Params_p2           = 0;
    custom_param->IMR_LDC_Params_fx           = 0.3f;
    custom_param->IMR_LDC_Params_fy           = 0.5f;
    custom_param->IMR_LDC_Params_cx           = 0.4f;
    custom_param->IMR_LDC_Params_cy           = 0.4f;
    custom_param->IMR_Resize                  = 1;
    custom_param->IMR_Ch_0_Enable             = 1;
    custom_param->IMR_Ch_1_Enable             = 1;
    custom_param->IMR_Ch_2_Enable             = 1;
    custom_param->IMR_Ch_3_Enable             = 0;
    custom_param->IMR_Ch_4_Enable             = 0;
    custom_param->IMR_Resize_Width_Ch_0       = 256;
    custom_param->IMR_Resize_Height_Ch_0      = 256;
    custom_param->IMR_Resize_Width_Ch_1       = 320;
    custom_param->IMR_Resize_Height_Ch_1      = 320;
    custom_param->IMR_Resize_Width_Ch_2       = 224;
    custom_param->IMR_Resize_Height_Ch_2      = 224;
    custom_param->IMR_Resize_Width_Ch_3       = 0;
    custom_param->IMR_Resize_Height_Ch_3      = 0;
    custom_param->IMR_Resize_Width_Ch_4       = 0;
    custom_param->IMR_Resize_Height_Ch_4      = 0;

    /* VOUT */
    custom_param->VOUT_Enable                 = 0;
    strcpy(custom_param->DRM_Module, "rcar-du");
    custom_param->VOUT_Display_Format         = 2;
    custom_param->VOUT_Display_Width          = 1920;
    custom_param->VOUT_Display_Height         = 1080;
    custom_param->VOUT_Pos_X                  = 0;
    custom_param->VOUT_Pos_Y                  = 0;

    /* Debug */
    custom_param->Debug_Enable                = 0;
    custom_param->Proc_Time                   = 1;
    custom_param->CPU_Load_Enable             = 1;
    strcpy(custom_param->Frame_File_Name, "frame_buffer_vin");
    custom_param->Image_Folder_Enable         = 1;
    strcpy(custom_param->Frame_Folder_Name, "Test_Images");

#if(CDNN)
    /* CDNN */
    custom_param->CDNN_Enable                 = 1;
    custom_param->OBJ_DET_Enable              = 1;
    custom_param->SEM_SEG_Enable              = 1;
    custom_param->POSE_EST_Enable             = 1;
    custom_param->CDNN_Load_Enable            = 0;
#endif
    custom_param->OBJ_DET_Width               = 320;
    custom_param->OBJ_DET_Height              = 320;
    custom_param->SEM_SEG_Width               = 256;
    custom_param->SEM_SEG_Height              = 256;
    custom_param->POSE_EST_Width              = 224;
    custom_param->POSE_EST_Height             = 224;
//New
    custom_param->MMAP_Contingency_enable     = 0;
}
/**********************************************************************************************************************
 End of function  R_CustomizeInit
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/* Function Name : R_CustomizeLoad */
/**********************************************************************************************************************
 * @brief       Loading the customization parameters
                [Covers: BPAP_FC_V4H_AD033]
 * @param[in]   custom_param              Customization pointer
 * @param[in]   file_name       Name of the file
 * @retval      true            success
 * @retval      false           fail
 *********************************************************************************************************************/
int R_CustomizeLoad(st_customize_t *custom_param, const char *file_name)
{
    int ret = 0;
    char buf[258];
    FILE * fp = NULL;
    fp = fopen(file_name, "r");
    if (fp == NULL)
    {
        return FAILED;
    }

    while (fgets(buf, 256, fp) != NULL)
    {
        sscanf(buf, "VIN_Enable %d", &custom_param->VIN_Enable);
        sscanf(buf, "VIN_Device %d", &custom_param->VIN_Device);
        sscanf(buf, "VIN_Capture_Format %d", &custom_param->VIN_Capture_Format);
        sscanf(buf, "VIN_Offset_X %d", &custom_param->VIN_Offset_X);
        sscanf(buf, "VIN_Offset_Y %d", &custom_param->VIN_Offset_Y);
        sscanf(buf, "VIN_Req_Buffer_Num %d", &custom_param->VIN_Req_Buffer_Num);
        sscanf(buf, "Frame_Width %d", &custom_param->Frame_Width);
        sscanf(buf, "Frame_Height %d", &custom_param->Frame_Height);
        sscanf(buf, "ISP_Enable %d", &custom_param->ISP_Enable);
        sscanf(buf, "ISP_Channel %d", &custom_param->ISP_Channel);
        sscanf(buf, "ISP_RAW_IN_Format %d", &custom_param->ISP_RAW_IN_Format);
        sscanf(buf, "ISP_RAW_OUT_Format %d", &custom_param->ISP_RAW_OUT_Format);
        sscanf(buf, "IMR_Channel %d", &custom_param->IMR_Channel);
        sscanf(buf, "IMR_LDC %d", &custom_param->IMR_LDC);
        sscanf(buf, "IMR_LDC_Params_k1 %f", &custom_param->IMR_LDC_Params_k1);
        sscanf(buf, "IMR_LDC_Params_k2 %f", &custom_param->IMR_LDC_Params_k2);
        sscanf(buf, "IMR_LDC_Params_k3 %f", &custom_param->IMR_LDC_Params_k3);
        sscanf(buf, "IMR_LDC_Params_p1 %f", &custom_param->IMR_LDC_Params_p1);
        sscanf(buf, "IMR_LDC_Params_p2 %f", &custom_param->IMR_LDC_Params_p2);
        sscanf(buf, "IMR_LDC_Params_fx %f", &custom_param->IMR_LDC_Params_fx);
        sscanf(buf, "IMR_LDC_Params_fy %f", &custom_param->IMR_LDC_Params_fy);
        sscanf(buf, "IMR_LDC_Params_cx %f", &custom_param->IMR_LDC_Params_cx);
        sscanf(buf, "IMR_LDC_Params_cy %f", &custom_param->IMR_LDC_Params_cy);
        sscanf(buf, "IMR_Resize %d", &custom_param->IMR_Resize);
        sscanf(buf, "IMR_Ch_0_Enable %d", &custom_param->IMR_Ch_0_Enable);
        sscanf(buf, "IMR_Ch_1_Enable %d", &custom_param->IMR_Ch_1_Enable);
        sscanf(buf, "IMR_Ch_2_Enable %d", &custom_param->IMR_Ch_2_Enable);
        sscanf(buf, "IMR_Ch_3_Enable %d", &custom_param->IMR_Ch_3_Enable);
        sscanf(buf, "IMR_Ch_4_Enable %d", &custom_param->IMR_Ch_4_Enable);
        sscanf(buf, "IMR_Resize_Width_Ch_0 %d", &custom_param->IMR_Resize_Width_Ch_0);
        sscanf(buf, "IMR_Resize_Height_Ch_0 %d", &custom_param->IMR_Resize_Height_Ch_0);
        sscanf(buf, "IMR_Resize_Width_Ch_1 %d", &custom_param->IMR_Resize_Width_Ch_1);
        sscanf(buf, "IMR_Resize_Height_Ch_1 %d", &custom_param->IMR_Resize_Height_Ch_1);
        sscanf(buf, "IMR_Resize_Width_Ch_2 %d", &custom_param->IMR_Resize_Width_Ch_2);
        sscanf(buf, "IMR_Resize_Height_Ch_2 %d", &custom_param->IMR_Resize_Height_Ch_2);
        sscanf(buf, "IMR_Resize_Width_Ch_3 %d", &custom_param->IMR_Resize_Width_Ch_3);
        sscanf(buf, "IMR_Resize_Height_Ch_3 %d", &custom_param->IMR_Resize_Height_Ch_3);
        sscanf(buf, "IMR_Resize_Width_Ch_4 %d", &custom_param->IMR_Resize_Width_Ch_4);
        sscanf(buf, "IMR_Resize_Height_Ch_4 %d", &custom_param->IMR_Resize_Height_Ch_4);
        sscanf(buf, "VOUT_Enable %d", &custom_param->VOUT_Enable);
        getstr(buf, "DRM_Module", custom_param->DRM_Module);
        sscanf(buf, "VOUT_Display_Format %d", &custom_param->VOUT_Display_Format);
        sscanf(buf, "VOUT_Display_Width %d", &custom_param->VOUT_Display_Width);
        sscanf(buf, "VOUT_Display_Height %d", &custom_param->VOUT_Display_Height);
        sscanf(buf, "VOUT_Pos_X %d", &custom_param->VOUT_Pos_X);
        sscanf(buf, "VOUT_Pos_Y %d", &custom_param->VOUT_Pos_Y);
        sscanf(buf, "Debug_Enable %d", &custom_param->Debug_Enable);
        sscanf(buf, "Logging %d", &custom_param->Logging);
        sscanf(buf, "Proc_Time %d", &custom_param->Proc_Time);
        sscanf(buf, "CPU_Load_Enable %d", &custom_param->CPU_Load_Enable);
        getstr(buf, "Frame_File_Name ", custom_param->Frame_File_Name);
        sscanf(buf, "Image_Folder_Enable %d", &custom_param->Image_Folder_Enable);
        getstr(buf, "Frame_Folder_Name ", custom_param->Frame_Folder_Name);
        
        sscanf(buf, "Max_Camera_Width %d", &custom_param->Max_Camera_Width);
        sscanf(buf, "Max_Camera_Height %d", &custom_param->Max_Camera_Height);
#if(CDNN)  
        sscanf(buf, "CDNN_Enable %d", &custom_param->CDNN_Enable);
        sscanf(buf, "OBJ_DET_Enable %d", &custom_param->OBJ_DET_Enable);    
        sscanf(buf, "SEM_SEG_Enable %d", &custom_param->SEM_SEG_Enable);
        sscanf(buf, "POSE_EST_Enable %d", &custom_param->POSE_EST_Enable);
        sscanf(buf, "CDNN_Load_Enable %d", &custom_param->CDNN_Load_Enable);
#endif
        sscanf(buf, "OBJ_DET_Width %d", &custom_param->OBJ_DET_Width);
        sscanf(buf, "OBJ_DET_Height %d", &custom_param->OBJ_DET_Height);
        sscanf(buf, "SEM_SEG_Width %d", &custom_param->SEM_SEG_Width);
        sscanf(buf, "SEM_SEG_Height %d", &custom_param->SEM_SEG_Height);
        sscanf(buf, "POSE_EST_Width %d", &custom_param->POSE_EST_Width);
        sscanf(buf, "POSE_EST_Height %d", &custom_param->POSE_EST_Height);
        //New
        sscanf(buf, "MMAP_Contingency_enable %d", &custom_param->MMAP_Contingency_enable);
    }


    return SUCCESS;
}
/**********************************************************************************************************************
 End of function  R_CustomizeLoad
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/* Function Name :R_CustomizeValidate */
/**********************************************************************************************************************
 * @brief       Validating the customization parameters
                [Covers: BPAP_FC_V4H_AD033][Covers: BPAP_FC_V4H_AD047]
 * @param[in]   custom_param              Customization pointer
 * @param[out]  none
 * @retval      true            success
 * @retval      false           fail
 *********************************************************************************************************************/
int R_CustomizeValidate(st_customize_t *custom_param)
{
    int ret =0;

    ret += CustomizeRangeCheck("VIN_Enable", custom_param->VIN_Enable, 0, 1);
    ret += CustomizeRangeCheck("VIN_Device", custom_param->VIN_Device, 0, 7);
    ret += CustomizeRangeCheck("VIN_Capture_Format", custom_param->VIN_Capture_Format, 0, 3);
    ret += CustomizeRangeCheck("VIN_Req_Buffer_Num", custom_param->VIN_Req_Buffer_Num, 2, 4);
    ret += CustomizeRangeCheck("VIN_Offset_X", custom_param->VIN_Offset_X, 0, 
                                (custom_param->Max_Camera_Width - custom_param->Frame_Width));
    ret += CustomizeRangeCheck("VIN_Offset_Y", custom_param->VIN_Offset_Y, 0, 
                                (custom_param->Max_Camera_Height - custom_param->Frame_Height));
    ret += CustomizeRangeCheck("Max_Camera_Width", custom_param->Max_Camera_Width, 0, 1920);
    ret += CustomizeRangeCheck("Max_Camera_Height", custom_param->Max_Camera_Height, 0, 1020);
    ret += CustomizeRangeCheck("ISP_Enable", custom_param->ISP_Enable, 0, 1);
    ret += CustomizeRangeCheck("ISP_Channel", custom_param->ISP_Channel, 0, 1);
    ret += CustomizeRangeCheck("ISP_RAW_IN_Format", custom_param->ISP_RAW_IN_Format, 0, 1);
    ret += CustomizeRangeCheck("IMR_Channel", custom_param->IMR_Channel, 0, 4);
    ret += CustomizeRangeCheck("IMR_LDC", custom_param->IMR_LDC, 0, 1);
    ret += CustomizeRangeCheck("IMR_Resize", custom_param->IMR_Resize, 0, 1);
    ret += CustomizeRangeCheck("VOUT_Enable", custom_param->VOUT_Enable, 0, 1);
    ret += CustomizeRangeCheck("VOUT_Display_Format", custom_param->VOUT_Display_Format, 0, 2);
    ret += CustomizeRangeCheck("VOUT_Pos_X", custom_param->VOUT_Pos_X, 0, custom_param->VOUT_Display_Width);
    ret += CustomizeRangeCheck("VOUT_Pos_Y", custom_param->VOUT_Pos_Y, 0, custom_param->VOUT_Display_Height);
    ret += CustomizeRangeCheck("Debug_Enable", custom_param->Debug_Enable, 0, 1);
    ret += CustomizeRangeCheck("Logging", custom_param->Logging, 0, 1);
    ret += CustomizeRangeCheck("Proc_Time", custom_param->Proc_Time, 0, 1);
    ret += CustomizeRangeCheck("CPU_Load_Enable", custom_param->CPU_Load_Enable, 0, 1);
    ret += CustomizeRangeCheck("Image_Folder_Enable", custom_param->Image_Folder_Enable, 0, 1);
    ret += CustomizeRangeCheck("IMR_Ch_0_Enable", custom_param->IMR_Ch_0_Enable, 0, 1);
    ret += CustomizeRangeCheck("IMR_Ch_1_Enable", custom_param->IMR_Ch_1_Enable, 0, 1);
    ret += CustomizeRangeCheck("IMR_Ch_2_Enable", custom_param->IMR_Ch_2_Enable, 0, 1);
    ret += CustomizeRangeCheck("IMR_Ch_3_Enable", custom_param->IMR_Ch_3_Enable, 0, 1);
    ret += CustomizeRangeCheck("IMR_Ch_4_Enable", custom_param->IMR_Ch_4_Enable, 0, 1);
    if(true == g_customize.IMR_Ch_0_Enable)
    {
        ret += CustomizeRangeCheck("IMR_Resize_Width_Ch_0", custom_param->IMR_Resize_Width_Ch_0, 1, custom_param->Max_Camera_Width);
        ret += CustomizeRangeCheck("IMR_Resize_Height_Ch_0", custom_param->IMR_Resize_Height_Ch_0, 1, custom_param->Max_Camera_Height);
    }
    if(true == g_customize.IMR_Ch_1_Enable)
    {
        ret += CustomizeRangeCheck("IMR_Resize_Width_Ch_1", custom_param->IMR_Resize_Width_Ch_1, 1, custom_param->Max_Camera_Width);
        ret += CustomizeRangeCheck("IMR_Resize_Height_Ch_1", custom_param->IMR_Resize_Height_Ch_1, 1, custom_param->Max_Camera_Height);
    }
    if(true == g_customize.IMR_Ch_2_Enable)
    {
        ret += CustomizeRangeCheck("IMR_Resize_Width_Ch_2", custom_param->IMR_Resize_Width_Ch_2, 1, custom_param->Max_Camera_Width);
        ret += CustomizeRangeCheck("IMR_Resize_Height_Ch_2", custom_param->IMR_Resize_Height_Ch_2, 1, custom_param->Max_Camera_Height);
    }
    if(true == g_customize.IMR_Ch_3_Enable)
    {
        ret += CustomizeRangeCheck("IMR_Resize_Width_Ch_3", custom_param->IMR_Resize_Width_Ch_3, 1, custom_param->Max_Camera_Width);
        ret += CustomizeRangeCheck("IMR_Resize_Height_Ch_3", custom_param->IMR_Resize_Height_Ch_3, 1, custom_param->Max_Camera_Height);
    }
    if(true == g_customize.IMR_Ch_4_Enable)
    {
        ret += CustomizeRangeCheck("IMR_Resize_Width_Ch_4", custom_param->IMR_Resize_Width_Ch_4, 1, custom_param->Max_Camera_Width);
        ret += CustomizeRangeCheck("IMR_Resize_Height_Ch_4", custom_param->IMR_Resize_Height_Ch_4, 1, custom_param->Max_Camera_Height);
    }

#if(CDNN)
    ret += CustomizeRangeCheck("CDNN_Enable", custom_param->CDNN_Enable, 0, 1);
    ret += CustomizeRangeCheck("OBJ_DET_Enable", custom_param->OBJ_DET_Enable, 0, 1);
    ret += CustomizeRangeCheck("SEM_SEG_Enable", custom_param->SEM_SEG_Enable, 0, 1);
    ret += CustomizeRangeCheck("POSE_EST_Enable", custom_param->POSE_EST_Enable, 0, 1);
    ret += CustomizeRangeCheck("CDNN_Load_Enable", custom_param->CDNN_Load_Enable, 0, 1);
#endif
    ret += CustomizeRangeCheck("Frame_Width", custom_param->Frame_Width, 0, custom_param->Max_Camera_Width);
    ret += CustomizeRangeCheck("Frame_Height", custom_param->Frame_Height, 0, custom_param->Max_Camera_Height);
    ret += CustomizeRangeCheck("OBJ_DET_Width", custom_param->OBJ_DET_Width, 0, custom_param->Frame_Width);
    ret += CustomizeRangeCheck("OBJ_DET_Height", custom_param->OBJ_DET_Height, 0, custom_param->Frame_Height);
    ret += CustomizeRangeCheck("SEM_SEG_Width", custom_param->SEM_SEG_Width, 0, custom_param->Frame_Width);
    ret += CustomizeRangeCheck("SEM_SEG_Height", custom_param->SEM_SEG_Height, 0, custom_param->Frame_Height);
    ret += CustomizeRangeCheck("POSE_EST_Width", custom_param->POSE_EST_Width, 0, custom_param->Frame_Width);
    ret += CustomizeRangeCheck("POSE_EST_Height", custom_param->POSE_EST_Height, 0, custom_param->Frame_Height);

    ret += ImrCustomParamCheck(custom_param);

    if(!((custom_param->VOUT_Display_Width == 1920 && custom_param->VOUT_Display_Height == 1080) || 
        (custom_param->VOUT_Display_Width == 1600 && custom_param->VOUT_Display_Height == 900)))
    {
        PRINT_WARNING("Currently for V4H2 1920*1080 and 1600*900 are supported display resolutions.\n");
        return FAILED;
    }

    if (ret == 0)
    {
        return SUCCESS;
    }
    else
    {
        return FAILED;
    }

}
/**********************************************************************************************************************
 End of function  R_CustomizeValidate
 *********************************************************************************************************************/

/******************************************************************************************************************/
/* Function Name : R_CustomizePrint*/
/******************************************************************************************************************//**
 * @brief       Printing the customization parameters
                [Covers: BPAP_FC_V4H_AD033]
 * @param[in]   custom_param              Customization Pointer
 * @param[out]  none
 * @retval      true            success
 * @retval      false           fail
 *********************************************************************************************************************/
int R_CustomizePrint(st_customize_t *custom_param)
{

printf("FC V4H PIPE-LINE \n");
    printf("-------------------------------\n");

    if (true == custom_param->VIN_Enable)
    {
        printf("|VIN| --> ");
    }
    if (true == custom_param->ISP_Enable)
    {
        printf("|ISP| --> ");
    }
    if (true == custom_param->IMR_LDC)
    {
        printf("|IMR LDC| --> ");
    }
    if (true == custom_param->IMR_Resize)
    {
        printf("|IMR RES| --> ");
    }
#if(CDNN)
    if (true == custom_param->CDNN_Enable)
    {
        printf("|CDNN| --> ");
    }
#endif
    if (true == custom_param->VOUT_Enable)
    {
        printf("|VOUT| \n \n");
    }

    printf("FC Configuration \n");
    printf("-------------------------------\n");
    printf("\n[VIN] \n");
    printf("VIN_Enable                  : %d \n", custom_param->VIN_Enable);
    printf("VIN_Device                  : %d \n", custom_param->VIN_Device);
    printf("VIN_Capture_Format          : %d \n", custom_param->VIN_Capture_Format);
    printf("VIN_Offset_X                : %d \n", custom_param->VIN_Offset_X);
    printf("VIN_Offset_Y                : %d \n", custom_param->VIN_Offset_Y);
    printf("VIN_Req_Buffer_Num          : %d \n", custom_param->VIN_Req_Buffer_Num);
    printf("\n[Camera] \n");
    printf("Max_Camera_Width            : %d \n", custom_param->Max_Camera_Width);
    printf("Max_Camera_Height           : %d \n", custom_param->Max_Camera_Height);
    printf("\n[Pipeline Resolution] \n");
    printf("Frame_Width                 : %d \n", custom_param->Frame_Width);
    printf("Frame_Height                : %d \n", custom_param->Frame_Height);
    printf("\n[ISP] \n");
    printf("ISP_Enable                  : %d \n", custom_param->ISP_Enable);
    printf("ISP_Channel                 : %d \n", custom_param->ISP_Channel);
    printf("ISP_RAW_IN_Format           : %d \n", custom_param->ISP_RAW_IN_Format);
    printf("\n[IMR] \n");
    printf("IMR_Channel                 : %d \n", custom_param->IMR_Channel);
    printf("IMR_LDC                     : %d \n", custom_param->IMR_LDC);
    printf("IMR_LDC_Params_k1           : %f \n", custom_param->IMR_LDC_Params_k1);
    printf("IMR_LDC_Params_k2           : %f \n", custom_param->IMR_LDC_Params_k2);
    printf("IMR_LDC_Params_k3           : %f \n", custom_param->IMR_LDC_Params_k3);
    printf("IMR_LDC_Params_p1           : %f \n", custom_param->IMR_LDC_Params_p1);
    printf("IMR_LDC_Params_p2           : %f \n", custom_param->IMR_LDC_Params_p2);
    printf("IMR_LDC_Params_fx           : %f \n", custom_param->IMR_LDC_Params_fx);
    printf("IMR_LDC_Params_fy           : %f \n", custom_param->IMR_LDC_Params_fy);
    printf("IMR_LDC_Params_cx           : %f \n", custom_param->IMR_LDC_Params_cx);
    printf("IMR_LDC_Params_cy           : %f \n", custom_param->IMR_LDC_Params_cy);
    printf("IMR_Resize                  : %d \n", custom_param->IMR_Resize);
    printf("IMR_Ch_0_Enable             : %d \n", custom_param->IMR_Ch_0_Enable);
    printf("IMR_Ch_1_Enable             : %d \n", custom_param->IMR_Ch_1_Enable);
    printf("IMR_Ch_2_Enable             : %d \n", custom_param->IMR_Ch_2_Enable);
    printf("IMR_Ch_3_Enable             : %d \n", custom_param->IMR_Ch_3_Enable);
    printf("IMR_Ch_4_Enable             : %d \n", custom_param->IMR_Ch_4_Enable);
    printf("IMR_Resize_Width_Ch_0       : %d \n", custom_param->IMR_Resize_Width_Ch_0);
    printf("IMR_Resize_Height_Ch_0      : %d \n", custom_param->IMR_Resize_Height_Ch_0);
    printf("IMR_Resize_Width_Ch_1       : %d \n", custom_param->IMR_Resize_Width_Ch_1);
    printf("IMR_Resize_Height_Ch_1      : %d \n", custom_param->IMR_Resize_Height_Ch_1);
    printf("IMR_Resize_Width_Ch_2       : %d \n", custom_param->IMR_Resize_Width_Ch_2);
    printf("IMR_Resize_Height_Ch_2      : %d \n", custom_param->IMR_Resize_Height_Ch_2);
    printf("IMR_Resize_Width_Ch_3       : %d \n", custom_param->IMR_Resize_Width_Ch_3);
    printf("IMR_Resize_Height_Ch_3      : %d \n", custom_param->IMR_Resize_Height_Ch_3);
    printf("IMR_Resize_Width_Ch_4       : %d \n", custom_param->IMR_Resize_Width_Ch_4);
    printf("IMR_Resize_Height_Ch_4      : %d \n", custom_param->IMR_Resize_Height_Ch_4);

#if(CDNN)  
    printf("\n[CDNN] \n");
    printf("CDNN_Enable                 : %d \n", custom_param->CDNN_Enable);
    printf("OBJ_DET_Enable              : %d \n", custom_param->OBJ_DET_Enable);    
    printf("SEM_SEG_Enable              : %d \n", custom_param->SEM_SEG_Enable);
    printf("POSE_EST_Enable             : %d \n", custom_param->POSE_EST_Enable);
    printf("CDNN_Load_Enable            : %d \n", custom_param->CDNN_Load_Enable);
#endif
    printf("\n[VOUT] \n");
    printf("VOUT_Enable                 : %d \n", custom_param->VOUT_Enable);
    printf("DRM_Module                  : %s \n", custom_param->DRM_Module);
    printf("VOUT_Display_Format         : %d \n", custom_param->VOUT_Display_Format);
    printf("VOUT_Display_Width          : %d \n", custom_param->VOUT_Display_Width);
    printf("VOUT_Display_Height         : %d \n", custom_param->VOUT_Display_Height);
    printf("VOUT_Pos_X                  : %d \n", custom_param->VOUT_Pos_X);
    printf("VOUT_Pos_Y                  : %d \n", custom_param->VOUT_Pos_Y);
    printf("\n[DEBUG] \n");
    printf("Debug_Enable                : %d \n", custom_param->Debug_Enable);
    printf("Proc_Time                   : %d \n", custom_param->Proc_Time);
    printf("\n[CPULOAD] \n");
    printf("CPU_Load_Enable             : %d \n", custom_param->CPU_Load_Enable);
    printf("\n[File] \n");
    printf("Frame_File_Name             : %s \n", custom_param->Frame_File_Name);
    printf("Image_Folder_Enable         : %d \n", custom_param->Image_Folder_Enable);
    printf("Frame_Folder_Name           : %s \n", custom_param->Frame_Folder_Name);
    printf("OBJ_DET_Width               : %d \n", custom_param->OBJ_DET_Width);
    printf("OBJ_DET_Height              : %d \n", custom_param->OBJ_DET_Height);    
    printf("SEM_SEG_Width               : %d \n", custom_param->SEM_SEG_Width);
    printf("SEM_SEG_Height              : %d \n", custom_param->SEM_SEG_Height);
    printf("POSE_EST_Width              : %d \n", custom_param->POSE_EST_Width);
    printf("POSE_EST_Height             : %d \n", custom_param->POSE_EST_Height);
    //new
    printf("MMAP_Contingency_enable     : %d \n", custom_param->MMAP_Contingency_enable);
    printf("-------------------------------\n");
    return 0;

    return 0;
}
/**********************************************************************************************************************
 End of function  R_CustomizePrint
 *********************************************************************************************************************/

/******************************************************************************************************************/
/* Function Name : CustomizeRangeCheck*/
/******************************************************************************************************************//**
 * @brief       Checking the range
 * @param[in]   var      variable
 * @param[in]   val      Parameter value
 * @param[in]   min      Minimum value
 * @param[in]   max      Maximum value
 * @param[out]  none
 * @retval      true     success (0)
 * @retval      false    failure (1)
 *********************************************************************************************************************/                   
int CustomizeRangeCheck(char * var, int val, int min, int max)
{
    int ret = (val < min) ? (FAILED):((val > max) ? (FAILED):(SUCCESS));
    if (ret == 1)
    {
        printf ("Customize Validation failed for %s \n", var);
    }
    return ret;
}
/**********************************************************************************************************************
 End of function CustomizeRangeCheck
 *********************************************************************************************************************/

/******************************************************************************************************************/
/* Function Name : ImrCustomParamCheck*/
/******************************************************************************************************************//**
 * @brief           Checks the IMR and AI model parameter for mapping them 
 * @param[in-out]   custom_param    Customization Pointer
 * @param[out]      none
 * @retval          true            success (0)
 * @retval          false           failure (1)
 *********************************************************************************************************************/                   
static int ImrCustomParamCheck (st_customize_t *custom_param)
{

    int ret;

    if (custom_param->IMR_Resize)
    {
        if (!(custom_param->IMR_Ch_0_Enable | custom_param->IMR_Ch_1_Enable | custom_param->IMR_Ch_2_Enable |
          custom_param->IMR_Ch_3_Enable | custom_param->IMR_Ch_4_Enable))
        {
             return FAILED;
        }
#if (CDNN)
        if (!(custom_param->OBJ_DET_Enable | custom_param->SEM_SEG_Enable | custom_param->POSE_EST_Enable))
        {
            custom_param->IMR_Resize = 0;
            return SUCCESS;
        }
#endif
    }
    
#if (CDNN)
    if (((1 == custom_param->OBJ_DET_Enable)  && ((0 == custom_param->OBJ_DET_Width)  
        || (0 == custom_param->OBJ_DET_Height))) ||((1 == custom_param->SEM_SEG_Enable)  && 
        ((0 == custom_param->SEM_SEG_Width)  || (0 == custom_param->SEM_SEG_Height))) ||
        ((1 == custom_param->POSE_EST_Enable) && ((0 == custom_param->POSE_EST_Width) || 
        (0 == custom_param->POSE_EST_Height))))
#else
    if (((0 == custom_param->OBJ_DET_Width)  || (0 == custom_param->OBJ_DET_Height)) ||
        ((0 == custom_param->SEM_SEG_Width)  || (0 == custom_param->SEM_SEG_Height)) ||
        ((0 == custom_param->POSE_EST_Width) || (0 == custom_param->POSE_EST_Height)))
#endif
    {
        printf ("Invalid AI model paramaters\n");
        return FAILED;
    }

    if (custom_param->IMR_Ch_0_Enable)
    {
        map_imr_ch_to_ai_model(custom_param, 0, custom_param->IMR_Resize_Width_Ch_0, custom_param->IMR_Resize_Height_Ch_0);
    }

    if (custom_param->IMR_Ch_1_Enable)
    {
        map_imr_ch_to_ai_model(custom_param, 1, custom_param->IMR_Resize_Width_Ch_1, custom_param->IMR_Resize_Height_Ch_1);
    }

    if (custom_param->IMR_Ch_2_Enable)
    {
        map_imr_ch_to_ai_model(custom_param, 2, custom_param->IMR_Resize_Width_Ch_2, custom_param->IMR_Resize_Height_Ch_2);
    }

    if (custom_param->IMR_Ch_3_Enable)
    {
        map_imr_ch_to_ai_model(custom_param, 3, custom_param->IMR_Resize_Width_Ch_3, custom_param->IMR_Resize_Height_Ch_3);
    }

    if (custom_param->IMR_Ch_4_Enable)
    {
        map_imr_ch_to_ai_model(custom_param, 4, custom_param->IMR_Resize_Width_Ch_4, custom_param->IMR_Resize_Height_Ch_4);
    }

#if (CDNN)
    if (((1 == custom_param->OBJ_DET_Enable)  && (-1 == g_obj_det_map_ch)) || 
        ((1 == custom_param->SEM_SEG_Enable)  && (-1 == g_sem_seg_map_ch)) ||
        ((1 == custom_param->POSE_EST_Enable) && (1 == g_pose_est_map_ch)))
#else
    if ((-1 == g_obj_det_map_ch) || 
        (-1 == g_sem_seg_map_ch) ||
        (-1 == g_pose_est_map_ch))
#endif
    {
        return FAILED;
    }

    custom_param->IMR_Ch_0_Enable = 0;
    custom_param->IMR_Ch_1_Enable = 0;
    custom_param->IMR_Ch_2_Enable = 0;
    custom_param->IMR_Ch_3_Enable = 0;
    custom_param->IMR_Ch_4_Enable = 0;

    enable_imr_channel (custom_param, g_obj_det_map_ch);
    enable_imr_channel (custom_param, g_sem_seg_map_ch);
    enable_imr_channel (custom_param, g_pose_est_map_ch);

    return SUCCESS;
}
/**********************************************************************************************************************
 End of function ImrCustomParamCheck
 *********************************************************************************************************************/

/******************************************************************************************************************/
/* Function Name : map_imr_ch_to_ai_model*/
/******************************************************************************************************************//**
 * @brief       Map thr enabled IMR channel to AI model with compatible width and height
 * @param[in]   custom_param    Customization Pointer
 * @param[in]   ch              IMR channel
 * @param[in]   ch_width        IMR resize width
 * @param[in]   ch_height       IMR resize height
 * @param[out]  none
 * @retval      None
 *********************************************************************************************************************/                   
static void map_imr_ch_to_ai_model (st_customize_t *custom_param, int ch, int ch_width, int ch_height)
{
    if (0 != ch_width && 0 != ch_height) 
    {
#if (CDNN)
        if (custom_param->OBJ_DET_Enable && (-1 == g_obj_det_map_ch))
#else
        if ((custom_param->OBJ_DET_Width > 0) && (custom_param->OBJ_DET_Height > 0) && (-1 == g_obj_det_map_ch))
#endif
        {
            if (ch_width == custom_param->OBJ_DET_Width && ch_height == custom_param->OBJ_DET_Height)
            {
                g_obj_det_map_ch = ch;
            }
        }

#if (CDNN)
        if (custom_param->SEM_SEG_Enable && (-1 == g_sem_seg_map_ch))
#else
        if ((custom_param->SEM_SEG_Width > 0) && (custom_param->SEM_SEG_Height > 0) && (-1 == g_sem_seg_map_ch))
#endif
        {
            if (ch_width == custom_param->SEM_SEG_Width && ch_height == custom_param->SEM_SEG_Height)
            {
                g_sem_seg_map_ch = ch;
            }
        }

#if (CDNN)
        if (custom_param->POSE_EST_Enable && (-1 == g_pose_est_map_ch))
#else
        if ((custom_param->POSE_EST_Width > 0) && (custom_param->POSE_EST_Height > 0) && (-1 == g_pose_est_map_ch))
#endif
        {
            if (ch_width == custom_param->POSE_EST_Width && ch_height == custom_param->POSE_EST_Height)
            {
                g_pose_est_map_ch = ch;
            }
        }
    }

    return;
}
/**********************************************************************************************************************
 End of function map_imr_ch_to_ai_model
 *********************************************************************************************************************/

/******************************************************************************************************************/
/* Function Name : enable_imr_channel*/
/******************************************************************************************************************//**
 * @brief       Enable the IMR channels with AI models mapped (All others disabled)
 * @param[in]   channel         IMR channel with AI model mapping
 * @param[out]  custom_param    Customization Pointer
 * @retval      None
 *********************************************************************************************************************/                   
static void enable_imr_channel (st_customize_t *custom_param, int channel)
{
    switch (channel)
    {
    case 0:
        custom_param->IMR_Ch_0_Enable = 1;
        break;
    case 1:
        custom_param->IMR_Ch_1_Enable = 1;
        break;
    case 2:
        custom_param->IMR_Ch_2_Enable = 1;
        break;
    case 3:
        custom_param->IMR_Ch_3_Enable = 1;
        break;
    case 4:
        custom_param->IMR_Ch_4_Enable = 1;
        break;
    default:
        break;
    }

}
/**********************************************************************************************************************
 End of function enable_imr_channel
 *********************************************************************************************************************/
