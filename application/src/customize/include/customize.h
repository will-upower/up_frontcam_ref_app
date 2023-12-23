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
* File Name    : customize.h
* Version      : 1.0
* Description  : header File
***********************************************************************************************************************/
/***********************************************************************************************************************
* History : Version DD.MM.YYYY Description
*         : 1.0.0   27.09.2022 First Release 
***********************************************************************************************************************/

#ifndef CUSTOMIZE_H_
#define CUSTOMIZE_H_

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/

#define FC_CustomizeFile "config/frontcam_customize.config"

/**********************************************************************************************************************
 Typedef definitions
 *********************************************************************************************************************/
typedef struct 
{
    /* VIN */
    int   VIN_Enable;                            /* 0:Disable, 1:Enable - (default:1)*/
    int   VIN_Device;
    int   VIN_Capture_Format;                    /* 0:"YUYV", 1:UYVY, 2:RGB24, 3:Y10 - (default:0) */
    int   VIN_Offset_X;
    int   VIN_Offset_Y;
    int   VIN_Req_Buffer_Num;

    /* Camera */
    int   Max_Camera_Width;                     /* Maximum resolution width supported by camera */ 
    int   Max_Camera_Height;                    /* Maximum resolution Height supported by camera */ 
                       
    /* Pipeline Resolution */
    int   Frame_Width;
    int   Frame_Height;
    int   High_Res_Enable;                        /* 1 : High resolution image input enabled (4K x 2K) */
    int   High_Res_Width;                         /*.(default:4096) */
    int   High_Res_Height;  

    /* ISP */
    int   ISP_Enable;                             /* 0:Disable, 1:Enable - (default:0) */
    int   ISP_Channel;                            /* 0:Channel 0, Channel 1 - (default:0) */
    int   ISP_RAW_IN_Format;                      /* 0:"RGGB", 1:BGGR - (default:0) */
    int   ISP_RAW_OUT_Format;                     /* 0:"Y+UV", 1:RGB24 - (default:0) */

    /* IMR */
    int   IMR_Channel;
    int   IMR_LDC;                               /* 0:Disable, 1:Enable - (default:0) */
    int   IMR_Ch_0_Enable;
    int   IMR_Ch_1_Enable;
    int   IMR_Ch_2_Enable;
    int   IMR_Ch_3_Enable;
    int   IMR_Ch_4_Enable;
    float IMR_LDC_Params_k1;                     /* 0.4060391299 */
    float IMR_LDC_Params_k2;
    float IMR_LDC_Params_k3;
    float IMR_LDC_Params_p1;
    float IMR_LDC_Params_p2;
    float IMR_LDC_Params_fx;                     /* (fx * Input Width) */
    float IMR_LDC_Params_fy;                     /* (fy * Input Height) */
    float IMR_LDC_Params_cx;                     /* (cx * input Width) */
    float IMR_LDC_Params_cy;                     /* (fy * Input Height) */
    int   IMR_Resize;                            /* 0:Disable, 1:Enable  - (default:1) */
    int   IMR_Resize_Width_Ch_0;                    
    int   IMR_Resize_Height_Ch_0;                    
    int   IMR_Resize_Width_Ch_1;                      
    int   IMR_Resize_Height_Ch_1;
    int   IMR_Resize_Width_Ch_2;                     
    int   IMR_Resize_Height_Ch_2;
    int   IMR_Resize_Width_Ch_3;                      
    int   IMR_Resize_Height_Ch_3;
    int   IMR_Resize_Width_Ch_4;                     
    int   IMR_Resize_Height_Ch_4;
    
    /* CDNN */
    int   CDNN_Enable;                           /* 0:Disable, 1:Enable  - (default:1) */   
    int   OBJ_DET_Enable;
    int   SEM_SEG_Enable;
    int   POSE_EST_Enable ;
    int   TRF_SGN_DET_Enable;
    int   DEPTH_EST_Enable ;
    int   OPT_FLOW_Enable ;
    int   CDNN_Load_Enable;                      /* 0:Disable, 1:Enable  - (default:1) */

    /* VOUT */
    int   VOUT_Enable;                           /* 0:Disable, 1:Enable - (default:1)*/
    char  DRM_Module[64];
    int   VOUT_Display_Format;                   /* 0:"YUYV", 1:UYVY, 1:RGB24 (default:0) */
    int   VOUT_Display_Width;
    int   VOUT_Display_Height;
    int   VOUT_Pos_X;
    int   VOUT_Pos_Y;

    /* Debug */
    int   Debug_Enable;                          /* 0:OFF, 1:ON (default:1) */
    int   Logging;                               /* 0:OFF, 1:ON (default:1) */
    int   Proc_Time;

    /*CPULOAD*/
    int CPU_Load_Enable;                         /* 0:OFF, 1:ON (default:1)*/
    /* Frame.file*/
    char  Frame_File_Name[64];
    int   Image_Folder_Enable;                   /* 0:Disable, 1:Enable (default:0)*/
    char  Frame_Folder_Name[64];

    int OBJ_DET_Width;       
    int OBJ_DET_Height;        
    int SEM_SEG_Width;      
    int SEM_SEG_Height;         
    int POSE_EST_Width;        
    int POSE_EST_Height; 
    //New
    int MMAP_Contingency_enable;

}st_customize_t;

/**********************************************************************************************************************
 Global variables
 *********************************************************************************************************************/
extern int g_obj_det_map_ch;
extern int g_sem_seg_map_ch;
extern int g_pose_est_map_ch;
extern st_customize_t g_customize;

/**********************************************************************************************************************
 Exported global functions
 *********************************************************************************************************************/
void  R_CustomizeInit (st_customize_t * custom_param);
int  R_CustomizeLoad (st_customize_t * custom_param, const char * file_name);
int  R_CustomizePrint (st_customize_t * custom_param);
int R_CustomizeValidate (st_customize_t * custom_param);
extern int CustomizeRangeCheck (char * var, int val, int min, int max);
extern char * getstr(char *buffer, char *tag_name, char *dest_buffer);
#endif
