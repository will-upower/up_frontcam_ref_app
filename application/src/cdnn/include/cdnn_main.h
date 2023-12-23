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
* File Name    : cdnn_main.h
* Version      : 1.0.0
* Description  : header File
***********************************************************************************************************************/

/***********************************************************************************************************************
* History : DD.MM.YYYY  Version  Description
 *          30.05.2023  1.0.0    First Release
***********************************************************************************************************************/

/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/
#ifndef CDNN_MAIN_H
#define CDNN_MAIN_H

/**********************************************************************************************************************
 Includes   <System Includes> , "Project Includes"
 *********************************************************************************************************************/
#include "rcar-xos/ai_lib/ai_lib.h"
/**********************************************************************************************************************
 Macro definitions
 *********************************************************************************************************************/
 #define INF_INPUT_CHANNELS (3)

/**** Object Detection ****/
/* Size of input image to the model */
#define OBJ_WIDTH       (320)
#define OBJ_HEIGHT      (320)
/* Thresholds */ 
#define TH_PROB         (0.5)
#define TH_NMS          (0.5)
/* Number for [region] layer num parameter */
#define NUM_BB          (3)
/* Number of class to be detected */
#define NUM_CLASS       (27)
/* Display */
#define WIDTH_ADJUST_OBJ    (320)
#define HEIGHT_ADJUST_OBJ   (320)
/**********************************************************************************************************************
 Typedef definitions
 *********************************************************************************************************************/

/**********************************************************************************************************************
 Exported global variables and functions
 *********************************************************************************************************************/
int R_CDNN_Execute();
void inferencePreprocess_ss();
void inferencePreprocess_od();
void inferencePostprocess_ss(signed char * data);
void inferencePostprocess_od(signed char * data);
//New
int R_CDNN_Execute_MMAP();
#endif /* CDNN_MAIN_H */
