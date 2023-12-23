/*********************************************************************************************************************
* File Name    : draw_detection.cpp
* Version      : 1.0.0
* Product Name : FCN Application
* Device(s)    : R-Car V4H
* Description  : draw bounding box , semantic segmentation and pose estimation
*********************************************************************************************************************/

/*********************************************************************************************************************
* History : Version DD.MM.YYYY Description
*         : 1.0.0   18.07.2022 Initial version 
*********************************************************************************************************************/
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui.hpp>
#include <time.h>
#include "read_coordinate.h"
#include "read_road_coordinate.h"
#include "../include/common.h"
#include "draw_detection.h"
#include <iostream>
#include <fstream>
#include "pose.hpp"
#include "postprocess.hpp"
using namespace cv;

 const char *class_names[] = {"person","bicycle","car","motorcycle","bus","train","truck","traffic_light","stop_sign",
                              "curve_warning","bumpy","slippery_road","pedestrian_crossing","double_curve",
                              "traffic_signal_ahead","crossroad_junction_ahead","roundabout","road_narrow",
                              "no_motorcycle_and_car_allowed","no_turn","no_overtaking","speed_limit","no_stopping","turn",
                              "proceed_or_turn","keep_left_or_right","schoolzone"};

const char *class_names_trf_sgn[] = {"Left curve warning", "Right curve warning", "Bump", "Slippery road", "Pedestrian crossing", "Road work ahead",       
                                     "Double curve first to left", "Double curve first to right", "Traffic signal ahead", "Caution", "Crossroad junction ahead",
                                     "Roundabout", "Level crossing", "Road Narrows Left", "Narrow road ahead", "Road Narrows Right", "Bumpy Road", "Pothole",
                                     "Pedestrian crossing", "No motorcycles and cars allowed", "Stop", "No left turn", "No right turn", "Overtaking prohibited",
                                     "No entry for vehicular and pedestrians", "Speed Limit 10kmph", "Speed Limit 30kmph", "Speed Limit 40kmph", "Speed Limit 50 kmph",
                                     "Speed Limit 60kmph","Speed Limit 70kmph", "No stopping", "No parking", "No entry for vehicular traffic", "Turn right", "Turn left",
                                     "Proceed straight or turn left", "Proceed straight or turn right", "Proceed right or left", "Keep left", "Keep right", "Roundabout",
                                     "Curve marker right", "Curve marker left", "School zone right", "School zone", "School zone left",
                                     "Speed radar", "Proceed straight", "Parking spot", "First aid"};
#define BGR2RGB(r, g, b) (r, b, g)


#define DEPTH_EST_IMG_WIDTH     384
#define DEPTH_EST_IMG_HEIGHT    384
#define DEPTH_EST_CHANNEL       3
#define OPTICAL_HEIGHT          96
#define OPTICAL_WIDTH           61
#define OPTICAL_CHANNEL         3

unsigned char sem_seg_array[SEM_SEG_IMG_WIDTH*SEM_SEG_IMG_HEIGHT];
unsigned char depth_est_arr[DEPTH_EST_IMG_WIDTH*DEPTH_EST_IMG_HEIGHT*DEPTH_EST_CHANNEL];

uint8_t opticalflow_final[OPTICAL_WIDTH * OPTICAL_HEIGHT * OPTICAL_CHANNEL];
Scalar clr_unknown = CLR_UNKNOWN;
Scalar clr_none = CLR_UNKNOWN;
Scalar clr_red = CLR_RED;
Scalar clr_yellow = CLR_YELLOW;
Scalar clr_blue = CLR_BLUE;
Scalar clr_purple = CLR_PURPLE;

#define CLR_GREEN_LIGHT Scalar(191,  255,  0)
#define CLR_BLUE_LIGHT Scalar(0,  127,255)
#define CLR_PURPLE_LIGHT Scalar(192, 67, 251)

cv::Vec3b div_cls = {192, 67, 251};
cv::Vec3b road_cls = {0, 191,  255};

Scalar clr_bb = CLR_GREEN_LIGHT;
Scalar clr_road = CLR_BLUE_LIGHT;
Scalar clr_division = CLR_PURPLE_LIGHT;

float pe_array_heatmaps[POSE_EST_HEATMAP_CHANNEL * POSE_EST_INFERENCE_WIDTH * POSE_EST_INFERENCE_HEIGHT];
float pe_array_pafs[POSE_EST_PAF_CHANNEL * POSE_EST_INFERENCE_WIDTH * POSE_EST_INFERENCE_HEIGHT];
Pose_Estimation::Pose_Estimation pe;

Scalar clr_person = clr_purple;
Scalar clr_arry[] = {clr_person, clr_unknown};
Scalar clr_road_map = Scalar(127, 127, 160);
Scalar seg_colors[] = {clr_division, clr_none,clr_road};
int cv_mat_array[5] = {CV_8UC2, CV_8UC2, CV_8UC3, CV_8UC2, CV_8UC2};

char *input_cord_file = "%s.txt";
char cord_file_name[50];

bbox_cor  _bbox_cor[30];
bbox_cor  _bbox_cor_ts[30];
int detObjCount = 0;
int detObjCount_ts = 0;
typedef enum{
    PLN_1 = 1,
    PLN_2,
    PLN_3
}planes;
static int readYUVImage(char *file_name, cv::Size szSize, cv::Mat &mSrc) {
    uchar *b_Buffer = new uchar[szSize.width * szSize.height * 2];
    mSrc = cv::Mat(szSize,CV_8UC2, b_Buffer);

    string binFile(file_name);
    ifstream input_file;
    input_file.open(binFile, ios::in | ios::binary);

    if (!input_file.is_open())
    {
        std::cerr << "[ERROR] cannot open the YUV Input File " << binFile << std::endl;
        std::cerr << std::endl;
        return 1;
    }
    input_file.read((char*)b_Buffer, sizeof(uchar)*szSize.width*szSize.height*2);
    input_file.close();
    return 0;
}

#define MODE 1
#if MODE
int x, y;
Scalar clr;
struct PTime {
    clock_t start;
    clock_t end;
};


struct PTimeList {
    PTime ptime[10];
    int count;
};

PTimeList ptime_list = {0};

static int approach2(cv::Mat &src) {
    cv::Size szSize(MAX_ROAD_CORD_WIDTH, MAX_ROAD_CORD_HEIGHT);
    cv::Size inSize(src.cols, src.rows);

    Mat color_segmap = Mat(szSize, CV_8UC3);

    Mat det_segmap = Mat(szSize, CV_8U, (unsigned char*)g_road_map.road);
    Mat nonZeroCoordinates;
    findNonZero(det_segmap, nonZeroCoordinates);

    for (size_t i = 0; i < nonZeroCoordinates.total(); i++ ) {
        x = nonZeroCoordinates.at<Point>(i).x;
        y = nonZeroCoordinates.at<Point>(i).y;

        clr = seg_colors[det_segmap.at<unsigned char>(y, x)];
        color_segmap.at<cv::Vec3b>(y, x)[0] = clr.val[0];
        color_segmap.at<cv::Vec3b>(y, x)[1] = clr.val[1];
        color_segmap.at<cv::Vec3b>(y, x)[2] = clr.val[2];
    }

    ptime_list.ptime[ptime_list.count].start = clock();
    resize(color_segmap, color_segmap, inSize);
    ptime_list.ptime[ptime_list.count++].end = clock();

    Mat gray;
    cvtColor(color_segmap, gray, CV_BGR2GRAY, 1);
    findNonZero(gray, nonZeroCoordinates);

    for (size_t i = 0; i < nonZeroCoordinates.total(); i++ ) {
        x = nonZeroCoordinates.at<Point>(i).x;
        y = nonZeroCoordinates.at<Point>(i).y;

        cv::Vec3b cls = color_segmap.at<cv::Vec3b>(y, x);
        if(cls == road_cls || cls == div_cls) {
            src.at<cv::Vec3b>(y, x) = cls;
        }
    }
    return 0;
}

/**********************************************************************************************************************
/* Function Name : approach1 */
/**********************************************************************************************************************
 * @brief       semantic segmentation drawing
                [Covers: BPAP_FC_V4H_AD004][Covers: BPAP_FC_V4H_AD014]
 * @param[in]   cv::Mat &src
 * @param[out]  none
 * @retval      0                    success
 * @retval      1                    fail
 *********************************************************************************************************************/
static int approach1(cv::Mat &src) {

    cv::Size szSize(SEM_SEG_IMG_WIDTH, SEM_SEG_IMG_HEIGHT);
    cv::Size inSize(g_frame_width, g_frame_height);

    Mat det_segmap = Mat(szSize, CV_8U, (unsigned char*)sem_seg_array);
    Mat det_segmap_rz = Mat(inSize, CV_8U);

    Mat color_segmap = src;

    resize(det_segmap, det_segmap_rz, inSize);     /* upscaling */

    Mat nonZeroCoordinates;
    findNonZero(det_segmap_rz, nonZeroCoordinates);   /* finding the coordinates with non zero values */

    for (size_t i = 0; i < nonZeroCoordinates.total(); i++ )
    {
        x = nonZeroCoordinates.at<Point>(i).x;
        y = nonZeroCoordinates.at<Point>(i).y;

        if(det_segmap_rz.at<unsigned char>(y, x) == eLANE)
        {
            clr = seg_colors[det_segmap_rz.at<unsigned char>(y, x)];
            color_segmap.at<cv::Vec3b>(y, x)[1] = clr.val[1];
            color_segmap.at<cv::Vec3b>(y, x)[2] = clr.val[2];
        }
        
        if(det_segmap_rz.at<unsigned char>(y, x) == eROAD)
        {
            clr = seg_colors[det_segmap_rz.at<unsigned char>(y, x)];
            color_segmap.at<cv::Vec3b>(y, x)[2] = clr.val[2];
        }
    }
    //complete garbage ignore please
    /* std::ofstream outFile("segmap_data.bin", std::ios::binary);
    if (!outFile.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        return 0;
    }
    outFile.write(reinterpret_cast<char*>(det_segmap.data), det_segmap.total() * det_segmap.elemSize());
    if (!outFile) {
        std::cerr << "Error writing to file." << std::endl;
        outFile.close(); // Close the file before returning
        return 0; // Return an error code
    }
    outFile.close(); */
    //For Will commented out for performance reasons
    /* cv::FileStorage fs("semseg_output.yaml", cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open the file semseg_output.yaml" << std::endl;
        return -1;
    }
    fs << "ImageData" << det_segmap;
    fs.release();
    std::cout << "Data written to semseg_output.yaml successfully." << std::endl;
    return 0; */
}
/**********************************************************************************************************************
 End of function approach1
 *********************************************************************************************************************/


static int approach3(cv::Mat &src) {
    cv::Size szSize(MAX_ROAD_CORD_WIDTH, MAX_ROAD_CORD_HEIGHT);
    cv::Size inSize(src.cols, src.rows);

    Mat color_segmap = Mat(szSize, CV_8U, (unsigned char*)g_road_map.road);
    ptime_list.ptime[ptime_list.count].start = clock();
    resize(color_segmap, color_segmap, inSize);
    ptime_list.ptime[ptime_list.count++].end = clock();

    Mat nonZeroCoordinates;
    findNonZero(color_segmap, nonZeroCoordinates);

    for (size_t i = 0; i < nonZeroCoordinates.total(); i++ ) {
        x = nonZeroCoordinates.at<Point>(i).x;
        y = nonZeroCoordinates.at<Point>(i).y;

        clr = seg_colors[color_segmap.at<unsigned char>(y, x)];
        src.at<cv::Vec3b>(y, x)[0] = clr.val[0];
        src.at<cv::Vec3b>(y, x)[1] = clr.val[1];
        src.at<cv::Vec3b>(y, x)[2] = clr.val[2];
    }
    return 0;
}

static int draw_map(cv::Mat &src, int id) {
    clock_t start, end;
    double time;
    start = clock();

    switch (id)
    {
    case 1:
        approach1(src);
        break;
    case 2:
        approach2(src);
        break;
    case 3:
        approach3(src);
        break;
    
    default:
        break;
    }
    
    end = clock();
    time = (double) (end - start)/60;
    if (true==g_customize.Debug_Enable)
    {
        cout << "Approach " << id << ", Time: " << time << "ms" << endl;
        for (int i = 0; i < ptime_list.count; i++) {
            time = (double) (ptime_list.ptime[i].end - ptime_list.ptime[i].start)/60;
            cout << "Resize " << i + 1 << ", Time: " << time << "ms" << endl;
        }
    }
    memset((void *)&ptime_list, 0, sizeof(ptime_list));
    return 0;
}
#endif

static int draw_detection(cv::Mat &src) {
    int thickness = 2;
    int det = 0;
    int ret= 0;
    st_obj_det  bbox_que_recv = {0};
    bbox_cor bb_list[30] = {0};

    memcpy((void*)bb_list,(void *)_bbox_cor,sizeof(bb_list));

    for (det = 0; det < detObjCount; det++) 
    {  
        bbox_cor bb_data = bb_list[det];



        // Top Left Corner
        Point ps(bb_data.start.x, bb_data.start.y);
    
        // Bottom Right Corner
        Point pe(bb_data.end.x, bb_data.end.y);
    
        // Drawing the Rectangle
        cv::rectangle(src, ps, pe,
                clr_bb,
                thickness, LINE_8);
        Scalar bb_sclr = clr_arry[0];

        int text_size = FONT_HERSHEY_PLAIN;

        cv::Size siz = getTextSize(class_names[bb_data.cls], text_size, 1, 1, 0);

        // set the text start position
        int text_offset_x = 1;
        int text_offset_y = 4;
        // make the coords of the box with a small padding of two pixels

        Point txt_bb_ps(bb_data.start.x, bb_data.start.y - text_offset_y - siz.height);
        Point txt_bb_pe(bb_data.start.x + text_offset_x + siz.width, bb_data.start.y);

        // Drawing the Rectangle
        rectangle(src, txt_bb_ps, txt_bb_pe,
                bb_sclr,
                FILLED);

        // Test point
        Point pt(bb_data.start.x + 1, bb_data.start.y - 3);

        putText(src, class_names[bb_data.cls], pt, text_size, 1, clr_unknown, 1, 8, 0);
        ret = 1;
    }

    return ret;
}

/**********************************************************************************************************************
/* Function Name : draw_pose */
/**********************************************************************************************************************
 * @brief       pose estimation drawing
                [Covers: BPAP_FC_V4H_AD059]
 * @param[in]   cv::Mat &src
 * @param[out]  none
 * @retval      0                    success
 * @retval      1                    fail
 *********************************************************************************************************************/
int draw_pose(cv::Mat &src)
{

    int data_len = 0;
    bool break_flag = false;
    std::vector<Pose_Estimation::Pose> poses = pe.run(pe_array_heatmaps,pe_array_pafs);
    for(int i = 0; i < poses.size(); i++)
    {
        poses[i].pose_draw(src, true);
    }

    return 0;
}
/**********************************************************************************************************************
 End of function draw_pose
 *********************************************************************************************************************/


/***********************************************************************************************************************
* Function Name: TrafficSignDetection_PreProcess
* Description  : Pre-processing of Traffic sign Detection. 
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static int ts_draw(cv::Mat &src) {
    int thickness = 2;
    int det = 0;
    int ret= 0;
    st_obj_det  bbox_que_recv = {0};
    bbox_cor bb_list[30] = {0};

    memcpy((void*)bb_list,(void *)_bbox_cor_ts,sizeof(bb_list));
    for (det = 0; det < detObjCount_ts; det++) 
    {
        bbox_cor bb_data = bb_list[det];

        // Top Left Corner
        Point ps(bb_data.start.x, bb_data.start.y);
    
        // Bottom Right Corner
        Point pe(bb_data.end.x, bb_data.end.y);
    
        // Drawing the Rectangle
        cv::rectangle(src, ps, pe,
                clr_person,
                thickness, LINE_8);
        Scalar bb_sclr = clr_arry[0];

        int text_size = FONT_HERSHEY_PLAIN;

        cv::Size siz = getTextSize(class_names_trf_sgn[bb_data.cls], text_size, 1, 1, 0);

        // set the text start position
        int text_offset_x = 1;
        int text_offset_y = 4;
        // make the coords of the box with a small padding of two pixels

        Point txt_bb_ps(bb_data.start.x, bb_data.start.y - text_offset_y - siz.height);
        Point txt_bb_pe(bb_data.start.x + text_offset_x + siz.width, bb_data.start.y);

        // Drawing the Rectangle
        rectangle(src, txt_bb_ps, txt_bb_pe,
                bb_sclr,
                FILLED);

        // Test point
        Point pt(bb_data.start.x + 1, bb_data.start.y - 3);

        putText(src, class_names_trf_sgn[bb_data.cls], pt, text_size, 1, clr_red, 1, 8, 0);
        ret = 1;
    }
    return ret;
}


int process_yuv(void *inImage, Mat &outimg, int plane) {
    int ret = 0;
    int cv_mat_type = cv_mat_array[g_customize.VIN_Capture_Format]; 
    cv::Size inSize(g_frame_width, g_frame_height);
    cv::Size outSize = inSize;
    R_FC_SyncStart(eFC_DRAW, gp_mtx_handle_opencv, gp_opencv_cond_handle, 0);
    cv::Mat mSrc(inSize, cv_mat_type, inImage);
    R_FC_SyncEnd(eFC_DRAW, gp_mtx_handle_opencv, gp_opencv_cond_handle, 0);
    cv::Mat mSrc_BGR(inSize, CV_8UC3);
    cvtColor(mSrc, mSrc_BGR, COLOR_YUV2RGB_YUYV);
    
    cv::Mat src = mSrc_BGR;

    static uint32_t det_que_rcv =0;
#if(CDNN)
    if((true == g_customize.CDNN_Enable) || (true == g_customize.MMAP_Contingency_enable))
    {
        //TODO chek the hadle is proper after AI implementation
        e_osal_return_t osal_ret = R_OSAL_MqReceiveForTimePeriod(g_mq_handle_aiactivity, TIMEOUT_MS, 
                                                            (void *)&det_que_rcv, g_mq_config_aiactivity.msg_size);
        if(OSAL_RETURN_OK != osal_ret)
        {
            PRINT_ERROR("receiving message to MQ was failed, osal_ret = %d\n", osal_ret);
        }
    }
#endif

    if(plane == PLN_1)
    {
#if (1)        
        {
            draw_map(src, 1);
            draw_detection(src);
            draw_pose(src);
        }
#endif
    }
    outimg = src;

    return 0;
}
