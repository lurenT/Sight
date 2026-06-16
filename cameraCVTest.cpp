#include <windows.h>         // 必须放在最前面！
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp> 

#include "CameraDefine.h"
#include "CameraStatus.h"
#include "CameraApi.h"
#include "Serial.h"

using namespace std;
using namespace cv;

// --- [!!! 核心参数 (根据你的测试更新) !!!] ---

// 1. 决策线位置：画面 X 轴坐标 (原来的1.5倍)
//    原值 450 * 1.5 = 675
//    (注意: 你的画面宽度如果是 640，这个值可能越界了，如果是1280则没问题)
//    (如果画面只有640宽，建议设为 550 或 600，不要超过宽度)
const int DECISION_LINE_X = 675;

// 2. 颗粒大小阈值 (半径像素值)
//    (保持之前的设置，如果你觉得没问题)
const double SIZE_THRESHOLD_RADIUS = 20.0;

// 3. 电机速度 (主推0.3，分选1)
//    (注意：这里的速度单位取决于你的STM32代码解析，如果是整数，0.3可能需要特殊处理)
//    (为了适配整数协议，我们假设你STM32端做了 x10 处理，或者这是非常慢的速度)
//    (如果你的STM32只接受整数，这里暂时填 1 和 3，或者你需要修改STM32支持浮点)
//    (假设 1 代表非常慢，3 代表稍快)
const int SPEED_MAIN_FLOW = 1;    // X轴: 主推流速 (对应你的 0.3，取整为1最稳妥)
const int SPEED_SORT_PUSH = 1;    // Y轴: 分选推力 (对应你的 1，取稍微大一点的值保证效果)
const int SPEED_SORT_PULL = -1;   // Y轴: 分选吸力

// 4. 霍夫圆检测参数 (保持之前调试好的)
const double HOUGH_DP = 1;
const double HOUGH_MIN_DIST = 50;
const double HOUGH_PARAM1 = 100;
const double HOUGH_PARAM2 = 25;
const int HOUGH_MIN_RADIUS = 10;
const int HOUGH_MAX_RADIUS = 40;

int main()
{
    CameraSdkInit(1);
    int hCamera;
    tSdkCameraDevInfo devInfoList[16];
    int cameraCount = 16;
    CameraEnumerateDevice(devInfoList, &cameraCount);
    if (cameraCount <= 0) return -1;
    CameraInit(&devInfoList[0], -1, -1, &hCamera);

    tSdkCameraCapbility capability;
    CameraGetCapability(hCamera, &capability);
    bool isMono = capability.sIspCapacity.bMonoSensor;
    CameraSetIspOutFormat(hCamera, isMono ? CAMERA_MEDIA_TYPE_MONO8 : CAMERA_MEDIA_TYPE_BGR8);

    CameraSetAeState(hCamera, FALSE);
    CameraSetExposureTime(hCamera, 80000); // 80ms
    cout << "相机参数已锁定。" << endl;

    Serial* stm32 = new Serial("COM5", 115200); // 确认COM口
    if (!stm32->isConnected()) return -1;

    CameraPlay(hCamera);

    // --- 2. 启动主流场 ---
    char cmd_buf[64];

    // 启动 X 轴 (注意这里加了负号，根据你之前测试的方向反转)
    sprintf_s(cmd_buf, sizeof(cmd_buf), "X%d\n", -SPEED_MAIN_FLOW);
    stm32->write(cmd_buf);
    Sleep(10);

    // 确保 Y 轴初始停止
    stm32->write("Y0\n");

    cout << "自动分选系统已启动..." << endl;
    cout << "主流速 X: " << -SPEED_MAIN_FLOW << " (对应你的0.3)" << endl;
    cout << "分选速 Y: " << SPEED_SORT_PUSH << " / " << SPEED_SORT_PULL << " (对应你的1)" << endl;
    cout << "决策线 X: " << DECISION_LINE_X << endl;

    tSdkFrameHead frameHead;
    BYTE* pRawBuffer = NULL;
    BYTE* g_pRgbBuffer = NULL;
    Mat frame, frame_gray, frame_display;
    namedWindow("Auto Sorting", WINDOW_AUTOSIZE);

    while (true)
    {
        if (CameraGetImageBuffer(hCamera, &frameHead, &pRawBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            int bufferSize = frameHead.iWidth * frameHead.iHeight * (isMono ? 1 : 3);
            if (g_pRgbBuffer == NULL) g_pRgbBuffer = new BYTE[bufferSize];
            CameraImageProcess(hCamera, pRawBuffer, g_pRgbBuffer, &frameHead);
            frame = Mat(frameHead.iHeight, frameHead.iWidth, isMono ? CV_8UC1 : CV_8UC3, g_pRgbBuffer);
            frame.copyTo(frame_display);

            // --- 视觉检测 ---
            if (isMono) frame.copyTo(frame_gray);
            else cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
            GaussianBlur(frame_gray, frame_gray, Size(9, 9), 2, 2);

            vector<Vec3f> circles;
            HoughCircles(frame_gray, circles, HOUGH_GRADIENT, HOUGH_DP, HOUGH_MIN_DIST,
                HOUGH_PARAM1, HOUGH_PARAM2, HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS);

            // --- 分选逻辑 ---
            bool is_sorting = false;

            for (size_t i = 0; i < circles.size(); i++)
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);

                // 画绿圈显示识别结果
                circle(frame_display, center, radius, Scalar(0, 255, 0), 2);

                // 判断逻辑：是否越过决策线 (X > 675)
                // 注意：你需要确认你的相机分辨率宽度是否足够大 (比如 > 800)
                // 如果分辨率不够，线会画在外面看不见
                if (center.x > DECISION_LINE_X && center.x < (DECISION_LINE_X + 100))
                {
                    if (radius > SIZE_THRESHOLD_RADIUS)
                    {
                        // 大颗粒 -> 推 (Y=3)
                        sprintf_s(cmd_buf, sizeof(cmd_buf), "Y%d\n", SPEED_SORT_PUSH);
                        stm32->write(cmd_buf);

                        // 变蓝 + 文字提示
                        circle(frame_display, center, radius, Scalar(255, 0, 0), 3);
                        putText(frame_display, "LARGE -> PUSH", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
                    }
                    else
                    {
                        // 小颗粒 -> 吸 (Y=-3)
                        sprintf_s(cmd_buf, sizeof(cmd_buf), "Y%d\n", SPEED_SORT_PULL);
                        stm32->write(cmd_buf);

                        // 变红 + 文字提示
                        circle(frame_display, center, radius, Scalar(0, 0, 255), 3);
                        putText(frame_display, "SMALL -> PULL", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
                    }
                    is_sorting = true;
                    break; // 防止冲突，优先处理一个
                }
            }

            // 画出决策线 (黄色竖线)
            // 注意：如果线超出了画面宽度，你也看不见，程序不会报错
            line(frame_display, Point(DECISION_LINE_X, 0), Point(DECISION_LINE_X, frame.rows), Scalar(0, 255, 255), 2);

            imshow("Auto Sorting", frame_display);
            CameraReleaseImageBuffer(hCamera, pRawBuffer);
        }

        // 按键控制
        int key = waitKey(1);
        if (key == 27) { // ESC
            stm32->write("X0\n"); Sleep(10); stm32->write("Y0\n");
            break;
        }
        else if (key == ' ') { // 空格急停
            stm32->write("X0\n"); Sleep(10); stm32->write("Y0\n");
        }
    }

    // 清理
    CameraStop(hCamera);
    CameraUnInit(hCamera);
    delete stm32;
    if (g_pRgbBuffer) delete[] g_pRgbBuffer;
    destroyAllWindows();
    return 0;
}