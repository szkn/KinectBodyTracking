#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

#include <k4a/k4a.h>
#include <k4abt.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <ctime>

using std::cout; using std::endl;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;



#define FRAME_NUM 10000
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }              


int main()
{   
    //Kinectを開く
    k4a_device_t device = NULL;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    //体のトラッキング結果を取得するためのトラッカーを作成
    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");
    
    k4abt_tracker_t tracker = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

    //実際にフレームをキャプチャしていく。
    //構造体 skeletonの配列の作成
    //k4abt_skeleton_t skeleton[FRAME_NUM];
    vector<k4abt_skeleton_t> skeleton(FRAME_NUM);
    //int frame_count = 0;
    //timesramp作成
    /*
    while (cv::waitKey(1) != 'q') {
        // Capture a depth frame
        k4a_capture_t capture;

        switch (k4a_device_get_capture(device, &capture, 1000))
        {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            printf("Timed out waiting for a capture\n");
            continue;
            break;
        case K4A_WAIT_RESULT_FAILED:
            printf("Failed to read a capture\n");
            return 1;
        }

        // Kinect for Azure color & depth.
        const auto k4a_color = k4a_capture_get_color_image(capture);
        const auto k4a_ir = k4a_capture_get_ir_image(capture);
        const auto k4a_depth = k4a_capture_get_depth_image(capture);

        if (k4a_color == NULL || k4a_ir == NULL) {
            continue;
        }

        // Print depth image details.
        printf(" | Depth16 res:%4dx%4d stride:%5d\n",
            k4a_image_get_height_pixels(k4a_ir),
            k4a_image_get_width_pixels(k4a_ir),
            k4a_image_get_stride_bytes(k4a_ir));

        // Get color as cv::Mat
        const auto cv_color = cv::Mat_<cv::Vec4b>(
            k4a_image_get_height_pixels(k4a_color),
            k4a_image_get_width_pixels(k4a_color),
            (cv::Vec4b*)k4a_image_get_buffer(k4a_color),
            k4a_image_get_stride_bytes(k4a_color));
        cv::imshow("color", cv_color);

        const auto cv_ir = cv::Mat_<short>(
            k4a_image_get_height_pixels(k4a_ir),
            k4a_image_get_width_pixels(k4a_ir),
            (short*)k4a_image_get_buffer(k4a_ir),
            k4a_image_get_stride_bytes(k4a_ir));
        cv::imshow("IR", cv_ir * 20);

        const auto cv_depth = cv::Mat_<short>(
            k4a_image_get_height_pixels(k4a_depth),
            k4a_image_get_width_pixels(k4a_depth),
            (short*)k4a_image_get_buffer(k4a_depth),
            k4a_image_get_stride_bytes(k4a_depth));
        cv::imshow("depth", cv_depth * 40);

        cv::waitKey(1);

        // Release the image and capture.
        k4a_image_release(k4a_color);
        k4a_image_release(k4a_ir);
        k4a_image_release(k4a_depth);
        k4a_capture_release(capture);
    }
    */
    
    int frame_count = 0;
    // バイナリ出力モードで開く
    fstream file("./skeleton/skeleton40.dat", ios::binary | ios::out);
    //timesramp作成
    do
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            //k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            //フレームの取得が成功したとき
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                // Successfully popped the body tracking result. Start your processing

                size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                printf("%zu bodies are detected!\n", num_bodies);
                for (size_t i = 0; i < num_bodies; i++)
                {
                    k4abt_frame_get_body_skeleton(body_frame, i, &skeleton[frame_count-1]);
                    //uint32_t id = k4abt_frame_get_body_id(body_frame, i);
                    //printf("id %zu is detected!!!\n", id);
                    //printf("position is %zu !\n", skeleton[frame_count - 1].joints);
                    //フレームのi番のスケルトンデータ
                    //skeleton[frame_count-1]
                    // // 書き込む
                    file.write((char*)&skeleton[frame_count-1], sizeof(skeleton[frame_count - 1]));
                    //cout << "milliseconds since epoch: " << &skeleton[frame_count-1].joints[12].position.xyz.x;
                    //k4abt_skeleton_t
                }

                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it

                // Kinect for Azure color & depth.
                const auto k4a_color = k4a_capture_get_color_image(sensor_capture);
                const auto k4a_ir = k4a_capture_get_ir_image(sensor_capture);
                const auto k4a_depth = k4a_capture_get_depth_image(sensor_capture);

                if (k4a_color == NULL || k4a_ir == NULL) {
                    continue;
                }

                // Print depth image details.
                //printf(" | Depth16 res:%4dx%4d stride:%5d\n",
                //    k4a_image_get_height_pixels(k4a_ir),
                //    k4a_image_get_width_pixels(k4a_ir),
                //    k4a_image_get_stride_bytes(k4a_ir));

                // Get color as cv::Mat
                const auto cv_color = cv::Mat_<cv::Vec4b>(
                    k4a_image_get_height_pixels(k4a_color),
                    k4a_image_get_width_pixels(k4a_color),
                    (cv::Vec4b*)k4a_image_get_buffer(k4a_color),
                    k4a_image_get_stride_bytes(k4a_color));
                cv::imshow("color", cv_color);

                const auto cv_ir = cv::Mat_<short>(
                    k4a_image_get_height_pixels(k4a_ir),
                    k4a_image_get_width_pixels(k4a_ir),
                    (short*)k4a_image_get_buffer(k4a_ir),
                    k4a_image_get_stride_bytes(k4a_ir));
                cv::imshow("IR", cv_ir * 20);

                const auto cv_depth = cv::Mat_<short>(
                    k4a_image_get_height_pixels(k4a_depth),
                    k4a_image_get_width_pixels(k4a_depth),
                    (short*)k4a_image_get_buffer(k4a_depth),
                    k4a_image_get_stride_bytes(k4a_depth));
                cv::imshow("depth", cv_depth * 40);

                //ここでスケルトン以外の画像データを保存する
                //cv_color, cv_ir, cv_depth
                //cv::Mat
                //cout << "milliseconds since epoch: " << millisec_since_epoch << endl;
                cv::imwrite("./color/" + std::to_string(millisec_since_epoch) + ".jpg", cv_color);
                cv::imwrite("./ir/" + std::to_string(millisec_since_epoch) + ".jpg", cv_ir);
                cv::imwrite("./depth/" + std::to_string(millisec_since_epoch) + ".jpg", cv_depth);

                cv::waitKey(1);

                // Release the image and capture.
                k4a_image_release(k4a_color);
                k4a_image_release(k4a_ir);
                k4a_image_release(k4a_depth);
                k4a_capture_release(sensor_capture);
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }


        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }
        


    } while (frame_count < FRAME_NUM);
    

    cv::destroyAllWindows();

    printf("Finished body tracking processing!\n");

    file.close();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    //Fileへの構造体の書き込み
    //fstream file("C:Users\suzuk\data\skelton.dat", ios::binary | ios::out);
    fstream file2("..\skelton.dat", ios::binary | ios::out);
    for (int i = 0; i < FRAME_NUM; i++) {
        file2.write((char*)&skeleton, sizeof(skeleton));
        //printf("position of x = %f\n", skeleton[i].joints[0].position.xyz);
    }
    file2.close();

    return 0;
}