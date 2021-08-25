#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <list>
using namespace std;

#include <k4a/k4a.h>
#include <k4abt.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <ctime>
#include <time.h>
#include <conio.h>

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

string getDatetimeStr() {
    time_t t = time(nullptr);
    struct tm now_time;
    errno_t error;
    error = localtime_s(&now_time, &t);
    std::stringstream s;
    s << "20" << now_time.tm_year - 100;
    // setw(),setfill()で0詰め
    s << setw(2) << setfill('0') << now_time.tm_mon + 1;
    s << setw(2) << setfill('0') << now_time.tm_mday;
    s << setw(2) << setfill('0') << now_time.tm_hour;
    s << setw(2) << setfill('0') << now_time.tm_min;
    s << setw(2) << setfill('0') << now_time.tm_sec;
    // std::stringにして値を返す
    return s.str();
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

    // バイナリ出力モードで開く
    //fstream file("./skeleton/skeleton41.txt", ios::binary | ios::out);
    
    //計測開始時のtimesramp作成
    auto start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    //関節点のデータを書き込む用のファイルを作成
    std::ofstream writing_file;
    std::string timedata = getDatetimeStr();
    std::string filename = "./jointdata/" + timedata + ".csv";

    //関節点を書き込む用のvectorを作成しておく。毎フレーム中に入れるlistを生成する
    vector<list<string>> vec_jointlist;
    int frame_count = 0; //現在のFrame位置を記録しておく変数
    uint32_t temp_time = NULL; //FPS算出用の変数
    do
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;
            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
            //k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
            

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
                uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                printf("%zu body, ", num_bodies);

                //現在の時刻を取得する（マイクロ秒の情報を取得)
                uint32_t now_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
                if (frame_count > 1) {
                    int FPS = (int)1000 / (now_time - temp_time);
                    //printf("\nnow time =  %zu\n", FPS);
                    //printf("past time = %zu\n", temp_time);
                    printf("%dFPS \n", FPS);
                }
                temp_time = now_time;
                float during_millisec = (now_time - start_time);
                std::string str_during_millisec = std::to_string(during_millisec);
                //std::cout << now;
                for (uint32_t i = 0; i < num_bodies; i++)
                {
                    k4abt_skeleton_t skeleton;
                    k4a_result_t result = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                    if (result == K4A_RESULT_FAILED)
                    {
                        printf("Error! cannot capture result\n");
                        break;
                    }

                    //現在時刻と、idを書き込む
                    //writing_file << str_during_millisec + ",";
                    uint32_t id = k4abt_frame_get_body_id(body_frame, i);
                    //writing_file << std::to_string(id) + ",";
                    
                    //dataを格納するリストの作成
                    list<string> l_joint;
                    l_joint.push_back(str_during_millisec);
                    l_joint.push_back(std::to_string(id));
                    //printf("id %zu is detected!!!\n", id);
        
                    //フレームのi番のスケルトンデータ
                    for (int jointId = 0; jointId < 33; ++jointId)
                    {   
                        k4abt_joint_t joint = skeleton.joints[jointId];
                        //printf("jointId = %d position is %f.\n", jointId, joint.position.xyz.x);
                        //printf("position is %f, %f, %f", joint.Position.X, joint.Position.Y, joint.Position.Z);
                        //実際の書き込み
                        std::string x = std::to_string(joint.position.xyz.x);
                        std::string y = std::to_string(joint.position.xyz.y);
                        std::string z = std::to_string(joint.position.xyz.z);
                        std::string writing_text = x + "," + y + "," + z + ",";
                        //writing_file << writing_text;
                        l_joint.push_back(x);
                        l_joint.push_back(y);
                        l_joint.push_back(z);
                    }
                    //改行コードを書き込む
                    //writing_file << std::endl;
                    //vectorにいれる
                    vec_jointlist.push_back(l_joint);
                    //時刻の記録

                }

                k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it

                // Kinect for Azure color & depth.
                const auto k4a_color = k4a_capture_get_color_image(sensor_capture);
                //const auto k4a_ir = k4a_capture_get_ir_image(sensor_capture);
                //const auto k4a_depth = k4a_capture_get_depth_image(sensor_capture);

                //if (k4a_color == NULL || k4a_ir == NULL) {
                //    continue;
                //}

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

                /*
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
                */

                //ここでスケルトン以外の画像データを保存する
                //cv_color, cv_ir, cv_depth
                //cv::Mat
                //cout << "milliseconds since epoch: " << start_time << endl;
                /*cv::imwrite("./color/" + std::to_string(start_time) + ".jpg", cv_color);
                cv::imwrite("./ir/" + std::to_string(start_time) + ".jpg", cv_ir);
                cv::imwrite("./depth/" + std::to_string(start_time) + ".jpg", cv_depth);*/

                cv::waitKey(1);

                /*
                // Release the image and capture.
                k4a_image_release(k4a_color);
                k4a_image_release(k4a_ir);
                k4a_image_release(k4a_depth);
                k4a_capture_release(sensor_capture);
                */
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

        //キー入力を待っている状態
        if (_kbhit()) {
            if (getchar() == 'q') {
                printf("stop recording\n");
                break;
            }
        }
    } while (frame_count < FRAME_NUM);
    
    printf("saving");
    writing_file.open(filename, std::ios::out);  //ファイルを開く
    for (const auto& e : vec_jointlist) {
        list<string> l = e;
        for (string s : l) {
            writing_file << s;
            writing_file << ',';
        }
        writing_file << '\n';
        printf(".");
    }

    //書き込みファイルを閉じる
    writing_file.close();
    cv::destroyAllWindows();

    printf("\nFinished body tracking processing!\n");

    //file.close();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    //Fileへの構造体の書き込み
    //fstream file("C:Users\suzuk\data\skelton.dat", ios::binary | ios::out);
    /*
    fstream file2("..\skelton42.txt", ios::binary | ios::out);
    for (int i = 0; i < FRAME_NUM; i++) {
        file2.write((char*)&skeleton, sizeof(skeleton));
        //printf("position of x = %f\n", skeleton[i].joints[0].position.xyz);
    }
    file2.close();
    */
    return 0;
}