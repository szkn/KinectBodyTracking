#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <filesystem>
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
#include <direct.h>
#include <Windows.h>

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
    
k4a_float2_t g_fSkeleton2D[K4ABT_JOINT_COUNT] = { 0.0f, };	// ユーザーの 2D 骨格座標 (表示用)
k4a_device_t device = nullptr;
k4abt_tracker_t tracker = nullptr;  //体のトラッキング結果を取得するためのトラッカーを作成
k4abt_frame_t body_frame = nullptr;
k4a_calibration_t sensor_calibration;    
k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
k4abt_skeleton_t skeleton;

//関節点を書き込む用のvectorを作成しておく。毎フレーム中に入れるlistを生成する
vector<list<string>> vec_jointlist_2d; //2D描写データを格納するvector
vector<list<string>> vec_jointlist; //3Dcamera座標のデータを取得するvector
vector<cv::Mat_<cv::Vec4b>> vec_image;      //画像データ保存用のvector
vector<string> vec_time; //画像データが生成された時刻を保存しておくvector
int frame_count = 0; //現在のFrame位置を記録しておく変数
uint32_t temp_time = NULL; //FPS算出用の変数



// 関数群
// Kinect を初期化する
k4a_result_t CreateKinect() {
    //Kinectを開く
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");
    //キャリブレーションを取得
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
        "Get depth camera calibration failed!");
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");

}

//後片付け
k4a_result_t DestroyKinect() {
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    cv::destroyAllWindows();
    printf("\nFinished body tracking processing!\n");
    return K4A_RESULT_SUCCEEDED;
}

string CreateSaveDir(std::string timedata) {
    //画像の保存用のディレクトリの作成
    char dir[30] = "KinectImage/";
    strcat_s(dir, timedata.c_str());
    if (_mkdir(dir) == 0) {
        printf("Directory is created\nsaving");
    }
    return dir;
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

int cap_result(k4a_wait_result_t queue_capture_result) {
    if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        //printf("Error! Add capture to tracker process queue timeout!\n");
        return 0;
    }
    else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
    {
        //printf("Error! Add capture to tracker process queue failed!\n");
        return 0;
    }
    else { return 1; }
}

list<string> Camera3dto2d(uint32_t during_millisec, uint32_t id) {
    list<string> l_joint_2d;   //キャリブレーションデータを格納するリストの作成
    l_joint_2d.empty();
    l_joint_2d.push_back(std::to_string(during_millisec));
    l_joint_2d.push_back(std::to_string(id));
    //キャリブレーションを行い、camera座標からカラー二次元座標に変換して、リストにいれる
    for (int iJoint = K4ABT_JOINT_PELVIS; iJoint < K4ABT_JOINT_COUNT; iJoint++)
    {
        int iValid = 0;
        // 3D の骨格座標を 2D スクリーン座標に変換する
        k4a_calibration_3d_to_2d(&sensor_calibration, &skeleton.joints[iJoint].position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &g_fSkeleton2D[iJoint], &iValid);
        if (iValid == 0)
        {
            // 無効な値は (0,0) に設定
            g_fSkeleton2D[iJoint].xy.x = g_fSkeleton2D[iJoint].xy.y = 0.0f;
        }
        std::string x = std::to_string(g_fSkeleton2D[iJoint].xy.x);
        std::string y = std::to_string(g_fSkeleton2D[iJoint].xy.y);
        l_joint_2d.push_back(x);
        l_joint_2d.push_back(y);
    }
    return l_joint_2d;
}

void Save3dPoint(string timedata) {
    //3Dの関節点のデータを書き込む用のファイルを作成
    std::ofstream writing_file;
    std::string filename = "./jointdata_csv/" + timedata + ".csv";
    writing_file.open(filename, std::ios::out);  //ファイルを開く
    //ファイルへのデータの書き込み
    for (const auto& e : vec_jointlist) {
        list<string> l = e;
        for (string s : l) {
            writing_file << s;
            writing_file << ',';
        }
        writing_file << '\n';
        cout << ".";
    }
    //printf("\n");
    writing_file.close();
}

//キャリブレーションを行った画面描写用の2D関節点情報をファイルに保存する
void Save2dColor(string timedata) {
    std::ofstream writing_file;
    std::string filename = "./jointdata_csv_2D/" + timedata + ".csv";
    writing_file.open(filename, std::ios::out);  //ファイルを開く
    //ファイルへのデータの書き込み
    for (const auto& e : vec_jointlist_2d) {
        list<string> l = e;
        for (string s : l) {
            writing_file << s;
            writing_file << ',';
        }
        writing_file << '\n';
        cout << ".";
    }
    //printf("\n");
    writing_file.close();

}

// color Imageを保存する
void SaveImg(vector<cv::Mat_<cv::Vec4b>> vec_image, vector<string>vec_time, string timedata) {
    std::size_t size = vec_image.size();
    for (unsigned int i = 0; i < size; i++) {
        cv::imwrite("KinectImage/" + timedata + "/" + vec_time[i] + ".jpg", vec_image[i]);
        cout << ".";
    }
}

void Proc(uint32_t start_time) {
    k4a_capture_t sensor_capture;
    k4a_wait_result_t get_capture_result;
    get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
    if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        frame_count++;
        k4a_wait_result_t queue_capture_result = \
            k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
        //k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
        //printf("%u result", queue_capture_result);
        if (cap_result(queue_capture_result) == 0) {
            MessageBox(NULL, TEXT("結果をキャプチャーできませんでした"), TEXT("エラー"), MB_OK);
        }

        k4a_wait_result_t pop_frame_result = \
            k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);

        //フレームの取得が成功したとき
        if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // Successfully popped the body tracking result. Start your processing
            uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
            std::cout << "  number of body = " << num_bodies << "\n";

            //現在の時刻を取得する（マイクロ秒の情報を取得)
            uint32_t now_time = \
                duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            //FPSの画面表示
            if (frame_count > 1) {
                int FPS = (int)1000 / (now_time - temp_time);
                std::cout << FPS << "FPS";
            }
            temp_time = now_time;
            uint32_t during_millisec = (now_time - start_time);

            //体が検出されたとき
            for (uint32_t i = 0; i < num_bodies; i++)
            {
                k4a_result_t result = k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
                if (result == K4A_RESULT_FAILED)
                {
                    MessageBox(NULL, TEXT("骨格情報を取得できませんでした。"), TEXT("エラー"), MB_OK);
                }
                uint32_t id = k4abt_frame_get_body_id(body_frame, i); //idを格納

                list<string> l_joint_2d = Camera3dto2d(during_millisec, id);
                //vectorにいれる
                vec_jointlist_2d.push_back(l_joint_2d);

                //3D camera dataを格納するリストの作成
                list<string> l_joint;
                l_joint.push_back(std::to_string(during_millisec));
                l_joint.push_back(std::to_string(id));
                //printf("id %zu is detected!!!\n", id);

                //フレームのi番のスケルトンデータでの3次元のスケルトンデータを保存する
                for (int jointId = 0; jointId < 33; ++jointId)
                {
                    k4abt_joint_t joint = skeleton.joints[jointId];
                    std::string x = std::to_string(joint.position.xyz.x);
                    std::string y = std::to_string(joint.position.xyz.y);
                    std::string z = std::to_string(joint.position.xyz.z);
                    l_joint.push_back(x);
                    l_joint.push_back(y);
                    l_joint.push_back(z);
                }
                //vectorにいれる
                vec_jointlist.push_back(l_joint);
            }
            k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it

            // Kinect for Azure color & depth.
            const auto k4a_color = k4a_capture_get_color_image(sensor_capture);

            // Get color as cv::
            const auto cv_color = cv::Mat_<cv::Vec4b>(
                k4a_image_get_height_pixels(k4a_color),
                k4a_image_get_width_pixels(k4a_color),
                (cv::Vec4b*)k4a_image_get_buffer(k4a_color),
                k4a_image_get_stride_bytes(k4a_color));
            cv::imshow("color", cv_color);
            //vectorにcolor画像を格納
            vec_image.push_back(cv_color);
            vec_time.push_back(to_string(during_millisec));

            cv::waitKey(1);
            //k4a_image_release(k4a_color); //これあるとポインタが被って正しく保存できない。
            k4a_capture_release(sensor_capture);
        }
        else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            //  It should never hit timeout when K4A_WAIT_INFINITE is set.
            //printf("Error! Pop body frame result timeout!\n");
            MessageBox(NULL, TEXT("Error! Pop body frame result timeout!\n"), TEXT("エラー"), MB_OK);

        }
        else
        {
            printf("Pop body frame result failed!\n");
            MessageBox(NULL, TEXT("Pop body frame result failed!\n"), TEXT("エラー"), MB_OK);
        }
    }
    //エラー処理
    else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    {
        // It should never hit time out when K4A_WAIT_INFINITE is set.
        printf("Error! Get depth frame time out!\n");
        MessageBox(NULL, TEXT("Error! Get depth frame time out!\n"), TEXT("エラー"), MB_OK);
    }
    else
    {
        printf("Get depth capture returned error: %d\n", get_capture_result);
        MessageBox(NULL, TEXT("Get depth capture returned error:\n"), TEXT("エラー"), MB_OK);
    }
}

//実プログラム
int main()
{
    CreateKinect(); //Kinectの立ち上げ
    //計測開始時のtimesramp作成
    uint32_t start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    std::string timedata = getDatetimeStr(); //現在の時間を取得
    std::string dir = CreateSaveDir(timedata); //timestampを保持した名前のフォルダを作成する

    while (frame_count < FRAME_NUM) {
        Proc(start_time); // 繰り返し動作、スケルトン情報の取得など
        //キー入力待機 qを押下したら動作停止
        if (_kbhit()) {
            if (getchar() == 'q') {
                printf("キャプチャが終了しました\n");
                break;
            }
        }
    }
    cout << "saving data\n";
    std::thread t1(Save3dPoint, timedata);  //3Dの関節点データを書き込む
    std::thread t2(Save2dColor, timedata); //画面描写用の2Dの関節点のデータを書き込む
    //cout << "saving img data\n";
    std::thread t3(SaveImg, vec_image, vec_time, timedata);  //画像データの保存

    t1.join();
    t2.join();
    t3.join();

    DestroyKinect();
    return 0;
}