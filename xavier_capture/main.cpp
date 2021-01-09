#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <fstream>
#include <iterator>
#include <unistd.h>
#include <termios.h>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// camera-related
class Camera_Hdr{
public:
    int cam_id;
    bool stat;
    Mat frame;
    VideoCapture cap;
    int init_cam(int);
    int init_cam(string);
    int get_frame();
    int save_frame(string, string, string);
};

// lidar-related
int numOfVelodynes = 0;
vector<velodyne::VLP16Capture*> captures;
std::vector<std::string> vecOfStr[6];
int center_front_lidar_flag = 1;
int left_front_lidar_flag = 1;
int right_front_lidar_flag = 1;
int left_side_lidar_flag = 1;
int right_side_lidar_flag = 1;
int center_rear_lidar_flag = 1;


char getch(void)
{
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if(tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if(tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if(read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if(tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    printf("%c\n", buf);
    return buf;
}


/** @brief
 * Function to Print the vector as a string
*/
void print_vec(vector<int> const &ip_vector){
    for(uint i = 0; i < ip_vector.size(); i++){
        cout << ip_vector.at(i) << " ";
    }
}


string Get_DateTime(){
    time_t rawtime;
    timeval currentTime;
    struct tm *timeinfo;

    char buffer[80];
    gettimeofday(&currentTime, NULL);
    int milli = currentTime.tv_usec / 1000;

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%d_%m_%Y_%H_%M_%S", timeinfo);

    char Current_Time[84] = "";
    sprintf(Current_Time, "%s_%03d", buffer, milli);
    return Current_Time;
}


void laser_to_cartesian(vector<velodyne::Laser> &lasers, vector<cv::Vec3f> &buffer, int pos) {
    buffer.resize(lasers.size());
    int idx = 0;
    vecOfStr[pos].clear();
    for (int i = 0; i < lasers.size(); i++) {

        if( lasers[i].distance == 0 ) continue;

        const double distance = static_cast<double>( lasers[i].distance );
        const double azimuth  = lasers[i].azimuth  * CV_PI / 180.0;
        const double vertical = lasers[i].vertical * CV_PI / 180.0;

        float x = static_cast<float>( ( distance * cos( vertical ) ) * sin( azimuth ) );
        float y = static_cast<float>( ( distance * cos( vertical ) ) * cos( azimuth ) );
        float z = static_cast<float>( ( distance * sin( vertical ) ) );

        if( x == 0.0f && y == 0.0f && z == 0.0f ){
            x = std::numeric_limits<float>::quiet_NaN();
            y = std::numeric_limits<float>::quiet_NaN();
            z = std::numeric_limits<float>::quiet_NaN();
        }
        string newline;
        if (!isnan(x)){
            // file << x << " " << y << " " << z << " " << std::endl;
            newline = to_string(x) + " " + to_string(y) + " " + to_string(z);
            //cout << newline << endl;
            vecOfStr[pos].push_back(newline);
        }
    }
    buffer.resize(idx);
}

void addToBuffer(int i, vector<velodyne::Laser> &laser_i, vector<cv::Vec3f> &buffer_i, vector<cv::Vec3f> &buffer) {
    laser_to_cartesian(laser_i, buffer_i, i);
}


bool lidar_initialize_single(void)
{
    printf( "lidar_initialize\n" );

    // Live capture
    if (center_front_lidar_flag) {
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.201" );
        const unsigned short port = 2368;
        captures.push_back(new velodyne::VLP16Capture(address, port));
    }
    numOfVelodynes = captures.size();

    printf("Velodyne Reader is initialized with %d lidars\n", numOfVelodynes);
    getch();

    return true;
}

bool lidar_initialize_ct6(void)
{
    printf( "lidar_initialize\n" );

    // Live capture
    if (center_front_lidar_flag) {
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "10.2.0.101" );
        const unsigned short port = 2368;
        captures.push_back(new velodyne::VLP16Capture(address, port));
    }
    if (left_front_lidar_flag) {
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "10.2.0.102" );
        const unsigned short port = 2369;
        captures.push_back(new velodyne::VLP16Capture(address, port));
    }
    if (right_front_lidar_flag) {
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "10.2.0.103" );
        const unsigned short port = 2370;
        captures.push_back(new velodyne::VLP16Capture(address, port));
    }
    if (left_side_lidar_flag) {
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "10.2.0.104" );
        const unsigned short port = 2371;
        captures.push_back(new velodyne::VLP16Capture(address, port));
    }
    if (right_side_lidar_flag) {
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "10.2.0.105" );
        const unsigned short port = 2372;
        captures.push_back(new velodyne::VLP16Capture(address, port));
    }
    if (center_rear_lidar_flag) {
        const boost::asio::ip::address address = boost::asio::ip::address::from_string( "10.2.0.106" );
        const unsigned short port = 2373;
        captures.push_back(new velodyne::VLP16Capture(address, port));
    }
    for (int i = 0; i > captures.size(); i++) {
        if( !captures[i]->isOpen()){
            printf("Can't open VelodyneCapture %d\n", i);
            return false;
        }
    }

    numOfVelodynes = captures.size();

    printf("Velodyne Reader is initialized with %d lidars\n", numOfVelodynes);

    return true;
}


bool lidar_get_frame(void)
{
    auto now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
    vector<vector<velodyne::Laser>> lasers(numOfVelodynes);
    vector<cv::Vec3f> buffer;
    vector<vector<cv::Vec3f>> buffers(numOfVelodynes);

    // Capture One Rotation Data
    for (int i = 0; i < numOfVelodynes; i++) {
        lasers[i].clear();
        *captures[i] >> lasers[i];
        printf("lidar %d, size %ld\n", i, lasers[i].size());
    }

    // Check if there is any missing data
    int emptyCnt = 0;
    for (int i = 0; i < numOfVelodynes; i++) {
        if( lasers[i].empty()) emptyCnt++;
    }
    if (emptyCnt == numOfVelodynes) {
        printf("All lidars are empty\n");
        return false;
    }

    // Convert raw data to x y z
    thread th[numOfVelodynes];
    for (int i = 0; i < numOfVelodynes; i++) {
        // Convert to 3-dimention Coordinates
        th[i] = thread(addToBuffer, i, ref(lasers[i]), ref(buffers[i]), ref(buffer));
    }
    for (int i = 0; i < numOfVelodynes; i++) th[i].join();
    return true;
}


bool lidar_get_last_frame(void)
{
    int old_frame_cnt = 0;

    for (int i = 0; i < numOfVelodynes; i++) {
        // clear lidar frame buffer
        vecOfStr[i].clear();
    }

    // read all the lidar packets in the udp buffers
    while(1) {
        // get lidar data
        if (lidar_get_frame() == 0) {
            printf("old_frame_cnt %d\n", old_frame_cnt);
            break;
        }
        old_frame_cnt++;
    }

    // wait for new lidar cycle
    usleep(100000); // 100ms
    
    // get lidar new lidar data
    if (lidar_get_frame() == 0) {
        printf("new lidar frames but some lidar data are not received\n");
        return false;
    }
    printf("All new lidar frames are ready!\n");
    return true;
}

/** @brief
 * Initialize the camera with the given cam number and open it
 * if it opens return 0 indicating success or if not, 1 indicating
 * failure.
 */
int Camera_Hdr::init_cam(string cam_cmd){
    cap.open(cam_cmd);
    // if an error occured, print and exit
    if(!cap.isOpened()){
        cout<<"Error, unable to open " << cam_cmd <<endl;
        return 0;
    }

    // otherwise return 1 to indicate success
    cout << "Open" << cam_cmd << " success" << endl;
    return 1;
}

int Camera_Hdr::init_cam(int cam_num){
    cam_id = cam_num;
    // use the default camera api, or autodetect it
    int ApiID = cv::CAP_ANY;
    // open the camera with the said id
    cap.open(cam_num, ApiID);
    // if an error occured, print and exit
    if(!cap.isOpened()){
        cout<<"Error, unable to open the required camera"<<endl;
        return 0;
    }
    // otherwise return 0 to indicate success
    return 1;
}

/** @brief
 * Get the latest camera frame, useful for timed calls
 */
int Camera_Hdr::get_frame(){
    if (!cap.read(frame)) {
        std::cout<<"Capture read error"<<std::endl;
        return 0;
    }
    return 1;
}

/** @brief
 * Save the frames to desired folder
 */
int Camera_Hdr::save_frame(string current_date_time, string img_path, string img_name){
    string image_name = img_path + img_name + "_" + current_date_time +".jpg";
    //cout << image_name << endl;
    if (!imwrite(image_name, frame)) {
        std::cout<<"Capture write error"<<std::endl;
        return 0;
    }
    return 1;
}


vector<int> get_camIds(){
    vector<int> camIds;
    VideoCapture cap;
    for (int i=0; i<10; i++){
        cap.open(i);
        if (cap.isOpened()){
            camIds.push_back(i);
        }
        cap.release();
    }
    return camIds;
}


int main( int argc, char* argv[] )
{
    // initialize lidars
    if ( lidar_initialize_ct6() == 0 ) {printf("Lidar init Fail\n"); return 0;}
    //if (lidar_initialize_single() == 0 ) {printf("Lidar init Fail\n"); return 0;}

    // make camera objects
    Camera_Hdr irCamL;
    Camera_Hdr irCamR;
    Camera_Hdr d3RGBCamL;
    Camera_Hdr d3RGBCamR;

    // initialize cameras
    #if 1
    if ( irCamL.init_cam(3) == 0 ) {printf("irCamL Cam init Fail\n"); return 0;}
    if ( irCamR.init_cam(2) == 0 ) {printf("irCamR Cam init Fail\n"); return 0;}
    if ( d3RGBCamL.init_cam("nvarguscamerasrc sensor-id=0 name=rgbcamL maxperf=true aelock=false awblock=false wbmode=1 ! nvvidconv ! video/x-raw, width=1280, height=1080, format=(string)BGRx , framerate=(fraction)30/1 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false drop=true max-buffers=1") == 0 ) {printf("d3RGBCamL Cam init Fail\n"); return 0;}
    if ( d3RGBCamR.init_cam("nvarguscamerasrc sensor-id=1 name=rgbcamR maxperf=true aelock=false awblock=false wbmode=1 ! nvvidconv ! video/x-raw, width=1280, height=1080, format=(string)BGRx , framerate=(fraction)30/1 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false drop=true max-buffers=1") == 0 ) {printf("d3RGBCamR Cam init Fail\n"); return 0;}
    #endif

        
    printf("All sensors are ready to go : Press Enter...\n");
    
    int framecnt=0;
    while(1) {

        // get lidar data
        if (lidar_get_last_frame() == 0 ) {continue;}

        // get camera images
        #if 1
        if (irCamL.get_frame() == 0 ) {printf("irCamL get frame failed\n"); return 0;}
        if (irCamR.get_frame() == 0 ) {printf("irCamR get frame failed\n"); return 0;}
        if (d3RGBCamL.get_frame() == 0 ) {printf("d3RGBCamL get frame failed\n"); return 0;}
        if (d3RGBCamR.get_frame() == 0 ) {printf("d3RGBCamR get frame failed\n"); return 0;}
        #endif

        // get the current time stamp
        string current_time = Get_DateTime();

        cv::imshow("irCamL",irCamL.frame);
        cv::imshow("irCamR",irCamR.frame);
        cv::imshow("d3RGBCamL",d3RGBCamL.frame);
        cv::imshow("d3RGBCamR",d3RGBCamR.frame);

        // write lidar data
        #if 1
        for (int i = 0; i < numOfVelodynes; i++) {
            // open a file with current time
            ofstream file;
            file.open(("image/lidar/lidar_" + std::to_string(i) + "/" + current_time + "_" + std::to_string(i) + ".txt").c_str(), std::ofstream::out);
            std::ostream_iterator<std::string> output_iterator(file, "\n");
            std::copy(vecOfStr[i].begin(), vecOfStr[i].end(), output_iterator);
            // close the file
            file.close();
        }
        #endif

        // write camera data
        #if 1
        if(irCamL.save_frame(current_time, "image/irCamL/","irCamL") == 0) {printf("irCamL write frame failed\n"); return 0;}
        if(irCamR.save_frame(current_time, "image/irCamR/","irCamR") == 0) {printf("irCamR write frame failed\n"); return 0;}
        if(d3RGBCamL.save_frame(current_time, "image/d3RGBCamL/","d3RGBCamL") == 0) {printf("d3RGBCamL write frame failed\n"); return 0;}
        if(d3RGBCamR.save_frame(current_time, "image/d3RGBCamR/","d3RGBCamR") == 0) {printf("d3RGBCamR write frame failed\n"); return 0;}
        #endif

        // Get the pressed value
        int key = (cv::waitKey(30) & 0xFF);
        /*if ( key == 's' || framecnt++ == 3000) {
                printf("recording paused.. Press enter..\n");
                getch();
                framecnt = 0;
                printf("recording resumed..\n");
        }*/
        cout << current_time << endl;
    }
}
