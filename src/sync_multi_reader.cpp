//#include <code_utils/sys_utils.h>
#include "ptgrey_lib/multiCameraReader.h"
#include "preprocess/process.h"
#include <cv_bridge/cv_bridge.h>
#include <flycapture/FlyCapture2.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sstream>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Header.h>
#include <boost/format.hpp>

void
colorToGrey( cv::Mat& image_in, cv::Mat& image_out )
{
    uchar p_out;
    cv::Vec3b p_in;
    for ( int idx_row = 0; idx_row < image_in.rows; ++idx_row )
    {

        for ( int idx_col = 0; idx_col < image_in.cols; ++idx_col )
        {

            p_in  = image_in.at< cv::Vec3b >( idx_row, idx_col);

            // Red * 0.299 + Green * 0.587 + Blue * 0.114
            int dst = p_in[0]   //
                      + p_in[1] //
                      + p_in[2];

            image_out.at< uchar >( idx_row, idx_col) = dst > 255 ? 255 : dst;
        }
    }
}

using namespace ros;

class SyncMultiReader {
    ros::NodeHandle nh;
    ros::Subscriber trigger_time_sub;


    bool is_pub             = true;
    bool is_show            = false;
    bool is_print           = true;
    bool is_auto_shutter    = false;
    bool is_grey            = false;
    bool is_sync            = true;
    double brightness       = 0.1;
    double exposure         = 0.1;
    double gain             = 1.0;
    double frameRate        = 30.0;
    double shutter          = 5.0;
    bool is_first           = true;
    bool no_sync_recved     = true;

    cv::Mat image_grey;
    int src_cols            = 0;
    int src_rows            = 0;
    int cam_cnt             = 1;
    int imageCnt            = 0;

    std::vector< unsigned int > IDs;
    std::vector< ros::Publisher > Publishers;
    std::vector< ros::Publisher > imageROIPublishers;
    std::vector< preprocess::PreProcess* > pres;
    std::vector< bool > is_rois;

    preprocess::PreProcess* pre = nullptr;

    bool trigger_time_vaild = true;
    std_msgs::Header tri_header;
    sensor_msgs::TimeReference trigger_time;
    std::vector<std_msgs::Header> trigger_ts_queue;

    ptgrey_reader::multiCameraReader * camReader;

    cv_bridge::CvImage outImg;

    void on_time_reference(const sensor_msgs::TimeReference & _ref) {
        double from_last = _ref.header.stamp.toSec() - tri_header.stamp.toSec();
        double from_last2 = ros::Time::now().toSec() - tri_header.stamp.toSec();
        tri_header = _ref.header;

        trigger_ts_queue.push_back(tri_header);

        trigger_time_vaild = true;
        no_sync_recved = false;
        ROS_INFO_THROTTLE(1.0, "tri recved align %3.2fms dt %3.2f %3.2fms", (ros::Time::now() - _ref.time_ref).toSec()*1000, from_last*1000, from_last2*1000);
        trigger_time = _ref;
        auto ts = ros::Time::now();
        grab();
        ROS_INFO_THROTTLE(1.0, "grab time cost %4.2f", (ros::Time::now() - ts).toSec()*1000);
    }

    void grab() {
        bool need_regrab = true;
        std::vector< std::pair< cv::Mat, FlyCapture2::TimeStamp > > images_tmp;
        images_tmp.resize( cam_cnt );

        while (need_regrab) {
            auto ts = ros::Time::now();

            FlyCapture2::Error error;
            std::vector< FlyCapture2::TimeStamp > cam_time;

            camReader->Cameras()->captureImage( error, images_tmp );
            ++imageCnt;
            double dt_grab = (ros::Time::now() - ts).toSec() * 1000; 
            ROS_INFO_THROTTLE(1.0, "grab only time cost %4.2f", dt_grab);

            if ( images_tmp.at( 0 ).first.empty( ) ) {
                std::cout << "[#INFO] Grabbed no image." << std::endl;
                return;
            }

            outImg.header.stamp.sec  = images_tmp.at( 0 ).second.seconds;
            outImg.header.stamp.nsec = images_tmp.at( 0 ).second.microSeconds * 1000;

            auto image_ros_time = outImg.header.stamp;
            auto trigger_ros_time = trigger_time.time_ref;
            auto dt_trigger_image_ms = (trigger_ros_time - image_ros_time).toSec()*1000;

            if (is_sync) {
                outImg.header.stamp = tri_header.stamp;
                //Still have bugs... that trigger is faster than image, should be slower..
                if (is_print)
                    ROS_INFO("using trigger ts, dt %3.2f ms", dt_trigger_image_ms);
                
                if (dt_trigger_image_ms > 0) {
                    ROS_WARN("Image too old, will regrab");
                    continue;
                    //need_regrab = false;
                } 
                else {
                    need_regrab = false;
                }
                
                if (fabs( dt_trigger_image_ms + dt_grab) > 10) {
                    ROS_WARN("using unexpect trigger ts, dt ros %3.2fms ros-fc %3.2fms", dt_trigger_image_ms, (tri_header.stamp - image_ros_time).toSec()*1000);
                } 
                else {
                    ROS_INFO("Dt ros %3.2fms ros-fc %3.2fms", dt_trigger_image_ms + dt_grab);
                }                
            } 
            else {
                need_regrab = false;
            }
        }

        if (is_print) {
            std::cout << "Grabbed image " << imageCnt << std::endl;            
        }

        if (is_pub) {

            outImg.header.stamp.sec  = images_tmp.at( 0 ).second.seconds;
            outImg.header.stamp.nsec = images_tmp.at( 0 ).second.microSeconds * 1000;

            ros::Time t1 = ros::Time::now( );
            ros::Time t2;
            ros::Time t3;
            t2.sec  = images_tmp.at( 0 ).second.seconds;
            t2.nsec = images_tmp.at( 0 ).second.microSeconds * 1000;
            t3.sec  = images_tmp.at( 1 ).second.seconds;
            t3.nsec = images_tmp.at( 1 ).second.microSeconds * 1000;

            ros::Duration dt1 = t1 - t2;
            ros::Duration dt2 = t1 - t3;
            ros::Duration dt3 = t3 - t2;
            std::cout << "dt1 " << dt1.toSec( ) << std::endl;
            std::cout << "dt2 " << dt2.toSec( ) << std::endl;
            std::cout << "dt3 " << dt3.toSec( ) << std::endl;

            outImg.header.frame_id = "frame";
            if ( camReader->Cameras( )->isColorCamera( ) )
            {
                if (is_grey)    
                    outImg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
                else
                    outImg.encoding = sensor_msgs::image_encodings::BGR8;
            }

            else
                outImg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;


            for ( int pub_index = 0; pub_index < cam_cnt; ++pub_index )
            {
                if (is_grey){
                    if ( is_first )
                    {
                        src_rows = images_tmp.at( pub_index ).first.rows;
                        src_cols = images_tmp.at( pub_index ).first.cols;
                        cv::Mat img_tmp( src_rows, src_cols, CV_8UC1 );
                        img_tmp.copyTo( image_grey );
                        is_first = false;
                    }
                    colorToGrey( images_tmp.at( pub_index ).first, image_grey );
                    images_tmp.at( pub_index ).first = image_grey;
                }
                outImg.image = images_tmp.at( pub_index ).first;

                Publishers.at( pub_index ).publish( outImg );

                if ( is_rois.at( pub_index ) )
                {
                    outImg.image
                    = pres.at( pub_index )->do_preprocess( images_tmp.at( pub_index ).first      );
                    imageROIPublishers.at( pub_index ).publish( outImg );
                }
            }
        }

        if ( is_show ) {
            cv::namedWindow( "image", CV_WINDOW_NORMAL );
            cv::namedWindow( "image2", CV_WINDOW_NORMAL );
            if (is_grey){
                cv::imshow("ImgGrey", image_grey);
            }
            cv::waitKey( 10 );
        }
    }

public:
    SyncMultiReader(ros::NodeHandle & _nh): nh(_nh) {
        nh.getParam( "is_pub", is_pub );
        nh.getParam( "is_show", is_show );
        nh.getParam( "is_print", is_print );
        nh.getParam( "is_grey", is_grey );
        nh.getParam( "is_auto_shutter", is_auto_shutter );
        nh.getParam( "is_sync", is_sync );
        nh.getParam( "brightness", brightness );
        nh.getParam( "exposure", exposure );
        nh.getParam( "gain", gain );
        nh.getParam( "frameRate", frameRate );
        nh.getParam( "shutter", shutter );
        nh.getParam( "cam_cnt", cam_cnt );

        for ( int i = 0; i < cam_cnt; i++ ) {
            std::string prefix = boost::str( boost::format( "camera%d/" ) % i );
            int serialNum      = 17532721;
            std::string topic;
            std::string topic_roi;
            bool is_roi              = false;
            double down_sample_scale = 0.75;
            int size_x = 0, size_y = 0;
            int center_x = 0, center_y = 0;
            int cropper_x = 0, cropper_y = 0;

            nh.getParam( prefix + "serialNum", serialNum );
            nh.getParam( prefix + "topic", topic );
            nh.getParam( prefix + "topic_roi", topic_roi );
            nh.getParam( prefix + "is_roi", is_roi );
            nh.getParam( prefix + "down_sample_scale", down_sample_scale );
            nh.getParam( prefix + "size_x", size_x );
            nh.getParam( prefix + "size_y", size_y );
            nh.getParam( prefix + "center_x", center_x );
            nh.getParam( prefix + "center_y", center_y );
            nh.getParam( prefix + "cropper_x", cropper_x );
            nh.getParam( prefix + "cropper_y", cropper_y );

            preprocess::PreProcess* pre = new preprocess::PreProcess(   cv::Size( size_x, size_y ),
                                                                        cv::Size( cropper_x, cropper_y ),
                                                                        cv::Point( center_x, center_y ),
                                                                        down_sample_scale );

            pres.push_back( pre );
            unsigned int cameraId = serialNum;
            IDs.push_back( cameraId );
            ros::Publisher imagePublisher    = nh.advertise< sensor_msgs::Image >( topic, 3 );
            ros::Publisher imageROIPublisher = nh.advertise< sensor_msgs::Image >( topic_roi, 3 );
            Publishers.push_back( imagePublisher );
            imageROIPublishers.push_back( imageROIPublisher );
            is_rois.push_back( is_roi );

        } 

        camReader = new ptgrey_reader::multiCameraReader(IDs);

        if ( is_show )
        {
            cv::namedWindow( "image", CV_WINDOW_NORMAL );
            cv::namedWindow( "image2", CV_WINDOW_NORMAL );
        }
        bool is_cameraStarted
        = camReader->startCamera( IDs, frameRate, brightness, exposure, gain, is_auto_shutter, shutter, is_print, is_sync );

        if ( !is_cameraStarted )
        {
            ros::shutdown( );
            std::cout << "[#INFO] Camera cannot start" << std::endl;
        }
        else {
            ROS_INFO("Cameras started");
        }
        // if ( is_grey && camReader->Cameras( ).isColorCamera( ) )
        // {
        //     imageGreyPublisher = nh.advertise< sensor_msgs::Image >( "/image_grey", 3 );
        // }
        
        if (is_sync) {
            ROS_INFO("Is trigger, subscribe to time reference");
            trigger_time_sub = nh.subscribe("/dji_sdk_1/dji_sdk/trigger_time", 1, &SyncMultiReader::on_time_reference, this, ros::TransportHints().tcpNoDelay());
        }

        std::cout << "[#INFO] Loop start." << ros::ok( ) << std::endl;
    }

    ros::Timer timer;    

    void stop() {
        camReader->stopCamera( );
        std::cout << "[#INFO] stop Camera Done!" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sync_multi_reader");
    ros::NodeHandle nh("~");

    ROS_INFO("Start Sync Multi Reader");
    SyncMultiReader smr(nh);
    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin();
}
