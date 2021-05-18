#include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector> 
#include <string>
// #include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
// #include <tf2_ros/static_transform_broadcaster.h>
#include <urdf/model.h>
#include <opencv2/features2d.hpp>
#include <camera_info_manager/camera_info_manager.h>


#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <visp_bridge/3dpose.h>

#include <visp3/core/vpConfig.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/vision/vpPose.h>


//To grab ROS images instead of callback
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp_ros/vpROSGrabber.h>


using namespace std;
using namespace sensor_msgs;

class PLanarPoseRGBD
{
    public:
	    PLanarPoseRGBD(ros::NodeHandle nh_);
	    ~PLanarPoseRGBD();
        void poseDetect();
        // void initDisplay();
       
    protected:
        void imageCB(const sensor_msgs::ImageConstPtr& msg);
        void depthImageCB(const sensor_msgs::ImageConstPtr& msg);
        void infoCamCB(const sensor_msgs::CameraInfo& msg);
        

        ros::NodeHandle nh_;    
        ros::Subscriber colorCamInfoSub;
        ros::Subscriber transform_sub;

        image_transport::ImageTransport    _imageTransport;
        image_transport::Subscriber        image_sub;
        image_transport::Subscriber        deph_image_sub;

        sensor_msgs::ImageConstPtr rgbImage;
        sensor_msgs::ImageConstPtr depthImage;
        
        vpCameraParameters colorCamInfoVisp;

        vpImage<vpRGBa> vRGBImage(640,480);
        vpImage<vpRGBa> vRGBImage2;
        vpImage<unsigned char> vDepthImage;
        vpImage<uint16_t> vDepthImage_raw;
        // vpDisplay* display;
        vpImage<float> vDepthMap;

        vpDisplayX *dDepth = NULL;
        vpDisplayX *dRGB   = NULL;
        vpDisplayX *dRGB2  = NULL;
        // vpDisplayX dRGB;
        string target_frame_;
        string strImage_sub_topic;
        string strDepthImage_sub_topic;
        string strImage_pub_topic;
        string strCameraInfo_sub_topic;
        string strTransform_sub_topic;
        unsigned int width, height; 

        bool bImageShow;
        bool bDepthImageShow;
        bool display_off;

        //  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
        vpDetectorAprilTag detector;
        // vpCameraParameters cam;
        float quad_decimate;
        bool display_tag;
        int color_id;
        unsigned int thickness;
        double tagSize;
        float depth_scale;
};

// PLanarPoseRGBD::PLanarPoseRGBD(ros::NodeHandle nh_): _imageTransport(nh_), 
//                                                   cinfo_(new camera_info_manager::CameraInfoManager(nh_))
PLanarPoseRGBD::PLanarPoseRGBD(ros::NodeHandle nh_): _imageTransport(nh_)
{       
    //Parameters for topics 
    nh_.param<std::string>("strImage_sub_topic",      strImage_sub_topic,      "/xtion/rgb/image_raw");
    nh_.param<std::string>("strDepthImage_sub_topic", strDepthImage_sub_topic, "/xtion/depth/image_raw");
    nh_.param<std::string>("strCameraInfo_sub_topic", strCameraInfo_sub_topic, "/xtion/rgb/camera_info");
    // nh_.param<std::string>("strTransform_sub_topic", strTransform_sub_topic, "/agimus/vision/tags");

    //publisher & subcriber 
    // image_transport::TransportHints th("compressed");
    image_sub = _imageTransport.subscribe(strImage_sub_topic, 1, &PLanarPoseRGBD::imageCB, this, image_transport::TransportHints("raw")); 
    ROS_INFO("Subcribed to the topic: %s", strImage_sub_topic.c_str());

    deph_image_sub = _imageTransport.subscribe(strDepthImage_sub_topic, 1, &PLanarPoseRGBD::depthImageCB, this, image_transport::TransportHints("raw")); 
    ROS_INFO("Subcribed to the topic: %s", strDepthImage_sub_topic.c_str());

    colorCamInfoSub =  nh_.subscribe(strCameraInfo_sub_topic, 1, &PLanarPoseRGBD::infoCamCB, this);
    ROS_INFO("Subcribed to the topic: %s", strCameraInfo_sub_topic.c_str());

    rgbImage    = NULL;
    depthImage  = NULL;
    
    width = 640; 
    height = 480; 
    // display = new vpDisplayX();
    bImageShow = false;
    bDepthImageShow = false;
    display_off = true;

    quad_decimate = 1.0;
    display_tag = true;
    color_id = -1;
    thickness = 2;
    tagSize = 0.1725;


    detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
    detector.setAprilTagNbThreads(1);
    detector.setDisplayTag(display_tag, color_id < 0 ? vpColor::none : vpColor::getColor(color_id), thickness);
    detector.setZAlignedWithCameraAxis(false);

      vpImage<vpRGBa> color_image(240, 320);
    vpRGBa color(255, 0, 0);
    vRGBImage.init(width,height, vpRGBa(255,255,255));
    dRGB   = new vpDisplayX(vRGBImage, 100, 30, "Pose from Homography");

    // vRGBImage.init(width,height, vpRGBa(0,0,0));
    // dRGB   = new vpDisplayX(vRGBImage, 100, 30, "Pose from Homography");
    // dRGB   = new vpDisplayX();
    // dRGB2  = new vpDisplayX(vRGBImage2, vRGBImage.getWidth()+120, 30, "Pose from RGBD fusion");
    // dDepth = new vpDisplayX(vDepthImage, 100, vRGBImage.getHeight()+70, "Depth");
}

PLanarPoseRGBD::~PLanarPoseRGBD()
{
	cv::destroyAllWindows();
}vRGBImage
      // Stop the subscriber
      this->colorCamInfoSub.shutdown();
}

void PLanarPoseRGBD::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("imageCB");
    cv::Mat img;
	cv_bridge::CvImagePtr cvPtr;
    rgbImage = msg;

    try
	{
       vRGBImage = visp_bridge::toVispImageRGBa(*rgbImage);
        // vpDisplayX dRGB(vRGBImage);
        std::cout << "Color Image size: " << vRGBImage.getWidth() << " " << vRGBImage.getHeight() << std::endl;
        if (!display_off)
        {
            vpDisplay::display(vRGBImage);
            vpDisplay::displayText(vRGBImage, 640, 480, "A click to quit...", vpColor::red);
            vpDisplay::flush(vRGBImage);
            while (1)
            {
                if (vpDisplay::getClick(vRGBImage, false) )
                    break;
            }
        }

       
	}
    catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("imageCB cv_bridge exception: %s", e.what());
		return;
	}

   
}

void PLanarPoseRGBD::depthImageCB(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("depthImageCB");
    cv::Mat img, imgNormalized;
	cv_bridge::CvImagePtr cvPtr;
    depthImage = msg;   
    // vDepthImage_raw
    try
	{
        
        if ("16UC1" == msg->encoding)
        {
            ROS_INFO("TYPE_16UC1");
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
            // vDepthImage_raw  = visp_bridge::toVispImage(*(cvPtr->toImageMsg()));
            // cv::imshow("Image window", cvPtr->image);
            // cv::waitKey(3);

            vpImageConvert::convert(cvPtr->image, vDepthImage);
            vpDisplayX dDepth(vDepthImage);
            std::cout << "Image size: " << vDepthImage.getWidth() << " " << vDepthImage.getHeight() << std::endl;
            if (!display_off)
            {
                vpDisplay::display(vDepthImage);
                vpDisplay::displayText(vDepthImage, 640, 480, "A click to quit...", vpColor::red);
                vpDisplay::flush(vDepthImage);
                while (1)
                {
                    if (vpDisplay::getClick(vDepthImage, false) )
                        break;
                }
            }
        }
        else if ("32FC1" == msg->encoding)
        {
            ROS_INFO("TYPE_32FC1");
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
    
	}
    catch (cv_bridge::Exception& e) 
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

   
}
    
//output
void PLanarPoseRGBD::poseDetect()
{
    vpImage<unsigned char> vGrayImage;
    vpImage<vpRGBa>          vDepth;
    vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);

    try {
        if (vDepthImage.getWidth() > 0 && vDepthImage.getHeight() > 0 && vRGBImage.getWidth() > 0 && vRGBImage.getHeight() > 0)
        {
            vpImage<vpRGBa> vRGBImage2 = vRGBImage;
            vpImage<unsigned char> vGrayImage;
            vpImage<float> depthMap;
          
           
            // dRGB2  = new vpDisplayX(vRGBImage2, vRGBImage.getWidth()+120, 30, "Pose from RGBD fusion");
            // dDepth = new vpDisplayX(vDepthImage, 100, vRGBImage.getHeight()+70, "Depth");

            vpDisplay::display(vRGBImage);
            // vpDisplay::display(vRGBImage2);
            // vpDisplay::display(vDepthImage);


            // vpImageConvert::convert(vDepthImage, vDepthImage_raw);
            vpImageConvert::createDepthHistogram(vDepthImage_raw, vDepth);
            vpImageConvert::convert(vRGBImage, vGrayImage);
            depth_scale = 1.0;
            depthMap.resize(vDepthImage.getHeight(), vDepthImage.getWidth());
            for (unsigned int i = 0; i < vDepthImage.getHeight(); i++) {
                for (unsigned int j = 0; j < vDepthImage.getWidth(); j++) {
                    if (vDepthImage[i][j]) {
                        float Z = vDepthImage[i][j] * depth_scale;
                        depthMap[i][j] = Z;
                    } else {
                        depthMap[i][j] = 0;
                    }
                }
            }


            // vpDisplayX *dDepth = NULL;
            // vpDisplayX *dRGB   = NULL;
            // vpDisplayX *dRGB2  = NULL;

          
            std::cout << "Histogram Image size: " << vGrayImage.getWidth() << " " << vGrayImage.getHeight() << std::endl;
        
            std::vector<vpHomogeneousMatrix> cMo_vec;
            detector.detect(vGrayImage, tagSize, colorCamInfoVisp, cMo_vec );
            std::cout << cMo_vec.size();
            
            // // Display camera pose for each tag
            for (int i = 0; i < cMo_vec.size(); i++) {
                vpDisplay::displayFrame(vRGBImage, cMo_vec[i], colorCamInfoVisp, tagSize / 2, vpColor::none, 3);
                // std::cout << "passed \n";
            }
           

            // std::vector<std::vector<vpImagePoint> > tags_corners = detector.getPolygon();
            // std::vector<int> tags_id = detector.getTagsId();
            // std::map<int, double> tags_size;
            // tags_size[-1] = tagSize; // Default tag size
            // std::vector<std::vector<vpPoint> > tags_points3d = detector.getTagsPoints3D(tags_id, tags_size);
            // for (size_t i = 0; i < tags_corners.size(); i++) {
            //     vpHomogeneousMatrix cMo;
            //     double confidence_index;
            //     if (vpPose::computePlanarObjectPoseFromRGBD(depthMap, tags_corners[i], colorCamInfoVisp, tags_points3d[i], cMo, &confidence_index)) {
            //             if (confidence_index > 0.5) {
            //                 // std::cout << "DISPLAY " << std::endl;
            //                 vpDisplay::displayFrame(vRGBImage2, cMo, colorCamInfoVisp, tagSize/2, vpColor::none, 3);
            //                 }
            //             else if (confidence_index > 0.25) {
            //             vpDisplay::displayFrame(vRGBImage2, cMo, colorCamInfoVisp, tagSize/2, vpColor::orange, 3);
            //         }
            //         else {
            //             vpDisplay::displayFrame(vRGBImage2, cMo, colorCamInfoVisp, tagSize/2, vpColor::red, 3);
            //         }
            //         std::stringstream ss;
            //         ss << "Tag id " << tags_id[i] << " confidence: " << confidence_index;
            //         vpDisplay::displayText(vRGBImage2, 35 + i*15, 20, ss.str(), vpColor::red);

                    
            //     }
            // }

            vpDisplay::displayText(vRGBImage, 20, 20, "Pose from homography + VVS", vpColor::red);
            // vpDisplay::displayText(vRGBImage2, 50, 50, "Pose from RGBD fusion", vpColor::red);
            // vpDisplay::displayText(vRGBImage, 80, 80, "Click to quit.", vpColor::red);
          

            // if (vpDisplay::getClick(vRGBImage, false))
                // break;
            // vpDisplay::flush(vDepthImage);
            // vpDisplay::flush(vRGBImage2);
            vpDisplay::flush(vRGBImage);
            // while (1)
            // {
            //     if (vpDisplay::getClick(vRGBImage2, false) || vpDisplay::getClick(vRGBImage, false) || vpDisplay::getClick(vDepthImage, false) )
            //         break;
            // }
        }
 
    }
    catch(vpException e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planar_pose");
  ros::NodeHandle nh;  
  PLanarPoseRGBD ps(nh);

  ros::Rate loop_rate(100);
  while(ros::ok()) {
    // ROS_INFO("Looping");
    ros::spinOnce();
    ps.poseDetect();
    loop_rate.sleep();
  }
}