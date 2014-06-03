#include <iostream>

#include <cv.h>
#include <highgui.h>

#include <FlyCapture2.h>

#include "elas.h"



#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>




#define USE_OPENCV
//#define USE_ELAS


//! \brief Printing error
//!
//! \param FlyCapture2::Error error
//!
//!
void PrintError( FlyCapture2::Error error )
{
    error.PrintErrorTrace();
}


#define PRINT_ERROR( error ) \
if (error != FlyCapture2::PGRERROR_OK) \
{ \
    PrintError( error ); \
    return -1; \
}

#define PRINT_ERROR_MSG( error, msg ) \
if (error != FlyCapture2::PGRERROR_OK) \
{ \
    PrintError( error ); \
    std::cerr << msg << std::endl; \
    getchar(); \
    return -1; \
}



//! \brief Printing camera info
//!
//! \param FlyCapture2::CameraInfo* pCamInfo
//!
//!
void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo )
{
    std::cout <<
              "*** CAMERA INFORMATION ***" << std::endl <<
              "Serial number - "      << pCamInfo->serialNumber << std::endl <<
              "Camera model -  "      << pCamInfo->modelName << std::endl <<
              "Camera vendor - "      << pCamInfo->vendorName << std::endl <<
              "Sensor -        "      << pCamInfo->sensorInfo << std::endl <<
              "Resolution -    "      << pCamInfo->sensorResolution << std::endl <<
              "Firmware version -   " << pCamInfo->firmwareVersion << std::endl <<
              "Firmware build time -" << pCamInfo->firmwareBuildTime << std::endl;

}


//! \brief load calibration info used by cv::remap()
//!
//! \param cv::Mat& mx1
//! \param cv::Mat& my1
//! \param cv::Mat& mx2
//! \param cv::Mat& my2
//! \return mx1, my1, mx2, my2
//!
//!
void loadCalibrationInfo(cv::Mat& mx1,
                         cv::Mat& my1,
                         cv::Mat& mx2,
                         cv::Mat& my2)
{
//	cv::FileStorage fs ("../calibrationResult/Q.xml",   cv::FileStorage::READ);
    cv::FileStorage fx1("../calibrationResult/mx1.xml", cv::FileStorage::READ);
    cv::FileStorage fx2("../calibrationResult/mx2.xml", cv::FileStorage::READ);
    cv::FileStorage fy1("../calibrationResult/my1.xml", cv::FileStorage::READ);
    cv::FileStorage fy2("../calibrationResult/my2.xml", cv::FileStorage::READ);

//	fs["Q"] >> Q;
    fx1["mx1"] >> mx1;
    fx2["mx2"] >> mx2;
    fy1["my1"] >> my1;
    fy2["my2"] >> my2;

}



int main( int /*argc*/, char** /*argv*/ )
{
    const int k_numImages = 100000;
    FlyCapture2::Error error;

    cv::Mat Q, M1, D1, R1, P1, M2, D2, R2, P2;



    cv::Mat mx1, my1, mx2, my2;
    loadCalibrationInfo(mx1, my1, mx2, my2);








    FlyCapture2::BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras( &numCameras );
    PRINT_ERROR( error );
    std::cout << "Number of cameras detected: " << numCameras << std::endl;

    if ( numCameras < 1 )
    {
        std::cerr << "Insufficient number of cameras... press Enter to exit." << std::endl;
        getchar();
        return -1;
    }



    FlyCapture2::Camera** ppCameras = new FlyCapture2::Camera* [numCameras];


    // Connect to all detected cameras and attempt to set them to
    // a common video mode and frame rate
    for ( unsigned int i = 0; i < numCameras; i++)
    {
        ppCameras[i] = new FlyCapture2::Camera();

        std::cout << "setting camera " << i << std::endl;

        FlyCapture2::PGRGuid guid;
        error = busMgr.GetCameraFromIndex( i, &guid );
        PRINT_ERROR( error );

        //!< Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        PRINT_ERROR( error );


        //!< Get the camera information
        FlyCapture2::CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        PRINT_ERROR( error );


        PrintCameraInfo( &camInfo );

        // Set all cameras to a specific mode and frame rate so they
        // can be synchronized
        error = ppCameras[i]->SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_640x480Y8,
        FlyCapture2::FRAMERATE_60 );
        PRINT_ERROR_MSG( error,
        "Error starting cameras. \n"
        "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
        "If your camera does not support this mode, please edit the source code and recompile the application. \n"
        "Press Enter to exit. \n");
    }


    std::cout << "all camera setted to specific mode." << std::endl;


    error = FlyCapture2::Camera::StartSyncCapture( numCameras, (const FlyCapture2::Camera**)ppCameras );
    PRINT_ERROR_MSG( error,
    "Error starting cameras. \n"
    "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
    "If your camera does not support this mode, please edit the source code and recompile the application. \n"
    "Press Enter to exit. \n");


    std::cout << "StartSyncCapture." << std::endl;




//  cv::Mat imageCamera(rawImage.GetRows(), rawImage.GetCols(), CV_8UC1);
    cv::Mat imageCamera0(480, 640, CV_8UC1);
    cv::Mat imageCamera1(480, 640, CV_8UC1);
    cv::Mat imageDisparity(480, 640, CV_32F), disp8U;
    cv::Mat* pimageCamera[]= {&imageCamera0, &imageCamera1};
    cv::namedWindow( "cam0", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "cam1", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "disparity", CV_WINDOW_AUTOSIZE );
#ifdef USE_OPENCV
    int ndisparities = 128;
    cv::StereoBM SBM(cv::StereoBM::NARROW_PRESET, ndisparities, 11);
#endif

#ifdef USE_ELAS
    Elas::parameters param;
    param.postprocess_only_left = true;
    param.disp_min = -64;
    param.disp_max = 128;
    const int32_t dims[3] = {640, 480, 640};
    Elas elas(param);
    //float* D1_data = (float*)malloc(640*480*sizeof(float));
    //float* D2_data = (float*)malloc(width*height*sizeof(float));
#endif


    for ( int j = 0; j < k_numImages; j++ )
    {
        // Display the timestamps for all cameras to show that the image
        // capture is synchronized for each image



        for ( unsigned int i = 0; i < numCameras; i++ )
        {
            FlyCapture2::Image rawImage;
            error = ppCameras[i]->RetrieveBuffer( &rawImage );
            PRINT_ERROR( error );



            memcpy( pimageCamera[i]->data, rawImage.GetData(), rawImage.GetDataSize() );


        }
        cv::remap( *pimageCamera[1], *pimageCamera[1], mx1, my1, CV_INTER_LINEAR );
        cv::remap( *pimageCamera[0], *pimageCamera[0], mx2, my2, CV_INTER_LINEAR );

#ifdef USE_OPENCV
        SBM(*pimageCamera[1] , *pimageCamera[0] , imageDisparity, CV_16S);
#endif

#ifdef USE_ELAS
        int static skip = 0;
        if (skip % 15 ==0)
        {
            elas.process(imageCamera1.data,imageCamera0.data, (float*)imageDisparity.data,(float*)imageDisparity.data,dims);
        }
        skip++;
#endif

        double minVal, maxVal;
        minMaxLoc( imageDisparity, &minVal, &maxVal );
        imageDisparity.convertTo( disp8U, CV_8UC1, 255/(maxVal - minVal) );

        imshow( "disparity", disp8U );
        cv::imshow( "cam0", *pimageCamera[1] );
        cv::imshow( "cam1", *pimageCamera[0] );
        cv::imshow( "sub",  cv::abs( *pimageCamera[0] - *pimageCamera[1] ) );


        char keyValue = cv::waitKey( 10 );
        if  (keyValue == 'q' || keyValue == 27 /* ESC */ )
        {
            break;
        }

    }

    exit(0); // hack; code below may freeze.

    for ( unsigned int i = 0; i < numCameras; i++ )
    {
        ppCameras[i]->StopCapture();
        ppCameras[i]->Disconnect();
        delete ppCameras[i];
    }

    delete [] ppCameras;




    return 0;
}
