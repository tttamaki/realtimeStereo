//!<
//!< OpenCV version > git clone 2014/Jun/03
//!<

#include <iostream>
#include <vector>

#include <cv.hpp> //!< OpenCV C++ class. not "cv.h" which is for C.

#include <FlyCapture2.h> //!< Point Grey SDK FlyCapture2 version == 2.6.3.2

#include "elas.h" //!< Geiger's ACCV2010 paper implementation

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>




//#define USE_OPENCV
#define USE_ELAS


//! \brief Printing error
//!
//! \param FlyCapture2::Error error
//!
//!
void PrintError( const FlyCapture2::Error error )
{
    error.PrintErrorTrace();
}


#define PRINT_ERROR( error ) \
if (error != FlyCapture2::PGRERROR_OK) \
{ \
    PrintError( error ); \
    exit(-1); \
}

#define PRINT_ERROR_MSG( error, msg ) \
if (error != FlyCapture2::PGRERROR_OK) \
{ \
    PrintError( error ); \
    std::cerr << msg << std::endl; \
    getchar(); \
    exit(-1); \
}



//! \brief Printing camera info
//!
//! \param FlyCapture2::CameraInfo* pCamInfo
//!
//!
void PrintCameraInfo( const FlyCapture2::CameraInfo &pCamInfo )
{
    std::cout <<
              "*** CAMERA INFORMATION ***" << std::endl <<
              "Serial number - "      << pCamInfo.serialNumber << std::endl <<
              "Camera model -  "      << pCamInfo.modelName << std::endl <<
              "Camera vendor - "      << pCamInfo.vendorName << std::endl <<
              "Sensor -        "      << pCamInfo.sensorInfo << std::endl <<
              "Resolution -    "      << pCamInfo.sensorResolution << std::endl <<
              "Firmware version -   " << pCamInfo.firmwareVersion << std::endl <<
              "Firmware build time -" << pCamInfo.firmwareBuildTime << std::endl;

}


//! \brief Printing camera info
//!
//! \param FlyCapture2::Camera* pCamera
//!
//!
void PrintCameraInfo( FlyCapture2::Camera* pCamera )
{
    FlyCapture2::CameraInfo camInfo;
    FlyCapture2::Error error = pCamera->GetCameraInfo( &camInfo );
    PRINT_ERROR( error );
    PrintCameraInfo( camInfo );
}


//! \brief Printing camera property info
//!
//! \param const FlyCapture2::PropertyInfo &pPropInfo
//!
//!
void PrintCameraPropertyInfo( const FlyCapture2::PropertyInfo &pPropInfo )
{
    #define PNAMES(name) #name << ": " << pPropInfo.name << std::endl

    std::cout <<
            PNAMES(present) <<
            PNAMES(autoSupported) <<
            PNAMES(manualSupported) <<
            PNAMES(onOffSupported) <<
            PNAMES(onePushSupported) <<
            PNAMES(absValSupported) <<
            PNAMES(readOutSupported) <<
            PNAMES(min) <<
            PNAMES(max) <<
            PNAMES(absMin) <<
            PNAMES(absMax) <<
            PNAMES(pUnits) <<
            PNAMES(pUnitAbbr);

    #undef PNAMES

}

//! \brief Printing camera property
//!
//! \param const FlyCapture2::Property &pProp
//!
//!
void PrintCameraProperty( const FlyCapture2::Property &pProp )
{
    #define PNAMES(name) #name << ": " << pProp.name << std::endl

    std::cout <<
            PNAMES(present) <<
            PNAMES(absControl) <<
            PNAMES(onePush) <<
            PNAMES(onOff) <<
            PNAMES(autoManualMode) <<
            PNAMES(valueA) <<
            PNAMES(valueB) <<
            PNAMES(absValue);

    #undef PNAMES
}



//! \brief Printing camera property info
//!
//! \param FlyCapture2::Camera* pCamera
//!
//!
void PrintCameraPropertyInfo( FlyCapture2::Camera* pCamera )
{

    std::vector< FlyCapture2::PropertyType > types =
        {FlyCapture2::AUTO_EXPOSURE,
         FlyCapture2::SHUTTER,
         FlyCapture2::GAIN,
         FlyCapture2::GAMMA,
         FlyCapture2::FRAME_RATE,
         FlyCapture2::TEMPERATURE
        }; //!< -std=c++11 initialization
    std::vector< std::string > typenames =
    {"AUTO_EXPOSURE",
     "SHUTTER",
     "GAIN",
     "GAMMA",
     "FRAME_RATE",
     "TEMPERATURE"
     }; //!< -std=c++11 initialization

    FlyCapture2::PropertyInfo pPropInfo;
    FlyCapture2::Property pProp;
    FlyCapture2::Error error;

    for ( unsigned int i = 0; i < types.size(); i++ )
    {

        pPropInfo.type = types[i];
        error = pCamera->GetPropertyInfo( &pPropInfo );
        PRINT_ERROR( error );
        std::cout << "*** CAMERA PROPERTY INFO : " << typenames[i] << std::endl;
        PrintCameraPropertyInfo( pPropInfo );

        pProp.type = types[i];
        error = pCamera->GetProperty( &pProp );
        PRINT_ERROR( error );
        std::cout << "*** CAMERA PROPERTY : " << typenames[i] << std::endl;
        PrintCameraProperty( pProp );
    }

}



//! \brief Printing absValue of a camera property
//!
//! \param FlyCapture2::Camera* pCamera
//! \param FlyCapture2::PropertyType type
//!
//!
float getCameraPropertyAbsValue( FlyCapture2::Camera* pCamera, FlyCapture2::PropertyType type)
{
    FlyCapture2::Property pProp;
    pProp.type = type;
    FlyCapture2::Error error = pCamera->GetProperty( &pProp );
    PRINT_ERROR( error );
    return pProp.absValue;
}

//! \brief Printing valueA of a camera property
//!
//! \param FlyCapture2::Camera* pCamera
//! \param FlyCapture2::PropertyType type
//!
//!
unsigned int getCameraPropertyValueA( FlyCapture2::Camera* pCamera, FlyCapture2::PropertyType type)
{
    FlyCapture2::Property pProp;
    pProp.type = type;
    FlyCapture2::Error error = pCamera->GetProperty( &pProp );
    PRINT_ERROR( error );
    return pProp.valueA;
}



//! \brief draw strings of camera info on image
//!
//! \param FlyCapture2::Camera* pCamera
//! \param cv::Mat &img
//!
//!
void drawCameraInfo( FlyCapture2::Camera* pCamera, cv::Mat &img)
{

    #define DRAWTXT(img, str, x, y, s) \
        cv::putText( img, str, cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, s, cv::Scalar::all(255) )

    std::string msg;

    msg = std::string("shutter speed: ")
        + std::to_string( getCameraPropertyAbsValue( pCamera, FlyCapture2::SHUTTER ) )
        + std::string(" [ms]");
    DRAWTXT( img, msg, 10, 20, 0.5 );

    msg = std::string("gain: ")
        + std::to_string( getCameraPropertyAbsValue( pCamera, FlyCapture2::GAIN ) )
        + std::string(" [dB]");
    DRAWTXT( img, msg, 10, 50, 0.5 );

    msg = std::string("AE: ")
        + std::to_string( getCameraPropertyAbsValue( pCamera, FlyCapture2::AUTO_EXPOSURE ) )
        + std::string(" [EV]");
    DRAWTXT( img, msg, 10, 80, 0.5 );

    msg = std::string("temperature: ")
        + std::to_string( getCameraPropertyAbsValue( pCamera, FlyCapture2::TEMPERATURE ) )
        + std::string(" [Celcius]");
    DRAWTXT( img, msg, 10, 110, 0.5 );

    #undef DRAWTXT
}












//! \brief load calibration info used by cv::remap()
//!
//! \return cv::Mat& mx1
//! \return cv::Mat& my1
//! \return cv::Mat& mx2
//! \return cv::Mat& my2
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



//! \brief Connect cameras and return camera objects
//!
//! \return unsigned int &numCameras
//! \return FlyCapture2::Camera** &ppCameras
//!
//!
void prepareCameras(unsigned int &numCameras,
                    FlyCapture2::Camera** &ppCameras)
{

    FlyCapture2::Error error;


    //!<
    //!< Get camera bus
    //!<

    FlyCapture2::BusManager busMgr;

    error = busMgr.GetNumOfCameras( &numCameras );
    PRINT_ERROR( error );
    std::cout << "Number of cameras detected: " << numCameras << std::endl;

    if ( numCameras < 1 )
    {
        std::cerr << "Insufficient number of cameras... press Enter to exit." << std::endl;
        getchar();
        exit(-1);
    }



    ppCameras = new FlyCapture2::Camera* [ numCameras ];


    //!<
    //!< Connect to all detected cameras and attempt to set them to a common video mode and frame rate
    //!<

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


        PrintCameraInfo( ppCameras[i] );

        PrintCameraPropertyInfo( ppCameras[i] );



        //!< Set all cameras to a specific mode and frame rate so they can be synchronized
        error = ppCameras[i]->SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_640x480Y8, //!< size of 640x480, format Mono8bit (Y8)
                                                       FlyCapture2::FRAMERATE_60 );
        PRINT_ERROR_MSG( error,
                        "Error starting cameras. \n"
                        "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
                        "If your camera does not support this mode, please edit the source code and recompile the application. \n"
                        "Press Enter to exit. \n");
    }


    std::cout << "all camera set to specific mode." << std::endl;


    error = FlyCapture2::Camera::StartSyncCapture( numCameras, (const FlyCapture2::Camera**)ppCameras );
    PRINT_ERROR_MSG( error,
                    "Error starting cameras. \n"
                    "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
                    "If your camera does not support this mode, please edit the source code and recompile the application. \n"
                    "Press Enter to exit. \n");

    std::cout << "StartSyncCapture." << std::endl;

}






int main( int /*argc*/, char** /*argv*/ )
{





    //!< OpenCV control panel: block stereo matching parameters
    cv::namedWindow("control panel", cv::WINDOW_NORMAL );
    int ndisparities = 5;
    cv::createTrackbar( "(n+1)x16=ndisparities", "control panel", &ndisparities, 20, NULL ); //!< ndisparities % 16 == 0, positive, ndisparities < width
    int blockSize = 3;
    cv::createTrackbar( "(s+2)x2+1=blockSize", "control panel", &blockSize, 30, NULL ); //!<  5 <= blocksize <= 255, odd





    //!< load camera calibration info
    cv::Mat mx1, my1, mx2, my2;
    loadCalibrationInfo(mx1, my1, mx2, my2);


    //!< connect to cameras
    unsigned int numCameras;
    FlyCapture2::Camera** ppCameras;
    prepareCameras( numCameras, ppCameras );





    //!< Allocate images

    //!< an image par camera: a vector of pointers to images (cv::Mat)
    std::vector< std::shared_ptr< cv::Mat > > pimageCamera;
    for ( unsigned int i = 0; i < numCameras; i++ )
        pimageCamera.push_back( std::shared_ptr< cv::Mat > ( new cv::Mat(480, 640, CV_8UC1) ) ); //!< create cv::Mat*, cast to shared_ptr, then push_back

    //!< disparity maps
    cv::Mat imageDisparity(480, 640, CV_32F), disp8U;




#ifdef USE_OPENCV
    //!< sample code from https://github.com/Itseez/opencv/blob/52a785e95a30d9336bfbac97a0a0d0089ffaa7de/samples/cpp/stereo_match.cpp
    //!< stereo matching object
    cv::Ptr< cv::StereoBM > sbm = cv::createStereoBM( ndisparities, blockSize );
#endif

#ifdef USE_ELAS
    Elas::parameters param;
    param.postprocess_only_left = true;
    param.disp_min = 0;
    param.disp_max = (ndisparities + 1) * 16;
    const int32_t dims[3] = {640, 480, 640};
    Elas elas( param );
    //float* D1_data = (float*)malloc(640*480*sizeof(float));
    //float* D2_data = (float*)malloc(width*height*sizeof(float));
#endif


    FlyCapture2::Image rawImage; //!< buffer

    while ( 1 )
    {
        //!< Display the timestamps for all cameras to show that the image
        //!< capture is synchronized for each image

#ifdef USE_OPENCV
        sbm->setBlockSize( (blockSize + 2) * 2 + 1 );
        sbm->setNumDisparities( (ndisparities + 1) * 16 );
#endif

        for ( unsigned int i = 0; i < numCameras; i++ )
        {
            FlyCapture2::Error error;
            error = ppCameras[i]->RetrieveBuffer( &rawImage );
            PRINT_ERROR( error );

            memcpy( pimageCamera[i]->data, rawImage.GetData(), rawImage.GetDataSize() );
        }

        //!< rectifying images (and un-distorting?)
        cv::remap( *pimageCamera[1], *pimageCamera[1], mx1, my1, cv::INTER_LINEAR );
        cv::remap( *pimageCamera[0], *pimageCamera[0], mx2, my2, cv::INTER_LINEAR );


#ifdef USE_OPENCV
        sbm->compute( *pimageCamera[1], *pimageCamera[0], imageDisparity );
#endif

#ifdef USE_ELAS
//        int static skip = 0;
//        if (skip % 15 ==0)
        {
 //         elas.process(imageCamera1.data, imageCamera0.data, (float*)imageDisparity.data,(float*)imageDisparity.data, dims);
            elas.process(pimageCamera[1]->data, pimageCamera[0]->data, (float*)imageDisparity.data,(float*)imageDisparity.data, dims);

        }
//        skip++;
#endif

        //!< normalize disparity map for imshow
        double minVal, maxVal;
        minMaxLoc( imageDisparity, &minVal, &maxVal );
        imageDisparity.convertTo( disp8U, CV_8UC1, 255/(maxVal - minVal) );


        #define DRAWTXT(img, str, x, y, s) \
            cv::putText( img, str, cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, s, cv::Scalar::all(255) )

        DRAWTXT( disp8U, "disparity", 10, 20, 0.5 );
        cv::imshow( "disparity", disp8U );



        for ( unsigned int i = 0; i < numCameras; i++ )
        {


            // drawCameraInfo( ppCameras[i], *pimageCamera[i] );


            cv::imshow( std::string("cam") + std::to_string(i), *pimageCamera[i] );


        }




        cv::imshow( "sub",  cv::abs( *pimageCamera[0] - *pimageCamera[1] ) );



        char keyValue = cv::waitKey( 10 );
        if (keyValue == 'q' || keyValue == 27 /* ESC */ )
        {
            break;
        }

    }

    exit(0); //!< hack; code below may freeze.

    for ( unsigned int i = 0; i < numCameras; i++ )
    {
        ppCameras[i]->StopCapture();
        ppCameras[i]->Disconnect();
        delete ppCameras[i];
    }

    delete [] ppCameras;




    return 0;
}
