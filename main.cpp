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

using namespace FlyCapture2;

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}

int main(int /*argc*/, char** /*argv*/)
{
    const int k_numImages = 100000;
    Error error;

    /*char* imageName = "0.jpg";
    cv::Mat image;
    cv::namedWindow( imageName, CV_WINDOW_AUTOSIZE );
    image = cv::imread(imageName,CV_LOAD_IMAGE_UNCHANGED);
    cv::imshow( imageName, image );
    cv::waitKey(0);*/

    //output a list for calibration
    /*FILE* listFile = fopen("list.txt", "w");
    if (listFile == NULL)
    {
    	printf("Failed to create file in current folder.  Please check permissions.\n");
    	return -1;
    }*/
    //fprintf(listFile,  "test string \n");
    //fclose(listFile);

    //
//	cv::FileStorage fs ("../calibrationResult/Q.xml",   cv::FileStorage::READ);
    cv::FileStorage fx1("../calibrationResult/mx1.xml", cv::FileStorage::READ);
    cv::FileStorage fx2("../calibrationResult/mx2.xml", cv::FileStorage::READ);
    cv::FileStorage fy1("../calibrationResult/my1.xml", cv::FileStorage::READ);
    cv::FileStorage fy2("../calibrationResult/my2.xml", cv::FileStorage::READ);
    cv::Mat Q, M1, D1, R1, P1, M2, D2, R2, P2;
    cv::Mat mx1, my1, mx2, my2;

//	fs["Q"] >> Q;
    fx1["mx1"] >> mx1;
    fx2["mx2"] >> mx2;
    fy1["my1"] >> my1;
    fy2["my2"] >> my2;

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    printf( "Number of cameras detected: %u\n", numCameras );

    if ( numCameras < 1 )
    {
        printf( "Insufficient number of cameras... press Enter to exit.\n" );
        getchar();
        return -1;
    }

    Camera** ppCameras = new Camera*[numCameras];

    // Connect to all detected cameras and attempt to set them to
    // a common video mode and frame rate
    for ( unsigned int i = 0; i < numCameras; i++)
    {
        ppCameras[i] = new Camera();

        printf( "setting camera %d.\n", i );

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex( i, &guid );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        // Connect to a camera
        error = ppCameras[i]->Connect( &guid );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = ppCameras[i]->GetCameraInfo( &camInfo );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        PrintCameraInfo(&camInfo);

        // Set all cameras to a specific mode and frame rate so they
        // can be synchronized
        error = ppCameras[i]->SetVideoModeAndFrameRate(
                    VIDEOMODE_640x480Y8,
                    FRAMERATE_60 );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            printf(
                "Error starting cameras. \n"
                "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
                "If your camera does not support this mode, please edit the source code and recompile the application. \n"
                "Press Enter to exit. \n");
            getchar();
            return -1;
        }
    }
    printf( "all camera setted to specific mode.\n" );

    error = Camera::StartSyncCapture( numCameras, (const Camera**)ppCameras );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        printf(
            "Error starting cameras. \n"
            "This example requires cameras to be able to set to 640x480 Y8 at 30fps. \n"
            "If your camera does not support this mode, please edit the source code and recompile the application. \n"
            "Press Enter to exit. \n");
        //getchar();
        //return -1;
    }
    printf( "StartSyncCapture.\n" );

//cv::Mat imageCamera(rawImage.GetRows(), rawImage.GetCols(), CV_8UC1);
    cv::Mat imageCamera0(480, 640, CV_8UC1);
    cv::Mat imageCamera1(480, 640, CV_8UC1);
    cv::Mat imageDisparity(480, 640, CV_32F), disp8U;
    cv::Mat* pimageCamera[]= {&imageCamera0, &imageCamera1};
    cv::namedWindow( "cam0", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "cam1", CV_WINDOW_AUTOSIZE );
    cv::namedWindow( "disparity", CV_WINDOW_AUTOSIZE );
//cv::imshow( "test", imageCamera );
//cv::waitKey(0);
#ifdef USE_OPENCV
//  cv::StereoBM SBM(cv::StereoBM::BASIC_PRESET, 128, 11);
    int ndisparities = 256;
    cv::StereoBM SBM(cv::StereoBM::NARROW_PRESET, ndisparities, 11);
//	CvStereoBMState *BMState = SBM.state;
//	assert(BMState != 0);
//	BMState->SADWindowSize = 11;
//	BMState->minDisparity  = 0; //-64;
//	BMState->numberOfDisparities = 128;
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

    //for ( int j = 0; j < k_numImages; j++ )
    for ( int j = 0; j < k_numImages; j++ )
    {
        // Display the timestamps for all cameras to show that the image
        // capture is synchronized for each image


        /*for ( unsigned int i = 0; i < numCameras; i++ )
        {
            Image rawImage;
            error = ppCameras[i]->RetrieveBuffer( &rawImage );
            if (error != PGRERROR_OK)
            {
        	PrintError( error );
        	return -1;
            }

            TimeStamp timestamp = rawImage.GetTimeStamp();
            printf(
        	"Cam %d - Frame %d - TimeStamp [%d %d]\n",
        	i,
        	j,
        	timestamp.cycleSeconds,
        	timestamp.cycleCount);*

        	error = rawImage.Save( "TESTSYNC.pgm" );
        	if (error != PGRERROR_OK)
        	{
        	    PrintError( error );
        	    return -1;
        	}


            //std::cout<< rawImage.GetRows() << "  "<<rawImage.GetCols()<<std::endl;
        	// retrieve a frame image
        	FlyCapture2::Image      rawImage;
        	fc2Error = fc2Cam.RetrieveBuffer( &rawImage );
        	if( fc2Error != FlyCapture2::PGRERROR_OK ) {
        		PrintError( fc2Error );
        		return -1;
        	}

        	// convert the raw image
        	Image      cvtImage;
        	error = rawImage.Convert( PIXEL_FORMAT_MONO8, &cvtImage );
        	if( error != PGRERROR_OK ) {
        		PrintError( error );
        		return -1;

        	//memcpy( imageCamera.data, cvtImage.GetData(), cvtImage.GetDataSize() );

        	//imageCamera = cv::imread("test",CV_LOAD_IMAGE_UNCHANGED);


        	}
        }*/
        for ( unsigned int i = 0; i < numCameras; i++ )
        {
            Image rawImage;
            error = ppCameras[i]->RetrieveBuffer( &rawImage );
            if (error != PGRERROR_OK)
            {
                PrintError( error );
                return -1;
            }

            //char filename[512];
            //sprintf( filename, "SYNCTEST-%u-%d.pgm", i, j );

            /*error = rawImage.Save( filename );
            if (error != PGRERROR_OK)
            {
            	PrintError( error );
            	return -1;
            }*/

            memcpy( pimageCamera[i]->data, rawImage.GetData(), rawImage.GetDataSize() );
            //cv::remap( *pimageCamera[i], *pimageCamera[i], mx1, my1, CV_INTER_LINEAR );
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
        imageDisparity.convertTo( disp8U, CV_8UC1, 255/(maxVal - minVal));

        imshow("disparity", disp8U);
        cv::imshow( "cam0", *pimageCamera[1] );
        cv::imshow( "cam1", *pimageCamera[0] );
        cv::imshow( "sub",  cv::abs( *pimageCamera[0] - *pimageCamera[1] ) );


        char keyValue = cv::waitKey(10);
        /*if (keyValue == 's')
        {
        	char filename[512];
        	sprintf( filename, "grasshopperEx/SYNCIMAGE-%u-%d.pgm", j, 1 );
        	imwrite( filename, *pimageCamera[0] );

        	sprintf( filename, "grasshopperEx/SYNCIMAGE-%u-%d.pgm", j, 0 );
        	imwrite( filename, *pimageCamera[1] );


        	//add new line to list
        	fprintf(listFile,  "grasshopperEx/SYNCIMAGE-%u-%d.pgm\n", j ,0);
        	fprintf(listFile,  "grasshopperEx/SYNCIMAGE-%u-%d.pgm\n", j ,1);
        }*/

        if  (keyValue == 'q' || keyValue == 27 /* ESC */ )
        {
//            j = k_numImages;
            break;
//            cv::destroyAllWindows();
//            cv::waitKey(30);
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

//	printf( "Done! Press Enter to exit...\n" );
//	getchar();


    return 0;
}
