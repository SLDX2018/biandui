#include "HIKvison.h"
using namespace std;
using namespace cv;


int nRet = MV_OK;


bool HIKvison::reset(MV_CC_DEVICE_INFO_LIST llist,int width,int height,float frame,float exposuretime,float gain,float gamma){
    bool a,b,c,d;
    a=visoncamera_found_decive(llist);
    b=visoncamera_open(Handle,list);
    c=visoncamera_set(Handle,width,height,frame,exposuretime,gain,gamma);
    d=visoncamera_start_grab(Handle,stImageinfo);
    if(a&b&c&d)
        return 1;
    else
        return 0;

}
bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        printf("Not support.\n");
        return  false;
    }

    return true;
}


int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
    if (NULL == pRgbData)
    {
        return MV_E_PARAMETER;
    }

    for (unsigned int j = 0; j < nHeight; j++)
    {
        for (unsigned int i = 0; i < nWidth; i++)
        {
            unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
            pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
            pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
        }
    }

    return MV_OK;
}
bool HIKvison::Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData)
{
    cv::Mat srcImage;
    //cout << pstImageInfo->enPixelType << "\n";
    //PixelType_Gvsp_BayerRG8;
    if (pstImageInfo->enPixelType == PixelType_Gvsp_Mono8)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);

    }
    else if (pstImageInfo->enPixelType == PixelType_Gvsp_BayerRG8)
    {
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC1, pData);
        cvtColor(srcImage, Image, COLOR_BayerBG2BGR);

    }
    else if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
        srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);

    }
    else
    {
        printf("unsupported pixel format\n");
        return false;
    }

    if (NULL == srcImage.data)
    {
        return false;
    }


    try {
#if defined (VC9_COMPILE)
        cvSaveImage("MatImage.bmp", &(IplImage(srcImage)));
#else

#endif
    }
    catch (cv::Exception& ex) {
        fprintf(stderr, "Exception saving image to bmp format: %s\n", ex.what());
    }

   //srcImage.release();

    return true;
}

bool HIKvison::visoncamera_found_decive(MV_CC_DEVICE_INFO_LIST stDeviceList){

    // Enum device
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("Enum Devices fail! nRet [0x%x]\n", nRet);
        return false;
    }
    if (stDeviceList.nDeviceNum > 0)
    {
        for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                return false;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
        this->list=stDeviceList;
    }
    else
    {
        printf("Find No Devices!\n");
        return false;
    }
    return true;
}


bool HIKvison::visoncamera_open(void *handle,MV_CC_DEVICE_INFO_LIST &stDeviceList){


    // input the format to convert
    printf("[0] OpenCV_Mat\n");
    printf("[1] OpenCV_IplImage\n");
    printf("Please Input Format to convert:");
    unsigned int nFormat = 0;
    //scanf_s("%d", &nFormat);
    if (nFormat >= 2)
    {
        printf("Input error!\n");
        return 0;
    }

    // select device to connect
    printf("Please Input camera index(0-%d):", stDeviceList.nDeviceNum - 1);
    unsigned int nIndex = 0;
    // scanf_s("%d", &nIndex);
    if (nIndex >= stDeviceList.nDeviceNum)
    {
        printf("Input error!\n");
        return 0;
    }

    // Select and create handle
    nRet = MV_CC_CreateHandle(&Handle, stDeviceList.pDeviceInfo[nIndex]);
    if (MV_OK != nRet)
    {
        printf("Create Handle fail! nRet [0x%x]\n", nRet);
        return 0;
    }

    // open device
    nRet = MV_CC_OpenDevice(Handle);
    if (MV_OK != nRet)
    {
        printf("Open Device fail! nRet [0x%x]\n", nRet);
        return 0;
    }
    // Detection network optimal package size(It only works for the GigE camera)
    if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
    {
        int nPacketSize = MV_CC_GetOptimalPacketSize(Handle);
        if (nPacketSize > 0)
        {
            nRet = MV_CC_SetIntValue(Handle, "GevSCPSPacketSize", nPacketSize);
            if (nRet != MV_OK)
            {
                printf("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
            }
        }
        else
        {
            printf("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
        }
    }
    //Handle=handle;
    //printf("句柄为%x",Handle);
     return true;

}

bool HIKvison::visoncamera_set(void*handle,int width,int height,float frame,float exposuretime,float gain,float gamma)
{
    // Set trigger mode as off
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (MV_OK != nRet)
    {
        printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // Get payload size

    memset(&stParam, 0, sizeof(MVCC_INTVALUE));
    nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
    if (MV_OK != nRet)
    {
        printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        return false;
    }
    MV_CC_SetWidth(handle,width);//1280
    MV_CC_SetHeight(handle,height);//960
    MV_CC_SetFrameRate(handle,frame);//180.0
    MV_CC_SetExposureTime(handle,exposuretime);//6000
    MV_CC_SetGain(handle,gain);
    MV_CC_SetGamma(handle,gamma);
    g_nPayloadSize = stParam.nCurValue;
    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }
    printf("Press a key to exit.\n");
    return  true;

}

bool HIKvison::visoncamera_start_grab(void* handle,MV_FRAME_OUT_INFO_EX stImageInfo){
        // Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            return false;
        }
        //MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
        stImageInfo = { 0 };
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        return true;


}

bool HIKvison::visoncamera_get_image(void* handle,MV_FRAME_OUT_INFO_EX stImageInfo,float gain){
    unsigned char* pData = (unsigned char*)malloc(sizeof(unsigned char) * (g_nPayloadSize));
    if (pData == NULL)
       {
           printf("Allocate memory failed.\n");
           return false;
       }
//    MVCC_FLOATVALUE frame;
//    MV_CC_GetFrameRate(handle,&frame);
//    printf("帧率为%f",frame.fCurValue);
//         MV_CC_GetWidth(handle,&stParam);
//         printf("宽度为%d",stParam.nCurValue);
   // get one frame from camera with timeout=1000ms
   nRet = MV_CC_GetOneFrameTimeout(handle, pData, g_nPayloadSize, &stImageInfo, 50);
   if (nRet == MV_OK)
   {
//       printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
//           stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
   }
   else
   {
       printf("No data[0x%x]\n", nRet);
       free(pData);
       pData = NULL;
       return false;


   }
   // 图像转换
   bool bConvertRet = false;
   bConvertRet = Convert2Mat(&stImageInfo, pData);
//        if (0 == nFormat)
//        {
//            bConvertRet = Convert2Mat(&stImageInfo, pData);
//        }
//        else
//        {
//            bConvertRet = Convert2Ipl(&stImageInfo, pData);
//        }
   // print result
   if (bConvertRet)
   {
       //printf("OpenCV format convert finished.\n");
       free(pData);
       pData = NULL;
   }
   else
   {
       printf("OpenCV format convert failed.\n");
       free(pData);
       pData = NULL;
       return false;
   }
   return  true;
}


bool HIKvison::close(void *handle){


    nRet = MV_CC_StopGrabbing(handle);
    if (MV_OK != nRet)
    {
        printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // Close device
    nRet = MV_CC_CloseDevice(handle);
    if (MV_OK != nRet)
    {
        printf("ClosDevice fail! nRet [0x%x]\n", nRet);
        return false;
    }

    // Destroy handle
    nRet = MV_CC_DestroyHandle(handle);
    if (MV_OK != nRet)
    {
        printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
        return false;
    }

}












