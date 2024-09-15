#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <mutex>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <opencv2/opencv.hpp>

#include <hcnetsdk/HCNetSDK.h>

// Global variables
cv::Mat g_frame;
std::mutex g_mutex;

// DecodeContext struct
typedef struct {
    AVCodecContext* codecCtx;
    AVCodecParserContext* parser;
    AVFrame* frame;
    AVPacket* packet;
    SwsContext* swsCtx;
} DecodeContext;

// Function declarations
void decode_frame(DecodeContext* decodeCtx, AVPacket* pkt);
void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer,DWORD dwBufSize,void* dwUser);
void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser);

int main()
{
    // Initialize the Hikvision SDK
    NET_DVR_Init();
    NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);

    // Login to the device
    LONG lUserID;
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {};
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {};
    struLoginInfo.bUseAsynLogin = false;
    struLoginInfo.wPort = 8000;
    strncpy((char*)struLoginInfo.sDeviceAddress, "192.168.68.54", NET_DVR_DEV_ADDRESS_MAX_LEN);
    strncpy((char*)struLoginInfo.sUserName, "admin", NAME_LEN);
    strncpy((char*)struLoginInfo.sPassword, "F4b10b$01", NAME_LEN);

    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
    if (lUserID < 0)
    {
        printf("Login error, %d\n", NET_DVR_GetLastError());
        NET_DVR_Cleanup();
        return -1;
    }

    // Prepare the decoding context
    DecodeContext decodeCtx = {};

    // Removed avcodec_register_all(); as it's deprecated and not needed

    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        printf("Codec not found\n");
        return -1;
    }

    decodeCtx.codecCtx = avcodec_alloc_context3(codec);
    if (!decodeCtx.codecCtx) {
        printf("Could not allocate video codec context\n");
        return -1;
    }

    if (avcodec_open2(decodeCtx.codecCtx, codec, NULL) < 0) {
        printf("Could not open codec\n");
        return -1;
    }

    decodeCtx.parser = av_parser_init(AV_CODEC_ID_H264);
    if (!decodeCtx.parser) {
        printf("Parser not found\n");
        return -1;
    }

    decodeCtx.frame = av_frame_alloc();
    if (!decodeCtx.frame) {
        printf("Could not allocate video frame\n");
        return -1;
    }

    decodeCtx.packet = av_packet_alloc();
    if (!decodeCtx.packet) {
        printf("Could not allocate AVPacket\n");
        return -1;
    }

    // Start real-time preview and set callback
    NET_DVR_PREVIEWINFO struPlayInfo = {};
    struPlayInfo.lChannel = 1;
    struPlayInfo.dwStreamType = 0;
    struPlayInfo.dwLinkMode = 0;
    struPlayInfo.bBlocked = 1;

    LONG lRealPlayHandle;
    lRealPlayHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, g_RealDataCallBack_V30, &decodeCtx);
    if (lRealPlayHandle < 0)
    {
        printf("NET_DVR_RealPlay_V40 error\n");
        NET_DVR_Logout(lUserID);
        NET_DVR_Cleanup();
        return -1;
    }

    cv::namedWindow("Video", cv::WINDOW_AUTOSIZE);

    while (true)
    {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            if (!g_frame.empty())
            {
                g_frame.copyTo(frame);
            }
        }

        if (!frame.empty())
        {
            cv::imshow("Video", frame);
            if (cv::waitKey(1) == 27) // Press 'Esc' to exit
            {
                break;
            }
        }
        else
        {
            // No frame available, wait a bit
            cv::waitKey(10);
        }
    }

    // Clean up
    NET_DVR_StopRealPlay(lRealPlayHandle);
    NET_DVR_Logout(lUserID);
    NET_DVR_Cleanup();

    av_parser_close(decodeCtx.parser);
    avcodec_free_context(&decodeCtx.codecCtx);
    av_frame_free(&decodeCtx.frame);
    av_packet_free(&decodeCtx.packet);
    if (decodeCtx.swsCtx)
        sws_freeContext(decodeCtx.swsCtx);

    return 0;
}

void decode_frame(DecodeContext* decodeCtx, AVPacket* pkt)
{
    int ret = avcodec_send_packet(decodeCtx->codecCtx, pkt);
    if (ret < 0)
    {
        printf("Error sending packet for decoding\n");
        return;
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_frame(decodeCtx->codecCtx, decodeCtx->frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        {
            return;
        }
        else if (ret < 0)
        {
            printf("Error during decoding\n");
            return;
        }

        // Convert the frame to BGR for OpenCV
        AVFrame* pFrameBGR = av_frame_alloc();
        int numBytes = av_image_get_buffer_size(AV_PIX_FMT_BGR24, decodeCtx->codecCtx->width, decodeCtx->codecCtx->height, 1);
        uint8_t* buffer = (uint8_t*)av_malloc(numBytes * sizeof(uint8_t));

        av_image_fill_arrays(pFrameBGR->data, pFrameBGR->linesize, buffer, AV_PIX_FMT_BGR24, decodeCtx->codecCtx->width, decodeCtx->codecCtx->height, 1);

        // Set up sws context
        if (!decodeCtx->swsCtx)
        {
            decodeCtx->swsCtx = sws_getContext(decodeCtx->codecCtx->width, decodeCtx->codecCtx->height, decodeCtx->codecCtx->pix_fmt,
                                               decodeCtx->codecCtx->width, decodeCtx->codecCtx->height, AV_PIX_FMT_BGR24,
                                               SWS_BICUBIC, NULL, NULL, NULL);
        }

        sws_scale(decodeCtx->swsCtx, (uint8_t const* const*)decodeCtx->frame->data, decodeCtx->frame->linesize, 0, decodeCtx->codecCtx->height,
                  pFrameBGR->data, pFrameBGR->linesize);

        // Create OpenCV Mat
        cv::Mat img(decodeCtx->codecCtx->height, decodeCtx->codecCtx->width, CV_8UC3, pFrameBGR->data[0], pFrameBGR->linesize[0]);

        // Copy the image to global Mat under mutex
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            img.copyTo(g_frame);
        }

        // Free the frame
        av_free(buffer);
        av_frame_free(&pFrameBGR);
    }
}

void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer,DWORD dwBufSize,void* dwUser)
{
    DecodeContext* decodeCtx = (DecodeContext*)dwUser;
    if (!decodeCtx)
        return;

    if (dwDataType == NET_DVR_STREAMDATA || dwDataType == NET_DVR_STD_VIDEODATA)
    {
        BYTE* pData = pBuffer;
        int nSize = dwBufSize;

        while (nSize > 0)
        {
            int ret = av_parser_parse2(decodeCtx->parser, decodeCtx->codecCtx, &decodeCtx->packet->data, &decodeCtx->packet->size, pData, nSize, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);

            if (ret < 0)
            {
                printf("Error while parsing\n");
                break;
            }

            pData += ret;
            nSize -= ret;

            if (decodeCtx->packet->size > 0)
            {
                // Decode the packet
                decode_frame(decodeCtx, decodeCtx->packet);
            }
        }
    }
}

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
    switch(dwType)
    {
        case EXCEPTION_RECONNECT:
            printf("----------reconnect--------%ld\n", time(NULL));
            break;
        default:
            break;
    }
}
