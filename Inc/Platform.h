/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     AWE_Wrapper.h
********************************************************************************
*
*     Description:  AudioWeaver Wrapper Header File
*
*     Copyright:    (c) 2018 DSP Concepts Inc., All rights reserved
*                   3235 Kifer Road
*                   Santa Clara, CA 95054
*
*******************************************************************************/
 #ifndef __AWE_WRAPPER_H__
 #define __AWE_WRAPPER_H__

#include "aweCore.h"
#include "TargetInfo.h"
#include "Errors.h"
#include <cmsis_compiler.h>

#include "dspc_asrc_48000_to_48000_32m_080db.h"

/** Create handle. */
#define CREATE_OBJECT_HANDLE(isArray, isFloat, objectID, index) \
	( ((isArray & 1) << 31) | ((isFloat & 1) << 30) | ((objectID & 0x3FFFF) << 12) | (index & 0xFF) )

#define INPUT_CHANNEL_COUNT 8
#define OUTPUT_CHANNEL_COUNT 4

#define AWE_FRAME_SIZE 32

#define TOTAL_BUFFERS 2
#define CHANNEL_BLOCK_SIZE_IN_SAMPLES 32
#define STEREO_BLOCK_SIZE_IN_SAMPLES 64
#define MIC_BLOCK_SIZE_IN_SAMPLES 128
#define MIC_SINGLE_CHANNEL_BLOCK_SIZE 32
#define SCRATCH_BUFFER_SIZE 256
#define PCM_SIZE_IN_BYTES 2

#define AUDIO_EP_BUFFER_SIZE 192
#define USB_AUDIO_BUFFER_SIZE_IN_SAMPLES 192
#define NEW_USB_SAMPLES 96

#define MIC_DOUBLE_BUFFER_SAMPLES 64
#define NEW_MIC_SAMPLES 32
#define NEW_USB_RECORD_SAMPLES 32

#define INPUT_BLOCKSIZE 48
#define OUTPUT_BLOCKSIZE 32

#define USB_RECORD_INPUT_BLOCKSIZE 32
#define USB_RECORD_OUTPUT_BLOCKSIZE 48
#define USB_PLAYBACK_INPUT_BLOCKSIZE 48
#define USB_PLAYBACK_OUTPUT_BLOCKSIZE 32

#define STEREO_CHANNEL_COUNT 2
#define MONO_CHANNEL_COUNT 1
#define USB_PLAYBACK_CHANNEL_COUNT 2
#define USB_RECORD_CHANNEL_COUNT 2

#define INPUT_AUDIO_BUFFER_SIZE (TOTAL_BUFFERS * STEREO_BLOCK_SIZE_IN_SAMPLES)
#define OUTPUT_AUDIO_BUFFER_SIZE (TOTAL_BUFFERS * STEREO_BLOCK_SIZE_IN_SAMPLES)

#define NUMBER_MICS 4
#define SCRATCH_BUFF_SIZE (TOTAL_BUFFERS * CHANNEL_BLOCK_SIZE_IN_SAMPLES * NUMBER_MICS)

#define MIC_BUFF_SIZE (TOTAL_BUFFERS * CHANNEL_BLOCK_SIZE_IN_SAMPLES * NUMBER_MICS)
#define NEW_MIC_BUFFER_SAMPLES 128

#define HID_EP_BUFFER_SIZE 64
#define HID_REPORT_PACKET_SIZE 56
#define HID_REPORT_DATA_SIZE 52
#define DATA_SIZE_IN_DWORDS 13
#define AUDIO_OUT_PACKET_SIZE_BYTES 192

#define MAX_PINS 2
#define GPIO_DIR_OUTPUT 1
#define MAX_AWE_BUFFERSIZE 264
#define ASRC_PLAYBACK_BUFFER_SIZE_IN_SAMPLES DSPC_ASRC_BUFFER_LEN(ASRC_FS_IN_48000, ASRC_FS_OUT_48000, ASRC_PHASELEN_28, INPUT_BLOCKSIZE, OUTPUT_BLOCKSIZE, USB_PLAYBACK_CHANNEL_COUNT)
#define ASRC_RECORD_BUFFER_SIZE_IN_SAMPLES DSPC_ASRC_BUFFER_LEN(ASRC_FS_IN_48000, ASRC_FS_OUT_48000, ASRC_PHASELEN_28, USB_RECORD_INPUT_BLOCKSIZE, USB_RECORD_OUTPUT_BLOCKSIZE, USB_RECORD_CHANNEL_COUNT)

#define USB_PLAYBACK_STATE_LEN DSPC_ASRC_STATE_LEN(USB_PLAYBACK_INPUT_BLOCKSIZE, USB_PLAYBACK_OUTPUT_BLOCKSIZE, ASRC_FS_IN_48000, ASRC_PHASELEN_28)
#define USB_RECORD_STATE_LEN DSPC_ASRC_STATE_LEN(USB_RECORD_INPUT_BLOCKSIZE, USB_RECORD_OUTPUT_BLOCKSIZE, ASRC_FS_IN_48000, ASRC_PHASELEN_28)

#define MAX_PUMP_COUNT 100

#define FAILURE 0
#define SUCCESS 1

#define STRIDE1 1
#define STRIDE2 2
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
#define CHANNEL5 4
#define CHANNEL6 5
#define CHANNEL7 6
#define CHANNEL8 7
#define CHANNEL9 8
#define CHANNEL10 9
#define CHANNEL11 10
#define CHANNEL12 11
#define CHANNEL13 12
#define CHANNEL14 13
#define CHANNEL15 14
#define CHANNEL16 15

#define ProcessWriteASRC_IRQHandler CAN1_TX_IRQHandler
#define ProcessWriteASRC_IRQ CAN1_TX_IRQn

#define AudioWeaverPump_IRQHandler1 CAN1_RX0_IRQHandler
#define AudioWeaverPump_IRQ1 CAN1_RX0_IRQn

#define ProcessUSBMsg_IRQHandler CAN1_RX1_IRQHandler
#define ProcessUSBMsg_IRQ CAN1_RX1_IRQn

#define AudioWeaverPump_IRQHandler2 CAN1_SCE_IRQHandler
#define AudioWeaverPump_IRQ2 CAN1_SCE_IRQn

#define MicProcessWrite_IRQHandler CAN2_TX_IRQHandler
#define MicProcessWrite_IRQ CAN2_TX_IRQn

#define OBJECT_FOUND 0

extern INT32 g_nVolume;

extern volatile BOOL g_bReadyToSend;
extern volatile BOOL g_bPacketReceived;
extern volatile BOOL g_bDeferredProcessingRequired;
extern volatile BOOL g_bPlaying;
extern volatile BOOL g_bRecording;

extern UINT32 g_packet_buffer[MAX_COMMAND_BUFFER_LEN];
extern UINT8 HIDInBuff[HID_EP_BUFFER_SIZE];
extern UINT8 HIDOutBuff[HID_EP_BUFFER_SIZE];

extern INT16 USBBufferIn[USB_AUDIO_BUFFER_SIZE_IN_SAMPLES];
extern INT16 USBBufferOut[USB_AUDIO_BUFFER_SIZE_IN_SAMPLES];

extern USBD_HandleTypeDef USBD_Device;

extern INT32 Scratch1[MIC_DOUBLE_BUFFER_SAMPLES];
extern INT32 Scratch2[MIC_DOUBLE_BUFFER_SAMPLES];
extern INT32 Scratch3[MIC_DOUBLE_BUFFER_SAMPLES];
extern INT32 Scratch4[MIC_DOUBLE_BUFFER_SAMPLES];

extern INT16 MicBufferIn[MIC_BUFF_SIZE];

extern volatile BOOL g_bBlinkLED2ForBoardAlive;
extern volatile BOOL g_bUSBPacketReceived;

extern AWEInstance g_AWEInstance;
extern volatile UINT32 g_nPumpCount;

extern DSPC_ASRC USB_Record_ASRC;
extern fract16 ZeroedSamples[OUTPUT_BLOCKSIZE * USB_PLAYBACK_CHANNEL_COUNT];

void SetupAudio(void);

void targetInit(void);

void CoreInit(void);
void BoardInit(void);
void AudioInit(void);
void USBMsgInit(void);

void awe_pltGPIOSetPinDir(UINT32 nPinNo, UINT32 nPinDir);
void awe_pltGPIOSetPin(UINT32 nPinNo, UINT32 nValue);
void awe_pltGPIOGetPin(UINT32 nPinNo, UINT32 * nValue);

void USBSendReply(AWEInstance * pAWE);

void AWEIdleLoop(void);

void BSP_AUDIO_IN_DMA_Handler(void);


#endif
