/**
******************************************************************************
* @file    mp3process.h
* @author  MCD Application Team
* @brief   This file includes MP3 processing layer headers
******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MP3PROCESS_H_
#define __MP3PROCESS_H_

#ifdef __cplusplus
extern "C" {
#endif

  /* Includes ------------------------------------------------------------------*/
#include <stdlib.h>

#include "ff.h"
#include "audio_conf.h"
#include "songutilities.h"
#include "coders.h"

#ifdef __MP3_ENCODER__
#include "spiritMP3Enc.h"
#endif

#ifdef __MP3_DECODER__
#include "spiritMP3Dec.h"
#endif

  /** @addtogroup STM32_Audio_Utilities
  * @{
  */

  /** @defgroup MP3PROCESS
  * @brief This file is the Header file for mp3process.c
  * @{
  */

  /** @defgroup MP3PROCESS_Exported_Defines
  * @{
  */

  /* Specify "malloc" function to be used by MP3 Encoder to allocate memory region */
#ifndef MP3_ENC_MALLOC
#define MP3_ENC_MALLOC(x)       malloc(x)
#endif /* MP3_ENC_MALLOC */

  /* Specify "free" function to be used by MP3 Encoder to release, previously, allocated memory region */
#ifndef MP3_ENC_FREE
#define MP3_ENC_FREE(x)         free(x)
#endif /* MP3_ENC_FREE */

  /* Specify "malloc" function to be used by MP3 Decoder to allocate memory region */
#ifndef MP3_DEC_MALLOC
#define MP3_DEC_MALLOC(x)       malloc(x)
#endif /* MP3_DEC_MALLOC */

  /* Specify "free" function to be used by MP3 Decoder to release, previously, allocated memory region */
#ifndef MP3_DEC_FREE
#define MP3_DEC_FREE(x)         free(x)
#endif /* MP3_DEC_FREE */

  /* Supported VBR types */
#define VBR_Xing                  1
#define VBR_Info                  2
#define VBR_VBRI                  3

#define MAX_AUDIO_MP3_HEADER_SIZE    200

  /**
  * @}
  */

  /**
  * @}
  */


  /** @defgroup AUDIO_SYNCHPROCESS_Exported_Variables
  * @{
  */
  /**
  * @}
  */

  /** @defgroup AUDIO_SYNCHPROCESS_Exported_Functions
  * @{
  */
  TSpiritMP3Info * MP3Process_GetFileInfoStruct(void);

#ifdef __MP3_DECODER__

  uint32_t Mp3Process_DecoderInit(uint8_t* pHeader,
                                  fnReadCallback_TypeDef* pReadCallback,
                                  fnSetPositionCallback_TypeDef* pSetPosCallback);
  uint32_t Mp3Process_DecoderDeInit(void);
  uint32_t Mp3Process_DecodeData(__IO int16_t* OutPutBuffer,uint32_t NbSamples,void * Usr);
  uint32_t Mp3Process_RetrieveIDTAGS(TAGS_TypeDef * pIDTAG, FIL* file);
  uint32_t Mp3Process_GetStreamLenght(uint32_t fLength);
  uint32_t Mp3Process_GetElapsedTime(uint32_t currpos);
  uint32_t Mp3Process_GetSamplingRate(void);

  uint32_t Mp3Process_GetBitRateKbps(void);
  uint32_t Mp3Process_GetNbChannels(void);
  uint32_t Mp3Process_GetNbBitsReadAfterFrameHeader(void);

  uint32_t Mp3Process_GetNbSamplesFrame(void);

  uint32_t Mp3Process_StreamBackward(void);
  uint32_t Mp3Process_StreamForward(void);
  uint32_t Mp3Process_StopMode(void);
  uint32_t Mp3Process_VBRDetected(void);

#endif /* __MP3_DECODER__ */

#ifdef __MP3_ENCODER__

  uint32_t Mp3Process_EncoderInit            (uint32_t Freq, uint8_t* Param);
  uint32_t Mp3Process_EncoderDeInit          (void);
  uint32_t Mp3Process_EncoderStopMode        (void);
  uint32_t Mp3Process_EncodeData             (int8_t *pBufIn, int8_t *pBufOut, uint32_t* nEnc, void* pOpt);
  uint32_t Mp3Process_SetTags                (TAGS_TypeDef* IdTags, void *file);
  uint32_t Mp3Process_EncGetSampleRate       (void);
  uint32_t Mp3Process_EncGetStreamLength     (void);
  uint32_t Mp3Process_EncGetElapsedTime      (void);

#endif /* __MP3_ENCODER__ */


  /**
  * @}
  */

  /**
  * @}
  */

  /**
  * @}
  */
#endif /* __MP3PROCESS_H_ */

#ifdef __cplusplus
}
#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/