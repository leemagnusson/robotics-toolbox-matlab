// =============================================================================
/*!
 *   This file is part of the ATRACSYS fusiontrack library.
 *   Copyright (C) 2003-2015 by Atracsys LLC. All rights reserved.
 *
 *  THIS FILE CANNOT BE SHARED, MODIFIED OR REDISTRIBUTED WITHOUT THE
 *  WRITTEN PERMISSION OF ATRACSYS.
 *
 *  \file     ftkInterfacePrivate.h
 *  \brief    Private interface functions
 *
 */
// =============================================================================

#ifndef ftkInterfacePrivate_h
#define ftkInterfacePrivate_h

#include "ftkInterface.h"

namespace measurement
{
    class Stereo;
}

#ifndef FTK_OPT_DEV_REG
    #define FTK_OPT_DEV_REG  4000 /*!< Start of device registers */
#endif

#ifndef FTK_OPT_DEV_MEM
    #define FTK_OPT_DEV_MEM  5120 /*!< Start of device internal bus */
#endif

#ifndef REG_READ
    #define REG_READ  1
#endif

#ifndef REG_WRITE
    #define REG_WRITE 2
#endif

enum ftkMemory
{
    MEM_REGISTER = 0,
    MEM_AVALON = 1,
    MEM_UPLOAD = 2
};

/** \brief Structure holding the header built from the cameras information.
 *
 * This header is sent together with the data and is analysed in when a
 * frame is asked: the two headers from the two pictures must have matching
 * timestamps / counter.
 */
PACK1_STRUCT_BEGIN( ftkCameraHeader )
{
    // header byte 0 (byte => 128bits)
    uint32 u4ImageType                      :  4; /**< */
    uint32 u8ImageFormat                    :  8; /**< */
    uint32 u4Binning                        :  4; /**< */
    uint32 u2PixelCoding                    :  2; /**< */
    uint32 u1Compression                    :  1; /**< */
    uint32 u10Threshold                     : 10; /**< */
    uint32 u2Factor                         :  2; /**< */
    uint32 u1Reserved0                      :  1; /**< */
    uint32 u32ImageAuxiliaryInfo; /**< */
    uint64 u64Timestamp; /**< */

    // header byte 1 (byte => 128bits)
    uint32 u32ImageCount; /**< */
    uint32 u32ImagePayload; /**< */
    uint16 u16ImageColumns                  : 16; /**< */
    uint16 u16ImageOffsetY                  : 16; /**< */
    uint16 u16ImageColumnWidth              : 16; /**< */
    uint16 u16ImageSizeY                    : 16; /**< */

    // header byte 2 (byte => 128bits)
    uint32 u14ImageOffset                   : 14; /**< */
    uint32 u2ImageFlipping                  :  2; /**< */
    uint32 u8VRamp1                         :  8; /**< */
    uint32 u8VRamp2                         :  8; /**< */


    uint32 u8ADCgain                        :  8; /**< */
    uint32 u3ImagePGA                       :  3; /**< */
    uint32 u3OtherSpecialModes              :  3; /**< */
    uint32 u2NrSlopes                       :  2; /**< */
    uint32 u8Vlow2                          :  8; /**< */
    uint32 u8Vlow3                          :  8; /**< */

    uint64 u24ExposureDurationTimeus        : 24; /**< */
    uint64 u23StrobeStartTimeus             : 23; /**< */
    uint64 u1Reserved1                      :  1; /**< */
    uint64 u13StrobeDurationTimeus          : 13; /**< */
    uint64 u3Reserved2                      :  3; /**< */

    // header byte 3 (byte => 128bits)
    uint64 u23TriggerStartTimeus            : 23; /**< */
    uint64 u1Reserved3                      :  1; /**< */
    uint64 u24TriggerActualDurationTimeus   : 24; /**< */
    uint64 u1TriggerActivatedDuringExposure :  1; /**< */
    uint64 u15Reserved4                     : 15; /**< */

    uint64 u64Reserved5; /**< */
}
PACK1_STRUCT_END( ftkCameraHeader );

typedef void ( _CDECL_ * ftkRegistersEnumCallback )( void* user,
                                                     char* name,
                                                     uint32 address,
                                                     char* desc,
                                                     uint32 rw /* REG_READ
                                                                * and/or
                                                                * REG_WRITE */                 );

ATR_EXPORT ftkError ftkEnumerateRegisters( ftkLibrary lib,
                                           uint64 sn,
                                           ftkRegistersEnumCallback cb,
                                           void* user );

ATR_EXPORT ftkError ftkMemWrite( ftkLibrary lib,
                                 uint64 sn,
                                 ftkMemory mem,
                                 uint32 atAddress,
                                 uint8* data,
                                 uint32 dataSizeIn );

ATR_EXPORT ftkError ftkMemRead( ftkLibrary lib,
                                uint64 sn,
                                ftkMemory mem,
                                uint32 atAddress,
                                uint8* data,
                                uint32* dataSizeInOut );

/** \brief Getter for the camera header info.
 *
 * This function allows to get the header sent by the FPGA for the two
 * pictures.
 *
 * \param[in] lib initialised library handle.
 * \param[in] frameQueryIn an initialized ftkFrameQuery structure,
 * containing sensible information (i.e. output of ftkGetLastFrame).
 * \param[out] leftHeader instance of ftkCameraHeader, will contain the
 * header for the left image.
 * \param[out] rightHeader instance of ftkCameraHeader, will contain the
 * header for the right image.
 *
 * \retval FTK_OK if everyting went fine,
 * \retval FTK_ERR_INV_PTR if \c lib is uninitialised or if \c frameQueryIn
 * is a null pointer,
 * \retval FTK_WAR_NOT_SUPPORTED if the library version is too old.
 */
ATR_EXPORT ftkError ftkGetCameraHeader( ftkLibrary lib,
                                        ftkFrameQuery* frameQueryIn,
                                        ftkCameraHeader& leftHeader,
                                        ftkCameraHeader& rightHeader );

/** \brief Function reprojecting a 3D point into a pair of stereo raw data.
 *
 * This function allows to get the coordinates of two raw data obtained by
 * reprojecting the 3D point.
 *
 * \param[in] lib initialised library handle.
 * \param[in] sn serial number of the wanted device.
 * \param[in] inPoint pointer on the 3D point to reproject.
 * \param[out] outLeftData pointer on the \e initialised left raw data
 * instance.
 * \param[out] outRightData pointer on the \e initialised right raw data
 * instance.
 *
 * \retval
 */
ATR_EXPORT ftkError ftkReprojectPoint( ftkLibrary lib,
                                       uint64 sn,
                                       ftk3DPoint* inPoint,
                                       ftkRawData* outLeftData,
                                       ftkRawData* outRightData );

/** \brief Getter for the current calibration.
 *
 * This function allows to get the \e current calibration from the device.
 *
 * \param[in] lib initialised library handle.
 * \param[in] sn serial number of the wanted device.
 * \param[out] stereo pointer on an initialised Stereo instance, will
 * contain the parameters.
 *
 * \retval FTK_OK if everything went fine,
 * \retval FTK_ERR_INV_PTR if \c lib is uninitialised or if \c sn is not
 * correct.
 */
ATR_EXPORT ftkError ftkGetCalibration( ftkLibrary lib,
                                       uint64 sn,
                                       measurement::Stereo* stereo );

/** \brief Setter for the current calibration.
 *
 * This function allows to set the \e current calibration from the device.
 *
 * \param[in] lib initialised library handle.
 * \param[in] sn serial number of the wanted device.
 * \param[out] stereo pointer on an initialised Stereo instance, contains
 * new values of the parameters.
 *
 * \retval FTK_OK if everyting went fine,
 * \retval FTK_ERR_INV_PTR if \c lib is uninitialised or if \c sn is not
 * correct.
 */
ATR_EXPORT ftkError ftkSetCalibration( ftkLibrary lib,
                                       uint64 sn,
                                       measurement::Stereo* stereo );

#endif

