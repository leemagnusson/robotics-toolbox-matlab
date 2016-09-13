// ===========================================================================
/*!
 *   This file is part of the ATRACSYS fusiontrack library.
 *   Copyright (C) 2003-2015 by Atracsys LLC. All rights reserved.
 *
 *  THIS FILE CANNOT BE SHARED, MODIFIED OR REDISTRIBUTED WITHOUT THE
 *  WRITTEN PERMISSION OF ATRACSYS.
 *
 *  \file     ftkErrors.h
 *  \brief    Device error codes
 *
 */
// ===========================================================================

#ifndef ftkErrors_h
#define ftkErrors_h

#include "ftkTypes.h"

/** \brief Error definition super-macro
 *
 * This macro defines the identifier name, the associated value and the
 * displayed text, i.e. the description.
 *
 * \param N triplet ( identifier name, value, description ).
 */
#define ERRORS( N )                                                                   \
                                                                                      \
    /* The following errors are the same as device or directly relayed */             \
    /* So never change the ID of these definitions */                                 \
                                                                                      \
    N( FTK_WAR_NOT_EXISTING,   -21, "The callback was not present" )                  \
    N( FTK_WAR_ALREADY_PRESENT, -20, "The callback was already present" )             \
    N( FTK_WAR_NOT_SUPPORTED,  -2, "Not supported" )                                  \
    N( FTK_WAR_NO_FRAME,       -1, "No new frame available" )                         \
    N( FTK_OK,                  0, "No error" )                                       \
    N( FTK_ERR_INV_PTR,         1, "Invalid pointer" )                                \
    N( FTK_ERR_INV_SN,          2, "Invalid serial number" )                          \
    N( FTK_ERR_INV_INDEX,       3, "Invalid index" )                                  \
    N( FTK_ERR_INTERNAL,        4,                                                    \
       "Internal error, usually this should not happen and be reported to Atracsys" ) \
    N( FTK_ERR_WRITE,           5, "Error when writing" )                             \
    N( FTK_ERR_READ,            6, "Error when reading" )                             \
    N( FTK_ERR_IMG_DEC,         7, "Image decompression error" )                      \
    N( FTK_ERR_IMG_FMT,         8, "Image format error" )                             \
    N( FTK_ERR_VERSION,         9, "Invalid version" )                                \
    N( FTK_ERR_INIT,           10,                                                    \
       "Frame query instance not properly initialised" )                              \
                                                                                      \
    N( FTK_ERR_INV_OPT,       100, "Invalid option" )                                 \
    N( FTK_ERR_INV_OPT_ACC,   101, "Invalid option access" )                          \
    N( FTK_ERR_INV_OPT_VAL,   102, "Invalid option value" )                           \
    N( FTK_ERR_INV_OPT_PAR,   103, "Invalid option parameter" )                       \
                                                                                      \
    /* The following errors are specifically for fusionTrack module */                \
                                                                                      \
    N( FTK_WAR_REJECTED_PIC,  -18,                                                    \
       "The pictures were discarded due to their size" )                              \
    N( FTK_WAR_SHOCK_DETECTED, -17,                                                   \
       "A shock as been detected and calibration may be obsolete" )                   \
    N( FTK_WAR_SN_ABSENT,     -16,                                                    \
       "The device serial number is not in the calibration file" )                    \
    N( FTK_WAR_TEMP_HIGH,     -15, "Device is too hot for compensation" )             \
    N( FTK_WAR_TEMP_LOW,      -14, "Device is too cold for compensation" )            \
    N( FTK_WAR_FILE_NOT_FOUND, -12, "File is not found" )                             \
    N( FTK_WAR_GEOM_ID,       -11, "Geometry ID is not registered" )                  \
    N( FTK_WAR_FRAME,         -10,                                                    \
       "Error when updating the frame in ftkGetLastFrame" )                           \
    N( FTK_ERR_IM_TOO_SMALL,   11, "Image size too small" )                           \
    N( FTK_ERR_INV_INI_FILE,   12,                                                    \
       "Internal or external INI file syntax error" )                                 \
    N( FTK_ERR_SEG_OVERFLOW,   13, "Overflow during image segmentation" )             \
    N( FTK_ERR_GEOM_PTS,       20, "Geometry has not enough points" )                 \
    N( FTK_ERR_SYNC,           30, "Synchronisation error" )                          \
    N( FTK_ERR_COMP_ALGO,      40,                                                    \
       "Temperature compensation algorithm not implemented" )                         \


/** \brief Macro for error codes enumeration.
 *
 * This macro generates the identifiers values for the enum, and adds the
 * description as a comment.
 *
 * \param name name of the identifier.
 * \param val value corresponding to the identifier.
 * \param desc description, used as the documentation.
 */
#define ERROR_ENUM( name, val, desc ) name = val, /**< desc */

/** \brief Error codes.
 *
 * Negative values are used for warnings (i.e. non-fatal issues),
 * whilst positive values are used for errors. An error should stop the
 * normal execution, as the data may be corrupted, missing, old, etc.
 */
TYPED_ENUM( int32, ftkError )
{
    ERRORS( ERROR_ENUM )
};

#undef ERROR_ENUM

#endif
