// ===========================================================================
/*!
 *   This file is part of the ATRACSYS fusionTrack library.
 *   Copyright (C) 2003-2015 by Atracsys LLC. All rights reserved.
 *
 *  THIS FILE CANNOT BE SHARED, MODIFIED OR REDISTRIBUTED WITHOUT THE
 *  WRITTEN PERMISSION OF ATRACSYS.
 *
 *  \file     ftkInterface.h
 *  \brief    Main interface of Atracsys Passive Tracking SDK
 *
 */
// ===========================================================================

#ifndef ftkInterface_h
#define ftkInterface_h

#include "ftkTypes.h"
#include "ftkPlatform.h"
#include "ftkErrors.h"
#include "ftkEvent.h"
#include "ftkOptions.h"

#ifdef __cplusplus
    #include <cstring>
#endif

#ifndef BUFFER_MAX_SIZE
/** \brief Maximal size of ftkBuffer.
 *
 * The value is 10kB.
 */
    #define BUFFER_MAX_SIZE 10u * 1024u
#endif // BUFFER_MAX_SIZE

struct ftkLibraryImp;

/// Library abstract handle
typedef ftkLibraryImp* ftkLibrary;

/// Available pixel formats
TYPED_ENUM( uint8, ftkPixelFormat )
{
    GRAY8 = 0, /*!< Pixels are represented as grayscale values ranging from 0
                * (black) to 255 (white) */
    // GRAY8_COMPRESSED = 0x80 /*< Internal use only */
};

#ifdef ATR_MSVC
// Disable warning because Visual C++ bug when using signed enums
    #pragma warning( push )
    #pragma warning(disable: 4341)
#endif
/*! \brief Frame members status
 *   \see ftkFrameQuery
 */
TYPED_ENUM( int8, ftkQueryStatus )
{
    QS_WAR_SKIPPED = -1, /*!< This field is not initialized */
    QS_OK = 0, /*!< This field is initialized correctly */
    QS_ERR_OVERFLOW = 1, /*!< This field is initialized correctly, data are
                          * missing because buffer size is too small */
    QS_ERR_INVALID_RESERVED_SIZE = 2, /*!< The reserved size is not a multiple
                                       * of the type's size */
    QS_REPROCESS = 10, /*!< This field is requested to be reprocessed */
};
#ifdef ATR_MSVC
// Disable warning because Visual C++ bug when using signed enums
    #pragma warning( pop )
#endif

/*! \brief Type of device connected
 *   \see ftkDeviceEnumCallback
 */
TYPED_ENUM( int8, ftkDeviceType )
{
    DEV_SIMULATOR = 0, /*!< Internal use only */
    DEV_INFINITRACK = 1, /*!< Device is an infiniTrack */
    DEV_FUSIONTRACK_500 = 2, /*!< Device is a fusionTrack 500 */
    DEV_FUSIONTRACK_250 = 3, /*!< Device is a fusionTrack 250 */
};

/// \brief Structure carrying binary data in/out of library functions.
///
/// This structure is used to get / set binary data. It helps reducing memory
/// leaks by taking care of the destruction of the memory.
PACK1_STRUCT_BEGIN( ftkBuffer )
{
    /*! \brief Data buffer.
     *
     *   This is a simple array of bytes.
     */
    uint8 data[ BUFFER_MAX_SIZE ];
    /*! \brief Actual size of the data.
     *
     *   This member stores the real size of the data stored in the
     *   buffer.
     */
    uint32 size;
    /** \brief Method resetting the structure.
     *
     * This method erases the data buffer and sets the size to 0.
     */
#ifdef __cplusplus
    void reset();
#endif
}
PACK1_STRUCT_END( ftkBuffer );

#ifdef __cplusplus
inline void ftkBuffer::reset()
{
    memset( data, 0, BUFFER_MAX_SIZE ); size = 0u;
}
#endif

/// \brief Structure holding the status of a blob.
///
/// This structure stores the status of the blob.
PACK1_STRUCT_BEGIN( ftkStatus )
{
#ifdef __cplusplus
    private:
        uint8 Reserved   : 4; /*!< Reserved space, not used. */
    public:
#endif
    uint8 RightEdge  : 1; /*!< Contains 1 if the blob touches the right
                           *    edge of the picture */
    uint8 BottomEdge : 1; /*!< Contains 1 if the blob touches the
                           *    bottom edge of the picture. */
    uint8 LeftEdge   : 1; /*!< Contains 1 if the blob touches the left
                           *    edge of the picture. */
    uint8 TopEdge    : 1; /*!< Contains 1 if the blob touches the top
                           *    edge of the picture. */
#ifdef __cplusplus
    /*! \brief Conversion operator.
     *
     * This operator converts the structure into a unsigned integer.
     *
     * \return the value of the bit fields, seen as a 8-bits unsigned
     * integer.
     */
    operator uint8() const;
    /*! \brief Affectation operator.
     *
     * This operator allows to promote a 8-bits unsigned integer into
     * a working instance.
     *
     * \param[in] val value to "dump" in the instance.
     *
     * \return a reference on the current instance.
     */
    ftkStatus& operator=( const uint8& val );
#endif
}
PACK1_STRUCT_END( ftkStatus );

#ifdef __cplusplus
inline ftkStatus::operator uint8() const
{
    union
    {
        uint8 nbr; ftkStatus status;
    };
    status = *this;
    return nbr;
}

inline ftkStatus& ftkStatus::operator=( const uint8& val )
{
    union
    {
        uint8 nbr; ftkStatus status;
    };
    nbr = val;
    *this = status;
    return *this;
}
#endif

/// \brief Structure to hold version and size
///
/// This structure stores a version number and the reserved size for
/// an array in ftkFrameQuery.
PACK1_STRUCT_BEGIN( ftkVersionSize )
{
    uint32 Version : 8; /*!< Version number. */
    uint32 ReservedSize : 24; /*!< Size of the array in bytes. */
#ifdef __cplusplus
    /*! \brief Conversion operator.
     *
     * This operator converts the structure into a unsigned integer.
     *
     * \return the value of the bit fields, seen as a 32-bits unsigned
     * integer.
     */
    operator uint32() const;
    /*! \brief Affectation operator.
     *
     * This operator allows to promote a 32-bits unsigned integer into
     * a working instance.
     *
     * \param[in] val value to "dump" in the instance.
     *
     * \return a reference on the current instance.
     */
    ftkVersionSize& operator=( const uint32& val );
#endif
}
PACK1_STRUCT_END( ftkVersionSize );

#ifdef __cplusplus
/*! \brief Conversion operator.
 *
 * This operator converts the structure into a unsigned integer.
 *
 * \return the value of the bit fields, seen as a 32-bits unsigned
 * integer.
 */
inline ftkVersionSize::operator uint32() const
{
    union
    {
        uint32 nbr; ftkVersionSize ver;
    };
    ver = *this;
    return nbr;
}
/*! \brief Affectation operator.
 *
 * This operator allows to promote a 32-bits unsigned integer into
 * a working instance.
 *
 * \param[in] val value to "dump" in the instance.
 *
 * \return a reference on the current instance.
 */
inline ftkVersionSize& ftkVersionSize::operator=( const uint32& val )
{
    union
    {
        uint32 nbr; ftkVersionSize ver;
    };
    nbr = val;
    *this = ver;
    return *this;
}
#endif

/*! \brief Image header
 *
 *   The image header contains basic information on the picture
 *   dimensions and timing information.
 */
PACK1_STRUCT_BEGIN( ftkImageHeader )
{
    uint64 timestampUS; /*!< Timestamp of the image in micro-seconds */
    uint64 desynchroUS; /*!< Desynchronisation between left and right frames
                         * (infiniTrack only, 0 otherwise) */
    uint32 counter; /*!< Image counter */
    ftkPixelFormat format; /*!< Pixel format */
    uint16 width; /*!< Image width (in pixels) */
    uint16 height; /*!< Image height (in pixels) */
    int32 imageStrideInBytes; /*!< Image width * size of pixel + padding in
                               * bytes */
}
PACK1_STRUCT_END( ftkImageHeader );

/*! \brief Fiducial raw data
 *   Fiducial raw data are low-level detection results from left and right
 * images.
 */
PACK1_STRUCT_BEGIN( ftkRawData )
{
    floatXX centerXPixels; /*!< Horizontal position of the center of the
                            * fiducial in image reference (unit pixels) */
    floatXX centerYPixels; /*!< Vertical position of the center of the fiducial
                            * in image reference (unit pixels)  */
    ftkStatus status; /*!< Status of the blob. */
    uint32 pixelsCount; /*!< Contain the surface of pixels composing the
                         * fiducial (unit pixels^2) */
    floatXX probability; /*!< Use the aspect ratio of the fiducial to define a
                          * probability (range 0..1) of being a valid fiducial
                          * */
}
PACK1_STRUCT_END( ftkRawData );

/** \brief 3D point structure
 *
 * This stores the coordinates of a 3D point.
 */
PACK1_STRUCT_BEGIN( ftk3DPoint )
{
    floatXX x, /*!< 3D position, x component (unit mm) */
            y, /*!< 3D position, y component (unit mm) */
            z; /*!< 3D position, z component (unit mm) */
}
PACK1_STRUCT_END( ftk3DPoint );

/*! \brief Fiducial 3D data after left-right triangulation
 *
 *   3D fiducials are retrieved after triangulation and matching of raw data.
 *
 *   Errors description:
 *
 *   "Epipolar geometry is the geometry of stereo vision. When two cameras view
 * a 3D scene from
 *   two distinct positions, there are a number of geometric relations between
 * the 3D points
 *   and their projections onto the 2D images that lead to constraints between
 * the image points.
 *   These relations are derived based on the assumption that the cameras can be
 * approximated
 *   by the pinhole camera model." [source: wikipedia]
 *
 *   - epipolarErrorPixels represents the signed distance between the right
 * epipolar line
 *     (of the left fiducial) and its matching right fiducial. Units are in
 * pixels.
 *   - triangulationErrorMM represents the minimum distance of the 3D lines
 * defined by the
 *     left optical center and the left projection and the right optical center
 * and the
 *     right position. Units are in millimeters.
 *
 *   Probability:
 *
 *   The probability is defined by the number of potential matches. Basically
 * defined by
 *   number of potential matched points that are at a specified distance from
 * the epipolar
 *   lines. This ambiguity is usually disambiguated once the 3D point is matched
 * with a
 *   marker geometry.
 *
 *   \see ftkRawData
 *   \see http://en.wikipedia.org/wiki/Epipolar_geometry
 */
PACK1_STRUCT_BEGIN( ftk3DFiducial )
{
    uint32 leftIndex; /*!< Index of the corresponding ftkRawData in the left
                       * image */
    uint32 rightIndex; /*!< Index of the corresponding ftkRawData in the right
                        * image */
    ftk3DPoint positionMM; /*!< 3D position of the center of the fiducial (unit
                            * mm) */
    floatXX epipolarErrorPixels; /*!< Epipolar matching error (unit pixel) (see
                                  * introduction)  */
    floatXX triangulationErrorMM; /*!< Triangulation error (unit mm) (see
                                   * introduction)  */
    floatXX probability; /*!< Probability (range 0..1) (see introduction)  */
}
PACK1_STRUCT_END( ftk3DFiducial );

/// Maximum number of fiducials that define a geometry
#define FTK_MAX_FIDUCIALS 4

/*! \brief Geometric description of a marker
 *
 *   The geometry can be defined in any referential. It will only influence to
 * output of the pose
 *   (which is the transformation between the geometry referential and the
 * device referential).
 *
 */
PACK1_STRUCT_BEGIN( ftkGeometry )
{
    uint32 geometryId; /**< Unique Id defining the geometry */
    uint32 version     : 8; /**< Version of the geometry structure. */
    uint32 pointsCount : 24; /**< Number of points defining the geometry */
    ftk3DPoint positions[ FTK_MAX_FIDUCIALS ]; /**< 3D position of points
                                                * defining the geometry (unit
                                                * mm) */
}
PACK1_STRUCT_END( ftkGeometry );

/// Define an invalid fiducial correspondence
#define INVALID_ID 0xffffffff

/*! \brief Marker data after left-right triangulation and marker retrieval
 *
 *   Marker are retrieved within the 3D fiducials data based on their unique
 * geometry.
 *   When several markers are used, it is recommended to use specific geometries
 * in order
 *   to provide a unique tracking.
 *
 *   Tracking id is provided to differentiate between several markers of the
 *   same geometry. The id is reset when less than 3 spheres composing the
 * marker are visible.
 *
 *   geometryPresenceMask enabled to get the correspondence between the indexes
 * of
 *   fiducials specified in the geometry and 3D fiducial (ftk3DFiducial) indexes
 * of the
 *   current measure. When a match is invalid, it is set to \c INVALID_ID.
 * Alternatively,
 *   valid matches can be retrieved via the geometryPresenceMask.
 *
 *   Marker rigid transformation (rotation and translation) is the
 * transformation from
 *   the geometry referential to the measures of the sensor. Registration error
 * is
 *   the mean distance of geometry and measured fiducials, expressed in the same
 * referential.
 *
 *   \see ftkGeometry
 *   \see ftk3DFiducial
 */
PACK1_STRUCT_BEGIN( ftkMarker )
{
    uint32 id; /*!< Tracking id */
    uint32 geometryId; /*!< Geometric id, i.e. the unique id of the used
                        * geometry. */
    uint32 geometryPresenceMask; /*!< Presence mask of fiducials expressed as
                                  * their geometrical indexes */
    uint32 fiducialCorresp[ FTK_MAX_FIDUCIALS ]; /*!< Correspondence between
                                                  * geometry index and 3D
                                                  * fiducials indexes or
                                                  * INVALID_ID */
    floatXX rotation[ 3 ][ 3 ]; /*!< Rotation matrix: format [row][column] */
    floatXX translationMM[ 3 ]; /*!< translation vector (unit mm) */
    floatXX registrationErrorMM; /*!< Registration mean error (unit mm) */
}
PACK1_STRUCT_END( ftkMarker );

/*! \brief Stores internal parameters.
 *
 * This is meant to contain binary data for the driver internal use.
 */
PACK1_STRUCT_BEGIN( ftkPrivate )
{
    uint8 Reserved[ 256u ]; /**< Internal data. */
}
PACK1_STRUCT_END( ftkPrivate );

/*! \brief Store all data from a pair of images
 *
 *   This structure stores all the buffers that can be retrieved from a pair of
 * images.
 *   The control of the fields to be retrieved can be specified during
 * initialization.
 *
 *   Fields can be grouped in different section:
 *   - imageHeader: get current image information;
 *   - imageLeftPixels: get left image pixels;
 *   - imageRightPixels: get right image pixels;
 *   - rawDataLeft: retrieve the raw data from left image;
 *   - rawDataRight: retrieve the raw data from right image;
 *   - threeDFiducials: retrieve 3D positions of left-right matches
 *   - markers: retrieve marker poses (rotation + translation)
 *
 *   Every group of fields must be initialized to retrieve corresponding
 * information.
 *   The following example, presents such an initialization:
 *
 *   \code
 *       ftkFrameQuery fq;
 *       memset (&fq, 0, sizeof (fq));
 *
 *       // Initialize a buffer of 100 raw data coming from the left image
 *       fq.rawDataLeft = new ftkRawData [100];
 *       fq.rawDataLeftVersionSize.ReservedSize = sizeof (ftkRawData) * 100;
 *       // fq.rawDataLeftVersionSize.Version will be set after calling
 * ftkGetLastFrame
 *       // fq.rawDataLeftCount will be set after calling ftkGetLastFrame
 *       // fq.rawDataLeftStat will be set after calling ftkGetLastFrame
 *   \endcode
 *
 *   Notes:
 *   - imageHeader, imageLeftPixels, imageRightPixels have only one occurrence
 *   - imageLeftVersionSize and imageRightVersionSize present the allocated size
 * for the image
 *   - the different status inform if the operation is successful or not
 *
 */
PACK1_STRUCT_BEGIN( ftkFrameQuery )
{
    ftkImageHeader* imageHeader; /*!< IN  - Address of the image header to store
                                  * to */
    ftkVersionSize imageHeaderVersionSize; /*!< IN  - 8 MSB contain the version
                                            * info, 24 LSB should contain sizeof
                                            * (ftkImageHeader) */
    ftkQueryStatus imageHeaderStat; /*!< OUT - Status of the image header (set
                                     * by driver) */

    uint8* imageLeftPixels; /*!< IN  - Address of the the left pixels to store
                             * to */
    ftkVersionSize imageLeftVersionSize; /*!< IN  - 8 MSB contain the version
                                          * info, 24 LSB should contain at least
                                          * imageStrideInBytes * height */
    ftkQueryStatus imageLeftStat; /*!< OUT - Status of the left image pixels
                                   * (set by driver) */

    uint8* imageRightPixels; /*!< IN  - Address of the the right pixels to store
                              * to */
    ftkVersionSize imageRightVersionSize; /*!< IN  - 8 MSB contain the version
                                           * info, 24 LSB should contain at
                                           * least imageStrideInBytes * height
                                           * */
    ftkQueryStatus imageRightStat; /*!< OUT - Status of the right image pixels
                                    * (set by driver) */

    ftkRawData* rawDataLeft; /*!< IN  - Address of the left raw data to store to
                              * */
    ftkVersionSize rawDataLeftVersionSize; /*!< IN  - 8 MSB contain the version
                                            * info, 24 LSB should contain X *
                                            * sizeof (ftkRawData), X > 0 */
    uint32 rawDataLeftCount; /*!< OUT - Number of resulting raw data */
    ftkQueryStatus rawDataLeftStat; /*!< OUT - Status of the left raw data (set
                                     * by driver) */

    ftkRawData* rawDataRight; /*!< IN  - Address of the right raw data to store
                               * to */
    ftkVersionSize rawDataRightVersionSize; /*!< IN  - 8 MSB contain the version
                                             * info, 24 LSB should contain X *
                                             * sizeof (ftkRawData), X > 0 */
    uint32 rawDataRightCount; /*!< OUT - Number of resulting raw data */
    ftkQueryStatus rawDataRightStat; /*!< OUT - Status of the right raw data
                                      * (set by driver) */

    ftk3DFiducial* threeDFiducials; /*!< IN  - Address of the 3D fiducials to
                                     * store to */
    ftkVersionSize threeDFiducialsVersionSize; /*!< IN  - 8 MSB contain the
                                                * version info, 24 LSB should
                                                * contain X * sizeof
                                                * (ftk3DFiducial), X > 0 */
    uint32 threeDFiducialsCount; /*!< OUT - Number of resulting 3D fiducials */
    ftkQueryStatus threeDFiducialsStat; /*!< OUT - Status of the 3d fiducials
                                         * (set by driver) */

    ftkMarker* markers; /*!< IN  - Address of the markers to store to */
    ftkVersionSize markersVersionSize; /*!< IN  - 8 MSB contain the version
                                        * info, 24 LSB should contain X * sizeof
                                        * (ftkMarker), X > 0 */
    uint32 markersCount; /*!< OUT - Number of resulting markers */
    ftkQueryStatus markersStat; /*!< OUT - Status of the markers (set by driver)
                                 * */

    ftkPrivate internals; /*!< Usage private to the library, touching this field
                           * may break the processing. */
}
PACK1_STRUCT_END( ftkFrameQuery );

#ifdef __cplusplus

/** \brief Structure providing helpers for advanced information getting.
 *
 * This structure allows to get extra information from / on a frame.
 */
    #ifdef ATR_MSVC
struct __declspec ( dllexport ) ftkFrameQueryExt : public ftkFrameQuery
    #else
struct ftkFrameQueryExt : public ftkFrameQuery
    #endif
{
    ftkFrameQueryExt( ftkLibrary lib, uint64 sn );
    /** \brief Getter for the \e total number of detected raw data on the
     * left camera.
     *
     * This method allows to get the \e total number of detected raw data
     * for the left camera. This number does \e not depend on the allocated
     * memory in the ftkFrameQuery::rawDataLeft field.
     *
     * \warning The information is valid for the \e last frame gotten using
     * ftkGetLastFrame.
     *
     * \param[out] count pointer on the number of detected blobs for the
     * left camera.
     *
     * \retval FTK_OK if the value could be successfully read,
     */
    ftkError totalRawDataLeftCount( uint32* count );
    /** \brief Getter for the \e total number of detected raw data on the
     * right camera.
     *
     * This method allows to get the \e total number of detected raw data
     * for the right camera. This number does \e not depend on the allocated
     * memory in the ftkFrameQuery::rawDataRight field.
     *
     * \warning The information is valid for the \e last frame gotten using
     * ftkGetLastFrame.
     *
     * \param[out] count pointer on the number of detected blobs for the
     * right camera.
     *
     * \retval FTK_OK if the value could be successfully read,
     */
    ftkError totalRawDataRightCount( uint32* count );
    /** \brief Getter for the \e total number of detected 3D fiducials.
     *
     * This method allows to get the \e total number of detected 3D
     * fiducials. This number does \e not depend on the allocated memory in
     * the ftkFrameQuery::threeDFiducials field.
     *
     * \warning The information is valid for the \e last frame gotten using
     * ftkGetLastFrame.
     *
     * \param[out] count pointer on the number of detected 3D fiducials.
     *
     * \retval FTK_OK if the value could be successfully read,
     */
    ftkError total3DFiducialsCount( uint32* count );
    /** \brief Getter for the \e total number of detected markers.
     *
     * This method allows to get the \e total number of detected markers
     * This number does \e not depend on the allocated memory in the
     * ftkFrameQuery::threeDFiducials field.
     *
     * \warning The information is valid for the \e last frame gotten using
     * ftkGetLastFrame.
     *
     * \param[out] count pointer on the number of detected markers.
     *
     * \retval FTK_OK if the value could be successfully read,
     */
    ftkError totalMarkersCount( uint32* count );

    /** \brief Library handle.
     */
    ftkLibrary lib;
    /** \brief Serial number of the read device.
     */
    uint64 SerialNumber;
};

#endif


/// System dependant function prefixes for DLLs
#ifdef ATR_WIN
    #   define _CDECL_ __cdecl
#else
    #   define _CDECL_
#endif

/// Callback for device enumeration
typedef void ( _CDECL_ * ftkDeviceEnumCallback )( uint64 sn, void* user,
                                                  ftkDeviceType type );

/// Callback for options enumeration
typedef void ( _CDECL_ * ftkOptionsEnumCallback )( uint64 sn, void* user,
                                                   ftkOptionsInfo* oi );

/// Callback for geometry enumeration
typedef void ( _CDECL_ * ftkGeometryEnumCallback )( uint64 sn, void* user,
                                                    ftkGeometry* in );

// ----------------------------------------------------------------
/** \defgroup library General Library Functions
 *   \brief Functions to initialize and close the library.
 \{ */


/*! \brief Initialize the library.
 *
 *   This function initialises the library and creates the handle needed by
 *   the other library functions. It must therefore be called before any
 *   other function from the library.
 *
 *   \return the library handle or 0 if an error occurred.
 *
 *   \code
 *
 *       // Initialize and close library
 *
 *       ftkLibrary handle = ftkInit ();
 *       if (!handle)
 *           ERROR ("Cannot open library");
 *
 *       // ...
 *
 *       if (ftkClose (handle) != FTK_OK)
 *           ERROR ("Cannot close library");
 *
 *   \endcode
 *
 *   \see ftkClose
 */
ATR_EXPORT ftkLibrary ftkInit();

/*! \brief Close the library.
 *
 *   This function destroys the library handle and frees the resources. Any
 *   call to a library function after this will fail.
 *
 *   \param[in] lib an initialized library handle
 *
 *   \retval FTK_OK
 *
 *   \see ftkInit
 */
ATR_EXPORT ftkError ftkClose( ftkLibrary lib );


/*! \brief Getter for the library version
 *
 *   This function allows to access the current SDK version as a string. The
 *   format is Major.Minor.Revision.Build (Unix timestamp).
 *
 *   \param[out] bufferOut pointer on the output buffer, contains the data
 *   and the actual data size.
 */
ATR_EXPORT void ftkVersion( ftkBuffer* bufferOut );

/* \} */

// ----------------------------------------------------------------
/** \defgroup device Device Functions
 *   \brief Function to enumerate devices
 \{ */

/*! \brief Enumerate available infiniTrack devices.
 *
 *   This function allows to scan all the connected devices and to call
 *   the user-defined callback function for any found device.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] cb the device enumeration callback
 *   \param[in] user parameter of the callback
 *
 *   \code
 *
 *       void deviceEnumCallback (uint64 sn, void* user, ftkDeviceType type)
 *       {
 *           uint64* lastDevice = (uint64*) user;
 *           if (lastDevice)
 * lastDevice = sn;
 *       }
 *
 *       main ()
 *       {
 *           // Initialize library (see ftkInit example)
 *
 *           uint64 sn = 0LL;
 *           if (ftkEnumerateDevices (lib, deviceEnumCallback, &sn) != FTK_OK)
 *               ERROR ("Cannot enumerate devices");
 *           if (!sn)
 *               ERROR ("No device connected");
 *
 *           // ...
 *       }
 *
 *   \endcode
 *
 *   \see ftkInit
 *   \see ftkClose
 *
 *   \retval FTK_OK if the enumeration could be done correctly,
 *   \retval FTK_ERR_INV_PTR if the \c lib handle was not correctly
 *   initialised.
 */
ATR_EXPORT ftkError ftkEnumerateDevices( ftkLibrary lib,
                                         ftkDeviceEnumCallback cb, void* user );

/* \} */

// ----------------------------------------------------------------
/** \defgroup frame Frame Functions
 *   \brief Functions to acquire frames (data from the tracking system)
 \{ */

/** \brief Creates a frame instance.
 *
 * This function allows to initialise a frame instance. The following
 * example is valid.
 *
 * \code
 * ftkFrameQuery* frame = ftkCreateFrame();
 *
 * if ( frame == 0 )
 * {
 *     // error management.
 *     return ;
 * }
 * \endcode
 *
 * \warning Not using this function may lead to problems getting the data
 * when using ftkGetLastFrame.
 *
 * \warning The user must call ftkDeleteFrame to correctly deallocate the
 * memory.
 *
 * \return a ftkFrameQuery pointer, or \c 0 if an error occurred.
 */
ATR_EXPORT ftkFrameQuery* ftkCreateFrame();

/** \brief Initialises a frame.
 *
 * This function allows to initialise a frame instance, which content can
 * be parametrised. This function can also be used to reinitialise an
 * instance, e.g. to increase the number of retrieved markers. The
 * following example is valid.
 *
 * \code
 * ftkFrameQuery* frame = ftkCreateFrame();
 *
 * if ( frame == 0 )
 * {
 *     // error management.
 *     return ;
 * }
 *
 * if ( ftkSetFrameOptions( false, false, 128u, 128u, 100u, 10u, frame ) !=
 *FTK_OK )
 * {
 *   // error management
 *   return;
 * }
 *
 * if ( ftkSetFrameOptions( false, false, 64u, 64u, 42u, 10u, frame ) != FTK_OK
 *)
 * {
 *   // error management
 *   return;
 * }
 * \endcode
 *
 * \warning Not using this function may lead to problems getting the data
 * when using ftkGetLastFrame.
 *
 * \warning The user must call ftkDeleteFrame to correctly deallocate the
 * memory.
 *
 * \param[in] leftPixels should be \c true if the left picture is to be
 * retrieved.
 * \param[in] rightPixels should be \c true if the right picture is to be
 * retrieved.
 * \param[in] leftRawDataSize maximal number of retrieved ftkRawData
 * instances for the left camera.
 * \param[in] rightRawDataSize maximal number of retrieved ftkRawData
 * instances for the right camera.
 * \param[in] threeDFiducialsSize maximal number of retrieved ftk3DFiducial
 * instances.
 * \param[in] markersSize maximal number of retrieved ftkMarker instances.
 * \param[in,out] frame pointer on an initialised ftkFrameQuery instance.
 *
 * \retval FTK_OK if the option setting could be successfully performed,
 * \retval FTK_ERR_INIT if the \c frame pointer was not initialised using
 * ftkCreateFrame,
 * \retval FTK_ERR_INV_POINTER if an allocation failed.
 */
ATR_EXPORT ftkError ftkSetFrameOptions( bool leftPixels, bool rightPixels,
                                        uint32 leftRawDataSize,
                                        uint32 rightRawDataSize,
                                        uint32 threeDFiducialsSize,
                                        uint32 markersSize,
                                        ftkFrameQuery* frame );

/** \brief Frees the memory from the allocated fields.
 *
 * This function deallocates the used memory for the \e members of the
 * instance, \c not the instance itself!. The
 * following example is valid.
 *
 * \code
 * ftkFrameQuery* frame = ftkCreateFrame();
 *
 * if ( frame == 0 )
 * {
 *     // error management.
 *     return;
 * }
 *
 * if ( ftkSetFrameOptions( false, false, 128u, 128u, 100u, 10u, frame ) !=
 *FTK_OK )
 * {
 *   // error management
 *   return;
 * }
 *
 * if ( ftkSetFrameOptions( false, false, 64u, 64u, 42u, 10u, frame ) != FTK_OK
 *)
 * {
 *   // error management
 *   return;
 * }
 *
 * err = ftkDeleteFrame( frame );
 * if ( err != FTK_OK )
 * {
 *     // error management
 *     return;
 * }
 *
 * // Using frame will crash the API, as it was deleted!
 * \endcode
 *
 * \param[in] frame pointer on the instance which will be deleted.
 *
 * \retval FTK_OK if the cleaning performed successfully,
 * \retval FTK_ERR_INIT if the \c frame pointer was not initialised using
 * ftkCreateFrame.
 */
ATR_EXPORT ftkError ftkDeleteFrame( ftkFrameQuery* frame );

/*! \brief Retrieve the latest available frame.
 *
 *   A frame contains all the data related to a pair of image.
 *   The frame query structure must be initialized prior to this function call
 *   in order to specify what type of information should be retrieved.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device
 *   \param[in,out] frameQueryInOut an initialized ftkFrameQuery structure
 *   \param[in] timeoutMS (fusionTrack only) 0 -> block the call until the
 *   latest frame is available, X > 0 -> generate a timeout if no frame
 *   available within X ms.
 *
 *   \code
 *
 *       main ()
 *       {
 *           // Initialize library (see ftkInit example)
 *           // Get an attached device (see ftkEnumerateDevices example)
 *
 *           ftkFrameQuery fq;
 *           memset (&fq, 0, sizeof (fq));
 *           if ( ftkInitFramw( false, false, 0u, 0u, 0u, 16u, &fq ) != FTK_OK )
 *           {
 *               ERROR ("Error initialising frame");
 *           }
 *
 *           // Wait until the next frame is available
 *           if (ftkGetLastFrame (lib, sn, &fq, 0) != FTK_OK)
 *               ERROR ("Error acquiring frame");
 *
 *           if (markersStat == QS_OK)
 *               for (uint32 u = 0; u < fq.markersCount; u++)
 *               {
 *                   // Access marker data here (qt.markers [u] ...)
 *               }
 *       }
 *
 *   \endcode
 *
 *   \see ftkInit
 *   \see ftkEnumerateDevices
 *
 *   \retval FTK_OK if the frame could be retrieved correctly,
 *   \retval FTK_ERR_INIT if the \c frame pointer was not initialised using
 *   ftkCreateFrame,
 *   \retval FTK_ERR_INV_PTR if the \c lib handle was not correctly
 *   initialised or if the \c frameQueryInOut is null or if the internal
 *   data for the picture could not be allocated,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_INTERNAL if the triangulation or the marker matcher
 *   class are not properly initialised  or if no image are retrieved or if
 *   the image size is invalid or if a compressed image is corrupted,
 *   \retval FTK_ERR_COMP_ALGO if the temperature compensation algorithm is
 *   undefined,
 *   \retval FTK_ERR_SYNC if the retrieved pictures are not
 *   synchronised,
 *   \retval FTK_WAR_NO_FRAME if no frame are available,
 *   \retval FTK_ERR_SEG_OVERFLOW if an overflow occurred during image
 *   segmentation,
 *   \retval FTK_WAR_SHOCK_DETECTED if a shock which potentially
 *   decalibrated the device has been detected.
 *   \retval FTK_WAR_TEMP_LOW if the current temperature is too low for
 *   compensation,
 *   \retval FTK_WAR_TEMP_HIGH if the current temperature of the device is
 *   too high for compensation.
 */
ATR_EXPORT ftkError ftkGetLastFrame( ftkLibrary lib, uint64 sn,
                                     ftkFrameQuery* frameQueryInOut,
                                     uint32 timeoutMS /* 0 = block, X > 0 =
                                                       * timeout in ms */                    );

/*! \brief Reprocess the given frame.
 *
 *   Frame reprocessing allows the user to reprocess only a part of the
 *   contained data, i.e. the markers, or 3D and markers. The current
 *   implementation does \e not support a built-in reprocessing of the
 *   pictures, meaning that pixel reprocessing must be done by a user
 *   defined function.
 *
 *   The user must specify which data must be reprocessed by setting the
 *   corresponding status flag to \c QS_REPROCESS before calling the
 *   function. This flag is recursive, in the sense that asking for a
 *   reprocessing of the raw data will trigger a reprocessing of the 3D
 *   fiducials and the markers.
 *
 *   \warning Does \e not supply pixel reprocessing!
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device
 *   \param[in,out] frameQueryInOut an initialized ftkFrameQuery structure
 *
 *   \retval FTK_OK if the frame could be reprocessed correctly (which may
 *   indicate that no reprocessing was actually asked as well),
 *   \retval FTK_ERR_INV_PTR if the \c lib handle was not correctly
 *   initialised or if the \c frameQueryInOut is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_INTERNAL if the triangulation or marker matcher class are
 * not
 *   initialised correctly or image reprocessing is requested,
 *   \retval FTK_WAR_FRAME if there are no images provided,
 *   \retval FTK_ERR_SEG_OVERFLOW if an overflow occurred during image
 *   segmentation,
 *   \retval FTK_WAR_TEMP_LOW if the current temperature is too low for
 *   compensation,
 *   \retval FTK_WAR_TEMP_HIGH if the current temperature of the device is
 *   too high for compensation.
 */
ATR_EXPORT ftkError ftkReprocessFrame( ftkLibrary lib, uint64 sn,
                                       ftkFrameQuery* frameQueryInOut );

/* \} */

// ----------------------------------------------------------------

#ifdef ATR_FTK
/** \defgroup data Data Functions
 *   \brief Functions read data from the device sensors.
 * \{
 */

/*! \brief Getter for the accelerometer data.
 *
 *   This function allows to access the current accelerometer data. The
 *   acceleration are returned as a 3D vector in standard units.
 *
 *   \warning In releases prior to 2.0.3.150, the acceleration was given
 *   in units of the Earth gravitational constant (\f$ 9.81 m \, s^{-2}\f$).
 *
 *   \deprecated The second accelerometer reading was introduced in release
 *   2.3, because a second accelerometer was added in the fusionTrack. The
 *   second reading may become mandatory.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device
 *   \param[out] firstValue pointer on the output acceleration read by the
 *   first accelerometer in \f$m \, s^{-2}\f$ units.
 *   \param[out] secondValue pointer on the output acceleration read by the
 *   second accelerometer in \f$m \, s^{-2}\f$ units.
 *
 *   \retval FTK_OK if the data could be retrieved successfully.
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly
 *   initialised, or the \c firstValue pointer is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_INV_OPT_PAR if the wanted register cannot be read.
 */
ATR_EXPORT ftkError ftkGetAcceleration( ftkLibrary lib, uint64 sn,
                                        ftk3DPoint* firstValue,
                                        ftk3DPoint* secondValue = 0 );

/** \brief Getter for the real time clock timestamp.
 *
 *   This function allows to get the current timestamp of the fusionTrack,
 *   given by the on board real time clock module. The value is the linux
 *   timestamp: the number of seconds since January 1st 1970 at 00:00:00.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device
 *   \param[out] timestamp pointer on the written timestamp.
 *
 *   \retval FTK_OK if the data could be retrieved successfully.
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly
 *   initialised, or the \c firstValue pointer is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_INV_OPT_PAR if the wanted register cannot be read.
 */
ATR_EXPORT ftkError ftkGetRealTimeClock( ftkLibrary lib, uint64 sn,
                                         uint64* timestamp );

/** \}
 */
#endif


// ----------------------------------------------------------------
/** \defgroup geometries Geometries Functions
 *   \brief Functions to set, clear or enumerate marker geometries
 \{ */

/*! \brief Register a new marker geometry to be detect.
 *
 *   This function tells the driver to look for the given geometry in the
 *   data.
 *
 *   The system will try to match the registered geometry with the raw data.
 *   Adding a geometry is immediate. You can remove a geometry with
 *   ftkClearGeometry or enumerate them with ftkEnumerateGeometries.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device
 *   \param[in] geometryIn a valid geometry to be detected
 *
 *   \code
 *
 *       // Initialize library (see ftkInit example)
 *       // Get an attached device (see ftkEnumerateDevices example)
 *
 *       // Attach a new geometry (id=52) composed of four fiducials
 *       // Note that you can define the geometry in any referential.
 *       // It will only change the pose of the marker
 *
 *       ftkGeometry markerGeometry;
 *       markerGeometry.geometryId = 52;
 *       markerGeometry.pointsCount = 4;
 *       markerGeometry.positions[0].x = 0.000000f;
 *       markerGeometry.positions[0].y = 0.000000f;
 *       markerGeometry.positions[0].z = 0.000000f;
 *
 *       markerGeometry.positions[1].x = 78.736931f;
 *       markerGeometry.positions[1].y = 0.000000f;
 *       markerGeometry.positions[1].z = 0.000000f;
 *
 *       markerGeometry.positions[2].x = 21.860918f;
 *       markerGeometry.positions[2].y = 47.757847f;
 *       markerGeometry.positions[2].z = 0.000000f;
 *
 *       markerGeometry.positions[3].x = 111.273277f;
 *       markerGeometry.positions[3].y = 51.558617f;
 *       markerGeometry.positions[3].z = -2.107444f;
 *
 *       if (ftkSetGeometry (lib, sn, &markerGeometry) != FTK_OK)
 *           ERROR ("Cannot set geometry");
 *
 *   \endcode
 *
 *   \see ftkInit
 *   \see ftkEnumerateDevices
 *   \see ftkClearGeometry
 *   \see ftkEnumerateGeometries
 *
 *   \retval FTK_OK if the geometry could be set correctly,
 *   \retval FTK_ERR_INV_PTR if the \c lib handle was not correctly
 *   initialised or if the \c geometryIn pointer is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_GEOM_PTS if the number of points in the geometry is
 *   strictly lower than 3.
 */
ATR_EXPORT ftkError ftkSetGeometry( ftkLibrary lib, uint64 sn,
                                    ftkGeometry* geometryIn );

/*! \brief  Clear a registered geometry (giving its geometry id).
 *
 *   This function tells the driver to stop looking for the given geometry
 *   in the data.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device
 *   \param[in] geometryId a valid geometry to be unregistered
 *
 *  \retval FTK_OK if the geometry could be set correctly,
 *  \retval FTK_ERR_INV_PTR the \c lib handle was not correctly initialised,
 *  \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *  \retval FTK_WAR_GEOM_ID if the wanted geometry is not registered.
 */
ATR_EXPORT ftkError ftkClearGeometry( ftkLibrary lib, uint64 sn,
                                      uint32 geometryId );

/*! \brief Enumerate the registered geometries.
 *
 *   This function enumerates all the registered geometries and allows to
 *   apply a user-defined function on each of them.
 *
 *   \see ftkEnumerateDevices for an example of enumeration).
 *
 *   \retval FTK_OK if the geometry could be set correctly,
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly initialised,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved.
 */
ATR_EXPORT ftkError ftkEnumerateGeometries( ftkLibrary lib, uint64 sn,
                                            ftkGeometryEnumCallback cb,
                                            void* user );

/* \} */

// ----------------------------------------------------------------


/*! \defgroup options Options Functions
 *   \brief Functions to get, set or enumerate options
 *
 *   Options enable getting or setting configuration information from/to the
 * driver module.
 *   Options may be global or specific to a type of device.
 *   Most options can be tested and are accessible in the demo program.
 *
 *   See the different options structures and enumerators for more detailed
 * information.
 *
 *   \see ftkComponent
 *   \see ftkOptionType
 *   \see ftkOptionGetter
 *   \see ftkOptionStatus
 *   \see ftkOptionsInfo
 */
/* \{ */


/*! \brief Enumerate available options.
 *
 *   This function enumerates all the available options and allows to
 *   apply a user-defined function on each of them.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, sn=0LL for general purpose options)
 *   \param[in] cb the option enumeration callback
 *   \param[in] user parameter of the callback
 *
 *   Example of code to display available option and their respective ids.
 *
 *   \code
 *
 *       void optionEnumerator (uint64 sn, void* user, ftkOptionsInfo* oi)
 *       {
 *           printf("Option (%u)  %s\n", oi->id, oi->name);
 *       }
 *
 *       main ()
 *       {
 *           // Initialize library (see ftkInit example)
 *           // Get an attached device (see ftkEnumerateDevices example)
 *
 *           if (ftkEnumerateOptions(lib, sn, optionEnumerator, NULL) != FTK_OK)
 *               ERROR ("Cannot enumerate options");
 *       }
 *
 *   \endcode
 *
 *   \see ftkInit
 *   \see ftkEnumerateDevices
 *   \see ftkOptionsInfo
 *
 *   \retval FTK_OK if the enumeration could be done correctly,
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly
 *   initialised,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved.
 */
ATR_EXPORT ftkError ftkEnumerateOptions( ftkLibrary lib, uint64 sn,
                                         ftkOptionsEnumCallback cb,
                                         void* user );

/*! \brief Get an integer option.
 *
 *   This function allows to get an integer option value. Different values
 *   can be retrieved:
 *     - the minimum value for the option;
 *     - the maximum value for the option;
 *     - the default value of the option;
 *     - the current value of the option.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device (compulsory for
 *   device specific options, sn=0LL for general purpose options)
 *   \param[in] optID id of the option (can be retrieved with
 *   ftkEnumerateOptions)
 *   \param[out] out output value
 *   \param[in] what define what to retrieve minimum, maximum, default or
 *   actual value.
 *
 *   \see ftkSetInt32
 *
 *   \retval FTK_OK if the value retrieval could be done correctly,
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly initialised
 *   or if \c out is null, \retval FTK_ERR_INV_SN if the device could not be
 *   retrieved,
 *   \retval FTK_ERR_INV_OPT_PAR if the option does not exist, or is not of
 *   type \c int32.
 */
ATR_EXPORT ftkError ftkGetInt32( ftkLibrary lib, uint64 sn, uint32 optID,
                                 int32* out, ftkOptionGetter what );

/*! \brief Set an integer option.
 *
 *   This function allows to set an integer option.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device (compulsory for
 *   device specific options, sn=0LL for general purpose options)
 *   \param[in] optID id of the option (can be retrieved with
 *   ftkEnumerateOptions)
 *   \param[in] val value to set
 *
 *   \see ftkGetInt32
 *
 *   \retval FTK_OK if the value setting could be done correctly,
 *   \retval    FTK_ERR_INV_PTR the \c lib handle was not correctly initialised
 *   or if \c out is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_INV_OPT_VAL if the value is invalid,
 *   \retval FTK_ERR_INV_OPT if the option does not exist, or is not of type
 *   \c int32.
 */
ATR_EXPORT ftkError ftkSetInt32( ftkLibrary lib, uint64 sn, uint32 optID,
                                 int32 val );

/*! \brief Get a float option.
 *
 *   This function allows to get a floating-point option value. Different
 *   values can be retrieved:
 *     - the minimum value for the option;
 *     - the maximum value for the option;
 *     - the default value of the option;
 *     - the current value of the option.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, sn=0LL for general purpose options)
 *   \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions)
 *   \param[out] out output value
 *   \param[in] what define what to retrieve minimum, maximum, default or actual
 * value
 *
 *   \see ftkSetFloat32
 *
 *   \retval FTK_OK if the value retrieval could be done correctly,
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly initialised or
 * if
 *   \c out is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_INV_OPT_PAR if the option does not exist, or
 *   is not of type \c float32.
 */
ATR_EXPORT ftkError ftkGetFloat32( ftkLibrary lib, uint64 sn, uint32 optID,
                                   float32* out, ftkOptionGetter what );

/*! \brief Set a float option.
 *
 *   This function allows to set a floating-point option.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, sn=0LL for general purpose options)
 *   \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions)
 *   \param[in] val value to set
 *
 *   \see ftkGetFloat32
 *
 *   \retval FTK_OK if the value setting could be done correctly,
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly initialised or
 * if
 *   \c out is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_INV_OPT_VAL if the value is invalid,
 *   \retval FTK_ERR_INV_OPT if the option does not exist, or is not of type \c
 * float32.
 */
ATR_EXPORT ftkError ftkSetFloat32( ftkLibrary lib, uint64 sn, uint32 optID,
                                   float32 val );

/*! \brief Get a binary option.
 *
 *   This function allows to get the value of a binary option. Note that
 *   only the current value can be retrieved, there are no min/max or
 *   default values for binary options.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device (compulsory for
 *   device specific options, sn=0LL for general purpose options)
 *   \param[in] optID id of the option (can be retrieved with
 *   ftkEnumerateOptions)
 *   \param[out] bufferOut pointer on the output buffer, contains the data
 *   and the actual data size.
 *
 *   \see ftkSetData
 *
 *   \retval FTK_OK if the value retrieval could be done correctly,
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly initialised or
 * if
 *   \c dataOut or \c dataSizeInBytes is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_INTERNAL if the needed file cannot be read,
 *   \retval FTK_INV_OPT_PAR if the wanted register cannot be read or does not
 *   exist (invalid address).
 */
ATR_EXPORT ftkError ftkGetData( ftkLibrary lib, uint64 sn, uint32 optID,
                                ftkBuffer* bufferOut );

/*! \brief Set a binary option.
 *
 *   This function allows to set an binary option.
 *
 *   \param[in] lib an initialized library handle
 *   \param[in] sn a valid serial number of the device (compulsory for device
 * specific options, sn=0LL for general purpose options)
 *   \param[in] optID id of the option (can be retrieved with
 * ftkEnumerateOptions)
 *   \param[in] bufferIn  pointer on the input buffer, contains the data and the
 * actual data size.
 *   \see ftkGetData
 *
 *   \retval FTK_OK if the value retrieval could be done correctly,
 *   \retval FTK_ERR_INV_PTR the \c lib handle was not correctly initialised
 *   or if \c dataOut or \c dataSizeInBytes is null,
 *   \retval FTK_ERR_INV_SN if the device could not be retrieved,
 *   \retval FTK_ERR_INTERNAL if a memory allocation error occurred or if
 *   the wanted file cannot be written,
 *   \retval FTK_INV_OPT_PAR if the wanted register cannot be read or does
 *   not exits (invalid address),
 *   \retval FTK_ERR_INV_OPT_VAL if the value cannot be set.
 */
ATR_EXPORT ftkError ftkSetData( ftkLibrary lib, uint64 sn, uint32 optID,
                                ftkBuffer* bufferIn );

/* \} */

#endif

