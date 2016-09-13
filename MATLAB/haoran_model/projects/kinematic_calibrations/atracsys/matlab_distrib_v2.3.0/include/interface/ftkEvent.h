// ===========================================================================
/*!
 *   This file is part of the ATRACSYS fusiontrack library.
 *   Copyright (C) 2015-2015 by Atracsys LLC. All rights reserved.
 *
 *  THIS FILE CANNOT BE SHARED, MODIFIED OR REDISTRIBUTED WITHOUT THE
 *  WRITTEN PERMISSION OF ATRACSYS.
 *
 *  \file     ftkEvent.h
 *  \brief    Definition of the class handling fTk events.
 *
 */
// ===========================================================================
#ifndef FTKEVENT_H
#define FTKEVENT_H

#include <ftkErrors.h>
#include <ftkPlatform.h>

#ifdef __cplusplus
struct ftkLibraryImp;
#else
typedef struct ftkLibraryImp ftkLibraryImp;
#endif
typedef ftkLibraryImp* ftkLibrary;


/** \brief Definitions of the event types.
 *
 * The event type indicates both the type \e and the version. For instance,
 * sending the 8 temperatures is type 0, in the future, if only 6 of them are
 * sent, this could be \c detTempV2 \c = \c 42.
 */
TYPED_ENUM( int32, FtkEventType )
{
    /** \brief Temperature sending, version 1.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board.
     */
    fetTempV1 = 0,
    /** \brief Event indicating the current device temperature is too low.
     */
    fetLowTemp = 1,
    /** \brief Event indicating the current device temperature is too high.
     */
    fetHighTemp = 2,
    /** \brief Event indicating the device has received at least one shock
     * above the threshold, leading to a possible decalibration.
     */
    fetShockDetected = 3,
    /** \brief Event indicating the watchdog timer deactivated the data
     * sending.
     */
    fetWatchdogTimeout = 4,
    /** \brief Temperature sending, version 2.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board;
     * and then two byte for the fan `frequency':
     *   -# fan 0;
     *   -# fan 1.
     */
    fetTempV2 = 5,
    /** \brief Temperature sending, version 3.
     *
     * The payload contains the 8 \c float32 values for the following
     * temperatures:
     *   -# camera 0, sensor 0;
     *   -# camera 0, sensor 1;
     *   -# camera 1, sensor 0;
     *   -# camera 1, sensor 1;
     *   -# IR board 0;
     *   -# IR board 1;
     *   -# main board;
     *   -# power supply board;
     * followed by:
     *   -# one byte indicating the status of the fan readings;
     *   -# 1 byte for the fan 0 input in percent;
     *   -# 2 bytes for the fan 0 speed;
     *   -# 1 byte for the fan 1 input in percent;
     *   -# 2 bytes for the fan 1 speed;
     *   -# 1 reserved byte (unused).
     */
    fetTempV3 = 6,
    /** \brief Event related to presence and state of wireless markers.
     *
     * The payload contains 16 mandatory bits, indicating which marker was
     * detected (one bit per marker). For each \e present marker, 16 additional
     * bits are present, currently not used.
     */
    fetWirelessMarkerV1 = 7,
    /** \brief Event related to availability of the calibration for a wireless
     * marker.
     *
     * The payload contains a ftkGeometry instance.
     *
     * \see ftkGeometry
     */
    fetWirelessMarkerCalibV1 = 8,
};

/** \brief Structure holding the fan status.
 *
 * This structure holds the various status bits related to fan control (for the
 * fTk 250).
 */
STRUCT_BEGIN( ftkFanStatus )
{
    /** \brief Contains 1 if the module is enabled.
     */
    uint8 FanModuleEnabled : 1;
    /** \brief Contains 1 if the fan 0 is on.
     */
    uint8 Fan0PWMEnabled   : 1;
    /** \brief Contains 1 if the fan 1 is on.
     */
    uint8 Fan1PWMEnabled   : 1;
    /** \brief Contains 1 if the speed reading for fan 0 is valid.
     */
    uint8 Fan0SpeedValid   : 1;
    /** \brief Contains 1 if the speed reading for fan 1 is valid.
     */
    uint8 Fan1SpeedValid   : 1;
#ifdef __cplusplus
    private:
        /** \brief Reserved unused bits.
         */
        uint8 Reserved         : 3;
#endif
}
STRUCT_END( ftkFanStatus );

/** \brief Structure holding an event as sent by the fusionTrack.
 *
 * This structure implements the device-oriented data structure of an event.
 *
 * Events are generated and sent by the fusionTrack itself. Some are handled
 * directly by the driver itself, others are forwarded to the user. In order to
 * get those event in a custom made program, a callback must be set using
 * the ftkRegisterEventCallback function.
 */
STRUCT_BEGIN( ftkEvent )
{
    /** \brief Type of the event.
     */
    FtkEventType Type;
    /** \brief Timestamp of the event
     *
     * This is created by the fusionTrack, it is the \f$ \mu{}s \f$ counter
     * value at the generation of the event.
     */
    uint64 Timestamp;
    /** \brief Serial number of the sending device.
     */
    uint64 SerialNumber;
    /** \brief Size of the data contained in the payload.
     */
    uint32 Payload;
    /** \brief Pointer on the payload data.
     */
    uint8* Data;
}
STRUCT_END( ftkEvent );

/** \brief Function creating a ftkEvent instance.
 *
 * This function allows to create an instance of an Event. If needed, the
 * Event::Data pointer is allocated, but has to be manually set.
 *
 * \param[in] type type of the received (or sent) event.
 * \param[in] timestamp fTk timestamp corresponding to the creation of the
 * event.
 * \param[in] serial serial number of the sending device.
 * \param[in] payload size of the additional data.
 *
 * \return a pointer on the allocated instance, or \c 0 if an error occurred.
 */
ATR_EXPORT ftkEvent* ftkCreateEvent( FtkEventType type, uint64 timestamp,
                                     uint64 serial, uint32 payload );

/** \brief Function deleting a ftkEvent instance.
 *
 * This function allows to free the allocated memory for an Event.
 *
 * \param[in] evt instance to delete.
 *
 * \retval FTK_OK if the deletion could be successfully performed,
 * \retval FTK_ERR_INV_PTR if \c evt is null.
 */
ATR_EXPORT ftkError ftkDeleteEvent( ftkEvent* evt );

/** \brief Callback definition used to forward events to the SDK layer.
 *
 * This is the signature for the callback function allowing to forward the
 * events to a custom made program. Those callbacks are executed in a dedicated
 * thread, so that it doesn't impact data reading. This function will forward
 * \e any event, a correct dispatching mechanism must be implemented by the
 * user.
 *
 * \param[in] sn serial number of the device from which the event was
 * generated.
 * \param[in] user pointer for the user of the callback function.
 * \param[in] evt pointer on the event instance.
 */
typedef void ( _CDECL_ * ftkEventCallback )( uint64 sn, void* user,
                                             ftkEvent* evt );


/** \brief Function allowing to set the event callback.
 *
 * This function sets the callback function called when an event is
 * correctly handled.
 *
 * In order to get the events in a custom made software, a callback must be
 * implemented and set.
 *
 * \param[in] lib initialised library handle.
 * \param[in] fct callback pointer.
 * \param[in] user callback data.
 *
 * \retval FTK_OK if the setting could be performed successfully,
 * \retval FTK_ERR_INV_PTR if the \c fct ptr is null,
 * \retval FTK_WAR_ALREADY_PRESENT if the callback was already defined.
 */
ATR_EXPORT ftkError ftkRegisterEventCallback( ftkLibrary lib,
                                              ftkEventCallback fct,
                                              void* user );

#endif // FTKEVENT_H
