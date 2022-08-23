
//  ==> COPYRIGHT (C) 2021 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2021 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#ifndef XSMFMCALLBACKPLAINC_H
#define XSMFMCALLBACKPLAINC_H

#include <xstypes/pstdint.h>
#include <xstypes/xsresultvalue.h>
#include <xstypes/xsdeviceidarray.h>
#include <xstypes/xsdatapacketptrarray.h>
#include <xstypes/xsbytearray.h>
#include <xstypes/xsmfmresultvalue.h>

#ifndef __cplusplus
#define XSMFMCALLBACK_INITIALIZER		{ 0, 0, 0, 0 }
#endif

struct XsString;

/*! \brief Structure that contains callback functions for the Magnetic Field Mapper
	\details When using C++, please use the overloaded class XsMfMCallback instead.

	This structure contains pointers to functions that will be called by
	the magnetic field mapper library when certain events occur. To use it
	in C, set any callback you do not wish to use to 0 and put a valid
	function pointer in the others. Then pass the object to the XsMfm
	object's addCallbackHandler function.

	\note XsMfm does not copy the structure contents and does not take
	ownership of it. So make sure it is allocated on the heap or at least
	removed from wherever it was added by calling removeCallbackHandler
	before it is destroyed.
*/
typedef struct XsMfMCallbackPlainC
{
/*! \defgroup Callbacks Callback functions.
	\addtogroup Callbacks
	@{
*/
	/*! \brief Called when a device scan is done
		\param devices The list of deviceids that were detected during the scan
	*/
	void (*m_onScanDone)(struct XsMfMCallbackPlainC* thisPtr, const XsDeviceIdArray* devices);

	/*! \brief Called when the magfield mapping has completed
		\param dev The deviceid of which the magfieldmapping as completed
		\param resValue The result of the magfield mapping
	*/
	void (*m_onMfmDone)(struct XsMfMCallbackPlainC* thisPtr, XsDeviceId dev, XsMfmResultValue resValue);

	/*! \brief Called when an error has occurred while handling incoming data
		\param dev The deviceid that generated the error message
		\param error The error code that specifies exactly what problem occurred
		\param description A string with more details of the specific problem. May be empty, but not NULL.
	*/
	void (*m_onMfmError)(struct XsMfMCallbackPlainC* thisPtr, XsDeviceId dev, XsResultValue error, const struct XsString* description);

	/*! \brief Called when the library needs to send raw data to a device connected using a custom communication channel.
	\param channelId The user provided identifier associated with the custom channel.
	\param data The array of bytes that must be forwarded to the device
	*/
	void (*m_onTransmissionRequest)(struct XsMfMCallbackPlainC* thisPtr, int channelId, const struct XsByteArray* data);

	/*! \brief Called when new data has been received from a device or read from a file. When processing on PC is enabled, this callback occurs after processing has been done and so the packet will contain the processing output.
		\details This callback is for the Live stream, so there may be gaps in the data, but it will always contain the latest data.
		\param devs The IDs of the devices that initiated the callback.
		\param packets The data packet that has been received (and processed). This may be 0 when the callback originates from a non-device, such as the XsDataBundler.
		\note For most applications, attaching to the m_onDataAvailable callback is sufficient, the specific stream callbacks are only provided for exceptional cases
	*/
	void (*m_onAllLiveDataAvailable)(struct XsMfMCallbackPlainC* thisPtr, struct XsDeviceIdArray* devs, const struct XsDataPacketPtrArray* packets);
//! @}

#ifdef __cplusplus
	// Make sure that this struct is not used in C++ (except as base class for XsCallback)
	friend class XsMfmCallback;
	XsMfMCallbackPlainC() {}
	~XsMfMCallbackPlainC() throw() {}
private:
	XsMfMCallbackPlainC(XsMfMCallbackPlainC const &);
	XsMfMCallbackPlainC& operator = (XsMfMCallbackPlainC const &);

#endif

} XsMfMCallbackPlainC;

#endif
