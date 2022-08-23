
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

#ifndef GUARD_XSMFM_H_TMP
#define GUARD_XSMFM_H_TMP
#include <xstypes/xsdeviceidarray.h>
#include <xstypes/xsstringarray.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xsmatrix.h>
#include <xstypes/xsvector.h>
#include <xstypes/xsversion.h>
#include "xsmfmcallbackplainc.h"
#include "mfmconfig.h"
#ifdef __cplusplus
extern "C" {
#endif
/*! \addtogroup cinterface C Interface
	@{ */
struct XsMfm;
#ifndef __cplusplus
typedef struct XsMfm XsMfm;
#endif
MFM_DLL_API struct XsMfm* XsMfm_construct(void);
MFM_DLL_API void XsMfm_destruct(struct XsMfm* thisPtr);/*!< \copydoc XsMfm::~XsMfm()*/
MFM_DLL_API XsVersion* XsMfm_version(XsVersion* returnValue);/*!< \copydoc XsMfm::version()*/
MFM_DLL_API void XsMfm_reset(struct XsMfm* thisPtr);/*!< \copydoc XsMfm::reset()*/
MFM_DLL_API int XsMfm_scanMfmDevices(struct XsMfm* thisPtr);/*!< \copydoc XsMfm::scanMfmDevices()*/
MFM_DLL_API int XsMfm_scanMfmDevice(struct XsMfm* thisPtr, const XsPortInfo* portInfo);/*!< \copydoc XsMfm::scanMfmDevice(const XsPortInfo&)*/
MFM_DLL_API int XsMfm_scanMfmDeviceOnCustomChannel(struct XsMfm* thisPtr, int channelId, uint32_t defaultTimeout, int detectRs485);/*!< \copydoc XsMfm::scanMfmDeviceOnCustomChannel(int,uint32_t,bool)*/
MFM_DLL_API XsResultValue* XsMfm_loadInputFile(struct XsMfm* thisPtr, XsResultValue* returnValue, const XsString* inputFile, const XsDeviceIdArray* deviceIds, XsDeviceIdArray* detectedDeviceIds);/*!< \copydoc XsMfm::loadInputFile(const XsString&,const XsDeviceIdArray&,XsDeviceIdArray&)*/
MFM_DLL_API XsDeviceId* XsMfm_loadedDeviceId(const struct XsMfm* thisPtr, XsDeviceId* returnValue);/*!< \copydoc XsMfm::loadedDeviceId() const*/
MFM_DLL_API XsDeviceIdArray* XsMfm_loadedDeviceIds(const struct XsMfm* thisPtr, XsDeviceIdArray* returnValue);/*!< \copydoc XsMfm::loadedDeviceIds() const*/
MFM_DLL_API XsDeviceId* XsMfm_master(const struct XsMfm* thisPtr, XsDeviceId* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::master(const XsDeviceId&) const*/
MFM_DLL_API int XsMfm_startLogging(struct XsMfm* thisPtr, const XsDeviceId* deviceId, const XsString* logfilename);/*!< \copydoc XsMfm::startLogging(const XsDeviceId&,const XsString&)*/
MFM_DLL_API int XsMfm_startLogging_1(struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray, const XsStringArray* logfilenameArray, XsStringArray* finalFilenameArray);/*!< \copydoc XsMfm::startLogging(const XsDeviceIdArray&,const XsStringArray&,XsStringArray&)*/
MFM_DLL_API int XsMfm_stopLogging(struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::stopLogging(const XsDeviceId&)*/
MFM_DLL_API int XsMfm_stopLogging_1(struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray);/*!< \copydoc XsMfm::stopLogging(const XsDeviceIdArray&)*/
MFM_DLL_API int XsMfm_startProcessing(struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::startProcessing(const XsDeviceId&)*/
MFM_DLL_API int XsMfm_startProcessing_1(struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray);/*!< \copydoc XsMfm::startProcessing(const XsDeviceIdArray&)*/
MFM_DLL_API int XsMfm_writeResultToMt(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::writeResultToMt(const XsDeviceId&) const*/
MFM_DLL_API int XsMfm_writeResultToMt_1(const struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray);/*!< \copydoc XsMfm::writeResultToMt(const XsDeviceIdArray&) const*/
MFM_DLL_API XsString* XsMfm_writeResultToString(const struct XsMfm* thisPtr, XsString* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::writeResultToString(const XsDeviceId&) const*/
MFM_DLL_API int XsMfm_writeResultToFile(const struct XsMfm* thisPtr, const XsDeviceId* deviceId, const XsString* filename);/*!< \copydoc XsMfm::writeResultToFile(const XsDeviceId&,const XsString&) const*/
MFM_DLL_API int XsMfm_writeResultToFile_1(const struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray, const XsStringArray* filenameArray);/*!< \copydoc XsMfm::writeResultToFile(const XsDeviceIdArray&,const XsStringArray&) const*/
MFM_DLL_API void XsMfm_transmissionReceived(struct XsMfm* thisPtr, int channelId, const XsByteArray* data);/*!< \copydoc XsMfm::transmissionReceived(int,const XsByteArray&)*/
MFM_DLL_API void XsMfm_restorePreviousResults(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::restorePreviousResults(const XsDeviceId&) const*/
MFM_DLL_API void XsMfm_restorePreviousResults_1(const struct XsMfm* thisPtr, const XsDeviceIdArray* deviceIdArray);/*!< \copydoc XsMfm::restorePreviousResults(const XsDeviceIdArray&) const*/
MFM_DLL_API void XsMfm_waitForProcessingComplete(struct XsMfm* thisPtr);/*!< \copydoc XsMfm::waitForProcessingComplete()*/
MFM_DLL_API void XsMfm_clearCallbackHandlers(struct XsMfm* thisPtr, int chain);/*!< \copydoc XsMfm::clearCallbackHandlers(bool)*/
MFM_DLL_API void XsMfm_addCallbackHandler(struct XsMfm* thisPtr, XsMfMCallbackPlainC* cb, int chain);/*!< \copydoc XsMfm::addCallbackHandler(XsMfMCallbackPlainC*,bool)*/
MFM_DLL_API void XsMfm_removeCallbackHandler(struct XsMfm* thisPtr, XsMfMCallbackPlainC* cb, int chain);/*!< \copydoc XsMfm::removeCallbackHandler(XsMfMCallbackPlainC*,bool)*/
MFM_DLL_API int XsMfm_isValid(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::isValid(const XsDeviceId&) const*/
MFM_DLL_API int XsMfm_isMap3D(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::isMap3D(const XsDeviceId&) const*/
MFM_DLL_API XsVersion* XsMfm_getVersion(const struct XsMfm* thisPtr, XsVersion* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getVersion(const XsDeviceId&) const*/
MFM_DLL_API XsVersion* XsMfm_getFirmwareVersion(const struct XsMfm* thisPtr, XsVersion* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getFirmwareVersion(const XsDeviceId&) const*/
MFM_DLL_API XsMatrix* XsMfm_getMagFieldMeas(const struct XsMfm* thisPtr, XsMatrix* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getMagFieldMeas(const XsDeviceId&) const*/
MFM_DLL_API XsMatrix* XsMfm_getVerticalMeas(const struct XsMfm* thisPtr, XsMatrix* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getVerticalMeas(const XsDeviceId&) const*/
MFM_DLL_API XsMatrix* XsMfm_getMagFieldMfm(const struct XsMfm* thisPtr, XsMatrix* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getMagFieldMfm(const XsDeviceId&) const*/
MFM_DLL_API XsMatrix* XsMfm_getGeoSelMagFieldMfmModel(const struct XsMfm* thisPtr, XsMatrix* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getGeoSelMagFieldMfmModel(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getHardIronCompensation(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getHardIronCompensation(const XsDeviceId&) const*/
MFM_DLL_API XsMatrix* XsMfm_getSoftIronCompensation(const struct XsMfm* thisPtr, XsMatrix* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getSoftIronCompensation(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getGeoSelMfm(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getGeoSelMfm(const XsDeviceId&) const*/
MFM_DLL_API XsMatrix* XsMfm_getGeoSelMagFieldMeas(const struct XsMfm* thisPtr, XsMatrix* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getGeoSelMagFieldMeas(const XsDeviceId&) const*/
MFM_DLL_API XsMatrix* XsMfm_getGeoSelMagFieldMfm(const struct XsMfm* thisPtr, XsMatrix* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getGeoSelMagFieldMfm(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getNormMagFieldMeas(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getNormMagFieldMeas(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getNormGeoSelMagFieldMeas(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getNormGeoSelMagFieldMeas(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getNormMagFieldMfm(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getNormMagFieldMfm(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getHistResidualsModel(const struct XsMfm* thisPtr, XsVector* returnValue);/*!< \copydoc XsMfm::getHistResidualsModel() const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualVertical(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getNormalizedHistResidualVertical(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualDipAngle(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getNormalizedHistResidualDipAngle(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualMagnetic(const struct XsMfm* thisPtr, XsVector* returnValue, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getNormalizedHistResidualMagnetic(const XsDeviceId&) const*/
MFM_DLL_API XsVector* XsMfm_getHistResidualsModelBins(const struct XsMfm* thisPtr, XsVector* returnValue);/*!< \copydoc XsMfm::getHistResidualsModelBins() const*/
MFM_DLL_API XsVector* XsMfm_getNormalizedHistResidualBins(const struct XsMfm* thisPtr, XsVector* returnValue);/*!< \copydoc XsMfm::getNormalizedHistResidualBins() const*/
MFM_DLL_API XsReal XsMfm_getStdDevNormGeoSelMagFieldMeas(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getStdDevNormGeoSelMagFieldMeas(const XsDeviceId&) const*/
MFM_DLL_API XsReal XsMfm_getAvgValNormGeoSelMagFieldMeas(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getAvgValNormGeoSelMagFieldMeas(const XsDeviceId&) const*/
MFM_DLL_API XsReal XsMfm_getMaxErrorValNormGeoSelMagFieldMeas(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getMaxErrorValNormGeoSelMagFieldMeas(const XsDeviceId&) const*/
MFM_DLL_API XsReal XsMfm_getStdDevNormGeoSelMagFieldMfm(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getStdDevNormGeoSelMagFieldMfm(const XsDeviceId&) const*/
MFM_DLL_API XsReal XsMfm_getAvgValNormGeoSelMagFieldMfm(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getAvgValNormGeoSelMagFieldMfm(const XsDeviceId&) const*/
MFM_DLL_API XsReal XsMfm_getMaxErrorValNormGeoSelMagFieldMfm(const struct XsMfm* thisPtr, const XsDeviceId* deviceId);/*!< \copydoc XsMfm::getMaxErrorValNormGeoSelMagFieldMfm(const XsDeviceId&) const*/
/*! @} */
#ifdef __cplusplus
} // extern "C"
struct XsMfm {
	//! \brief Construct a new XsMfm* object. Clean it up with the destruct() function or delete the object
	inline static XsMfm* construct(void)
	{
		return XsMfm_construct();
	}

	//! \brief Destruct a XsMfm object and free all memory allocated for it
	inline void destruct(void)
	{
		XsMfm_destruct(this);
	}

	inline static XsVersion version(void)
	{
		XsVersion returnValue;
		return *XsMfm_version(&returnValue);
	}

	inline void reset(void)
	{
		XsMfm_reset(this);
	}

	inline bool scanMfmDevices(void)
	{
		return 0 != XsMfm_scanMfmDevices(this);
	}

	inline bool scanMfmDevice(const XsPortInfo& portInfo)
	{
		return 0 != XsMfm_scanMfmDevice(this, &portInfo);
	}

	inline bool scanMfmDeviceOnCustomChannel(int channelId, uint32_t defaultTimeout, bool detectRs485 = false)
	{
		return 0 != XsMfm_scanMfmDeviceOnCustomChannel(this, channelId, defaultTimeout, detectRs485);
	}

	inline XsResultValue loadInputFile(const XsString& inputFile, const XsDeviceIdArray& deviceIds, XsDeviceIdArray& detectedDeviceIds)
	{
		XsResultValue returnValue;
		return *XsMfm_loadInputFile(this, &returnValue, &inputFile, &deviceIds, &detectedDeviceIds);
	}

	inline XsDeviceId loadedDeviceId(void) const
	{
		XsDeviceId returnValue;
		return *XsMfm_loadedDeviceId(this, &returnValue);
	}

	inline XsDeviceIdArray loadedDeviceIds(void) const
	{
		XsDeviceIdArray returnValue;
		return *XsMfm_loadedDeviceIds(this, &returnValue);
	}

	inline XsDeviceId master(const XsDeviceId& deviceId) const
	{
		XsDeviceId returnValue;
		return *XsMfm_master(this, &returnValue, &deviceId);
	}

	inline bool startLogging(const XsDeviceId& deviceId, const XsString& logfilename)
	{
		return 0 != XsMfm_startLogging(this, &deviceId, &logfilename);
	}

	inline bool startLogging(const XsDeviceIdArray& deviceIdArray, const XsStringArray& logfilenameArray, XsStringArray& finalFilenameArray)
	{
		return 0 != XsMfm_startLogging_1(this, &deviceIdArray, &logfilenameArray, &finalFilenameArray);
	}

	inline bool stopLogging(const XsDeviceId& deviceId)
	{
		return 0 != XsMfm_stopLogging(this, &deviceId);
	}

	inline bool stopLogging(const XsDeviceIdArray& deviceIdArray)
	{
		return 0 != XsMfm_stopLogging_1(this, &deviceIdArray);
	}

	inline bool startProcessing(const XsDeviceId& deviceId)
	{
		return 0 != XsMfm_startProcessing(this, &deviceId);
	}

	inline bool startProcessing(const XsDeviceIdArray& deviceIdArray)
	{
		return 0 != XsMfm_startProcessing_1(this, &deviceIdArray);
	}

	inline bool writeResultToMt(const XsDeviceId& deviceId) const
	{
		return 0 != XsMfm_writeResultToMt(this, &deviceId);
	}

	inline bool writeResultToMt(const XsDeviceIdArray& deviceIdArray) const
	{
		return 0 != XsMfm_writeResultToMt_1(this, &deviceIdArray);
	}

	inline XsString writeResultToString(const XsDeviceId& deviceId) const
	{
		XsString returnValue;
		return *XsMfm_writeResultToString(this, &returnValue, &deviceId);
	}

	inline bool writeResultToFile(const XsDeviceId& deviceId, const XsString& filename) const
	{
		return 0 != XsMfm_writeResultToFile(this, &deviceId, &filename);
	}

	inline bool writeResultToFile(const XsDeviceIdArray& deviceIdArray, const XsStringArray& filenameArray) const
	{
		return 0 != XsMfm_writeResultToFile_1(this, &deviceIdArray, &filenameArray);
	}

	inline void transmissionReceived(int channelId, const XsByteArray& data)
	{
		XsMfm_transmissionReceived(this, channelId, &data);
	}

	inline void restorePreviousResults(const XsDeviceId& deviceId) const
	{
		XsMfm_restorePreviousResults(this, &deviceId);
	}

	inline void restorePreviousResults(const XsDeviceIdArray& deviceIdArray) const
	{
		XsMfm_restorePreviousResults_1(this, &deviceIdArray);
	}

	inline void waitForProcessingComplete(void)
	{
		XsMfm_waitForProcessingComplete(this);
	}

	inline void clearCallbackHandlers(bool chain = true)
	{
		XsMfm_clearCallbackHandlers(this, chain);
	}

	inline void addCallbackHandler(XsMfMCallbackPlainC* cb, bool chain = true)
	{
		XsMfm_addCallbackHandler(this, cb, chain);
	}

	inline void removeCallbackHandler(XsMfMCallbackPlainC* cb, bool chain = true)
	{
		XsMfm_removeCallbackHandler(this, cb, chain);
	}

	inline bool isValid(const XsDeviceId& deviceId) const
	{
		return 0 != XsMfm_isValid(this, &deviceId);
	}

	inline bool isMap3D(const XsDeviceId& deviceId) const
	{
		return 0 != XsMfm_isMap3D(this, &deviceId);
	}

	inline XsVersion getVersion(const XsDeviceId& deviceId) const
	{
		XsVersion returnValue;
		return *XsMfm_getVersion(this, &returnValue, &deviceId);
	}

	inline XsVersion getFirmwareVersion(const XsDeviceId& deviceId) const
	{
		XsVersion returnValue;
		return *XsMfm_getFirmwareVersion(this, &returnValue, &deviceId);
	}

	inline XsMatrix getMagFieldMeas(const XsDeviceId& deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getMagFieldMeas(this, &returnValue, &deviceId);
	}

	inline XsMatrix getVerticalMeas(const XsDeviceId& deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getVerticalMeas(this, &returnValue, &deviceId);
	}

	inline XsMatrix getMagFieldMfm(const XsDeviceId& deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getMagFieldMfm(this, &returnValue, &deviceId);
	}

	inline XsMatrix getGeoSelMagFieldMfmModel(const XsDeviceId& deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getGeoSelMagFieldMfmModel(this, &returnValue, &deviceId);
	}

	inline XsVector getHardIronCompensation(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getHardIronCompensation(this, &returnValue, &deviceId);
	}

	inline XsMatrix getSoftIronCompensation(const XsDeviceId& deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getSoftIronCompensation(this, &returnValue, &deviceId);
	}

	inline XsVector getGeoSelMfm(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getGeoSelMfm(this, &returnValue, &deviceId);
	}

	inline XsMatrix getGeoSelMagFieldMeas(const XsDeviceId& deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getGeoSelMagFieldMeas(this, &returnValue, &deviceId);
	}

	inline XsMatrix getGeoSelMagFieldMfm(const XsDeviceId& deviceId) const
	{
		XsMatrix returnValue;
		return *XsMfm_getGeoSelMagFieldMfm(this, &returnValue, &deviceId);
	}

	inline XsVector getNormMagFieldMeas(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormMagFieldMeas(this, &returnValue, &deviceId);
	}

	inline XsVector getNormGeoSelMagFieldMeas(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormGeoSelMagFieldMeas(this, &returnValue, &deviceId);
	}

	inline XsVector getNormMagFieldMfm(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormMagFieldMfm(this, &returnValue, &deviceId);
	}

	inline XsVector getHistResidualsModel(void) const
	{
		XsVector returnValue;
		return *XsMfm_getHistResidualsModel(this, &returnValue);
	}

	inline XsVector getNormalizedHistResidualVertical(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualVertical(this, &returnValue, &deviceId);
	}

	inline XsVector getNormalizedHistResidualDipAngle(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualDipAngle(this, &returnValue, &deviceId);
	}

	inline XsVector getNormalizedHistResidualMagnetic(const XsDeviceId& deviceId) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualMagnetic(this, &returnValue, &deviceId);
	}

	inline XsVector getHistResidualsModelBins(void) const
	{
		XsVector returnValue;
		return *XsMfm_getHistResidualsModelBins(this, &returnValue);
	}

	inline XsVector getNormalizedHistResidualBins(void) const
	{
		XsVector returnValue;
		return *XsMfm_getNormalizedHistResidualBins(this, &returnValue);
	}

	inline XsReal getStdDevNormGeoSelMagFieldMeas(const XsDeviceId& deviceId) const
	{
		return XsMfm_getStdDevNormGeoSelMagFieldMeas(this, &deviceId);
	}

	inline XsReal getAvgValNormGeoSelMagFieldMeas(const XsDeviceId& deviceId) const
	{
		return XsMfm_getAvgValNormGeoSelMagFieldMeas(this, &deviceId);
	}

	inline XsReal getMaxErrorValNormGeoSelMagFieldMeas(const XsDeviceId& deviceId) const
	{
		return XsMfm_getMaxErrorValNormGeoSelMagFieldMeas(this, &deviceId);
	}

	inline XsReal getStdDevNormGeoSelMagFieldMfm(const XsDeviceId& deviceId) const
	{
		return XsMfm_getStdDevNormGeoSelMagFieldMfm(this, &deviceId);
	}

	inline XsReal getAvgValNormGeoSelMagFieldMfm(const XsDeviceId& deviceId) const
	{
		return XsMfm_getAvgValNormGeoSelMagFieldMfm(this, &deviceId);
	}

	inline XsReal getMaxErrorValNormGeoSelMagFieldMfm(const XsDeviceId& deviceId) const
	{
		return XsMfm_getMaxErrorValNormGeoSelMagFieldMfm(this, &deviceId);
	}

	//! \brief Destructor, calls destruct() function to clean up object
	~XsMfm()
	{
		XsMfm_destruct(this);
	}

	//! \brief overloaded delete operator to allow user to use delete instead of calling destruct() function
	void operator delete (void*)
	{
	}

private:
	XsMfm(); // Default constructor not implemented to prevent faulty memory allocation, use construct function instead
#ifndef SWIG
	void* operator new (size_t); //!< \brief new operator not implemented to prevent faulty memory allocation by user, use construct() function instead
	void* operator new[] (size_t); //!< \brief array new operator not implemented to prevent faulty memory allocation by user, use construct() function instead
	void operator delete[] (void*); //!< \brief array delete operator not implemented to prevent faulty memory deallocation by user, use destruct() function instead
#endif
};
#endif // __cplusplus
#endif // GUARD_XSMFM_H_TMP
