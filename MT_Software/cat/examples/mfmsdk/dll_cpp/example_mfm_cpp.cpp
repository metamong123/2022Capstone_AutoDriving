
//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

//--------------------------------------------------------------------------------
// Xsens device API example for an MTi / MTx / Mtmk4 device using the C++ API
//
//--------------------------------------------------------------------------------
#include <xstypes/xsexception.h>
#include <magfieldmapper.h> // The Xsens magfieldmapper
#include <iostream>
#include <list>
#include <iomanip>
#include <stdexcept>
#include <xstypes/xstime.h>
#include "custom_channel.h"
#include <atomic>

//--------------------------------------------------------------------------------
// CallbackHandler is an object derived from XsMfmCallback that can be attached to
// an XsMfm using the XsMfm::setCallbackHandler method.
// Various virtual methods which are automatically called by the XsMfm can be
// overridden (See XsCallback)
// This example shows how onScanDone() and onMfmDone() may be used.
// Note that this method will be called from within the thread of the XsMfm.
// Proper synchronization is required when accessing the data (omitted here).
// It is recommended to keep the implementation of these methods fast; therefore
// the only action here is to copy the packet to a queue where it can be
// retrieved later by the main thread to be displayed.
//--------------------------------------------------------------------------------
class CallbackHandler : public XsMfmCallback
{

public:
	CallbackHandler() : m_result(MRV_Processing), m_hasDevice(false), m_mfmDone(false), m_scanDone(false) { }
	XsDeviceId devId() { return m_devId; }
	bool hasDevice() { return m_hasDevice; }
	bool mfmDone() { return m_mfmDone; }
	bool scanDone() { return m_scanDone; }
	XsMfmResultValue result() { return m_result; }
	void setDevId(XsDeviceId devId)
	{
		m_devId = devId;
	}
protected:
	virtual void onScanDone(const XsDeviceIdArray* devices)
	{
		if (devices->size()>0)
		{
			m_devId = devices->at(0);
			m_hasDevice = true;
		}
		m_scanDone = true;
	}

	virtual void onMfmDone(XsDeviceId dev, XsMfmResultValue resValue)
	{
		if (dev.toInt() == m_devId.toInt())
			m_result = resValue;
		m_mfmDone = true;
	}

	virtual void onTransmissionRequest(int channelId, const XsByteArray* data)
	{
		(void)channelId; (void) data;
	}

private:
	XsDeviceId m_devId;
	XsMfmResultValue m_result;
	bool m_hasDevice;
	volatile std::atomic_bool m_mfmDone;
	volatile std::atomic_bool m_scanDone;
};

//--------------------------------------------------------------------------------
int main(void)
{
	XsMfm *mfm = XsMfm::construct();
	// create callbackhandler
	CallbackHandler callback;
	mfm->addCallbackHandler(&callback);

	CustomChannel* customChannel = 0;

	std::cout << "MFM Options:" << std::endl;
	std::cout << "  f: From file" << std::endl;
	std::cout << "  l: Live (default)" << std::endl;
	if (CustomChannel::platformSupported())
		std::cout << "  c: Live using a custom port" << std::endl;
	std::cout << "Enter option: " << std::endl;

	std::string answer;
	std::cin >> answer;
	std::cin.ignore(1000,'\n');
	XsDeviceIdArray deviceIds;
	int N=0;
	bool live = true;
	if (answer == "f")
	{
		live = false;
		std::string filename;
		std::cout << "Enter filename:" << std::endl;
		std::cin >> filename;
		std::cin.ignore(1000,'\n');
		XsDeviceIdArray deviceIdsGiven;
		XsString inputfile;
		inputfile = filename;
		try
		{
			//load filename, function gives loaded deviceIds back
			mfm->loadInputFile(inputfile, deviceIdsGiven, deviceIds);
			if (!deviceIds.empty())
			{
				callback.setDevId(deviceIds[0]);
			}
			else
			{
				std::cout << "No valid ID, press enter to close" << std::endl;
				std::cin.get();
				std::cout << "Closing" << std::endl;
				return 1;
			}
		}
		catch (XsException&)
		{
			std::cout << "Could not read file, press enter to close" << std::endl;
			std::cin.get();
			std::cout << "Closing" << std::endl;
			return 1;
		}
	}
	else
	{
		if (answer == "c" && CustomChannel::platformSupported())
		{
			customChannel = new CustomChannel(mfm);
			customChannel->setup();
			mfm->scanMfmDeviceOnCustomChannel(customChannel->channelId(), customChannel->defaultTimeout(), true);
			std::cout << "Scanning for device on custom channel..." << std::endl;
		}
		else
		{
			//scan for devices, scan calls callback when done
			std::cout << "Scanning for devices..." << std::endl;
			mfm->scanMfmDevices();
		}

		while (!callback.scanDone())
			N++;

		// check if a device is found
		if (!callback.hasDevice())
		{
			std::cout << "Did not find a sensor, press enter to close" << std::endl;
			std::cin.get();
			std::cout << "Closing" << std::endl;
			return 1;
		}

		std::cout << "Device found: " << std::hex << callback.devId().toInt() << "." << std::endl;
		deviceIds.push_back(callback.devId());

		//start logging
		std::cout << "Start logging" << std::endl;
		mfm->startLogging(deviceIds[0], "logfile.mtb");

		//stop logging
		std::cout << "Press enter to stop logging" << std::endl;
		std::cin.get();
		mfm->stopLogging(deviceIds[0]);
		std::cout << "Stop logging" << std::endl;
	}
	assert(!deviceIds.empty());
	// start processing of the magfieldmapper, startProcessing calls callback when done
	std::cout << "Start processing" << std::endl;
	bool startProcessing = mfm->startProcessing(deviceIds[0]);
	if (startProcessing)
	{
		N=0;
		while (!callback.mfmDone())
			N++;

		//callback also gets result of mfm
		if (callback.result() != MRV_Failed)
		{
			std::cout << "MFM succeeded" << std::endl;

			// write file
			if (!live)
				std::cout << "Write to file (f) or do not write (d, default)" << std::endl;
			else
				std::cout << "Write to file (f), sensor (s) or do not write (d, default)" << std::endl;
			std::cin >> answer;
			std::cin.ignore(1000,'\n');
			if (answer=="f")
			{
				if (mfm->writeResultToFile(deviceIds[0], "MFM_output.bin"))
					std::cout << "Written to file" << std::endl;
				else
					std::cout << "Write to file failed" << std::endl;
			}
			else if (answer=="s")
			{
				if (mfm->writeResultToMt(deviceIds[0]))
					std::cout << "Written to sensor" << std::endl;
				else
					std::cout << "Write to sensor failed" << std::endl;
			}
		}
		else
			std::cout << "MFM failed" << std::endl;
	}
	else
	{
		std::cout << "MFM failed, processing did not start" << std::endl;
	}

	//close application
	std::cout << "Press enter to close window" << std::endl;
	std::cin.get();
	std::cout << "Closing" << std::endl;

	if (customChannel)
	{
		customChannel->cleanup();
		delete customChannel;
	}
}
