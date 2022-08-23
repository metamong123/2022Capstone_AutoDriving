
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


#include "custom_channel.h"
#include <string>
#include <thread>
#include <mutex>
#include <xstypes/xstime.h>
#include <iostream>

#define CHANNELID 1
#define DEFAULT_TIMEOUT 1000

/* Very simple serial port class for Windows platforms
*/
class SerialPort
{
private:
#ifdef _WIN32
	HANDLE m_handle;
#endif
	std::mutex m_mutex;

public:
	/* Constructs a serial port. If successfull the port is immediately opened
	\param portName: The name of the serial port (e.g. "COM1")
	\param baudrate: The baudrate at which to open the serial port
	*/
	SerialPort(XsString const& portName, int baudrate)
	{
#ifdef _WIN32
		XsString fullPortName("\\\\.\\");
		fullPortName.append(portName);
		m_handle = CreateFile(fullPortName.toStdWString().c_str(), GENERIC_READ|GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

		if (m_handle == INVALID_HANDLE_VALUE)
			throw("ERROR: Could not open COM port");

		// set timeouts
		COMMTIMEOUTS cto = { MAXDWORD, 0, 0, 0, 0};
		DCB dcb;
		if(!SetCommTimeouts(m_handle, &cto))
		{
			SerialPort::~SerialPort();
			throw("ERROR: Could not set com port time-outs");
		}

		// set DCB
		memset(&dcb,0,sizeof(dcb));
		dcb.DCBlength = sizeof(dcb);
		dcb.BaudRate = baudrate;
		dcb.fBinary = 1;
		dcb.fDtrControl = DTR_CONTROL_ENABLE;
		dcb.fRtsControl = RTS_CONTROL_ENABLE;

		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;
		dcb.ByteSize = 8;

		if(!SetCommState(m_handle, &dcb))
		{
			SerialPort::~SerialPort();
			throw("ERROR: Could not set com port parameters");
		}
#else
		(void)portName; (void)baudrate;
#endif
	}

	~SerialPort()
	{
#ifdef _WIN32
		CloseHandle(m_handle);
#endif
	}

	/* Writes data to the serial port
	\param buffer: The bytes to write to the serial port
	\param bufferSize: The number of bytes to write
	\returns The actual number of bytes written to the serial port
	*/
	XsSize write(uint8_t const* buffer, XsSize bufferSize)
	{
#ifdef _WIN32
		std::lock_guard<std::mutex> guard(m_mutex);
		DWORD numWritten;
		WriteFile(m_handle, buffer, (DWORD)bufferSize, &numWritten, NULL);
		return numWritten;
#else
		(void)buffer; (void)bufferSize;
		return 0;
#endif
	}

	/* Reads data from the serial port
	\param buffer: Buffer in which to read received bytes
	\param bufferSize: The maximum number of bytes to read
	\returns The actual number of bytes read from the serial port
	*/
	XsSize read(uint8_t *buffer, XsSize bufferSize)
	{
#ifdef _WIN32
		std::lock_guard<std::mutex> guard(m_mutex);
		DWORD numRead;
		if (!ReadFile(m_handle, buffer, (DWORD)bufferSize, &numRead, NULL))
			return 0;
		return numRead;
#else
		(void)buffer; (void)bufferSize;
		return 0;
#endif
	}

	static bool platformSupported()
	{
#ifdef _WIN32
		return true;
#else
		return false;
#endif
	}
};


/* MFM callback handler used by the custom channel
*/
class CustomChannelCallback : public XsMfmCallback
{
private:
	SerialPort* m_customPort;
public:
	CustomChannelCallback(SerialPort* customPort)
		: m_customPort(customPort)
	{
	}

	virtual ~CustomChannelCallback() throw()
	{
	}

	/* The MFM library will call this function to ask the user to transport the given data over
	the custom channel indicated by \a channelId
	*/
	void onTransmissionRequest(int channelId, const XsByteArray* data) override
	{
		(void)channelId;
		m_customPort->write(data->data(), data->size());
	}
};

/* Thread that reads data from the serial port
*/
class SerialReadThread
{
private:
	SerialPort* m_customPort;
	XsMfm* m_mfm;
	std::thread m_thread;
	bool m_running;

public:
	/* Creates and starts a serial reader thread
	\param customPort: The serial port this thread must read data from
	\param xda: The XsControl object to which received data must forwarded
	*/
	SerialReadThread(SerialPort* customPort, XsMfm* mfm)
		: m_customPort(customPort)
		, m_mfm(mfm)
		, m_running(true)
	{
		m_thread = std::thread(&SerialReadThread::run, this);
	}

	~SerialReadThread()
	{
		if (m_running)
			stop();
	}


	/* The function that runs in the thread performing the actual read operation
	*/
	void run()
	{
		while(m_running)
		{
			uint8_t temp[512];
			XsSize c = m_customPort->read(temp, sizeof(temp));
			if (c > 0)
			{
				XsByteArray bar(temp, c, XSDF_None);
				m_mfm->transmissionReceived(CHANNELID, bar);
			}
			XsTime::msleep(10);
		}
	}

	/* Stops the reader thread
	*/
	void stop()
	{
		m_running = false;
		m_thread.join();
	}
};

struct CustomChannelPrivate
{
	SerialPort* m_serialPort;
	SerialReadThread* m_reader;
	CustomChannelCallback* m_callback;

	CustomChannelPrivate()
		: m_serialPort(0)
		, m_reader(0)
		, m_callback(0)
	{}

	~CustomChannelPrivate()
	{
		if (m_reader)
			delete m_reader;

		if (m_serialPort)
			delete m_serialPort;

		if (m_callback)
			delete m_callback;
	}
};

CustomChannel::CustomChannel(XsMfm* mfm)
	: m_mfm(mfm)
	, d(new CustomChannelPrivate)
{
}

CustomChannel::~CustomChannel()
{
	delete d;
}

void CustomChannel::setup()
{
	//Asking the user for input
	std::string portName;
	std::cout << "Enter serial port to open (e.g. COM3): ";
	std::cin >> portName;
	std::cin.ignore(1000,'\n');
	int baudrate;
	std::cout << "Enter baudrate to use (e.g. 115200): ";
	std::cin >> baudrate;
	std::cin.ignore(1000,'\n');

	//Setting up the custom channel
	try
	{
		d->m_serialPort = new SerialPort(XsString(portName), baudrate);
		d->m_callback = new CustomChannelCallback(d->m_serialPort);
		m_mfm->addCallbackHandler(d->m_callback);
		d->m_reader = new SerialReadThread(d->m_serialPort, m_mfm);
	}
	catch(...)
	{
	}
}

void CustomChannel::cleanup()
{
	if (d->m_reader)
		d->m_reader->stop();
}

int CustomChannel::channelId() const
{
	return CHANNELID;
}

uint32_t CustomChannel::defaultTimeout() const
{
	return DEFAULT_TIMEOUT;
}

bool CustomChannel::platformSupported()
{
	return SerialPort::platformSupported();
}
