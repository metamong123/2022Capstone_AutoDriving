
#  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
#  All rights reserved.
#  
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#  
#  1.	Redistributions of source code must retain the above copyright notice,
#  	this list of conditions, and the following disclaimer.
#  
#  2.	Redistributions in binary form must reproduce the above copyright notice,
#  	this list of conditions, and the following disclaimer in the documentation
#  	and/or other materials provided with the distribution.
#  
#  3.	Neither the names of the copyright holders nor the names of their contributors
#  	may be used to endorse or promote products derived from this software without
#  	specific prior written permission.
#  
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
#  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
#  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
#  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
#  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
#  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
#  

import xsensdeviceapi as xda
import time
from threading import Lock


class XdaCallback(xda.XsCallback):
    def __init__(self):
        xda.XsCallback.__init__(self)
        self.m_progress = 0
        self.m_lock = Lock()

    def progress(self):
        return self.m_progress

    def onProgressUpdated(self, dev, current, total, identifier):
        self.m_lock.acquire()
        self.m_progress = current
        self.m_lock.release()


if __name__ == '__main__':
    print("Creating XsControl object...")
    control = xda.XsControl_construct()
    assert(control is not 0)

    xdaVersion = xda.XsVersion()
    xda.xdaVersion(xdaVersion)
    print("Using XDA version %s" % xdaVersion.toXsString())

    try:
        print("Opening log file...")
        logfileName = "logfile.mtb"
        if not control.openLogFile(logfileName):
            raise RuntimeError("Failed to open log file. Aborting.")
        print("Opened log file: %s" % logfileName)

        deviceIdArray = control.mainDeviceIds()
        for i in range(deviceIdArray.size()):
            if deviceIdArray[i].isMti() or deviceIdArray[i].isMtig():
                mtDevice = deviceIdArray[i]
                break

        if not mtDevice:
            raise RuntimeError("No MTi device found. Aborting.")

        # Get the device object
        device = control.device(mtDevice)
        assert(device is not 0)

        print("Device: %s, with ID: %s found in file" % (device.productCode(), device.deviceId().toXsString()))

        # Create and attach callback handler to device
        callback = XdaCallback()
        device.addCallbackHandler(callback)

        # By default XDA does not retain data for reading it back.
        # By enabling this option XDA keeps the buffered data in a cache so it can be accessed 
        # through XsDevice::getDataPacketByIndex or XsDevice::takeFirstDataPacketInQueue
        device.setOptions(xda.XSO_RetainBufferedData, xda.XSO_None);

        # Load the log file and wait until it is loaded
        # Wait for logfile to be fully loaded, there are three ways to do this:
        # - callback: Demonstrated here, which has loading progress information
        # - waitForLoadLogFileDone: Blocking function, returning when file is loaded
        # - isLoadLogFileInProgress: Query function, used to query the device if the loading is done
        #
        # The callback option is used here.

        print("Loading the file...")
        device.loadLogFile()
        while callback.progress() != 100:
            time.sleep(0)
        print("File is fully loaded")


        # Get total number of samples
        packetCount = device.getDataPacketCount()

        # Export the data
        print("Exporting the data...")
        s = ''
        index = 0
        while index < packetCount:
            # Retrieve a packet
            packet = device.getDataPacketByIndex(index)

            if packet.containsCalibratedData():
                acc = packet.calibratedAcceleration()
                s += "Acc X: %.2f" % acc[0] + ", Acc Y: %.2f" % acc[1] + ", Acc Z: %.2f" % acc[2]

                gyr = packet.calibratedGyroscopeData()
                s += " |Gyr X: %.2f" % gyr[0] + ", Gyr Y: %.2f" % gyr[1] + ", Gyr Z: %.2f" % gyr[2]

                mag = packet.calibratedMagneticField()
                s += " |Mag X: %.2f" % mag[0] + ", Mag Y: %.2f" % mag[1] + ", Mag Z: %.2f" % mag[2]

            if packet.containsOrientation():
                quaternion = packet.orientationQuaternion()
                s += "q0: %.2f" % quaternion[0] + ", q1: %.2f" % quaternion[1] + ", q2: %.2f" % quaternion[2] + ", q3: %.2f " % quaternion[3]

                euler = packet.orientationEuler()
                s += " |Roll: %.2f" % euler.x() + ", Pitch: %.2f" % euler.y() + ", Yaw: %.2f " % euler.z()

            if packet.containsLatitudeLongitude():
                latlon = packet.latitudeLongitude()
                s += " |Lat: %7.2f" % latlon[0] + ", Lon: %7.2f " % latlon[1]

            if packet.containsAltitude():
                s += " |Alt: %7.2f " % packet.altitude()

            if packet.containsVelocity():
                vel = packet.velocity(xda.XDI_CoordSysEnu)
                s += " |E: %7.2f" % vel[0] + ", N: %7.2f" % vel[1] + ", U: %7.2f " % vel[2]

            s += "\n"

            index += 1

        exportFileName = "exportfile.txt"
        with open(exportFileName, "w") as outfile:
            outfile.write(s)
        print("File is exported to: %s" % exportFileName)

        print("Removing callback handler...")
        device.removeCallbackHandler(callback)

        print("Closing XsControl object...")
        control.close()

    except RuntimeError as error:
        print(error)
    except:
        print("An unknown fatal error has occured. Aborting.")
    else:
        print("Successful exit.")
