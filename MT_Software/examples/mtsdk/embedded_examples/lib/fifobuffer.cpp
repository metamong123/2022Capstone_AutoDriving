
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

#include "fifobuffer.h"

/*!	\class FifoBuffer
	\brief First in, first out (fifo) container for bytes
*/


/*!	\brief Constructs a FifoBuffer
*/
FifoBuffer::FifoBuffer()
{
	clear();
}


/*!	\brief Clears the buffer
*/
void FifoBuffer::clear()
{
	m_idxRead = 0;
	m_idxWrite = 0;
	m_nofBytes = 0;
}


/*!	\brief Insert a byte in the buffer
	\param[in] byte The byte to insert
*/
void FifoBuffer::insertByte(uint8_t byte)
{
	if (m_nofBytes == m_bufferSize)
		return;

	m_buffer[m_idxWrite] = byte;
	if (++m_idxWrite == m_bufferSize)
		m_idxWrite = 0;
	m_nofBytes++;
}


/*!	\brief Extract a byte from the buffer
	\param[out] byte Reference for receiving the byte
	\return true if a byte was extracted, false otherwise
*/
bool FifoBuffer::extractByte(uint8_t& byte)
{
	if (m_nofBytes == 0)
		return false;

	byte = m_buffer[m_idxRead];
	if (++m_idxRead == m_bufferSize)
		m_idxRead = 0;
	m_nofBytes--;
	return true;
}


/*!	\brief Returns the number of bytes in the buffer
*/
int FifoBuffer::nofBytes() const
{
	return m_nofBytes;
}

