#include "epochrtc.h"

///////////////////////////////////////////////////////////////////////
//
//		Implementation of RTC Class
//
///////////////////////////////////////////////////////////////////////

void device_epoch_rtc::processCommand(void)
{
	uint8_t cmd = m_InputChar;

	switch (cmd)
	{
		case 0xfd:	// Read Clock Command
			m_ReplyBuffer[0] = 0x00;	// Seconds
			m_ReplyBuffer[1] = 0x00; // Minutes
			m_ReplyBuffer[2] = 0x00; // Hours
			m_ReplyBuffer[3] = 0x01; // Day
			m_ReplyBuffer[4] = 0x01; // MSN: DOW, LSN: Month
			m_ReplyBuffer[5] = 0x00; // Year
			m_ReplySize = 6;
			m_ReplyOffset = -1;
			m_OutBitCount = 0;
			m_SendReceive = true;	// Now sending back to CPU
			break;
		case 0xfe: 	// Set Clock Command
		case 0xfb:	// Set Security Output 2 Params
		case 0xfc:	// Set Security Output 1 Params
		case 0xfa:	// Read Security Output 1
		case 0xf9:	// Read Security Output 2
		default:	// Unknown Command
			break;
	}
}

void device_epoch_rtc::write(bool clock, bool data)
{
	// If the data line from the H8 CPU has changed, store
	// the new value
	if (m_LastData |= data)
	{
		m_LastData = data;
	}

	// H8 CPU controls the clocking in/out of data to/from the PIC
	// Check if the clock state has changed
	if (clock != m_LastClock)
	{
		// Store the new clock state
		m_LastClock = clock;

		// Check the current (expected) comms direction
		if (!m_SendReceive)
		{
			// RTC is currently receiving from CPU
			if (clock)
			{
				// CPU signalling it is ready to send next bit
				// (on rising clock edge) so acknowledge by
				// raising the RTC Clock line back to the CPU
				m_RTCClock = true;
			}
			else
			{
				// CPU Signalling that Data Line is correctly asserted,
				// so read the data bit
				m_InputChar <<= 1;
				m_InputChar &= 0xfe;

				if (data)
				{
					m_InputChar |= 0x01;
				}

				m_InBitCount++;
				if (m_InBitCount == 8)
				{
					// We have a valid command or data byte
					m_InBitCount = 0;
					processCommand();
				}

				// Finally, acknowledge the read by dropping
				// the RTC Clock line back to the CPU
				m_RTCClock = false;
			}
		}
		else
		{
			// RTC is currently sending to CPU
			if (clock)
			{
				// CPU has acknowledged Ready by raising
				// its clock line - Set next bit
				if (m_OutBitCount == 0)
				{
					// Obtain next character
					m_OutputChar = m_ReplyBuffer[m_ReplyOffset];
					m_ReplyOffset++;
				}

				m_RTCData = (m_OutputChar & 0x80) ? true : false;

				// and indicate we are ready
				m_RTCClock = false;
			}
			else
			{
				// CPU has confirmed receipt of bit
				m_OutBitCount++;
				m_OutputChar <<= 1;

				if (m_OutBitCount == 8)
				{
					// All bits delivered
					m_OutBitCount = 0;

					// Check if any more bytes remain
					if (m_ReplyOffset < m_ReplySize)
					{
						// Yes, there's more, so raise the clock
						// to let the CPU know
						m_RTCClock = true;
					}
					else
					{
						// Return to receive mode
						m_SendReceive = false;
					}
				}
				else
				{
					// Indicate that we're ready for the next bit
					m_RTCClock = true;
				}
			}
		}
	}
}

uint8_t device_epoch_rtc::get_port_8(void)
{
	if (m_GateID < 3)
	{
		return (m_RTCData ? 0x10 : 0x00);
	}

	return (m_RTCData ? 0x04 : 0x00);
}

uint8_t device_epoch_rtc::get_port_a(void)
{
	uint8_t rv = 0x00;

	// Set correct lines in response
	rv |= m_RTCClock 		? 0x08 : 0x00;
	rv |= m_LastClock	? 0x02 : 0x00;
	rv |= m_LastData 	? 0x04 : 0x00;

	if ((m_ReplyOffset == -1) && (m_SendReceive == true))
	{
		// Special case: Make clock high as we're about to enter
		// reply mode
		m_RTCClock = true;
		m_ReplyOffset = 0;
	}

	return rv;
}

///////////////////////////////////////////////////////////////////////
//
//  FUNCTION:	device_epoch_rtc::Reset
//
//  PURPOSE:	(POR) Reset the emulated RTC Hardware
//
//  INPUTS:		void
//
//  OUTPUT:		void
//
///////////////////////////////////////////////////////////////////////
void device_epoch_rtc::reset()
{
	// Set initial state lines
	m_RTCClock = false;
	m_RTCData = false;
	m_LastClock = false;
	m_LastData = false;
	m_OutBitCount = 0;
	m_InBitCount = 0;
	m_SendReceive = false;
	m_ReplySize = 0;
	m_ReplyOffset = 0;

	for (int i = 0; i < 128; i++)
	{
		m_ReplyBuffer[i] = 0x00;
	}
}

///////////////////////////////////////////////////////////////////////
//
//  FUNCTION:	device_epoch_rtc::device_epoch_rtc
//
//  PURPOSE:	Constructor for device_epoch_rtc Class
//
//  INPUTS:		int						ID of ASIC Gate (Version)
//
//  OUTPUT:		(none)
//
///////////////////////////////////////////////////////////////////////
device_epoch_rtc::device_epoch_rtc(int gateID)
{
    m_GateID = gateID;
	reset();
}

///////////////////////////////////////////////////////////////////////
//
//  FUNCTION:	device_epoch_rtc::~device_epoch_rtc
//
//  PURPOSE:	Destructor for device_epoch_rtc Class
//
//  INPUTS:		(none)
//
//  OUTPUT:		(none)
//
///////////////////////////////////////////////////////////////////////
device_epoch_rtc::~device_epoch_rtc()
{
}
