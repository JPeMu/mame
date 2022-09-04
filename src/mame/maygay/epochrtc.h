// license:BSD-3-Clause
// copyright-holders:Tony Friery

#ifndef MAME_MAYGAY_EPOCHRTC_H
#define MAME_MAYGAY_EPOCHRTC_H

#pragma once


#include "emu.h"

class device_epoch_rtc
{
    public:
        device_epoch_rtc(int gate_id);
        virtual ~device_epoch_rtc();

        void reset(void);
        void write(bool clock, bool data);
        uint8_t get_port_8(void);
        uint8_t get_port_a(void);

    private:
        int     m_ReplySize;            // Number of bytes in reply
        int     m_ReplyOffset;          // Current Reply Byte
        uint8_t m_ReplyBuffer[128];     // Reply Buffer
        uint8_t m_OutputChar;           // Character being Output to CPU
        uint8_t m_InputChar;            // Character being Read from CPU
        int     m_OutBitCount;          // Bit Count of Output
        int     m_InBitCount;           // Bit Count of Input
        int     m_GateID;               // ID of the ASIC Gate
        bool    m_LastClock;            // Previous Hitachi Clock State
        bool    m_LastData;             // Previous Hitachi Data State
        bool    m_RTCClock;             // Current RTC Clock State
        bool    m_RTCData;              // Current RTC Data State
        bool    m_SendReceive;          // Send/Receive Mode (false = RTC receiving)

        void    processCommand(void);   // Process currently received command        
};

#endif // MAME_MAYGAY_EPOCHRTC_H