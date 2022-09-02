// license:BSD-3-Clause
// copyright-holders:David Haywood, Tony Friery
/************************************************************************************************************

    Maygay EPOCH hardware
     'mechanical' fruit machine HW
     primary use 1997 - 2003

    HD6413002FN16 (H8-3002) CPU

    YMZ280B sound

    2x PICs for security
    the PIC marked 'Security' is apparently the RTC, and doesn't change
    the other is per game / manufacturer and provides data needed for booting?

    The majority of system tasks are handled by a custom ASIC which is capable
    of reading and writing directly from/to the main RAM.  This includes handling of
    the inputs, and lamps / LEDs

    -------------------------------------------------
    MANY GAMES DO NOT HAVE ANY SOUND ROMS DUMPED
    -------------------------------------------------
    SOME OF THESE GAMES USED HDDS, ONLY ONE IS DUMPED
    -------------------------------------------------

    Even where we have sound rom dumps they probably shouldn't be trusted, the
    source of them had many badly dumped Scorpion 4 sound roms (0x080000 bytes where
    they should be double that) and many of these sound roms are that size...

    Some games have EPOCH NEVADA in the header, others have EPOCH APOLLO, is
    there a difference?

    TECHNICAL INFORMATION
    =====================

    There is support for a total of 512 Lamps with up to 6 independent flash timers and
    8 steps of dimming controlled globally.

    There is support for a total of 512 LED Segments, again with up to 6 independent
    flash timers and 8 steps of dimming controlled globally.

    The Gate Array generates a pair of timer interrupts (when enabled). A 6.4kHz
    "refresh" timer, and a 100Hz sync timer. All of these are presented to the CPU
    using external IRQ5 (vector 17)

    MEMORY MAP
    ==========

    -- Note: Where bitmaps are shown, these are in MSB -> LSB order

    FE0000 - FE07FF = RAM (in Gate Array or on-board?)

    FE0800 - FE09FF = Lamp Array
      512 8-bit Lamp Values. Each lamp byte:
        x... ....       PHASEBIT                    Flash Phase Bit
                                                    (Inverts the "on" state relative to
                                                    other lamps in the same flash group)
        .x.. ....       DIMBIT                      Dim Enable Bit
        .... xxx.                                   Flash Group ID
        .... ...x                                   Lamp On

    FE0A00 - FE0BFF = LED Array
      512 8-bit Lamp Values. Each lamp byte:
        x... ....       PHASEBIT                    Flash Phase Bit
                                                    (Inverts the "on" state relative to
                                                    other LEDs in the same flash group)
        .x.. ....       DIMBIT                      Dim Enable Bit
        .... xxx.                                   Flash Group ID
        .... ...x                                   LED On
    
    FE0C00 - FE0DFF = Input Array
     R-  FE0C02 = Coin Mech + Stake Key
     R-  FE0C03 = Stake + Percent Key
     R-  FE0C04 = Reel Optos
     R-  FE0C05 = Hopper Returns, Meter SW, Note Alarm

    FE0E00 - FE0FFF = Input Interrupt Enable Array

    FE1000 - FE121B = Misc Outputs
     -W  FE1000 = Alpha Data
     -W  FE1001 = Alpha Control + Meter/Mech Lamps
     -W  FE1002 = Coin Mech
     -W  FE1003 = Coin Diverts
     -W  FE1004 = Reel 1+2 Outputs
     -W  FE1005 = Reel 3+4 Outputs
     -W  FE1006 = Note Acceptor + Meters?
     -W  FE1007 = HopperDrive

    ------------------------------------------------------------------------------------
    FE1200 - FE120B = Lamp Flash-Rate Timer Area
        Each timer is 16-bits in size, giving a total of 6 timers

    FE120C - FE1217 = LED Flash-Rate Timer Area
        Each timer is 16-bits in size, giving a total of 6 timers

    FE1218 - FE1218 = Lamp Dimmer Control
        ???? ?xxx       Only the lowest 3 bits are sane. Assume other bits must be 0

    FE1219 - FE1219 = LED Dimmer Control
        ???? ?xxx       Only the lowest 3 bits are sane. Assume other bits must be 0

    FE121A - FE121B = Option Switches

    FE121C - FEFFFF = RAM? NVRAM even?

    FFFC00 - FFFC00 = Read/Write YMZ280 Register Number
    FFFC02 - FFFC02 = Read/Write YMZ280 Register Data

    ==================================================================================
    SYSTEM REGISTERS
    (on Gate Array)
    ==================================================================================

    FFFF10 - FFFF10 = (EPSYSSTT)                System Status Register
        .x.. ....       ENBACC                      Primary uP Access to Emulator RAM
                                                    Enabled
        .... x...       GEMODE                      Game Memory Mode Selection

    FFFF11 - FFFF11 = (EPSYSCTL)                System Status/Control Register
        x... ....       MASRES                      Force Primary, Boot and Security Reset 
        .x.. ....       TRGRES                      Force Primary and Boot Reset
        .... ..x.                                   Status LED

    FFFF12 - FFFF12 = (EPIOCTL)                 I/O Control Register
    

    FFFF13 - FFFF13 = (EPSYSENB)                System Enable Register
        x... ....       EMDSYN                      Force Matrix Display Sync
        .x.. ....       EMD2PL                      2nd Matrix Plane Enable
        ..x. ....       EMDMLT                      Multi-Plane Matrix Enable
        .... .x..       EIONLK                      I/O Link Enable Bit
        .... ..x.       ENVRAM                      NVRAM Enable Bit
        .... ...x       WATCHDOG                    Watchdog Bit

    FFFF14 - FFFF14 = (EPINTENB)                Interrupt Enable Register
    FFFF15 - FFFF15 = (EPINTCTL)                Interrupt Status/Control Register
                      (EPINTSTT)        
        x... ....       FMATRIX                     Matrix
        .x.. ....                                   Frame
        ..x. ....                                   Refresh (6.4 kHz)
        ...x ....       FUNCSW                      Function Switch
        .... x...                                   Sync (100 Hz)
        .... .x..                                   I2C
        .... ..x.                                   Audio
        .... ...x                                   Input

    FFFF16 - FFFF16 = (EPPCICTL)                PCI Control Register

    FFFF17 - FFFF17 = (EPPCISTT)                PCI Status Register

    FFFF18 - FFFF18 = (EPI2CCTL)                I2C Status/Control Register
        x... ....       I2CREADY                    Interface Ready Bit
        .x.. ....       I2CERROR                    Error On Interface Bit
        .... ...x       I2CSEL                      Interface Select Bit

    FFFF19 - FFFF19 = (EPI2CDAT)                I2C Data Register
    FFFF1A - FFFF1A = (EPPCIUAR)                PCI Upper Address Register
    FFFF1B - FFFF1B = (EPGATEID)                Gate ID Register

    OTHER DEVICES
    =============

    Real-Time Clock
        H8 Port 8 = RTC Input 
        H8 Port A = RTC Output

************************************************************************************************************/

#include "emu.h"
#include "maygayep.h"
#include "speaker.h"

//#define VERBOSE 1
#include "logmacro.h"

#include "maygayep.lh"

// Interrupt Mask Defines
#define INT_MATRIX 0x80
#define INT_FRAME 0x40
#define INT_REFRESH 0x20
#define INT_FUNCSW 0x10
#define INT_SYNC 0x08
#define INT_I2C 0x04
#define INT_AUDIO 0x02
#define INT_INPUT 0x01


unsigned char ascokitab[64] = {
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,		// 00 - 07
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,		// 08 - 1f
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,		// 10 - 17
	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,		// 18 - 1f
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,		// 20 - 27
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,		// 28 - 2f
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,		// 30 - 37
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f		// 38 - 3f
};

void maygayep_state::machine_start() {
  logerror("maygayep::machine_start\n");
	m_lamps.resolve();
  m_leds.resolve();
}

void maygayep_state::machine_reset() {
  logerror("maygayep::machine_reset\n");
  ep_int_enable = 0;
  ep_int_status = 0;
  ep_sys_enable = 0;
  ep_sys_control = 0;
  ep_sys_status = 0;
  ep_io_control = 0;

  ep_led_dim_level = 0;
  ep_lamp_dim_level = 0;
  for (int i = 0; i < 8; i++) {
    ep_led_timers[i] = 0;
    ep_lamp_timers[i] = 0;
    ep_reel_drives[i] = 0;
  }

  ep_last_read_int_status = 0;
}

INTERRUPT_GEN_MEMBER(maygayep_state::external_interrupt)
{
  device.execute().pulse_input_line(5, device.execute().minimum_quantum_time());
}

// NMI is periodic? or triggered by a write?
TIMER_DEVICE_CALLBACK_MEMBER( maygayep_state::refreshtimer )
{
  if (!(ep_int_status & INT_REFRESH)) {
    ep_int_status |= INT_REFRESH;

    if (ep_int_enable & INT_REFRESH) {
        raise_ext_irq5();
    }
  }
}

TIMER_DEVICE_CALLBACK_MEMBER( maygayep_state::synctimer )
{
  if (!(ep_int_status & INT_SYNC)) {
    ep_int_status |= INT_SYNC;

    if (ep_int_enable & INT_SYNC) {
        raise_ext_irq5();
    }
  }
}

void maygayep_state::raise_ext_irq5(void) {
  // TODO
  logerror("Should be raising INT5!\n");
  m_maincpu->pulse_input_line(5, m_maincpu->minimum_quantum_time());
  
}

// bp 29e58 in ep_simp reads the 'INITIALISE . . .' string
void maygayep_state::maygayep_map(address_map &map) {
  map.global_mask(0xFFFFFF);

  map(0x000000, 0x07FFFF).rom().region("maincpu", 0);
  map(0xFE0000, 0xFE07FF).ram().share("nvram"); // merln at least?

  map(0xFE0800, 0xFE09FF).rw(FUNC(maygayep_state::lamps_read), FUNC(maygayep_state::lamps_write));
  map(0xFE0A00, 0xFE0BFF).rw(FUNC(maygayep_state::leds_read), FUNC(maygayep_state::leds_write));
  map(0xFE0C00, 0xFE0DFF).rw(FUNC(maygayep_state::inputs_read), FUNC(maygayep_state::inputs_write));
  map(0xFE0E00, 0xFE0FFF).rw(FUNC(maygayep_state::input_int_read), FUNC(maygayep_state::input_int_write));

  map(0xFE1000, 0xFE1007).rw(FUNC(maygayep_state::misc_read), FUNC(maygayep_state::misc_write));
  map(0xFE1200, 0xFE1219).rw(FUNC(maygayep_state::fade_dim_read), FUNC(maygayep_state::fade_dim_write));

  map(0xFE121C, 0xFEFFFF).ram(); // ?

  map(0xFFFC00, 0xFFFC02).rw(FUNC(maygayep_state::ymz_read), FUNC(maygayep_state::ymz_write));

  // EPOCH System Registers
  map(0xFFFF10, 0xFFFF10).rw(FUNC(maygayep_state::epsysstt_r), FUNC(maygayep_state::epsysstt_w));
  map(0xFFFF11, 0xFFFF11).rw(FUNC(maygayep_state::epsysctl_r), FUNC(maygayep_state::epsysctl_w));
  map(0xFFFF12, 0xFFFF12).rw(FUNC(maygayep_state::epioctl_r), FUNC(maygayep_state::epioctl_w));
  map(0xFFFF13, 0xFFFF13).rw(FUNC(maygayep_state::epsysenb_r), FUNC(maygayep_state::epsysenb_w));
  map(0xFFFF14, 0xFFFF14).rw(FUNC(maygayep_state::epintenb_r), FUNC(maygayep_state::epintenb_w));
  map(0xFFFF15, 0xFFFF15).rw(FUNC(maygayep_state::epintstt_r), FUNC(maygayep_state::epintctl_w));

}

uint8_t maygayep_state::ymz_read(offs_t addr) {
  uint8_t value = m_ymz->read(addr >> 1);
  logerror("Read %02x from YMZ280 offset %d\n", value, addr);
  return value;
}

void maygayep_state::ymz_write(offs_t addr, uint8_t value) {
  logerror("Wrote %02x to YMZ280 offset %d\n", value, addr);
  m_ymz->write(addr >> 1, value);
}

uint8_t maygayep_state::epsysstt_r(offs_t addr) {
  logerror("Read %02x from EPSYSSTT\n", ep_sys_status);
  return ep_sys_status;
}

uint8_t maygayep_state::epsysctl_r(offs_t addr) {
  logerror("Read from EPSYSCTL\n");
  return ep_sys_control;
}

uint8_t maygayep_state::epioctl_r(offs_t addr) {
  logerror("Read from EPIOCTL\n");
  return ep_io_control;
}

uint8_t maygayep_state::epsysenb_r(offs_t addr) {
  // logerror("Read from EPSYSENB\n");
  return ep_sys_enable;
}

uint8_t maygayep_state::epintenb_r(offs_t addr) {
  return ep_int_enable;
}

uint8_t maygayep_state::epintstt_r(offs_t addr) {
  ep_int_status &= ((~ep_last_read_int_status) | ep_int_enable);
  ep_last_read_int_status = ep_int_status;
  return ep_int_status;
}

void maygayep_state::epsysstt_w(offs_t addr, uint8_t value) {
  logerror("Wrote %02x to EPSYSSTT\n", value);
  ep_sys_status = value;
  m_lamps[1] = (value & 0x02) >> 1;
}

void maygayep_state::epsysctl_w(offs_t addr, uint8_t value) {
  logerror("Wrote %02x to EPSYSCTL\n", value);
  ep_sys_control = value;
}

void maygayep_state::epioctl_w(offs_t addr, uint8_t value) {
  logerror("Wrote %02x to EPIOCTL\n", value);
  ep_io_control = value;
  m_lamps[2] = (value & 0x02) >> 1;
}

void maygayep_state::epsysenb_w(offs_t addr, uint8_t value) {
  // logerror("Wrote %02x to EPSYSENB\n", value);
  ep_sys_enable = value;
}

void maygayep_state::epintenb_w(offs_t addr, uint8_t value) {
  logerror("Wrote %02x to EPINTENB\n", value);
  ep_int_enable = value;
}

/*
  A write to the Interrupt Control register clears the status in the bits supplied
  provided that the bit was last set to 1 when previously read
*/
void maygayep_state::epintctl_w(offs_t addr, uint8_t value) {
  uint8_t masked_bits = ep_last_read_int_status & value;
  ep_int_status &= ~masked_bits;

  logerror("Wrote %02x to EPINTCTL. Clearing: %02x\n", value, masked_bits);
  for (int i = 0; i < 8; i++) {
    m_lamps[128 + i] = BIT(value, i);
  }
}


/*
  There are 512 lamps which can be both read and written
*/
uint8_t maygayep_state::lamps_read(offs_t addr) {
    return m_lamps[addr];
}

void maygayep_state::lamps_write(offs_t addr, uint8_t value) {
  // logerror("Wrote %02x to lamp %d\n", value, addr);
  m_lamps[addr] = value & 1;
}


uint8_t maygayep_state::misc_read(offs_t addr) {
  uint8_t value = 0x00;

  switch(addr) {
    case 4: // Reels 1+2
      value = ep_reel_drives[0] | (ep_reel_drives[1] << 4);
      break;
    case 5: // Reels 3+4
      value = ep_reel_drives[1] | (ep_reel_drives[2] << 4);
      break;
  }

  return value;
}

void maygayep_state::misc_write(offs_t addr, uint8_t value) {
  switch(addr) {
    case 0: // Alpha
      logerror("Alpha output: %02x [%c]\n", value, ascokitab[value & 0x3f]);
      break;
    case 4: // Reels 1+2
      ep_reel_drives[0] = value & 0x0f;
      ep_reel_drives[1] = (value >> 4) & 0x0f;
      reel12_w(value);
      break;
    case 5: // Reels 3+4
      ep_reel_drives[2] = value & 0x0f;
      ep_reel_drives[3] = (value >> 4) & 0x0f;
      reel34_w(value);
      break;
    default:
      logerror("Wrote %02x to MISC %08x\n", value, addr);
      for (int i = 0; i < 8; i++) {
        int base = 128 + ((addr & 0x07) * 8);
        m_lamps[base + i] = BIT(value, i);
      }
  }
}

uint8_t maygayep_state::inputs_read(offs_t addr) {
  if (addr == 4) {
    return ((m_optic_pattern << 4) & 0xf0) | 0x01;
  }

  return 0xff;
}

void maygayep_state::inputs_write(offs_t addr, uint8_t value) {
  // logerror("Wrote %02x to INPUTS %d\n", value, addr);
}

uint8_t maygayep_state::input_int_read(offs_t addr) {
    return 0xff;
}

void maygayep_state::input_int_write(offs_t addr, uint8_t value) {
  logerror("Wrote %02x to INPUT INTERRUPTS %d\n", value, addr);
}

uint8_t maygayep_state::leds_read(offs_t addr) {
    return m_leds[addr];
}

void maygayep_state::leds_write(offs_t addr, uint8_t value) {
  // logerror("Wrote %02x to LED %d\n", value, addr);
  m_leds[addr] = value & 1;
}

/*
  Lamp/LED Flash and Dim control
*/
uint8_t maygayep_state::fade_dim_read(offs_t addr) {
  uint8_t value = 0x00;

  switch(addr) {
    case 0x18: // Lamp Dimmer
      // Only the lowest 3 bits are used
      value = ep_lamp_dim_level;
      break;
    case 0x19: // LED Dimmer
      // Only the lowest 3 bits are used
      value = ep_led_dim_level;
      break;
    default:
      uint16_t timer_value = 0;
      if (addr < 0x0c) {
        // Lamp Flash Timers
        timer_value = ep_lamp_timers[addr >> 1];
      } else {
        // LED Flash Timers
        timer_value = ep_led_timers[addr >> 1];
      }
      value = (addr & 1) ? (timer_value & 0xff00) >> 8 : timer_value & 0xff;
      break;
  }

  return value;
}

void maygayep_state::fade_dim_write(offs_t addr, uint8_t value) {
  // logerror("Wrote %02x to LED %d\n", value, addr);

  switch(addr) {
    case 0x18: // Lamp Dimmer
      // Only the lowest 3 bits are used
      ep_lamp_dim_level = value & 0x07;
      logerror("Set Lamp Dim to %02x\n", ep_lamp_dim_level);
      break;
    case 0x19: // LED Dimmer
      // Only the lowest 3 bits are used
      ep_led_dim_level = value & 0x07;
      logerror("Set LED Dim to %02x\n", ep_led_dim_level);
      break;
    default:
      uint16_t* timer = NULL;
      char timer_name[10];
      if (addr < 0x0c) {
        // Lamp Flash Timers
        uint8_t timer_num = (addr >> 1) & 0x07;
        timer = &ep_lamp_timers[addr >> 1];
        sprintf(timer_name, "LAMP%1d", timer_num);
      } else {
        // LED Flash Timers
        uint8_t timer_num = ((addr - 0x0c) >> 1) & 0x07;
        timer = &ep_led_timers[(addr - 0x0c) >> 1];
        sprintf(timer_name, "LED%1d", timer_num);
      }

      if (timer) {
        if (addr & 1) {
          *timer &= 0xff;
          *timer |= (value << 8) & 0xff;
        } else {
          *timer &= 0xff00;
          *timer |= value & 0xff;
        }

        logerror("Set timer %s to %d\n", timer_name, *timer);
      }
  }
}

void maygayep_state::reel12_w(uint8_t data) {
  // logerror("Wrote %02x to Reels 1/2\n", data);
	m_reels[0]->update( data     & 0x0F);
	m_reels[1]->update((data>>4) & 0x0F);

	awp_draw_reel(machine(),"reel1", *m_reels[0]);
	awp_draw_reel(machine(),"reel2", *m_reels[1]);
}

void maygayep_state::reel34_w(uint8_t data) {
  // logerror("Wrote %02x to Reels 3/4\n", data);
	m_reels[2]->update( data     & 0x0F);
	m_reels[3]->update((data>>4) & 0x0F);

	awp_draw_reel(machine(),"reel3", *m_reels[2]);
	awp_draw_reel(machine(),"reel4", *m_reels[3]);
}

void maygayep_state::reel56_w(uint8_t data) {
  // logerror("Wrote %02x to Reels 5/6\n", data);
	m_reels[4]->update( data     & 0x0F);
	m_reels[5]->update((data>>4) & 0x0F);

	awp_draw_reel(machine(),"reel5", *m_reels[4]);
	awp_draw_reel(machine(),"reel6", *m_reels[5]);
}

INPUT_PORTS_START(maygayep)
	PORT_START("SW1")
	PORT_DIPNAME( 0x01, 0x00, "SW101" ) PORT_DIPLOCATION("SW1:01")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On  ) )
	PORT_DIPNAME( 0x02, 0x00, "SW102" ) PORT_DIPLOCATION("SW1:02")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On  ) )
	PORT_DIPNAME( 0x04, 0x00, "SW103" ) PORT_DIPLOCATION("SW1:03")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On  ) )
	PORT_DIPNAME( 0x08, 0x00, "SW104" ) PORT_DIPLOCATION("SW1:04")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On  ) )
	PORT_DIPNAME( 0x10, 0x00, "SW105" ) PORT_DIPLOCATION("SW1:05")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On  ) )
	PORT_DIPNAME( 0x20, 0x00, "SW106" ) PORT_DIPLOCATION("SW1:06")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On  ) )
	PORT_DIPNAME( 0x40, 0x00, "SW107" ) PORT_DIPLOCATION("SW1:07")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On  ) )
	PORT_DIPNAME( 0x80, 0x00, "AntiFraud Protection" ) PORT_DIPLOCATION("SW1:08")
	PORT_DIPSETTING(    0x80, DEF_STR( Off  ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )

	PORT_START("SW2")
	PORT_DIPNAME( 0x01, 0x00, "SW201" ) PORT_DIPLOCATION("SW2:01")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x01, DEF_STR( On  ) )
	PORT_DIPNAME( 0x02, 0x00, "SW202" ) PORT_DIPLOCATION("SW2:02")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x02, DEF_STR( On  ) )
	PORT_DIPNAME( 0x04, 0x00, "SW203" ) PORT_DIPLOCATION("SW2:03")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x04, DEF_STR( On  ) )
	PORT_DIPNAME( 0x08, 0x00, "SW204" ) PORT_DIPLOCATION("SW2:04")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x08, DEF_STR( On  ) )
	PORT_DIPNAME( 0x10, 0x00, "SW205" ) PORT_DIPLOCATION("SW2:05")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x10, DEF_STR( On  ) )
	PORT_DIPNAME( 0x20, 0x00, "SW206" ) PORT_DIPLOCATION("SW2:06")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x20, DEF_STR( On  ) )
	PORT_DIPNAME( 0x40, 0x00, "SW207" ) PORT_DIPLOCATION("SW2:07")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x40, DEF_STR( On  ) )
	PORT_DIPNAME( 0x80, 0x00, "SW208" ) PORT_DIPLOCATION("SW2:08")
	PORT_DIPSETTING(    0x00, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x80, DEF_STR( On  ) )
INPUT_PORTS_END

/* in many sets the 0x100 - 0x1ff area contains revision information
    and sometimes bugfix notes..

    in other sets 0x200 - 0x204 contains 'bank' or 'reels' maybe to
    indicate if the roms are for a topbox or the player unit?

    there doesn't seem to be a hard defined standard for this header
    information, it probably varies between manufacturer

    todo: use this info to better sort the sets (note, the time / build
    dates don't appear to be accurate at least)
*/

void maygayep_state::init_maygayep() {
  uint8_t *src = memregion("maincpu")->base();
  for (int i = 0x100; i < 0x210; i++) {
    uint8_t val = src[i ^ 1];

    if (i % 0x40 == 0)
      logerror("\n");

    if ((val >= 0x20) && (val <= 0x7e)) {
      logerror("%c", val);
    } else {
      logerror(" ");
    }
  }

  logerror("\n");
}

void maygayep_state::maygayep(machine_config &config) {
  H83002(config, m_maincpu, 16000000);
  m_maincpu->set_addrmap(AS_PROGRAM, &maygayep_state::maygayep_map);

  SPEAKER(config, "lspeaker").front_left();
  SPEAKER(config, "rspeaker").front_right();

	YMZ280B(config, m_ymz, 10000000);
	m_ymz->add_route(ALL_OUTPUTS, "lspeaker", 1.0); // Channel 0
	m_ymz->add_route(ALL_OUTPUTS, "rspeaker", 1.0); // Channel 1

  TIMER(config, "refreshtimer").configure_periodic(FUNC(maygayep_state::refreshtimer), attotime::from_hz(6400));
  TIMER(config, "synctimer").configure_periodic(FUNC(maygayep_state::synctimer), attotime::from_hz(100));

  REEL(config, m_reels[0], STARPOINT_48STEP_REEL, 1, 3, 0x09, 4);
  m_reels[0]->optic_handler().set(FUNC(maygayep_state::reel_optic_cb<0>));
  REEL(config, m_reels[1], STARPOINT_48STEP_REEL, 1, 3, 0x09, 4);
  m_reels[1]->optic_handler().set(FUNC(maygayep_state::reel_optic_cb<1>));
  REEL(config, m_reels[2], STARPOINT_48STEP_REEL, 1, 3, 0x09, 4);
  m_reels[2]->optic_handler().set(FUNC(maygayep_state::reel_optic_cb<2>));
  REEL(config, m_reels[3], STARPOINT_48STEP_REEL, 1, 3, 0x09, 4);
  m_reels[3]->optic_handler().set(FUNC(maygayep_state::reel_optic_cb<3>));
  REEL(config, m_reels[4], STARPOINT_48STEP_REEL, 1, 3, 0x09, 4);
  m_reels[4]->optic_handler().set(FUNC(maygayep_state::reel_optic_cb<4>));
  REEL(config, m_reels[5], STARPOINT_48STEP_REEL, 1, 3, 0x09, 4);
  m_reels[5]->optic_handler().set(FUNC(maygayep_state::reel_optic_cb<5>));

  METERS(config, m_meters, 0).set_number(8);

  NVRAM(config, "nvram", nvram_device::DEFAULT_ALL_0);

  config.set_default_layout(layout_maygayep);
}
