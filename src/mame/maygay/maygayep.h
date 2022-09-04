// license:BSD-3-Clause
// copyright-holders:David Haywood, Tony Friery

#ifndef MAME_MAYGAY_MAYGAYEP_H
#define MAME_MAYGAY_MAYGAYEP_H

#pragma once

#include "awpvid.h"       //Fruit Machines Only

#include "cpu/h8/h83002.h"
#include "sound/ymz280b.h"
#include "machine/timer.h"
#include "machine/meters.h"
#include "machine/nvram.h"
#include "machine/roc10937.h"   // vfd
#include "machine/steppers.h"   // stepper motor
#include "epochrtc.h"

class maygayep_state : public driver_device {
public:
  maygayep_state(const machine_config &mconfig, device_type type, const char *tag) :
      driver_device(mconfig, type, tag), 
      m_maincpu(*this, "maincpu"),
      m_sw1_port(*this, "SW1"),
      m_sw2_port(*this, "SW2"),
      m_vfd(*this, "vfd"),
      m_reels(*this, "reel%u", 0U),
      m_meters(*this, "meters"),
      m_lamps(*this, "lamp%u", 0U),
      m_leds(*this, "digit%u", 0U),
      m_ymz(*this, "ymz"),
      m_status(*this, "status")
  {    
  }

  void maygayep(machine_config &config);

  void init_maygayep();

private:

  // devices
  required_device<cpu_device> m_maincpu;
  required_ioport m_sw1_port;
  required_ioport m_sw2_port;
	optional_device<s16lf01_device> m_vfd;
	required_device_array<stepper_device, 6> m_reels;
	optional_device<meters_device> m_meters;
	output_finder<512> m_lamps;
	output_finder<512> m_leds;
	required_device<ymz280b_device> m_ymz;
  output_finder<1> m_status;

  device_epoch_rtc m_rtc = { 0 };
  
  // Internal
  uint8_t ep_int_enable;
  uint8_t ep_int_status;
  uint8_t ep_sys_control;
  uint8_t ep_sys_status;
  uint8_t ep_io_control;
  uint8_t ep_last_read_int_status;
  uint8_t ep_sys_enable;
  uint16_t ep_led_timers[8];
  uint16_t ep_lamp_timers[8];
  uint8_t ep_led_dim_level;
  uint8_t ep_lamp_dim_level;
  uint8_t ep_reel_drives[8];

  uint8_t ep_hopper_drive;
  uint8_t ep_mech_diverts;
  uint8_t ep_mech_drive;
  uint8_t ep_input_irqen[512];
	
  // Hardware
	int m_optic_pattern = 0;

	virtual void machine_start() override;
	virtual void machine_reset() override;

  TIMER_DEVICE_CALLBACK_MEMBER( refreshtimer ); // 6.4 kHz Refresh Timer
  TIMER_DEVICE_CALLBACK_MEMBER( synctimer );    // 100Hz Sync Timer

	template <unsigned N> DECLARE_WRITE_LINE_MEMBER(reel_optic_cb) { if (state) m_optic_pattern |= (1 << N); else m_optic_pattern &= ~(1 << N); }

  void raise_ext_irq5(void);
  void maygayep_map(address_map &map);
  void maygayep_iomap(address_map &map);
  
  uint8_t ymz_read(offs_t addr);
  void ymz_write(offs_t addr, uint8_t value);
  uint8_t inputs_read(offs_t addr);
  uint8_t input_int_read(offs_t addr);
  void input_int_write(offs_t addr, uint8_t value);
  uint8_t lamps_read(offs_t addr);
  void lamps_write(offs_t addr, uint8_t value);
  uint8_t leds_read(offs_t addr);
  void leds_write(offs_t addr, uint8_t value);
  uint8_t misc_read(offs_t addr);
  void misc_write(offs_t addr, uint8_t value);
  uint8_t fade_dim_read(offs_t addr);
  void fade_dim_write(offs_t addr, uint8_t value);
  uint8_t dips_r(offs_t addr);
  void dummy_write(offs_t addr, uint8_t value);

  void reel12_w(uint8_t data);
  void reel34_w(uint8_t data);
  void reel56_w(uint8_t data);

  // IO Ports
  uint8_t port8_read(void);
  uint8_t porta_read(void);
  void port8_write(uint8_t value);
  void porta_write(uint8_t value);

  uint8_t epsysstt_r(offs_t addr);
  void epsysstt_w(offs_t addr, uint8_t value);
  uint8_t epsysctl_r(offs_t addr);
  void epsysctl_w(offs_t addr, uint8_t value);
  uint8_t epioctl_r(offs_t addr);
  void epioctl_w(offs_t addr, uint8_t value);
  uint8_t epsysenb_r(offs_t addr);
  void epsysenb_w(offs_t addr, uint8_t value);
  uint8_t epintenb_r(offs_t addr);
  void epintenb_w(offs_t addr, uint8_t value);
  uint8_t epintstt_r(offs_t addr);
  void epintctl_w(offs_t addr, uint8_t value);

};

INPUT_PORTS_EXTERN( maygayep );

#endif // MAME_MAYGAY_MAYGAYEP_H
