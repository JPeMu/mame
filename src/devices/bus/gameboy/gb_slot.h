// license:BSD-3-Clause
// copyright-holders:Fabio Priuli, Wilbert Pol
/***************************************************************************

Cardridge port pinouts:
 Pin  Name     Description
  1   VCC      +5 VDC
  2   PHI      CPU clock ?
  3   /WR      Write
  4   /RD      Read
  5   /CS      SRAM select
  6   A0       Address 0
  7   A1       Address 1
  8   A2       Address 2
  9   A3       Address 3
 10   A4       Address 4
 11   A5       Address 5
 12   A6       Address 6
 13   A7       Address 7
 14   A8       Address 8
 15   A9       Address 9
 16   A10      Address 10
 17   A11      Address 11
 18   A12      Address 12
 19   A13      Address 13
 20   A14      Address 14
 21   A15      Address 15
 22   D0       Data 0
 23   D1       Data 1
 24   D2       Data 2
 25   D3       Data 3
 26   D4       Data 4
 27   D5       Data 5
 28   D6       Data 6
 29   D7       Data 7
 30   /RST     Reset
 31   AUDIOIN  Never used ?
 32   GND      Ground

***************************************************************************/
#ifndef MAME_BUS_GAMEBOY_GB_SLOT_H
#define MAME_BUS_GAMEBOY_GB_SLOT_H

#include "imagedev/cartrom.h"


/***************************************************************************
 TYPE DEFINITIONS
 ***************************************************************************/


/* PCB */
enum
{
	GB_MBC_NONE = 0,     /*  32KB ROM - No memory bank controller         */
	GB_MBC_MBC1,         /*   2MB ROM,   8KB RAM -or- 512KB ROM, 32KB RAM */
	GB_MBC_MBC2,         /* 256KB ROM,  32KB RAM                          */
	GB_MBC_MBC3,         /*   2MB ROM,  32KB RAM, RTC                     */
	GB_MBC_MBC4,         /*    ?? ROM,    ?? RAM                          */
	GB_MBC_MBC5,         /*   8MB ROM, 128KB RAM (32KB w/ Rumble)         */
	GB_MBC_TAMA5,        /*    ?? ROM     ?? RAM - What is this?          */
	GB_MBC_HUC1,         /*    ?? ROM,    ?? RAM - Hudson Soft Controller */
	GB_MBC_HUC3,         /*    ?? ROM,    ?? RAM - Hudson Soft Controller */
	GB_MBC_MBC6,         /*    ?? ROM,  32KB SRAM                         */
	GB_MBC_MBC7,         /*    ?? ROM,    ?? RAM                          */
	GB_MBC_M161,         /* 256kB ROM,    No RAM                          */
	GB_MBC_MMM01,        /*   8MB ROM, 128KB RAM                          */
	GB_MBC_WISDOM,       /*    ?? ROM,    ?? RAM - Wisdom tree controller */
	GB_MBC_MBC1_COL,     /*   1MB ROM,  32KB RAM - workaround for MBC1 on PCB that maps rom address lines differently */
	GB_MBC_SACHEN1,      /*   4MB ROM,    No RAM - Sachen MMC-1 variant */
	GB_MBC_SACHEN2,      /*   4MB ROM,    No RAM - Sachen MMC-2 variant */
	GB_MBC_YONGYONG,     /*    ?? ROM,    ?? RAM - Appears in Sonic 3D Blast 5 pirate */
	GB_MBC_LASAMA,       /*    ?? ROM,    ?? RAM - Appears in La Sa Ma */
	GB_MBC_ATVRACIN,
	GB_MBC_CAMERA,
	GB_MBC_188IN1,
	GB_MBC_SINTAX,
	GB_MBC_CHONGWU,
	GB_MBC_LICHENG,
	GB_MBC_DIGIMON,
	GB_MBC_ROCKMAN8,
	GB_MBC_SM3SP,
	GB_MBC_DKONG5,
	GB_MBC_UNK01,
	GB_MBC_MEGADUCK,     /* MEGADUCK style banking                        */
	GB_MBC_UNKNOWN       /* Unknown mapper                                */
};


// ======================> device_gb_cart_interface

class device_gb_cart_interface : public device_interface
{
public:
	// construction/destruction
	virtual ~device_gb_cart_interface();

	// reading and writing
	virtual uint8_t read_rom(offs_t offset) { return 0xff; }
	virtual void write_bank(offs_t offset, uint8_t data) {}
	virtual uint8_t read_ram(offs_t offset) { return 0xff; }
	virtual void write_ram(offs_t offset, uint8_t data) {}

	void rom_alloc(uint32_t size);
	void ram_alloc(uint32_t size);
	uint8_t* get_rom_base() { return m_rom; }
	uint8_t* get_ram_base() { return &m_ram[0]; }
	uint32_t get_rom_size() { return m_rom_size; }
	uint32_t get_ram_size() { return m_ram.size(); }

	void rom_map_setup(uint32_t size);
	void ram_map_setup(uint8_t banks);

	virtual void set_additional_wirings(uint8_t mask, int shift) { }  // MBC-1 will then overwrite this!
	void set_has_timer(bool val) { has_timer = val; }
	void set_has_rumble(bool val) { has_rumble = val; }
	void set_has_battery(bool val) { has_battery = val; }
	bool get_has_battery() { return has_battery; }

	void save_ram() { device().save_item(NAME(m_ram)); }

protected:
	device_gb_cart_interface(const machine_config &mconfig, device_t &device);

	// internal state
	uint8_t *m_rom;
	uint32_t m_rom_size;
	std::vector<uint8_t> m_ram;

	// bankswitch variables
	// we access ROM/RAM banks through these bank maps
	// default accesses are:
	// 0x0000-0x3fff = rom_bank_map[m_latch_bank]   (generally defaults to m_latch_bank = 0)
	// 0x4000-0x7fff = rom_bank_map[m_latch_bank2]  (generally defaults to m_latch_bank2 = 1)
	// 0xa000-0xbfff = ram_bank_map[m_ram_bank]   (generally defaults to m_ram_bank = 0)
	// suitable writes to 0x0000-0x7fff can then modify m_latch_bank/m_latch_bank2
	uint8_t rom_bank_map[512];    // 16K chunks of ROM
	uint8_t ram_bank_map[256];    // 16K chunks of RAM
	uint8_t m_ram_bank;
	uint16_t m_latch_bank, m_latch_bank2;

	bool has_rumble, has_timer, has_battery;
};


// ======================> gb_cart_slot_device_base

class gb_cart_slot_device_base : public device_t,
								public device_cartrom_image_interface,
								public device_single_card_slot_interface<device_gb_cart_interface>
{
public:
	// construction/destruction
	virtual ~gb_cart_slot_device_base();

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual void call_unload() override;

	virtual bool is_reset_on_load() const noexcept override { return true; }
	virtual const char *image_interface() const noexcept override { return "gameboy_cart"; }
	virtual const char *file_extensions() const noexcept override { return "bin,gb,gbc"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;

	int get_type() { return m_type; }
	static int get_cart_type(const uint8_t *ROM, uint32_t len);
	static bool get_mmm01_candidate(const uint8_t *ROM, uint32_t len);
	static bool is_mbc1col_game(const uint8_t *ROM, uint32_t len);

	void setup_ram(uint8_t banks);
	void internal_header_logging(uint8_t *ROM, uint32_t len);
	void save_ram() { if (m_cart && m_cart->get_ram_size()) m_cart->save_ram(); }

	// reading and writing
	virtual uint8_t read_rom(offs_t offset);
	virtual void write_bank(offs_t offset, uint8_t data);
	virtual uint8_t read_ram(offs_t offset);
	virtual void write_ram(offs_t offset, uint8_t data);


protected:
	gb_cart_slot_device_base(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	// device-level overrides
	virtual void device_start() override;

	int m_type;
	device_gb_cart_interface*       m_cart;
};

// ======================> gb_cart_slot_device

class gb_cart_slot_device :  public gb_cart_slot_device_base
{
public:
	// construction/destruction
	template <typename T>
	gb_cart_slot_device(machine_config const &mconfig, char const *tag, device_t *owner, T &&opts, char const *dflt)
		: gb_cart_slot_device(mconfig, tag, owner, (uint32_t)0)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}
	gb_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


// ======================> megaduck_cart_slot_device

class megaduck_cart_slot_device :  public gb_cart_slot_device_base
{
public:
	// construction/destruction
	template <typename T>
	megaduck_cart_slot_device(machine_config const &mconfig, char const *tag, device_t *owner, T &&opts, char const *dflt)
		: megaduck_cart_slot_device(mconfig, tag, owner, (uint32_t)0)
	{
		option_reset();
		opts(*this);
		set_default_option(dflt);
		set_fixed(false);
	}
	megaduck_cart_slot_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	// image-level overrides
	virtual image_init_result call_load() override;
	virtual const char *image_interface() const noexcept override { return "megaduck_cart"; }
	virtual const char *file_extensions() const noexcept override { return "bin"; }

	// slot interface overrides
	virtual std::string get_default_card_software(get_default_card_software_hook &hook) const override;
};




// device type definition
DECLARE_DEVICE_TYPE(GB_CART_SLOT,       gb_cart_slot_device)
DECLARE_DEVICE_TYPE(MEGADUCK_CART_SLOT, megaduck_cart_slot_device)

#endif // MAME_BUS_GAMEBOY_GB_SLOT_H
