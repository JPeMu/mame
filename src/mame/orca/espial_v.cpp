// license:BSD-3-Clause
// copyright-holders:Brad Oliver
/***************************************************************************

  video.c

  Functions to emulate the video hardware of the machine.

***************************************************************************/

#include "emu.h"
#include "espial.h"

/***************************************************************************

  Convert the color PROMs into a more useable format.

  Espial has two 256x4 palette PROMs.

  I don't know for sure how the palette PROMs are connected to the RGB
  output, but it's probably the usual:

  bit 3 -- 220 ohm resistor  -- BLUE
        -- 470 ohm resistor  -- BLUE
        -- 220 ohm resistor  -- GREEN
  bit 0 -- 470 ohm resistor  -- GREEN
  bit 3 -- 1  kohm resistor  -- GREEN
        -- 220 ohm resistor  -- RED
        -- 470 ohm resistor  -- RED
  bit 0 -- 1  kohm resistor  -- RED

***************************************************************************/

void espial_state::espial_palette(palette_device &palette) const
{
	uint8_t const *const color_prom = memregion("proms")->base();

	for (int i = 0; i < palette.entries(); i++)
	{
		int bit0, bit1, bit2;

		// red component
		bit0 = BIT(color_prom[i], 0);
		bit1 = BIT(color_prom[i], 1);
		bit2 = BIT(color_prom[i], 2);
		int const r = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		// green component
		bit0 = BIT(color_prom[i], 3);
		bit1 = BIT(color_prom[i + palette.entries()], 0);
		bit2 = BIT(color_prom[i + palette.entries()], 1);
		int const g = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;
		// blue component
		bit0 = 0;
		bit1 = BIT(color_prom[i + palette.entries()], 2);
		bit2 = BIT(color_prom[i + palette.entries()], 3);
		int const b = 0x21 * bit0 + 0x47 * bit1 + 0x97 * bit2;

		palette.set_pen_color(i, rgb_t(r, g, b));
	}
}



/***************************************************************************

  Callbacks for the TileMap code

***************************************************************************/

TILE_GET_INFO_MEMBER(espial_state::get_tile_info)
{
	uint8_t code = m_videoram[tile_index];
	uint8_t col = m_colorram[tile_index];
	uint8_t attr = m_attributeram[tile_index];
	tileinfo.set(0,
					code | ((attr & 0x03) << 8),
					col & 0x3f,
					TILE_FLIPYX(attr >> 2));
}



/*************************************
 *
 *  Video system start
 *
 *************************************/

void espial_state::video_start()
{
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(*this, FUNC(espial_state::get_tile_info)), TILEMAP_SCAN_ROWS, 8, 8, 32, 32);
	m_bg_tilemap->set_scroll_cols(32);

	save_item(NAME(m_flipscreen));
}

VIDEO_START_MEMBER(espial_state,netwars)
{
	/* Net Wars has a tile map that's twice as big as Espial's */
	m_bg_tilemap = &machine().tilemap().create(*m_gfxdecode, tilemap_get_info_delegate(*this, FUNC(espial_state::get_tile_info)), TILEMAP_SCAN_ROWS, 8, 8, 32, 64);

	m_bg_tilemap->set_scroll_cols(32);
	m_bg_tilemap->set_scrolldy(0, 0x100);

	save_item(NAME(m_flipscreen));
}


/*************************************
 *
 *  Memory handlers
 *
 *************************************/

void espial_state::espial_videoram_w(offs_t offset, uint8_t data)
{
	m_videoram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}


void espial_state::espial_colorram_w(offs_t offset, uint8_t data)
{
	m_colorram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}


void espial_state::espial_attributeram_w(offs_t offset, uint8_t data)
{
	m_attributeram[offset] = data;
	m_bg_tilemap->mark_tile_dirty(offset);
}


void espial_state::espial_scrollram_w(offs_t offset, uint8_t data)
{
	m_scrollram[offset] = data;
	m_bg_tilemap->set_scrolly(offset, data);
}


void espial_state::espial_flipscreen_w(uint8_t data)
{
	m_flipscreen = data;
	m_bg_tilemap->set_flip(m_flipscreen ? TILEMAP_FLIPX | TILEMAP_FLIPY : 0);
}


/*************************************
 *
 *  Video update
 *
 *************************************/

void espial_state::draw_sprites( bitmap_ind16 &bitmap, const rectangle &cliprect )
{
	int offs;

	/* Note that it is important to draw them exactly in this */
	/* order, to have the correct priorities. */
	for (offs = 0; offs < 16; offs++)
	{
		int sx, sy, code, color, flipx, flipy;


		sx = m_spriteram_1[offs + 16];
		sy = m_spriteram_2[offs];
		code = m_spriteram_1[offs] >> 1;
		color = m_spriteram_2[offs + 16];
		flipx = m_spriteram_3[offs] & 0x04;
		flipy = m_spriteram_3[offs] & 0x08;

		if (m_flipscreen)
		{
			flipx = !flipx;
			flipy = !flipy;
		}
		else
		{
			sy = 240 - sy;
		}

		if (m_spriteram_1[offs] & 1) /* double height */
		{
			if (m_flipscreen)
			{
				m_gfxdecode->gfx(1)->transpen(bitmap,cliprect,
						code,color,
						flipx,flipy,
						sx,sy + 16,0);
				m_gfxdecode->gfx(1)->transpen(bitmap,cliprect,
						code + 1,
						color,
						flipx,flipy,
						sx,sy,0);
			}
			else
			{
				m_gfxdecode->gfx(1)->transpen(bitmap,cliprect,
						code,color,
						flipx,flipy,
						sx,sy - 16,0);
				m_gfxdecode->gfx(1)->transpen(bitmap,cliprect,
						code + 1,color,
						flipx,flipy,
						sx,sy,0);
			}
		}
		else
		{
			m_gfxdecode->gfx(1)->transpen(bitmap,cliprect,
					code,color,
					flipx,flipy,
					sx,sy,0);
		}
	}
}


uint32_t espial_state::screen_update_espial(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect)
{
	m_bg_tilemap->draw(screen, bitmap, cliprect, 0, 0);
	draw_sprites(bitmap, cliprect);
	return 0;
}
