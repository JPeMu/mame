// license:BSD-3-Clause
// copyright-holders:David Haywood
/* Super Kaneko Nova System Sprites

   "CG24173 6186" & "CG24143 4181" (always used as a pair?)

   used by suprnova.cpp
           galpani3.cpp
           jchan.cpp

   TODO:
   - Get rid of sprite position kludges
   - Fix zooming precision/rounding (most noticeable on jchan backgrounds)
   - RLE uses fifo7200 chips

   reference
   - jchan : https://youtu.be/PJijmhRwMUk

*/

#include "emu.h"
#include "sknsspr.h"
#include "screen.h"

DEFINE_DEVICE_TYPE(SKNS_SPRITE, sknsspr_device, "sknsspr", "SKNS Sprite")


sknsspr_device::sknsspr_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SKNS_SPRITE, tag, owner, clock)
	, device_video_interface(mconfig, *this)
	, device_rom_interface(mconfig, *this)
{
}

void sknsspr_device::device_start()
{
	m_decodebuffer = make_unique_clear<u8[]>(SUPRNOVA_DECODE_BUFFER_SIZE);

	save_pointer(NAME(m_decodebuffer), SUPRNOVA_DECODE_BUFFER_SIZE);
	//printf("sknsspr_device::device_start()\n");
}

void sknsspr_device::device_reset()
{
	//printf("sknsspr_device::device_reset()\n");
}

void sknsspr_device::rom_bank_updated()
{
	//printf("sknsspr_device::rom_bank_updated()\n");
}

int sknsspr_device::skns_rle_decode ( int romoffset, int size )
{
	int decodeoffset = 0;

	while(size>0) {
		u8 code = read_byte((romoffset++) & ((1<<27) - 1));
		size -= (code & 0x7f) + 1;
		if(code & 0x80) { /* (code & 0x7f) normal values will follow */
			code &= 0x7f;
			do {
				m_decodebuffer[(decodeoffset++) & (SUPRNOVA_DECODE_BUFFER_SIZE-1)] = read_byte((romoffset++) & ((1<<27) - 1));
				code--;
			} while(code != 0xff);
		} else {  /* repeat next value (code & 0x7f) times */
			u8 val = read_byte((romoffset++) & ((1<<27) - 1));
			do {
				m_decodebuffer[(decodeoffset++) & (SUPRNOVA_DECODE_BUFFER_SIZE-1)] = val;
				code--;
			} while(code != 0xff);
		}
	}
	return romoffset & ((1<<27) - 1);
}

void sknsspr_device::skns_sprite_kludge(int x, int y)
{
	m_sprite_kludge_x = x << 6;
	m_sprite_kludge_y = y << 6;
}

/* Zooming blitter, zoom is by way of both source and destination offsets */
/* We are working in .16 fixed point if you hadn't guessed */

#define z_decls(step)               \
	u32 zxs = 0x10000-(zx_m);            \
	u32 zxd = 0x10000-(zx_s);        \
	u32 zys = 0x10000-(zy_m);            \
	u32 zyd = 0x10000-(zy_s);        \
	u32 bxs = 0, bys = 0, xs, ys;               \
	int xd, yd, old, old2;      \
	int step_spr = step;                \
	rectangle clip;                 \
	clip.min_x = cliprect.min_x<<16;                 \
	clip.max_x = (cliprect.max_x+1)<<16;                 \
	clip.min_y = cliprect.min_y<<16;                 \
	clip.max_y = (cliprect.max_y+1)<<16;                 \
	sx <<= 16;                   \
	sy <<= 16;                   \
	x <<= 10;                    \
	y <<= 10;

#define z_clamp_x_min()         \
	if(x < clip.min_x) {                    \
		do {                    \
			bxs += zxs;             \
			x += zxd;                   \
		} while(x < clip.min_x);                \
	}

#define z_clamp_x_max()         \
	if(x >= clip.max_x) {                \
		do {                    \
			bxs += zxs;             \
			x -= zxd;                   \
		} while(x >= clip.max_x);                \
	}

#define z_clamp_y_min()         \
	if(y < clip.min_y) {                    \
		do {                    \
			bys += zys;             \
			y += zyd;                   \
		} while(y < clip.min_y);                \
		src += (bys>>16)*step_spr;           \
	}

#define z_clamp_y_max()         \
	if(y >= clip.max_y) {                \
		do {                    \
			bys += zys;             \
			y -= zyd;                   \
		} while(y >= clip.max_y);                \
		src += (bys>>16)*step_spr;           \
	}

#define z_loop_x()          \
	xs = bxs;                   \
	xd = x;                 \
	while(xs < sx && xd < clip.max_x)

#define z_loop_x_flip()         \
	xs = bxs;                   \
	xd = x;                 \
	while(xs < sx && xd >= clip.min_x)

#define z_loop_y()          \
	ys = bys;                   \
	yd = y;                 \
	while(ys < sy && yd < clip.max_y)

#define z_loop_y_flip()         \
	ys = bys;                   \
	yd = y;                 \
	while(ys < sy && yd >= clip.min_y)

#define z_draw_pixel()              \
	u8 val = src[(xs >> 16) & 0x3f];    \
	if(val)                 \
		bitmap.pix(yd>>16, xd>>16) = val + colour;

#define z_x_dst(op)         \
	old = xd;                   \
	do {                        \
		xs += zxs;                  \
		xd op zxd;                  \
	} while(!((xd^old) & ~0xffff));

#define z_y_dst(op)         \
	old = yd;                   \
	old2 = ys;                  \
	do {                        \
		ys += zys;                  \
		yd op zyd;                  \
	} while(!((yd^old) & ~0xffff));           \
	while((ys^old2) & ~0xffff) {          \
		src += step_spr;                \
		old2 += 0x10000;               \
	}

static void blit_nf_z(bitmap_ind16 &bitmap, const rectangle &cliprect, const u8 *src, int x, int y, int sx, int sy, u16 zx_m, u16 zx_s, u16 zy_m, u16 zy_s, int colour)
{
	z_decls(sx);
	z_clamp_x_min();
	z_clamp_y_min();
	z_loop_y() {
		z_loop_x() {
			z_draw_pixel();
			z_x_dst(+=);
		}
		z_y_dst(+=);
	}
}

static void blit_fy_z(bitmap_ind16 &bitmap, const rectangle &cliprect, const u8 *src, int x, int y, int sx, int sy, u16 zx_m, u16 zx_s, u16 zy_m, u16 zy_s, int colour)
{
	z_decls(sx);
	z_clamp_x_min();
	z_clamp_y_max();
	z_loop_y_flip() {
		z_loop_x() {
			z_draw_pixel();
			z_x_dst(+=);
		}
		z_y_dst(-=);
	}
}

static void blit_fx_z(bitmap_ind16 &bitmap, const rectangle &cliprect, const u8 *src, int x, int y, int sx, int sy, u16 zx_m, u16 zx_s, u16 zy_m, u16 zy_s, int colour)
{
	z_decls(sx);
	z_clamp_x_max();
	z_clamp_y_min();
	z_loop_y() {
		z_loop_x_flip() {
			z_draw_pixel();
			z_x_dst(-=);
		}
		z_y_dst(+=);
	}
}

static void blit_fxy_z(bitmap_ind16 &bitmap, const rectangle &cliprect, const u8 *src, int x, int y, int sx, int sy, u16 zx_m, u16 zx_s, u16 zy_m, u16 zy_s, int colour)
{
	z_decls(sx);
	z_clamp_x_max();
	z_clamp_y_max();
	z_loop_y_flip() {
		z_loop_x_flip() {
			z_draw_pixel();
			z_x_dst(-=);
		}
		z_y_dst(-=);
	}
}

static void (*const blit_z[4])(bitmap_ind16 &bitmap, const rectangle &cliprect, const u8 *src, int x, int y, int sx, int sy, u16 zx_m, u16 zx_s, u16 zy_m, u16 zy_s, int colour) = {
	blit_nf_z,
	blit_fy_z,
	blit_fx_z,
	blit_fxy_z,
};

void sknsspr_device::skns_draw_sprites(bitmap_ind16 &bitmap, const rectangle &cliprect, u32* spriteram_source, size_t spriteram_size, u32* sprite_regs)
{
	/*- SPR RAM Format -**

	  16 bytes per sprite

0x00  --ss --SS  z--- ----  jjjg g-ff  ppcc cccc

	  s = y size
	  S = x size
	  j = joint
	  g = group sprite is part of (if groups are enabled)
	  f = flip
	  p = priority
	  c = palette

0x04  ---- -aaa  aaaa aaaa  aaaa aaaa  aaaa aaaa

	  a = ROM address of sprite data

0x08  ZZZZ ZZ--  zzzz zz--  xxxx xxxx  xxxx xxxx

	  Z = horizontal zoom table
	  z = horizontal zoom subtable
	  x = x position (10.6 fixed point)

0x0C  ZZZZ ZZ--  zzzz zz--  yyyy yyyy  yyyy yyyy

	  Z = vertical zoom table
	  z = vertical zoom subtable
	  x = y position (10.6 fixed point)

	**- End of Comments -*/

	/* sprite ram start / end is not really fixed registers change it */

	//printf ("addr %08x\n", (sprite_regs[0x14/4]));


	u32 *source = spriteram_source;
	u32 *finish = source + spriteram_size/4;

	s16 group_x_offset[4];
	int group_y_offset[4];
	int group_enable;
	int group_number;
	int sprite_flip;
	s16 sprite_x_scroll;
	s16 sprite_y_scroll;
	/* galpani3 uses sprite trail effect (disable clearing sprite bitmap) */
	int clear_bitmap = (~sprite_regs[0x04/4] & 0x04); // RWR1
	int disabled = sprite_regs[0x04/4] & 0x08; // RWR1
	int xsize,ysize, size, pri=0, romoffset, colour=0, xflip,yflip, joint;
	int sx,sy;
	int endromoffs=0;
	int grow;
	u16 zoomx_m, zoomx_s, zoomy_m, zoomy_s;
	s16 xpos=0,ypos=0;

	if (clear_bitmap)
	{
		bitmap.fill(0x0000, cliprect);
	}

	if ((!disabled)){
		group_enable    = (sprite_regs[0x00/4] & 0x0040) >> 6; // RWR0

		/* Sengekis uses global flip */
		sprite_flip = (sprite_regs[0x04/4] & 0x03); // RWR1

		sprite_y_scroll = s16((sprite_regs[0x08/4] & 0x7fff) << 1) >> 1; // RWR2
		sprite_x_scroll = s16((sprite_regs[0x10/4] & 0x7fff) << 1) >> 1; // RWR4

		group_x_offset[0] = (sprite_regs[0x18/4] & 0xffff); // RWR6
		group_y_offset[0] = (sprite_regs[0x1c/4] & 0xffff); // RWR7

		group_x_offset[1] = (sprite_regs[0x20/4] & 0xffff); // RWR8
		group_y_offset[1] = (sprite_regs[0x24/4] & 0xffff); // RWR9

		group_x_offset[2] = (sprite_regs[0x28/4] & 0xffff); // RWR10
		group_y_offset[2] = (sprite_regs[0x2c/4] & 0xffff); // RWR11

		group_x_offset[3] = (sprite_regs[0x30/4] & 0xffff); // RWR12
		group_y_offset[3] = (sprite_regs[0x34/4] & 0xffff); // RWR13

	//  popmessage ("x %08x y %08x x2 %08x y2 %08x",sprite_x_scroll, sprite_y_scroll,group_x_offset[1], group_y_offset[1]);
	//  popmessage("%d %d %d %d A:%d B:%d", m_sprite_kludge_x, m_sprite_kludge_y, sprite_x_scroll, sprite_y_scroll, (skns_pal_regs[0x00/4] & 0x7000) >> 12, (skns_pal_regs[0x10/4] & 0x7000) >> 12);
	//  if (keyboard_pressed(KEYCODE_Q)) m_sprite_kludge_x++;
	//  if (keyboard_pressed(KEYCODE_W)) m_sprite_kludge_x--;
	//  if (keyboard_pressed(KEYCODE_E)) m_sprite_kludge_y++;
	//  if (keyboard_pressed(KEYCODE_D)) m_sprite_kludge_y--;

		// Tilemap Pri/enables
	//  popmessage("A: %x %x B: %x %x", skns_v3_regs[0x10/4]>>3, skns_v3_regs[0x10/4]&7, skns_v3_regs[0x34/4]>>3, skns_v3_regs[0x34/4]&7);

		/* Seems that sprites are consistently off by a fixed no. of pixels in different games
		   (Patterns emerge through Manufacturer/Date/Orientation) */
		sprite_x_scroll += m_sprite_kludge_x;
		sprite_y_scroll += m_sprite_kludge_y;

		while( source<finish )
		{
			xflip = (source[0] & 0x00000200) >> 9;
			yflip = (source[0] & 0x00000100) >> 8;

			ysize = (source[0] & 0x30000000) >> 28;
			xsize = (source[0] & 0x03000000) >> 24;
			xsize ++;
			ysize ++;

			xsize *= 16;
			ysize *= 16;

			size = xsize * ysize;

			joint = (source[0] & 0x0000e000) >> 13;

			if (!(joint & 1))
			{
				xpos =  (source[2] & 0x0000ffff);
				ypos =  (source[3] & 0x0000ffff);

				xpos += sprite_x_scroll; // Global offset
				ypos += sprite_y_scroll;

				if (group_enable)
				{
					group_number = (source[0] & 0x00001800) >> 11;

					/* the group positioning doesn't seem to be working as i'd expect,
					   if I apply the x position the cursor on galpani4 ends up moving
					   from the correct position to too far right, also the y offset
					   seems to cause the position to be off by one in galpans2 even if
					   it fixes the position in galpani4?

					   even if I take into account the global sprite scroll registers
					   it isn't right

					   global offset kludged using game specific offset -pjp */

					xpos += group_x_offset[group_number];
					ypos += group_y_offset[group_number];
				}
			}
			else
			{
				xpos +=  (source[2] & 0x0000ffff);
				ypos +=  (source[3] & 0x0000ffff);
			}

			/* Local sprite offset (for taking flip into account and drawing offset) */
			sx = xpos;
			sy = ypos;

			/* Global Sprite Flip (sengekis) */
			if (sprite_flip&2)
			{
				xflip ^= 1;
				sx = ((screen().visible_area().max_x+1) << 6) - sx;
			}
			if (sprite_flip&1)
			{
				yflip ^= 1;
				sy = ((screen().visible_area().max_y+1) << 6) - sy;
			}

			/* Palette linking */
			if (!(joint & 2))
			{
				colour = (source[0] & 0x0000003f) >> 0;
			}

			/* Priority and Tile linking */
			if (!(joint & 4))
			{
				romoffset = (source[1] & 0x07ffffff) >> 0;
				pri = (source[0] & 0x000000c0) >> 6;
			} else {
				romoffset = endromoffs;
			}

			grow = (source[0]>>23) & 1;

			if (!grow)
			{
				zoomx_m = (source[2] & 0xff000000) >> 16;
				zoomx_s = (source[2] & 0x00ff0000) >> 8;
				zoomy_m = (source[3] & 0xff000000) >> 16;
				zoomy_s = (source[3] & 0x00ff0000) >> 8;
			}
			else
			{
				// sengekis uses this on sprites which are shrinking as they head towards the ground
				// it's also used on the input test of Gals Panic S2
				//
				// it appears to offer a higher precision 'shrink' mode (although I'm not entirely
				//  convinced this implementation is correct because we simply end up ignoring
				//  part of the data)
				zoomx_m = 0;
				zoomx_s = (source[2] & 0xffff0000) >> 16;
				zoomy_m = 0;
				zoomy_s = (source[3] & 0xffff0000) >> 16;
			}


			endromoffs = skns_rle_decode( romoffset, size );

			// in Cyvern

			//  train in tunnel pri = 0x00
			//  nothing?         = 0x01
			//  players etc. pri = 0x02
			//  pickups etc. pri = 0x03

			// PriTest
//          if(!( (keyboard_pressed(KEYCODE_Q)&&(pri==0)) || (keyboard_pressed(KEYCODE_W)&&(pri==1)) || (keyboard_pressed(KEYCODE_E)&&(pri==2)) || (keyboard_pressed(KEYCODE_D)&&(pri==3)) ))
//          if( !(keyboard_pressed(KEYCODE_Q) && ((source[0] & 0x00800000)>>24)) )



			{
				int NewColour = (colour<<8) | (pri << 14);

				if(zoomx_m || zoomx_s || zoomy_m || zoomy_s)
				{
					blit_z[ (xflip<<1) | yflip ](bitmap, cliprect, m_decodebuffer.get(), sx, sy, xsize, ysize, zoomx_m, zoomx_s, zoomy_m, zoomy_s, NewColour);
				}
				else
				{
					sx >>= 6;
					sy >>= 6;
					if (!xflip && !yflip) {
						for (int xx = 0; xx<xsize; xx++)
						{
							if ((sx+xx < (cliprect.max_x+1)) && (sx+xx >= cliprect.min_x))
							{
								for (int yy = 0; yy<ysize; yy++)
								{
									if ((sy+yy < (cliprect.max_y+1)) && (sy+yy >= cliprect.min_y))
									{
										int const pix = m_decodebuffer[xsize*yy+xx];
										if (pix)
											bitmap.pix(sy+yy, sx+xx) = pix+ NewColour; // change later
									}
								}
							}
						}
					} else if (!xflip && yflip) {
						sy -= ysize;

						for (int xx = 0; xx<xsize; xx++)
						{
							if ((sx+xx < (cliprect.max_x+1)) && (sx+xx >= cliprect.min_x))
							{
								for (int yy = 0; yy<ysize; yy++)
								{
									if ((sy+(ysize-1-yy) < (cliprect.max_y+1)) && (sy+(ysize-1-yy) >= cliprect.min_y))
									{
										int const pix = m_decodebuffer[xsize*yy+xx];
										if (pix)
											bitmap.pix(sy+(ysize-1-yy), sx+xx) = pix+ NewColour; // change later
									}
								}
							}
						}
					} else if (xflip && !yflip) {
						sx -= xsize;

						for (int xx = 0; xx<xsize; xx++)
						{
							if ( (sx+(xsize-1-xx) < (cliprect.max_x+1)) && (sx+(xsize-1-xx) >= cliprect.min_x))
							{
								for (int yy = 0; yy<ysize; yy++)
								{
									if ((sy+yy < (cliprect.max_y+1)) && (sy+yy >= cliprect.min_y))
									{
										int const pix = m_decodebuffer[xsize*yy+xx];
										if (pix)
											bitmap.pix(sy+yy, sx+(xsize-1-xx)) = pix+ NewColour; // change later
									}
								}
							}
						}
					} else if (xflip && yflip) {
						sx -= xsize;
						sy -= ysize;

						for (int xx = 0; xx<xsize; xx++)
						{
							if ((sx+(xsize-1-xx) < (cliprect.max_x+1)) && (sx+(xsize-1-xx) >= cliprect.min_x))
							{
								for (int yy = 0; yy<ysize; yy++)
								{
									if ((sy+(ysize-1-yy) < (cliprect.max_y+1)) && (sy+(ysize-1-yy) >= cliprect.min_y))
									{
										int const pix = m_decodebuffer[xsize*yy+xx];
										if (pix)
											bitmap.pix(sy+(ysize-1-yy), sx+(xsize-1-xx)) = pix+ NewColour; // change later
									}
								}
							}
						}
					}
				}
			}//End PriTest

			source+=4;
		}
	}
}
