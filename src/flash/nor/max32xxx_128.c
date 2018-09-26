/***************************************************************************
 *   Copyright (C) 2012 by Maxim Integrated                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/***************************************************************************
* Maxim Flash
***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/algorithm.h>
#include <target/armv7m.h>

#define FLSH_ADDR		0x000
#define FLSH_CLKDIV		0x004
#define FLSH_CN			0x008
#define PR1E_ADDR		0x00C
#define PR2S_ADDR		0x010
#define PR2E_ADDR		0x014
#define PR3S_ADDR		0x018
#define PR3E_ADDR		0x01C
#define FLSH_MD			0x020
#define FLSH_INT		0x024
#define FLSH_DATA0		0x030
#define FLSH_DATA1		0x034
#define FLSH_DATA2		0x038
#define FLSH_DATA3		0x03C

#define FLSH_INT_AF		0x00000002

#define FLSH_CN_UNLOCK_MASK	0xF0000000
#define FLSH_CN_UNLOCK_VALUE	0x20000000

#define FLSH_CN_PEND		0x01000000

#define FLSH_CN_ERASE_CODE_MASK	0x0000FF00
#define FLSH_CN_ERASE_CODE_PGE	0x00005500
#define FLSH_CN_ERASE_CODE_ME	0x0000AA00

#define FLSH_CN_PGE		0x00000004
#define FLSH_CN_ME		0x00000002
#define FLSH_CN_WR		0x00000001

#define MASK_FLASH_BUSY         (0x048800E0 & ~0x04000000)
#define MASK_DISABLE_INTS       0xFFFFFCFC
#define MASK_FLASH_UNLOCKED     (0xF588FFEF & ~0x04000000)
#define MASK_FLASH_LOCK         (0xF588FFEF & ~0x04000000)
#define MASK_FLASH_ERASE        (0xF588FFEF & ~0x04000000)
#define MASK_FLASH_ERASED       (0xF48800EB & ~0x04000000)
#define MASK_ACCESS_VIOLATIONS  0xFFFFFCFC
#define MASK_FLASH_WRITE        (0xF588FFEF & ~0x04000000)
#define MASK_WRITE_ALIGNED      (0xF588FFEF & ~0x04000000)
#define MASK_WRITE_COMPLETE     (0xF488FFEE & ~0x04000000)
#define MASK_WRITE_BURST        (0xF588FFEF & ~0x04000000)
#define MASK_BURST_COMPLETE     (0xF488FFEE & ~0x04000000)
#define MASK_WRITE_REMAINING    (0xF588FFEF & ~0x04000000)
#define MASK_REMAINING_COMPLETE (0xF488FFEE & ~0x04000000)
#define MASK_MASS_ERASE         (0xF588FFEF & ~0x04000000)
#define MASK_ERASE_COMPLETE     (0xF48800ED & ~0x04000000)

static int maxim128_mass_erase(struct flash_bank *bank);

struct maxim128_flash_bank
{
	int probed;
	unsigned int flash_size;
	unsigned int flc_base;
	unsigned int sector_size;
	unsigned int clkdiv_value;
	unsigned int int_state;
	unsigned int burst_size_bits;
};
#if 0
/* see contib/loaders/flash/maxim128.s for src */
static uint8_t maxim128_write_code0[] = {
					/* write: */
	0xDF, 0xF8, 0x4C, 0x40,		/* ldr 	  r4, pFLASH_CTRL_BASE */
					/* wait_fifo: */
	0xD0, 0xF8, 0x00, 0x80,		/* ldr 	  r8, [r0, #0] */
	0xB8, 0xF1, 0x00, 0x0F,		/* cmp 	  r8, #0 */
	0x00, 0xF0, 0x1E, 0x80,		/* beq 	  exit */
	0x47, 0x68,			/* ldr 	  r7, [r0, #4] */
	0x47, 0x45,			/* cmp 	  r7, r8 */
	0x3F, 0xF4, 0xF6, 0xAF,		/* beq 	  wait_fifo */
					/* mainloop: */
	0x22, 0x60,			/* str	  r2, [r4, #0] */
	0x02, 0xF1, 0x04, 0x02,		/* add	  r2, r2, #4 */
	0x57, 0xF8, 0x04, 0x8B,		/* ldr	  r8, [r7], #4 */
	0xC4, 0xF8, 0x30, 0x80,		/* str	  r8, [r4, #0x30] */
	0xA5, 0x68,			/* ldr	  r5, [r4, #0x08] */
	0x45, 0xF0, 0x01, 0x05,		/* orr	  r5, r5, #1 */
	0xA5, 0x60,			/* str	  r5, [r4, #0x08] */
					/* busy: */
	0xD4, 0xF8, 0x08, 0x80,		/* ldr	  r8, [r4, #8] */
	0x18, 0xF0, 0x01, 0x0F,		/* tst	  r8, #1 */
	0x7F, 0xF4, 0xFA, 0xAF,		/* bne	  busy */
	0x8F, 0x42,			/* cmp 	  r7, r1 */
	0x28, 0xBF,			/* it  	  cs */
	0x00, 0xF1, 0x08, 0x07,		/* addcs  r7, r0, #8 */
	0x47, 0x60,			/* str 	  r7, [r0, #4] */
	0x01, 0x3B,			/* subs	  r3, r3, #1 */
	0x0B, 0xB1,			/* cbz 	  r3, exit */
	0xFF, 0xF7, 0xDC, 0xBF,		/* b	  wait_fifo */
					/* exit: */
	0x00, 0xBE,			/* bkpt	  #0 */
	0x00, 0xBF,			/* nop */

	/* pFLASH_CTRL_BASE: */
	0x00, 0x00, 0x00, 0x40		/* .word 0x40000000 (value will be overwritten) */
};
#endif

static 
uint8_t maxim128_write_code[] = {
// <write>:
   	0xdf,0xf8, 0x78,0x40, 	//ldr.w	r4, [pc, #96]	;  <pFLASH_CTRL_BASE>
        0xa5,0x68,              //ldr	r5, [r4, #8]
        0x25,0xf0, 0x10,0x05,   //bic.w	r5, r5, #16
        0xa5,0x60,              //str	r5, [r4, #8]

// <wait_fifo>:
   	0xd0,0xf8, 0x00,0x80, 	//ldr.w	r8, [r0]
   	0xb8,0xf1, 0x00,0x0f, 	//cmp.w	r8, #0
        0x00,0xf0, 0x2a,0x80,   //beq.w <error>
   	0x47,0x68,      	//ldr	r7, [r0, #4]
  	0x47,0x45,      	//cmp	r7, r8
        0x3f,0xf4, 0xf6,0xaf,   //beq.w	4 <wait_fifo>

// <mainloop>:
  	0x22,0x60,      	//str	r2, [r4, #0]
  	0x02,0xf1, 0x10,0x02, 	//add.w	r2, r2, #16
  	0x57,0xf8, 0x04,0x8b, 	//ldr.w	r8, [r7], #4
  	0xc4,0xf8, 0x30,0x80, 	//str.w	r8, [r4, #48]	; 0x30
  	0x57,0xf8, 0x04,0x8b, 	//ldr.w	r8, [r7], #4
  	0xc4,0xf8, 0x34,0x80, 	//str.w	r8, [r4, #52]	; 0x34
	0x57,0xf8, 0x04,0x8b, 	//ldr.w	r8, [r7], #4
  	0xc4,0xf8, 0x38,0x80, 	//str.w	r8, [r4, #56]	; 0x38
  	0x57,0xf8, 0x04,0x8b, 	//ldr.w	r8, [r7], #4
  	0xc4,0xf8, 0x3c,0x80, 	//str.w	r8, [r4, #60]	; 0x3c
  	0xa5,0x68,      	//ldr	r5, [r4, #8]
  	0x45,0xf0, 0x01,0x05, 	//orr.w	r5, r5, #1
  	0xa5,0x60,      	//str	r5, [r4, #8]

// <busy>:
  	0xd4,0xf8, 0x08,0x80, 	//ldr.w	r8, [r4, #8]
  	0x18,0xf0, 0x01,0x0f, 	//tst.w	r8, #1
        0x7f,0xf4, 0xfa,0xaf,   //bne.w <busy>
  	0x8f,0x42,      	//cmp	r7, r1
  	0x28,0xbf,      	//it	cs
  	0x00,0xf1, 0x08,0x07, 	//addcs.w	r7, r0, #8
  	0x47,0x60,      	//str	r7, [r0, #4]
  	0x04,0x3b,      	//subs	r3, #4  
  	0x1b,0xb1,      	//cbz	r3, <exit>
        0xff,0xf7, 0xd0,0xbf,   //b.w	 <wait_fifo>

// <error>:
        0x00,0x21,              //movs	r1, #0
        0x41,0x60,              //str	r1, [r0, #4]

// <exit>:
        0xa5,0x68,              //ldr	r5, [r4, #8]
        0x45,0xf0, 0x10,0x05,   //orr.w	r5, r5, #16
        0xa5,0x60,              //str	r5, [r4, #8]
  	0x00,0xbe,      	//bkpt	0x0000
        0x00,0xbf,              //nop

// <pFLASH_CTRL_BASE>:
  	0x00,0x00,0x00,0x40 	//.word	0x40000000
  };

/***************************************************************************
*	openocd command interface                                              *
***************************************************************************/

/* Config Command: flash bank name driver base size chip_width bus_width target [driver_options]
      flash bank maxim128 <base> <size> 0 0 <target> <FLC base> <sector size> <clkdiv> [burst_bits]
 */
FLASH_BANK_COMMAND_HANDLER(maxim128_flash_bank_command)
{
	struct maxim128_flash_bank *maxim_info;

	if (CMD_ARGC < 9) {
		LOG_WARNING("incomplete flash bank maxim128 configuration: <base> <size> 0 0 <target> <FLC base> <sector size> <clkdiv> [burst_bits]");
		return ERROR_FLASH_BANK_INVALID;
	}

	maxim_info = calloc(sizeof(struct maxim128_flash_bank), 1);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], maxim_info->flash_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], maxim_info->flc_base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], maxim_info->sector_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], maxim_info->clkdiv_value);

	if (CMD_ARGC > 9)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[9], maxim_info->burst_size_bits);
	else
		maxim_info->burst_size_bits = 32;
    
	maxim_info->int_state = 0;
	bank->driver_priv = maxim_info;

	/* Insert the flash controller base address into the write algorithm code */
	maxim128_write_code[sizeof(maxim128_write_code) - 4] = maxim_info->flc_base & 0xFF;
	maxim128_write_code[sizeof(maxim128_write_code) - 3] = (maxim_info->flc_base >> 8) & 0xFF;
	maxim128_write_code[sizeof(maxim128_write_code) - 2] = (maxim_info->flc_base >> 16) & 0xFF;
	maxim128_write_code[sizeof(maxim128_write_code) - 1] = (maxim_info->flc_base >> 24) & 0xFF;

	return ERROR_OK;
}

static int get_maxim_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed;
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;

	if (maxim_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	printed = snprintf(buf, buf_size, "\nMaxim Integrated\n");
	buf += printed;
	buf_size -= printed;

	return ERROR_OK;
}

/***************************************************************************
*	target_read_u32 wrapper
***************************************************************************/

static int target_read_u32_validate(struct target *target, uint32_t address, uint32_t *value, uint32_t valid_mask, uint32_t valid_value)
{
	int retval;
	uint32_t tempvalue;

	retval = target_read_u32(target, address, &tempvalue);

	if (retval != ERROR_OK) {
		LOG_WARNING("target_read_u32() failed with %d", retval);
		return retval;
	} else if ( (tempvalue & valid_mask) == (valid_value & valid_mask) ) {
		*value = tempvalue;
		return retval;
	}

	LOG_WARNING("target_read_u32/validate value invalid (0x%08x,0x%08x,0x%08x,0x%08x)", 
        tempvalue,valid_value,valid_mask,address);

	return ERROR_FAIL;
}

/***************************************************************************
*	flash operations                                                       *
***************************************************************************/

static int maxim128_flash_op_pre(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;
	uint32_t flsh_cn;

	/* Check if the flash controller is busy */
	if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_FLASH_BUSY, 0) != ERROR_OK)
		return ERROR_FAIL;
		
	if (flsh_cn & (FLSH_CN_PEND | FLSH_CN_ERASE_CODE_MASK | FLSH_CN_PGE | FLSH_CN_ME | FLSH_CN_WR))
		return ERROR_FLASH_BUSY;

	/* Refresh flash controller timing */
	target_write_u32(target, maxim_info->flc_base + FLSH_CLKDIV, maxim_info->clkdiv_value);

	/* Clear and disable flash programming interrupts */
	if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_INT, &maxim_info->int_state, MASK_DISABLE_INTS, 0) != ERROR_OK)
		return ERROR_FAIL;
		
	target_write_u32(target, maxim_info->flc_base + FLSH_INT, 0x00000000);

	/* Unlock flash */
	flsh_cn &= ~FLSH_CN_UNLOCK_MASK;
	flsh_cn |= FLSH_CN_UNLOCK_VALUE;
	target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

	/* Confirm flash is unlocked */
	if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_FLASH_UNLOCKED, FLSH_CN_UNLOCK_VALUE) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int maxim128_flash_op_post(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;
	uint32_t flsh_cn;

	/* Restore flash programming interrupts */
	target_write_u32(target, maxim_info->flc_base + FLSH_INT, maxim_info->int_state);

	/* Lock flash */
	if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_FLASH_LOCK, FLSH_CN_UNLOCK_VALUE) != ERROR_OK)
		return ERROR_FAIL;
	flsh_cn &= ~FLSH_CN_UNLOCK_MASK;
	target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

	return ERROR_OK;
}

static int maxim128_protect_check(struct flash_bank *bank)
{
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;
	int i;

	if (maxim_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	for (i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = -1;

/* TODO */
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

static int maxim128_erase(struct flash_bank *bank, int first, int last)
{
	int banknr;
	uint32_t flsh_cn, flsh_int;
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;
	int retry;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (maxim_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1))) {
		return maxim128_mass_erase(bank);
	}

	/* Prepare to issue flash operation */
	retval = maxim128_flash_op_pre(bank);
	if (retval != ERROR_OK)
		return retval;

	for (banknr = first; banknr <= last; banknr++) {
		/* Address is first word in page */
		target_write_u32(target, maxim_info->flc_base + FLSH_ADDR, banknr * maxim_info->sector_size);

		/* Write page erase code */
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_FLASH_ERASE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
		
		flsh_cn |= FLSH_CN_ERASE_CODE_PGE;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		/* Issue page erase command */
		flsh_cn |= 0x4;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		/* Wait until erase complete */
		retry = 1000;
		do {
			if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_FLASH_ERASED, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
				maxim128_flash_op_post(bank);
				return ERROR_FAIL;
			}
		} while ( (--retry > 0) && (flsh_cn & FLSH_CN_PEND));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash page erase @ 0x%08x", banknr * maxim_info->sector_size);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		/* Check access violations */
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_INT, &flsh_int, MASK_ACCESS_VIOLATIONS, 0) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
		
		if (flsh_int & FLSH_INT_AF) {
			LOG_ERROR("Error erasing flash page %i", banknr);
			target_write_u32(target, maxim_info->flc_base + FLSH_INT, 0);
			maxim128_flash_op_post(bank);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bank->sectors[banknr].is_erased = 1;
	}

	if (maxim128_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int maxim128_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (maxim_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

/* TODO */
	return ERROR_FLASH_OPER_UNSUPPORTED;
}
//#if 0
static int maxim128_write_block(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t wcount)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384+8;
	struct working_area *source;
	struct working_area *write_algorithm;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	int pad = 0;
// FIXME - Fails for > 128 bytes in uppoer flash 
return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* power of two, and multiple of word size */
	static const unsigned buf_min = 128+8;

	/* for small buffers it's faster not to download an algorithm */
	if ((wcount+pad) * 16 < buf_min)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(maxim128_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* plus a buffer big enough for this data */
	if ((wcount+pad) * 16 < buffer_size)
		buffer_size = (wcount+pad) * 16;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
                buffer_size &= 0x0f; /* aligned to flash width */
                buffer_size += 8; /* add params */
		if (buffer_size <= buf_min) {
			target_free_working_area(target, write_algorithm);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		LOG_DEBUG("retry target_alloc_working_area(%s, size=%u)",
				target_name(target), (unsigned) buffer_size);
	};

	target_write_buffer(target, write_algorithm->address,
			sizeof(maxim128_write_code),
			(uint8_t *) maxim128_write_code);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[3].value, 0, 32, wcount);

	retval = target_run_flash_async_algorithm(target, buffer, wcount, 16,
			0, NULL,
			4, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("error %d executing maxim128 flash write algorithm", retval);

	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}
//#endif
static int maxim128_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flsh_cn, flsh_int;
	uint32_t address = offset;
	uint32_t remaining = count;
	uint32_t words_remaining;
	int retval;
	int retry;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (maxim_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (offset & 0xF) {
		LOG_WARNING("offset size must be word aligned");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	/* Prepare to issue flash operation */
	retval = maxim128_flash_op_pre(bank);
	
	if (retval != ERROR_OK)
		return retval;

	if (remaining >= 16) {
		// write in 32-bit units
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_FLASH_WRITE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
		flsh_cn &= 0xF7FFFFFF;
		flsh_cn &= ~0x00000010;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		/* try using a block write */
		words_remaining = remaining / 16;
		retval = maxim128_write_block(bank, buffer, offset,
				words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				LOG_DEBUG("writing flash word-at-a-time");
			} else {
				maxim128_flash_op_post(bank);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		} else {
			// all 128-bit words have been written
			buffer += words_remaining * 16;
			address += words_remaining * 16;
			remaining -= words_remaining * 16;
		}
	}

	if ( (remaining >= 16) && ((address & 0x7F) != 0) ) {

		// write in 32-bit units until we are 128-bit aligned
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_WRITE_ALIGNED, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
		
		flsh_cn &= 0xF7FFFFFF;
		flsh_cn &= ~0x00000010;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		while ( (remaining >= 16) && ((address & 0x7F) != 0) ) {
			target_write_u32(target, maxim_info->flc_base + FLSH_ADDR, address);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0, 4, buffer);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+4, 4, buffer+4);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+8, 4, buffer+8);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+12, 4, buffer+12);
			flsh_cn |= 0x00000001;
			target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

			/* Wait until flash operation is complete */
			retry = 10;
			do {
				if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_WRITE_COMPLETE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
					maxim128_flash_op_post(bank);
					return ERROR_FAIL;
				}
			} while ( (--retry > 0) && (flsh_cn & FLSH_CN_PEND));

			if (retry <= 0) {
				LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			buffer += 16;
			address += 16;
			remaining -= 16;
		}
	}

    if ( (maxim_info->burst_size_bits == 128) && (remaining >= 16) ) {
		// write in 128-bit bursts while we can
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_WRITE_BURST, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
		
		flsh_cn &= 0xFFFFFFEF;
		flsh_cn |= 0x08000000;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		target_write_u32(target, maxim_info->flc_base + FLSH_ADDR, address);

		while (remaining >= 16) {

			if ((address & 0xFFF) == 0) {
				LOG_DEBUG("Writing @ 0x%08x", address);
			}

			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0, 4, buffer);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+4, 4, buffer+4);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+8, 4, buffer+8);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+12, 4, buffer+12);
			flsh_cn |= 0x00000001;
			target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

			/* Wait until flash operation is complete */
			retry = 10;
			do {
				if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_BURST_COMPLETE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
					maxim128_flash_op_post(bank);
					return ERROR_FAIL;
				}
			} while ( (--retry > 0) && (flsh_cn & FLSH_CN_PEND));

			if (retry <= 0) {
				LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			buffer += 16;
			address += 16;
			remaining -= 16;
		}
	}

	if (remaining >= 16) {
		// write in 32-bit units while we can
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_WRITE_REMAINING, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
		
		flsh_cn &= 0xF7FFFFFF;
		flsh_cn &= ~0x00000010;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		while (remaining >= 16) {
			target_write_u32(target, maxim_info->flc_base + FLSH_ADDR, address);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0, 4, buffer);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+4, 4, buffer+4);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+8, 4, buffer+8);
			target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+12, 4, buffer+12);
			flsh_cn |= 0x00000001;
			target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

			/* Wait until flash operation is complete */
			retry = 10;
			do {
				if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_REMAINING_COMPLETE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
					maxim128_flash_op_post(bank);
					return ERROR_FAIL;
				}
			} while ( (--retry > 0) && (flsh_cn & FLSH_CN_PEND));

			if (retry <= 0) {
				LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			buffer += 16;
			address += 16;
			remaining -= 16;
		}
	}

	if (remaining > 0) {
		// write remaining bytes in a 32-bit unit
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_WRITE_REMAINING, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
		
		flsh_cn &= 0xF7FFFFFF;
		flsh_cn &= ~0x00000010;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		uint8_t last_words[16] = {0xff, 0xff, 0xff, 0xff,
                                  0xff, 0xff, 0xff, 0xff,
                                  0xff, 0xff, 0xff, 0xff,
                                  0xff, 0xff, 0xff, 0xff};
		int i = 0;

		while (remaining > 0) {
			last_words[i++] = *buffer;
			buffer++;
			remaining--;
		}

		target_write_u32(target, maxim_info->flc_base + FLSH_ADDR, address);
		target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0, 4, last_words);
		target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+4, 4, last_words+4);
		target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+8, 4, last_words+8);
		target_write_buffer(target, maxim_info->flc_base + FLSH_DATA0+12, 4, last_words+12);
		flsh_cn |= 0x00000001;
		target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

		/* Wait until flash operation is complete */
		retry = 10;
		do {
			if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_WRITE_COMPLETE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
				maxim128_flash_op_post(bank);
				return ERROR_FAIL;
			}
		} while ( (--retry > 0) && (flsh_cn & FLSH_CN_PEND));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	
	flsh_cn |= 0x00000010; // reset to 32 bits
	target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

	/* Check access violations */
	if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_INT, &flsh_int, MASK_ACCESS_VIOLATIONS, 0) != ERROR_OK) {
		maxim128_flash_op_post(bank);
		return ERROR_FAIL;
	}
	
	if (flsh_int & FLSH_INT_AF) {
		LOG_ERROR("Flash Error writing 0x%x bytes at 0x%08x", count, offset);
		maxim128_flash_op_post(bank);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (maxim128_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int maxim128_probe(struct flash_bank *bank)
{
	struct maxim128_flash_bank *maxim_info = bank->driver_priv;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* provide this for the benefit of the NOR flash framework */
	bank->size = maxim_info->flash_size;
	bank->num_sectors = maxim_info->flash_size / maxim_info->sector_size;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));

	for (int i = 0; i < bank->num_sectors; i++)	{
		bank->sectors[i].offset = i * maxim_info->sector_size;
		bank->sectors[i].size = maxim_info->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	maxim_info->probed = 1;

	return ERROR_OK;
}

static int maxim128_mass_erase(struct flash_bank *bank)
{
	struct target *target = NULL;
	struct maxim128_flash_bank *maxim_info = NULL;
	uint32_t flsh_cn, flsh_int;
	int retval;
	int retry;

	maxim_info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED)	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (maxim_info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* Prepare to issue flash operation */
	retval = maxim128_flash_op_pre(bank);
	
	if (retval != ERROR_OK)
		return retval;

	/* Write mass erase code */
	if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_MASS_ERASE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
		maxim128_flash_op_post(bank);
		return ERROR_FAIL;
	}
	
	flsh_cn |= FLSH_CN_ERASE_CODE_ME;
	target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

	/* Issue mass erase command */
	flsh_cn |= 0x2;
	target_write_u32(target, maxim_info->flc_base + FLSH_CN, flsh_cn);

	/* Wait until erase complete */
	retry = 1000;
	do {
		if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_CN, &flsh_cn, MASK_ERASE_COMPLETE, FLSH_CN_UNLOCK_VALUE) != ERROR_OK) {
			maxim128_flash_op_post(bank);
			return ERROR_FAIL;
		}
	} while ( (--retry > 0) && (flsh_cn & FLSH_CN_PEND));

	if (retry <= 0) {
		LOG_ERROR("Timed out waiting for flash mass erase");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check access violations */
	if (target_read_u32_validate(target, maxim_info->flc_base + FLSH_INT, &flsh_int, MASK_ACCESS_VIOLATIONS, 0) != ERROR_OK) {
		maxim128_flash_op_post(bank);
		return ERROR_FAIL;
	}
	
	if (flsh_int & FLSH_INT_AF) {
		LOG_ERROR("Error mass erasing");
		target_write_u32(target, maxim_info->flc_base + FLSH_INT, 0);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (maxim128_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(maxim128_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "maxim128 mass_erase <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (maxim128_mass_erase(bank) == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "maxim128 mass erase complete");
	} else {
		command_print(CMD_CTX, "maxim128 mass erase failed");
	}

	return ERROR_OK;
}

static const struct command_registration maxim128_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = maxim128_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "erase entire device",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration maxim128_command_handlers[] = {
	{
		.name = "maxim128",
		.mode = COMMAND_EXEC,
		.help = "Maxim flash command group",
		.chain = maxim128_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver maxim128_flash = {
	.name = "maxim128",
	.commands = maxim128_command_handlers,
	.flash_bank_command = maxim128_flash_bank_command,
	.erase = maxim128_erase,
	.protect = maxim128_protect,
	.write = maxim128_write,
	.read = default_flash_read,
	.probe = maxim128_probe,
	.auto_probe = maxim128_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = maxim128_protect_check,
	.info = get_maxim_info,
};
