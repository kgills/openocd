/***************************************************************************
 *   Copyright (C) 2016 by Maxim Integrated                                *
 *   Kevin Gillespie <kevin.gillespie@maximintegrated.com>                 *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <target/algorithm.h>
#include <target/armv7m.h>

/* Register addresses */
#define FLC_ADDR				0x00000000
#define FLC_CLKDIV				0x00000004
#define FLC_CN					0x00000008
#define FLC_PR1E_ADDR			0x0000000C
#define FLC_PR2S_ADDR			0x00000010
#define FLC_PR2E_ADDR			0x00000014
#define FLC_PR3S_ADDR			0x00000018
#define FLC_PR3E_ADDR			0x0000001C
#define FLC_MD					0x00000020
#define FLC_INT					0x00000024
#define FLC_DATA0				0x00000030
#define FLC_DATA1				0x00000034
#define FLC_DATA2				0x00000038
#define FLC_DATA3				0x0000003C
#define FLC_BL_CTRL				0x00000170
#define FLC_PROT				0x00000300

#define ARM_PID_REG				0xE00FFFE0
#define MAX326XX_ID_REG			0x40000838

/* Register settings */
#define FLC_INT_AF				0x00000002

#define FLC_CN_UNLOCK_MASK		0xF0000000
#define FLC_CN_UNLOCK_VALUE		0x20000000

#define FLC_CN_PEND				0x01000000
#define FLC_CN_ERASE_CODE_MASK	0x0000FF00
#define FLC_CN_ERASE_CODE_PGE	0x00005500
#define FLC_CN_ERASE_CODE_ME	0x0000AA00
#define FLC_CN_32BIT			0x00000010
#define FLC_CN_PGE				0x00000004
#define FLC_CN_ME				0x00000002
#define FLC_CN_WR				0x00000001
#define FLC_CN_PGE				0x00000004
#define FLC_CN_ME				0x00000002
#define FLC_CN_WR				0x00000001

#define FLC_BL_CTRL_23			0x00020000
#define FLC_BL_CTRL_IFREN		0x00000001

#define MASK_FLASH_BUSY			(0x048800E0 & ~0x04000000)
#define MASK_DISABLE_INTS		(0xFFFFFCFC)
#define MASK_FLASH_UNLOCKED		(0xF588FFEF & ~0x04000000)
#define MASK_FLASH_LOCK			(0xF588FFEF & ~0x04000000)
#define MASK_FLASH_ERASE		(0xF588FFEF & ~0x04000000)
#define MASK_FLASH_ERASED		(0xF48800EB & ~0x04000000)
#define MASK_ACCESS_VIOLATIONS	(0xFFFFFCFC)
#define MASK_FLASH_WRITE		(0xF588FFEF & ~0x04000000)
#define MASK_WRITE_ALIGNED		(0xF588FFEF & ~0x04000000)
#define MASK_WRITE_COMPLETE		(0xF488FFEE & ~0x04000000)
#define MASK_WRITE_BURST		(0xF588FFEF & ~0x04000000)
#define MASK_BURST_COMPLETE		(0xF488FFEE & ~0x04000000)
#define MASK_WRITE_REMAINING	(0xF588FFEF & ~0x04000000)
#define MASK_REMAINING_COMPLETE	(0xF488FFEE & ~0x04000000)
#define MASK_MASS_ERASE			(0xF588FFEF & ~0x04000000)
#define MASK_ERASE_COMPLETE		(0xF48800ED & ~0x04000000)

#define ARM_PID_DEFAULT_CM3		0x0000B4C3
#define ARM_PID_DEFAULT_CM4		0x0000B4C4
#define MAX326XX_ID				0x0000004D

#define WRITE32BIT				0
#define WRITE128BIT				1

static int max32xxx_mass_erase(struct flash_bank *bank);

struct max32xxx_flash_bank {
	int probed;
	int max326xx;
	unsigned int flash_size;
	unsigned int flc_base;
	unsigned int sector_size;
	unsigned int clkdiv_value;
	unsigned int int_state;
	unsigned int burst_size_bits;
};

/* see contib/loaders/flash/max32xxx/max32xxx.s for src */
static const uint8_t write_code[] = {
#include "../../contrib/loaders/flash/max32xxx/max32xxx.inc"
};

/*	Config Command: flash bank name driver base size chip_width bus_width target [driver_option]
	flash bank max32xxx <base> <size> 0 0 <target> <FLC base> <sector size> <clkdiv> [burst_bits]
 */
FLASH_BANK_COMMAND_HANDLER(max32xxx_flash_bank_command)
{
	struct max32xxx_flash_bank *info;

	if (CMD_ARGC < 9) {
		LOG_ERROR("incorrect flash bank max32xxx configuration: <base> <size> 0 0 <target> <FLC base> <sector size> <clkdiv> [burst_bits]");
		return ERROR_FLASH_BANK_INVALID;
	} else if (CMD_ARGC > 10) {
		LOG_ERROR("incorrect flash bank max32xxx configuration: <base> <size> 0 0 <target> <FLC base> <sector size> <clkdiv> [burst_bits]");
		return ERROR_FLASH_BANK_INVALID;
	}

	info = calloc(sizeof(struct max32xxx_flash_bank), 1);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], info->flash_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->flc_base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], info->sector_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], info->clkdiv_value);

	if (CMD_ARGC == 10)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[9], info->burst_size_bits);
	else {
		/* Default burst size of 32 bits */
		info->burst_size_bits = 32;
	}

	if ((info->burst_size_bits != 128) && (info->burst_size_bits != 32)) {
		LOG_ERROR("Invalid burst size %d, must be 32 or 128", info->burst_size_bits);
		return ERROR_FLASH_BANK_INVALID;
	}

	info->int_state = 0;
	bank->driver_priv = info;
	return ERROR_OK;
}

static int get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int printed;
	struct max32xxx_flash_bank *info = bank->driver_priv;

	if (info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	printed = snprintf(buf, buf_size, "\nMaxim Integrated max32xxx flash driver\n");
	buf += printed;
	buf_size -= printed;
	return ERROR_OK;
}

static int max32xxx_flash_busy(uint32_t flash_cn)
{
	if (flash_cn & (FLC_CN_PGE | FLC_CN_ME | FLC_CN_WR))
		return ERROR_FLASH_BUSY;

	return ERROR_OK;
}

static int max32xxx_flash_op_pre(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct max32xxx_flash_bank *info = bank->driver_priv;
	uint32_t flash_cn;
	uint32_t bootloader;

	/* Check if the flash controller is busy */
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	if (max32xxx_flash_busy(flash_cn))
		return ERROR_FLASH_BUSY;

	/* Refresh flash controller timing */
	target_write_u32(target, info->flc_base + FLC_CLKDIV, info->clkdiv_value);

	/* Clear and disable flash programming interrupts */
	target_read_u32(target, info->flc_base + FLC_INT, &info->int_state);
	target_write_u32(target, info->flc_base + FLC_INT, 0);

	/* Clear the lower bit in the bootloader configuration register in case flash page 0 has been replaced */
	if (target_read_u32(target, info->flc_base + FLC_BL_CTRL, &bootloader) != ERROR_OK) {
		LOG_ERROR("Read failure on FLC_BL_CTRL");
		return ERROR_FAIL;
	}
	if (bootloader & FLC_BL_CTRL_23) {
		LOG_WARNING("FLC_BL_CTRL indicates BL mode 2 or mode 3.");
		if (bootloader & FLC_BL_CTRL_IFREN) {
			LOG_WARNING("Flash page 0 swapped out, attempting to swap back in for programming");
			bootloader &= ~(FLC_BL_CTRL_IFREN);
			if (target_write_u32(target, info->flc_base + FLC_BL_CTRL, bootloader) != ERROR_OK) {
				LOG_ERROR("Write failure on FLC_BL_CTRL");
				return ERROR_FAIL;
			}
			if (target_read_u32(target, info->flc_base + FLC_BL_CTRL, &bootloader) != ERROR_OK) {
				LOG_ERROR("Read failure on FLC_BL_CTRL");
				return ERROR_FAIL;
			}
			if (bootloader & FLC_BL_CTRL_IFREN) {
				LOG_ERROR("Unable to swap flash page 0 back in. Writes to page 0 will fail.");
			}
		}
	}

	/* Unlock flash */
	flash_cn &= ~(FLC_CN_UNLOCK_MASK);
	flash_cn |= FLC_CN_UNLOCK_VALUE;
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

	/* Confirm flash is unlocked */
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	if ((flash_cn & FLC_CN_UNLOCK_VALUE) != FLC_CN_UNLOCK_VALUE)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int max32xxx_flash_op_post(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct max32xxx_flash_bank *info = bank->driver_priv;
	uint32_t flash_cn;

	/* Restore flash programming interrupts */
	target_write_u32(target, info->flc_base + FLC_INT, info->int_state);

	/* Lock flash */
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	flash_cn &= ~(FLC_CN_UNLOCK_MASK);
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);
	return ERROR_OK;
}

static int max32xxx_protect_check(struct flash_bank *bank)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int i;
	uint32_t temp_reg;

	if (info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (!info->max326xx) {
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_protected = -1;

		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	/* Check the protection */
	for (i = 0; i < bank->num_sectors; i++) {
		if (i%32 == 0)
			target_read_u32(target, info->flc_base + FLC_PROT + ((i/32)*4), &temp_reg);

		if (temp_reg & (0x1 << i%32))
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}
	return ERROR_OK;
}

static int max32xxx_erase(struct flash_bank *bank, int first, int last)
{
	int banknr;
	uint32_t flash_cn, flash_int;
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;
	int retry;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	if ((first == 0) && (last == (bank->num_sectors - 1)))
		return max32xxx_mass_erase(bank);

	/* Prepare to issue flash operation */
	retval = max32xxx_flash_op_pre(bank);

	if (retval != ERROR_OK)
		return retval;

	int erased = 0;
	for (banknr = first; banknr <= last; banknr++) {

		/* Check the protection */
		if (bank->sectors[banknr].is_protected == 1) {
			LOG_WARNING("Flash sector %d is protected", banknr);
			continue;
		} else
			erased = 1;

		/* Address is first word in page */
		target_write_u32(target, info->flc_base + FLC_ADDR, banknr * info->sector_size);

		/* Write page erase code */
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn |= FLC_CN_ERASE_CODE_PGE;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		/* Issue page erase command */
		flash_cn |= FLC_CN_PGE;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		/* Wait until erase complete */
		retry = 1000;
		do {
			target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash page erase @ 0x%08x",
				banknr * info->sector_size);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		/* Check access violations */
		target_read_u32(target, info->flc_base + FLC_INT, &flash_int);
		if (flash_int & FLC_INT_AF) {
			LOG_ERROR("Error erasing flash page %i", banknr);
			target_write_u32(target, info->flc_base + FLC_INT, 0);
			max32xxx_flash_op_post(bank);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bank->sectors[banknr].is_erased = 1;
	}

	if (!erased) {
		LOG_ERROR("All pages protected %d to %d", first, last);
		max32xxx_flash_op_post(bank);
		return ERROR_FAIL;
	}

	if (max32xxx_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int max32xxx_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int page;
	uint32_t temp_reg;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (!info->max326xx)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	/* Setup the protection on the pages given */
	for (page = first; page <= last; page++) {
		if (set) {
			/* Set the write/erase bit for this page */
			target_read_u32(target, info->flc_base + FLC_PROT + (page/32), &temp_reg);
			temp_reg |= (0x1 << page%32);
			target_write_u32(target, info->flc_base + FLC_PROT + (page/32), temp_reg);
			bank->sectors[page].is_protected = 1;
		} else {
			/* Clear the write/erase bit for this page */
			target_read_u32(target, info->flc_base + FLC_PROT + (page/32), &temp_reg);
			temp_reg &= ~(0x1 << page%32);
			target_write_u32(target, info->flc_base + FLC_PROT + (page/32), temp_reg);
			bank->sectors[page].is_protected = 0;
		}
	}

	return ERROR_OK;
}

static int max32xxx_write_block(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t wcount)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *source;
	struct working_area *write_algorithm;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	/* power of two, and multiple of word size */
	static const unsigned buf_min = 128;

	/* for small buffers it's faster not to download an algorithm */
	if (wcount * 4 < buf_min)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	LOG_DEBUG("(bank=%p buffer=%p offset=%08" PRIx32 " wcount=%08" PRIx32 "",
		bank, buffer, offset, wcount);

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(write_code), &write_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* plus a buffer big enough for this data */
	if (wcount * 4 < buffer_size)
		buffer_size = wcount * 4;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;

		if (buffer_size <= buf_min) {
			target_free_working_area(target, write_algorithm);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		LOG_DEBUG("retry target_alloc_working_area(%s, size=%u)",
			target_name(target), (unsigned) buffer_size);
	}

	target_write_buffer(target, write_algorithm->address, sizeof(write_code),
		write_code);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_OUT);
	init_reg_param(&reg_params[5], "r5", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, address);
	buf_set_u32(reg_params[4].value, 0, 32, info->flc_base);

	if (info->burst_size_bits == 32) {
		buf_set_u32(reg_params[3].value, 0, 32, wcount);
		buf_set_u32(reg_params[5].value, 0, 32, WRITE32BIT);
	} else {
		buf_set_u32(reg_params[3].value, 0, 32, wcount/4);
		buf_set_u32(reg_params[5].value, 0, 32, WRITE128BIT);
	}

	retval = target_run_flash_async_algorithm(target, buffer, wcount, 4, 0, NULL,
		6, reg_params, source->address, source->size, write_algorithm->address, 0, &armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("error %d executing max32xxx flash write algorithm", retval);

	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);
	return retval;
}

static int max32xxx_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flash_cn, flash_int;
	uint32_t address = offset;
	uint32_t remaining = count;
	uint32_t words_remaining;
	int retval;
	int retry;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("bank=%p buffer=%p offset=%08" PRIx32 " count=%08" PRIx32 "",
		bank, buffer, offset, count);

	if (info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (info->burst_size_bits == 32) {
		if (offset & 0x3) {
			LOG_WARNING("offset size must be 32-bit aligned");
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
	} else {
		if (offset & 0xF) {
			LOG_ERROR("offset size must be 128-bit aligned");
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	/* Prepare to issue flash operation */
	retval = max32xxx_flash_op_pre(bank);

	if (retval != ERROR_OK)
		return retval;

	if (remaining >= 4) {
		/* try using a block write */

		/* 128-bit align the words_remaining */
		words_remaining = remaining / 4;
		words_remaining -= words_remaining % 4;

		retval = max32xxx_write_block(bank, buffer, offset, words_remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
				LOG_DEBUG("writing flash word-at-a-time");
			else {
				max32xxx_flash_op_post(bank);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		} else {
			/* all words_remaining have been written */
			buffer += words_remaining * 4;
			address += words_remaining * 4;
			remaining -= words_remaining * 4;
		}
	}

	if ((info->burst_size_bits == 32) && (remaining >= 4)) {
		/* write in 32-bit units*/
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn |= FLC_CN_32BIT;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		while (remaining >= 4) {
			target_write_u32(target, info->flc_base + FLC_ADDR, address);
			target_write_buffer(target, info->flc_base + FLC_DATA0, 4, buffer);
			flash_cn |= FLC_CN_WR;
			target_write_u32(target, info->flc_base + FLC_CN, flash_cn);
			/* Wait until flash operation is complete */
			retry = 10;

			do {
				target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
			} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

			if (retry <= 0) {
				LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			buffer += 4;
			address += 4;
			remaining -= 4;
		}
	}

	if ((info->burst_size_bits == 128) && (remaining >= 16)) {
		/* write in 128-bit units */
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn &= ~(FLC_CN_32BIT);
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		while (remaining >= 16) {
			target_write_u32(target, info->flc_base + FLC_ADDR, address);
			
			if ((address & 0xFFF) == 0)
				LOG_DEBUG("Writing @ 0x%08x", address);

			target_write_buffer(target, info->flc_base + FLC_DATA0, 16, buffer);
			flash_cn |= FLC_CN_WR;
			target_write_u32(target, info->flc_base + FLC_CN, flash_cn);
			/* Wait until flash operation is complete */
			retry = 10;

			do {
				target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
			} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

			if (retry <= 0) {
				LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			buffer += 16;
			address += 16;
			remaining -= 16;
		}
	}

	if ((info->burst_size_bits == 32) && remaining > 0) {
		/* write remaining bytes in a 32-bit unit */
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn |= FLC_CN_32BIT;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		uint8_t last_word[4] = {0xFF, 0xFF, 0xFF, 0xFF};
		int i = 0;

		while (remaining > 0) {
			last_word[i++] = *buffer;
			buffer++;
			remaining--;
		}

		target_write_u32(target, info->flc_base + FLC_ADDR, address);
		target_write_buffer(target, info->flc_base + FLC_DATA0, 4, last_word);
		flash_cn |= FLC_CN_WR;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		/* Wait until flash operation is complete */
		retry = 10;

		do {
			target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	if ((info->burst_size_bits == 128) && remaining > 0) {
		/* write remaining bytes in a 128-bit unit */
		if (target_read_u32(target, info->flc_base + FLC_CN, &flash_cn) != ERROR_OK) {
			max32xxx_flash_op_post(bank);
			return ERROR_FAIL;
		}

		flash_cn &= ~(FLC_CN_32BIT);
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		uint8_t last_words[16] = {0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF,
			0xFF, 0xFF, 0xFF, 0xFF};

		int i = 0;

		while (remaining > 0) {
			last_words[i++] = *buffer;
			buffer++;
			remaining--;
		}

		target_write_u32(target, info->flc_base + FLC_ADDR, address);
		target_write_buffer(target, info->flc_base + FLC_DATA0, 4, last_words);
		target_write_buffer(target, info->flc_base + FLC_DATA0 + 4, 4, last_words + 4);
		target_write_buffer(target, info->flc_base + FLC_DATA0 + 8, 4, last_words + 8);
		target_write_buffer(target, info->flc_base + FLC_DATA0 + 12, 4, last_words + 12);
		flash_cn |= FLC_CN_WR;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		/* Wait until flash operation is complete */
		retry = 10;
		do {
			if (target_read_u32(target, info->flc_base + FLC_CN, &flash_cn) != ERROR_OK) {
				max32xxx_flash_op_post(bank);
				return ERROR_FAIL;
			}
		} while ((--retry > 0) && (flash_cn & FLC_CN_PEND));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash write @ 0x%08x", address);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	/* Check access violations */
	target_read_u32(target, info->flc_base + FLC_INT, &flash_int);
	if (flash_int & FLC_INT_AF) {
		LOG_ERROR("Flash Error writing 0x%x bytes at 0x%08x", count, offset);
		max32xxx_flash_op_post(bank);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (max32xxx_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int max32xxx_probe(struct flash_bank *bank)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t arm_id[2];
	uint16_t arm_pid;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	/* provide this for the benefit of the NOR flash framework */
	bank->size = info->flash_size;
	bank->num_sectors = info->flash_size / info->sector_size;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));

	for (int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * info->sector_size;
		bank->sectors[i].size = info->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	/* Probe to determine if this part is in the max326xx family */
	info->max326xx = 0;
	target_read_u32(target, ARM_PID_REG, &arm_id[0]);
	target_read_u32(target, ARM_PID_REG+4, &arm_id[1]);
	arm_pid = (arm_id[1] << 8) + arm_id[0];
	LOG_DEBUG("arm_pid = 0x%x", arm_pid);

	if ((arm_pid == ARM_PID_DEFAULT_CM3) || arm_pid == ARM_PID_DEFAULT_CM4) {
		uint32_t max326xx_id;
		target_read_u32(target, MAX326XX_ID_REG, &max326xx_id);
		LOG_DEBUG("max326xx_id = 0x%x", max326xx_id);
		max326xx_id = ((max326xx_id & 0xFF000000) >> 24);
		if (max326xx_id == MAX326XX_ID)
			info->max326xx = 1;
	}
	LOG_DEBUG("info->max326xx = %d", info->max326xx);

	/* Initialize the protection bits for each flash page */
	if (max32xxx_protect_check(bank) == ERROR_FLASH_OPER_UNSUPPORTED)
		LOG_DEBUG("Flash protection not supported on this device");

	info->probed = 1;
	return ERROR_OK;
}

static int max32xxx_mass_erase(struct flash_bank *bank)
{
	struct target *target = NULL;
	struct max32xxx_flash_bank *info = NULL;
	uint32_t flash_cn, flash_int;
	int retval;
	int retry;
	info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (info->probed == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	int not_protected = 0;
	for (int i = 0; i < bank->num_sectors; i++) {
		if (bank->sectors[i].is_protected == 1)
			LOG_WARNING("Flash sector %d is protected", i);
		else
			not_protected = 1;
	}

	if (!not_protected) {
		LOG_ERROR("All pages protected");
		return ERROR_FAIL;
	}

	/* Prepare to issue flash operation */
	retval = max32xxx_flash_op_pre(bank);

	if (retval != ERROR_OK)
		return retval;

	/* Write mass erase code */
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	flash_cn |= FLC_CN_ERASE_CODE_ME;
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

	/* Issue mass erase command */
	flash_cn |= FLC_CN_ME;
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

	/* Wait until erase complete */
	retry = 1000;
	do {
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

	if (retry <= 0) {
		LOG_ERROR("Timed out waiting for flash mass erase");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Check access violations */
	target_read_u32(target, info->flc_base + FLC_INT, &flash_int);
	if (flash_int & FLC_INT_AF) {
		LOG_ERROR("Error mass erasing");
		target_write_u32(target, info->flc_base + FLC_INT, 0);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (max32xxx_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_mass_erase_command)
{
	int i;
	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "max32xxx mass_erase <bank>");
		return ERROR_OK;
	}

	if (ERROR_OK != retval)
		return retval;

	if (max32xxx_mass_erase(bank) == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "max32xxx mass erase complete");
	} else
		command_print(CMD_CTX, "max32xxx mass erase failed");

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_protection_set_command)
{
	struct flash_bank *bank;
	int retval;
	struct max32xxx_flash_bank *info;
	uint32_t addr, len;

	if (CMD_ARGC != 3) {
		command_print(CMD_CTX, "max32xxx protection_set <bank> <addr> <size>");
		return ERROR_OK;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;
	info = bank->driver_priv;

	/* Convert the range to the page numbers */
	if (1 != sscanf(CMD_ARGV[1], "0x%"SCNx32, &addr)) {
		LOG_WARNING("Error parsing address");
		command_print(CMD_CTX, "max32xxx protection_set <bank> <addr> <size>");
		return ERROR_FAIL;
	}
	/* Mask off the top portion on the address */
	addr = (addr & 0x0FFFFFFF);

	if (1 != sscanf(CMD_ARGV[2], "0x%"SCNx32, &len)) {
		LOG_WARNING("Error parsing length");
		command_print(CMD_CTX, "max32xxx protection_set <bank> <addr> <size>");
		return ERROR_FAIL;
	}

	/* Check the address is in the range of the flash */
	if ((addr+len) >= info->flash_size)
		return ERROR_FLASH_SECTOR_INVALID;

	if (len == 0)
		return ERROR_OK;

	/* Convert the address and length to the page boundaries */
	addr = addr - (addr % info->sector_size);
	if (len % info->sector_size)
		len = len + info->sector_size - (len % info->sector_size);

	/* Convert the address and length to page numbers */
	addr = (addr / info->sector_size);
	len = addr + (len / info->sector_size) - 1;

	if (max32xxx_protect(bank, 1, addr, len) == ERROR_OK)
		command_print(CMD_CTX, "max32xxx protection set complete");
	else
		command_print(CMD_CTX, "max32xxx protection set failed");

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_protection_clr_command)
{
	struct flash_bank *bank;
	int retval;
	struct max32xxx_flash_bank *info;
	uint32_t addr, len;

	if (CMD_ARGC != 3) {
		command_print(CMD_CTX, "max32xxx protection_clr <bank> <addr> <size>");
		return ERROR_OK;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;
	info = bank->driver_priv;

	/* Convert the range to the page numbers */
	if (1 != sscanf(CMD_ARGV[1], "0x%"SCNx32, &addr)) {
		LOG_WARNING("Error parsing address");
		command_print(CMD_CTX, "max32xxx protection_clr <bank> <addr> <size>");
		return ERROR_FAIL;
	}
	/* Mask off the top portion on the address */
	addr = (addr & 0x0FFFFFFF);

	if (1 != sscanf(CMD_ARGV[2], "0x%"SCNx32, &len)) {
		LOG_WARNING("Error parsing length");
		command_print(CMD_CTX, "max32xxx protection_clr <bank> <addr> <size>");
		return ERROR_FAIL;
	}

	/* Check the address is in the range of the flash */
	if ((addr+len) >= info->flash_size)
		return ERROR_FLASH_SECTOR_INVALID;

	if (len == 0)
		return ERROR_OK;

	/* Convert the address and length to the page boundaries */
	addr = addr - (addr % info->sector_size);
	if (len % info->sector_size)
		len = len + info->sector_size - (len % info->sector_size);

	/* Convert the address and length to page numbers */
	addr = (addr / info->sector_size);
	len = addr + (len / info->sector_size) - 1;

	if (max32xxx_protect(bank, 0, addr, len) == ERROR_OK)
		command_print(CMD_CTX, "max32xxx protection clear complete");
	else
		command_print(CMD_CTX, "max32xxx protection clear failed");

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_protection_check_command)
{
	struct flash_bank *bank;
	int retval;
	struct max32xxx_flash_bank *info;
	int i;

	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "max32xxx protection_check <bank>");
		return ERROR_OK;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;
	info = bank->driver_priv;

	/* Update the protection array */
	retval = max32xxx_protect_check(bank);
	if (ERROR_OK != retval) {
		LOG_WARNING("Error updating the protection array");
		return retval;
	}

	LOG_WARNING("s:<sector number> a:<address> p:<protection bit>");
	for (i = 0; i < bank->num_sectors; i += 4) {
		LOG_WARNING("s:%03d a:0x%06x p:%d | s:%03d a:0x%06x p:%d | s:%03d a:0x%06x p:%d | s:%03d a:0x%06x p:%d",
			(i+0), (i+0)*info->sector_size, bank->sectors[(i+0)].is_protected,
			(i+1), (i+1)*info->sector_size, bank->sectors[(i+1)].is_protected,
			(i+2), (i+2)*info->sector_size, bank->sectors[(i+2)].is_protected,
			(i+3), (i+3)*info->sector_size, bank->sectors[(i+3)].is_protected);
	}

	return ERROR_OK;
}

static const struct command_registration max32xxx_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = max32xxx_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "mass erase flash",
	},
	{
		.name = "protection_set",
		.handler = max32xxx_handle_protection_set_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id addr size",
		.help = "set flash protection for address range",
	},
	{
		.name = "protection_clr",
		.handler = max32xxx_handle_protection_clr_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id addr size",
		.help = "clear flash protection for address range",
	},
	{
		.name = "protection_check",
		.handler = max32xxx_handle_protection_check_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "check flash protection",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration max32xxx_command_handlers[] = {
	{
		.name = "max32xxx",
		.mode = COMMAND_EXEC,
		.help = "max32xxx flash command group",
		.chain = max32xxx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver max32xxx_flash = {
	.name = "max32xxx",
	.commands = max32xxx_command_handlers,
	.flash_bank_command = max32xxx_flash_bank_command,
	.erase = max32xxx_erase,
	.protect = max32xxx_protect,
	.write = max32xxx_write,
	.read = default_flash_read,
	.probe = max32xxx_probe,
	.auto_probe = max32xxx_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = max32xxx_protect_check,
	.info = get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
