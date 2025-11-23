/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* System module for Chrome EC : NPCX hardware specific implementation */

#include "clock.h"
#include "clock_chip.h"
#include "common.h"
#include "console.h"
#include "cpu.h"
#include "gpio.h"
#include "hooks.h"
#include "host_command.h"
#include "hwtimer_chip.h"
#include "registers.h"
#include "rom_chip.h"
#include "system.h"
#include "system_chip.h"
#include "task.h"
#include "timer.h"
#include "util.h"
#include "watchdog.h"

/* Delay after writing TTC for value to latch */
#define MTC_TTC_LOAD_DELAY_US 250
#define MTC_ALARM_MASK     ((1 << 25) - 1)
#define MTC_WUI_GROUP      MIWU_GROUP_4
#define MTC_WUI_MASK       MASK_PIN7

/* ROM address of chip revision */
#define CHIP_REV_ADDR 0x00007FFC

/* Console output macros */
#define CPUTS(outstr) cputs(CC_SYSTEM, outstr)
#define CPRINTS(format, args...) cprints(CC_SYSTEM, format, ## args)
#define CPRINTF(format, args...) cprintf(CC_SYSTEM, format, ## args)

/*****************************************************************************/
/* Internal functions */

void system_watchdog_reset(void)
{
	/* Unlock & stop watchdog registers */
	NPCX_WDSDM = 0x87;
	NPCX_WDSDM = 0x61;
	NPCX_WDSDM = 0x63;

	/* Reset TWCFG */
	NPCX_TWCFG = 0;
	/* Select T0IN clock as watchdog prescaler clock */
	SET_BIT(NPCX_TWCFG, NPCX_TWCFG_WDCT0I);

	/* Clear watchdog reset status initially*/
	SET_BIT(NPCX_T0CSR, NPCX_T0CSR_WDRST_STS);

	/* Keep prescaler ratio timer0 clock to 1:1 */
	NPCX_TWCP = 0x00;

	/* Set internal counter and prescaler */
	NPCX_TWDT0 = 0x00;
	NPCX_WDCNT = 0x01;

	/* Disable interrupt */
	interrupt_disable();
	/* Reload and restart Timer 0*/
	SET_BIT(NPCX_T0CSR, NPCX_T0CSR_RST);
	/* Wait for timer is loaded and restart */
	while (IS_BIT_SET(NPCX_T0CSR, NPCX_T0CSR_RST))
		;
	/* Enable interrupt */
	interrupt_enable();
}

/* Return true if index is stored as a single byte in bbram */
static int bbram_is_byte_access(enum bbram_data_index index)
{
	return (index >= BBRM_DATA_INDEX_VBNVCNTXT &&
		index <  BBRM_DATA_INDEX_RAMLOG)
#ifdef CONFIG_USB_PD_DUAL_ROLE
		|| index == BBRM_DATA_INDEX_PD0
		|| index == BBRM_DATA_INDEX_PD1
#endif
		|| index == BBRM_DATA_INDEX_PANIC_FLAGS
	;
}

/* Check and clear BBRAM status on any reset */
void system_check_bbram_on_reset(void)
{
	if (IS_BIT_SET(NPCX_BKUP_STS, NPCX_BKUP_STS_IBBR)) {
		/*
		 * If the reset cause is not power-on reset and VBAT has ever
		 * dropped, print a warning message.
		 */
		if (IS_BIT_SET(NPCX_RSTCTL, NPCX_RSTCTL_VCC1_RST_SCRATCH) ||
			IS_BIT_SET(NPCX_RSTCTL, NPCX_RSTCTL_VCC1_RST_STS))
			CPRINTF("VBAT drop!\n");

		/* Clear IBBR bit */
		SET_BIT(NPCX_BKUP_STS, NPCX_BKUP_STS_IBBR);
	}
}

/* Check index is within valid BBRAM range and IBBR is not set */
static int bbram_valid(enum bbram_data_index index, int bytes)
{
	/* Check index */
	if (index < 0 || index + bytes > NPCX_BBRAM_SIZE)
		return 0;

	/* Check BBRAM is valid */
	if (IS_BIT_SET(NPCX_BKUP_STS, NPCX_BKUP_STS_IBBR)) {
		SET_BIT(NPCX_BKUP_STS, NPCX_BKUP_STS_IBBR);
		panic_printf("IBBR set: BBRAM corrupted!\n");
		return 0;
	}
	return 1;
}

/**
 * Read battery-backed ram (BBRAM) at specified index.
 *
 * @return The value of the register or 0 if invalid index.
 */
static uint32_t bbram_data_read(enum bbram_data_index index)
{
	uint32_t value = 0;
	int bytes = bbram_is_byte_access(index) ? 1 : 4;

	if (!bbram_valid(index, bytes))
		return 0;

	/* Read BBRAM */
	if (bytes == 4) {
		value += NPCX_BBRAM(index + 3);
		value = value << 8;
		value += NPCX_BBRAM(index + 2);
		value = value << 8;
		value += NPCX_BBRAM(index + 1);
		value = value << 8;
	}
	value += NPCX_BBRAM(index);

	return value;
}

/**
 * Write battery-backed ram (BBRAM) at specified index.
 *
 * @return nonzero if error.
 */
static int bbram_data_write(enum bbram_data_index index, uint32_t value)
{
	int bytes = bbram_is_byte_access(index) ? 1 : 4;

	if (!bbram_valid(index, bytes))
		return EC_ERROR_INVAL;

	/* Write BBRAM */
	NPCX_BBRAM(index) = value & 0xFF;
	if (bytes == 4) {
		NPCX_BBRAM(index + 1) = (value >> 8)  & 0xFF;
		NPCX_BBRAM(index + 2) = (value >> 16) & 0xFF;
		NPCX_BBRAM(index + 3) = (value >> 24) & 0xFF;
	}

	/* Wait for write-complete */
	return EC_SUCCESS;
}

/* Map idx to a returned BBRM_DATA_INDEX_*, or return -1 on invalid idx */
static int bbram_idx_lookup(enum system_bbram_idx idx)
{
	if (idx >= SYSTEM_BBRAM_IDX_VBNVBLOCK0 &&
	    idx <= SYSTEM_BBRAM_IDX_VBNVBLOCK15)
		return BBRM_DATA_INDEX_VBNVCNTXT +
		       idx - SYSTEM_BBRAM_IDX_VBNVBLOCK0;
#ifdef CONFIG_USB_PD_DUAL_ROLE
	if (idx == SYSTEM_BBRAM_IDX_PD0)
		return BBRM_DATA_INDEX_PD0;
	if (idx == SYSTEM_BBRAM_IDX_PD1)
		return BBRM_DATA_INDEX_PD1;
#endif
#ifdef CONFIG_VBOOT_EFS
	if (idx == SYSTEM_BBRAM_IDX_TRY_SLOT)
		return BBRM_DATA_INDEX_TRY_SLOT;
#endif
	return -1;
}

int system_get_bbram(enum system_bbram_idx idx, uint8_t *value)
{
	int bbram_idx = bbram_idx_lookup(idx);

	if (bbram_idx < 0)
		return EC_ERROR_INVAL;

	*value = bbram_data_read(bbram_idx);
	return EC_SUCCESS;
}

int system_set_bbram(enum system_bbram_idx idx, uint8_t value)
{
	int bbram_idx = bbram_idx_lookup(idx);

	if (bbram_idx < 0)
		return EC_ERROR_INVAL;

	return bbram_data_write(bbram_idx, value);
}

/* MTC functions */
uint32_t system_get_rtc_sec(void)
{
	/* Get MTC counter unit:seconds */
	uint32_t sec = NPCX_TTC;
	return sec;
}

void system_set_rtc(uint32_t seconds)
{
	/*
	 * Set MTC counter unit:seconds, write twice to ensure values
	 * latch to NVMem.
	 */
	NPCX_TTC = seconds;
	udelay(MTC_TTC_LOAD_DELAY_US);
	NPCX_TTC = seconds;
	udelay(MTC_TTC_LOAD_DELAY_US);
}

#ifdef CONFIG_CHIP_PANIC_BACKUP
/*
 * Following information from panic data is stored in BBRAM:
 *
 * index     |       data
 * ==========|=============
 *   36      |       MMFS
 *   40      |       HFSR
 *   44      |       BFAR
 *   48      |      LREG1
 *   52      |      LREG3
 *   56      |      LREG4
 *   60      |     reserved
 *
 * Above registers are chosen to be saved in case of panic because:
 * 1. MMFS, HFSR and BFAR seem to provide more information about the fault.
 * 2. LREG1, LREG3 and LREG4 store exception, reason and info in case of
 * software panic.
 */
#define BKUP_MMFS		(BBRM_DATA_INDEX_PANIC_BKUP + 0)
#define BKUP_HFSR		(BBRM_DATA_INDEX_PANIC_BKUP + 4)
#define BKUP_BFAR		(BBRM_DATA_INDEX_PANIC_BKUP + 8)
#define BKUP_LREG1		(BBRM_DATA_INDEX_PANIC_BKUP + 12)
#define BKUP_LREG3		(BBRM_DATA_INDEX_PANIC_BKUP + 16)
#define BKUP_LREG4		(BBRM_DATA_INDEX_PANIC_BKUP + 20)

#define BKUP_PANIC_DATA_VALID	(1 << 0)

void chip_panic_data_backup(void)
{
	struct panic_data *d = panic_get_data();

	if (!d)
		return;

	bbram_data_write(BKUP_MMFS, d->cm.mmfs);
	bbram_data_write(BKUP_HFSR, d->cm.hfsr);
	bbram_data_write(BKUP_BFAR, d->cm.dfsr);
	bbram_data_write(BKUP_LREG1, d->cm.regs[1]);
	bbram_data_write(BKUP_LREG3, d->cm.regs[3]);
	bbram_data_write(BKUP_LREG4, d->cm.regs[4]);
	bbram_data_write(BBRM_DATA_INDEX_PANIC_FLAGS, BKUP_PANIC_DATA_VALID);
}

static void chip_panic_data_restore(void)
{
	struct panic_data *d = PANIC_DATA_PTR;

	/* Ensure BBRAM is valid. */
	if (!bbram_valid(BKUP_MMFS, 4))
		return;

	/* Ensure Panic data in BBRAM is valid. */
	if (!(bbram_data_read(BBRM_DATA_INDEX_PANIC_FLAGS) &
	      BKUP_PANIC_DATA_VALID))
		return;

	memset(d, 0, sizeof(*d));
	d->magic = PANIC_DATA_MAGIC;
	d->struct_size = sizeof(*d);
	d->struct_version = 2;
	d->arch = PANIC_ARCH_CORTEX_M;

	d->cm.mmfs = bbram_data_read(BKUP_MMFS);
	d->cm.hfsr = bbram_data_read(BKUP_HFSR);
	d->cm.dfsr = bbram_data_read(BKUP_BFAR);

	d->cm.regs[1] = bbram_data_read(BKUP_LREG1);
	d->cm.regs[3] = bbram_data_read(BKUP_LREG3);
	d->cm.regs[4] = bbram_data_read(BKUP_LREG4);

	/* Reset panic data in BBRAM. */
	bbram_data_write(BBRM_DATA_INDEX_PANIC_FLAGS, 0);
}
#endif /* CONFIG_CHIP_PANIC_BACKUP */

void chip_save_reset_flags(int flags)
{
	bbram_data_write(BBRM_DATA_INDEX_SAVED_RESET_FLAGS, flags);
}

uint32_t chip_read_reset_flags(void)
{
	return bbram_data_read(BBRM_DATA_INDEX_SAVED_RESET_FLAGS);
}

/* Check reset cause */
void system_check_reset_cause(void)
{
	uint32_t hib_wake_flags = bbram_data_read(BBRM_DATA_INDEX_WAKE);
	uint32_t flags = bbram_data_read(BBRM_DATA_INDEX_SAVED_RESET_FLAGS);

	/* Clear saved reset flags in bbram */
#ifdef CONFIG_POWER_BUTTON_INIT_IDLE
	/* We'll clear AP_OFF on S5->S3 transition */
	chip_save_reset_flags(flags & RESET_FLAG_AP_OFF);
#else
	chip_save_reset_flags(0);
#endif
	/* Clear saved hibernate wake flag in bbram , too */
	bbram_data_write(BBRM_DATA_INDEX_WAKE, 0);

	/* Use scratch bit to check power on reset or VCC1_RST reset */
	if (!IS_BIT_SET(NPCX_RSTCTL, NPCX_RSTCTL_VCC1_RST_SCRATCH)) {
#if defined(BOARD_WHEATLEY) || defined(BOARD_EVE) || defined(BOARD_POPPY) || defined(BOARD_SORAKA)\
	|| defined(BOARD_NAUTILUS) || defined(BOARD_NAMI)

		/* TODO(crosbug.com/p/61028): Remove workaround for Eve */
		flags |= RESET_FLAG_RESET_PIN;
#else
		/* Check for VCC1 reset */
		if (IS_BIT_SET(NPCX_RSTCTL, NPCX_RSTCTL_VCC1_RST_STS))
			flags |= RESET_FLAG_RESET_PIN;
		else
			flags |= RESET_FLAG_POWER_ON;
#endif
	}

	/*
	 * Set scratch bit to distinguish VCC1RST# is asserted again
	 * or not. This bit will be clear automatically when VCC1RST#
	 * is asserted or power-on reset occurs
	 */
	SET_BIT(NPCX_RSTCTL, NPCX_RSTCTL_VCC1_RST_SCRATCH);

	/* Software debugger reset */
	if (IS_BIT_SET(NPCX_RSTCTL, NPCX_RSTCTL_DBGRST_STS)) {
		flags |= RESET_FLAG_SOFT;
		/* Clear debugger reset status initially*/
		SET_BIT(NPCX_RSTCTL, NPCX_RSTCTL_DBGRST_STS);
	}

	/* Reset by hibernate */
	if (hib_wake_flags & HIBERNATE_WAKE_PIN)
		flags |= RESET_FLAG_WAKE_PIN | RESET_FLAG_HIBERNATE;
	else if (hib_wake_flags & HIBERNATE_WAKE_MTC)
		flags |= RESET_FLAG_RTC_ALARM | RESET_FLAG_HIBERNATE;

	/* Watchdog Reset */
	if (IS_BIT_SET(NPCX_T0CSR, NPCX_T0CSR_WDRST_STS)) {
		/*
		 * Don't set RESET_FLAG_WATCHDOG flag if watchdog is issued by
		 * system_reset or hibernate in order to distinguish reset cause
		 * is panic reason or not.
		 */
		if (!(flags & (RESET_FLAG_SOFT | RESET_FLAG_HARD |
				RESET_FLAG_HIBERNATE)))
			flags |= RESET_FLAG_WATCHDOG;

		/* Clear watchdog reset status initially*/
		SET_BIT(NPCX_T0CSR, NPCX_T0CSR_WDRST_STS);
	}

	system_set_reset_flags(flags);
}

/**
 * Chip-level function to set GPIOs and wake-up inputs for hibernate.
 */
#ifdef CONFIG_SUPPORT_CHIP_HIBERNATION
static void system_set_gpios_and_wakeup_inputs_hibernate(void)
{
	int table, i;

	/* Disable all MIWU inputs before entering hibernate */
	for (table = MIWU_TABLE_0 ; table < MIWU_TABLE_2 ; table++) {
		for (i = 0 ; i < 8 ; i++) {
			/* Disable all wake-ups */
			NPCX_WKEN(table, i)  = 0x00;
			/* Clear all pending bits of wake-ups */
			NPCX_WKPCL(table, i) = 0xFF;
			/*
			 * Disable all inputs of wake-ups to prevent leakage
			 * caused by input floating.
			 */
			NPCX_WKINEN(table, i) = 0x00;
		}
	}

#if defined(CHIP_FAMILY_NPCX7)
	/* Disable MIWU 2 group 6 inputs which used for the additional GPIOs */
	NPCX_WKEN(MIWU_TABLE_2, MIWU_GROUP_6)  = 0x00;
	NPCX_WKPCL(MIWU_TABLE_2, MIWU_GROUP_6) = 0xFF;
	NPCX_WKINEN(MIWU_TABLE_2, MIWU_GROUP_6) = 0x00;
#endif

	/* Enable wake-up inputs of hibernate_wake_pins array */
	for (i = 0; i < hibernate_wake_pins_used; i++) {
		gpio_reset(hibernate_wake_pins[i]);
		/* Re-enable interrupt for wake-up inputs */
		gpio_enable_interrupt(hibernate_wake_pins[i]);
#if defined(CONFIG_HIBERNATE_PSL)
		/* Config PSL pins setting for wake-up inputs */
		if (!system_config_psl_mode(hibernate_wake_pins[i]))
			ccprintf("Invalid PSL setting in wake-up pin %d\n", i);
#endif
	}
}

/**
 * hibernate function for npcx ec.
 *
 * @param seconds      Number of seconds to sleep before LCT alarm
 * @param microseconds Number of microseconds to sleep before LCT alarm
 */
void __enter_hibernate(uint32_t seconds, uint32_t microseconds)
{
	int i;

	/* Disable ADC */
	NPCX_ADCCNF = 0;
	usleep(1000);

	/* Set SPI pins to be in Tri-State */
	SET_BIT(NPCX_DEVCNT, NPCX_DEVCNT_F_SPI_TRIS);

	/* Disable instant wake up mode for better power consumption */
	CLEAR_BIT(NPCX_ENIDL_CTL, NPCX_ENIDL_CTL_LP_WK_CTL);

	/* Disable interrupt */
	interrupt_disable();

	/* ITIM event module disable */
	CLEAR_BIT(NPCX_ITCTS(ITIM_EVENT_NO), NPCX_ITCTS_ITEN);
	/* ITIM time module disable */
	CLEAR_BIT(NPCX_ITCTS(ITIM32), NPCX_ITCTS_ITEN);
	/* ITIM watchdog warn module disable */
	CLEAR_BIT(NPCX_ITCTS(ITIM_WDG_NO), NPCX_ITCTS_ITEN);

	/* Unlock & stop watchdog */
	NPCX_WDSDM = 0x87;
	NPCX_WDSDM = 0x61;
	NPCX_WDSDM = 0x63;

	/* Initialize watchdog */
	NPCX_TWCFG = 0; /* Select T0IN clock as watchdog prescaler clock */
	SET_BIT(NPCX_TWCFG, NPCX_TWCFG_WDCT0I);
	NPCX_TWCP = 0x00; /* Keep prescaler ratio timer0 clock to 1:1 */
	NPCX_TWDT0 = 0x00; /* Set internal counter and prescaler */

	/* Disable interrupt */
	interrupt_disable();

	/*
	 * Set gpios and wake-up input for better power consumption before
	 * entering hibernate.
	 */
	system_set_gpios_and_wakeup_inputs_hibernate();

	/*
	 * Give the board a chance to do any late stage hibernation work.
	 * This is likely going to configure GPIOs for hibernation.
	 */
	if (board_hibernate_late)
		board_hibernate_late();

	/* Clear all pending IRQ otherwise wfi will have no affect */
	for (i = NPCX_IRQ_0 ; i < NPCX_IRQ_COUNT ; i++)
		task_clear_pending_irq(i);

	/*
	 * Set RTC interrupt in time to wake up before
	 * next event.
	 */
	if (seconds || microseconds)
		system_set_rtc_alarm(seconds, microseconds);


	/* execute hibernate func depend on chip series */
	__hibernate_npcx_series();

}
#endif /* CONFIG_SUPPORT_CHIP_HIBERNATION */

static char system_to_hex(uint8_t x)
{
	if (x <= 9)
		return '0' + x;
	return 'a' + x - 10;
}

/*****************************************************************************/
/* IC specific low-level driver */

/*
 * Microseconds will be ignored.  The WTC register only
 * stores wakeup time in seconds.
 * Set seconds = 0 to disable the alarm
 */
void system_set_rtc_alarm(uint32_t seconds, uint32_t microseconds)
{
	uint32_t cur_secs, alarm_secs;

	if (seconds == EC_RTC_ALARM_CLEAR && !microseconds) {
		CLEAR_BIT(NPCX_WTC, NPCX_WTC_WIE);
		SET_BIT(NPCX_WTC, NPCX_WTC_PTO);

		return;
	}

	/* Get current clock */
	cur_secs = NPCX_TTC;

	/* If alarm clock is not sequential or not in range */
	alarm_secs = cur_secs + seconds;
	alarm_secs = alarm_secs & MTC_ALARM_MASK;

	/*
	 * We should set new alarm (first 25 bits of clock value) first before
	 * clearing PTO in case issue rtc interrupt immediately.
	 */
	NPCX_WTC = alarm_secs;

	/* Reset alarm first */
	system_reset_rtc_alarm();

	/* Enable interrupt mode alarm */
	SET_BIT(NPCX_WTC, NPCX_WTC_WIE);

	/* Enable MTC interrupt */
	task_enable_irq(NPCX_IRQ_MTC_WKINTAD_0);

	/* Enable wake-up input sources & clear pending bit */
	NPCX_WKPCL(MIWU_TABLE_0, MTC_WUI_GROUP)  |= MTC_WUI_MASK;
	NPCX_WKINEN(MIWU_TABLE_0, MTC_WUI_GROUP) |= MTC_WUI_MASK;
	NPCX_WKEN(MIWU_TABLE_0, MTC_WUI_GROUP)   |= MTC_WUI_MASK;
}

void system_reset_rtc_alarm(void)
{
	/*
	 * Clear interrupt & Disable alarm interrupt
	 * Update alarm value to zero
	 */
	CLEAR_BIT(NPCX_WTC, NPCX_WTC_WIE);
	SET_BIT(NPCX_WTC, NPCX_WTC_PTO);

	/* Disable MTC interrupt */
	task_disable_irq(NPCX_IRQ_MTC_WKINTAD_0);
}

/*
 * Return the seconds remaining before the RTC alarm goes off.
 * Returns 0 if alarm is not set.
 */
uint32_t system_get_rtc_alarm(void)
{
	/*
	 * Return 0:
	 * 1. If alarm is not set to go off, OR
	 * 2. If alarm is set and has already gone off
	 */
	if (!IS_BIT_SET(NPCX_WTC, NPCX_WTC_WIE) ||
	    IS_BIT_SET(NPCX_WTC, NPCX_WTC_PTO)) {
		return 0;
	}
	/* Get seconds before alarm goes off */
	return (NPCX_WTC - NPCX_TTC) & MTC_ALARM_MASK;
}

/**
 * Enable hibernate interrupt
 */
void system_enable_hib_interrupt(void)
{
	task_enable_irq(NPCX_IRQ_MTC_WKINTAD_0);
}

void system_hibernate(uint32_t seconds, uint32_t microseconds)
{
	/* Flush console before hibernating */
	cflush();

	if (board_hibernate)
		board_hibernate();

#ifdef CONFIG_SUPPORT_CHIP_HIBERNATION
	/* Add additional hibernate operations here */
	__enter_hibernate(seconds, microseconds);
#endif
}

void chip_pre_init(void)
{
	/* Setting for fixing JTAG issue */
	NPCX_DBGCTRL = 0x04;
	/* Enable automatic freeze mode */
	CLEAR_BIT(NPCX_DBGFRZEN3, NPCX_DBGFRZEN3_GLBL_FRZ_DIS);

	/*
	 * Enable JTAG functionality by SW without pulling down strap-pin
	 * nJEN0 or nJEN1 during ec POWERON or VCCRST reset occurs.
	 * Please notice it will change pinmux to JTAG directly.
	 */
#ifdef NPCX_ENABLE_JTAG
#if NPCX_JTAG_MODULE2
	CLEAR_BIT(NPCX_DEVALT(ALT_GROUP_5), NPCX_DEVALT5_NJEN1_EN);
#else
	CLEAR_BIT(NPCX_DEVALT(ALT_GROUP_5), NPCX_DEVALT5_NJEN0_EN);
#endif
#endif
}

void system_pre_init(void)
{
	uint8_t pwdwn6;

	/*
	 * Add additional initialization here
	 * EC should be initialized in Booter
	 */

	/* Power-down the modules we don't need */
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_1) = 0xF9; /* Skip SDP_PD FIU_PD */
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_2) = 0xFF;
#if defined(CHIP_FAMILY_NPCX5)
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_3) = 0x0F; /* Skip GDMA */
#elif defined(CHIP_FAMILY_NPCX7)
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_3) = 0x1F; /* Skip GDMA */
#endif
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_4) = 0xF4; /* Skip ITIM2/1_PD */
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_5) = 0xF8;

	pwdwn6 = 0x70 |
		(1 << NPCX_PWDWN_CTL6_ITIM6_PD) |
		(1 << NPCX_PWDWN_CTL6_ITIM4_PD); /* Skip ITIM5_PD */
#if !defined(CONFIG_ESPI)
	pwdwn6 |= 1 << NPCX_PWDWN_CTL6_ESPI_PD;
#endif
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_6) = pwdwn6;

#if defined(CHIP_FAMILY_NPCX7)
	NPCX_PWDWN_CTL(NPCX_PMC_PWDWN_7) = 0x07;
#endif

	/* Following modules can be powered down automatically in npcx7 */
#if defined(CHIP_FAMILY_NPCX5)
	/* Power down the modules of npcx5 used internally */
	NPCX_INTERNAL_CTRL1 = 0x03;
	NPCX_INTERNAL_CTRL2 = 0x03;
	NPCX_INTERNAL_CTRL3 = 0x03;

	/* Enable low-power regulator */
	CLEAR_BIT(NPCX_LFCGCALCNT, NPCX_LFCGCALCNT_LPREG_CTL_EN);
	SET_BIT(NPCX_LFCGCALCNT, NPCX_LFCGCALCNT_LPREG_CTL_EN);
#endif

	/*
	 * Configure LPRAM in the MPU as a regular memory
	 * and DATA RAM to prevent code execution
	 */
	system_mpu_config();

#ifdef CONFIG_CHIP_PANIC_BACKUP
	chip_panic_data_restore();
#endif
}

void system_reset(int flags)
{
	uint32_t save_flags;

	/* Disable interrupts to avoid task swaps during reboot */
	interrupt_disable();

	/*  Get flags to be saved in BBRAM */
	system_encode_save_flags(flags, &save_flags);

	/* Store flags to battery backed RAM. */
	chip_save_reset_flags(save_flags);

	/* If WAIT_EXT is set, then allow 10 seconds for external reset */
	if (flags & SYSTEM_RESET_WAIT_EXT) {
		int i;

		/* Wait 10 seconds for external reset */
		for (i = 0; i < 1000; i++) {
			watchdog_reload();
			udelay(10000);
		}
	}

	/* Ask the watchdog to trigger a hard reboot */
	system_watchdog_reset();

	/* Spin and wait for reboot; should never return */
	while (1)
		;
}

/**
 * Return the chip vendor/name/revision string.
 */
const char *system_get_chip_vendor(void)
{
	static char str[15] = "Unknown-";
	char *p = str + 8;

	/* Read Vendor ID in core register */
	uint8_t fam_id = NPCX_SID_CR;
	switch (fam_id) {
	case 0x20:
		return "Nuvoton";
	default:
		*p       = system_to_hex((fam_id & 0xF0) >> 4);
		*(p + 1) = system_to_hex(fam_id & 0x0F);
		*(p + 2) = '\0';
		return str;
	}
}

const char *system_get_chip_name(void)
{
	static char str[15] = "Unknown-";
	char *p = str + 8;

	/* Read Chip ID in core register */
	uint8_t chip_id = NPCX_DEVICE_ID_CR;
	switch (chip_id) {
#if defined(CHIP_FAMILY_NPCX5)
	case 0x12:
		return "NPCX585G";
	case 0x13:
		return "NPCX575G";
	case 0x16:
		return "NPCX586G";
	case 0x17:
		return "NPCX576G";
#elif defined(CHIP_FAMILY_NPCX7)
	case 0x21:
		return "NPCX796F";
#endif
	default:
		*p       = system_to_hex((chip_id & 0xF0) >> 4);
		*(p + 1) = system_to_hex(chip_id & 0x0F);
		*(p + 2) = '\0';
		return str;
	}
}

const char *system_get_chip_revision(void)
{
	static char rev[5];
	/* Read ROM data for chip revision directly */
	uint8_t rev_num = *((uint8_t *)CHIP_REV_ADDR);

	*(rev) = 'A';
	*(rev + 1) = '.';
	*(rev + 2) = system_to_hex((rev_num & 0xF0) >> 4);
	*(rev + 3) = system_to_hex(rev_num & 0x0F);
	*(rev + 4) = '\0';

	return rev;
}

BUILD_ASSERT(BBRM_DATA_INDEX_VBNVCNTXT + EC_VBNV_BLOCK_SIZE <= NPCX_BBRAM_SIZE);

/**
 * Set a scratchpad register to the specified value.
 *
 * The scratchpad register must maintain its contents across a
 * software-requested warm reset.
 *
 * @param value		Value to store.
 * @return EC_SUCCESS, or non-zero if error.
 */
int system_set_scratchpad(uint32_t value)
{
	return bbram_data_write(BBRM_DATA_INDEX_SCRATCHPAD, value);
}

uint32_t system_get_scratchpad(void)
{
	return bbram_data_read(BBRM_DATA_INDEX_SCRATCHPAD);
}

int system_is_reboot_warm(void)
{
	uint32_t reset_flags;

	/*
	 * Check reset cause here,
	 * gpio_pre_init is executed faster than system_pre_init
	 */
	system_check_reset_cause();
	reset_flags = system_get_reset_flags();

	if ((reset_flags & RESET_FLAG_RESET_PIN) ||
	    (reset_flags & RESET_FLAG_POWER_ON) ||
	    (reset_flags & RESET_FLAG_WATCHDOG) ||
	    (reset_flags & RESET_FLAG_HARD) ||
	    (reset_flags & RESET_FLAG_SOFT) ||
	    (reset_flags & RESET_FLAG_HIBERNATE))
		return 0;
	else
		return 1;
}

/*****************************************************************************/
/* Console commands */
#ifdef CONFIG_CMD_RTC
void print_system_rtc(enum console_channel ch)
{
	uint32_t sec = system_get_rtc_sec();

	cprintf(ch, "RTC: 0x%08x (%d.00 s)\n", sec, sec);
}

static int command_system_rtc(int argc, char **argv)
{
	if (argc == 3 && !strcasecmp(argv[1], "set")) {
		char *e;
		uint32_t t = strtoi(argv[2], &e, 0);
		if (*e)
			return EC_ERROR_PARAM2;

		system_set_rtc(t);
	} else if (argc > 1) {
		return EC_ERROR_INVAL;
	}

	print_system_rtc(CC_COMMAND);

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(rtc, command_system_rtc,
		"[set <seconds>]",
		"Get/set real-time clock");

#ifdef CONFIG_CMD_RTC_ALARM
/**
 * Test the RTC alarm by setting an interrupt on RTC match.
 */
static int command_rtc_alarm_test(int argc, char **argv)
{
	int s = 1, us = 0;
	char *e;

	ccprintf("Setting RTC alarm\n");
	system_enable_hib_interrupt();

	if (argc > 1) {
		s = strtoi(argv[1], &e, 10);
		if (*e)
			return EC_ERROR_PARAM1;

	}
	if (argc > 2) {
		us = strtoi(argv[2], &e, 10);
		if (*e)
			return EC_ERROR_PARAM2;

	}

	system_set_rtc_alarm(s, us);

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(rtc_alarm, command_rtc_alarm_test,
		"[seconds [microseconds]]",
		"Test alarm");
#endif /* CONFIG_CMD_RTC_ALARM */
#endif /* CONFIG_CMD_RTC */

/*****************************************************************************/
/* Host commands */

#ifdef CONFIG_HOSTCMD_RTC
static int system_rtc_get_value(struct host_cmd_handler_args *args)
{
	struct ec_response_rtc *r = args->response;

	r->time = system_get_rtc_sec();
	args->response_size = sizeof(*r);

	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_RTC_GET_VALUE,
		system_rtc_get_value,
		EC_VER_MASK(0));

static int system_rtc_set_value(struct host_cmd_handler_args *args)
{
	const struct ec_params_rtc *p = args->params;

	system_set_rtc(p->time);
	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_RTC_SET_VALUE,
		system_rtc_set_value,
		EC_VER_MASK(0));

static int system_rtc_set_alarm(struct host_cmd_handler_args *args)
{
	const struct ec_params_rtc *p = args->params;

	system_set_rtc_alarm(p->time, 0);
	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_RTC_SET_ALARM,
		system_rtc_set_alarm,
		EC_VER_MASK(0));

static int system_rtc_get_alarm(struct host_cmd_handler_args *args)
{
	struct ec_response_rtc *r = args->response;

	r->time = system_get_rtc_alarm();
	args->response_size = sizeof(*r);

	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_RTC_GET_ALARM,
		system_rtc_get_alarm,
		EC_VER_MASK(0));

#endif /* CONFIG_HOSTCMD_RTC */
#ifdef CONFIG_EXTERNAL_STORAGE
void system_jump_to_booter(void)
{
	enum API_RETURN_STATUS_T status __attribute__((unused));
	static uint32_t flash_offset;
	static uint32_t flash_used;
	static uint32_t addr_entry;

	/*
	 * Get memory offset and size for RO/RW regions.
	 * Both of them need 16-bytes alignment since GDMA burst mode.
	 */
	switch (system_get_shrspi_image_copy()) {
	case SYSTEM_IMAGE_RW:
		flash_offset = CONFIG_EC_WRITABLE_STORAGE_OFF +
				CONFIG_RW_STORAGE_OFF;
		flash_used = CONFIG_RW_SIZE;
		break;
#ifdef CONFIG_RW_B
	case SYSTEM_IMAGE_RW_B:
		flash_offset = CONFIG_EC_WRITABLE_STORAGE_OFF +
				CONFIG_RW_B_STORAGE_OFF;
		flash_used = CONFIG_RW_SIZE;
		break;
#endif
	case SYSTEM_IMAGE_RO:
	default: /* Jump to RO by default */
		flash_offset = CONFIG_EC_PROTECTED_STORAGE_OFF +
				CONFIG_RO_STORAGE_OFF;
		flash_used = CONFIG_RO_SIZE;
		break;
	}

	/* Make sure the reset vector is inside the destination image */
	addr_entry = *(uintptr_t *)(flash_offset +
				    CONFIG_MAPPED_STORAGE_BASE + 4);

	/*
	 * Speed up FW download time by increasing clock freq of EC. It will
	 * restore to default in clock_init() later.
	 */
	clock_turbo();

	/* Bypass for GMDA issue of ROM api utilities */
#if defined(CHIP_FAMILY_NPCX5)
	system_download_from_flash(
		flash_offset,      /* The offset of the data in spi flash */
		CONFIG_PROGRAM_MEMORY_BASE, /* RAM Addr of downloaded data */
		flash_used,        /* Number of bytes to download      */
		addr_entry         /* jump to this address after download */
	);
#else
	download_from_flash(
		flash_offset,      /* The offset of the data in spi flash */
		CONFIG_PROGRAM_MEMORY_BASE, /* RAM Addr of downloaded data */
		flash_used,        /* Number of bytes to download      */
		SIGN_NO_CHECK,     /* Need CRC check or not               */
		addr_entry,        /* jump to this address after download */
		&status            /* Status fo download */
	);
#endif
}

uint32_t system_get_lfw_address()
{
	/*
	 * In A3 version, we don't use little FW anymore
	 * We provide the alternative function in ROM
	 */
	uint32_t jump_addr = (uint32_t)system_jump_to_booter;
	return jump_addr;
}

/*
 * Set and clear image copy flags in MDC register.
 *
 * NPCX_FWCTRL_RO_REGION: 1 - RO, 0 - RW
 * NPCX_FWCTRL_FW_SLOT: 1 - SLOT_A, 0 - SLOT_B
 */
void system_set_image_copy(enum system_image_copy_t copy)
{
	switch (copy) {
	case SYSTEM_IMAGE_RW:
		CLEAR_BIT(NPCX_FWCTRL, NPCX_FWCTRL_RO_REGION);
		SET_BIT(NPCX_FWCTRL, NPCX_FWCTRL_FW_SLOT);
		break;
#ifdef CONFIG_RW_B
	case SYSTEM_IMAGE_RW_B:
		CLEAR_BIT(NPCX_FWCTRL, NPCX_FWCTRL_RO_REGION);
		CLEAR_BIT(NPCX_FWCTRL, NPCX_FWCTRL_FW_SLOT);
		break;
#endif
	default:
		CPRINTS("Invalid copy (%d) is requested as a jump destination. "
			"Change it to %d.", copy, SYSTEM_IMAGE_RO);
		/* Fall through to SYSTEM_IMAGE_RO */
	case SYSTEM_IMAGE_RO:
		SET_BIT(NPCX_FWCTRL, NPCX_FWCTRL_RO_REGION);
		SET_BIT(NPCX_FWCTRL, NPCX_FWCTRL_FW_SLOT);
		break;
	}
}

enum system_image_copy_t system_get_shrspi_image_copy(void)
{
	if (IS_BIT_SET(NPCX_FWCTRL, NPCX_FWCTRL_RO_REGION)) {
		/* RO image */
#ifdef CHIP_HAS_RO_B
		if (!IS_BIT_SET(NPCX_FWCTRL, NPCX_FWCTRL_FW_SLOT))
			return SYSTEM_IMAGE_RO_B;
#endif
		return SYSTEM_IMAGE_RO;
	} else {
#ifdef CONFIG_RW_B
		/* RW image */
		if (!IS_BIT_SET(NPCX_FWCTRL, NPCX_FWCTRL_FW_SLOT))
			/* Slot A */
			return SYSTEM_IMAGE_RW_B;
#endif
		return SYSTEM_IMAGE_RW;
	}
}

#endif

/* BEGIN mx_chipmem */

#ifndef mx_assert
#define mx_assert(...)
#define MX_ADDR_NATIVE(addr) (addr)
#include <stdint.h>
/* Not available, since cmdline for EC uses "-nostdinc -nostdlib" */
/* #include <inttypes.h> */
#include "common.h"
#include "console.h"
#include "timer.h"
#include "util.h"
#endif

/* #include <stdint.h>
 * 	uint*_t
 * 	UINT*_MAX
 * #include <inttypes.h>
 * 	PRIx*
 * #include "common.h"
 * 	ec_error_list - EC_* codes
 * #include "console.h"
 * 	DECLARE_CONSOLE_COMMAND(...)
 * 	cprintf(...)
 * #include "timer.h"
 * 	udelay(...)
 * #include "util.h"
 * 	strcasecmp(...)
 * 	MIN(...)
 * 	isspace(...)
 */

#ifndef MX_ADDR_NATIVE
#define MX_ADDR_NATIVE(addr) (addr)
#endif

/* This is used if "-nostdinc -nostdlib" disables some stuff. */
/* BEGIN CHIP SPECIFIC */
/* Based on target Nuvoton NPCX586G ARMv7 Cortex-M4 (firmware-fizz-10139.B) and
 * EC vfnprintf() implementation.
 */
#ifndef PRIu8
#define MX_PRIu8         "u"
#define MX_PRIx8         "x"
#define MX_PRIx16        "x"
#define MX_PRIu32        "u"
#define MX_PRIx32        "x"
#define MX_PRIx64        "lx"
#define MX_PRIuPTR       "u"
#define MX_PRIxPTR       "x"
#define MX_UINT32_MAX    __UINT32_MAX__
#define MX_UINT64_MAX    __UINT64_MAX__
#define MX_UINTPTR_MAX   __UINTPTR_MAX__
#else /* END CHIP SPECIFIC */
#define MX_PRIu8         PRIu8
#define MX_PRIx8         PRIx8
#define MX_PRIx16        PRIx16
#define MX_PRIu32        PRIu32
#define MX_PRIx32        PRIx32
#define MX_PRIx64        PRIx64
#define MX_PRIuPTR       PRIuPTR
#define MX_PRIxPTR       PRIxPTR
#define MX_UINT32_MAX    UINT32_MAX
#define MX_UINT64_MAX    UINT64_MAX
#define MX_UINTPTR_MAX   UINTPTR_MAX
#endif

#define MX_DECLARE_CONSOLE_COMMAND(...) DECLARE_CONSOLE_COMMAND(__VA_ARGS__)
#define mx_cprintf(...)                 cprintf(CC_COMMAND, __VA_ARGS__)
#define mx_udelay(...)                  udelay(__VA_ARGS__)
#define mx_strcasecmp(...)              strcasecmp(__VA_ARGS__)
#define mx_MIN(...)                     MIN(__VA_ARGS__)
#define mx_isspace(...)                 isspace(__VA_ARGS__)

#define MX_EC_SUCCESS     EC_SUCCESS
#define MX_EC_ERROR_INVAL EC_ERROR_INVAL

#define MX_RETCPRINTF(ret_code, ...) \
	{ \
		mx_cprintf(__VA_ARGS__); \
		return ret_code; \
	}

#define MX_ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/* Memory address */
typedef uintptr_t mx_maddr_t;
#define MX_MADDR_MAX MX_UINTPTR_MAX
#define MX_PRIuMADDR MX_PRIuPTR
#define MX_PRIxMADDR MX_PRIxPTR

/* Largest supported value */
#if MX_MADDR_MAX > MX_UINT32_MAX
typedef mx_maddr_t mx_lsval_t;
#define MX_LSVAL_MAX MX_MADDR_MAX
#define MX_PRIuLSVAL MX_PRIuMADDR
#define MX_PRIxLSVAL MX_PRIxMADDR
#else
typedef uint32_t mx_lsval_t;
#define MX_LSVAL_MAX MX_UINT32_MAX
#define MX_PRIuLSVAL MX_PRIu32
#define MX_PRIxLSVAL MX_PRIx32
#endif

#if MX_LSVAL_MAX >= MX_UINT64_MAX
#define MX_HAS_U64 1
#if MX_MADDR_MAX >= MX_UINT64_MAX
#define MX_HAS_U64_WPTR 1
#endif /* MX_MADDR_MAX */
#endif /* MX_LSVAL_MAX */

typedef enum {
	MX_STUX_U8 = 0,
	MX_STUX_U16,
	MX_STUX_U32,
#ifdef MX_HAS_U64
	MX_STUX_U64,
#endif
	MX_STUX_MADDR,
	MX_STUX_LSVAL
} mx_strtouX_type_t;
static int mx_strtouX_ex(const char *nptr, char **endptr,
		mx_strtouX_type_t result_type, void *out_result, int base)
{
	mx_lsval_t result = 0;
	int c = '\0';

	if (endptr)
		*endptr = (char *)nptr;

	while ((c = *nptr++) && mx_isspace(c));

	if (c == '0' && *nptr == 'x') {
		base = 16;
		c = nptr[1];
		nptr += 2;
	} else if (base == 0) {
		base = 10;
	}

	while (c) {
		if (c >= '0' && c < '0' + mx_MIN(base, 10))
			result = result * base + (c - '0');
		else if (c >= 'A' && c < 'A' + base - 10)
			result = result * base + (c - 'A' + 10);
		else if (c >= 'a' && c < 'a' + base - 10)
			result = result * base + (c - 'a' + 10);
		else
			break;

		if (endptr)
			*endptr = (char *)nptr;
		c = *nptr++;
	}

#define _MX_STRTOUX_RESULT(type, max) \
		if (result > max) return 0; \
		*((type *)out_result) = (type)result; \
		break

	switch (result_type) {
	case MX_STUX_U8:    _MX_STRTOUX_RESULT(uint8_t, UINT8_MAX);
	case MX_STUX_U16:   _MX_STRTOUX_RESULT(uint16_t, UINT16_MAX);
	case MX_STUX_U32:   _MX_STRTOUX_RESULT(uint32_t, UINT32_MAX);
#ifdef MX_HAS_U64
	case MX_STUX_U64:   _MX_STRTOUX_RESULT(uint64_t, UINT64_MAX);
#endif
	case MX_STUX_MADDR: _MX_STRTOUX_RESULT(mx_maddr_t, MX_MADDR_MAX);
	case MX_STUX_LSVAL: _MX_STRTOUX_RESULT(mx_lsval_t, MX_LSVAL_MAX);
	default:            return 0;
	}

#undef _MX_STRTOUX_RESULT

	return 1;
}

static inline int mx_strtouX(const char *nptr, mx_strtouX_type_t result_type,
		void *out_result, int base)
{
	char *s;
	return (!mx_strtouX_ex(nptr, &s, result_type, out_result, base) || *s ?
			0 : 1);
}

static char *mx_strstr(const char *haystack, const char *needle)
{
	const char *h;
	const char *n;
	if (*needle == '\0')
		return (char *)haystack;
	for (; *haystack != '\0'; ++haystack) {
		h = haystack;
		n = needle;
		while (*n != '\0' && *h == *n) {
			++h;
			++n;
		}
		if (*n == '\0')
			return (char *)haystack;
	}
	return NULL;
}

typedef struct mx_conf_t mx_conf_t;

typedef struct {
	const char *name_short;
	const char *name_long;
} mx_name_sl_t;

typedef struct {
	uint8_t index;
} mx_mem_bit_t;

typedef struct {
	mx_maddr_t offset;
} mx_mem_data_t;

typedef struct {
	mx_maddr_t addr;
	mx_maddr_t len;
} mx_mem_region_t;

typedef struct {
	const char *name_bytes;
	const char *name_type;
	const char *name_singular;
	const char *name_plural;
	const int bytes;
	const uint8_t index_max;
	const mx_lsval_t value_max;
} mx_accs_t;
typedef enum {
	MX_ACCS_1 = 0,
	MX_ACCS_2,
	MX_ACCS_4,
#ifdef MX_HAS_U64
	MX_ACCS_8,
#endif
	MX_ACCS_MADDR,
	MX_ACCS_LSVAL
} mx_access_size_idx_t;

typedef enum {
	MX_AEND_BBRAM = 0,
	MX_AEND_NATIVE
} mx_access_endianness_idx_t;
typedef mx_name_sl_t mx_aend_t;

typedef enum {
	MX_DMPM_BITS = 0,
	MX_DMPM_HEX
} mx_dump_mode_idx_t;
typedef mx_name_sl_t mx_dmpm_t;

typedef enum {
	MX_OPT_ACCESS_SIZE = 0,
	MX_OPT_ACCESS_ENDIANNESS,
	MX_OPT_DUMP_MODE,
	MX_OPT_DUMP_COLS
} mx_opts_idx_t;
typedef mx_name_sl_t mx_opts_t;

typedef enum {
	MX_TMPC_UNSPEC = 0,
	MX_TMPC_VALUE
} mx_task_mem_parse_common_t;
typedef struct {
	mx_task_mem_parse_common_t parse;
	mx_mem_region_t mem_region;
} mx_task_mem_region_t;
typedef struct {
	mx_task_mem_parse_common_t parse;
	mx_mem_data_t mem_data;
	mx_maddr_t c_addr;
} mx_task_mem_data_t;
typedef struct {
	mx_task_mem_parse_common_t parse;
	mx_mem_bit_t mem_bit;
} mx_task_mem_bit_t;

typedef enum {
	MX_TMPV_UNSPEC = 0,
	MX_TMPV_DATA,
	MX_TMPV_BIT
} mx_task_mem_parse_value_t;
typedef struct {
	mx_task_mem_parse_value_t parse;
	mx_lsval_t value;
} mx_task_mem_value_t;

typedef struct {
	mx_task_mem_region_t tmem_region;
} mx_task_DUMP_data_t;

typedef struct {
	mx_task_mem_region_t tmem_region;
	mx_task_mem_data_t tmem_data;
} mx_task_GETVALUE_data_t;

typedef struct {
	mx_task_mem_region_t tmem_region;
	mx_task_mem_data_t tmem_data;
	mx_task_mem_value_t tmem_value;
} mx_task_SETVALUE_data_t;

typedef struct {
	mx_task_mem_region_t tmem_region;
	mx_task_mem_data_t tmem_data;
	mx_task_mem_bit_t tmem_bit;
} mx_task_GETBIT_data_t;

typedef struct {
	mx_task_mem_region_t tmem_region;
	mx_task_mem_data_t tmem_data;
	mx_task_mem_bit_t tmem_bit;
	mx_task_mem_value_t tmem_value;
} mx_task_SETBIT_data_t;

typedef struct {
	mx_task_mem_region_t tmem_region;
	mx_task_mem_data_t tmem_data;
} mx_task_LIST_data_t;

typedef enum mx_task_idx_t {
	MX_TASK_NONE = -1,
	MX_TASK_DUMP,
	MX_TASK_GETVALUE,
	MX_TASK_SETVALUE,
	MX_TASK_GETBIT,
	MX_TASK_SETBIT
} mx_task_idx_t;
typedef int (* mx_init_conf_task_fn_t)(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos);
typedef int (* mx_run_task_fn_t)(const mx_conf_t *conf);
typedef struct mx_task_t {
	mx_name_sl_t name_sl;
	mx_init_conf_task_fn_t init_conf_fn;
	mx_run_task_fn_t run_task_fn;
} mx_task_t;

typedef enum {
	MX_PMF_OPT_REGION            = (1 << 0),
	MX_PMF_REGION_VALUE_HAS_LEN  = (1 << 1),
	MX_PMF_REGION_ACCS_CHECK     = (1 << 2),
	MX_PMF_OPT_DATA              = (1 << 3),
	MX_PMF_OPT_BIT               = (1 << 4),
	MX_PMF_OPT_VALUE             = (1 << 5)
} mx_parse_task_mem_flags_t;
typedef struct {
	mx_task_idx_t task_idx;
	mx_task_mem_region_t *tmem_region;
	mx_task_mem_data_t *tmem_data;
	mx_task_mem_bit_t *tmem_bit;
	mx_task_mem_value_t *tmem_value;
	int flags;
} mx_parse_task_mem_t;

typedef void (*mx_dump_fn_t)(mx_access_size_idx_t access_size_idx,
		mx_lsval_t value, int prefix);
typedef int (*mx_read_mem_fn_t)(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t *out_value);
typedef int (*mx_write_mem_fn_t)(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t value);
typedef struct mx_conf_t {
	mx_access_size_idx_t access_size_idx;
	mx_access_endianness_idx_t access_endianness_idx;
	mx_dump_mode_idx_t dump_mode_idx;
	uint8_t dump_cols;
	mx_task_idx_t task_idx;
	union {
		mx_task_DUMP_data_t DUMP;
		mx_task_GETVALUE_data_t GETVALUE;
		mx_task_SETVALUE_data_t SETVALUE;
		mx_task_GETBIT_data_t GETBIT;
		mx_task_SETBIT_data_t SETBIT;
	} task_data;
	mx_dump_fn_t dump_fn;
	mx_read_mem_fn_t read_chip_mem_fn;
	mx_write_mem_fn_t write_chip_mem_fn;
} mx_conf_t;

typedef struct {
	const char *s1;
	const char *s2;
	const char *s3;
	const char *s4;
} _mx_find_name4_t;
static int mx_find_idx_by_name(const void *arr, int arr_len, const char *name,
		int arr_size, int name_count)
{
	const uint8_t *elem;
	const _mx_find_name4_t *name4;
	const char *s;
	int i, j;
	for (i = 0; i < arr_len; ++i) {
		elem = (const uint8_t *)arr + i * arr_size;
		name4 = (const _mx_find_name4_t*)elem;
		for (j = 1; j <= name_count; ++j) {
			switch (j) {
			case 1:  s = name4->s1; break;
			case 2:  s = name4->s2; break;
			case 3:  s = name4->s3; break;
			case 4:  s = name4->s4; break;
			default: s = NULL;      break;
			}
			if (s && mx_strcasecmp(name, s) == 0)
				return i;
		}
	}
	return -1;
}

/* Implemented by chip specific code. */
static int mx_is_chip_mem_accs_supported(mx_access_size_idx_t access_size_idx,
		mx_access_endianness_idx_t access_endianness_idx);
static int mx_read_chip_mem_native(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t *out_value);
static int mx_write_chip_mem_native(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t value);
static int mx_read_chip_mem_bbram(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t *out_value);
static int mx_write_chip_mem_bbram(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t value);

/* const S_TYPE *mx_get_NAME_s(IDX_TYPE idx);
 */
#define MX_DEFINE_STATIC_ARR_GETRS(d_name, d_arr, d_s_type, d_idx_type) \
	static inline const d_s_type *mx_get_##d_name##_s(d_idx_type idx) { \
		return &d_arr[idx]; \
	}

/* const S_TYPE *mx_get_NAME_s_by_cnf(const mx_conf_t *conf);
 */
#define MX_DEFINE_STATIC_ARR_GETRS_WCNF(d_name, d_arr, d_s_type, d_idx_type, \
		d_conf_idx_field) \
	MX_DEFINE_STATIC_ARR_GETRS(d_name, d_arr, d_s_type, d_idx_type); \
	static inline const d_s_type *mx_get_##d_name##_s_by_cnf( \
			const mx_conf_t *conf) { \
		return &d_arr[conf->d_conf_idx_field]; \
	}

/* int mx_find_NAME_idx_by_name(const char *name)
 */
#define MX_DEFINE_STATIC_ARR_FIND_NAME(d_name, d_arr, d_s_type, d_names_len) \
	static inline int mx_find_##d_name##_idx_by_name(const char *name) { \
		return mx_find_idx_by_name(&d_arr, MX_ARRAY_SIZE(d_arr), \
				name, sizeof(d_s_type), d_names_len); \
	}

#define _MX_TYPE_MAX_INDEX(type) (sizeof(type) * 8 - 1)
static const mx_accs_t _mx_accs[] = {
	[MX_ACCS_1]     = {"1", "u8",  "byte",  "bytes",  sizeof(uint8_t),
			_MX_TYPE_MAX_INDEX(uint8_t), UINT8_MAX},
	[MX_ACCS_2]     = {"2", "u16", "word",  "words",  sizeof(uint16_t),
			_MX_TYPE_MAX_INDEX(uint16_t), UINT16_MAX},
	[MX_ACCS_4]     = {"4", "u32", "dword", "dwords", sizeof(uint32_t),
			_MX_TYPE_MAX_INDEX(uint32_t), UINT32_MAX},
#ifdef MX_HAS_U64
	[MX_ACCS_8]     = {"8", "u64", "qword", "qwords", sizeof(uint64_t),
			_MX_TYPE_MAX_INDEX(uint64_t), UINT64_MAX},
#endif
	[MX_ACCS_MADDR] = {"ma", "ma", "maddr", "maddrs", sizeof(mx_maddr_t),
			_MX_TYPE_MAX_INDEX(mx_maddr_t), MX_MADDR_MAX},
	[MX_ACCS_LSVAL] = {"lsv", "lsv", "lsval", "lsvals", sizeof(mx_lsval_t),
			_MX_TYPE_MAX_INDEX(mx_lsval_t), MX_LSVAL_MAX}
};
#undef _MX_TYPE_MAX_INDEX
MX_DEFINE_STATIC_ARR_GETRS_WCNF(accs, _mx_accs, mx_accs_t,
		mx_access_size_idx_t, access_size_idx);
MX_DEFINE_STATIC_ARR_FIND_NAME(accs, _mx_accs, mx_accs_t, 4);

static const mx_aend_t _mx_aend[] = {
	[MX_AEND_BBRAM]  = {"b", "bbram"},
	[MX_AEND_NATIVE] = {"n", "native"}
};
MX_DEFINE_STATIC_ARR_GETRS_WCNF(aend, _mx_aend, mx_aend_t,
		mx_access_endianness_idx_t, access_endianness_idx);
MX_DEFINE_STATIC_ARR_FIND_NAME(aend, _mx_aend, mx_aend_t, 2);

static const mx_dmpm_t _mx_dmpm[] = {
	[MX_DMPM_BITS] = {"b", "bits"},
	[MX_DMPM_HEX]  = {"h", "hex"}
};
MX_DEFINE_STATIC_ARR_GETRS_WCNF(dmpm, _mx_dmpm, mx_dmpm_t,
		mx_dump_mode_idx_t, dump_mode_idx);
MX_DEFINE_STATIC_ARR_FIND_NAME(dmpm, _mx_dmpm, mx_dmpm_t, 2);

static const mx_opts_t _mx_opts[] = {
	[MX_OPT_ACCESS_SIZE]       = {"-a", "--access-size"},
	[MX_OPT_ACCESS_ENDIANNESS] = {"-e", "--access-endianness"},
	[MX_OPT_DUMP_MODE]         = {"-m", "--dump-mode"},
	[MX_OPT_DUMP_COLS]         = {"-c", "--dump-cols"}
};
MX_DEFINE_STATIC_ARR_GETRS(opts, _mx_opts, mx_opts_t, mx_opts_idx_t);
MX_DEFINE_STATIC_ARR_FIND_NAME(opts, _mx_opts, mx_opts_t, 2);

/* dec: int _mx_init_conf_task_TASKNAME(mx_conf_t *conf, int argc, char **argv,
 * 		int *inout_argv_pos)
 * dec: int _mx_run_task_TASKNAME(const mx_conf_t *conf)
 * const TASK_DATA_TYPE *_mx_get_conf_task_TASKNAME_data(const mx_conf_t *conf)
 * void _mx_set_conf_task_TASKNAME(mx_conf_t *conf, TASK_DATA_TYPE *task_data)
 */
#define MX_DEFINE_TASK_CONF_HELPERS(d_task_name, d_task_idx, d_task_data_type) \
	static int _mx_init_conf_task_##d_task_name( \
			mx_conf_t *conf, int argc, char **argv, \
			int *inout_argv_pos); \
	static int _mx_run_task_##d_task_name(const mx_conf_t *conf); \
	static inline const d_task_data_type \
			*_mx_get_conf_task_##d_task_name##_data( \
					const mx_conf_t *conf) \
	{ \
		return (conf->task_idx == d_task_idx ? \
				&conf->task_data.d_task_name : NULL); \
	} \
	static inline void _mx_set_conf_task_##d_task_name( \
			mx_conf_t *conf, d_task_data_type *task_data) \
	{ \
		conf->task_idx = d_task_idx; \
		conf->task_data.d_task_name = *task_data; \
	} \

MX_DEFINE_TASK_CONF_HELPERS(DUMP, MX_TASK_DUMP, mx_task_DUMP_data_t);
MX_DEFINE_TASK_CONF_HELPERS(GETVALUE, MX_TASK_GETVALUE,
		mx_task_GETVALUE_data_t);
MX_DEFINE_TASK_CONF_HELPERS(SETVALUE, MX_TASK_SETVALUE,
		mx_task_SETVALUE_data_t);
MX_DEFINE_TASK_CONF_HELPERS(GETBIT, MX_TASK_GETBIT, mx_task_GETBIT_data_t);
MX_DEFINE_TASK_CONF_HELPERS(SETBIT, MX_TASK_SETBIT, mx_task_SETBIT_data_t);

static const mx_task_t _mx_task[] = {
	[MX_TASK_DUMP]     = { {"d",  "dump"}, &_mx_init_conf_task_DUMP,
			&_mx_run_task_DUMP},
	[MX_TASK_GETVALUE] = { {"gv", "get-value"},
			&_mx_init_conf_task_GETVALUE, &_mx_run_task_GETVALUE},
	[MX_TASK_SETVALUE] = { {"sv", "set-value"},
			&_mx_init_conf_task_SETVALUE, &_mx_run_task_SETVALUE},
	[MX_TASK_GETBIT]   = { {"gb", "get-bit"}, &_mx_init_conf_task_GETBIT,
			&_mx_run_task_GETBIT},
	[MX_TASK_SETBIT]   = { {"sb", "set-bit"}, &_mx_init_conf_task_SETBIT,
			&_mx_run_task_SETBIT}
};
MX_DEFINE_STATIC_ARR_GETRS_WCNF(task, _mx_task, mx_task_t, mx_task_idx_t,
		task_idx);
MX_DEFINE_STATIC_ARR_FIND_NAME(task, _mx_task, mx_task_t, 2);

static void mx_dump_bits(mx_access_size_idx_t access_size_idx, mx_lsval_t value,
		int prefix)
{
	int access_size_bits = mx_get_accs_s(access_size_idx)->bytes * 8;
	int i;
	if (prefix)
		mx_cprintf("b");
	for (i = access_size_bits - 1; i >= 0; --i) {
		mx_cprintf("%" MX_PRIu8, (uint8_t)((value >> i) & 1));
		if (i > 0 && i % 8 == 0)
			mx_cprintf("|");
	}
}
static inline void mx_dump_bits_by_cnf(const mx_conf_t *conf, mx_lsval_t value,
		int prefix)
{
	mx_dump_bits(conf->access_size_idx, value, prefix);
}

static void mx_dump_hex(mx_access_size_idx_t access_size_idx, mx_lsval_t value,
		int prefix)
{
	if (prefix)
		mx_cprintf("0x");
	switch (mx_get_accs_s(access_size_idx)->bytes) {
	case 1:
		mx_cprintf("%02" MX_PRIx8, (uint8_t)value);
		break;
	case 2:
		mx_cprintf("%04" MX_PRIx16, (uint16_t)value);
		break;
	case 4:
		mx_cprintf("%08" MX_PRIx32, (uint32_t)value);
		break;
#ifdef MX_HAS_U64
	case 8:
		mx_cprintf("%016" MX_PRIx64, (uint64_t)value);
		break;
#endif
	default:
		break;
	}
}
static inline void mx_dump_hex_by_cnf(const mx_conf_t *conf, mx_lsval_t value,
		int prefix)
{
	mx_dump_hex(conf->access_size_idx, value, prefix);
}

static inline void mx_dump(const mx_conf_t *conf,
		mx_access_size_idx_t access_size_idx, mx_lsval_t value,
		int prefix)
{
	conf->dump_fn(access_size_idx, value, prefix);
}
static inline void mx_dump_by_cnf(const mx_conf_t *conf, mx_lsval_t value,
		int prefix)
{
	mx_dump(conf, conf->access_size_idx, value, prefix);
}

static void mx_read_volatile_memcpy(uint8_t *buf, const volatile uint8_t *addr,
		size_t len)
{
	size_t i;
	for (i = 0; i < len; ++i) {
		buf[i] = addr[i];
	}
}

static void mx_write_volatile_memcpy(volatile uint8_t *addr, const uint8_t *buf,
		size_t len)
{
	size_t i;
	for (i = 0; i < len; ++i) {
		addr[i] = buf[i];
	}
}

static int mx_read_mem_native(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t *out_value)
{
	int read_bytes = mx_get_accs_s(access_size_idx)->bytes;
	uint8_t buf[sizeof(mx_lsval_t)] = {0};
	mx_lsval_t value = 0;

#define _MX_READ_MEM_ADDR_TO_VALUE(type) \
		mx_read_volatile_memcpy(buf, \
				(volatile uint8_t *)MX_ADDR_NATIVE(addr), \
				sizeof(type)); \
		value = *((type *)buf); \
		break

	switch (read_bytes) {
	case 1: _MX_READ_MEM_ADDR_TO_VALUE(uint8_t);
	case 2: _MX_READ_MEM_ADDR_TO_VALUE(uint16_t);
	case 4: _MX_READ_MEM_ADDR_TO_VALUE(uint32_t);
#ifdef MX_HAS_U64_WPTR
	case 8: _MX_READ_MEM_ADDR_TO_VALUE(uint64_t);
#endif
	default: return 0;
	}

#undef _MX_READ_MEM_ADDR_TO_VALUE

	*out_value = value;

	return 1;
}

static int mx_write_mem_native(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t value)
{
	int write_bytes = mx_get_accs_s(access_size_idx)->bytes;
	uint8_t buf[sizeof(mx_lsval_t)] = {0};

#define _MX_WRITE_MEM_ADDR_FROM_VALUE(type) \
		*((type *)buf) = (type)value; \
		mx_write_volatile_memcpy( \
				(volatile uint8_t *)MX_ADDR_NATIVE(addr), \
				buf, sizeof(type)); \
		break

	switch (write_bytes) {
	case 1: _MX_WRITE_MEM_ADDR_FROM_VALUE(uint8_t);
	case 2: _MX_WRITE_MEM_ADDR_FROM_VALUE(uint16_t);
	case 4: _MX_WRITE_MEM_ADDR_FROM_VALUE(uint32_t);
#ifdef MX_HAS_U64_WPTR
	case 8: _MX_WRITE_MEM_ADDR_FROM_VALUE(uint64_t);
#endif
	default: return 0;
	}

#undef _MX_WRITE_MEM_ADDR_FROM_VALUE

	return 1;
}

static inline int mx_read_chip_mem_by_cnf(const mx_conf_t *conf,
		mx_maddr_t addr, mx_lsval_t *out_value)
{
	return conf->read_chip_mem_fn(conf->access_size_idx, addr, out_value);
}

static inline int mx_write_chip_mem_by_cnf(const mx_conf_t *conf,
		mx_maddr_t addr, mx_lsval_t value)
{
	return conf->write_chip_mem_fn(conf->access_size_idx, addr, value);
}

static int mx_init_conf_opts(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos)
{
	static const char *str_inv_val_n = "Invalid value for opt '%s'\n";

	mx_access_size_idx_t access_size_idx             = MX_ACCS_1;
	mx_access_endianness_idx_t access_endianness_idx = MX_AEND_BBRAM;
	mx_dump_mode_idx_t dump_mode_idx                 = MX_DMPM_HEX;
	uint8_t dump_cols                                = 0;

	const mx_accs_t *accs_s, *accs1_s;
	const mx_aend_t *aend_s;
	const mx_dmpm_t *dmpm_s;
	int accs_bytes;

	int i = *inout_argv_pos;
	int value;
	const char *opt_name;
	int opt_index;

	for (; i < argc; ++i) {
		if (mx_strstr(argv[i], "-") != argv[i])
			break;
		opt_name = argv[i];
		opt_index = mx_find_opts_idx_by_name(opt_name);
		if (opt_index < 0)
			MX_RETCPRINTF(0, "Unknown option '%s'\n", opt_name);
		if (++i >= argc)
			MX_RETCPRINTF(0, str_inv_val_n, opt_name);
		switch (opt_index) {
		case MX_OPT_ACCESS_SIZE:
			value = mx_find_accs_idx_by_name(argv[i]);
			if (value < 0)
				MX_RETCPRINTF(0, str_inv_val_n, opt_name);
			access_size_idx = value;
			break;
		case MX_OPT_ACCESS_ENDIANNESS:
			value = mx_find_aend_idx_by_name(argv[i]);
			if (value < 0)
				MX_RETCPRINTF(0, str_inv_val_n, opt_name);
			access_endianness_idx = value;
			break;
		case MX_OPT_DUMP_MODE:
			value = mx_find_dmpm_idx_by_name(argv[i]);
			if (value < 0)
				MX_RETCPRINTF(0, str_inv_val_n, opt_name);
			dump_mode_idx = value;
			break;
		case MX_OPT_DUMP_COLS:
			if (!mx_strtouX(argv[i], MX_STUX_U8, &dump_cols, 0))
				MX_RETCPRINTF(0, str_inv_val_n, opt_name);
			break;
		}
	}

	accs_s = mx_get_accs_s(access_size_idx);
	aend_s = mx_get_aend_s(access_endianness_idx);

	if (!mx_is_chip_mem_accs_supported(access_size_idx,
			access_endianness_idx))
		MX_RETCPRINTF(0, "Chip does not support access size '%s' with"
		                 " endianness '%s'\n",
				accs_s->name_type, aend_s->name_long);

#define _MX_DUMP_COLS_SET_AUTO(if_hex_val, else_val) \
		dump_cols = (dump_mode_idx == MX_DMPM_HEX ? \
				if_hex_val : else_val); \
		break

	if (dump_cols == 0) {
		switch (accs_s->bytes) {
		case 1: _MX_DUMP_COLS_SET_AUTO(16, 4);
		case 2: _MX_DUMP_COLS_SET_AUTO(8, 2);
		case 4: _MX_DUMP_COLS_SET_AUTO(4, 1);
#ifdef MX_HAS_U64
		case 8: _MX_DUMP_COLS_SET_AUTO(2, 1);
#endif
		default: MX_RETCPRINTF(0, "Unsupported access size '%s' for"
		                          " dump columns auto size\n",
		                       accs_s->name_type);
		}
	}

#undef _MX_DUMP_COLS_SET_AUTO

	conf->access_size_idx       = access_size_idx;
	conf->access_endianness_idx = access_endianness_idx;
	conf->dump_mode_idx         = dump_mode_idx;
	conf->dump_cols             = dump_cols;

	if (access_endianness_idx == MX_AEND_BBRAM) {
		conf->read_chip_mem_fn  = &mx_read_chip_mem_bbram;
		conf->write_chip_mem_fn = &mx_write_chip_mem_bbram;
	} else {
		conf->read_chip_mem_fn  = &mx_read_chip_mem_native;
		conf->write_chip_mem_fn = &mx_write_chip_mem_native;
	}

	conf->dump_fn = (conf->dump_mode_idx == MX_DMPM_HEX ?
			&mx_dump_hex : &mx_dump_bits);

	*inout_argv_pos = i;

	accs1_s = mx_get_accs_s(MX_ACCS_1);
	dmpm_s = mx_get_dmpm_s(dump_mode_idx);
	accs_bytes = accs_s->bytes;
	mx_cprintf("> Access Size: %s (%d %s)", accs_s->name_type, accs_bytes,
			(accs_bytes > 1 ? accs1_s->name_plural
			                : accs1_s->name_singular));
	mx_cprintf(" | Access Endianness: %s\n", aend_s->name_long);
	mx_cprintf("> Dump Mode: %s", dmpm_s->name_long);
	mx_cprintf(" | Dump Cols: %" MX_PRIu8, dump_cols);
	mx_cprintf("\n");

	return 1;
}

static inline void mx_dump_task_mem_data_bit(const mx_conf_t *conf,
	mx_lsval_t value)
{
	mx_cprintf("%" MX_PRIuLSVAL, value);
}

static inline void mx_dump_task_mem_data_value(const mx_conf_t *conf,
	mx_lsval_t value, int include_bits)
{
	mx_dump_hex_by_cnf(conf, value, 1);
	if (!include_bits)
		return;
	mx_cprintf(" [");
	mx_dump_bits_by_cnf(conf, value, 1);
	mx_cprintf("]");
}

static void mx_cprintf_task(const mx_conf_t *conf, mx_task_idx_t task_idx)
{
	const mx_task_t *task;
	if (task_idx == MX_TASK_NONE)
		return;
	task = mx_get_task_s(task_idx);
	mx_cprintf("> Task: %s\n", task->name_sl.name_long);
}

static void mx_cprintf_task_mem(const mx_conf_t *conf,
		const mx_task_mem_region_t *tmem_region,
		const mx_task_mem_data_t *tmem_data,
		const mx_task_mem_bit_t *tmem_bit,
		const mx_task_mem_value_t *tmem_value)
{
	const mx_mem_region_t *mem_region;
	const mx_mem_data_t *mem_data;
	const mx_mem_bit_t *mem_bit;

	if (tmem_region && tmem_region->parse != MX_TMPC_UNSPEC) {
		mem_region = &tmem_region->mem_region;
		mx_cprintf("> Memory region > Addr: ");
		mx_dump_hex(MX_ACCS_MADDR, mem_region->addr, 1);
		if (mem_region->len > 0)
			mx_cprintf(" | Length: %" MX_PRIuMADDR,
					mem_region->len);
		mx_cprintf("\n");
	}

	if (tmem_data && tmem_data->parse != MX_TMPC_UNSPEC) {
		mem_data = &tmem_data->mem_data;
		mx_cprintf("> Region data > Offset: ");
		mx_dump_hex(MX_ACCS_MADDR, mem_data->offset, 1);
		mx_cprintf("\n");
	}

	if (tmem_bit && tmem_bit->parse != MX_TMPC_UNSPEC) {
		mem_bit = &tmem_bit->mem_bit;
		mx_cprintf("> Data bit > Index: %" MX_PRIu8, mem_bit->index);
		mx_cprintf("\n");
	}

	if (tmem_value && tmem_value->parse != MX_TMPV_UNSPEC) {
		mx_cprintf("> Set ");
		switch (tmem_value->parse) {
		case MX_TMPV_BIT:  mx_cprintf("bit ");  break;
		case MX_TMPV_DATA: mx_cprintf("data "); break;
		default: break;
		}
		mx_cprintf("value: ");
		if (tmem_value->parse == MX_TMPV_BIT) {
			mx_dump_task_mem_data_bit(conf, tmem_value->value);
			mx_cprintf("\n");
			return;
		}
		mx_dump_task_mem_data_value(conf, tmem_value->value, 1);
		mx_cprintf("\n");
	}
}

static void mx_cprintf_task_mem_data_value_prefix(const mx_conf_t *conf,
		const mx_task_mem_region_t *tmem_region,
		const mx_task_mem_data_t *tmem_data)
{
	if (!tmem_region || tmem_region->parse == MX_TMPC_UNSPEC ||
	    !tmem_data || tmem_data->parse == MX_TMPC_UNSPEC)
		return;
	mx_cprintf("%s ", mx_get_accs_s_by_cnf(conf)->name_type);
	mx_dump_hex(MX_ACCS_MADDR, tmem_data->c_addr, 1);
	mx_cprintf(": ");
}

static void mx_cprintf_task_mem_data_bit_prefix(const mx_conf_t *conf,
		const mx_task_mem_bit_t *tmem_bit)
{
	const mx_mem_bit_t *mem_bit;
	if (!tmem_bit || tmem_bit->parse == MX_TMPC_UNSPEC)
		return;
	mem_bit = &tmem_bit->mem_bit;
	mx_cprintf("B[%" MX_PRIu8 "]: ", mem_bit->index);
}

static inline int mx_abort_task_if_accs_problem(int accs_bytes,
		mx_maddr_t length)
{
	mx_assert(length > 0);
	if (length % accs_bytes != 0)
		MX_RETCPRINTF(1, "Invalid access size, length (%" MX_PRIuMADDR
		                 ") MOD access size (%d) != 0\n",
				length, accs_bytes);
	return 0;
}

static inline int mx_parse_uX(const char *arg, mx_strtouX_type_t result_type,
		void *out_result)
{
	return mx_strtouX(arg, result_type, out_result, 0);
}

static inline int mx_parse_mem_addr_and_len(const char *arg,
		mx_maddr_t *out_addr, mx_maddr_t *out_len)
{
	char *s;
	if (!mx_strtouX_ex(arg, &s, MX_STUX_MADDR, out_addr, 0) || *s != '+')
		return 0;
	return mx_strtouX(++s, MX_STUX_MADDR, out_len, 0);
}

static int mx_parse_task_mem_region(const mx_conf_t *conf, const char *arg,
		mx_parse_task_mem_t *tmem_parse)
{
	int accs_bytes;
	mx_maddr_t addr, len;
	mx_mem_region_t *mem_region;
	mx_task_mem_region_t *tmem_region = tmem_parse->tmem_region;

	if (!tmem_region)
		return 0;

	if (tmem_parse->flags & MX_PMF_REGION_VALUE_HAS_LEN) {
		if (!mx_parse_mem_addr_and_len(arg, &addr, &len))
			return 0;
	} else {
		if (!mx_parse_uX(arg, MX_STUX_MADDR, &addr))
			return 0;
		len = 0;
	}

	if ((tmem_parse->flags & MX_PMF_REGION_ACCS_CHECK) && len > 0) {
		accs_bytes = mx_get_accs_s_by_cnf(conf)->bytes;
		if (mx_abort_task_if_accs_problem(accs_bytes, len))
			return 0;
	}

	tmem_region->parse = MX_TMPC_VALUE;
	mem_region = &tmem_region->mem_region;
	mem_region->addr = addr;
	mem_region->len = len;

	return 1;
}

static int mx_parse_task_mem_data(const mx_conf_t *conf, const char *arg,
		mx_parse_task_mem_t *tmem_parse)
{
	mx_maddr_t offset;
	mx_mem_data_t *mem_data;
	const mx_task_mem_region_t *tmem_region = tmem_parse->tmem_region;
	mx_task_mem_data_t *tmem_data = tmem_parse->tmem_data;

	if (!tmem_region || !tmem_data ||
	    !mx_parse_uX(arg, MX_STUX_MADDR, &offset))
		return 0;

	tmem_data->parse = MX_TMPC_VALUE;
	mem_data = &tmem_data->mem_data;
	mem_data->offset = offset;
	tmem_data->c_addr = tmem_region->mem_region.addr + mem_data->offset;

	return 1;
}

static int mx_parse_task_mem_bit(const mx_conf_t *conf, const char *arg,
		mx_parse_task_mem_t *tmem_parse)
{
	uint8_t index, index_max;
	mx_mem_bit_t *mem_bit;
	const mx_task_mem_data_t *tmem_data = tmem_parse->tmem_data;
	mx_task_mem_bit_t *tmem_bit = tmem_parse->tmem_bit;

	if (!tmem_data || !tmem_bit || !mx_parse_uX(arg, MX_STUX_U8, &index))
		return 0;

	tmem_bit->parse = MX_TMPC_VALUE;
	mem_bit = &tmem_bit->mem_bit;
	mem_bit->index = index;

	index_max = mx_get_accs_s_by_cnf(conf)->index_max;
	if (index > index_max)
		MX_RETCPRINTF(0, "Bit index %" MX_PRIu8 " is out of range,"
		                 " valid is 0-%" MX_PRIu8 "\n",
				index, index_max);

	return 1;
}

static int mx_parse_task_mem_value(const mx_conf_t *conf, const char *arg,
		mx_parse_task_mem_t *tmem_parse)
{
	mx_lsval_t value, value_max;
	mx_task_mem_value_t *tmem_value = tmem_parse->tmem_value;

	if (!tmem_value || !mx_parse_uX(arg, MX_STUX_LSVAL, &value))
		return 0;

	switch (tmem_value->parse) {
	case MX_TMPV_BIT:
		value_max = 1;
		break;
	case MX_TMPV_DATA:
	case MX_TMPV_UNSPEC:
		value_max = mx_get_accs_s_by_cnf(conf)->value_max;
		break;
	default:
		return 0;
	}

	if (value > value_max) {
		mx_cprintf("Value %" MX_PRIuLSVAL " [", value);
		mx_dump_hex(MX_ACCS_LSVAL, value, 1);
		mx_cprintf("] is out of range, valid is 0-%" MX_PRIuLSVAL " [",
				value_max);
		mx_dump_hex_by_cnf(conf, value_max, 1);
		mx_cprintf("]\n");
		return 0;
	}

	if (tmem_value->parse == MX_TMPV_UNSPEC)
		tmem_value->parse = (value > 1 ? MX_TMPV_DATA : MX_TMPV_BIT);

	tmem_value->value = value;

	return 1;
}

static int mx_parse_task_mem(const mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos, mx_parse_task_mem_t *tmem_parse)
{
	static const char *str_missing_n = "Missing '%s'\n";
	static const char *str_not_parsed_n = "Unable to parse '%s'\n";
	static const char *str_region = "REGION-ADDR";
	static const char *str_region_len = "REGION-ADDR+LEN";
	static const char *str_data = "DATA-OFFSET";
	static const char *str_bit = "BIT-INDEX";
	static const char *str_value = "VALUE";
	static const char *str_value_data = "DATA-VALUE";
	static const char *str_value_bit = "BIT-VALUE";

	mx_task_mem_region_t *tmem_region = tmem_parse->tmem_region;
	mx_task_mem_data_t *tmem_data = tmem_parse->tmem_data;
	mx_task_mem_bit_t *tmem_bit = tmem_parse->tmem_bit;
	mx_task_mem_value_t *tmem_value = tmem_parse->tmem_value;
	int flags = tmem_parse->flags;

	int region_done = (!tmem_region ? 1 : 0);
	int data_done   = (!tmem_data ? 1 : 0);
	int bit_done    = (!tmem_bit ? 1 : 0);
	int value_done  = (!tmem_value ? 1 : 0);

	int region_req = !(flags & MX_PMF_OPT_REGION);
	int data_req   = !(flags & MX_PMF_OPT_DATA);
	int bit_req    = !(flags & MX_PMF_OPT_BIT);
	int value_req  = !(flags & MX_PMF_OPT_VALUE);

	int i = *inout_argv_pos;

	const char *str_region_res = (flags & MX_PMF_REGION_VALUE_HAS_LEN ?
			str_region_len : str_region);

	const char *str_value_res;
	switch (tmem_value->parse) {
	case MX_TMPV_DATA: str_value_res = str_value_data;
	case MX_TMPV_BIT:  str_value_res = str_value_bit;
	default:           str_value_res = str_value;
	}

	for (; i < argc; ++i) {
		if (!region_done) {
			region_done = 1;
			if (mx_parse_task_mem_region(conf, argv[i],
					tmem_parse))
				continue;
			else if (region_req)
				MX_RETCPRINTF(0, str_not_parsed_n,
						str_region_res);
		}
		if (!data_done) {
			data_done = 1;
			if (mx_parse_task_mem_data(conf, argv[i], tmem_parse))
				continue;
			else if (data_req)
				MX_RETCPRINTF(0, str_not_parsed_n, str_data);
		}
		if (!bit_done) {
			bit_done = 1;
			if (mx_parse_task_mem_bit(conf, argv[i], tmem_parse))
				continue;
			else if (bit_req)
				MX_RETCPRINTF(0, str_not_parsed_n, str_bit);
		}
		if (!value_done) {
			value_done = 1;
			if (mx_parse_task_mem_value(conf, argv[i], tmem_parse))
				continue;
			else if (value_req)
				MX_RETCPRINTF(0, str_not_parsed_n,
						str_value_res);
		}
		MX_RETCPRINTF(0, "Too many args for task '%s'\n",
			mx_get_task_s(tmem_parse->task_idx)->name_sl.name_long);
	}

	if (!region_done && region_req)
		MX_RETCPRINTF(0, str_missing_n, str_region_res);
	if (!data_done && data_req)
		MX_RETCPRINTF(0, str_missing_n, str_data);
	if (!bit_done && bit_req)
		MX_RETCPRINTF(0, str_missing_n, str_bit);
	if (!value_done && value_req)
		MX_RETCPRINTF(0, str_missing_n, str_value_res);

	*inout_argv_pos = i;

	return 1;
}

static inline mx_lsval_t mx_get_lsval_bit(mx_lsval_t lsv, uint8_t index)
{
	return (uint8_t)((lsv >> index) & 1);
}

static inline mx_lsval_t mx_set_lsval_bit(mx_lsval_t lsv, uint8_t index,
		uint8_t bit_value)
{
	return (mx_lsval_t)((lsv & ~(1 << index)) | ((bit_value & 1) << index));
}

static void _mx_cprintf_getset_task_mem_bit(const mx_conf_t *conf,
		const mx_task_mem_bit_t *tmem_bit, mx_lsval_t value)
{
	if (!tmem_bit)
		return;
	mx_cprintf(" - ");
	mx_cprintf_task_mem_data_bit_prefix(conf, tmem_bit);
	mx_dump_task_mem_data_bit(conf,
			mx_get_lsval_bit(value, tmem_bit->mem_bit.index));
}

static int mx_getset_task_mem_bitorval(const mx_conf_t *conf,
		const mx_task_mem_region_t *tmem_region,
		const mx_task_mem_data_t *tmem_data,
		const mx_task_mem_bit_t *tmem_bit,
		const mx_task_mem_value_t *tmem_value)
{
	static const char *str_n_spacer      = "\n        ";
	static const char *str_n_spacer_nval = "\n     -> ";

	const mx_accs_t *accs_s;
	const mx_mem_bit_t *mem_bit;
	mx_maddr_t daddr;
	mx_lsval_t value, value_new, value_upd;

	if (!tmem_region || !tmem_data)
		return 0;

	accs_s = mx_get_accs_s_by_cnf(conf);
	mem_bit = (tmem_bit ? &tmem_bit->mem_bit : NULL);
	daddr = tmem_data->c_addr;

	mx_cprintf((tmem_value ? "Setting" : "Getting"));
	mx_cprintf(" %s data %s...\n", accs_s->name_singular,
			(tmem_bit ? "bit" : "value"));

	if (!mx_read_chip_mem_by_cnf(conf, daddr, &value))
		return 0;

	mx_cprintf_task_mem_data_value_prefix(conf, tmem_region, tmem_data);
	mx_cprintf(str_n_spacer);
	mx_dump_task_mem_data_value(conf, value, 1);
	_mx_cprintf_getset_task_mem_bit(conf, tmem_bit, value);

	if (!tmem_value) {
		mx_cprintf("\n");
		return 1;
	}
	mx_cprintf(str_n_spacer_nval);

	value_new = (tmem_bit ? mx_set_lsval_bit(value, mem_bit->index,
	                                         tmem_value->value)
	                      : tmem_value->value);

	mx_dump_task_mem_data_value(conf, value_new, 1);
	_mx_cprintf_getset_task_mem_bit(conf, tmem_bit, value_new);
	mx_cprintf("\n");

	if (value == value_new)
		MX_RETCPRINTF(1, "No change, not updating\n");

	if (!mx_write_chip_mem_by_cnf(conf, daddr, value_new))
		return 0;

	mx_udelay(100);

	if (!mx_read_chip_mem_by_cnf(conf, daddr, &value_upd))
		return 0;

	if (value_upd != value_new) {
		mx_cprintf("Failed to set new value\n");
		mx_cprintf_task_mem_data_value_prefix(conf, tmem_region,
			tmem_data);
		mx_cprintf("[CURRENT VALUE]:");
		mx_cprintf(str_n_spacer);
		mx_dump_task_mem_data_value(conf, value_upd, 1);
		_mx_cprintf_getset_task_mem_bit(conf, tmem_bit, value_upd);
		mx_cprintf("\n");
		return 0;
	}

	return 1;
}

static int _mx_init_conf_task_DUMP(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos)
{
	mx_task_DUMP_data_t dump_data;
	mx_parse_task_mem_t tmem_parse = {
		MX_TASK_DUMP,
		&dump_data.tmem_region,
		NULL,
		NULL,
		NULL,
		(MX_PMF_REGION_VALUE_HAS_LEN | MX_PMF_REGION_ACCS_CHECK)
	};
	memset(&dump_data, 0, sizeof(mx_task_DUMP_data_t));
	mx_cprintf_task(conf, tmem_parse.task_idx);
	if (!mx_parse_task_mem(conf, argc, argv, inout_argv_pos, &tmem_parse))
		return 0;
	_mx_set_conf_task_DUMP(conf, &dump_data);
	mx_cprintf_task_mem(conf, tmem_parse.tmem_region, NULL, NULL, NULL);
	return 1;
}
static int _mx_run_task_DUMP(const mx_conf_t *conf)
{
	const mx_task_DUMP_data_t *dump_data;
	const mx_mem_region_t *mem_region;
	const mx_accs_t *accs1_s;
	int accs_bytes;
	mx_maddr_t raddr, rlen, i_r, dump_cols_res;
	mx_lsval_t rvalue;

	dump_data = _mx_get_conf_task_DUMP_data(conf);
	mx_assert(dump_data != NULL);

	accs1_s = mx_get_accs_s(MX_ACCS_1);
	accs_bytes = mx_get_accs_s_by_cnf(conf)->bytes;
	dump_cols_res = conf->dump_cols * accs_bytes;

	mem_region = &dump_data->tmem_region.mem_region;
	raddr = mem_region->addr;
	rlen = mem_region->len;

	mx_cprintf("Dumping region '");
	mx_dump_hex(MX_ACCS_MADDR, raddr, 1);
	mx_cprintf("' %" MX_PRIuMADDR " %s...\n",
			rlen, (rlen > 1 ? accs1_s->name_plural
					: accs1_s->name_singular));

	for (i_r = 0; i_r < rlen;) {
		mx_dump_hex(MX_ACCS_MADDR, i_r, 0);
		mx_cprintf(":");
		do {
			mx_cprintf(" ");
			if (!mx_read_chip_mem_by_cnf(conf, raddr + i_r,
					&rvalue))
				return 0;
			mx_dump_by_cnf(conf, rvalue, 0);
			i_r += accs_bytes;
		} while (i_r < rlen && i_r % dump_cols_res != 0);
		mx_cprintf("\n");
	}

	return 1;
}

static int _mx_init_conf_task_GETVALUE(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos)
{
	mx_task_GETVALUE_data_t getval_data;
	mx_parse_task_mem_t tmem_parse = {
		MX_TASK_GETVALUE,
		&getval_data.tmem_region,
		&getval_data.tmem_data,
		NULL,
		NULL,
		0
	};
	memset(&getval_data, 0, sizeof(mx_task_GETVALUE_data_t));
	mx_cprintf_task(conf, tmem_parse.task_idx);
	if (!mx_parse_task_mem(conf, argc, argv, inout_argv_pos, &tmem_parse))
		return 0;
	_mx_set_conf_task_GETVALUE(conf, &getval_data);
	mx_cprintf_task_mem(conf, tmem_parse.tmem_region, tmem_parse.tmem_data,
			NULL, NULL);
	return 1;
}
static int _mx_run_task_GETVALUE(const mx_conf_t *conf)
{
	const mx_task_GETVALUE_data_t *getval_data =
			_mx_get_conf_task_GETVALUE_data(conf);
	mx_assert(getval_data != NULL);
	return mx_getset_task_mem_bitorval(conf, &getval_data->tmem_region,
			&getval_data->tmem_data, NULL, NULL);
}

static int _mx_init_conf_task_SETVALUE(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos)
{
	mx_task_SETVALUE_data_t setval_data;
	mx_parse_task_mem_t tmem_parse = {
		MX_TASK_SETVALUE,
		&setval_data.tmem_region,
		&setval_data.tmem_data,
		NULL,
		&setval_data.tmem_value,
		0
	};
	memset(&setval_data, 0, sizeof(mx_task_SETVALUE_data_t));
	tmem_parse.tmem_value->parse = MX_TMPV_DATA;
	mx_cprintf_task(conf, tmem_parse.task_idx);
	if (!mx_parse_task_mem(conf, argc, argv, inout_argv_pos, &tmem_parse))
		return 0;
	_mx_set_conf_task_SETVALUE(conf, &setval_data);
	mx_cprintf_task_mem(conf, tmem_parse.tmem_region, tmem_parse.tmem_data,
			NULL, tmem_parse.tmem_value);
	return 1;
}
static int _mx_run_task_SETVALUE(const mx_conf_t *conf)
{
	const mx_task_SETVALUE_data_t *setval_data =
			_mx_get_conf_task_SETVALUE_data(conf);
	mx_assert(setval_data != NULL);
	return mx_getset_task_mem_bitorval(conf, &setval_data->tmem_region,
			&setval_data->tmem_data, NULL,
			&setval_data->tmem_value);
}

static int _mx_init_conf_task_GETBIT(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos)
{
	mx_task_GETBIT_data_t getbit_data;
	mx_parse_task_mem_t tmem_parse = {
		MX_TASK_GETBIT,
		&getbit_data.tmem_region,
		&getbit_data.tmem_data,
		&getbit_data.tmem_bit,
		NULL,
		0
	};
	memset(&getbit_data, 0, sizeof(mx_task_GETBIT_data_t));
	mx_cprintf_task(conf, tmem_parse.task_idx);
	if (!mx_parse_task_mem(conf, argc, argv, inout_argv_pos, &tmem_parse))
		return 0;
	_mx_set_conf_task_GETBIT(conf, &getbit_data);
	mx_cprintf_task_mem(conf, tmem_parse.tmem_region, tmem_parse.tmem_data,
			tmem_parse.tmem_bit, NULL);
	return 1;
}
static int _mx_run_task_GETBIT(const mx_conf_t *conf)
{
	const mx_task_GETBIT_data_t *getbit_data =
			_mx_get_conf_task_GETBIT_data(conf);
	mx_assert(getbit_data != NULL);
	return mx_getset_task_mem_bitorval(conf, &getbit_data->tmem_region,
			&getbit_data->tmem_data, &getbit_data->tmem_bit, NULL);
}

static int _mx_init_conf_task_SETBIT(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos)
{
	mx_task_SETBIT_data_t setbit_data;
	mx_parse_task_mem_t tmem_parse = {
		MX_TASK_SETBIT,
		&setbit_data.tmem_region,
		&setbit_data.tmem_data,
		&setbit_data.tmem_bit,
		&setbit_data.tmem_value,
		0
	};
	memset(&setbit_data, 0, sizeof(mx_task_SETBIT_data_t));
	tmem_parse.tmem_value->parse = MX_TMPV_BIT;
	mx_cprintf_task(conf, tmem_parse.task_idx);
	if (!mx_parse_task_mem(conf, argc, argv, inout_argv_pos, &tmem_parse))
		return 0;
	_mx_set_conf_task_SETBIT(conf, &setbit_data);
	mx_cprintf_task_mem(conf, tmem_parse.tmem_region, tmem_parse.tmem_data,
			tmem_parse.tmem_bit, tmem_parse.tmem_value);
	return 1;
}
static int _mx_run_task_SETBIT(const mx_conf_t *conf)
{
	const mx_task_SETBIT_data_t *setbit_data =
			_mx_get_conf_task_SETBIT_data(conf);
	mx_assert(setbit_data != NULL);
	return mx_getset_task_mem_bitorval(conf, &setbit_data->tmem_region,
			&setbit_data->tmem_data, &setbit_data->tmem_bit,
			&setbit_data->tmem_value);
}

static int mx_init_conf_task(mx_conf_t *conf, int argc, char **argv,
		int *inout_argv_pos)
{
	const char *task;
	int task_idx;
	conf->task_idx = MX_TASK_NONE;
	if (*inout_argv_pos >= argc)
		MX_RETCPRINTF(0, "No task specified\n");
	task = argv[(*inout_argv_pos)++];
	task_idx = mx_find_task_idx_by_name(task);
	if (task_idx < 0)
		MX_RETCPRINTF(0, "Unknown task '%s'\n", task);
	return mx_get_task_s(task_idx)->init_conf_fn(conf, argc, argv,
			inout_argv_pos);
}

static int mx_run_task(const mx_conf_t *conf)
{
	if (conf->task_idx == MX_TASK_NONE)
		MX_RETCPRINTF(0, "No task to run\n");
	return mx_get_task_s_by_cnf(conf)->run_task_fn(conf);
}

static int mx_init_conf(mx_conf_t *conf, int argc, char **argv)
{
	int argv_pos = 1;
	if (!mx_init_conf_opts(conf, argc, argv, &argv_pos))
		MX_RETCPRINTF(0, "Failed to init conf opt\n");
	if (!mx_init_conf_task(conf, argc, argv, &argv_pos))
		MX_RETCPRINTF(0, "Failed to init conf task\n");
	return 1;
}

static int command_mx_chipmem(int argc, char **argv)
{
	int ret = MX_EC_ERROR_INVAL;
	mx_conf_t conf = {0};
	mx_conf_t *pconf = &conf;
	if (mx_init_conf(pconf, argc, argv) && mx_run_task(pconf))
		ret = MX_EC_SUCCESS;
	mx_cprintf("Command exiting with code: %d\n", ret);
	return ret;
}
/*
mx_chipmem [OPTS] TASK [TASK_ARGS]

    [OPTS]:
        -a | --access-size ACCESS_SIZE = Set access size (DEFAULT: 1).
            ACCESS_SIZE:
                1 | u8  | byte  | bytes
                2 | u16 | word  | words
                4 | u32 | dword | dwords
                8 | u64 | qword | qwords (only on 64-bit systems)
        -c | --dump-cols DUMP_COLS     = Set dump column number (DEFAULT: 0).
            DUMP_COLS:
                0     = Auto, ACCESS_SIZE+DUMP_MODE=DUMP_COLS:
                            8+hex  = 2
                            8+bits = 1
                            4+hex  = 4
                            4+bits = 1
                            2+hex  = 8
                            2+bits = 2
                            1+hex  = 16
                            1+bits = 4
                1-255
        -e | --access-endianness ACCESS_ENDIANNESS = Set access endianness
                                         (DEFAULT: bbram). This only matters
                                         when access size is larger than 1 byte.
            ACCESS_ENDIANNESS:
                b | bbram  = Endianness adjusted to BBRAM.
                n | native = Native endianness.
        -m | --dump-mode DUMP_MODE     = Set dump mode (DEFAULT: hex).
            DUMP_MODE:
                b | bits
                h | hex

    TASK:
        d  | dump      REGION-ADDR+LEN         = Dumps region.
        gv | get-value REGION-ADDR DATA-OFFSET = Gets region data value.
        sv | set-value REGION-ADDR DATA-OFFSET
                       DATA-VALUE              = Sets region data value.
        gb | get-bit   REGION-ADDR DATA-OFFSET
                       BIT-INDEX               = Gets region data bit value.
        sb | set-bit   REGION-ADDR DATA-OFFSET
                       BIT-INDEX BIT-VALUE     = Sets region data bit value.

    TASK_ARGS:
        REGION-ADDR = Region address (hex or dec).
        LEN         = Length (hex or dec).
        DATA-OFFSET = Data offset (hex or dec).
        BIT-INDEX   = Bit index (hex or dec).
        DATA-VALUE  = Data value (0-MAX, MAX based on access size).
        BIT-VALUE   = Bit value (0|1).

Example: mx_chipmem dump 0x0+64
         mx_chipmem sv 0x0 0x1 0xff
*/
MX_DECLARE_CONSOLE_COMMAND(mx_chipmem, command_mx_chipmem,
		"[OPTS] TASK [TASK_ARGS]",
		"Dump, get, or set chip memory");

static int command_mx_sysrflags(int argc, char **argv)
{
	uint32_t cur_flags, set_flags;
	cur_flags = system_get_reset_flags();
	if (argc < 2) {
		mx_cprintf("Current flags:\n");
		mx_cprintf("        0x%08" MX_PRIx32 "\n", cur_flags);
		return MX_EC_SUCCESS;
	}
	if (argc > 2)
		MX_RETCPRINTF(MX_EC_ERROR_INVAL, "Too many args\n");
	if (!mx_strtouX(argv[1], MX_STUX_U32, &set_flags, 0))
		MX_RETCPRINTF(MX_EC_ERROR_INVAL, "Invalid set value\n");
	mx_cprintf("Setting flags:\n");
	mx_cprintf("        0x%08" MX_PRIx32 "\n", cur_flags);
	mx_cprintf("     -> 0x%08" MX_PRIx32 "\n", set_flags);
	system_clear_reset_flags(MX_UINT32_MAX);
	system_set_reset_flags(set_flags);
	mx_udelay(100);
	cur_flags = system_get_reset_flags();
	if (set_flags != cur_flags) {
		mx_cprintf("Failed to set flags, current: 0x%08" MX_PRIx32 "\n",
				cur_flags);
		return MX_EC_ERROR_INVAL;
	}
	return MX_EC_SUCCESS;
}
MX_DECLARE_CONSOLE_COMMAND(mx_sysrflags, command_mx_sysrflags,
		"[UINT32_HEX_OR_DEC]",
		"Get or set all active system reset flags");

/* BEGIN CHIP SPECIFIC */
/* Based on target Nuvoton NPCX586G ARMv7 Cortex-M4 (firmware-fizz-10139.B).
 * chip/npcx/registers.h
 * chip/npcx/system_chip.h
 * chip/npcx/system.c
 * include/system.h
 */

/*
#define NPCX_ESPI_BASE_ADDR              0x4000A000
#define NPCX_MDC_BASE_ADDR               0x4000C000
#define NPCX_PMC_BASE_ADDR               0x4000D000
#define NPCX_SIB_BASE_ADDR               0x4000E000
#define NPCX_SHI_BASE_ADDR               0x4000F000
#define NPCX_SHM_BASE_ADDR               0x40010000
#define NPCX_GDMA_BASE_ADDR              0x40011000
#define NPCX_FIU_BASE_ADDR               0x40020000
#define NPCX_KBSCAN_REGS_BASE            0x400A3000
#define NPCX_GLUE_REGS_BASE              0x400A5000
#define NPCX_BBRAM_BASE_ADDR             0x400AF000
#define NPCX_HFCG_BASE_ADDR              0x400B5000
#define NPCX_LFCG_BASE_ADDR              0x400B5100
#define NPCX_MTC_BASE_ADDR               0x400B7000
#define NPCX_MSWC_BASE_ADDR              0x400C1000
#define NPCX_SCFG_BASE_ADDR              0x400C3000
#define NPCX_CR_UART_BASE_ADDR           0x400C4000
#define NPCX_KBC_BASE_ADDR               0x400C7000
#define NPCX_ADC_BASE_ADDR               0x400D1000
#define NPCX_SPI_BASE_ADDR               0x400D2000
#define NPCX_PECI_BASE_ADDR              0x400D4000
#define NPCX_TWD_BASE_ADDR               0x400D8000

// Battery-Backed RAM (BBRAM) Registers
#define NPCX_BKUP_STS                REG8(NPCX_BBRAM_BASE_ADDR + 0x100)
#define NPCX_BBRAM(offset)           REG8(NPCX_BBRAM_BASE_ADDR + offset)

// BBRAM register fields
#define NPCX_BKUP_STS_IBBR               7
#define NPCX_BBRAM_SIZE                  64  // Size of BBRAM

// Flags for BBRM_DATA_INDEX_WAKE
#define HIBERNATE_WAKE_MTC        (1 << 0)  // MTC alarm
#define HIBERNATE_WAKE_PIN        (1 << 1)  // Wake pin

// Indices for battery-backed ram (BBRAM) data position
enum bbram_data_index {
	BBRM_DATA_INDEX_SCRATCHPAD = 0,        // General-purpose scratchpad
	BBRM_DATA_INDEX_SAVED_RESET_FLAGS = 4, // Saved reset flags
	BBRM_DATA_INDEX_WAKE = 8,	       // Wake reasons for hibernate
	BBRM_DATA_INDEX_PD0 = 12,	       // USB-PD saved port0 state
	BBRM_DATA_INDEX_PD1 = 13,	       // USB-PD saved port1 state
	BBRM_DATA_INDEX_TRY_SLOT = 14,         // Vboot EC try slot
	BBRM_DATA_INDEX_VBNVCNTXT = 16,	       // VbNvContext for ARM arch
	BBRM_DATA_INDEX_RAMLOG = 32,	       // RAM log for Booter
	BBRM_DATA_INDEX_PANIC_FLAGS = 35,      // Flag to indicate validity of
	                                       // panic data starting at index
	                                       // 36.
	BBRM_DATA_INDEX_PANIC_BKUP = 36,       // Panic data (index 35-63)
};

// Return true if index is stored as a single byte in bbram
static int bbram_is_byte_access(enum bbram_data_index index)
{
	return (index >= BBRM_DATA_INDEX_VBNVCNTXT &&
		index <  BBRM_DATA_INDEX_RAMLOG)
#ifdef CONFIG_USB_PD_DUAL_ROLE
		|| index == BBRM_DATA_INDEX_PD0
		|| index == BBRM_DATA_INDEX_PD1
#endif
		|| index == BBRM_DATA_INDEX_PANIC_FLAGS
	;
}

// Reset causes
#define RESET_FLAG_OTHER       (1 << 0)   // Other known reason
#define RESET_FLAG_RESET_PIN   (1 << 1)   // Reset pin asserted
#define RESET_FLAG_BROWNOUT    (1 << 2)   // Brownout
#define RESET_FLAG_POWER_ON    (1 << 3)   // Power-on reset
#define RESET_FLAG_WATCHDOG    (1 << 4)   // Watchdog timer reset
#define RESET_FLAG_SOFT        (1 << 5)   // Soft reset trigger by core
#define RESET_FLAG_HIBERNATE   (1 << 6)   // Wake from hibernate
#define RESET_FLAG_RTC_ALARM   (1 << 7)   // RTC alarm wake
#define RESET_FLAG_WAKE_PIN    (1 << 8)   // Wake pin triggered wake
#define RESET_FLAG_LOW_BATTERY (1 << 9)   // Low battery triggered wake
#define RESET_FLAG_SYSJUMP     (1 << 10)  // Jumped directly to this image
#define RESET_FLAG_HARD        (1 << 11)  // Hard reset from software
#define RESET_FLAG_AP_OFF      (1 << 12)  // Do not power on AP
#define RESET_FLAG_PRESERVED   (1 << 13)  // Some reset flags preserved from
                                          // previous boot
#define RESET_FLAG_USB_RESUME  (1 << 14)  // USB resume triggered wake
#define RESET_FLAG_RDD         (1 << 15)  // USB Type-C debug cable
#define RESET_FLAG_RBOX        (1 << 16)  // Fixed Reset Functionality
#define RESET_FLAG_SECURITY    (1 << 17)  // Security threat

// Power Management Controller (PMC) Registers
#define NPCX_PMCSR                     REG8(NPCX_PMC_BASE_ADDR + 0x000)
#define NPCX_ENIDL_CTL                 REG8(NPCX_PMC_BASE_ADDR + 0x003)
#define NPCX_DISIDL_CTL                REG8(NPCX_PMC_BASE_ADDR + 0x004)
#define NPCX_DISIDL_CTL1               REG8(NPCX_PMC_BASE_ADDR + 0x005)
#define NPCX_PWDWN_CTL_ADDR(offset)    (((offset) < 6) ? \
			(NPCX_PMC_BASE_ADDR + 0x008 + (offset)) : \
			(NPCX_PMC_BASE_ADDR + 0x024))
#define NPCX_PWDWN_CTL(offset)         REG8(NPCX_PWDWN_CTL_ADDR(offset))

// PMC register fields
#define NPCX_PMCSR_DI_INSTW              0
#define NPCX_PMCSR_DHF                   1
#define NPCX_PMCSR_IDLE                  2
#define NPCX_PMCSR_NWBI                  3
#define NPCX_PMCSR_OHFC                  6
#define NPCX_PMCSR_OLFC                  7
#define NPCX_DISIDL_CTL_RAM_DID          5
#define NPCX_ENIDL_CTL_ADC_LFSL          7
#define NPCX_ENIDL_CTL_LP_WK_CTL         6
#define NPCX_ENIDL_CTL_PECI_ENI          2
#define NPCX_ENIDL_CTL_ADC_ACC_DIS       1
#define NPCX_PWDWN_CTL1_KBS_PD           0
#define NPCX_PWDWN_CTL1_SDP_PD           1
#define NPCX_PWDWN_CTL1_FIU_PD           2
#define NPCX_PWDWN_CTL1_PS2_PD           3
#define NPCX_PWDWN_CTL1_UART_PD          4
#define NPCX_PWDWN_CTL1_MFT1_PD          5
#define NPCX_PWDWN_CTL1_MFT2_PD          6
#define NPCX_PWDWN_CTL1_MFT3_PD          7
#define NPCX_PWDWN_CTL2_PWM0_PD          0
#define NPCX_PWDWN_CTL2_PWM1_PD          1
#define NPCX_PWDWN_CTL2_PWM2_PD          2
#define NPCX_PWDWN_CTL2_PWM3_PD          3
#define NPCX_PWDWN_CTL2_PWM4_PD          4
#define NPCX_PWDWN_CTL2_PWM5_PD          5
#define NPCX_PWDWN_CTL2_PWM6_PD          6
#define NPCX_PWDWN_CTL2_PWM7_PD          7
#define NPCX_PWDWN_CTL3_SMB0_PD          0
#define NPCX_PWDWN_CTL3_SMB1_PD          1
#define NPCX_PWDWN_CTL3_SMB2_PD          2
#define NPCX_PWDWN_CTL3_SMB3_PD          3
#define NPCX_PWDWN_CTL3_GMDA_PD          7
#define NPCX_PWDWN_CTL4_ITIM1_PD         0
#define NPCX_PWDWN_CTL4_ITIM2_PD         1
#define NPCX_PWDWN_CTL4_ITIM3_PD         2
#define NPCX_PWDWN_CTL4_ADC_PD           4
#define NPCX_PWDWN_CTL4_PECI_PD          5
#define NPCX_PWDWN_CTL4_PWM6_PD          6
#define NPCX_PWDWN_CTL4_SPIP_PD          7
#define NPCX_PWDWN_CTL5_SHI_PD           1
#define NPCX_PWDWN_CTL5_MRFSH_DIS        2
#define NPCX_PWDWN_CTL5_C2HACC_PD        3
#define NPCX_PWDWN_CTL5_SHM_REG_PD       4
#define NPCX_PWDWN_CTL5_SHM_PD           5
#define NPCX_PWDWN_CTL5_DP80_PD          6
#define NPCX_PWDWN_CTL5_MSWC_PD          7
#define NPCX_PWDWN_CTL6_ITIM4_PD         0
#define NPCX_PWDWN_CTL6_ITIM5_PD         1
#define NPCX_PWDWN_CTL6_ITIM6_PD         2
#define NPCX_PWDWN_CTL6_ESPI_PD          7

// NPCX_BBRAM (excl. BKUP_STS)       = -a 1 d 0x400AF000+64
// NPCX_BKUP_STS                     = -a 1 gv 0x400AF000 0x100
// BBRM_DATA_INDEX_SAVED_RESET_FLAGS = -a 4 gv 0x400AF000 4
// PMC                               = -a 1 d 0x4000D000+0x024

*/

static int mx_is_chip_mem_accs_supported(mx_access_size_idx_t access_size_idx,
		mx_access_endianness_idx_t access_endianness_idx)
{
	switch (mx_get_accs_s(access_size_idx)->bytes) {
	case 1:
	case 2:
	case 4:
		return 1;
	}
	return 0;
}

static int mx_read_chip_mem_native(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t *out_value)
{
	return mx_read_mem_native(access_size_idx, addr, out_value);
}

static int mx_write_chip_mem_native(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t value)
{
	return mx_write_mem_native(access_size_idx, addr, value);
}

static int mx_read_chip_mem_bbram(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t *out_value)
{
	int read_bytes = mx_get_accs_s(access_size_idx)->bytes;
	mx_lsval_t value = 0;

	switch (read_bytes) {
	case 4:
		value += REG8(addr + 3);
		value = value << 8;
		value += REG8(addr + 2);
		value = value << 8;
	case 2:
		value += REG8(addr + 1);
		value = value << 8;
	case 1:
		value += REG8(addr);
		break;
	default:
		return 0;
	}

	*out_value = value;

	return 1;
}

static int mx_write_chip_mem_bbram(mx_access_size_idx_t access_size_idx,
		mx_maddr_t addr, mx_lsval_t value)
{
	int write_bytes = mx_get_accs_s(access_size_idx)->bytes;

	switch (write_bytes) {
	case 4:
		REG8(addr + 3) = (value >> 24) & 0xFF;
		REG8(addr + 2) = (value >> 16) & 0xFF;
	case 2:
		REG8(addr + 1) = (value >> 8) & 0xFF;
	case 1:
		REG8(addr) = value & 0xFF;
		break;
	default:
		return 0;
	}

	return 1;
}

/* END CHIP SPECIFIC */

/* END mx_chipmem */
