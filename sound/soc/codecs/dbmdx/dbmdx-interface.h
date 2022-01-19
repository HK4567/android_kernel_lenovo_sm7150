/*
 * dbmdx-interface.h  --  DBMDX interface definitions
 *
 * Copyright (C) 2014 DSP Group
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DBMDX_INTERFACE_H
#define _DBMDX_INTERFACE_H

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/kfifo.h>
#include "dbmdx-customer-def.h"
#include <dt-bindings/sound/dbmdx-va-regmap.h>

#define DBMDX_I2S_AS_MASTER 1

#if defined(CONFIG_SND_SOC_DBMDX_COMPAT)
#include "dbmdx-compat.h"
#else
#include <linux/atomic.h>
#endif


#if defined(CONFIG_DBMDX_NO_DTS_SUPPORT)
#if IS_ENABLED(CONFIG_OF)
#undef CONFIG_OF
#endif /* CONFIG_OF */
#endif

#if defined(CONFIG_SND_SOC_DBMDX_DBMD2_DRIVES_DCLASS_SPEAKER)
#if !defined(DBMD2_DRIVES_DCLASS_SPEAKER)
#define DBMD2_DRIVES_DCLASS_SPEAKER		1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_AEC_REF_32_TO_16_BIT)
#if !defined(AEC_REF_32_TO_16_BIT)
#define AEC_REF_32_TO_16_BIT		1
#endif
#endif


#if defined(CONFIG_SND_SOC_DBMDX_I2S_BUFFERING_SUPPORTED)
#if !defined(DBMDX_I2S_BUFFERING_SUPPORTED)
#define DBMDX_I2S_BUFFERING_SUPPORTED		1
#endif
#endif


#if defined(CONFIG_SND_SOC_DBMDX_I2S_STREAMING_SUPPORTED)
#if !defined(DBMDX_I2S_STREAMING_SUPPORTED)
#define DBMDX_I2S_STREAMING_SUPPORTED		1
#endif
#if !defined(DBMDX_VC_I2S_STREAMING_SUPPORTED)
#define DBMDX_VC_I2S_STREAMING_SUPPORTED	1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_VC_I2S_STREAMING_SUPPORTED)
#if !defined(DBMDX_VC_I2S_STREAMING_SUPPORTED)
#define DBMDX_VC_I2S_STREAMING_SUPPORTED	1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_I2S_CATCHUP_SUPPORTED)
#if !defined(DBMDX_I2S_CATCHUP_SUPPORTED)
#define DBMDX_I2S_CATCHUP_SUPPORTED		1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_VT_WHILE_VC_SUPPORTED)
#if !defined(VT_WHILE_VC_SUPPORTED)
#define VT_WHILE_VC_SUPPORTED		1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_VC_RX_PROCESS_SUPPORTED)
#if !defined(VC_RX_PROCESS_SUPPORTED)
#define VC_RX_PROCESS_SUPPORTED		1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_VC_SRATE_16KHZ)
#if !defined(VC_SRATE_16KHZ)
#define VC_SRATE_16KHZ		1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_VA_VE_TDM_RX_MASTER)
/* #if !defined(VA_VE_TDM_RX_MASTER_SUPPORTED)
#define VA_VE_TDM_RX_MASTER		1
*/
#if !defined(VA_VE_I2S_MASTER_SUPPORTED)
#define VA_VE_I2S_MASTER		1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_QED_SUPPORTED)
#if !defined(DBMDX_QED_SUPPORTED)
#define DBMDX_QED_SUPPORTED		1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_OKG_WWE_SUPPORT)
#if !defined(DMBDX_OKG_AMODEL_SUPPORT)
#define DMBDX_OKG_AMODEL_SUPPORT	1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_OKG_2_CH_AEC_ONLY)
#if !defined(OKG_2_CH_AEC_ONLY_AUDIO_BUF)
#define OKG_2_CH_AEC_ONLY_AUDIO_BUF	1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_SENSORY_WWE_SUPPORT)
#if !defined(DMBDX_SENSORY_AMODEL_SUPPORT)
#define DMBDX_SENSORY_AMODEL_SUPPORT	1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_MELON_SRATE_48KHZ)
#if !defined(DBMDX_MELON_SRATE_48000)
#define DBMDX_MELON_SRATE_48000	1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_USE_EVBS_CONFIG)
#if !defined(USE_EVBS_CONFIG)
#define USE_EVBS_CONFIG	1
#endif
#endif

#if defined(CONFIG_SND_SOC_DBMDX_USE_ANALOG_MICS)
#if !defined(DBMDX_MIC_TYPE_IS_ANALOG)
#define DBMDX_MIC_TYPE_IS_ANALOG	1
#endif
#if defined(DBMDX_MIC_TYPE_IS_DIGITAL)
#undef DBMDX_MIC_TYPE_IS_DIGITAL
#endif
#else
#if !defined(DBMDX_MIC_TYPE_IS_DIGITAL)
#define DBMDX_MIC_TYPE_IS_DIGITAL	1
#endif
#if defined(DBMDX_MIC_TYPE_IS_ANALOG)
#undef DBMDX_MIC_TYPE_IS_ANALOG
#endif
#endif



#include <linux/dbmdx.h>
#include <sound/dbmdx-export.h>

#ifndef DBMD2_VA_FIRMWARE_NAME
#define DBMD2_VA_FIRMWARE_NAME			"dbmd2_va_fw.bin"
#endif

#ifndef DBMD2_VA_PREBOOT_FIRMWARE_NAME
#define DBMD2_VA_PREBOOT_FIRMWARE_NAME		"dbmd2_va_preboot_fw.bin"
#endif

#ifndef DBMD2_VQE_FIRMWARE_NAME
#define DBMD2_VQE_FIRMWARE_NAME			"dbmd2_vqe_fw.bin"
#endif

#ifndef DBMD2_VQE_OVERLAY_FIRMWARE_NAME
#define DBMD2_VQE_OVERLAY_FIRMWARE_NAME		"dbmd2_vqe_overlay_fw.bin"
#endif

#ifndef DBMD4_VA_FIRMWARE_NAME
#define DBMD4_VA_FIRMWARE_NAME			"dbmd4_va_fw.bin"
#endif

#ifndef DBMD4_VA_PREBOOT_FIRMWARE_NAME
#define DBMD4_VA_PREBOOT_FIRMWARE_NAME		"dbmd4_va_preboot_fw.bin"
#endif

#ifndef DBMD2_VA_VE_FIRMWARE_NAME
#define DBMD2_VA_VE_FIRMWARE_NAME		"dbmd2_va_ve_fw.bin"
#endif

#ifndef DBMDX_VT_GRAM_NAME
#define DBMDX_VT_GRAM_NAME			"voice_grammar.bin"
#endif

#ifndef DBMDX_VT_NET_NAME
#define DBMDX_VT_NET_NAME			"voice_net.bin"
#endif

#ifndef DBMDX_VT_AMODEL_NAME
#define DBMDX_VT_AMODEL_NAME			"voice_amodel.bin"
#endif

#ifndef DBMDX_VC_GRAM_NAME
#define DBMDX_VC_GRAM_NAME			"vc_grammar.bin"
#endif

#ifndef DBMDX_VC_NET_NAME
#define DBMDX_VC_NET_NAME			"vc_net.bin"
#endif

#ifndef DBMDX_VC_AMODEL_NAME
#define DBMDX_VC_AMODEL_NAME			"vc_amodel.bin"
#endif

#ifndef DBMDX_VC_OKG_NAME
#define DBMDX_VC_OKG_NAME			"okg_amodel.bin"
#endif

#ifndef DBMDX_VE_GRAM_NAME
#define DBMDX_VE_GRAM_NAME			"ve_grammar.bin"
#endif

#ifndef DBMDX_VE_NET_NAME
#define DBMDX_VE_NET_NAME			"ve_net.bin"
#endif

#ifndef DBMDX_VE_AMODEL_NAME
#define DBMDX_VE_AMODEL_NAME			"ve_amodel.bin"
#endif


#define MAX_REQ_SIZE				(8192*2)

#define DBMDX_AMODEL_HEADER_SIZE		12
#define DBMDX_AMODEL_MAX_CHUNKS			3

#define DBMDX_MSLEEP_WAKEUP			35
#define DBMDX_MSLEEP_HIBARNATE			20
#define DBMDX_MSLEEP_CONFIG_VA_MODE_REG		50
#define DBMDX_MSLEEP_NON_OVERLAY_BOOT		300
#define DBMDX_MSLEEP_REQUEST_FW_FAIL		200
#define DBMDX_MSLEEP_ON_SV_INTERRUPT		100
#define DBMDX_MSLEEP_PCM_STREAMING_WORK		100
#define DBMDX_MSLEEP_DBG_AFTER_DETECTION	50
#define DBMDX_MSLEEP_CONFIG_TDM_CLOCK		50
#define DBMDX_MSLEEP_CONFIG_HOST_CLOCK_32KHZ	50
#define DBMDX_MSLEEP_TDM_ACTIVATION		40
#define DBMDX_MSLEEP_AFTER_LOAD_ASRP		100
#define DBMDX_MSLEEP_BUFFERING_PAUSED		100
#define DBMDX_MSLEEP_AFTER_MIC_ENABLED		100
#define DBMDX_MSLEEP_IS_ALIVE			20
#define DBMDX_MSLEEP_IF_AUDIO_BUFFER_EMPTY	90


#define DBMDX_USLEEP_DBG_MODE_CMD_RX		5000
#define DBMDX_USLEEP_DBG_MODE_CMD_TX		5000
#define DBMDX_USLEEP_VQE_ALIVE			21000
#define DBMDX_USLEEP_VQE_ALIVE_ON_FAIL		10000
#define DBMDX_USLEEP_NO_SAMPLES			10000
#define DBMDX_USLEEP_SET_MODE			15000
#define DBMDX_USLEEP_AMODEL_HEADER		2000
#define DBMDX_USLEEP_AFTER_LOAD_AMODEL		10000
#define DBMDX_USLEEP_RESET_TOGGLE		10000
#define DBMDX_USLEEP_AUDIO_PROCESSING_CONFIG	5000
#define DBMDX_USLEEP_GAIN_NORM_SET		15000
#define DBMDX_USLEEP_BEFORE_INIT_CONFIG		20000
#define DBMDX_USLEEP_CONFIG_HOST_CLOCK		10000

#define DBMDX_MSLEEP_UART_PROBE			50
#define DBMDX_MSLEEP_UART_WAKEUP		50
#define DBMDX_MSLEEP_UART_WAIT_TILL_ALIVE	100
#define DBMDX_MSLEEP_UART_WAIT_FOR_CHECKSUM	50
#define DBMDX_MSLEEP_UART_D2_AFTER_LOAD_FW	50
#define DBMDX_MSLEEP_UART_D2_AFTER_RESET_32K	275
#define DBMDX_MSLEEP_UART_D4_AFTER_RESET_32K	80

#define DBMDX_USLEEP_UART_VQE_CMD_AFTER_SEND	20000
#define DBMDX_USLEEP_UART_VA_CMD_AFTER_SEND	500
#define DBMDX_USLEEP_UART_VA_CMD_AFTER_SEND_2	500
#define DBMDX_USLEEP_UART_AFTER_LOAD_AMODEL	10000
#define DBMDX_USLEEP_UART_AFTER_WAKEUP_BYTE	100
#define DBMDX_USLEEP_UART_D2_BEFORE_RESET	10000
#define DBMDX_USLEEP_UART_D2_AFTER_RESET	15000
#define DBMDX_USLEEP_UART_D4_AFTER_RESET	15000
#define DBMDX_USLEEP_UART_D4_AFTER_LOAD_FW	10000

#define DBMDX_MSLEEP_SPI_VQE_SYS_CFG_CMD	60
#define DBMDX_MSLEEP_SPI_FINISH_BOOT_1		10
#define DBMDX_MSLEEP_SPI_FINISH_BOOT_2		10
#define DBMDX_MSLEEP_SPI_WAKEUP			50
#define DBMDX_MSLEEP_SPI_D2_AFTER_RESET_32K	300
#define DBMDX_MSLEEP_SPI_D2_AFTER_SBL		20
#define DBMDX_MSLEEP_SPI_D2_BEFORE_FW_CHECKSUM	20
#define DBMDX_MSLEEP_SPI_D4_AFTER_RESET_32K	85
#define DBMDX_MSLEEP_SPI_D4_AFTER_PLL_CHANGE	80
#define DBMDX_MSLEEP_SPI_D2_AFTER_PLL_CHANGE	5

#define DBMDX_USLEEP_SPI_VQE_CMD_AFTER_SEND	20000
#define DBMDX_USLEEP_SPI_VA_CMD_AFTER_SEND	500
#define DBMDX_USLEEP_SPI_VA_CMD_AFTER_SEND_2	500
#define DBMDX_USLEEP_SPI_VA_CMD_AFTER_BOOT	1000
#define DBMDX_USLEEP_SPI_AFTER_CHUNK_READ	10000
#define DBMDX_USLEEP_SPI_AFTER_LOAD_AMODEL	10000
#define DBMDX_USLEEP_SPI_D2_AFTER_RESET		5000
#define DBMDX_USLEEP_SPI_D2_AFTER_SBL		10000
#define DBMDX_USLEEP_SPI_D2_AFTER_BOOT		10000
#define DBMDX_USLEEP_SPI_D4_AFTER_BOOT		10000
#define DBMDX_USLEEP_SPI_D4_AFTER_RESET		15000
#define DBMDX_USLEEP_SPI_D4_POST_PLL		2000

#define DBMDX_MSLEEP_I2C_FINISH_BOOT		275
#define DBMDX_MSLEEP_I2C_VQE_SYS_CFG_CMD	60
#define DBMDX_MSLEEP_I2C_WAKEUP			50
#define DBMDX_MSLEEP_I2C_D2_AFTER_RESET_32K	300
#define DBMDX_MSLEEP_I2C_D2_CALLBACK		20
#define DBMDX_MSLEEP_I2C_D2_AFTER_SBL		20
#define DBMDX_MSLEEP_I2C_D2_BEFORE_FW_CHECKSUM	20
#define DBMDX_MSLEEP_I2C_D4_AFTER_RESET_32K	100
#define DBMDX_MSLEEP_I2C_D4_AFTER_SBL		20
#define DBMDX_MSLEEP_I2C_D4_BEFORE_FW_CHECKSUM	20

#define DBMDX_USLEEP_I2C_VQE_CMD_AFTER_SEND	20000
#define DBMDX_USLEEP_I2C_VA_CMD_AFTER_SEND	10000
#define DBMDX_USLEEP_I2C_VA_CMD_AFTER_SEND_2	10000
#define DBMDX_USLEEP_I2C_AFTER_BOOT_CMD		10000
#define DBMDX_USLEEP_I2C_AFTER_LOAD_AMODEL	10000
#define DBMDX_USLEEP_I2C_D2_AFTER_RESET		15000
#define DBMDX_USLEEP_I2C_D2_AFTER_SBL		10000
#define DBMDX_USLEEP_I2C_D2_AFTER_BOOT		10000
#define DBMDX_USLEEP_I2C_D4_AFTER_RESET		15000
#define DBMDX_USLEEP_I2C_D4_AFTER_BOOT		10000

#define DBMDX_DEFAULT_HW_REV			0x0000

#define DBMDX_DEBUG_MODE_OFF			0x0000
#define DBMDX_DEBUG_MODE_RECORD			0x0001
#define DBMDX_DEBUG_MODE_FW_LOG			0x0002

#define DBMDX_BOOT_MODE_NORMAL_BOOT		0x0000
#define DBMDX_BOOT_MODE_RESET_DISABLED		0x0001

#define DBMDX_BOOT_OPT_SEND_PREBOOT		0x0001
#define DBMDX_BOOT_OPT_VA_NO_UART_SYNC		0x0002
#define DBMDX_BOOT_OPT_NO_I2C_FREQ_CALLBACK	0x0004
#define DBMDX_BOOT_OPT_DONT_SENT_SBL		0x0008
#define DBMDX_BOOT_OPT_DONT_SET_PLL		0x0020
#define DBMDX_BOOT_OPT_DONT_CLR_CRC		0x0040
#define DBMDX_BOOT_OPT_DONT_VERIFY_CRC		0x0080
#define DBMDX_BOOT_OPT_DONT_SEND_START_BOOT	0x0100
#define DBMDX_BOOT_OPT_VERIFY_CHIP_ID		0x0200

#define DBMDX_BOOT_OPT_INTERFACE_DISABLED	0xFFFF

#define DBMDX_AMODEL_DEFAULT_OPTIONS		0x0000
#define DBMDX_AMODEL_INCLUDES_HEADERS		0x0001
#define DBMDX_AMODEL_SVT_ENCODING		0x0002
#define DBMDX_AMODEL_SINGLE_FILE_NO_HEADER	0x0004
#define DBMDX_LOAD_AMODEL_FOR_VE		0x0008
#define DBMDX_VE_SEND_DUMMY_AMODEL_4B		0x0010

#define DBMDX_AMODEL_TYPE_PRIMARY		0x0001
#define DBMDX_AMODEL_TYPE_SECONDARY		0x0002

#define DBMDX_NO_MODEL_SELECTED			0x0000
#define DBMDX_SV_MODEL_SELECTED			0x0001
#define DBMDX_OKG_MODEL_SELECTED		0x0002

#define DBMDX_LOAD_MODEL_NO_DETECTION		0x0001
#define DBMDX_DO_NOT_RELOAD_MODEL		0x0002
#define DBMDX_LOAD_MODEL_FROM_MEMORY		0x0004

#define DBMDX_NO_EXT_DETECTION_MODE_PARAMS	0x0000

#define AMODEL_DETECTION_MODE_MASK		0x000f
#define AMODEL_DETECTION_MODE_BITS		0
#define AMODEL_MODEL_SELECT_MASK		0x0030
#define AMODEL_MODEL_SELECT_BITS		4
#define AMODEL_MODEL_OPTIONS_MASK		0x0f00
#define AMODEL_MODEL_OPTIONS_BITS		8
#define AMODEL_MODEL_CUSTOM_PARAMS_MASK		0xf000
#define AMODEL_MODEL_CUSTOM_PARAMS_BITS		12



#define DBMDX_CHANGE_TO_HOST_CLOCK_D4_ENABLED		0x0001
#define DBMDX_CHANGE_TO_INTERNAL_CLOCK_D4_ENABLED	0x0002
#define DBMDX_CHANGE_TO_HOST_CLOCK_D2_ENABLED		0x0004
#define DBMDX_CHANGE_TO_INTERNAL_CLOCK_D2_ENABLED	0x0008

#define DBMDX_CHANGE_CLOCK_MASK				0x000F

#define DBMDX_CHANGE_CLOCK_DISABLED			0x0000
#define DBMDX_CHANGE_CLOCK_D4_ENABLED	\
				(DBMDX_CHANGE_TO_HOST_CLOCK_D4_ENABLED | \
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D4_ENABLED)

#define DBMDX_CHANGE_CLOCK_D2_ENABLED	\
				(DBMDX_CHANGE_TO_HOST_CLOCK_D2_ENABLED | \
				DBMDX_CHANGE_TO_INTERNAL_CLOCK_D2_ENABLED)

#define DBMDX_CHANGE_CLOCK_ENABLED	\
				(DBMDX_CHANGE_CLOCK_D4_ENABLED | \
				DBMDX_CHANGE_CLOCK_D2_ENABLED)

#ifndef DEFAULT_D2_CLOCK_HZ
#define DEFAULT_D2_CLOCK_HZ	294912000
#endif

#ifndef DEFAULT_D4_CLOCK_HZ
#define DEFAULT_D4_CLOCK_HZ	92160000
#endif

#define DBMDX_DEFAULT_USECASE_MODE		0xFF
#define DBMDX_BUFFERING_WITH_BACKLOG_MASK	0x10
#define DBMDX_USECASE_MODE_MASK			0x0F

#define DBMDX_ALSA_STREAMING_DEFAULT_OPTIONS		0x0000
#define DBMDX_ALSA_STREAMING_DISABLED			0x0001
#define DBMDX_ALSA_STREAMING_BUFFERING_WITH_HISTORY	0x0002
#define DBMDX_ALSA_STREAMING_IDLE_AFTER_BUFFERING	0x0004
#define DBMDX_ALSA_STREAMING_SET_DEFAULT_USECASE	0x0008

#define DBMDX_ASRP_PARAMS_OPTIONS_DEFAULT		0x0000
#define DBMDX_ASRP_PARAMS_OPTIONS_ALWAYS_RELOAD		0x0001

#define DBMDX_TDM_VA_BASE	0x1
#define DBMDX_TDM_0_VA		(DBMDX_TDM_VA_BASE)
#define DBMDX_TDM_1_VA		(DBMDX_TDM_VA_BASE << 1)
#define DBMDX_TDM_2_VA		(DBMDX_TDM_VA_BASE << 2)
#define DBMDX_TDM_3_VA		(DBMDX_TDM_VA_BASE << 3)
#define DBMDX_TDM_VA_VE_BASE	(DBMDX_TDM_3_VA << 1)
#define DBMDX_TDM_0_VA_VE	(DBMDX_TDM_VA_VE_BASE << 0)
#define DBMDX_TDM_1_VA_VE	(DBMDX_TDM_VA_VE_BASE << 1)
#define DBMDX_TDM_2_VA_VE	(DBMDX_TDM_VA_VE_BASE << 2)
#define DBMDX_TDM_3_VA_VE	(DBMDX_TDM_VA_VE_BASE << 3)

struct chip_interface;

enum dbmdx_firmware_active {
	/* firmware pre-boot */
	DBMDX_FW_PRE_BOOT = 0,
	/* va firmware was powered off */
	DBMDX_FW_POWER_OFF_VA,
	/* voice authentication */
	DBMDX_FW_VA,
	/* voice quality enhancement */
#ifdef DBMDX_VA_VE_SUPPORT
	/* voice authentication */
	DBMDX_FW_VA_VE,
#endif
	DBMDX_FW_VQE,
	/* max supported firmwares */
	DBMDX_FW_MAX
};


enum dbmdx_audio_channel_operation {
	AUDIO_CHANNEL_OP_COPY = 0,
	AUDIO_CHANNEL_OP_DUPLICATE_1_TO_2,
	AUDIO_CHANNEL_OP_DUPLICATE_1_TO_4,
	AUDIO_CHANNEL_OP_DUPLICATE_2_TO_4,
	AUDIO_CHANNEL_OP_TRUNCATE_2_TO_1,
	AUDIO_CHANNEL_OP_TRUNCATE_4_TO_1,
	AUDIO_CHANNEL_OP_TRUNCATE_4_TO_2,
	AUDIO_CHANNEL_OP_MAX = AUDIO_CHANNEL_OP_TRUNCATE_4_TO_2
};

enum dbmdx_chip {
	DBMDX_CHIP_VA = 0,
#ifdef DBMDX_VA_VE_SUPPORT
	DBMDX_CHIP_VA_VE,
#endif
	DBMDX_NR_OF_CHIP_TYPES,
};

enum dbmdx_sv_recognition_mode {
	SV_RECOGNITION_MODE_DISABLED = 0,
	SV_RECOGNITION_MODE_VOICE_PHRASE_OR_CMD = 1,
	SV_RECOGNITION_MODE_VOICE_ENERGY = 2,
};

#ifdef DMBDX_OKG_AMODEL_SUPPORT
enum dbmdx_okg_recognition_mode {
	OKG_RECOGNITION_MODE_DISABLED = 0,
	OKG_RECOGNITION_MODE_ENABLED = 1,
};
#endif

enum dbmdx_detection_after_buffering_mode {
	DETECTION_AFTER_BUFFERING_OFF = 0,
	DETECTION_AFTER_BUFFERING_MODE_ENABLE_ALL = 1,
	DETECTION_AFTER_BUFFERING_MODE_SET_MODE_ONLY = 2,
	DETECTION_AFTER_BUFFERING_MODE_MAX,
};


struct va_flags {
	int	irq_inuse;
	int	a_model_downloaded_to_fw;
	int	amodel_len;
	enum dbmdx_sv_recognition_mode sv_recognition_mode;
#ifdef DMBDX_OKG_AMODEL_SUPPORT
	int	okg_a_model_downloaded_to_fw;
	int	okg_amodel_len;
	bool	okg_a_model_enabled;
	enum	dbmdx_okg_recognition_mode okg_recognition_mode;
#endif
	int	buffering;
	int	buffering_paused;
	int	qed_active;
	int	pcm_worker_active;
	int	pcm_streaming_active;
	bool	pcm_streaming_pushing_zeroes;
	int	sleep_not_allowed;
	int	auto_detection_disabled;
	int	padded_cmds_disabled;
	bool	sv_capture_on_detect_disabled;
	bool	reconfigure_mic_on_vad_change;
	bool	disabling_mics_not_allowed;
	bool	microphones_enabled;
	int	cancel_pm_work;
	unsigned int	mode;
};

#ifdef DBMDX_VA_VE_SUPPORT
struct va_ve_flags {
	unsigned int	active_usecase_id;
	enum dbmdx_chip	drv_seletected_chip;
	unsigned int	va_ve_chip_mode;
	struct usecase_config *cur_usecase;
	const char	*va_last_loaded_asrp_params_file_name;
	const char	*va_ve_last_loaded_asrp_params_file_name;
	unsigned int	idle_usecase_active;
	bool		tdm_is_reset;
	int	sleep_not_allowed;
	unsigned int	usecase_mode;
	unsigned int	mode;
};

enum dbmdx_va_ve_project_sub_types {
	DBMDX_VA_VE_PROJECT_SUB_TYPE_VT_ASRP = 0,
	DBMDX_VA_VE_PROJECT_SUB_TYPE_VT_ONLY = 1,
	DBMDX_VA_VE_PROJECT_SUB_TYPE_ASRP_ONLY = 2,
};
#endif

struct vqe_flags {
	int	in_call;
	int	use_case;
	int	speaker_volume_level;
};

struct vqe_fw_info {
	u16	major;
	u16	minor;
	u16	version;
	u16	patch;
	u16	debug;
	u16	tuning;
};

struct amodel_info {
	char	*amodel_buf;
	ssize_t amodel_size;
	ssize_t	amodel_chunks_size[DBMDX_AMODEL_MAX_CHUNKS];
	int	num_of_amodel_chunks;
	bool	amodel_loaded;
};

enum dbmd2_xtal_id {
	DBMD2_XTAL_FREQ_24M_IMG1 = 0,
	DBMD2_XTAL_FREQ_24M_IMG2,
	DBMD2_XTAL_FREQ_24M_IMG3,
	DBMD2_XTAL_FREQ_9M_IMG4,
	DBMD2_XTAL_FREQ_24M_IMG5,
	DBMD2_XTAL_FREQ_19M_IMG6,
	DBMD2_XTAL_FREQ_32K_IMG7,
	DBMD2_XTAL_FREQ_32K_IMG8,
};

enum dbmdx_load_amodel_mode {
	LOAD_AMODEL_PRIMARY = 0,
	LOAD_AMODEL_2NDARY = 1,
	LOAD_AMODEL_CUSTOM,
#ifdef DMBDX_OKG_AMODEL_SUPPORT
	LOAD_AMODEL_OKG = 4,
	LOAD_AMODEL_MAX = LOAD_AMODEL_OKG
#else
	LOAD_AMODEL_MAX = LOAD_AMODEL_CUSTOM
#endif
};

enum dbmdx_states {
	DBMDX_IDLE = 0,
	DBMDX_DETECTION = 1,
#ifdef DBMDX_FW_BELOW_300
	DBMDX_RESERVED = 2,
	DBMDX_BUFFERING = 3,
#else
	DBMDX_BUFFERING = 2,
	DBMDX_UART_RECORDING = 3,
#endif
	DBMDX_SLEEP_PLL_ON = 4,
	DBMDX_SLEEP_PLL_OFF = 5,
	DBMDX_HIBERNATE = 6,
	DBMDX_STREAMING = 7,	/* CUSTOM STATE */
	DBMDX_DETECTION_AND_STREAMING = 8,	/* CUSTOM STATE */
	DBMDX_NR_OF_STATES,
};

enum dbmdx_bus_interface {
	DBMDX_INTERFACE_NONE = 0,
	DBMDX_INTERFACE_I2C,
	DBMDX_INTERFACE_SPI,
	DBMDX_INTERFACE_UART,
	DBMDX_NR_OF_INTERFACES,
};

enum dbmdx_power_modes {
	/*
	 * no firmware is active and the device is booting
	 */
	DBMDX_PM_BOOTING = 0,
	/*
	 * a firmware is active and the device can be used
	 */
	DBMDX_PM_ACTIVE,
	/*
	 * a firmware is active and the device is going to sleep
	 */
	DBMDX_PM_FALLING_ASLEEP,
	/*
	 * chip is sleeping and needs to be woken up to be functional
	 */
	DBMDX_PM_SLEEPING,
	/* number of PM states */
	DBMDX_PM_STATES,
};
#ifdef DBMDX_VA_VE_SUPPORT
#define MAX_USECASE_NAME_LEN	64
#endif

struct dbmdx_private {
	struct dbmdx_platform_data		*pdata;
	/* lock for private data */
	struct mutex			p_lock;
	enum dbmdx_firmware_active	active_fw;
	enum dbmdx_firmware_active	active_fw_va_chip;
	enum dbmdx_power_modes		power_mode;
	enum dbmdx_bus_interface	active_interface;
	enum dbmdx_interface_type	active_interface_type_va;
#ifdef DBMDX_VA_VE_SUPPORT
	enum dbmdx_firmware_active	active_fw_va_ve_chip;
	enum dbmdx_interface_type	active_interface_type_va_ve;
#endif
	enum dbmdx_chip			active_chip;
	enum dbmdx_chip			usr_selected_chip;
	int				sv_irq;
	struct platform_device		*pdev;
	struct device			*dev;

	bool				va_chip_enabled;
	const struct firmware		*va_fw;
	const struct firmware		*va_preboot_fw;

#ifdef DBMDX_VA_VE_SUPPORT
	bool				va_ve_chip_enabled;
	const struct firmware		*va_ve_fw;
	const struct firmware		*va_ve_preboot_fw;
#endif
	unsigned int			tdm_enable;

	const struct firmware		*vqe_fw;
	const struct firmware		*vqe_non_overlay_fw;

	struct firmware			*dspg_gram;
	struct firmware			*dspg_net;
	bool				asleep;
	bool				device_ready;
#ifdef DBMDX_VA_VE_SUPPORT
	bool				asleep_va;
	bool				asleep_va_ve;
	bool				device_ready_va_ve;
	u32				va_ve_mic_mode;
	bool				start_usecase_disabled;
	bool				external_usecase_loaded;
	char				usecase_info[MAX_USECASE_NAME_LEN+1];
	bool				reference_playback_active;
	bool				mics_connected_to_va_ve_chip;
	bool				asrp_runs_on_vt;
	int				qed_enabled;
	int				qed_expiration_time_ms;
	int				qed_no_signal_ms;
	int				qed_min_query_duration_ms;
	u32				va_load_asrp_params_options;
	u32				va_ve_load_asrp_params_options;
#endif
	struct clk			*clocks[DBMDX_NR_OF_CLKS];
	struct regulator		*vregulator;
	struct work_struct		sv_work;
	struct work_struct		pcm_streaming_work;
	struct work_struct		uevent_work;
	unsigned int			audio_mode;
	unsigned int			clk_type;
	unsigned int			master_pll_rate;
	unsigned int			bytes_per_sample;
	unsigned int			current_pcm_rate;
	unsigned int			audio_pcm_rate;
	unsigned int			audio_pcm_channels;
	enum dbmdx_audio_channel_operation	pcm_achannel_op;
	enum dbmdx_audio_channel_operation	detection_achannel_op;
	unsigned int			fw_vad_type;
	dev_t				record_chrdev;
	struct cdev			record_cdev;
	struct device			*record_dev;
	struct cdev			record_cdev_block;
	struct device			*record_dev_block;
	struct kfifo			pcm_kfifo;
	struct kfifo			detection_samples_kfifo;
	void				*detection_samples_kfifo_buf;
	unsigned int			detection_samples_kfifo_buf_size;
	atomic_t			audio_owner;
	struct amodel_info		primary_amodel;
	struct amodel_info		secondary_amodel;
#ifdef DMBDX_OKG_AMODEL_SUPPORT
	struct amodel_info		okg_amodel;
#endif
	u8				*sbl_data;
	struct va_flags			va_flags;
#ifdef DBMDX_VA_VE_SUPPORT
	struct va_ve_flags		va_ve_flags;
#endif
	struct vqe_flags		vqe_flags;
	u32				vqe_vc_syscfg;
	u32				va_current_mic_config;
	u32				va_active_mic_config;
	u32				va_detection_mode;
	u16				va_detection_mode_custom_params;
	u32				va_cur_backlog_length;
	u32				va_last_word_id;
	bool				va_capture_on_detect;
#ifdef DMBDX_OKG_AMODEL_SUPPORT
	bool				okg_a_model_support;
#endif
	bool				sv_a_model_support;
	bool				sleep_disabled;
	bool				mic_disabling_blocked;
	int				va_debug_mode;

	int				va_cur_digital_mic_digital_gain;
	int				va_cur_analog_mic_analog_gain;
	int				va_cur_analog_mic_digital_gain;

	int				cur_reset_gpio;
	int				cur_wakeup_gpio;
	int				cur_wakeup_disabled;
	int				cur_wakeup_set_value;
	int				cur_send_wakeup_seq;
	int				cur_use_gpio_for_wakeup;
	int				cur_firmware_id;
	u32				cur_boot_options;

	u32				boot_mode;
	u32				debug_delay_usec;


	unsigned int			num_dais;
	struct snd_soc_dai_driver	*dais;
	int				remote_codec_in_use;

	u8				read_audio_buf[MAX_REQ_SIZE + 8];
	struct snd_pcm_substream	*active_substream;

	struct delayed_work		delayed_pm_work;
	struct workqueue_struct		*dbmdx_workq;

	/* limit request size of audio data from the firmware */
	unsigned long				rxsize;

	/* sysfs */
	struct class				*ns_class;
	struct device				*dbmdx_dev;
	/* common helper functions */
	void (*reset_set)(struct dbmdx_private *p);
	void (*reset_release)(struct dbmdx_private *p);
	void (*reset_sequence)(struct dbmdx_private *p);
	void (*wakeup_set)(struct dbmdx_private *p);
	void (*wakeup_release)(struct dbmdx_private *p);
	void (*wakeup_toggle)(struct dbmdx_private *p);
	void (*lock)(struct dbmdx_private *p);
	void (*unlock)(struct dbmdx_private *p);
	int (*verify_checksum)(struct dbmdx_private *p,
			       const u8 *expect, const u8 *got, size_t size);
	int (*va_set_speed)(struct dbmdx_private *p,
			    enum dbmdx_va_speeds speed);
	unsigned long (*clk_get_rate)(struct dbmdx_private *p,
				      enum dbmdx_clocks clk);
	long (*clk_set_rate)(struct dbmdx_private *p,
			     enum dbmdx_clocks clk);
	int (*clk_enable)(struct dbmdx_private *p, enum dbmdx_clocks clk);
	int (*clk_disable)(struct dbmdx_private *p, enum dbmdx_clocks clk);

	/* external callbacks */
	set_i2c_freq_cb	set_i2c_freq_callback;
	event_cb		event_callback;

	/* interface to the chip */
	struct chip_interface			*chip;
	struct chip_interface		**interfaces;
	enum dbmdx_bus_interface	*interface_types;
	unsigned int			nr_of_interfaces;

};

/*
 * main interface between the core layer and the chip
 */
struct chip_interface {
	/* wait till booting is allowed */
	int (*can_boot)(struct dbmdx_private *p);
	/* prepare booting (e.g. increase speed) */
	int (*prepare_boot)(struct dbmdx_private *p);
	/* send firmware to the chip and boot it */
	int (*boot)(const void *fw_data, size_t fw_size,
		struct dbmdx_private *p, const void *checksum,
		size_t chksum_len, int load_fw);
	/* finish booting */
	int (*finish_boot)(struct dbmdx_private *p);
	/* dump chip state */
	int (*dump)(struct chip_interface *chip, char *buf);
	/* set VA firmware ready, (e.g. lower speed) */
	int (*set_va_firmware_ready)(struct dbmdx_private *p);
	/* set VQE firmware ready */
	int (*set_vqe_firmware_ready)(struct dbmdx_private *p);
	/* Enable/Disable Transport layer (UART/I2C/SPI) */
	void (*transport_enable)(struct dbmdx_private *p, bool enable);
	/* read data from the chip */
	ssize_t (*read)(struct dbmdx_private *p, void *buf, size_t len);
	/* write data to the chip */
	ssize_t (*write)(struct dbmdx_private *p, const void *buf, size_t len);
	/* send command in VQE protocol format to the chip */
	ssize_t (*send_cmd_vqe)(struct dbmdx_private *p,
			    u32 command, u16 *response);
	/* send command in VA protocol format to the chip */
	ssize_t (*send_cmd_va)(struct dbmdx_private *p,
				  u32 command, u16 *response);
	/* send command in VA protocol 32 bit format to the chip */
	ssize_t (*send_cmd_va_32)(struct dbmdx_private *p,
				  u32 command, u32 value, u32 *response);
	/* send command in boot protocol format to the chip */
	int (*send_cmd_boot)(struct dbmdx_private *p, u32 command);
	/* verify boot checksum */
	int (*verify_boot_checksum)(struct dbmdx_private *p,
	const void *checksum, size_t chksum_len);
	/* prepare buffering of audio data (e.g. increase speed) */
	int (*prepare_buffering)(struct dbmdx_private *p);
	/* read audio data */
	int (*read_audio_data)(struct dbmdx_private *p,
		void *buf,
		size_t samples,
		bool to_read_metadata,
		size_t *available_samples,
		size_t *data_offset);
	/* finish buffering of audio data (e.g. lower speed) */
	int (*finish_buffering)(struct dbmdx_private *p);
	/* prepare amodel loading (e.g. increase speed) */
	int (*prepare_amodel_loading)(struct dbmdx_private *p);
	/* load acoustic model */
	int (*load_amodel)(struct dbmdx_private *p,  const void *data,
			   size_t size, int num_of_chunks, size_t *chunk_sizes,
			   const void *checksum, size_t chksum_len,
			   u16 load_amodel_mode_cmd);
	/* finish amodel loading (e.g. lower speed) */
	int (*finish_amodel_loading)(struct dbmdx_private *p);
	/* Get Read Chunk Size */
	u32 (*get_read_chunk_size)(struct dbmdx_private *p);
	/* Get Write Chunk Size */
	u32 (*get_write_chunk_size)(struct dbmdx_private *p);
	/* Set Read Chunk Size */
	int (*set_read_chunk_size)(struct dbmdx_private *p, u32 size);
	/* Set Write Chunk Size */
	int (*set_write_chunk_size)(struct dbmdx_private *p, u32 size);
	/* Resume Chip Interface */
	void (*resume)(struct dbmdx_private *p);
	/* Suspend Chip Interface */
	void (*suspend)(struct dbmdx_private *p);

	/* private data */
	void *pdata;
};

/*
 * character device
 */
int dbmdx_register_cdev(struct dbmdx_private *p);
void dbmdx_deregister_cdev(struct dbmdx_private *p);
void stream_set_position(struct snd_pcm_substream *substream,
				u32 position);
u32 stream_get_position(struct snd_pcm_substream *substream);
int dbmdx_set_pcm_timer_mode(struct snd_pcm_substream *substream,
				bool enable_timer);
#ifndef CONFIG_SND_SOC_DBMDX
extern int (*dbmdx_init_interface)(void);
extern void (*dbmdx_deinit_interface)(void);

int board_dbmdx_snd_init(void);
void board_dbmdx_snd_exit(void);

int snd_dbmdx_pcm_init(void);
void snd_dbmdx_pcm_exit(void);
#endif
#endif
int dbmdx_snd_init(struct dbmdx_private *p);
void dbmdx_snd_deinit(struct dbmdx_private *p);
int dbmdx_dev_remove(struct snd_soc_codec *codec);
int dbmdx_dev_probe(struct snd_soc_codec *codec);
int dbmdx_set_usecase_mode(struct dbmdx_private *p, int mode);
int dbmdx_set_power_mode(
		struct dbmdx_private *p, enum dbmdx_power_modes mode);
int dbmdx_set_va_usecase_name(struct dbmdx_private *p, const char *buf);
int dbmdx_io_register_write(struct dbmdx_private *p,
				u32 addr, u32 value);
int dbmdx_io_register_read(struct dbmdx_private *p,
				u32 addr, u32 *presult);
int dbmdx_switch_to_va_chip_interface(struct dbmdx_private *p,
		enum dbmdx_interface_type interface_type);
#ifdef DBMDX_VA_VE_SUPPORT
int dbmdx_switch_to_va_ve_chip_interface(struct dbmdx_private *p,
	enum dbmdx_interface_type interface_type);
int dbmdx_va_get_usecase_by_id(struct dbmdx_private *p, int usecase_id,
				struct usecase_config **usecase);
int dbmdx_va_usecase_update(struct dbmdx_private *p,
		struct usecase_config *usecase);
int dbmdx_va_set_usecase_id(struct dbmdx_private *p, int usecase_id);
#endif
