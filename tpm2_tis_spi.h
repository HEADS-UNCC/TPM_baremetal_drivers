//SPDX-License-Identifier: GPL-2.0
/*
 * tpm2_tis_spi.h
 *
 *  Created on: Aug 10, 2018
 *      Author: Ali Shuja Siddiqui <asiddiq6@uncc.edu>
 *      Correspondence: Fareena Saqib <fsaqib@uncc.edu>
 *  Description:
 *  This code is a device driver for TPM 2.0 SPI devices at the
 *  baremetal layer. This code is written for Xilinx FPGA SoCs
 *  but can be easily ported to any embedded architecture.
 *
 *  This code calls the TPM 2.0 API functions as described by
 *  tpm2_* array constants.
 *
 *  Refer to the TPM 2.0 specifications for more information
 *  on implementing TPM structures. 
 *
 *  This code is a derivative of U-boot's tpm2_is_spi 
 *  driver found here: https://gitlab.denx.de/u-boot/u-boot/-/blob/master/drivers/tpm/tpm2_tis_spi.c
 *  
 */


#ifndef SRC_TPM2_TIS_SPI_H_
#define SRC_TPM2_TIS_SPI_H_

#include <xil_types.h>

#define __MSB(x) ((x) >> 8)
#define __LSB(x) ((x) & 0xFF)
#define tpm_u16(x) __MSB(x), __LSB(x)
#define tpm_u32(x) tpm_u16((x) >> 16), tpm_u16((x) & 0xFFFF)


#define TPM_ACCESS(l)					(0x0000 | ((l) << 12))
#define TPM_INT_ENABLE(l)				(0x0008 | ((l) << 12))
#define TPM_STS(l)						(0x0018 | ((l) << 12))
#define TPM_DATA_FIFO(l)				(0x0024 | ((l) << 12))
#define TPM_DID_VID(l)					(0x0F00 | ((1) << 12))
#define TPM_RID(l)						(0x0F04 | ((l) << 12))

#define TPM_HASH_START				    0x4028
#define TPM_HASH_DATA					0x4024
#define TPM_HASH_END					0x4020


#define TPM_WAIT_STATES					100
#define TPM_DEV_BUFSIZE					1260

//TPM ACCESS
#define TPM_ACCESS_ACTIVE_LOCALITY		0x20
#define TPM_ACCESS_VALID				0x80
#define TPM_ACCESS_REQUEST_PENDING		0x04
#define TPM_ACCESS_REQUEST_USE			0x02

//TPM STATUS
#define TPM_STS_VALID					0x80
#define TPM_STS_COMMAND_READY			0x40
#define TPM_STS_GO						0x20
#define TPM_STS_DATA_AVAIL				0x10
#define TPM_STS_DATA_EXPECT				0x08

//TPM TIMEOUT
#define TPM_TIMEOUT_MS					5
#define TIS_SHORT_TIMEOUT_MS			750
#define TIS_LONG_TIMEOUT_MS				2000
#define SLEEP_DURATION_US				60
#define SLEEP_DURATION_LONG_US			210

#define TPM_HEADER_SIZE					10

#define TPM2_DIGEST_LEN					32
#define TPM2_LOC4_DIGEST_LEN			64


u32 spi_init();
int tpm_xfer(u32 addr, const u8 * out, u8 * in, u16 len);
int tpm_read(u16 addr, u8 *in, u16 len);
int tpm_read32(u32 addr, u32 * result);
int tpm_write(u16 addr, const u8 * out, u16 len);
int tpm_check_loc(int loc);
void tpm_release_loc(int loc, u8 force);
int tpm_request_loc(int loc);
int tpm_status(int loc, u8 * status);
int tpm_get_burstcount(int loc, u32 * burstcount);
int tpm_cancel(int loc);
int tpm_wait_for_stat(int loc, u8 mask, unsigned long timeout, u8 * status);
int tpm_recv_data (int loc, u8 * buf, size_t count);
int tpm_send(int loc, const u8 * buf, size_t len);
int tpm_wait_init(int loc);
int tpm_pcr_read(int loc, int pcr_index, u8 * pcr_value_buf);
int tpm_pcr_reset(int loc, u8 pcr_index);
int tpm_read_capability_fixed(int loc);
int tpm_hash_start();
int tpm_hash_data(u8 * buf, size_t len);
int tpm_hash_end();


static const uint8_t sha1_alg[] = {
	0x00, 0x04
};
static const uint8_t sha256_alg[] = {
	0x00, 0x0B			// command for sha256 alg
};


static const uint8_t tpm2_startup_clear_command[] = {
	0x80, 0x01,			// TPM_ST_NO_SESSIONS
	0x00, 0x00, 0x00, 0x0C,		// commandSize
	0x00, 0x00, 0x01, 0x44,		// TPM_CC_Startup
	0x00, 0x00			// TPM_ST_CLEAR
};

static const uint8_t tpm2_pcr_read_command[] = {
	0x80, 0x01,                     // TPM_ST_NO_SESSIONS
	0x00, 0x00, 0x00, 0x14,         // commandSize
	0x00, 0x00, 0x01, 0x7E,         // TPM_CC_PCR_Read
	0x00, 0x00, 0x00, 0x01,         // count (TPML_PCR_SELECTION)
	0x00, 0x00,                     // hash (TPMS_PCR_SELECTION; will be set later)
	0x03,                           // sizeofSelect (TPMS_PCR_SELECTION)
	0x00, 0x00, 0x00                // pcrSelect (TPMS_PCR_SELECTION)

};

static const uint8_t tpm2_pcr_reset_command[] = {
	0x80, 0x02,			// TPM_ST_SESSIONS
	0x00, 0x00, 0x00, 0x1B,		// commandSize
	0x00, 0x00, 0x01, 0x3D,		// TPM_CC_PCR_Reset
	0x00, 0x00, 0x00, 0x00,		// {PCR_FIRST:PCR_LAST} (TPMI_DH_PCR)
	0x00, 0x00,			// authSize (NULL Password)
					// null (indicate a NULL Password)
	0x00, 0x09,			// authSize (password authorization session)
	0x40, 0x00, 0x00, 0x09,		// TPM_RS_PW (indicate a password authorization session)
	0x00, 0x00, 0x01, 0x00, 0x00
};

static const uint8_t tpm2_getcapability_fixed[] ={
	0x80, 0x01,			// TPM_ST_NO_SESSIONS
	0x00, 0x00, 0x00, 0x16,		// commandSize
	0x00, 0x00, 0x01, 0x7A,		// TPM_CC_GetCapability
	0x00, 0x00, 0x00, 0x06,		// TPM_CAP_TPM_PROPERTIES (Property Type: TPM_PT)
	0x00, 0x00, 0x01, 0x00,		// Property: TPM_PT_FAMILY_INDICATOR: PT_GROUP * 1 + 0
	0x00, 0x00, 0x00, 0x2D		// PropertyCount 2D (from 100 - 201)
};

static const uint8_t tpm2_pcr_extend[] = {
	0x80, 0x02,			// TPM_ST_SESSIONS
	0x00, 0x00, 0x00, 0x00,		// commandSize (will be set later)
	0x00, 0x00, 0x01, 0x82,		// TPM_CC_PCR_Extend
	0x00, 0x00, 0x00, 0x00,		// {PCR_FIRST:PCR_LAST} (TPMI_DH_PCR)
	0x00, 0x00,			// authSize (NULL Password)
					// null (indicate a NULL Password)
	0x00, 0x09,			// authSize (password authorization session)
	0x40, 0x00, 0x00, 0x09,		// TPM_RS_PW (indicate a password authorization session)
	0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01,		// count (TPML_DIGEST_VALUES)
	0x00, 0x00			// hashAlg (TPMT_HA; will be set later)
					// digest (TPMT_HA; will be added later)
};

#endif /* SRC_TPM2_TIS_SPI_H_ */
