//SPDX-License-Identifier: GPL-2.0
/*
 * tpm2_tis_spi.c
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

#include "tpm2_tis_spi.h"
#include "xparameters.h"	/* EDK generated parameters */
#include "xspips.h"		/* SPI device driver */
#include "fsbl_debug.h"
#include "xtime_l.h"
#include "byteshift.h"
#include <stdarg.h>

#define SPI_DEVICE_ID		XPAR_XSPIPS_0_DEVICE_ID
#define MAX_BUFFER_SIZE		256
#define TPM_WAIT_STATES		100


static XSpiPs SpiInstance;

u32 spi_init(){
	int Status;

	XSpiPs_Config *SpiConfig;
	u16 SpiDeviceId = SPI_DEVICE_ID;

	XSpiPs * SpiInstancePtr = &SpiInstance;
	/*
	 * Initialize the SPI driver so that it's ready to use
	 */
	SpiConfig = XSpiPs_LookupConfig(SpiDeviceId);
	if (NULL == SpiConfig) {
		return XST_FAILURE;
	}

	Status = XSpiPs_CfgInitialize(SpiInstancePtr, SpiConfig,
				       SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build
	 */
	Status = XSpiPs_SelfTest(SpiInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set the Spi device as a master. External loopback is required.
	 */
	XSpiPs_SetOptions(SpiInstancePtr, XSPIPS_MASTER_OPTION |
			   XSPIPS_FORCE_SSELECT_OPTION);

	XSpiPs_SetClkPrescaler(SpiInstancePtr, XSPIPS_CLK_PRESCALE_256 );

	XSpiPs_SetSlaveSelect(SpiInstancePtr, 0x0);

	return XST_SUCCESS;

}

int tpm_xfer(uint32_t addr, const uint8_t * out, uint8_t * in, uint16_t len){
	u8 tx_buf[MAX_BUFFER_SIZE];
	u8 rx_buf[MAX_BUFFER_SIZE];
	int transfer_len, ret;

	if (in && out){
		fsbl_printf(DEBUG_GENERAL, "tpm_xfer: in and out cannot both be not NULL.\r\n");
		return XST_FAILURE;
	}

	while(len){
		transfer_len = (len<MAX_BUFFER_SIZE) ? len: MAX_BUFFER_SIZE;
		tx_buf[0] = (in ? 0x80 : 0) | (transfer_len -1);
		tx_buf[1] = 0xD4;
		tx_buf[2] = (addr) >>8;
		tx_buf[3] = addr;

		if (out){
			memcpy(&tx_buf[4], out, transfer_len);
			out+=transfer_len;
		}


		ret = XSpiPs_PolledTransfer(&SpiInstance, tx_buf, rx_buf,4 + transfer_len );

		if (!(rx_buf[3] & 0x01)){
			fsbl_printf(DEBUG_GENERAL, "tpm_xfer: need for wait states\r\n");
			int i;
			for (i=0; i<TPM_WAIT_STATES; i++){
				ret = XSpiPs_PolledTransfer(&SpiInstance, tx_buf, rx_buf,1);
				if (ret !=XST_SUCCESS){
					fsbl_printf(DEBUG_GENERAL, "TPM Wait state failed\r\n");
					return XST_FAILURE;
				}
				if (rx_buf[0] & 0x01)
					break;

			}
		}

		if (ret!=XST_SUCCESS){
			fsbl_printf(DEBUG_GENERAL, "TPM transfer failed\r\n");
			return XST_FAILURE;
		}
		if (in){
			memcpy(in, &rx_buf[4], transfer_len);
			in += transfer_len;
		}

		len -=transfer_len;
	}
	return XST_SUCCESS;
}

int tpm_read(u16 addr, u8 *in, u16 len){
	return tpm_xfer(addr, NULL, in, len);
}

int tpm_read32(u32 addr, u32 * result){
	u32 result_le;
	int ret;
	ret = tpm_read(addr, &result_le, sizeof(u32));
	if (ret==XST_SUCCESS)
		*result = result_le;
	return ret;
}
int tpm_write(u16 addr, const u8 * out, u16 len){
	return tpm_xfer(addr, out, NULL, len);
}

int tpm_check_loc(int loc){
	const u8 mask = TPM_ACCESS_ACTIVE_LOCALITY | TPM_ACCESS_VALID;
	int ret;
	u8 buf;
	ret = tpm_read(TPM_ACCESS(loc), &buf, 1);
	if (ret!=XST_SUCCESS)
		return XST_FAILURE;

	if ((buf & mask) == mask){
		return XST_SUCCESS;
	}
	return XST_FAILURE;
}
void tpm_release_loc(int loc, u8 force){
	const u8 mask = TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID;
	u8 buf;
	if (tpm_read(TPM_ACCESS(loc), &buf, 1) != XST_SUCCESS)
		return;
//	if (force || (buf & mask) == mask) {
		buf = TPM_ACCESS_ACTIVE_LOCALITY;
		tpm_write(TPM_ACCESS(loc), &buf,1);
//	}
}

int tpm_request_loc(int loc){
	u8 buf = TPM_ACCESS_REQUEST_USE;
	int ret;

	ret = tpm_check_loc(loc);
	if (ret==XST_SUCCESS)
		return XST_SUCCESS;
	ret = tpm_write(TPM_ACCESS(loc), &buf, 1);
	return tpm_check_loc(loc);
}

int tpm_status(int loc, u8 * status){
	return tpm_read(TPM_STS(loc), status, 1);
}

int tpm_get_burstcount(int loc, u32 * burstcount){
	u32 bc_tmp=0;
	int ret = XST_FAILURE;
	ret = tpm_read32(TPM_STS(loc), &bc_tmp);
	bc_tmp = bc_tmp & 0xFFFF;
	memcpy(burstcount, &bc_tmp, sizeof(bc_tmp));
	return ret;
}
int tpm_cancel(int loc){
	u8 data = TPM_STS_COMMAND_READY;
	return tpm_write(TPM_STS(loc), &data, 1);
}

int tpm_wait_for_stat(int loc, u8 mask, unsigned long timeout, u8 * status){
	XTime tStart, tEnd;
	XTime_GetTime(&tStart);
	int ret;
	do{
		usleep(TPM_TIMEOUT_MS * 1000);
		ret = tpm_status(loc, status);
		if (ret!= XST_SUCCESS)
			return ret;
		if(((*status) & mask) == mask)
			return XST_SUCCESS;
		XTime_GetTime(&tEnd);
	}while((1.0 *(tEnd-tStart)/(COUNTS_PER_SECOND / 1000)) < timeout);
	return XST_FAILURE;
}



int tpm_send(int loc, const u8 * buf, size_t len){
	u32 i, size;
	u8 status;
	int burstcnt, ret;
	u8 data;

	if(len > TPM_DEV_BUFSIZE){
		fsbl_printf(DEBUG_GENERAL, "tpm_send: Length is greater than TPM_DEV_BUFSIZE\r\n");
		return XST_FAILURE;
	}
	ret = tpm_request_loc(loc);
	if(ret!=XST_SUCCESS){
		fsbl_printf(DEBUG_GENERAL, "tpm_send: Locality request failed.\r\n");
		return XST_FAILURE;
	}

	ret = tpm_status(loc, &status);
	if (ret!=XST_SUCCESS)
		return ret;
	if (!(status & TPM_STS_COMMAND_READY)){
		ret = tpm_cancel(loc);
		if (ret!= XST_SUCCESS){
			fsbl_printf(DEBUG_GENERAL, "tpm_send: Unable to cancel previous operation.\r\n");
			return XST_FAILURE;
		}

		ret = tpm_wait_for_stat(loc, TPM_STS_COMMAND_READY, TIS_LONG_TIMEOUT_MS, &status);
		if ((ret != XST_SUCCESS) || !(status & TPM_STS_COMMAND_READY)){
			fsbl_printf(DEBUG_GENERAL, "tpm_send: status %0x after wait for stat returned.\r\n")
					return XST_FAILURE;
		}
	}

	for (i = 0; i< len-1; ){
		tpm_get_burstcount(loc, &burstcnt);

		if (burstcnt<0){
			fsbl_printf(DEBUG_GENERAL, "tpm_send: Burst count < 0\r\n");
			return XST_FAILURE;
		}
		size = ((len-i-1) < burstcnt) ? (len-i-1): burstcnt;
		usleep(1000); //May remove this delay when hardware interface stable.
		ret = tpm_write(TPM_DATA_FIFO(loc), buf + i, size);
		if (ret !=XST_SUCCESS)
			return XST_FAILURE;
		i += size;
	}
	ret = tpm_status(loc, &status);
 	if (ret != XST_SUCCESS){
		fsbl_printf(DEBUG_GENERAL, "tpm_send: Status read fail.\r\n");
		return XST_FAILURE;
	}
	if ((status & TPM_STS_DATA_EXPECT)==0){
		fsbl_printf(DEBUG_GENERAL, "tpms_send: DATA_EXPECT not bit high.\r\n");
		return XST_FAILURE;
	}
	usleep(1000);//May remove this delay when hardware interface stable.
	ret = tpm_write(TPM_DATA_FIFO(loc), buf + len -1, 1);

	if (ret != XST_SUCCESS){
		fsbl_printf(DEBUG_GENERAL, "tpm_send: Command Write Fail.\r\n");
		return XST_FAILURE;
	}

	ret = tpm_status(loc, &status);
	if (ret != XST_SUCCESS){
		fsbl_printf(DEBUG_GENERAL, "tpm_send: Status read fail.\r\n");
		return XST_FAILURE;
	}

	if ((status & TPM_STS_DATA_EXPECT) != 0){
		fsbl_printf(DEBUG_GENERAL, "tpm_send: DATA_EXPECT bit high. Status: %02x.\r\n", status);
		return XST_FAILURE;
	}
	data = TPM_STS_GO;
	ret = tpm_write(TPM_STS(loc), &data, 1);
	if (ret!= XST_SUCCESS){
		fsbl_printf(DEBUG_GENERAL, "tpm_send: Write STS Fail.\r\n");
		return XST_FAILURE;
	}
	return len;
}



int tpm_wait_init(int loc){
	XTime tStart, tEnd;
	XTime_GetTime(&tStart);
	int ret;
	u8 status;
	unsigned long timeout = TPM_TIMEOUT_MS;
	do{
		ret = tpm_read(TPM_ACCESS(loc), &status, 1);
		if (ret!= XST_SUCCESS)
			return ret;
		if(status & TPM_ACCESS_VALID)
			return XST_SUCCESS;
		XTime_GetTime(&tEnd);
	}while((1.0 *(tEnd-tStart)/(COUNTS_PER_SECOND / 1000)) < timeout);
	return XST_FAILURE;
}

int tpm_recv_data (int loc, u8 * buf, size_t count){
	int size = 0, burstcnt, len, ret;
	u8 status;
	while ((size < count) && (tpm_wait_for_stat(loc, TPM_STS_DATA_AVAIL | TPM_STS_VALID, TIS_SHORT_TIMEOUT_MS,&status) == XST_SUCCESS)){
		tpm_get_burstcount(loc, &burstcnt);
		if (burstcnt < 0)
			return burstcnt;
		len = burstcnt<(count-size) ? burstcnt : (count-size);
		ret = tpm_read(TPM_DATA_FIFO(loc), buf + size, len);
		if (ret !=XST_SUCCESS)
			return ret;

		size += len;
	}
	return size;
}

int tpm_recv(int loc, u8 * buf, size_t count){
	int size, expected;
	if (count <TPM_HEADER_SIZE){
		fsbl_printf(DEBUG_GENERAL, "tpm_recv: count < TPM_HEADER SIZE\r\n");
		return XST_FAILURE;
	}
	size = tpm_recv_data(loc, buf, count);
	if (size < TPM_HEADER_SIZE){
		fsbl_printf(DEBUG_GENERAL, "tpm_recv: size (%d) <TPM_HEADER_SIZE (%d)\r\n", size, TPM_HEADER_SIZE);
		return XST_FAILURE;
	}
	expected = __get_unaligned_be32( &buf[2]);
	if (expected >count ){
		fsbl_printf(DEBUG_GENERAL, "tpm_recv: expected (%x) > count\r\n", expected);
		return XST_FAILURE;
	}


	return expected;
}

int tpm_pcr_read(int loc, int pcr_index, u8 * pcr_value_buf){
	u8 * pcr_cmd_buf;
	int pcr_byte_index  = 0;
	u8 pcr_select = 0; //PCR index as mapped bit value
	int ret = XST_FAILURE;
	u8 response[500];

	pcr_cmd_buf  = malloc(sizeof(tpm2_pcr_read_command));
	memcpy(pcr_cmd_buf, tpm2_pcr_read_command, sizeof(tpm2_pcr_read_command));
	pcr_byte_index = pcr_index / 8;
	pcr_select = (1 << (pcr_index%8));

	pcr_cmd_buf[17 + pcr_byte_index] = pcr_select;
	memcpy(pcr_cmd_buf + 14, sha256_alg, sizeof(sha256_alg));
	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_read: INFO Command bytes: ");
	for (int i=0; i < sizeof((tpm2_pcr_read_command)); i++){
		fsbl_printf(DEBUG_GENERAL,"%02x.", pcr_cmd_buf[i]);
	}
	fsbl_printf(DEBUG_GENERAL, "\r\n");

	//Sending the command to the TPM.
	ret = tpm_send(loc, pcr_cmd_buf, sizeof(tpm2_pcr_read_command));
	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_read(): INFO tpm_send() return: %d\r\n", ret);

	memset(response, 0xFF, sizeof(response));
	ret = tpm_recv(loc, response, 500);
	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_read(): INFO return size: %d\r\n", ret);
	if (ret>0){
		fsbl_printf(DEBUG_GENERAL, "tpm_pcr_read(): INFO PCR_READ(REG:%d): ", pcr_index);
		for (int i=0;i<ret;i++){
			fsbl_printf(DEBUG_GENERAL, "%02x.", response[i]);
		}
		fsbl_printf(DEBUG_GENERAL, "\r\n");
		if(pcr_value_buf!=NULL){
			memcpy(pcr_value_buf, response + 30, 32);
		}
	}

	free(pcr_cmd_buf);
}

int tpm_pcr_extend(int loc, u32 index, const uint8_t *digest){
	u8  * pcr_cmd_buf;
	unsigned int offset = 33;
	int ret=0;
	u8 response[500];

	pcr_cmd_buf = malloc(sizeof(tpm2_pcr_extend) + TPM2_DIGEST_LEN);
	memcpy(pcr_cmd_buf, tpm2_pcr_extend, sizeof(tpm2_pcr_extend));

	put_unaligned_be32(33+TPM2_DIGEST_LEN, pcr_cmd_buf +2); //copy digest length
	put_unaligned_be32(index, pcr_cmd_buf +10); //copy index
	memcpy(pcr_cmd_buf + 31, sha256_alg, sizeof(sha256_alg)); //copy sha type
	//copy digest
	if (digest ==NULL)
		return XST_FAILURE;

	memcpy(pcr_cmd_buf + sizeof(tpm2_pcr_extend), digest, 32);
/*	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_extend: INFO Command Bytes: ");
	for(int i=0; i<sizeof(tpm2_pcr_extend) + TPM2_DIGEST_LEN; i++){
		fsbl_printf(DEBUG_GENERAL, "%02x.", pcr_cmd_buf[i]);
	}
	fsbl_printf(DEBUG_GENERAL, "\r\n");
*/
	ret = tpm_send(loc, pcr_cmd_buf, sizeof(tpm2_pcr_extend) + TPM2_DIGEST_LEN);
//	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_extend(): INFO tpm_send() return: %d\r\n", ret);

	memset(response, 0xFF, sizeof(response));
	ret = tpm_recv(loc, response, 500);
/*	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_extend(): INFO return size: %d\r\n", ret);
	if (ret>0){
		fsbl_printf(DEBUG_GENERAL, "tpm_pcr_extend(): INFO PCR_READ(REG:%d): ", index);
		for (int i=0;i<ret;i++){
			fsbl_printf(DEBUG_GENERAL, "%02x.", response[i]);
		}
		fsbl_printf(DEBUG_GENERAL, "\r\n");
	}
*/	free(pcr_cmd_buf);

}

int tpm_pcr_reset(int loc, u8 pcr_index){
	u8 * pcr_cmd_buf;
	u8 response[500];
	int ret = XST_FAILURE;
	pcr_cmd_buf = malloc(sizeof(tpm2_pcr_reset_command));

	memcpy(pcr_cmd_buf, tpm2_pcr_reset_command, sizeof(tpm2_pcr_reset_command));
	pcr_cmd_buf[13] = pcr_index;
	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_reset: INFO Command bytes: ");
	for (int i=0; i < sizeof((tpm2_pcr_read_command)); i++){
		fsbl_printf(DEBUG_GENERAL,"%02x.", pcr_cmd_buf[i]);
	}
	fsbl_printf(DEBUG_GENERAL, "\r\n");
	//Sending the command to the TPM.
	ret = tpm_send(loc, pcr_cmd_buf, sizeof(tpm2_pcr_reset_command));
	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_reset(): INFO tpm_send() return: %d\r\n", ret);

	memset(response, 0xFF, sizeof(response));
	ret = tpm_recv(loc, response, 500);
	fsbl_printf(DEBUG_GENERAL, "tpm_pcr_reset(): INFO return size: %d\r\n", ret);
	if (ret>0){
		fsbl_printf(DEBUG_GENERAL, "tpm_pcr_reset(): INFO PCR_READ(REG:%d): ", pcr_index);
		for (int i=0;i<ret;i++){
			fsbl_printf(DEBUG_GENERAL, "%02x.", response[i]);
		}
		fsbl_printf(DEBUG_GENERAL, "\r\n");
	}

	free(pcr_cmd_buf);
}

int tpm_hash_start(){
	u8 buf = 0; //random data to initiate request
	int ret;

	ret = tpm_write(TPM_HASH_START, &buf, 1);
	return ret;

}

int tpm_hash_end(){
	u8 buf = 0; //random data to initiate request
	int ret;

	ret = tpm_write(TPM_HASH_END, &buf, 1);
	return ret;

}

int tpm_hash_data(u8 * buf, size_t len){
	return tpm_write(TPM_HASH_DATA, buf, len);
}
