/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 ******************************************************************************/
#define _HAL_INIT_C_

#include <linux/firmware.h>
#include <drv_types.h>
#include <rtw_efuse.h>

#include <rtl8723a_hal.h>

static void _FWDownloadEnable(struct rtw_adapter *padapter, bool enable)
{
	u8 tmp;

	if (enable) {
		/*  8051 enable */
		tmp = rtw_read8(padapter, REG_SYS_FUNC_EN + 1);
		rtw_write8(padapter, REG_SYS_FUNC_EN + 1, tmp | 0x04);

		/*  MCU firmware download enable. */
		tmp = rtw_read8(padapter, REG_MCUFWDL);
		rtw_write8(padapter, REG_MCUFWDL, tmp | 0x01);

		/*  8051 reset */
		tmp = rtw_read8(padapter, REG_MCUFWDL + 2);
		rtw_write8(padapter, REG_MCUFWDL + 2, tmp & 0xf7);
	} else {
		/*  MCU firmware download disable. */
		tmp = rtw_read8(padapter, REG_MCUFWDL);
		rtw_write8(padapter, REG_MCUFWDL, tmp & 0xfe);

		/*  Reserved for fw extension. */
		rtw_write8(padapter, REG_MCUFWDL + 1, 0x00);
	}
}

static int _BlockWrite(struct rtw_adapter *padapter, void *buffer, u32 buffSize)
{
	int ret = _SUCCESS;
	/*  (Default) Phase #1 : PCI muse use 4-byte write to download FW */
	u32 blockSize_p1 = 4;
	/*  Phase #2 : Use 8-byte, if Phase#1 use big size to write FW. */
	u32 blockSize_p2 = 8;
	/*  Phase #3 : Use 1-byte, the remnant of FW image. */
	u32 blockSize_p3 = 1;
	u32 blockCount_p1 = 0, blockCount_p2 = 0, blockCount_p3 = 0;
	u32 remainSize_p1 = 0, remainSize_p2 = 0;
	u8 *bufferPtr = (u8 *) buffer;
	u32 i = 0, offset = 0;

	blockSize_p1 = 254;

	/* 3 Phase #1 */
	blockCount_p1 = buffSize / blockSize_p1;
	remainSize_p1 = buffSize % blockSize_p1;

	if (blockCount_p1) {
		RT_TRACE(_module_hal_init_c_, _drv_notice_,
			 ("_BlockWrite: [P1] buffSize(%d) blockSize_p1(%d) "
			  "blockCount_p1(%d) remainSize_p1(%d)\n",
			  buffSize, blockSize_p1, blockCount_p1,
			  remainSize_p1));
	}

	for (i = 0; i < blockCount_p1; i++) {
		ret = rtw_writeN(padapter,
				 (FW_8723A_START_ADDRESS + i * blockSize_p1),
				 blockSize_p1, (bufferPtr + i * blockSize_p1));
		if (ret == _FAIL)
			goto exit;
	}

	/* 3 Phase #2 */
	if (remainSize_p1) {
		offset = blockCount_p1 * blockSize_p1;

		blockCount_p2 = remainSize_p1 / blockSize_p2;
		remainSize_p2 = remainSize_p1 % blockSize_p2;

		if (blockCount_p2) {
			RT_TRACE(_module_hal_init_c_, _drv_notice_,
				 ("_BlockWrite: [P2] buffSize_p2(%d) "
				  "blockSize_p2(%d) blockCount_p2(%d) "
				  "remainSize_p2(%d)\n",
				  (buffSize - offset), blockSize_p2,
				  blockCount_p2, remainSize_p2));
		}

		for (i = 0; i < blockCount_p2; i++) {
			ret = rtw_writeN(padapter,
					 (FW_8723A_START_ADDRESS + offset +
					  i * blockSize_p2), blockSize_p2,
					 (bufferPtr + offset +
					  i * blockSize_p2));

			if (ret == _FAIL)
				goto exit;
		}
	}

	/* 3 Phase #3 */
	if (remainSize_p2) {
		offset = (blockCount_p1 * blockSize_p1) +
			(blockCount_p2 * blockSize_p2);

		blockCount_p3 = remainSize_p2 / blockSize_p3;

		RT_TRACE(_module_hal_init_c_, _drv_notice_,
			 ("_BlockWrite: [P3] buffSize_p3(%d) blockSize_p3(%d) "
			  "blockCount_p3(%d)\n",
			  (buffSize - offset), blockSize_p3, blockCount_p3));

		for (i = 0; i < blockCount_p3; i++) {
			ret = rtw_write8(padapter,
					 (FW_8723A_START_ADDRESS + offset + i),
					 *(bufferPtr + offset + i));

			if (ret == _FAIL)
				goto exit;
		}
	}

exit:
	return ret;
}

static int
_PageWrite(struct rtw_adapter *padapter, u32 page, void *buffer, u32 size)
{
	u8 value8;
	u8 u8Page = (u8) (page & 0x07);

	value8 = (rtw_read8(padapter, REG_MCUFWDL + 2) & 0xF8) | u8Page;
	rtw_write8(padapter, REG_MCUFWDL + 2, value8);

	return _BlockWrite(padapter, buffer, size);
}

static int _WriteFW(struct rtw_adapter *padapter, void *buffer, u32 size)
{
	/*  Since we need dynamic decide method of dwonload fw, so we
	    call this function to get chip version. */
	/*  We can remove _ReadChipVersion from ReadpadapterInfo8192C later. */
	int ret = _SUCCESS;
	u32 pageNums, remainSize;
	u32 page, offset;
	u8 *bufferPtr = (u8 *) buffer;

	pageNums = size / MAX_PAGE_SIZE;
	/* RT_ASSERT((pageNums <= 4),
	   ("Page numbers should not greater then 4 \n")); */
	remainSize = size % MAX_PAGE_SIZE;

	for (page = 0; page < pageNums; page++) {
		offset = page * MAX_PAGE_SIZE;
		ret = _PageWrite(padapter, page, bufferPtr + offset,
				 MAX_PAGE_SIZE);

		if (ret == _FAIL)
			goto exit;
	}
	if (remainSize) {
		offset = pageNums * MAX_PAGE_SIZE;
		page = pageNums;
		ret = _PageWrite(padapter, page, bufferPtr + offset,
				 remainSize);

		if (ret == _FAIL)
			goto exit;
	}
	RT_TRACE(_module_hal_init_c_, _drv_info_,
		 ("_WriteFW Done- for Normal chip.\n"));

exit:
	return ret;
}

static s32 _FWFreeToGo(struct rtw_adapter *padapter)
{
	u32 counter = 0;
	u32 value32;

	/*  polling CheckSum report */
	do {
		value32 = rtw_read32(padapter, REG_MCUFWDL);
		if (value32 & FWDL_ChkSum_rpt)
			break;
	} while (counter++ < POLLING_READY_TIMEOUT_COUNT);

	if (counter >= POLLING_READY_TIMEOUT_COUNT) {
		RT_TRACE(_module_hal_init_c_, _drv_err_,
			 ("%s: chksum report fail! REG_MCUFWDL:0x%08x\n",
			  __func__, value32));
		return _FAIL;
	}
	RT_TRACE(_module_hal_init_c_, _drv_info_,
		 ("%s: Checksum report OK! REG_MCUFWDL:0x%08x\n", __func__,
		  value32));

	value32 = rtw_read32(padapter, REG_MCUFWDL);
	value32 |= MCUFWDL_RDY;
	value32 &= ~WINTINI_RDY;
	rtw_write32(padapter, REG_MCUFWDL, value32);

	/*  polling for FW ready */
	counter = 0;
	do {
		value32 = rtw_read32(padapter, REG_MCUFWDL);
		if (value32 & WINTINI_RDY) {
			RT_TRACE(_module_hal_init_c_, _drv_info_,
				 ("%s: Polling FW ready success!! "
				  "REG_MCUFWDL:0x%08x\n",
				  __func__, value32));
			return _SUCCESS;
		}
		udelay(5);
	} while (counter++ < POLLING_READY_TIMEOUT_COUNT);

	RT_TRACE(_module_hal_init_c_, _drv_err_,
		 ("%s: Polling FW ready fail!! REG_MCUFWDL:0x%08x\n",
		  __func__, value32));
	return _FAIL;
}

#define IS_FW_81xxC(padapter)	(((GET_HAL_DATA(padapter))->FirmwareSignature & 0xFFF0) == 0x88C0)

void rtl8723a_FirmwareSelfReset(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);
	u8 u1bTmp;
	u8 Delay = 100;

	if (!(IS_FW_81xxC(padapter) &&
	      ((pHalData->FirmwareVersion < 0x21) ||
	       (pHalData->FirmwareVersion == 0x21 &&
		pHalData->FirmwareSubVersion < 0x01)))) {
		/*  after 88C Fw v33.1 */
		/* 0x1cf = 0x20. Inform 8051 to reset. 2009.12.25. tynli_test */
		rtw_write8(padapter, REG_HMETFR + 3, 0x20);

		u1bTmp = rtw_read8(padapter, REG_SYS_FUNC_EN + 1);
		while (u1bTmp & BIT2) {
			Delay--;
			if (Delay == 0)
				break;
			udelay(50);
			u1bTmp = rtw_read8(padapter, REG_SYS_FUNC_EN + 1);
		}
		RT_TRACE(_module_hal_init_c_, _drv_info_,
			 ("-%s: 8051 reset success (%d)\n", __func__,
			  Delay));

		if ((Delay == 0)) {
			/* force firmware reset */
			u1bTmp = rtw_read8(padapter, REG_SYS_FUNC_EN + 1);
			rtw_write8(padapter, REG_SYS_FUNC_EN + 1,
				   u1bTmp & (~BIT2));
		}
	}
}

/*  */
/*	Description: */
/*		Download 8192C firmware code. */
/*  */
/*  */
s32 rtl8723a_FirmwareDownload(struct rtw_adapter *padapter)
{
	s32 rtStatus = _SUCCESS;
	u8 writeFW_retry = 0;
	unsigned long fwdl_start_time;
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);
	struct dvobj_priv *dvobj = adapter_to_dvobj(padapter);
	struct device *device = dvobj_to_dev(dvobj);
	struct rt_8723a_firmware_hdr *pFwHdr = NULL;
	const struct firmware *fw;
	char *fw_name;
	u8 *firmware_buf = NULL;
	u8 *buf;
	int fw_size;
	static int log_version;

	RT_TRACE(_module_hal_init_c_, _drv_info_, ("+%s\n", __func__));

	if (IS_8723A_A_CUT(pHalData->VersionID)) {
		fw_name = "rtlwifi/rtl8723aufw.bin";
		RT_TRACE(_module_hal_init_c_, _drv_info_,
			 ("rtl8723a_FirmwareDownload: R8723FwImageArray_UMC "
			  "for RTL8723A A CUT\n"));
	} else if (IS_8723A_B_CUT(pHalData->VersionID)) {
		/*  WLAN Fw. */
		if (padapter->registrypriv.wifi_spec == 1) {
			fw_name = "rtlwifi/rtl8723aufw_B_NoBT.bin";
			DBG_8723A(" Rtl8723_FwUMCBCutImageArrayWithoutBT for "
				  "RTL8723A B CUT\n");
		} else {
#ifdef CONFIG_8723AU_BT_COEXIST
			fw_name = "rtlwifi/rtl8723aufw_B.bin";
			DBG_8723A(" Rtl8723_FwUMCBCutImageArrayWithBT for "
				  "RTL8723A B CUT\n");
#else
			fw_name = "rtlwifi/rtl8723aufw_B_NoBT.bin";
			DBG_8723A(" Rtl8723_FwUMCBCutImageArrayWithoutBT for "
				  "RTL8723A B CUT\n");
#endif
		}
	} else {
		/*  <Roger_TODO> We should download proper RAM Code here
		    to match the ROM code. */
		RT_TRACE(_module_hal_init_c_, _drv_err_,
			 ("%s: unknow version!\n", __func__));
		rtStatus = _FAIL;
		goto Exit;
	}

	pr_info("rtl8723au: Loading firmware %s\n", fw_name);
	if (request_firmware(&fw, fw_name, device)) {
		pr_err("rtl8723au: request_firmware load failed\n");
		rtStatus = _FAIL;
		goto Exit;
	}
	if (!fw) {
		pr_err("rtl8723au: Firmware %s not available\n", fw_name);
		rtStatus = _FAIL;
		goto Exit;
	}
	firmware_buf = kzalloc(fw->size, GFP_KERNEL);
	if (!firmware_buf) {
		rtStatus = _FAIL;
		goto Exit;
	}
	memcpy(firmware_buf, fw->data, fw->size);
	buf = firmware_buf;
	fw_size = fw->size;
	release_firmware(fw);

	/*  To Check Fw header. Added by tynli. 2009.12.04. */
	pFwHdr = (struct rt_8723a_firmware_hdr *)firmware_buf;

	pHalData->FirmwareVersion = le16_to_cpu(pFwHdr->Version);
	pHalData->FirmwareSubVersion = pFwHdr->Subversion;
	pHalData->FirmwareSignature = le16_to_cpu(pFwHdr->Signature);

	DBG_8723A("%s: fw_ver =%d fw_subver =%d sig = 0x%x\n",
		  __func__, pHalData->FirmwareVersion,
		  pHalData->FirmwareSubVersion, pHalData->FirmwareSignature);

	if (!log_version++)
		pr_info("%sFirmware Version %d, SubVersion %d, Signature "
			"0x%x\n", DRIVER_PREFIX, pHalData->FirmwareVersion,
			pHalData->FirmwareSubVersion,
			pHalData->FirmwareSignature);

	if (IS_FW_HEADER_EXIST(pFwHdr)) {
		/*  Shift 32 bytes for FW header */
		buf = buf + 32;
		fw_size = fw_size - 32;
	}

	/*  Suggested by Filen. If 8051 is running in RAM code, driver should
	    inform Fw to reset by itself, */
	/*  or it will cause download Fw fail. 2010.02.01. by tynli. */
	if (rtw_read8(padapter, REG_MCUFWDL) & RAM_DL_SEL) {
		/* 8051 RAM code */
		rtl8723a_FirmwareSelfReset(padapter);
		rtw_write8(padapter, REG_MCUFWDL, 0x00);
	}

	_FWDownloadEnable(padapter, true);
	fwdl_start_time = jiffies;
	while (1) {
		/* reset the FWDL chksum */
		rtw_write8(padapter, REG_MCUFWDL,
			   rtw_read8(padapter, REG_MCUFWDL) | FWDL_ChkSum_rpt);

		rtStatus = _WriteFW(padapter, buf, fw_size);

		if (rtStatus == _SUCCESS ||
		    (jiffies_to_msecs(jiffies - fwdl_start_time) > 500 &&
		     writeFW_retry++ >= 3))
			break;

		DBG_8723A("%s writeFW_retry:%u, time after fwdl_start_time:"
			  "%ums\n", __func__, writeFW_retry,
			  jiffies_to_msecs(jiffies - fwdl_start_time));
	}
	_FWDownloadEnable(padapter, false);
	if (_SUCCESS != rtStatus) {
		DBG_8723A("DL Firmware failed!\n");
		goto Exit;
	}

	rtStatus = _FWFreeToGo(padapter);
	if (_SUCCESS != rtStatus) {
		RT_TRACE(_module_hal_init_c_, _drv_err_,
			 ("DL Firmware failed!\n"));
		goto Exit;
	}
	RT_TRACE(_module_hal_init_c_, _drv_info_,
		 ("Firmware is ready to run!\n"));

Exit:
	kfree(firmware_buf);
	return rtStatus;
}

void rtl8723a_InitializeFirmwareVars(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	/*  Init Fw LPS related. */
	padapter->pwrctrlpriv.bFwCurrentInPSMode = false;

	/*  Init H2C counter. by tynli. 2009.12.09. */
	pHalData->LastHMEBoxNum = 0;
}

static void rtl8723a_free_hal_data(struct rtw_adapter *padapter)
{

	kfree(padapter->HalData);
	padapter->HalData = NULL;

}

/*  */
/*				Efuse related code */
/*  */
static u8
hal_EfuseSwitchToBank(struct rtw_adapter *padapter, u8 bank)
{
	u8 bRet = false;
	u32 value32 = 0;

	DBG_8723A("%s: Efuse switch bank to %d\n", __func__, bank);
	value32 = rtw_read32(padapter, EFUSE_TEST);
	bRet = true;
	switch (bank) {
	case 0:
		value32 = (value32 & ~EFUSE_SEL_MASK) |
			EFUSE_SEL(EFUSE_WIFI_SEL_0);
		break;
	case 1:
		value32 = (value32 & ~EFUSE_SEL_MASK) |
			EFUSE_SEL(EFUSE_BT_SEL_0);
		break;
	case 2:
		value32 = (value32 & ~EFUSE_SEL_MASK) |
			EFUSE_SEL(EFUSE_BT_SEL_1);
		break;
	case 3:
		value32 = (value32 & ~EFUSE_SEL_MASK) |
			EFUSE_SEL(EFUSE_BT_SEL_2);
		break;
	default:
		value32 = (value32 & ~EFUSE_SEL_MASK) |
			EFUSE_SEL(EFUSE_WIFI_SEL_0);
		bRet = false;
		break;
	}
	rtw_write32(padapter, EFUSE_TEST, value32);

	return bRet;
}

static void
Hal_GetEfuseDefinition(struct rtw_adapter *padapter,
		       u8 efuseType, u8 type, void *pOut)
{
	u8 *pu1Tmp;
	u16 *pu2Tmp;
	u8 *pMax_section;

	switch (type) {
	case TYPE_EFUSE_MAX_SECTION:
		pMax_section = (u8 *) pOut;

		if (efuseType == EFUSE_WIFI)
			*pMax_section = EFUSE_MAX_SECTION_8723A;
		else
			*pMax_section = EFUSE_BT_MAX_SECTION;
		break;

	case TYPE_EFUSE_REAL_CONTENT_LEN:
		pu2Tmp = (u16 *) pOut;

		if (efuseType == EFUSE_WIFI)
			*pu2Tmp = EFUSE_REAL_CONTENT_LEN_8723A;
		else
			*pu2Tmp = EFUSE_BT_REAL_CONTENT_LEN;
		break;

	case TYPE_AVAILABLE_EFUSE_BYTES_BANK:
		pu2Tmp = (u16 *) pOut;

		if (efuseType == EFUSE_WIFI)
			*pu2Tmp = (EFUSE_REAL_CONTENT_LEN_8723A -
				   EFUSE_OOB_PROTECT_BYTES);
		else
			*pu2Tmp = (EFUSE_BT_REAL_BANK_CONTENT_LEN -
				   EFUSE_PROTECT_BYTES_BANK);
		break;

	case TYPE_AVAILABLE_EFUSE_BYTES_TOTAL:
		pu2Tmp = (u16 *) pOut;

		if (efuseType == EFUSE_WIFI)
			*pu2Tmp = (EFUSE_REAL_CONTENT_LEN_8723A -
				   EFUSE_OOB_PROTECT_BYTES);
		else
			*pu2Tmp = (EFUSE_BT_REAL_CONTENT_LEN -
				   (EFUSE_PROTECT_BYTES_BANK * 3));
		break;

	case TYPE_EFUSE_MAP_LEN:
		pu2Tmp = (u16 *) pOut;

		if (efuseType == EFUSE_WIFI)
			*pu2Tmp = EFUSE_MAP_LEN_8723A;
		else
			*pu2Tmp = EFUSE_BT_MAP_LEN;
		break;

	case TYPE_EFUSE_PROTECT_BYTES_BANK:
		pu1Tmp = (u8 *) pOut;

		if (efuseType == EFUSE_WIFI)
			*pu1Tmp = EFUSE_OOB_PROTECT_BYTES;
		else
			*pu1Tmp = EFUSE_PROTECT_BYTES_BANK;
		break;

	case TYPE_EFUSE_CONTENT_LEN_BANK:
		pu2Tmp = (u16 *) pOut;

		if (efuseType == EFUSE_WIFI)
			*pu2Tmp = EFUSE_REAL_CONTENT_LEN_8723A;
		else
			*pu2Tmp = EFUSE_BT_REAL_BANK_CONTENT_LEN;
		break;

	default:
		pu1Tmp = (u8 *) pOut;
		*pu1Tmp = 0;
		break;
	}
}

#define VOLTAGE_V25		0x03
#define LDOE25_SHIFT	28

static void
Hal_EfusePowerSwitch(struct rtw_adapter *padapter, u8 bWrite, u8 PwrState)
{
	u8 tempval;
	u16 tmpV16;

	if (PwrState == true) {
		rtw_write8(padapter, REG_EFUSE_ACCESS, EFUSE_ACCESS_ON);

		/*  1.2V Power: From VDDON with Power
		    Cut(0x0000h[15]), defualt valid */
		tmpV16 = rtw_read16(padapter, REG_SYS_ISO_CTRL);
		if (!(tmpV16 & PWC_EV12V)) {
			tmpV16 |= PWC_EV12V;
			rtw_write16(padapter, REG_SYS_ISO_CTRL, tmpV16);
		}
		/*  Reset: 0x0000h[28], default valid */
		tmpV16 = rtw_read16(padapter, REG_SYS_FUNC_EN);
		if (!(tmpV16 & FEN_ELDR)) {
			tmpV16 |= FEN_ELDR;
			rtw_write16(padapter, REG_SYS_FUNC_EN, tmpV16);
		}

		/*  Clock: Gated(0x0008h[5]) 8M(0x0008h[1]) clock
		    from ANA, default valid */
		tmpV16 = rtw_read16(padapter, REG_SYS_CLKR);
		if ((!(tmpV16 & LOADER_CLK_EN)) || (!(tmpV16 & ANA8M))) {
			tmpV16 |= (LOADER_CLK_EN | ANA8M);
			rtw_write16(padapter, REG_SYS_CLKR, tmpV16);
		}

		if (bWrite == true) {
			/*  Enable LDO 2.5V before read/write action */
			tempval = rtw_read8(padapter, EFUSE_TEST + 3);
			tempval &= 0x0F;
			tempval |= (VOLTAGE_V25 << 4);
			rtw_write8(padapter, EFUSE_TEST + 3, (tempval | 0x80));
		}
	} else {
		rtw_write8(padapter, REG_EFUSE_ACCESS, EFUSE_ACCESS_OFF);

		if (bWrite == true) {
			/*  Disable LDO 2.5V after read/write action */
			tempval = rtw_read8(padapter, EFUSE_TEST + 3);
			rtw_write8(padapter, EFUSE_TEST + 3, (tempval & 0x7F));
		}
	}
}

static void
hal_ReadEFuse_WiFi(struct rtw_adapter *padapter,
		   u16 _offset, u16 _size_byte, u8 *pbuf)
{
	u8 *efuseTbl = NULL;
	u16 eFuse_Addr = 0;
	u8 offset, wden;
	u8 efuseHeader, efuseExtHdr, efuseData;
	u16 i, total, used;

	/*  Do NOT excess total size of EFuse table.
	    Added by Roger, 2008.11.10. */
	if ((_offset + _size_byte) > EFUSE_MAP_LEN_8723A) {
		DBG_8723A("%s: Invalid offset(%#x) with read bytes(%#x)!!\n",
			  __func__, _offset, _size_byte);
		return;
	}

	efuseTbl = (u8 *) kmalloc(EFUSE_MAP_LEN_8723A, GFP_KERNEL);
	if (efuseTbl == NULL) {
		DBG_8723A("%s: alloc efuseTbl fail!\n", __func__);
		return;
	}
	/*  0xff will be efuse default value instead of 0x00. */
	memset(efuseTbl, 0xFF, EFUSE_MAP_LEN_8723A);

	/*  switch bank back to bank 0 for later BT and wifi use. */
	hal_EfuseSwitchToBank(padapter, 0);

	while (AVAILABLE_EFUSE_ADDR(eFuse_Addr)) {
		ReadEFuseByte23a(padapter, eFuse_Addr++, &efuseHeader);
		if (efuseHeader == 0xFF) {
			DBG_8723A("%s: data end at address =%#x\n", __func__,
				  eFuse_Addr);
			break;
		}

		/*  Check PG header for section num. */
		if (EXT_HEADER(efuseHeader)) {	/* extended header */
			offset = GET_HDR_OFFSET_2_0(efuseHeader);

			ReadEFuseByte23a(padapter, eFuse_Addr++, &efuseExtHdr);
			if (ALL_WORDS_DISABLED(efuseExtHdr)) {
				continue;
			}

			offset |= ((efuseExtHdr & 0xF0) >> 1);
			wden = (efuseExtHdr & 0x0F);
		} else {
			offset = ((efuseHeader >> 4) & 0x0f);
			wden = (efuseHeader & 0x0f);
		}

		if (offset < EFUSE_MAX_SECTION_8723A) {
			u16 addr;
			/*  Get word enable value from PG header */

			addr = offset * PGPKT_DATA_SIZE;
			for (i = 0; i < EFUSE_MAX_WORD_UNIT; i++) {
				/* Check word enable condition in the section */
				if (!(wden & (0x01 << i))) {
					ReadEFuseByte23a(padapter, eFuse_Addr++,
						      &efuseData);
					efuseTbl[addr] = efuseData;

					ReadEFuseByte23a(padapter, eFuse_Addr++,
						      &efuseData);
					efuseTbl[addr + 1] = efuseData;
				}
				addr += 2;
			}
		} else {
			DBG_8723A(KERN_ERR "%s: offset(%d) is illegal!!\n",
				  __func__, offset);
			eFuse_Addr += Efuse_CalculateWordCnts23a(wden) * 2;
		}
	}

	/*  Copy from Efuse map to output pointer memory!!! */
	for (i = 0; i < _size_byte; i++)
		pbuf[i] = efuseTbl[_offset + i];

	/*  Calculate Efuse utilization */
	EFUSE_GetEfuseDefinition23a(padapter, EFUSE_WIFI,
				 TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, &total);
	used = eFuse_Addr - 1;
	rtw_hal_set_hwreg23a(padapter, HW_VAR_EFUSE_BYTES, (u8 *)&used);

	kfree(efuseTbl);
}

static void
hal_ReadEFuse_BT(struct rtw_adapter *padapter,
		 u16 _offset, u16 _size_byte, u8 *pbuf)
{
	u8 *efuseTbl;
	u8 bank;
	u16 eFuse_Addr;
	u8 efuseHeader, efuseExtHdr, efuseData;
	u8 offset, wden;
	u16 i, total, used;

	/*  Do NOT excess total size of EFuse table.
	    Added by Roger, 2008.11.10. */
	if ((_offset + _size_byte) > EFUSE_BT_MAP_LEN) {
		DBG_8723A("%s: Invalid offset(%#x) with read bytes(%#x)!!\n",
			  __func__, _offset, _size_byte);
		return;
	}

	efuseTbl = kmalloc(EFUSE_BT_MAP_LEN, GFP_KERNEL);
	if (efuseTbl == NULL) {
		DBG_8723A("%s: efuseTbl malloc fail!\n", __func__);
		return;
	}
	/*  0xff will be efuse default value instead of 0x00. */
	memset(efuseTbl, 0xFF, EFUSE_BT_MAP_LEN);

	EFUSE_GetEfuseDefinition23a(padapter, EFUSE_BT,
				 TYPE_AVAILABLE_EFUSE_BYTES_BANK, &total);

	for (bank = 1; bank < EFUSE_MAX_BANK; bank++) {
		if (hal_EfuseSwitchToBank(padapter, bank) == false) {
			DBG_8723A("%s: hal_EfuseSwitchToBank Fail!!\n",
				  __func__);
			goto exit;
		}

		eFuse_Addr = 0;

		while (AVAILABLE_EFUSE_ADDR(eFuse_Addr)) {
			ReadEFuseByte23a(padapter, eFuse_Addr++, &efuseHeader);
			if (efuseHeader == 0xFF)
				break;

			/*  Check PG header for section num. */
			if (EXT_HEADER(efuseHeader)) {	/* extended header */
				offset = GET_HDR_OFFSET_2_0(efuseHeader);

				ReadEFuseByte23a(padapter, eFuse_Addr++,
					      &efuseExtHdr);
				if (ALL_WORDS_DISABLED(efuseExtHdr)) {
					continue;
				}

				offset |= ((efuseExtHdr & 0xF0) >> 1);
				wden = (efuseExtHdr & 0x0F);
			} else {
				offset = ((efuseHeader >> 4) & 0x0f);
				wden = (efuseHeader & 0x0f);
			}

			if (offset < EFUSE_BT_MAX_SECTION) {
				u16 addr;

				/*  Get word enable value from PG header */

				addr = offset * PGPKT_DATA_SIZE;
				for (i = 0; i < EFUSE_MAX_WORD_UNIT; i++) {
					/*  Check word enable condition in
					    the section */
					if (!(wden & (0x01 << i))) {
						ReadEFuseByte23a(padapter,
							      eFuse_Addr++,
							      &efuseData);
						efuseTbl[addr] = efuseData;

						ReadEFuseByte23a(padapter,
							      eFuse_Addr++,
							      &efuseData);
						efuseTbl[addr + 1] = efuseData;
					}
					addr += 2;
				}
			} else {
				DBG_8723A(KERN_ERR
					  "%s: offset(%d) is illegal!!\n",
					  __func__, offset);
				eFuse_Addr += Efuse_CalculateWordCnts23a(wden) * 2;
			}
		}

		if ((eFuse_Addr - 1) < total) {
			DBG_8723A("%s: bank(%d) data end at %#x\n",
				  __func__, bank, eFuse_Addr - 1);
			break;
		}
	}

	/*  switch bank back to bank 0 for later BT and wifi use. */
	hal_EfuseSwitchToBank(padapter, 0);

	/*  Copy from Efuse map to output pointer memory!!! */
	for (i = 0; i < _size_byte; i++)
		pbuf[i] = efuseTbl[_offset + i];

	/*  */
	/*  Calculate Efuse utilization. */
	/*  */
	EFUSE_GetEfuseDefinition23a(padapter, EFUSE_BT,
				 TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, &total);
	used = (EFUSE_BT_REAL_BANK_CONTENT_LEN * (bank - 1)) + eFuse_Addr - 1;
	rtw_hal_set_hwreg23a(padapter, HW_VAR_EFUSE_BT_BYTES, (u8 *) &used);

exit:
	kfree(efuseTbl);
}

static void
Hal_ReadEFuse(struct rtw_adapter *padapter,
	      u8 efuseType, u16 _offset, u16 _size_byte, u8 *pbuf)
{
	if (efuseType == EFUSE_WIFI)
		hal_ReadEFuse_WiFi(padapter, _offset, _size_byte, pbuf);
	else
		hal_ReadEFuse_BT(padapter, _offset, _size_byte, pbuf);
}

static u16
hal_EfuseGetCurrentSize_WiFi(struct rtw_adapter *padapter)
{
	u16 efuse_addr = 0;
	u8 hoffset = 0, hworden = 0;
	u8 efuse_data, word_cnts = 0;

	rtw23a_hal_get_hwreg(padapter, HW_VAR_EFUSE_BYTES, (u8 *) &efuse_addr);

	DBG_8723A("%s: start_efuse_addr = 0x%X\n", __func__, efuse_addr);

	/*  switch bank back to bank 0 for later BT and wifi use. */
	hal_EfuseSwitchToBank(padapter, 0);

	while (AVAILABLE_EFUSE_ADDR(efuse_addr)) {
		if (efuse_OneByteRead23a(padapter, efuse_addr, &efuse_data) ==
		    false) {
			DBG_8723A(KERN_ERR "%s: efuse_OneByteRead23a Fail! "
				  "addr = 0x%X !!\n", __func__, efuse_addr);
			break;
		}

		if (efuse_data == 0xFF)
			break;

		if (EXT_HEADER(efuse_data)) {
			hoffset = GET_HDR_OFFSET_2_0(efuse_data);
			efuse_addr++;
			efuse_OneByteRead23a(padapter, efuse_addr, &efuse_data);
			if (ALL_WORDS_DISABLED(efuse_data)) {
				continue;
			}

			hoffset |= ((efuse_data & 0xF0) >> 1);
			hworden = efuse_data & 0x0F;
		} else {
			hoffset = (efuse_data >> 4) & 0x0F;
			hworden = efuse_data & 0x0F;
		}

		word_cnts = Efuse_CalculateWordCnts23a(hworden);
		efuse_addr += (word_cnts * 2) + 1;
	}

	rtw_hal_set_hwreg23a(padapter, HW_VAR_EFUSE_BYTES, (u8 *) &efuse_addr);

	DBG_8723A("%s: CurrentSize =%d\n", __func__, efuse_addr);

	return efuse_addr;
}

static u16
hal_EfuseGetCurrentSize_BT(struct rtw_adapter *padapter)
{
	u16 btusedbytes;
	u16 efuse_addr;
	u8 bank, startBank;
	u8 hoffset = 0, hworden = 0;
	u8 efuse_data, word_cnts = 0;
	u16 retU2 = 0;

	rtw23a_hal_get_hwreg(padapter, HW_VAR_EFUSE_BT_BYTES, (u8 *) &btusedbytes);

	efuse_addr = (u16) ((btusedbytes % EFUSE_BT_REAL_BANK_CONTENT_LEN));
	startBank = (u8) (1 + (btusedbytes / EFUSE_BT_REAL_BANK_CONTENT_LEN));

	DBG_8723A("%s: start from bank =%d addr = 0x%X\n", __func__, startBank,
		  efuse_addr);

	EFUSE_GetEfuseDefinition23a(padapter, EFUSE_BT,
				 TYPE_AVAILABLE_EFUSE_BYTES_BANK, &retU2);

	for (bank = startBank; bank < EFUSE_MAX_BANK; bank++) {
		if (hal_EfuseSwitchToBank(padapter, bank) == false) {
			DBG_8723A(KERN_ERR "%s: switch bank(%d) Fail!!\n",
				  __func__, bank);
			bank = EFUSE_MAX_BANK;
			break;
		}

		/*  only when bank is switched we have to reset
		    the efuse_addr. */
		if (bank != startBank)
			efuse_addr = 0;

		while (AVAILABLE_EFUSE_ADDR(efuse_addr)) {
			if (efuse_OneByteRead23a(padapter, efuse_addr,
					      &efuse_data) == false) {
				DBG_8723A(KERN_ERR "%s: efuse_OneByteRead23a Fail!"
					  " addr = 0x%X !!\n",
					  __func__, efuse_addr);
				bank = EFUSE_MAX_BANK;
				break;
			}

			if (efuse_data == 0xFF)
				break;

			if (EXT_HEADER(efuse_data)) {
				hoffset = GET_HDR_OFFSET_2_0(efuse_data);
				efuse_addr++;
				efuse_OneByteRead23a(padapter, efuse_addr,
						  &efuse_data);
				if (ALL_WORDS_DISABLED(efuse_data)) {
					efuse_addr++;
					continue;
				}

				hoffset |= ((efuse_data & 0xF0) >> 1);
				hworden = efuse_data & 0x0F;
			} else {
				hoffset = (efuse_data >> 4) & 0x0F;
				hworden = efuse_data & 0x0F;
			}
			word_cnts = Efuse_CalculateWordCnts23a(hworden);
			/* read next header */
			efuse_addr += (word_cnts * 2) + 1;
		}

		/*  Check if we need to check next bank efuse */
		if (efuse_addr < retU2) {
			break;	/*  don't need to check next bank. */
		}
	}

	retU2 = ((bank - 1) * EFUSE_BT_REAL_BANK_CONTENT_LEN) + efuse_addr;
	rtw_hal_set_hwreg23a(padapter, HW_VAR_EFUSE_BT_BYTES, (u8 *)&retU2);

	DBG_8723A("%s: CurrentSize =%d\n", __func__, retU2);
	return retU2;
}

static u16
Hal_EfuseGetCurrentSize(struct rtw_adapter *pAdapter, u8 efuseType)
{
	u16 ret = 0;

	if (efuseType == EFUSE_WIFI)
		ret = hal_EfuseGetCurrentSize_WiFi(pAdapter);
	else
		ret = hal_EfuseGetCurrentSize_BT(pAdapter);

	return ret;
}

static u8
Hal_EfuseWordEnableDataWrite(struct rtw_adapter *padapter,
			     u16 efuse_addr, u8 word_en, u8 *data)
{
	u16 tmpaddr = 0;
	u16 start_addr = efuse_addr;
	u8 badworden = 0x0F;
	u8 tmpdata[PGPKT_DATA_SIZE];

	memset(tmpdata, 0xFF, PGPKT_DATA_SIZE);

	if (!(word_en & BIT(0))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite23a(padapter, start_addr++, data[0]);
		efuse_OneByteWrite23a(padapter, start_addr++, data[1]);

		efuse_OneByteRead23a(padapter, tmpaddr, &tmpdata[0]);
		efuse_OneByteRead23a(padapter, tmpaddr + 1, &tmpdata[1]);
		if ((data[0] != tmpdata[0]) || (data[1] != tmpdata[1])) {
			badworden &= (~BIT(0));
		}
	}
	if (!(word_en & BIT(1))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite23a(padapter, start_addr++, data[2]);
		efuse_OneByteWrite23a(padapter, start_addr++, data[3]);

		efuse_OneByteRead23a(padapter, tmpaddr, &tmpdata[2]);
		efuse_OneByteRead23a(padapter, tmpaddr + 1, &tmpdata[3]);
		if ((data[2] != tmpdata[2]) || (data[3] != tmpdata[3])) {
			badworden &= (~BIT(1));
		}
	}
	if (!(word_en & BIT(2))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite23a(padapter, start_addr++, data[4]);
		efuse_OneByteWrite23a(padapter, start_addr++, data[5]);

		efuse_OneByteRead23a(padapter, tmpaddr, &tmpdata[4]);
		efuse_OneByteRead23a(padapter, tmpaddr + 1, &tmpdata[5]);
		if ((data[4] != tmpdata[4]) || (data[5] != tmpdata[5])) {
			badworden &= (~BIT(2));
		}
	}
	if (!(word_en & BIT(3))) {
		tmpaddr = start_addr;
		efuse_OneByteWrite23a(padapter, start_addr++, data[6]);
		efuse_OneByteWrite23a(padapter, start_addr++, data[7]);

		efuse_OneByteRead23a(padapter, tmpaddr, &tmpdata[6]);
		efuse_OneByteRead23a(padapter, tmpaddr + 1, &tmpdata[7]);
		if ((data[6] != tmpdata[6]) || (data[7] != tmpdata[7])) {
			badworden &= (~BIT(3));
		}
	}

	return badworden;
}

static s32
Hal_EfusePgPacketRead(struct rtw_adapter *padapter, u8 offset, u8 *data)
{
	u8 efuse_data, word_cnts = 0;
	u16 efuse_addr = 0;
	u8 hoffset = 0, hworden = 0;
	u8 i;
	u8 max_section = 0;
	s32 ret;

	if (data == NULL)
		return false;

	EFUSE_GetEfuseDefinition23a(padapter, EFUSE_WIFI, TYPE_EFUSE_MAX_SECTION,
				 &max_section);
	if (offset > max_section) {
		DBG_8723A("%s: Packet offset(%d) is illegal(>%d)!\n",
			  __func__, offset, max_section);
		return false;
	}

	memset(data, 0xFF, PGPKT_DATA_SIZE);
	ret = true;

	/*  */
	/*  <Roger_TODO> Efuse has been pre-programmed dummy 5Bytes at the
	    end of Efuse by CP. */
	/*  Skip dummy parts to prevent unexpected data read from Efuse. */
	/*  By pass right now. 2009.02.19. */
	/*  */
	while (AVAILABLE_EFUSE_ADDR(efuse_addr)) {
		if (efuse_OneByteRead23a(padapter, efuse_addr++, &efuse_data) ==
		    false) {
			ret = false;
			break;
		}

		if (efuse_data == 0xFF)
			break;

		if (EXT_HEADER(efuse_data)) {
			hoffset = GET_HDR_OFFSET_2_0(efuse_data);
			efuse_OneByteRead23a(padapter, efuse_addr++, &efuse_data);
			if (ALL_WORDS_DISABLED(efuse_data)) {
				DBG_8723A("%s: Error!! All words disabled!\n",
					  __func__);
				continue;
			}

			hoffset |= ((efuse_data & 0xF0) >> 1);
			hworden = efuse_data & 0x0F;
		} else {
			hoffset = (efuse_data >> 4) & 0x0F;
			hworden = efuse_data & 0x0F;
		}

		if (hoffset == offset) {
			for (i = 0; i < EFUSE_MAX_WORD_UNIT; i++) {
				/* Check word enable condition in the section */
				if (!(hworden & (0x01 << i))) {
					ReadEFuseByte23a(padapter, efuse_addr++,
						      &efuse_data);
					data[i * 2] = efuse_data;

					ReadEFuseByte23a(padapter, efuse_addr++,
						      &efuse_data);
					data[(i * 2) + 1] = efuse_data;
				}
			}
		} else {
			word_cnts = Efuse_CalculateWordCnts23a(hworden);
			efuse_addr += word_cnts * 2;
		}
	}

	return ret;
}

static u8
hal_EfusePgCheckAvailableAddr(struct rtw_adapter *pAdapter, u8 efuseType)
{
	u16 max_available = 0;
	u16 current_size;

	EFUSE_GetEfuseDefinition23a(pAdapter, efuseType,
				 TYPE_AVAILABLE_EFUSE_BYTES_TOTAL,
				 &max_available);

	current_size = Efuse_GetCurrentSize23a(pAdapter, efuseType);
	if (current_size >= max_available) {
		DBG_8723A("%s: Error!! current_size(%d)>max_available(%d)\n",
			  __func__, current_size, max_available);
		return false;
	}
	return true;
}

static void
hal_EfuseConstructPGPkt(u8 offset, u8 word_en, u8 *pData,
			struct pg_pkt_struct *pTargetPkt)
{
	memset(pTargetPkt->data, 0xFF, PGPKT_DATA_SIZE);
	pTargetPkt->offset = offset;
	pTargetPkt->word_en = word_en;
	efuse_WordEnableDataRead23a(word_en, pData, pTargetPkt->data);
	pTargetPkt->word_cnts = Efuse_CalculateWordCnts23a(pTargetPkt->word_en);
}

static u8
hal_EfusePartialWriteCheck(struct rtw_adapter *padapter, u8 efuseType,
			   u16 *pAddr, struct pg_pkt_struct *pTargetPkt)
{
	u8 bRet = false;
	u16 startAddr = 0, efuse_max_available_len = 0, efuse_max = 0;
	u8 efuse_data = 0;

	EFUSE_GetEfuseDefinition23a(padapter, efuseType,
				 TYPE_AVAILABLE_EFUSE_BYTES_TOTAL,
				 &efuse_max_available_len);
	EFUSE_GetEfuseDefinition23a(padapter, efuseType,
				 TYPE_EFUSE_CONTENT_LEN_BANK, &efuse_max);

	if (efuseType == EFUSE_WIFI) {
		rtw23a_hal_get_hwreg(padapter, HW_VAR_EFUSE_BYTES,
				  (u8 *) &startAddr);
	} else {
		rtw23a_hal_get_hwreg(padapter, HW_VAR_EFUSE_BT_BYTES,
				  (u8 *) &startAddr);
	}
	startAddr %= efuse_max;

	while (1) {
		if (startAddr >= efuse_max_available_len) {
			bRet = false;
			DBG_8723A("%s: startAddr(%d) >= efuse_max_available_"
				  "len(%d)\n", __func__, startAddr,
				  efuse_max_available_len);
			break;
		}

		if (efuse_OneByteRead23a(padapter, startAddr, &efuse_data) &&
		    (efuse_data != 0xFF)) {
			bRet = false;
			DBG_8723A("%s: Something Wrong! last bytes(%#X = 0x%02X) "
				  "is not 0xFF\n", __func__,
				  startAddr, efuse_data);
			break;
		} else {
			/*  not used header, 0xff */
			*pAddr = startAddr;
			bRet = true;
			break;
		}
	}

	return bRet;
}

static u8
hal_EfusePgPacketWrite1ByteHeader(struct rtw_adapter *pAdapter, u8 efuseType,
				  u16 *pAddr, struct pg_pkt_struct *pTargetPkt)
{
	u8 pg_header = 0, tmp_header = 0;
	u16 efuse_addr = *pAddr;
	u8 repeatcnt = 0;

	pg_header = ((pTargetPkt->offset << 4) & 0xf0) | pTargetPkt->word_en;

	do {
		efuse_OneByteWrite23a(pAdapter, efuse_addr, pg_header);
		efuse_OneByteRead23a(pAdapter, efuse_addr, &tmp_header);
		if (tmp_header != 0xFF)
			break;
		if (repeatcnt++ > EFUSE_REPEAT_THRESHOLD_) {
			DBG_8723A("%s: Repeat over limit for pg_header!!\n",
				  __func__);
			return false;
		}
	} while (1);

	if (tmp_header != pg_header) {
		DBG_8723A(KERN_ERR "%s: PG Header Fail!!(pg = 0x%02X "
			  "read = 0x%02X)\n", __func__,
			  pg_header, tmp_header);
		return false;
	}

	*pAddr = efuse_addr;

	return true;
}

static u8
hal_EfusePgPacketWrite2ByteHeader(struct rtw_adapter *padapter, u8 efuseType,
				  u16 *pAddr, struct pg_pkt_struct *pTargetPkt)
{
	u16 efuse_addr, efuse_max_available_len = 0;
	u8 pg_header = 0, tmp_header = 0;
	u8 repeatcnt = 0;

	EFUSE_GetEfuseDefinition23a(padapter, efuseType,
				 TYPE_AVAILABLE_EFUSE_BYTES_BANK,
				 &efuse_max_available_len);

	efuse_addr = *pAddr;
	if (efuse_addr >= efuse_max_available_len) {
		DBG_8723A("%s: addr(%d) over avaliable(%d)!!\n", __func__,
			  efuse_addr, efuse_max_available_len);
		return false;
	}

	pg_header = ((pTargetPkt->offset & 0x07) << 5) | 0x0F;

	do {
		efuse_OneByteWrite23a(padapter, efuse_addr, pg_header);
		efuse_OneByteRead23a(padapter, efuse_addr, &tmp_header);
		if (tmp_header != 0xFF)
			break;
		if (repeatcnt++ > EFUSE_REPEAT_THRESHOLD_) {
			DBG_8723A("%s: Repeat over limit for pg_header!!\n",
				  __func__);
			return false;
		}
	} while (1);

	if (tmp_header != pg_header) {
		DBG_8723A(KERN_ERR
			  "%s: PG Header Fail!!(pg = 0x%02X read = 0x%02X)\n",
			  __func__, pg_header, tmp_header);
		return false;
	}

	/*  to write ext_header */
	efuse_addr++;
	pg_header = ((pTargetPkt->offset & 0x78) << 1) | pTargetPkt->word_en;

	do {
		efuse_OneByteWrite23a(padapter, efuse_addr, pg_header);
		efuse_OneByteRead23a(padapter, efuse_addr, &tmp_header);
		if (tmp_header != 0xFF)
			break;
		if (repeatcnt++ > EFUSE_REPEAT_THRESHOLD_) {
			DBG_8723A("%s: Repeat over limit for ext_header!!\n",
				  __func__);
			return false;
		}
	} while (1);

	if (tmp_header != pg_header) {	/* offset PG fail */
		DBG_8723A(KERN_ERR
			  "%s: PG EXT Header Fail!!(pg = 0x%02X read = 0x%02X)\n",
			  __func__, pg_header, tmp_header);
		return false;
	}

	*pAddr = efuse_addr;

	return true;
}

static u8
hal_EfusePgPacketWriteHeader(struct rtw_adapter *padapter, u8 efuseType,
			     u16 *pAddr, struct pg_pkt_struct *pTargetPkt)
{
	u8 bRet = false;

	if (pTargetPkt->offset >= EFUSE_MAX_SECTION_BASE) {
		bRet = hal_EfusePgPacketWrite2ByteHeader(padapter, efuseType,
							 pAddr, pTargetPkt);
	} else {
		bRet = hal_EfusePgPacketWrite1ByteHeader(padapter, efuseType,
							 pAddr, pTargetPkt);
	}

	return bRet;
}

static u8
hal_EfusePgPacketWriteData(struct rtw_adapter *pAdapter, u8 efuseType,
			   u16 *pAddr, struct pg_pkt_struct *pTargetPkt)
{
	u16 efuse_addr;
	u8 badworden;

	efuse_addr = *pAddr;
	badworden =
	    Efuse_WordEnableDataWrite23a(pAdapter, efuse_addr + 1,
				      pTargetPkt->word_en, pTargetPkt->data);
	if (badworden != 0x0F) {
		DBG_8723A("%s: Fail!!\n", __func__);
		return false;
	}

	return true;
}

static s32
Hal_EfusePgPacketWrite(struct rtw_adapter *padapter,
		       u8 offset, u8 word_en, u8 *pData)
{
	struct pg_pkt_struct targetPkt;
	u16 startAddr = 0;
	u8 efuseType = EFUSE_WIFI;

	if (!hal_EfusePgCheckAvailableAddr(padapter, efuseType))
		return false;

	hal_EfuseConstructPGPkt(offset, word_en, pData, &targetPkt);

	if (!hal_EfusePartialWriteCheck(padapter, efuseType,
					&startAddr, &targetPkt))
		return false;

	if (!hal_EfusePgPacketWriteHeader(padapter, efuseType,
					  &startAddr, &targetPkt))
		return false;

	if (!hal_EfusePgPacketWriteData(padapter, efuseType,
					&startAddr, &targetPkt))
		return false;

	return true;
}

static bool
Hal_EfusePgPacketWrite_BT(struct rtw_adapter *pAdapter,
			  u8 offset, u8 word_en, u8 *pData)
{
	struct pg_pkt_struct targetPkt;
	u16 startAddr = 0;
	u8 efuseType = EFUSE_BT;

	if (!hal_EfusePgCheckAvailableAddr(pAdapter, efuseType))
		return false;

	hal_EfuseConstructPGPkt(offset, word_en, pData, &targetPkt);

	if (!hal_EfusePartialWriteCheck(pAdapter, efuseType,
					&startAddr, &targetPkt))
		return false;

	if (!hal_EfusePgPacketWriteHeader(pAdapter, efuseType,
					  &startAddr, &targetPkt))
		return false;

	if (!hal_EfusePgPacketWriteData(pAdapter, efuseType,
					&startAddr, &targetPkt))
		return false;

	return true;
}

static struct hal_version ReadChipVersion8723A(struct rtw_adapter *padapter)
{
	u32 value32;
	struct hal_version ChipVersion;
	struct hal_data_8723a *pHalData;

	pHalData = GET_HAL_DATA(padapter);

	value32 = rtw_read32(padapter, REG_SYS_CFG);
	ChipVersion.ICType = CHIP_8723A;
	ChipVersion.ChipType = ((value32 & RTL_ID) ? TEST_CHIP : NORMAL_CHIP);
	ChipVersion.RFType = RF_TYPE_1T1R;
	ChipVersion.VendorType =
		((value32 & VENDOR_ID) ? CHIP_VENDOR_UMC : CHIP_VENDOR_TSMC);
	ChipVersion.CUTVersion = (value32 & CHIP_VER_RTL_MASK) >> CHIP_VER_RTL_SHIFT;	/*  IC version (CUT) */

	/*  For regulator mode. by tynli. 2011.01.14 */
	pHalData->RegulatorMode = ((value32 & SPS_SEL) ?
				   RT_LDO_REGULATOR : RT_SWITCHING_REGULATOR);

	value32 = rtw_read32(padapter, REG_GPIO_OUTSTS);
	/*  ROM code version. */
	ChipVersion.ROMVer = ((value32 & RF_RL_ID) >> 20);

	/*  For multi-function consideration. Added by Roger, 2010.10.06. */
	pHalData->MultiFunc = RT_MULTI_FUNC_NONE;
	value32 = rtw_read32(padapter, REG_MULTI_FUNC_CTRL);
	pHalData->MultiFunc |=
		((value32 & WL_FUNC_EN) ? RT_MULTI_FUNC_WIFI : 0);
	pHalData->MultiFunc |= ((value32 & BT_FUNC_EN) ? RT_MULTI_FUNC_BT : 0);
	pHalData->MultiFunc |=
		((value32 & GPS_FUNC_EN) ? RT_MULTI_FUNC_GPS : 0);
	pHalData->PolarityCtl =
		((value32 & WL_HWPDN_SL) ? RT_POLARITY_HIGH_ACT :
		 RT_POLARITY_LOW_ACT);
	dump_chip_info23a(ChipVersion);
	pHalData->VersionID = ChipVersion;

	if (IS_1T2R(ChipVersion))
		pHalData->rf_type = RF_1T2R;
	else if (IS_2T2R(ChipVersion))
		pHalData->rf_type = RF_2T2R;
	else
		pHalData->rf_type = RF_1T1R;

	MSG_8723A("RF_Type is %x!!\n", pHalData->rf_type);

	return ChipVersion;
}

static void rtl8723a_read_chip_version(struct rtw_adapter *padapter)
{
	ReadChipVersion8723A(padapter);
}

/*  */
/*  */
/*  20100209 Joseph: */
/*  This function is used only for 92C to set REG_BCN_CTRL(0x550) register. */
/*  We just reserve the value of the register in variable
    pHalData->RegBcnCtrlVal and then operate */
/*  the value of the register via atomic operation. */
/*  This prevents from race condition when setting this register. */
/*  The value of pHalData->RegBcnCtrlVal is initialized in
    HwConfigureRTL8192CE() function. */
/*  */
void SetBcnCtrlReg23a(struct rtw_adapter *padapter, u8 SetBits, u8 ClearBits)
{
	struct hal_data_8723a *pHalData;
	u32 addr;
	u8 *pRegBcnCtrlVal;

	pHalData = GET_HAL_DATA(padapter);
	pRegBcnCtrlVal = (u8 *)&pHalData->RegBcnCtrlVal;

	addr = REG_BCN_CTRL;

	*pRegBcnCtrlVal = rtw_read8(padapter, addr);
	*pRegBcnCtrlVal |= SetBits;
	*pRegBcnCtrlVal &= ~ClearBits;

	rtw_write8(padapter, addr, *pRegBcnCtrlVal);
}

void rtl8723a_InitBeaconParameters(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	rtw_write16(padapter, REG_BCN_CTRL, 0x1010);
	pHalData->RegBcnCtrlVal = 0x1010;

	/*  TODO: Remove these magic number */
	rtw_write16(padapter, REG_TBTT_PROHIBIT, 0x6404);	/*  ms */
	/*  Firmware will control REG_DRVERLYINT when power saving is enable, */
	/*  so don't set this register on STA mode. */
	if (check_fwstate(&padapter->mlmepriv, WIFI_STATION_STATE) == false)
		rtw_write8(padapter, REG_DRVERLYINT, DRIVER_EARLY_INT_TIME);
	/*  2ms */
	rtw_write8(padapter, REG_BCNDMATIM, BCN_DMA_ATIME_INT_TIME);

	/*  Suggested by designer timchen. Change beacon AIFS to the
	    largest number beacause test chip does not contension before
	    sending beacon. by tynli. 2009.11.03 */
	rtw_write16(padapter, REG_BCNTCFG, 0x660F);
}

static void ResumeTxBeacon(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	/*  2010.03.01. Marked by tynli. No need to call workitem beacause
	    we record the value */
	/*  which should be read from register to a global variable. */

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("+ResumeTxBeacon\n"));

	pHalData->RegFwHwTxQCtrl |= BIT(6);
	rtw_write8(padapter, REG_FWHW_TXQ_CTRL + 2, pHalData->RegFwHwTxQCtrl);
	rtw_write8(padapter, REG_TBTT_PROHIBIT + 1, 0xff);
	pHalData->RegReg542 |= BIT(0);
	rtw_write8(padapter, REG_TBTT_PROHIBIT + 2, pHalData->RegReg542);
}

static void StopTxBeacon(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	/*  2010.03.01. Marked by tynli. No need to call workitem beacause
	    we record the value */
	/*  which should be read from register to a global variable. */

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("+StopTxBeacon\n"));

	pHalData->RegFwHwTxQCtrl &= ~BIT(6);
	rtw_write8(padapter, REG_FWHW_TXQ_CTRL + 2, pHalData->RegFwHwTxQCtrl);
	rtw_write8(padapter, REG_TBTT_PROHIBIT + 1, 0x64);
	pHalData->RegReg542 &= ~BIT(0);
	rtw_write8(padapter, REG_TBTT_PROHIBIT + 2, pHalData->RegReg542);

	CheckFwRsvdPageContent23a(padapter); /*  2010.06.23. Added by tynli. */
}

static void _BeaconFunctionEnable(struct rtw_adapter *padapter, u8 Enable,
				  u8 Linked)
{
	SetBcnCtrlReg23a(padapter, DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_BCNQ_SUB,
		      0);
	rtw_write8(padapter, REG_RD_CTRL + 1, 0x6F);
}

static void rtl8723a_SetBeaconRelatedRegisters(struct rtw_adapter *padapter)
{
	u32 value32;
	struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;

	/* reset TSF, enable update TSF, correcting TSF On Beacon */

	/* REG_BCN_INTERVAL */
	/* REG_BCNDMATIM */
	/* REG_ATIMWND */
	/* REG_TBTT_PROHIBIT */
	/* REG_DRVERLYINT */
	/* REG_BCN_MAX_ERR */
	/* REG_BCNTCFG (0x510) */
	/* REG_DUAL_TSF_RST */
	/* REG_BCN_CTRL (0x550) */

	/*  */
	/*  ATIM window */
	/*  */
	rtw_write16(padapter, REG_ATIMWND, 2);

	/*  */
	/*  Beacon interval (in unit of TU). */
	/*  */
	rtw_write16(padapter, REG_BCN_INTERVAL, pmlmeinfo->bcn_interval);

	rtl8723a_InitBeaconParameters(padapter);

	rtw_write8(padapter, REG_SLOT, 0x09);

	/*  */
	/*  Reset TSF Timer to zero, added by Roger. 2008.06.24 */
	/*  */
	value32 = rtw_read32(padapter, REG_TCR);
	value32 &= ~TSFRST;
	rtw_write32(padapter, REG_TCR, value32);

	value32 |= TSFRST;
	rtw_write32(padapter, REG_TCR, value32);

	/*  NOTE: Fix test chip's bug (about contention windows's randomness) */
	if (check_fwstate(&padapter->mlmepriv, WIFI_ADHOC_STATE |
			  WIFI_ADHOC_MASTER_STATE | WIFI_AP_STATE) == true) {
		rtw_write8(padapter, REG_RXTSF_OFFSET_CCK, 0x50);
		rtw_write8(padapter, REG_RXTSF_OFFSET_OFDM, 0x50);
	}

	_BeaconFunctionEnable(padapter, true, true);

	ResumeTxBeacon(padapter);
	SetBcnCtrlReg23a(padapter, DIS_BCNQ_SUB, 0);
}

static void rtl8723a_GetHalODMVar(struct rtw_adapter *Adapter,
				  enum hal_odm_variable eVariable,
				  void *pValue1, bool bSet)
{
	switch (eVariable) {
	case HAL_ODM_STA_INFO:
		break;
	default:
		break;
	}
}

static void rtl8723a_SetHalODMVar(struct rtw_adapter *Adapter,
				  enum hal_odm_variable eVariable,
				  void *pValue1, bool bSet)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(Adapter);
	struct dm_odm_t *podmpriv = &pHalData->odmpriv;
	switch (eVariable) {
	case HAL_ODM_STA_INFO:
	{
		struct sta_info *psta = (struct sta_info *)pValue1;

		if (bSet) {
			DBG_8723A("Set STA_(%d) info\n", psta->mac_id);
			ODM_CmnInfoPtrArrayHook23a(podmpriv,
						ODM_CMNINFO_STA_STATUS,
						psta->mac_id, psta);
		} else {
			DBG_8723A("Clean STA_(%d) info\n", psta->mac_id);
				ODM_CmnInfoPtrArrayHook23a(podmpriv,
							ODM_CMNINFO_STA_STATUS,
							psta->mac_id, NULL);
		}
	}
		break;
	case HAL_ODM_P2P_STATE:
		ODM_CmnInfoUpdate23a(podmpriv, ODM_CMNINFO_WIFI_DIRECT, bSet);
		break;
	case HAL_ODM_WIFI_DISPLAY_STATE:
		ODM_CmnInfoUpdate23a(podmpriv, ODM_CMNINFO_WIFI_DISPLAY, bSet);
		break;
	default:
		break;
	}
}

static void hal_notch_filter_8723a(struct rtw_adapter *adapter, bool enable)
{
	if (enable) {
		DBG_8723A("Enable notch filter\n");
		rtw_write8(adapter, rOFDM0_RxDSP + 1,
			   rtw_read8(adapter, rOFDM0_RxDSP + 1) | BIT1);
	} else {
		DBG_8723A("Disable notch filter\n");
		rtw_write8(adapter, rOFDM0_RxDSP + 1,
			   rtw_read8(adapter, rOFDM0_RxDSP + 1) & ~BIT1);
	}
}

s32 c2h_id_filter_ccx_8723a(u8 id)
{
	s32 ret = false;
	if (id == C2H_CCX_TX_RPT)
		ret = true;

	return ret;
}

static s32 c2h_handler_8723a(struct rtw_adapter *padapter,
			     struct c2h_evt_hdr *c2h_evt)
{
	s32 ret = _SUCCESS;
	u8 i = 0;

	if (c2h_evt == NULL) {
		DBG_8723A("%s c2h_evt is NULL\n", __func__);
		ret = _FAIL;
		goto exit;
	}

	switch (c2h_evt->id) {
	case C2H_DBG:
		RT_TRACE(_module_hal_init_c_, _drv_info_,
			 ("C2HCommandHandler: %s\n", c2h_evt->payload));
		break;

	case C2H_CCX_TX_RPT:
		handle_txrpt_ccx_8723a(padapter, c2h_evt->payload);
		break;
	case C2H_EXT_RA_RPT:
		break;
	case C2H_HW_INFO_EXCH:
		RT_TRACE(_module_hal_init_c_, _drv_info_,
			 ("[BT], C2H_HW_INFO_EXCH\n"));
		for (i = 0; i < c2h_evt->plen; i++) {
			RT_TRACE(_module_hal_init_c_, _drv_info_,
				 ("[BT], tmpBuf[%d]= 0x%x\n", i,
				  c2h_evt->payload[i]));
		}
		break;

	case C2H_C2H_H2C_TEST:
		RT_TRACE(_module_hal_init_c_, _drv_info_,
			 ("[BT], C2H_H2C_TEST\n"));
		RT_TRACE(_module_hal_init_c_, _drv_info_,
			 ("[BT], tmpBuf[0]/[1]/[2]/[3]/[4]= 0x%x/ 0x%x/ "
			  "0x%x/ 0x%x/ 0x%x\n", c2h_evt->payload[0],
			  c2h_evt->payload[1], c2h_evt->payload[2],
			  c2h_evt->payload[3], c2h_evt->payload[4]));
		break;

#ifdef CONFIG_8723AU_BT_COEXIST
	case C2H_BT_INFO:
		DBG_8723A("%s ,  Got  C2H_BT_INFO \n", __func__);
		BT_FwC2hBtInfo(padapter, c2h_evt->payload, c2h_evt->plen);
		break;
#endif

	default:
		ret = _FAIL;
		break;
	}

exit:
	return ret;
}

void rtl8723a_set_hal_ops(struct hal_ops *pHalFunc)
{
	pHalFunc->free_hal_data = &rtl8723a_free_hal_data;

	pHalFunc->dm_init = &rtl8723a_init_dm_priv;
	pHalFunc->dm_deinit = &rtl8723a_deinit_dm_priv;

	pHalFunc->read_chip_version = &rtl8723a_read_chip_version;

	pHalFunc->set_bwmode_handler = &PHY_SetBWMode23a8723A;
	pHalFunc->set_channel_handler = &PHY_SwChnl8723A;

	pHalFunc->hal_dm_watchdog = &rtl8723a_HalDmWatchDog;

	pHalFunc->SetBeaconRelatedRegistersHandler =
		&rtl8723a_SetBeaconRelatedRegisters;

	pHalFunc->Add_RateATid = &rtl8723a_add_rateatid;
	pHalFunc->run_thread = &rtl8723a_start_thread;
	pHalFunc->cancel_thread = &rtl8723a_stop_thread;

	pHalFunc->read_bbreg = &PHY_QueryBBReg;
	pHalFunc->write_bbreg = &PHY_SetBBReg;
	pHalFunc->read_rfreg = &PHY_QueryRFReg;
	pHalFunc->write_rfreg = &PHY_SetRFReg;

	/*  Efuse related function */
	pHalFunc->EfusePowerSwitch = &Hal_EfusePowerSwitch;
	pHalFunc->ReadEFuse = &Hal_ReadEFuse;
	pHalFunc->EFUSEGetEfuseDefinition = &Hal_GetEfuseDefinition;
	pHalFunc->EfuseGetCurrentSize = &Hal_EfuseGetCurrentSize;
	pHalFunc->Efuse_PgPacketRead23a = &Hal_EfusePgPacketRead;
	pHalFunc->Efuse_PgPacketWrite23a = &Hal_EfusePgPacketWrite;
	pHalFunc->Efuse_WordEnableDataWrite23a = &Hal_EfuseWordEnableDataWrite;
	pHalFunc->Efuse_PgPacketWrite23a_BT = &Hal_EfusePgPacketWrite_BT;

	pHalFunc->sreset_init_value23a = &sreset_init_value23a;
	pHalFunc->sreset_reset_value23a = &sreset_reset_value23a;
	pHalFunc->silentreset = &sreset_reset;
	pHalFunc->sreset_xmit_status_check = &rtl8723a_sreset_xmit_status_check;
	pHalFunc->sreset_linked_status_check =
		&rtl8723a_sreset_linked_status_check;
	pHalFunc->sreset_get_wifi_status23a = &sreset_get_wifi_status23a;
	pHalFunc->sreset_inprogress = &sreset_inprogress;
	pHalFunc->GetHalODMVarHandler = &rtl8723a_GetHalODMVar;
	pHalFunc->SetHalODMVarHandler = &rtl8723a_SetHalODMVar;

	pHalFunc->hal_notch_filter = &hal_notch_filter_8723a;

	pHalFunc->c2h_handler = c2h_handler_8723a;
	pHalFunc->c2h_id_filter_ccx = c2h_id_filter_ccx_8723a;
}

void rtl8723a_InitAntenna_Selection(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData;
	u8 val;

	pHalData = GET_HAL_DATA(padapter);

	val = rtw_read8(padapter, REG_LEDCFG2);
	/*  Let 8051 take control antenna settting */
	val |= BIT(7);		/*  DPDT_SEL_EN, 0x4C[23] */
	rtw_write8(padapter, REG_LEDCFG2, val);
}

void rtl8723a_CheckAntenna_Selection(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData;
	u8 val;

	pHalData = GET_HAL_DATA(padapter);

	val = rtw_read8(padapter, REG_LEDCFG2);
	/*  Let 8051 take control antenna settting */
	if (!(val & BIT(7))) {
		val |= BIT(7);	/*  DPDT_SEL_EN, 0x4C[23] */
		rtw_write8(padapter, REG_LEDCFG2, val);
	}
}

void rtl8723a_DeinitAntenna_Selection(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData;
	u8 val;

	pHalData = GET_HAL_DATA(padapter);
	val = rtw_read8(padapter, REG_LEDCFG2);
	/*  Let 8051 take control antenna settting */
	val &= ~BIT(7);		/*  DPDT_SEL_EN, clear 0x4C[23] */
	rtw_write8(padapter, REG_LEDCFG2, val);
}

void rtl8723a_init_default_value(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData;
	struct dm_priv *pdmpriv;
	u8 i;

	pHalData = GET_HAL_DATA(padapter);
	pdmpriv = &pHalData->dmpriv;

	/*  init default value */
	pHalData->fw_ractrl = false;
	pHalData->bIQKInitialized = false;
	if (!padapter->pwrctrlpriv.bkeepfwalive)
		pHalData->LastHMEBoxNum = 0;

	pHalData->bIQKInitialized = false;

	/*  init dm default value */
	pdmpriv->TM_Trigger = 0;	/* for IQK */
/*	pdmpriv->binitialized = false; */
/*	pdmpriv->prv_traffic_idx = 3; */
/*	pdmpriv->initialize = 0; */

	pdmpriv->ThermalValue_HP_index = 0;
	for (i = 0; i < HP_THERMAL_NUM; i++)
		pdmpriv->ThermalValue_HP[i] = 0;

	/*  init Efuse variables */
	pHalData->EfuseUsedBytes = 0;
	pHalData->BTEfuseUsedBytes = 0;
}

u8 GetEEPROMSize8723A(struct rtw_adapter *padapter)
{
	u8 size = 0;
	u32 cr;

	cr = rtw_read16(padapter, REG_9346CR);
	/*  6: EEPROM used is 93C46, 4: boot from E-Fuse. */
	size = (cr & BOOT_FROM_EEPROM) ? 6 : 4;

	MSG_8723A("EEPROM type is %s\n", size == 4 ? "E-FUSE" : "93C46");

	return size;
}

/*  */
/*  */
/*  LLT R/W/Init function */
/*  */
/*  */
static s32 _LLTWrite(struct rtw_adapter *padapter, u32 address, u32 data)
{
	s32 status = _SUCCESS;
	s32 count = 0;
	u32 value = _LLT_INIT_ADDR(address) | _LLT_INIT_DATA(data) |
		    _LLT_OP(_LLT_WRITE_ACCESS);
	u16 LLTReg = REG_LLT_INIT;

	rtw_write32(padapter, LLTReg, value);

	/* polling */
	do {
		value = rtw_read32(padapter, LLTReg);
		if (_LLT_NO_ACTIVE == _LLT_OP_VALUE(value)) {
			break;
		}

		if (count > POLLING_LLT_THRESHOLD) {
			RT_TRACE(_module_hal_init_c_, _drv_err_,
				 ("Failed to polling write LLT done at "
				  "address %d!\n", address));
			status = _FAIL;
			break;
		}
	} while (count++);

	return status;
}

s32 InitLLTTable23a(struct rtw_adapter *padapter, u32 boundary)
{
	s32 status = _SUCCESS;
	u32 i;
	u32 txpktbuf_bndy = boundary;
	u32 Last_Entry_Of_TxPktBuf = LAST_ENTRY_OF_TX_PKT_BUFFER;

	for (i = 0; i < (txpktbuf_bndy - 1); i++) {
		status = _LLTWrite(padapter, i, i + 1);
		if (_SUCCESS != status) {
			return status;
		}
	}

	/*  end of list */
	status = _LLTWrite(padapter, (txpktbuf_bndy - 1), 0xFF);
	if (_SUCCESS != status) {
		return status;
	}

	/*  Make the other pages as ring buffer */
	/*  This ring buffer is used as beacon buffer if we config this
	    MAC as two MAC transfer. */
	/*  Otherwise used as local loopback buffer. */
	for (i = txpktbuf_bndy; i < Last_Entry_Of_TxPktBuf; i++) {
		status = _LLTWrite(padapter, i, (i + 1));
		if (_SUCCESS != status) {
			return status;
		}
	}

	/*  Let last entry point to the start entry of ring buffer */
	status = _LLTWrite(padapter, Last_Entry_Of_TxPktBuf, txpktbuf_bndy);
	if (_SUCCESS != status) {
		return status;
	}

	return status;
}

static void _DisableGPIO(struct rtw_adapter *padapter)
{
/***************************************
j. GPIO_PIN_CTRL 0x44[31:0]= 0x000
k.Value = GPIO_PIN_CTRL[7:0]
l. GPIO_PIN_CTRL 0x44[31:0] = 0x00FF0000 | (value <<8); write external PIN level
m. GPIO_MUXCFG 0x42 [15:0] = 0x0780
n. LEDCFG 0x4C[15:0] = 0x8080
***************************************/
	u32 value32;
	u32 u4bTmp;

	/* 1. Disable GPIO[7:0] */
	rtw_write16(padapter, REG_GPIO_PIN_CTRL + 2, 0x0000);
	value32 = rtw_read32(padapter, REG_GPIO_PIN_CTRL) & 0xFFFF00FF;
	u4bTmp = value32 & 0x000000FF;
	value32 |= ((u4bTmp << 8) | 0x00FF0000);
	rtw_write32(padapter, REG_GPIO_PIN_CTRL, value32);

	/*  */
	/*  <Roger_Notes> For RTL8723u multi-function configuration which
	    was autoload from Efuse offset 0x0a and 0x0b, */
	/*  WLAN HW GPIO[9], GPS HW GPIO[10] and BT HW GPIO[11]. */
	/*  Added by Roger, 2010.10.07. */
	/*  */
	/* 2. Disable GPIO[8] and GPIO[12] */

	/*  Configure all pins as input mode. */
	rtw_write16(padapter, REG_GPIO_IO_SEL_2, 0x0000);
	value32 = rtw_read32(padapter, REG_GPIO_PIN_CTRL_2) & 0xFFFF001F;
	u4bTmp = value32 & 0x0000001F;
	/*  Set pin 8, 10, 11 and pin 12 to output mode. */
	value32 |= ((u4bTmp << 8) | 0x001D0000);
	rtw_write32(padapter, REG_GPIO_PIN_CTRL_2, value32);

	/* 3. Disable LED0 & 1 */
	rtw_write16(padapter, REG_LEDCFG0, 0x8080);
}				/* end of _DisableGPIO() */

static void _DisableRFAFEAndResetBB8192C(struct rtw_adapter *padapter)
{
/**************************************
a.	TXPAUSE 0x522[7:0] = 0xFF		Pause MAC TX queue
b.	RF path 0 offset 0x00 = 0x00		disable RF
c.	APSD_CTRL 0x600[7:0] = 0x40
d.	SYS_FUNC_EN 0x02[7:0] = 0x16		reset BB state machine
e.	SYS_FUNC_EN 0x02[7:0] = 0x14		reset BB state machine
***************************************/
	u8 eRFPath = 0, value8 = 0;

	rtw_write8(padapter, REG_TXPAUSE, 0xFF);

	PHY_SetRFReg(padapter, (enum RF_RADIO_PATH) eRFPath, 0x0, bMaskByte0, 0x0);

	value8 |= APSDOFF;
	rtw_write8(padapter, REG_APSD_CTRL, value8);	/* 0x40 */

	/*  Set BB reset at first */
	value8 = 0;
	value8 |= (FEN_USBD | FEN_USBA | FEN_BB_GLB_RSTn);
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8);	/* 0x16 */

	/*  Set global reset. */
	value8 &= ~FEN_BB_GLB_RSTn;
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8);	/* 0x14 */

	/*  2010/08/12 MH We need to set BB/GLBAL reset to save power
	    for SS mode. */

/*	RT_TRACE(COMP_INIT, DBG_LOUD, ("======> RF off and reset BB.\n")); */
}

static void _DisableRFAFEAndResetBB(struct rtw_adapter *padapter)
{
	_DisableRFAFEAndResetBB8192C(padapter);
}

static void _ResetDigitalProcedure1_92C(struct rtw_adapter *padapter,
					bool bWithoutHWSM)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	if (IS_FW_81xxC(padapter) && (pHalData->FirmwareVersion <= 0x20)) {
	/*****************************
	f.	MCUFWDL 0x80[7:0]= 0		reset MCU ready status
	g.	SYS_FUNC_EN 0x02[10]= 0		reset MCU register, (8051 reset)
	h.	SYS_FUNC_EN 0x02[15-12]= 5	reset MAC register, DCORE
	i.     SYS_FUNC_EN 0x02[10]= 1		enable MCU register,
						(8051 enable)
	******************************/
		u16 valu16 = 0;
		rtw_write8(padapter, REG_MCUFWDL, 0);

		valu16 = rtw_read16(padapter, REG_SYS_FUNC_EN);
		/* reset MCU , 8051 */
		rtw_write16(padapter, REG_SYS_FUNC_EN, (valu16 & (~FEN_CPUEN)));

		valu16 = rtw_read16(padapter, REG_SYS_FUNC_EN) & 0x0FFF;
		rtw_write16(padapter, REG_SYS_FUNC_EN,
			    (valu16 | (FEN_HWPDN | FEN_ELDR)));	/* reset MAC */

		valu16 = rtw_read16(padapter, REG_SYS_FUNC_EN);
		/* enable MCU , 8051 */
		rtw_write16(padapter, REG_SYS_FUNC_EN, (valu16 | FEN_CPUEN));
	} else {
		u8 retry_cnts = 0;

		/*  2010/08/12 MH For USB SS, we can not stop 8051 when we
		    are trying to enter IPS/HW&SW radio off. For
		    S3/S4/S5/Disable, we can stop 8051 because */
		/*  we will init FW when power on again. */
		/* if (!pDevice->RegUsbSS) */
		/*  If we want to SS mode, we can not reset 8051. */
		if (rtw_read8(padapter, REG_MCUFWDL) & BIT1) {
			/* IF fw in RAM code, do reset */
			if (padapter->bFWReady) {
				/*  2010/08/25 MH Accordign to RD alfred's
				    suggestion, we need to disable other */
				/*  HRCV INT to influence 8051 reset. */
				rtw_write8(padapter, REG_FWIMR, 0x20);
				/*  2011/02/15 MH According to Alex's
				    suggestion, close mask to prevent
				    incorrect FW write operation. */
				rtw_write8(padapter, REG_FTIMR, 0x00);
				rtw_write8(padapter, REG_FSIMR, 0x00);

				/* 8051 reset by self */
				rtw_write8(padapter, REG_HMETFR + 3, 0x20);

				while ((retry_cnts++ < 100) &&
				       (FEN_CPUEN &
					rtw_read16(padapter, REG_SYS_FUNC_EN))) {
					udelay(50);	/* us */
				}

				if (retry_cnts >= 100) {
					/* Reset MAC and Enable 8051 */
					rtw_write8(padapter,
						   REG_SYS_FUNC_EN + 1, 0x50);
					mdelay(10);
				}
			}
		}
		/* Reset MAC and Enable 8051 */
		rtw_write8(padapter, REG_SYS_FUNC_EN + 1, 0x54);
		rtw_write8(padapter, REG_MCUFWDL, 0);
	}

	if (bWithoutHWSM) {
	/*****************************
		Without HW auto state machine
	g.	SYS_CLKR 0x08[15:0] = 0x30A3		disable MAC clock
	h.	AFE_PLL_CTRL 0x28[7:0] = 0x80		disable AFE PLL
	i.	AFE_XTAL_CTRL 0x24[15:0] = 0x880F	gated AFE DIG_CLOCK
	j.	SYS_ISO_CTRL 0x00[7:0] = 0xF9		isolated digital to PON
	******************************/
		/* modify to 0x70A3 by Scott. */
		rtw_write16(padapter, REG_SYS_CLKR, 0x70A3);
		rtw_write8(padapter, REG_AFE_PLL_CTRL, 0x80);
		rtw_write16(padapter, REG_AFE_XTAL_CTRL, 0x880F);
		rtw_write8(padapter, REG_SYS_ISO_CTRL, 0xF9);
	} else {
		/*  Disable all RF/BB power */
		rtw_write8(padapter, REG_RF_CTRL, 0x00);
	}
}

static void _ResetDigitalProcedure1(struct rtw_adapter *padapter,
				    bool bWithoutHWSM)
{
	_ResetDigitalProcedure1_92C(padapter, bWithoutHWSM);
}

static void _ResetDigitalProcedure2(struct rtw_adapter *padapter)
{
/*****************************
k.	SYS_FUNC_EN 0x03[7:0] = 0x44		disable ELDR runction
l.	SYS_CLKR 0x08[15:0] = 0x3083		disable ELDR clock
m.	SYS_ISO_CTRL 0x01[7:0] = 0x83		isolated ELDR to PON
******************************/
	/* modify to 0x70a3 by Scott. */
	rtw_write16(padapter, REG_SYS_CLKR, 0x70a3);
	/* modify to 0x82 by Scott. */
	rtw_write8(padapter, REG_SYS_ISO_CTRL + 1, 0x82);
}

static void _DisableAnalog(struct rtw_adapter *padapter, bool bWithoutHWSM)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);
	u16 value16 = 0;
	u8 value8 = 0;

	if (bWithoutHWSM) {
	/*****************************
	n.	LDOA15_CTRL 0x20[7:0] = 0x04	disable A15 power
	o.	LDOV12D_CTRL 0x21[7:0] = 0x54	disable digital core power
	r.	When driver call disable, the ASIC will turn off remaining
		clock automatically
	******************************/

		rtw_write8(padapter, REG_LDOA15_CTRL, 0x04);
		/* rtw_write8(padapter, REG_LDOV12D_CTRL, 0x54); */

		value8 = rtw_read8(padapter, REG_LDOV12D_CTRL);
		value8 &= (~LDV12_EN);
		rtw_write8(padapter, REG_LDOV12D_CTRL, value8);
/*		RT_TRACE(COMP_INIT, DBG_LOUD,
		(" REG_LDOV12D_CTRL Reg0x21:0x%02x.\n", value8)); */
	}

	/*****************************
	h.	SPS0_CTRL 0x11[7:0] = 0x23		enter PFM mode
	i.	APS_FSMCO 0x04[15:0] = 0x4802		set USB suspend
	******************************/
	value8 = 0x23;
	if (IS_81xxC_VENDOR_UMC_B_CUT(pHalData->VersionID))
		value8 |= BIT3;

	rtw_write8(padapter, REG_SPS0_CTRL, value8);

	if (bWithoutHWSM) {
		/* value16 |= (APDM_HOST | FSM_HSUS |/PFM_ALDN); */
		/*  2010/08/31 According to Filen description, we need to
		    use HW to shut down 8051 automatically. */
		/*  Becasue suspend operatione need the asistance of 8051
		    to wait for 3ms. */
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);
	} else {
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);
	}

	rtw_write16(padapter, REG_APS_FSMCO, value16);	/* 0x4802 */

	rtw_write8(padapter, REG_RSV_CTRL, 0x0e);
}

/*  HW Auto state machine */
s32 CardDisableHWSM(struct rtw_adapter *padapter, u8 resetMCU)
{
	int rtStatus = _SUCCESS;

	if (padapter->bSurpriseRemoved) {
		return rtStatus;
	}
	/*  RF Off Sequence ==== */
	_DisableRFAFEAndResetBB(padapter);

	/*   ==== Reset digital sequence   ====== */
	_ResetDigitalProcedure1(padapter, false);

	/*   ==== Pull GPIO PIN to balance level and LED control ====== */
	_DisableGPIO(padapter);

	/*   ==== Disable analog sequence === */
	_DisableAnalog(padapter, false);

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
		 ("======> Card disable finished.\n"));

	return rtStatus;
}

/*  without HW Auto state machine */
s32 CardDisableWithoutHWSM(struct rtw_adapter *padapter)
{
	s32 rtStatus = _SUCCESS;

	/* RT_TRACE(COMP_INIT, DBG_LOUD,
	   ("======> Card Disable Without HWSM .\n")); */
	if (padapter->bSurpriseRemoved) {
		return rtStatus;
	}

	/*  RF Off Sequence ==== */
	_DisableRFAFEAndResetBB(padapter);

	/*   ==== Reset digital sequence   ====== */
	_ResetDigitalProcedure1(padapter, true);

	/*   ==== Pull GPIO PIN to balance level and LED control ====== */
	_DisableGPIO(padapter);

	/*   ==== Reset digital sequence   ====== */
	_ResetDigitalProcedure2(padapter);

	/*   ==== Disable analog sequence === */
	_DisableAnalog(padapter, true);

	/* RT_TRACE(COMP_INIT, DBG_LOUD,
	   ("<====== Card Disable Without HWSM .\n")); */
	return rtStatus;
}

void Hal_InitPGData(struct rtw_adapter *padapter, u8 *PROMContent)
{
	struct eeprom_priv *pEEPROM = GET_EEPROM_EFUSE_PRIV(padapter);

	if (false == pEEPROM->bautoload_fail_flag) {	/*  autoload OK. */
		if (!pEEPROM->EepromOrEfuse) {
			/*  Read EFUSE real map to shadow. */
			EFUSE_ShadowMapUpdate23a(padapter, EFUSE_WIFI);
			memcpy((void *)PROMContent,
			       (void *)pEEPROM->efuse_eeprom_data,
			       HWSET_MAX_SIZE);
		}
	} else {		/* autoload fail */
		RT_TRACE(_module_hci_hal_init_c_, _drv_notice_,
			 ("AutoLoad Fail reported from CR9346!!\n"));
/*		pHalData->AutoloadFailFlag = true; */
		/* update to default value 0xFF */
		if (false == pEEPROM->EepromOrEfuse)
			EFUSE_ShadowMapUpdate23a(padapter, EFUSE_WIFI);
		memcpy((void *)PROMContent, (void *)pEEPROM->efuse_eeprom_data,
		       HWSET_MAX_SIZE);
	}
}

void Hal_EfuseParseIDCode(struct rtw_adapter *padapter, u8 *hwinfo)
{
	struct eeprom_priv *pEEPROM = GET_EEPROM_EFUSE_PRIV(padapter);
/*	struct hal_data_8723a	*pHalData = GET_HAL_DATA(padapter); */
	u16 EEPROMId;

	/*  Checl 0x8129 again for making sure autoload status!! */
	EEPROMId = le16_to_cpu(*((u16 *) hwinfo));
	if (EEPROMId != RTL_EEPROM_ID) {
		DBG_8723A("EEPROM ID(%#x) is invalid!!\n", EEPROMId);
		pEEPROM->bautoload_fail_flag = true;
	} else {
		pEEPROM->bautoload_fail_flag = false;
	}

	RT_TRACE(_module_hal_init_c_, _drv_info_,
		 ("EEPROM ID = 0x%04x\n", EEPROMId));
}

static void Hal_EEValueCheck(u8 EEType, void *pInValue, void *pOutValue)
{
	switch (EEType) {
	case EETYPE_TX_PWR:
	{
		u8 *pIn, *pOut;
		pIn = (u8 *) pInValue;
		pOut = (u8 *) pOutValue;
		if (*pIn >= 0 && *pIn <= 63) {
			*pOut = *pIn;
		} else {
			RT_TRACE(_module_hci_hal_init_c_, _drv_err_,
				 ("EETYPE_TX_PWR, value =%d is invalid, set "
				  "to default = 0x%x\n",
				  *pIn, EEPROM_Default_TxPowerLevel));
			*pOut = EEPROM_Default_TxPowerLevel;
		}
	}
		break;
	default:
		break;
	}
}

static void
Hal_ReadPowerValueFromPROM_8723A(struct txpowerinfo *pwrInfo,
				 u8 *PROMContent, bool AutoLoadFail)
{
	u32 rfPath, eeAddr, group, rfPathMax = 1;

	memset(pwrInfo, 0, sizeof(*pwrInfo));

	if (AutoLoadFail) {
		for (group = 0; group < MAX_CHNL_GROUP; group++) {
			for (rfPath = 0; rfPath < rfPathMax; rfPath++) {
				pwrInfo->CCKIndex[rfPath][group] =
					EEPROM_Default_TxPowerLevel;
				pwrInfo->HT40_1SIndex[rfPath][group] =
					EEPROM_Default_TxPowerLevel;
				pwrInfo->HT40_2SIndexDiff[rfPath][group] =
					EEPROM_Default_HT40_2SDiff;
				pwrInfo->HT20IndexDiff[rfPath][group] =
					EEPROM_Default_HT20_Diff;
				pwrInfo->OFDMIndexDiff[rfPath][group] =
					EEPROM_Default_LegacyHTTxPowerDiff;
				pwrInfo->HT40MaxOffset[rfPath][group] =
					EEPROM_Default_HT40_PwrMaxOffset;
				pwrInfo->HT20MaxOffset[rfPath][group] =
					EEPROM_Default_HT20_PwrMaxOffset;
			}
		}
		pwrInfo->TSSI_A[0] = EEPROM_Default_TSSI;
		return;
	}

	for (rfPath = 0; rfPath < rfPathMax; rfPath++) {
		for (group = 0; group < MAX_CHNL_GROUP; group++) {
			eeAddr =
			    EEPROM_CCK_TX_PWR_INX_8723A + (rfPath * 3) + group;
			/* pwrInfo->CCKIndex[rfPath][group] =
			   PROMContent[eeAddr]; */
			Hal_EEValueCheck(EETYPE_TX_PWR, &PROMContent[eeAddr],
					 &pwrInfo->CCKIndex[rfPath][group]);
			eeAddr = EEPROM_HT40_1S_TX_PWR_INX_8723A +
				(rfPath * 3) + group;
			/* pwrInfo->HT40_1SIndex[rfPath][group] =
			   PROMContent[eeAddr]; */
			Hal_EEValueCheck(EETYPE_TX_PWR, &PROMContent[eeAddr],
					 &pwrInfo->HT40_1SIndex[rfPath][group]);
		}
	}

	for (group = 0; group < MAX_CHNL_GROUP; group++) {
		for (rfPath = 0; rfPath < rfPathMax; rfPath++) {
			pwrInfo->HT40_2SIndexDiff[rfPath][group] = 0;
			pwrInfo->HT20IndexDiff[rfPath][group] =
				(PROMContent
				 [EEPROM_HT20_TX_PWR_INX_DIFF_8723A +
				  group] >> (rfPath * 4)) & 0xF;
			/* 4bit sign number to 8 bit sign number */
			if (pwrInfo->HT20IndexDiff[rfPath][group] & BIT3)
				pwrInfo->HT20IndexDiff[rfPath][group] |= 0xF0;

			pwrInfo->OFDMIndexDiff[rfPath][group] =
				(PROMContent[EEPROM_OFDM_TX_PWR_INX_DIFF_8723A +
					     group] >> (rfPath * 4)) & 0xF;

			pwrInfo->HT40MaxOffset[rfPath][group] =
				(PROMContent[EEPROM_HT40_MAX_PWR_OFFSET_8723A +
					     group] >> (rfPath * 4)) & 0xF;

			pwrInfo->HT20MaxOffset[rfPath][group] =
				(PROMContent[EEPROM_HT20_MAX_PWR_OFFSET_8723A +
					     group] >> (rfPath * 4)) & 0xF;
		}
	}

	pwrInfo->TSSI_A[0] = PROMContent[EEPROM_TSSI_A_8723A];
}

static u8 Hal_GetChnlGroup(u8 chnl)
{
	u8 group = 0;

	if (chnl < 3)		/*  Cjanel 1-3 */
		group = 0;
	else if (chnl < 9)	/*  Channel 4-9 */
		group = 1;
	else			/*  Channel 10-14 */
		group = 2;

	return group;
}

void
Hal_EfuseParsetxpowerinfo_8723A(struct rtw_adapter *padapter,
				u8 *PROMContent, bool AutoLoadFail)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);
	struct txpowerinfo pwrInfo;
	u8 rfPath, ch, group, rfPathMax = 1;
	u8 pwr, diff;

	Hal_ReadPowerValueFromPROM_8723A(&pwrInfo, PROMContent, AutoLoadFail);
	for (rfPath = 0; rfPath < rfPathMax; rfPath++) {
		for (ch = 0; ch < CHANNEL_MAX_NUMBER; ch++) {
			group = Hal_GetChnlGroup(ch);

			pHalData->TxPwrLevelCck[rfPath][ch] =
				pwrInfo.CCKIndex[rfPath][group];
			pHalData->TxPwrLevelHT40_1S[rfPath][ch] =
				pwrInfo.HT40_1SIndex[rfPath][group];

			pHalData->TxPwrHt20Diff[rfPath][ch] =
				pwrInfo.HT20IndexDiff[rfPath][group];
			pHalData->TxPwrLegacyHtDiff[rfPath][ch] =
				pwrInfo.OFDMIndexDiff[rfPath][group];
			pHalData->PwrGroupHT20[rfPath][ch] =
				pwrInfo.HT20MaxOffset[rfPath][group];
			pHalData->PwrGroupHT40[rfPath][ch] =
				pwrInfo.HT40MaxOffset[rfPath][group];

			pwr = pwrInfo.HT40_1SIndex[rfPath][group];
			diff = pwrInfo.HT40_2SIndexDiff[rfPath][group];

			pHalData->TxPwrLevelHT40_2S[rfPath][ch] =
			    (pwr > diff) ? (pwr - diff) : 0;
		}
	}
	for (rfPath = 0; rfPath < RF_PATH_MAX; rfPath++) {
		for (ch = 0; ch < CHANNEL_MAX_NUMBER; ch++) {
			RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
				 ("RF(%u)-Ch(%u) [CCK / HT40_1S / HT40_2S] = "
				  "[0x%x / 0x%x / 0x%x]\n",
				  rfPath, ch,
				  pHalData->TxPwrLevelCck[rfPath][ch],
				  pHalData->TxPwrLevelHT40_1S[rfPath][ch],
				  pHalData->TxPwrLevelHT40_2S[rfPath][ch]));

		}
	}
	for (ch = 0; ch < CHANNEL_MAX_NUMBER; ch++) {
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
			 ("RF-A Ht20 to HT40 Diff[%u] = 0x%x(%d)\n", ch,
			  pHalData->TxPwrHt20Diff[RF_PATH_A][ch],
			  pHalData->TxPwrHt20Diff[RF_PATH_A][ch]));
	}
	for (ch = 0; ch < CHANNEL_MAX_NUMBER; ch++)
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
			 ("RF-A Legacy to Ht40 Diff[%u] = 0x%x\n", ch,
			  pHalData->TxPwrLegacyHtDiff[RF_PATH_A][ch]));
	for (ch = 0; ch < CHANNEL_MAX_NUMBER; ch++) {
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
			 ("RF-B Ht20 to HT40 Diff[%u] = 0x%x(%d)\n", ch,
			  pHalData->TxPwrHt20Diff[RF_PATH_B][ch],
			  pHalData->TxPwrHt20Diff[RF_PATH_B][ch]));
	}
	for (ch = 0; ch < CHANNEL_MAX_NUMBER; ch++)
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
			 ("RF-B Legacy to HT40 Diff[%u] = 0x%x\n", ch,
			  pHalData->TxPwrLegacyHtDiff[RF_PATH_B][ch]));
	if (!AutoLoadFail) {
		struct registry_priv *registry_par = &padapter->registrypriv;
		if (registry_par->regulatory_tid == 0xff) {
			if (PROMContent[RF_OPTION1_8723A] == 0xff)
				pHalData->EEPROMRegulatory = 0;
			else
				pHalData->EEPROMRegulatory =
					PROMContent[RF_OPTION1_8723A] & 0x7;
		} else {
			pHalData->EEPROMRegulatory =
			    registry_par->regulatory_tid;
		}
	} else {
		pHalData->EEPROMRegulatory = 0;
	}
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
		 ("EEPROMRegulatory = 0x%x\n", pHalData->EEPROMRegulatory));

	if (!AutoLoadFail)
		pHalData->bTXPowerDataReadFromEEPORM = true;
}

void
Hal_EfuseParseBTCoexistInfo_8723A(struct rtw_adapter *padapter,
				  u8 *hwinfo, bool AutoLoadFail)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);
	u8 tempval;
	u32 tmpu4;

	if (!AutoLoadFail) {
		tmpu4 = rtw_read32(padapter, REG_MULTI_FUNC_CTRL);
		if (tmpu4 & BT_FUNC_EN)
			pHalData->EEPROMBluetoothCoexist = 1;
		else
			pHalData->EEPROMBluetoothCoexist = 0;
		pHalData->EEPROMBluetoothType = BT_RTL8723A;

		/*  The following need to be checked with newer version of */
		/*  eeprom spec */
		tempval = hwinfo[RF_OPTION4_8723A];
		pHalData->EEPROMBluetoothAntNum = (tempval & 0x1);
		pHalData->EEPROMBluetoothAntIsolation = ((tempval & 0x10) >> 4);
		pHalData->EEPROMBluetoothRadioShared = ((tempval & 0x20) >> 5);
	} else {
		pHalData->EEPROMBluetoothCoexist = 0;
		pHalData->EEPROMBluetoothType = BT_RTL8723A;
		pHalData->EEPROMBluetoothAntNum = Ant_x2;
		pHalData->EEPROMBluetoothAntIsolation = 0;
		pHalData->EEPROMBluetoothRadioShared = BT_Radio_Shared;
	}
#ifdef CONFIG_8723AU_BT_COEXIST
	BT_InitHalVars(padapter);
#endif
}

void
Hal_EfuseParseEEPROMVer(struct rtw_adapter *padapter,
			u8 *hwinfo, bool AutoLoadFail)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	if (!AutoLoadFail)
		pHalData->EEPROMVersion = hwinfo[EEPROM_VERSION_8723A];
	else
		pHalData->EEPROMVersion = 1;
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
		 ("Hal_EfuseParseEEPROMVer(), EEVer = %d\n",
		  pHalData->EEPROMVersion));
}

void
rtl8723a_EfuseParseChnlPlan(struct rtw_adapter *padapter,
			    u8 *hwinfo, bool AutoLoadFail)
{
	padapter->mlmepriv.ChannelPlan =
		hal_com_get_channel_plan23a(padapter, hwinfo ?
					 hwinfo[EEPROM_ChannelPlan_8723A]:0xFF,
					 padapter->registrypriv.channel_plan,
					 RT_CHANNEL_DOMAIN_WORLD_WIDE_13,
					 AutoLoadFail);

	DBG_8723A("mlmepriv.ChannelPlan = 0x%02x\n",
		  padapter->mlmepriv.ChannelPlan);
}

void
Hal_EfuseParseCustomerID(struct rtw_adapter *padapter,
			 u8 *hwinfo, bool AutoLoadFail)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	if (!AutoLoadFail) {
		pHalData->EEPROMCustomerID = hwinfo[EEPROM_CustomID_8723A];
		pHalData->EEPROMSubCustomerID =
		    hwinfo[EEPROM_SubCustomID_8723A];
	} else {
		pHalData->EEPROMCustomerID = 0;
		pHalData->EEPROMSubCustomerID = 0;
	}
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
		 ("EEPROM Customer ID: 0x%2x\n", pHalData->EEPROMCustomerID));
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
		 ("EEPROM SubCustomer ID: 0x%02x\n",
		  pHalData->EEPROMSubCustomerID));
}

void
Hal_EfuseParseAntennaDiversity(struct rtw_adapter *padapter,
			       u8 *hwinfo, bool AutoLoadFail)
{
}

void
Hal_EfuseParseRateIndicationOption(struct rtw_adapter *padapter,
				   u8 *hwinfo, bool AutoLoadFail)
{
}

void
Hal_EfuseParseXtal_8723A(struct rtw_adapter *pAdapter,
			 u8 *hwinfo, u8 AutoLoadFail)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(pAdapter);

	if (!AutoLoadFail) {
		pHalData->CrystalCap = hwinfo[EEPROM_XTAL_K_8723A];
		if (pHalData->CrystalCap == 0xFF)
			pHalData->CrystalCap = EEPROM_Default_CrystalCap_8723A;
	} else {
		pHalData->CrystalCap = EEPROM_Default_CrystalCap_8723A;
	}
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
		 ("%s: CrystalCap = 0x%2x\n", __func__,
		  pHalData->CrystalCap));
}

void
Hal_EfuseParseThermalMeter_8723A(struct rtw_adapter *padapter,
				 u8 *PROMContent, u8 AutoloadFail)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	/*  */
	/*  ThermalMeter from EEPROM */
	/*  */
	if (false == AutoloadFail)
		pHalData->EEPROMThermalMeter =
		    PROMContent[EEPROM_THERMAL_METER_8723A];
	else
		pHalData->EEPROMThermalMeter = EEPROM_Default_ThermalMeter;

	if ((pHalData->EEPROMThermalMeter == 0xff) || (true == AutoloadFail)) {
		pHalData->bAPKThermalMeterIgnore = true;
		pHalData->EEPROMThermalMeter = EEPROM_Default_ThermalMeter;
	}

	DBG_8723A("%s: ThermalMeter = 0x%x\n", __func__,
		  pHalData->EEPROMThermalMeter);
}

void Hal_InitChannelPlan23a(struct rtw_adapter *padapter)
{
}

static void rtl8723a_cal_txdesc_chksum(struct tx_desc *ptxdesc)
{
	u16 *usPtr = (u16 *) ptxdesc;
	u32 count = 16;		/*  (32 bytes / 2 bytes per XOR) => 16 times */
	u32 index;
	u16 checksum = 0;

	/*  Clear first */
	ptxdesc->txdw7 &= cpu_to_le32(0xffff0000);

	for (index = 0; index < count; index++) {
		checksum ^= le16_to_cpu(*(usPtr + index));
	}

	ptxdesc->txdw7 |= cpu_to_le32(checksum & 0x0000ffff);
}

static void fill_txdesc_sectype(struct pkt_attrib *pattrib,
				struct txdesc_8723a *ptxdesc)
{
	if ((pattrib->encrypt > 0) && !pattrib->bswenc) {
		switch (pattrib->encrypt) {
			/*  SEC_TYPE */
		case _WEP40_:
		case _WEP104_:
		case _TKIP_:
		case _TKIP_WTMIC_:
			ptxdesc->sectype = 1;
			break;

		case _AES_:
			ptxdesc->sectype = 3;
			break;

		case _NO_PRIVACY_:
		default:
			break;
		}
	}
}

static void fill_txdesc_vcs(struct pkt_attrib *pattrib,
			    struct txdesc_8723a *ptxdesc)
{
	/* DBG_8723A("cvs_mode =%d\n", pattrib->vcs_mode); */

	switch (pattrib->vcs_mode) {
	case RTS_CTS:
		ptxdesc->rtsen = 1;
		break;

	case CTS_TO_SELF:
		ptxdesc->cts2self = 1;
		break;

	case NONE_VCS:
	default:
		break;
	}

	if (pattrib->vcs_mode) {
		ptxdesc->hw_rts_en = 1;	/*  ENABLE HW RTS */

		/*  Set RTS BW */
		if (pattrib->ht_en) {
			if (pattrib->bwmode & HT_CHANNEL_WIDTH_40)
				ptxdesc->rts_bw = 1;

			switch (pattrib->ch_offset) {
			case HAL_PRIME_CHNL_OFFSET_DONT_CARE:
				ptxdesc->rts_sc = 0;
				break;

			case HAL_PRIME_CHNL_OFFSET_LOWER:
				ptxdesc->rts_sc = 1;
				break;

			case HAL_PRIME_CHNL_OFFSET_UPPER:
				ptxdesc->rts_sc = 2;
				break;

			default:
				ptxdesc->rts_sc = 3;	/*  Duplicate */
				break;
			}
		}
	}
}

static void fill_txdesc_phy(struct pkt_attrib *pattrib,
			    struct txdesc_8723a *ptxdesc)
{
	if (pattrib->ht_en) {
		if (pattrib->bwmode & HT_CHANNEL_WIDTH_40)
			ptxdesc->data_bw = 1;

		switch (pattrib->ch_offset) {
		case HAL_PRIME_CHNL_OFFSET_DONT_CARE:
			ptxdesc->data_sc = 0;
			break;

		case HAL_PRIME_CHNL_OFFSET_LOWER:
			ptxdesc->data_sc = 1;
			break;

		case HAL_PRIME_CHNL_OFFSET_UPPER:
			ptxdesc->data_sc = 2;
			break;

		default:
			ptxdesc->data_sc = 3;	/*  Duplicate */
			break;
		}
	}
}

static void rtl8723a_fill_default_txdesc(struct xmit_frame *pxmitframe,
					 u8 *pbuf)
{
	struct rtw_adapter *padapter;
	struct hal_data_8723a *pHalData;
	struct dm_priv *pdmpriv;
	struct mlme_ext_priv *pmlmeext;
	struct mlme_ext_info *pmlmeinfo;
	struct pkt_attrib *pattrib;
	struct txdesc_8723a *ptxdesc;
	s32 bmcst;

	padapter = pxmitframe->padapter;
	pHalData = GET_HAL_DATA(padapter);
	pdmpriv = &pHalData->dmpriv;
	pmlmeext = &padapter->mlmeextpriv;
	pmlmeinfo = &pmlmeext->mlmext_info;

	pattrib = &pxmitframe->attrib;
	bmcst = is_multicast_ether_addr(pattrib->ra);

	ptxdesc = (struct txdesc_8723a *)pbuf;

	if (pxmitframe->frame_tag == DATA_FRAMETAG) {
		ptxdesc->macid = pattrib->mac_id;	/*  CAM_ID(MAC_ID) */

		if (pattrib->ampdu_en == true)
			ptxdesc->agg_en = 1;	/*  AGG EN */
		else
			ptxdesc->bk = 1;	/*  AGG BK */

		ptxdesc->qsel = pattrib->qsel;
		ptxdesc->rate_id = pattrib->raid;

		fill_txdesc_sectype(pattrib, ptxdesc);

		ptxdesc->seq = pattrib->seqnum;

		if ((pattrib->ether_type != 0x888e) &&
		    (pattrib->ether_type != 0x0806) &&
		    (pattrib->dhcp_pkt != 1)) {
			/*  Non EAP & ARP & DHCP type data packet */

			fill_txdesc_vcs(pattrib, ptxdesc);
			fill_txdesc_phy(pattrib, ptxdesc);

			ptxdesc->rtsrate = 8;	/*  RTS Rate = 24M */
			ptxdesc->data_ratefb_lmt = 0x1F;
			ptxdesc->rts_ratefb_lmt = 0xF;

			/*  use REG_INIDATA_RATE_SEL value */
			ptxdesc->datarate =
				pdmpriv->INIDATA_RATE[pattrib->mac_id];

		} else {
			/*  EAP data packet and ARP packet. */
			/*  Use the 1M data rate to send the EAP/ARP packet. */
			/*  This will maybe make the handshake smooth. */

			ptxdesc->bk = 1;	/*  AGG BK */
			ptxdesc->userate = 1;	/*  driver uses rate */
			if (pmlmeinfo->preamble_mode == PREAMBLE_SHORT)
				ptxdesc->data_short = 1;
			ptxdesc->datarate = MRateToHwRate23a(pmlmeext->tx_rate);
		}
	} else if (pxmitframe->frame_tag == MGNT_FRAMETAG) {
/*		RT_TRACE(_module_hal_xmit_c_, _drv_notice_,
		("%s: MGNT_FRAMETAG\n", __func__)); */

		ptxdesc->macid = pattrib->mac_id;	/*  CAM_ID(MAC_ID) */
		ptxdesc->qsel = pattrib->qsel;
		ptxdesc->rate_id = pattrib->raid;	/*  Rate ID */
		ptxdesc->seq = pattrib->seqnum;
		ptxdesc->userate = 1;	/*  driver uses rate, 1M */
		ptxdesc->rty_lmt_en = 1;	/*  retry limit enable */
		ptxdesc->data_rt_lmt = 6;	/*  retry limit = 6 */

		/* CCX-TXRPT ack for xmit mgmt frames. */
		if (pxmitframe->ack_report)
			ptxdesc->ccx = 1;

		ptxdesc->datarate = MRateToHwRate23a(pmlmeext->tx_rate);
	} else if (pxmitframe->frame_tag == TXAGG_FRAMETAG) {
		RT_TRACE(_module_hal_xmit_c_, _drv_warning_,
			 ("%s: TXAGG_FRAMETAG\n", __func__));
	} else {
		RT_TRACE(_module_hal_xmit_c_, _drv_warning_,
			 ("%s: frame_tag = 0x%x\n", __func__,
			  pxmitframe->frame_tag));

		ptxdesc->macid = 4;	/*  CAM_ID(MAC_ID) */
		ptxdesc->rate_id = 6;	/*  Rate ID */
		ptxdesc->seq = pattrib->seqnum;
		ptxdesc->userate = 1;	/*  driver uses rate */
		ptxdesc->datarate = MRateToHwRate23a(pmlmeext->tx_rate);
	}

	ptxdesc->pktlen = pattrib->last_txcmdsz;
	ptxdesc->offset = TXDESC_SIZE + OFFSET_SZ;
	if (bmcst)
		ptxdesc->bmc = 1;
	ptxdesc->ls = 1;
	ptxdesc->fs = 1;
	ptxdesc->own = 1;

	/*  2009.11.05. tynli_test. Suggested by SD4 Filen for FW LPS. */
	/*  (1) The sequence number of each non-Qos frame / broadcast /
	 *   multicast / mgnt frame should be controled by Hw because Fw
	 * will also send null data which we cannot control when Fw LPS enable.
	 *  --> default enable non-Qos data sequense number.
	 2010.06.23. by tynli. */
	/*  (2) Enable HW SEQ control for beacon packet,
	 * because we use Hw beacon. */
	/*  (3) Use HW Qos SEQ to control the seq num of Ext port
	 * non-Qos packets. */
	/*  2010.06.23. Added by tynli. */
	if (!pattrib->qos_en) {
		/*  Hw set sequence number */
		ptxdesc->hwseq_en = 1;	/*  HWSEQ_EN */
		ptxdesc->hwseq_sel = 0;	/*  HWSEQ_SEL */
	}
}

/*
 *	Description:
 *
 *	Parameters:
 *		pxmitframe	xmitframe
 *		pbuf		where to fill tx desc
 */
void rtl8723a_update_txdesc(struct xmit_frame *pxmitframe, u8 *pbuf)
{
	struct tx_desc *pdesc;

	pdesc = (struct tx_desc *)pbuf;
	memset(pdesc, 0, sizeof(struct tx_desc));

	rtl8723a_fill_default_txdesc(pxmitframe, pbuf);

	pdesc->txdw0 = cpu_to_le32(pdesc->txdw0);
	pdesc->txdw1 = cpu_to_le32(pdesc->txdw1);
	pdesc->txdw2 = cpu_to_le32(pdesc->txdw2);
	pdesc->txdw3 = cpu_to_le32(pdesc->txdw3);
	pdesc->txdw4 = cpu_to_le32(pdesc->txdw4);
	pdesc->txdw5 = cpu_to_le32(pdesc->txdw5);
	pdesc->txdw6 = cpu_to_le32(pdesc->txdw6);
	pdesc->txdw7 = cpu_to_le32(pdesc->txdw7);
	rtl8723a_cal_txdesc_chksum(pdesc);
}

/*
 *  Description: In normal chip, we should send some packet to Hw which
 *  will be used by Fw in FW LPS mode. The function is to fill the Tx
 * descriptor of this packets, then
 */
/*			Fw can tell Hw to send these packet derectly. */
/*  Added by tynli. 2009.10.15. */
/*  */
void rtl8723a_fill_fake_txdesc(struct rtw_adapter *padapter, u8 *pDesc,
			       u32 BufferLen, u8 IsPsPoll, u8 IsBTQosNull)
{
	struct tx_desc *ptxdesc;

	/*  Clear all status */
	ptxdesc = (struct tx_desc *)pDesc;
	memset(pDesc, 0, TXDESC_SIZE);

	/* offset 0 */
	/* own, bFirstSeg, bLastSeg; */
	ptxdesc->txdw0 |= cpu_to_le32(OWN | FSG | LSG);

	/* 32 bytes for TX Desc */
	ptxdesc->txdw0 |= cpu_to_le32(((TXDESC_SIZE + OFFSET_SZ) <<
				       OFFSET_SHT) & 0x00ff0000);

	/*  Buffer size + command header */
	ptxdesc->txdw0 |= cpu_to_le32(BufferLen & 0x0000ffff);

	/* offset 4 */
	/*  Fixed queue of Mgnt queue */
	ptxdesc->txdw1 |= cpu_to_le32((QSLT_MGNT << QSEL_SHT) & 0x00001f00);

	/* Set NAVUSEHDR to prevent Ps-poll AId filed to be changed
	   to error vlaue by Hw. */
	if (IsPsPoll) {
		ptxdesc->txdw1 |= cpu_to_le32(NAVUSEHDR);
	} else {
		/*  Hw set sequence number */
		ptxdesc->txdw4 |= cpu_to_le32(BIT(7));
		/* set bit3 to 1. Suugested by TimChen. 2009.12.29. */
		ptxdesc->txdw3 |= cpu_to_le32((8 << 28));
	}

	if (true == IsBTQosNull) {
		ptxdesc->txdw2 |= cpu_to_le32(BIT(23));	/*  BT NULL */
	}

	/* offset 16 */
	ptxdesc->txdw4 |= cpu_to_le32(BIT(8));	/* driver uses rate */

	/*  USB interface drop packet if the checksum of descriptor isn't
	    correct. */
	/*  Using this checksum can let hardware recovery from packet bulk
	    out error (e.g. Cancel URC, Bulk out error.). */
	rtl8723a_cal_txdesc_chksum(ptxdesc);
}

static void hw_var_set_opmode(struct rtw_adapter *padapter, u8 mode)
{
	u8 val8;

	if ((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_)) {
		StopTxBeacon(padapter);

		/*  disable atim wnd */
		val8 = DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_ATIM;
		SetBcnCtrlReg23a(padapter, val8, ~val8);
	} else if ((mode == _HW_STATE_ADHOC_) /*|| (mode == _HW_STATE_AP_) */) {
		ResumeTxBeacon(padapter);

		val8 = DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_BCNQ_SUB;
		SetBcnCtrlReg23a(padapter, val8, ~val8);
	} else if (mode == _HW_STATE_AP_) {
#ifdef CONFIG_8723AU_BT_COEXIST
		/*  add NULL Data and BT NULL Data Packets to FW RSVD Page */
		rtl8723a_set_BTCoex_AP_mode_FwRsvdPkt_cmd(padapter);
#endif

		ResumeTxBeacon(padapter);

		val8 = DIS_TSF_UDT | DIS_BCNQ_SUB;
		SetBcnCtrlReg23a(padapter, val8, ~val8);

		/*  Set RCR */
		/* rtw_write32(padapter, REG_RCR, 0x70002a8e);
		   CBSSID_DATA must set to 0 */
		/* CBSSID_DATA must set to 0 */
		rtw_write32(padapter, REG_RCR, 0x7000228e);
		/*  enable to rx data frame */
		rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);
		/*  enable to rx ps-poll */
		rtw_write16(padapter, REG_RXFLTMAP1, 0x0400);

		/*  Beacon Control related register for first time */
		rtw_write8(padapter, REG_BCNDMATIM, 0x02);	/*  2ms */
		rtw_write8(padapter, REG_DRVERLYINT, 0x05);	/*  5ms */
		rtw_write8(padapter, REG_ATIMWND, 0x0a); /*  10ms for port0 */
		rtw_write16(padapter, REG_BCNTCFG, 0x00);
		rtw_write16(padapter, REG_TBTT_PROHIBIT, 0xff04);
		/*  +32767 (~32ms) */
		rtw_write16(padapter, REG_TSFTR_SYN_OFFSET, 0x7fff);

		/*  reset TSF */
		rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(0));

		/*  enable BCN Function */
		/*  don't enable update TSF (due to TSF update when
		    beacon/probe rsp are received) */
		val8 = DIS_TSF_UDT | EN_BCN_FUNCTION |
		       EN_TXBCN_RPT | DIS_BCNQ_SUB;
		SetBcnCtrlReg23a(padapter, val8, ~val8);
	}

	val8 = rtw_read8(padapter, MSR);
	val8 = (val8 & 0xC) | mode;
	rtw_write8(padapter, MSR, val8);
}

static void hw_var_set_macaddr(struct rtw_adapter *padapter, u8 *val)
{
	u8 idx = 0;
	u32 reg_macid;

	reg_macid = REG_MACID;

	for (idx = 0; idx < 6; idx++)
		rtw_write8(padapter, (reg_macid + idx), val[idx]);
}

static void hw_var_set_bssid(struct rtw_adapter *padapter, u8 *val)
{
	u8 idx = 0;
	u32 reg_bssid;

	reg_bssid = REG_BSSID;

	for (idx = 0; idx < 6; idx++)
		rtw_write8(padapter, (reg_bssid + idx), val[idx]);
}

static void hw_var_set_correct_tsf(struct rtw_adapter *padapter)
{
	u64 tsf;
	u32 reg_tsftr;
	struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;

	/* tsf = pmlmeext->TSFValue - ((u32)pmlmeext->TSFValue %
	   (pmlmeinfo->bcn_interval*1024)) - 1024; us */
	tsf = pmlmeext->TSFValue -
		rtw_modular6423a(pmlmeext->TSFValue,
			      (pmlmeinfo->bcn_interval * 1024)) - 1024;	/* us */

	if (((pmlmeinfo->state & 0x03) == WIFI_FW_ADHOC_STATE) ||
	    ((pmlmeinfo->state & 0x03) == WIFI_FW_AP_STATE)) {
		/* pHalData->RegTxPause |= STOP_BCNQ;BIT(6) */
		/* rtw_write8(padapter, REG_TXPAUSE,
		   (rtw_read8(Adapter, REG_TXPAUSE)|BIT(6))); */
		StopTxBeacon(padapter);
	}

	reg_tsftr = REG_TSFTR;

	/*  disable related TSF function */
	SetBcnCtrlReg23a(padapter, 0, EN_BCN_FUNCTION);

	rtw_write32(padapter, reg_tsftr, tsf);
	rtw_write32(padapter, reg_tsftr + 4, tsf >> 32);

	/* enable related TSF function */
	SetBcnCtrlReg23a(padapter, EN_BCN_FUNCTION, 0);

	if (((pmlmeinfo->state & 0x03) == WIFI_FW_ADHOC_STATE) ||
	    ((pmlmeinfo->state & 0x03) == WIFI_FW_AP_STATE))
		ResumeTxBeacon(padapter);
}

static void hw_var_set_mlme_disconnect(struct rtw_adapter *padapter)
{
	/*  reject all data frames */
	rtw_write16(padapter, REG_RXFLTMAP2, 0);

	/*  reset TSF */
	rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(0));

	/*  disable update TSF */
	SetBcnCtrlReg23a(padapter, DIS_TSF_UDT, 0);
}

static void hw_var_set_mlme_join(struct rtw_adapter *padapter, u8 type)
{
	u8 RetryLimit = 0x30;

	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;

	if (type == 0) {	/*  prepare to join */
		u32 v32;

		/*  enable to rx data frame.Accept all data frame */
		/* rtw_write32(padapter, REG_RCR,
		   rtw_read32(padapter, REG_RCR)|RCR_ADF); */
		rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);

		v32 = rtw_read32(padapter, REG_RCR);
		v32 |= RCR_CBSSID_DATA | RCR_CBSSID_BCN;
		rtw_write32(padapter, REG_RCR, v32);

		if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == true)
			RetryLimit =
			    (pHalData->CustomerID == RT_CID_CCX) ? 7 : 48;
		else		/*  Ad-hoc Mode */
			RetryLimit = 0x7;
	} else if (type == 1) {	/*  joinbss_event callback when join res < 0 */
		/*  config RCR to receive different BSSID & not to
		    receive data frame during linking */
		rtw_write16(padapter, REG_RXFLTMAP2, 0);
	} else if (type == 2) {	/*  sta add event callback */
		/*  enable update TSF */
		SetBcnCtrlReg23a(padapter, 0, DIS_TSF_UDT);

		if (check_fwstate(pmlmepriv,
				  WIFI_ADHOC_STATE | WIFI_ADHOC_MASTER_STATE)) {
			/*  fixed beacon issue for 8191su........... */
			rtw_write8(padapter, 0x542, 0x02);
			RetryLimit = 0x7;
		}
	}

	rtw_write16(padapter, REG_RL,
		    RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit <<
		    RETRY_LIMIT_LONG_SHIFT);

#ifdef CONFIG_8723AU_BT_COEXIST
	switch (type) {
	case 0:
		/*  prepare to join */
		BT_WifiAssociateNotify(padapter, true);
		break;
	case 1:
		/*  joinbss_event callback when join res < 0 */
		BT_WifiAssociateNotify(padapter, false);
		break;
	case 2:
		/*  sta add event callback */
/*		BT_WifiMediaStatusNotify(padapter, RT_MEDIA_CONNECT); */
		break;
	}
#endif
}

void SetHwReg8723A(struct rtw_adapter *padapter, u8 variable, u8 *val)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);
	u32 *val32 = (u32 *)val;

	switch (variable) {
	case HW_VAR_MEDIA_STATUS:
		rtl8723a_set_media_status(padapter, *val);
		break;

	case HW_VAR_MEDIA_STATUS1:
		rtl8723a_set_media_status1(padapter, *val);
		break;

	case HW_VAR_SET_OPMODE:
		hw_var_set_opmode(padapter, *val);
		break;

	case HW_VAR_MAC_ADDR:
		hw_var_set_macaddr(padapter, val);
		break;

	case HW_VAR_BSSID:
		hw_var_set_bssid(padapter, val);
		break;

	case HW_VAR_BASIC_RATE:
		HalSetBrateCfg23a(padapter, val);
		break;

	case HW_VAR_TXPAUSE:
		rtl8723a_set_tx_pause(padapter, *val);
		break;

	case HW_VAR_BCN_FUNC:
		rtl8723a_set_bcn_func(padapter, *val);
		break;

	case HW_VAR_CORRECT_TSF:
		hw_var_set_correct_tsf(padapter);
		break;

	case HW_VAR_CHECK_BSSID:
		rtl8723a_check_bssid(padapter, *val);
		break;

	case HW_VAR_MLME_DISCONNECT:
		hw_var_set_mlme_disconnect(padapter);
		break;

	case HW_VAR_MLME_SITESURVEY:
		rtl8723a_mlme_sitesurvey(padapter, *val);
		break;

	case HW_VAR_MLME_JOIN:
		hw_var_set_mlme_join(padapter, *val);
		break;

	case HW_VAR_ON_RCR_AM:
		rtl8723a_on_rcr_am(padapter);
		break;

	case HW_VAR_OFF_RCR_AM:
		rtl8723a_off_rcr_am(padapter);
		break;

	case HW_VAR_BEACON_INTERVAL:
		rtl8723a_set_beacon_interval(padapter, *((u16 *) val));
		break;

	case HW_VAR_SLOT_TIME:
		rtl8723a_set_slot_time(padapter, *val);
		break;

	case HW_VAR_RESP_SIFS:
		rtl8723a_set_resp_sifs(padapter, val[0], val[1],
				       val[2], val[3]);
		break;

	case HW_VAR_ACK_PREAMBLE:
		rtl8723a_ack_preamble(padapter, *val);
		break;

	case HW_VAR_SEC_CFG:
		rtl8723a_set_sec_cfg(padapter, *val);
		break;

	case HW_VAR_DM_FLAG:
		rtl8723a_odm_support_ability_write(padapter, *val32);
		break;
	case HW_VAR_DM_FUNC_OP:
		rtl8723a_odm_support_ability_backup(padapter, *val);
		break;
	case HW_VAR_DM_FUNC_SET:
		rtl8723a_odm_support_ability_set(padapter, *val32);
		break;

	case HW_VAR_DM_FUNC_CLR:
		rtl8723a_odm_support_ability_clr(padapter, *val32);
		break;

	case HW_VAR_CAM_EMPTY_ENTRY:
		rtl8723a_cam_empty_entry(padapter, *val);
		break;

	case HW_VAR_CAM_INVALID_ALL:
		rtl8723a_cam_invalid_all(padapter);
		break;

	case HW_VAR_CAM_WRITE:
		rtl8723a_cam_write(padapter, val32[0], val32[1]);
		break;

	case HW_VAR_AC_PARAM_VO:
		rtl8723a_set_ac_param_vo(padapter, *val32);
		break;

	case HW_VAR_AC_PARAM_VI:
		rtl8723a_set_ac_param_vi(padapter, *val32);
		break;

	case HW_VAR_AC_PARAM_BE:
		rtl8723a_set_ac_param_be(padapter, *val32);
		break;

	case HW_VAR_AC_PARAM_BK:
		rtl8723a_set_ac_param_bk(padapter, *val32);
		break;

	case HW_VAR_ACM_CTRL:
		rtl8723a_set_acm_ctrl(padapter, *val);
		break;

	case HW_VAR_AMPDU_MIN_SPACE:
		rtl8723a_set_ampdu_min_space(padapter, *val);
		break;

	case HW_VAR_AMPDU_FACTOR:
		rtl8723a_set_ampdu_factor(padapter, *val);
		break;

	case HW_VAR_RXDMA_AGG_PG_TH:
		rtl8723a_set_rxdma_agg_pg_th(padapter, *val);
		break;

	case HW_VAR_H2C_FW_PWRMODE:
		rtl8723a_set_FwPwrMode_cmd(padapter, *val);
		break;

	case HW_VAR_H2C_FW_JOINBSSRPT:
		rtl8723a_set_FwJoinBssReport_cmd(padapter, *val);
		break;

#ifdef CONFIG_8723AU_P2P
	case HW_VAR_H2C_FW_P2P_PS_OFFLOAD:
		rtl8723a_set_p2p_ps_offload_cmd(padapter, *val);
		break;
#endif /* CONFIG_8723AU_P2P */

	case HW_VAR_INITIAL_GAIN:
		rtl8723a_set_initial_gain(padapter, *val32);
		break;
	case HW_VAR_EFUSE_BYTES:
		pHalData->EfuseUsedBytes = *((u16 *) val);
		break;
	case HW_VAR_EFUSE_BT_BYTES:
		pHalData->BTEfuseUsedBytes = *((u16 *) val);
		break;
	case HW_VAR_FIFO_CLEARN_UP:
		rtl8723a_fifo_cleanup(padapter);
		break;
	case HW_VAR_CHECK_TXBUF:
		break;
	case HW_VAR_APFM_ON_MAC:
		rtl8723a_set_apfm_on_mac(padapter, *val);
		break;

	case HW_VAR_NAV_UPPER:
		rtl8723a_set_nav_upper(padapter, *val32);
		break;
	case HW_VAR_BCN_VALID:
		rtl8723a_bcn_valid(padapter);
		break;
	default:
		break;
	}

}

void GetHwReg8723A(struct rtw_adapter *padapter, u8 variable, u8 *val)
{
	struct hal_data_8723a *pHalData = GET_HAL_DATA(padapter);

	switch (variable) {
	case HW_VAR_BASIC_RATE:
		*((u16 *) val) = pHalData->BasicRateSet;
		break;

	case HW_VAR_TXPAUSE:
		*val = rtw_read8(padapter, REG_TXPAUSE);
		break;

	case HW_VAR_BCN_VALID:
		/* BCN_VALID, BIT16 of REG_TDECTRL = BIT0 of REG_TDECTRL+2 */
		val[0] = (BIT0 & rtw_read8(padapter, REG_TDECTRL + 2)) ? true :
			false;
		break;

	case HW_VAR_RF_TYPE:
		*val = pHalData->rf_type;
		break;

	case HW_VAR_DM_FLAG:
	{
		struct dm_odm_t *podmpriv = &pHalData->odmpriv;
		*((u32 *) val) = podmpriv->SupportAbility;
	}
		break;

	case HW_VAR_FWLPS_RF_ON:
	{
		/*  When we halt NIC, we should check if FW LPS is leave. */
		u32 valRCR;

		if ((padapter->bSurpriseRemoved == true) ||
		    (padapter->pwrctrlpriv.rf_pwrstate == rf_off)) {
			/*  If it is in HW/SW Radio OFF or IPS state, we do
			    not check Fw LPS Leave, because Fw is unload. */
			*val = true;
		} else {
			valRCR = rtw_read32(padapter, REG_RCR);
			valRCR &= 0x00070000;
			if (valRCR)
				*val = false;
			else
				*val = true;
		}
	}
		break;
	case HW_VAR_EFUSE_BYTES:
		*((u16 *) val) = pHalData->EfuseUsedBytes;
		break;

	case HW_VAR_EFUSE_BT_BYTES:
		*((u16 *) val) = pHalData->BTEfuseUsedBytes;
		break;

	case HW_VAR_APFM_ON_MAC:
		*val = pHalData->bMacPwrCtrlOn;
		break;
	case HW_VAR_CHK_HI_QUEUE_EMPTY:
		*val =
		    ((rtw_read32(padapter, REG_HGQ_INFORMATION) & 0x0000ff00) ==
		     0) ? true : false;
		break;
	}
}

#ifdef CONFIG_8723AU_BT_COEXIST

void rtl8723a_SingleDualAntennaDetection(struct rtw_adapter *padapter)
{
	struct hal_data_8723a *pHalData;
	struct dm_odm_t *pDM_Odm;
	struct sw_ant_sw *pDM_SWAT_Table;
	u8 i;

	pHalData = GET_HAL_DATA(padapter);
	pDM_Odm = &pHalData->odmpriv;
	pDM_SWAT_Table = &pDM_Odm->DM_SWAT_Table;

	/*  */
	/*  <Roger_Notes> RTL8723A Single and Dual antenna dynamic detection
	    mechanism when RF power state is on. */
	/*  We should take power tracking, IQK, LCK, RCK RF read/write
	    operation into consideration. */
	/*  2011.12.15. */
	/*  */
	if (!pHalData->bAntennaDetected) {
		u8 btAntNum = BT_GetPGAntNum(padapter);

		/*  Set default antenna B status */
		if (btAntNum == Ant_x2)
			pDM_SWAT_Table->ANTB_ON = true;
		else if (btAntNum == Ant_x1)
			pDM_SWAT_Table->ANTB_ON = false;
		else
			pDM_SWAT_Table->ANTB_ON = true;

		if (pHalData->CustomerID != RT_CID_TOSHIBA) {
			for (i = 0; i < MAX_ANTENNA_DETECTION_CNT; i++) {
				if (ODM_SingleDualAntennaDetection
				    (&pHalData->odmpriv, ANTTESTALL) == true)
					break;
			}

			/*  Set default antenna number for BT coexistence */
			if (btAntNum == Ant_x2)
				BT_SetBtCoexCurrAntNum(padapter,
						       pDM_SWAT_Table->
						       ANTB_ON ? 2 : 1);
		}
		pHalData->bAntennaDetected = true;
	}
}
#endif /*  CONFIG_8723AU_BT_COEXIST */

void rtl8723a_clone_haldata(struct rtw_adapter *dst_adapter,
			    struct rtw_adapter *src_adapter)
{
	memcpy(dst_adapter->HalData, src_adapter->HalData,
	       dst_adapter->hal_data_sz);
}

void rtl8723a_start_thread(struct rtw_adapter *padapter)
{
}

void rtl8723a_stop_thread(struct rtw_adapter *padapter)
{
}
