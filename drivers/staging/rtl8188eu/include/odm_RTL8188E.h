/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#ifndef	__ODM_RTL8188E_H__
#define __ODM_RTL8188E_H__

#define	MAIN_ANT	0
#define	AUX_ANT	1
#define	MAIN_ANT_CG_TRX	1
#define	AUX_ANT_CG_TRX	0
#define	MAIN_ANT_CGCS_RX	0
#define	AUX_ANT_CGCS_RX	1

void ODM_DIG_LowerBound_88E(struct odm_dm_struct *pDM_Odm);

void ODM_AntennaDiversityInit_88E(struct odm_dm_struct *pDM_Odm);

void ODM_AntennaDiversity_88E(struct odm_dm_struct *pDM_Odm);

void ODM_SetTxAntByTxInfo_88E(struct odm_dm_struct *pDM_Odm, u8 *pDesc,
			      u8 macId);

void ODM_UpdateRxIdleAnt_88E(struct odm_dm_struct *pDM_Odm, u8 Ant);

void ODM_AntselStatistics_88E(struct odm_dm_struct *pDM_Odm, u8	antsel_tr_mux,
			      u32 MacId, u8 RxPWDBAll);

void odm_FastAntTraining(struct odm_dm_struct *pDM_Odm);

void odm_FastAntTrainingCallback(struct odm_dm_struct *pDM_Odm);

void odm_FastAntTrainingWorkItemCallback(struct odm_dm_struct *pDM_Odm);

void odm_PrimaryCCA_Init(struct odm_dm_struct *pDM_Odm);

bool ODM_DynamicPrimaryCCA_DupRTS(struct odm_dm_struct *pDM_Odm);

void odm_DynamicPrimaryCCA(struct odm_dm_struct *pDM_Odm);

#endif
