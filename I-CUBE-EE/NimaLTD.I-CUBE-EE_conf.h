/**
  ******************************************************************************
  * File Name          : NimaLTD.I-CUBE-EE_conf.h
  * Description        : This file provides code for the configuration
  *                      of the NimaLTD.I-CUBE-EE_conf.h instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _NIMALTD_I_CUBE_EE_CONF_H_
#define _NIMALTD_I_CUBE_EE_CONF_H_

#ifdef __cplusplus
 extern "C" {
#endif

#define EE_PAGE_SECTOR_SIZE_1K            (1024 * 1)
#define EE_PAGE_SECTOR_SIZE_2K            (1024 * 2)
#define EE_PAGE_SECTOR_SIZE_4K            (1024 * 4)
#define EE_PAGE_SECTOR_SIZE_8K            (1024 * 8)
#define EE_PAGE_SECTOR_SIZE_16K           (1024 * 16)
#define EE_PAGE_SECTOR_SIZE_32K           (1024 * 32)
#define EE_PAGE_SECTOR_SIZE_64K           (1024 * 64)
#define EE_PAGE_SECTOR_SIZE_128K          (1024 * 128)
#define EE_PAGE_SECTOR_SIZE_256K          (1024 * 256)

/**
  MiddleWare name : NimaLTD.I-CUBE-EE.3.2.2
  MiddleWare fileName : ./NimaLTD.I-CUBE-EE_conf.h
*/
/*---------- EE_MANUAL_CONFIG  -----------*/
#define EE_MANUAL_CONFIG      false

/*---------- EE_SELECTED_PAGE_SECTOR_NUMBER  -----------*/
#define EE_SELECTED_PAGE_SECTOR_NUMBER      16

/*---------- EE_SELECTED_PAGE_SECTOR_SIZE  -----------*/
#define EE_SELECTED_PAGE_SECTOR_SIZE      EE_PAGE_SECTOR_SIZE_1K

/*---------- EE_SELECTED_BANK  -----------*/
#define EE_SELECTED_BANK      FLASH_BANK_1

/*---------- EE_SELECTED_ADDRESS  -----------*/
#define EE_SELECTED_ADDRESS      0x08000000

#ifdef __cplusplus
}
#endif
#endif /* _NIMALTD_I_CUBE_EE_CONF_H_ */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

