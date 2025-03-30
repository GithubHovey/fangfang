/*------------------------------------------------------------------------------
 * @file    PLATFORM_ERR.H
 * @author  Hovey https://space.bilibili.com/33582262?spm_id_from=333.1007.0.0
 * @date    2025/03/30 22:53:01
 * @brief   
 * -----------------------------------------------------------------------------
 * @attention 
 
------------------------------------------------------------------------------*/

/* Define to prevent recursive inclusion  ------------------------------------*/
#ifndef __PLATFORM_ERR_H
#define __PLATFORM_ERR_H

/* Files includes  -----------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
typedef enum {
    PLATFORM_OK = 0,
    PLATFORM_ERROR,
    PLATFORM_INVALID_ARG,
    PLATFORM_NOT_INIT,
    PLATFORM_TIMEOUT
} platform_err_t;

/* Variables -----------------------------------------------------------------*/


/* Functions ----------------------------------------------------------------*/

#endif
