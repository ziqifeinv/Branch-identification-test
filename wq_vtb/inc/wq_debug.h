/****************************************************************************

Copyright(c) 2019 by WuQi Technologies. ALL RIGHTS RESERVED.

This Information is proprietary to WuQi Technologies and MAY NOT
be copied by any method or incorporated into another program without
the express written consent of WuQi. This Information or any portion
thereof remains the property of WuQi. The Information contained herein
is believed to be accurate and WuQi assumes no responsibility or
liability for its use in any way and conveys no license or title under
any patent or copyright and makes no representation or warranty that this
Information is free from patent or copyright infringement.

****************************************************************************/

#ifndef __INCLUDE_WQ_DEBUG_H__
#define __INCLUDE_WQ_DEBUG_H__

#include <stdio.h>

#ifndef WQ_TOPO_DETECT_DEBUG_PRINT
#define WQ_TOPO_DETECT_DEBUG_PRINT  1
#endif

#ifndef WQ_TSFM_DETECT_DEBUG_PRINT
#define WQ_TSFM_DETECT_DEBUG_PRINT  1
#endif

#define iot_wq_printf printf

#if WQ_TOPO_DETECT_DEBUG_PRINT
#if (HW_PLATFORM == HW_PLATFORM_SIMU)
#define wq_dbg_printf(fmt, ...) do{\
        iot_wq_printf("[TOPO DEBUG]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##__VA_ARGS__);\
        }while(0)
#else /* (HW_PLATFORM == HW_PLATFORM_SIMU) */
#define wq_dbg_printf(fmt, arg...) do{\
        iot_wq_printf("[DEBUG]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##arg);\
        }while(0)
#endif /* (HW_PLATFORM == HW_PLATFORM_SIMU) */

#else /* WQ_TOPO_DETECT_DEBUG_PRINT */

#if (HW_PLATFORM == HW_PLATFORM_SIMU)
#define wq_dbg_printf(fmt, ...)
#else /* (HW_PLATFORM == HW_PLATFORM_SIMU) */
#define wq_dbg_printf(fmt, arg...)
#endif /* (HW_PLATFORM == HW_PLATFORM_SIMU) */

#endif /* WQ_TOPO_DETECT_DEBUG_PRINT */

#if (HW_PLATFORM == HW_PLATFORM_SIMU)
#define wq_info_printf(fmt, ...) do{\
        iot_wq_printf("[TOPO INFO]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##__VA_ARGS__);\
        }while(0)
#else /* (HW_PLATFORM == HW_PLATFORM_SIMU) */
#define wq_info_printf(fmt, arg...) do{\
        iot_wq_printf("[INFO]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##arg);\
        }while(0)
#endif /* (HW_PLATFORM == HW_PLATFORM_SIMU) */


#if WQ_TSFM_DETECT_DEBUG_PRINT
#if (HW_PLATFORM == HW_PLATFORM_SIMU)
#define wq_tsfm_dbg_printf(fmt, ...) do{\
        iot_wq_printf("[TSFM DEBUG]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##__VA_ARGS__);\
        }while(0)
#else /* (HW_PLATFORM == HW_PLATFORM_SIMU) */
#define wq_tsfm_dbg_printf(fmt, arg...) do{\
        iot_wq_printf("[TSFM DEBUG]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##arg);\
        }while(0)
#endif /* (HW_PLATFORM == HW_PLATFORM_SIMU) */

#else /* WQ_TSFM_DETECT_DEBUG_PRINT */

#if (HW_PLATFORM == HW_PLATFORM_SIMU)
#define wq_tsfm_dbg_printf(fmt, ...)
#else /* (HW_PLATFORM == HW_PLATFORM_SIMU) */
#define wq_tsfm_dbg_printf(fmt, arg...)
#endif /* (HW_PLATFORM == HW_PLATFORM_SIMU) */

#endif /* WQ_TOPO_DETECT_DEBUG_PRINT */

#if (HW_PLATFORM == HW_PLATFORM_SIMU)
#define wq_tsfm_info_printf(fmt, ...) do{\
        iot_wq_printf("[TSFM INFO]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##__VA_ARGS__);\
        }while(0)
#else /* (HW_PLATFORM == HW_PLATFORM_SIMU) */
#define wq_tsfm_info_printf(fmt, arg...) do{\
        iot_wq_printf("[TSFM INFO]L%04d@%s:" fmt "\n",\
        __LINE__, __FUNCTION__, ##arg);\
        }while(0)
#endif /* (HW_PLATFORM == HW_PLATFORM_SIMU) */

#endif /* __INCLUDE_WQ_DEBUG_H__ */