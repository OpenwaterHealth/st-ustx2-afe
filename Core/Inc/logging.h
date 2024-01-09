/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOGGING_H
#define __LOGGING_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

 void init_dma_logging();
 bool is_using_dma();

#ifdef __cplusplus
}
#endif

#endif /* __PACKBURN_LOGGING_H*/
