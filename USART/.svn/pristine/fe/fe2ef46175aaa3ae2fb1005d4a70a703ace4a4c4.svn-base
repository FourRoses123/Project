#ifndef INC_SDCONFIG_H_
#define INC_SDCONFIG_H_

#include "main.h"
#include "fatfs.h"
#include "sdcard.h"

#define CONFIG_SECTION  "[svpinger]"
#define RECODE_KEY  "Receivercode"
#define PEAKTH_KEY  "PEAKTH"
#define ALMSTTH_KEY  "ALMSTTH"
#define PKWND_KEY	"PKWND"

typedef struct {
    uint16_t receiver_code;   // 对应
    uint16_t peakth;       // 对应
    uint16_t almstth;     // 对应
    uint16_t pkwnd;    // 对应
} SD_Config;

FRESULT read_config(char *buffer, uint16_t buf_size);
SD_Config parse_config(const char *config_text);
void apply_config(SD_Config *config);
void create_default_config(void);
SD_Config load_and_apply_config(void);

#endif
