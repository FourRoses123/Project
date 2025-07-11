#include "sdconfig.h"
#include "user_spi.h"
#include "user_usart.h"

FRESULT read_config(char *buffer, uint16_t buf_size) //读配置文件
{
    FIL fil;
    FRESULT res = f_open(&fil, CONFIG_FILE, FA_READ);
    if(res == FR_OK)
    {
        UINT br;
        f_read(&fil, buffer, buf_size - 1, &br);
        buffer[br] = '\0';
        f_close(&fil);
    }
    return res;
}

SD_Config parse_config(const char *config_text) //寻找配置文件中的相关参数，查看是否有修改，将参数保存并返回
{
	SD_Config config =
	{
		.receiver_code = receivercode,
		.peakth = PEAKTH,
		.almstth = ALMSTTH,
		.pkwnd = PKWND
	};

    const char *section = strstr(config_text, CONFIG_SECTION);
    if (!section) return config;

    const char *ptr = section + strlen(CONFIG_SECTION);

    while (*ptr != '\0' && *ptr != '[')
    {
        if (strncmp(ptr, RECODE_KEY, strlen(RECODE_KEY)) == 0)
        {
            ptr += strlen(RECODE_KEY) + 1;
            config.receiver_code = atoi(ptr);
        }
        else if (strncmp(ptr, PEAKTH_KEY, strlen(PEAKTH_KEY)) == 0)
        {
            ptr += strlen(PEAKTH_KEY) + 1;
            config.peakth = atoi(ptr);
        }
        else if (strncmp(ptr, ALMSTTH_KEY, strlen(ALMSTTH_KEY)) == 0)
        {
            ptr += strlen(ALMSTTH_KEY) + 1;
            config.almstth = atoi(ptr);
        }
        else if (strncmp(ptr, PKWND_KEY, strlen(PKWND_KEY)) == 0)
		{
			ptr += strlen(PKWND_KEY) + 1;
			config.pkwnd = atoi(ptr);
		}

        while (*ptr != '\n' && *ptr != '\0') ptr++;
        if (*ptr == '\n') ptr++;
    }

    return config;
}

void apply_config(SD_Config *config) // 应用新配置
{
	receivercode = config->receiver_code;
	PEAKTH = config->peakth;
	ALMSTTH = config->almstth;
	PKWND = config->pkwnd;
	ASIC_CMD(0x01, PEAKTH);
	ASIC_CMD(0x02, ALMSTTH);
	ASIC_CMD(0x04, PKWND);
}

void create_default_config(void) //创建配置文件
{
    FIL fil;
    if(f_open(&fil, CONFIG_FILE, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
        const char *default_config =
            "[svpinger]\n"
            "Receivercode=1\n"
            "PEAKTH=200\n"
        	"ALMSTTH=32\n"
        	"PKWND=5000";

        UINT bw;
        f_write(&fil, default_config, strlen(default_config), &bw);
        f_close(&fil);
    }
}

SD_Config load_and_apply_config(void) //读取并应用配置
{
    char config_buf[64];
    SD_Config config =
    {
        .receiver_code = receivercode,
        .peakth = PEAKTH,
        .almstth = ALMSTTH,
        .pkwnd = PKWND
    };

    if(read_config(config_buf, sizeof(config_buf)) == FR_OK)
    {
        config = parse_config(config_buf);
        apply_config(&config);
    }
    return config;
}
