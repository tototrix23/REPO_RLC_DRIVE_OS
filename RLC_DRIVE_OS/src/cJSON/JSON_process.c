/*
 * JSON_process.c
 *
 *  Created on: 27 févr. 2024
 *      Author: Ch.Leclercq
 */
#include <cJSON/cJSON.h>
#include "JSON_process.h"
#include <rtc/rtc.h>
#include <return_codes.h>

return_t json_process_get_datetime(char *ptr)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }
    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    cJSON *json_unix = cJSON_GetObjectItemCaseSensitive(json_data, "timestamp_unix");
    if(json_unix == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }



    if(!cJSON_IsNumber(json_unix))
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }

    if((uint64_t)json_unix->valuedouble != 0)
    {
        rtc_set((uint64_t)json_unix->valuedouble);
    }



    end:
    cJSON_Delete(ptr_json);
    return ret;
}

return_t json_process_get_serial(char *ptr)
{
    return_t ret = X_RET_OK;

    const cJSON *json_data = NULL;
    cJSON *ptr_json = cJSON_Parse(ptr);
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_PARSE;
        goto end;
    }
    json_data = cJSON_GetObjectItemCaseSensitive(ptr_json, "data");
    if(ptr_json == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }

    cJSON *json_serial = cJSON_GetObjectItemCaseSensitive(json_data, "serial");
    if(json_serial == NULL)
    {
        ret = F_RET_JSON_FIND_OBJECT;
        goto end;
    }



    if(!cJSON_IsString(json_serial))
    {
        ret = F_RET_JSON_BAD_TYPE;
        goto end;
    }





    end:
    cJSON_Delete(ptr_json);
    return ret;
}
