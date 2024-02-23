/*
 * vee.c
 *
 *  Created on: 21 févr. 2024
 *      Author: Ch.Leclercq
 */
#include <_core/c_timespan/c_timespan.h>
#include <_hal/h_time/h_time.h>
#include "vee.h"
#include <system/system.h>
#include <return_codes.h>

#undef  LOG_LEVEL
#define LOG_LEVEL     LOG_LVL_DEBUG
#undef  LOG_MODULE
#define LOG_MODULE    "vee"

#define VEE_PATTERN_SIZE  16
const char vee_pattern_ref[VEE_PATTERN_SIZE] = "VEE PAT V02";
// Declaration des varibles à entretenir dans l'EEPROM virtuelle
static char vee_pattern[VEE_PATTERN_SIZE];

// Déclarations  et prototypes relatifs au fonctionnement des fonctions de traitement
#define DEFAULT_VEE_ID            (65535U)
static rm_vee_status_t p_status =
{ (rm_vee_state_t) 0, 0, 0, 0 };
static volatile bool_t vee_write_flag = FALSE;
return_t vee_write_operation(uint32_t const rec_id, void *p_data, uint32_t bytes);
return_t vee_read_operation(uint32_t const rec_id, uint8_t *ptr, uint32_t size);
return_t vee_format_operation(void);

typedef struct vee_var
{
    // Identifiant de la variable dans le tableau.
    uint16_t id;
    // Pointeur vers les données
    void *ptr_data;
    // Taille de la donnnée à stocker.
    uint32_t size;
} vee_var_st;

//@formatter:off
vee_var_st vee_var_array[EEPROM_VAR_COUNT] =
{
 {
  .id = EEPROM_PATTERN,
  .ptr_data = vee_pattern,
  .size = sizeof(vee_pattern)
 },
 {
  .id = EEPROM_SYSTEM_STATUS,
  .ptr_data = &system_inst,
  .size = sizeof(st_system_t)
 }
};
//@formatter:on


return_t vee_open(void)
{
    return_t ret = F_RET_OK;
    fsp_err_t err_fsp = 0;
    err_fsp = RM_VEE_FLASH_Open (&g_vee_ctrl, &g_vee_cfg);
    if (FSP_SUCCESS != err_fsp)
        ERROR_LOG_AND_RETURN(F_RET_ERROR_VEE_OPENING);
    return ret;
}

return_t vee_close(void)
{
    return_t ret = F_RET_OK;
    fsp_err_t err_fsp = 0;
    err_fsp = RM_VEE_FLASH_Close (&g_vee_ctrl);
    if (FSP_SUCCESS != err_fsp)
        ERROR_LOG_AND_RETURN(F_RET_ERROR_VEE_CLOSING);
    return ret;
}


return_t vee_init(void)
{
    return_t ret = F_RET_OK;
    volatile uint16_t i = 0;
    // Initialisation des variables entretenues dans la VEE
    for (i = 0; i < EEPROM_VAR_COUNT; i++)
    {
       memset (vee_var_array[i].ptr_data, 0x00, vee_var_array[i].size);
    }

    ret = vee_read_by_id (vee_var_array[0].id);

    if (strcmp (vee_pattern, vee_pattern_ref) != 0x0)
    {
        // Sauvegarde du numéro de série
        //ret = vee_read_by_id (err, EEPROM_SENTINEL_SERIAL);

        //uint64_t saved_serial = sentinel_serial;

        //
        for (i = 0; i < EEPROM_VAR_COUNT; i++)
        {
            memset (vee_var_array[i].ptr_data, 0x00, vee_var_array[i].size);
        }

        //sentinel_serial = saved_serial;

        // Effacement de la VEE
        ret = vee_format_operation ();
        // Copie de la chaine de caractère correspondant au firmware de référence
        // Copie du pattern de la VEE
        strcpy (vee_pattern, vee_pattern_ref);

        // Ecriture des données dans la VEE
        for (i = 0; i < EEPROM_VAR_COUNT; i++)
        {
            ret = vee_write_by_id (vee_var_array[i].id);
        }
    }
    else
    {
        for (i = 0; i < EEPROM_VAR_COUNT; i++)
        {
            ret = vee_read_by_id (vee_var_array[i].id);
        }
    }

    return ret;
}


return_t vee_read_by_id(uint16_t id)
{
    return_t ret = F_RET_OK;
    volatile vee_var_st *ptr = NULL;
    uint8_t i = 0;
    for (i = 0; i < EEPROM_VAR_COUNT; i++)
    {
        if (vee_var_array[i].id == id)
            ptr = &vee_var_array[i];
    }

    if (ptr == NULL)
    {
        ERROR_LOG_AND_RETURN(F_RET_ERROR_VEE_NOT_FOUND);
    }
    ret = vee_read_operation (ptr->id, ptr->ptr_data, ptr->size);

    return ret;
}

return_t vee_write_by_id(uint16_t id)
{
    return_t ret = F_RET_OK;
    vee_var_st *ptr = NULL;
    uint8_t i = 0;
    for (i = 0; i < EEPROM_VAR_COUNT; i++)
    {
        if (vee_var_array[i].id == id)
            ptr = &vee_var_array[i];
    }

    if (ptr == NULL)
    {
        ERROR_LOG_AND_RETURN(F_RET_ERROR_VEE_NOT_FOUND);
    }

    ret = vee_write_operation (ptr->id, ptr->ptr_data, ptr->size);

    return ret;
}

return_t vee_read_by_ptr(void *ptr_param)
{
    return_t ret = F_RET_OK;
    vee_var_st *ptr = NULL;
    uint8_t i = 0;
    for (i = 0; i < EEPROM_VAR_COUNT; i++)
    {
        if (vee_var_array[i].ptr_data == ptr_param)
            ptr = &vee_var_array[i];
    }
    if (ptr == NULL)
    {
        ERROR_LOG_AND_RETURN(F_RET_ERROR_VEE_NOT_FOUND);
    }
    ret = vee_read_operation (ptr->id, ptr->ptr_data, ptr->size);

    return ret;
}

return_t vee_write_by_ptr(void *ptr_param)
{
    return_t ret = F_RET_OK;
    if (ptr_param == 0)
        ERROR_LOG_AND_RETURN(X_RET_PARAM_NULL);

    vee_var_st *ptr = NULL;
    uint8_t i = 0;
    for (i = 0; i < EEPROM_VAR_COUNT; i++)
    {
        if (vee_var_array[i].ptr_data == ptr_param)
            ptr = &vee_var_array[i];
    }
    if (ptr == NULL)
    {
        ERROR_LOG_AND_RETURN(F_RET_ERROR_VEE_NOT_FOUND);
    }

    ret = vee_write_operation (ptr->id, ptr->ptr_data, ptr->size);
    return ret;
}

return_t vee_refresh(void)
{
    return_t ret = F_RET_OK;
    return ret;
}



return_t vee_write_operation(uint32_t const rec_id, void *p_data, uint32_t bytes)
{
    if (p_data == 0 || bytes == 0)
        ERROR_LOG_AND_RETURN(X_RET_PARAM_NULL);

    return_t ret = X_RET_OK;
    ret = vee_open ();
    if (ret != X_RET_OK)
        goto end;

    vee_write_flag = FALSE;
    fsp_err_t err_fsp = RM_VEE_FLASH_RecordWrite (&g_vee_ctrl, rec_id, p_data, bytes);
    if (FSP_SUCCESS != err_fsp)
    {
        ret = F_RET_ERROR_VEE_WRITING_OPERATION;
        goto end;
    }

    c_timespan_t ts;
    c_timespan_init(&ts);
    h_time_update ((c_timespan_h) &ts);

    while (TRUE != vee_write_flag)
    {
        bool_t elapsed = FALSE;
        h_time_is_elapsed_ms (&ts, 3000,&elapsed);
        if (elapsed == TRUE)
        {
            /* we have reached to a scenario where callback event did not occur */
            vee_write_flag = FALSE;
            ERROR_LOG_AND_SET(ret,F_RET_ERROR_VEE_WRITING_TIMEOUT);
            goto end;
        }
    }

    /* Reset the flag.*/

    end: vee_close ();
    vee_write_flag = FALSE;
    return ret;
}

return_t vee_read_operation(uint32_t const rec_id, uint8_t *ptr, uint32_t size)
{

    if (ptr == 0 || size == 0)
        ERROR_LOG_AND_RETURN(X_RET_PARAM_NULL);

    return_t ret = X_RET_OK;
    ret = vee_open ();
    if (ret != X_RET_OK)
        goto end;

    uint32_t out_len = 0;
    void *p_record_data = NULL;
    uint32_t i = 0;
    /* Get a pointer to the record that is stored in data flash. */
    fsp_err_t err_fsp = RM_VEE_FLASH_RecordPtrGet (&g_vee_ctrl, rec_id, (uint8_t**) &p_record_data, &out_len);
    if (FSP_SUCCESS == err_fsp)
    {
        if (out_len == size)
        {
            for (i = 0; i < out_len; i++)
            {
                *ptr = *((uint8_t*) p_record_data);
                ptr++;
                p_record_data = ((uint8_t*) p_record_data + 1);
            }
        }
    }
    /*This condition will occur, when user tries to read data from the Record_ID
     * where no data was written previously or flash is blank.*/
    else if (FSP_ERR_NOT_FOUND == err_fsp)
    {
        ERROR_LOG_AND_SET(ret,F_RET_ERROR_VEE_NOT_FOUND);
        goto end;
    }
    else
    {
        ERROR_LOG_AND_SET(ret,F_RET_ERROR_VEE_READING);
        goto end;
    }
    end: vee_close ();
    return ret;
}


return_t vee_format_operation(void)
{
    return_t ret = X_RET_OK;
    ret = vee_open ();
    if (ret != X_RET_OK)
        goto end;

    uint8_t ref_data = 0;
    /* Start a manual format operation.*/
    fsp_err_t err_fsp = RM_VEE_FLASH_Format (&g_vee_ctrl, &ref_data);
    if (FSP_SUCCESS != err_fsp)
    {
        ERROR_LOG_AND_SET(ret,F_RET_ERROR_VEE_FORMATING);
        //ret = F_RET_ERROR_VEE_FORMATING;
        goto end;
    }

    /* Get the current status of the driver.*/
    err_fsp = RM_VEE_FLASH_StatusGet (&g_vee_ctrl, &p_status);
    if (FSP_SUCCESS != err_fsp)
    {
        ERROR_LOG_AND_SET(ret,F_RET_ERROR_VEE_GETTING_STATUS);
        goto end;
    }
    /* Compare Last ID written with Default ID.*/
    if (DEFAULT_VEE_ID == p_status.last_id)
    {
        ret =  X_RET_OK;
        goto end;
    }
    else
    {
        ERROR_LOG_AND_SET(ret,F_RET_ERROR_VEE_NOT_ERASED);
        goto end;
    }
    end: vee_close ();
    vee_write_flag = FALSE;
    return ret;
}

void vee_callback(rm_vee_callback_args_t *p_args)
{
    if ((NULL != p_args) && (RM_VEE_STATE_READY == p_args->state))
    {
        vee_write_flag = TRUE;
    }
}

