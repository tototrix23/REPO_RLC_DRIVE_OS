/*
 * impl_spi_motors.c
 *
 *  Created on: 25 sept. 2023
 *      Author: Ch.Leclercq
 */

#include "hal_data.h"
#include "impl_spi_motors.h"


static volatile bool_t g_transfer_complete = FALSE;
static volatile bool_t spi_opened = FALSE;


void sci_b_spi_callback (spi_callback_args_t * p_args)
{
    if (SPI_EVENT_TRANSFER_COMPLETE == p_args->event)
    {
        g_transfer_complete = TRUE;
    }
}

return_t spi_motor_open(void)
{
    fsp_err_t err = FSP_SUCCESS;
    if(spi_opened == TRUE) return X_RET_OK;
    err = R_SCI_B_SPI_Open(&g_sci_spi0_ctrl, &g_sci_spi0_cfg);
    if(err != FSP_SUCCESS)
        return I_RET_ERROR_SPI_OPEN;
    else
    {
        spi_opened = TRUE;
        return X_RET_OK;
    }
}

return_t spi_motor_close(void)
{
    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_B_SPI_Close(&g_sci_spi0_ctrl);
    spi_opened = FALSE;
    if(err != FSP_SUCCESS)
        return I_RET_ERROR_SPI_CLOSE;
    else
        return X_RET_OK;
}



return_t spi_motor_read(char *buffer_tx,char* buffer_rx,uint16_t count)
{
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(buffer_tx != NULL);
    ASSERT(buffer_rx != NULL);
    ASSERT(count > 0);
#endif
    g_transfer_complete = FALSE;
    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_B_SPI_WriteRead(&g_sci_spi0_ctrl, buffer_tx, buffer_rx, count, SPI_BIT_WIDTH_8_BITS);
    if (err != FSP_SUCCESS)
    return I_RET_ERROR_SPI_READ;

    volatile uint32_t tempo = 10000;


    while(1)
    {
        if(TRUE == g_transfer_complete)
        {
            return X_RET_OK;
        }
        else
        {
            if(tempo>0) tempo--;
            if( tempo == 0)
            {
                volatile uint8_t i=0;
                return I_RET_ERROR_SPI_READ_TIMEOUT;
            }
        }
    }

    while (FALSE == g_transfer_complete)
    {
        if(tempo>0) tempo--;
        if( tempo == 0)
        {
            return I_RET_ERROR_SPI_READ_TIMEOUT;
        }
    }
    return X_RET_OK;
}


return_t spi_motor_write(char *buffer_tx,char* buffer_rx,uint16_t count)
{
#if FW_CHECK_PARAM_ENABLE == 1
    ASSERT(buffer_tx != NULL);
    ASSERT(count > 0);
#endif
    g_transfer_complete = FALSE;
    fsp_err_t err = FSP_SUCCESS;
    err = R_SCI_B_SPI_WriteRead(&g_sci_spi0_ctrl, buffer_tx,buffer_rx, count, SPI_BIT_WIDTH_8_BITS);
    if (err != FSP_SUCCESS)
    return I_RET_ERROR_SPI_WRITE;

    volatile uint32_t tempo = 10000;
    while (FALSE == g_transfer_complete)
    {
        if(tempo>0) tempo--;
        if( tempo == 0)
            return I_RET_ERROR_SPI_WRITE_TIMEOUT;
    }
    return X_RET_OK;
}


return_t spi_motor_mot1_cs_active(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MOT1_CS,BSP_IO_LEVEL_LOW);
    return X_RET_OK;
}
return_t spi_motor_mot1_cs_inactive(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MOT1_CS,BSP_IO_LEVEL_HIGH);
    return X_RET_OK;
}
return_t spi_motor_mot2_cs_active(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MOT2_CS,BSP_IO_LEVEL_LOW);
    return X_RET_OK;
}
return_t spi_motor_mot2_cs_inactive(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MOT2_CS,BSP_IO_LEVEL_HIGH);
    return X_RET_OK;
}
