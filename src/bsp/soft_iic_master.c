//===============================================================//
//= Include files.                                              =//
//===============================================================//
#include <bsp/soft_iic_master.h>
#include <stdbool.h>

//===============================================================//
//= Macro definition.                                           =//
//===============================================================//

//===============================================================//
//= Static function declare.                                    =//
//===============================================================//

//===============================================================//
//= Static variable declaration.                                =//
//===============================================================//

//===============================================================//
//= Function definition.                                        =//
//===============================================================//
/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_start                   **/
/** Purpose:        Generate a IIC start signal.                **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** Return:         None.                                       **/
/*****************************************************************/
void soft_iic_master_bsp_start(const soft_iic_master_t* iic)
{
    iic->sda_set_dir(SOFT_IIC_DOUT);
    iic->scl_set_bit(1);
    iic->delay(1);
    iic->sda_set_bit(1);
    iic->delay(1);
    iic->sda_set_bit(0);
    iic->delay(1);
    iic->scl_set_bit(0);
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_stop                    **/
/** Purpose:        Generate a IIC start signal.                **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** Return:         None.                                       **/
/*****************************************************************/
void soft_iic_master_bsp_stop(const soft_iic_master_t* iic)
{
    iic->sda_set_dir(SOFT_IIC_DOUT);
    iic->scl_set_bit(0);
    iic->delay(1);
    iic->sda_set_bit(0);
    iic->delay(1);
    iic->scl_set_bit(1);
    iic->delay(1);
    iic->sda_set_bit(1);
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_ack                     **/
/** Purpose:        Generate a ACK signal on IIC bus.           **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** Return:         None.                                       **/
/*****************************************************************/
void soft_iic_master_bsp_ack(const soft_iic_master_t* iic)
{
    iic->sda_set_dir(SOFT_IIC_DOUT);
    iic->scl_set_bit(0);
    iic->delay(1);
    iic->sda_set_bit(0);
    iic->delay(1);
    iic->scl_set_bit(1);
    iic->delay(1);
    iic->scl_set_bit(0);
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_nack                    **/
/** Purpose:        Generate a nACK signal on IIC bus.          **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** Return:         None.                                       **/
/*****************************************************************/
void soft_iic_master_bsp_nack(const soft_iic_master_t* iic)
{
    iic->sda_set_dir(SOFT_IIC_DOUT);
    iic->scl_set_bit(0);
    iic->sda_set_bit(1);
    iic->delay(2);
    iic->scl_set_bit(0);
    iic->delay(2);
    iic->scl_set_bit(0);
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_wait_ack                **/
/** Purpose:        Waiting a ACK signal on IIC bus.            **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** Return:         Ack bit value.                              **/
/*****************************************************************/
uint8_t soft_iic_master_bsp_wait_ack(const soft_iic_master_t* iic)
{
    iic->scl_set_bit(0);
    iic->delay(1);
    iic->sda_set_bit(1);
    iic->sda_set_dir(SOFT_IIC_DIN);
    iic->delay(1);
    iic->scl_set_bit(1);
    iic->delay(1);
    if(iic->sda_get_bit() == 1)
    {
        return 1;
    }
    else
    {
        iic->scl_set_bit(0);
        return 0;
    }
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_write_byte              **/
/** Purpose:        Write a byte to IIC bus.                    **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** @ [in]byte:     One byte data.                              **/
/** Return:         None.                                       **/
/*****************************************************************/
void soft_iic_master_bsp_write_byte(const soft_iic_master_t* iic, uint8_t byte)
{
    uint8_t i;
    iic->scl_set_bit(0);
    iic->sda_set_dir(SOFT_IIC_DOUT);
    for(i = 0; i < 8; i++)
    {
        if(byte & 0x80)
        {
            iic->sda_set_bit(1);
        }
        else
        {
            iic->sda_set_bit(0);
        }
        byte <<= 1;
        iic->delay(1);
        iic->scl_set_bit(1);
        iic->delay(1);
        iic->scl_set_bit(0);
    }
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_read_byte               **/
/** Purpose:        Initialize and config SPI controler.        **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** Return:         Read byte.                                  **/
/*****************************************************************/
uint8_t soft_iic_master_bsp_read_byte(const soft_iic_master_t* iic)
{
    uint8_t i;
    uint8_t receive = 0;
    iic->sda_set_bit(1);
    iic->sda_set_dir(SOFT_IIC_DIN);
    for(i = 0; i < 8; i++)
    {
        iic->scl_set_bit(0);
        iic->delay(1);
        iic->scl_set_bit(1);
        iic->delay(1);
        receive <<= 1;
        if(iic->sda_get_bit() == 1)
        {
            receive++;
        }
    }
    return receive;
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_flush_bus               **/
/** Purpose:        Clean and flush incomplete session.         **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** Return:         None.                                       **/
/*****************************************************************/
void soft_iic_master_bsp_flush_bus(const soft_iic_master_t* iic)
{
    soft_iic_master_bsp_write_byte(iic, 0xFF);
    soft_iic_master_bsp_stop(iic);
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_write_data              **/
/** Purpose:        Write data to bus.                          **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** @ [in]addr:     Device register address.                    **/
/** @ [in]data:     Data header pointer.                        **/
/** @ [in]len:      Data length.                                **/
/** Return:         The number of bytes written.                **/
/*****************************************************************/
uint16_t soft_iic_master_bsp_write_data(const soft_iic_master_t* iic, uint8_t addr, const void* data, uint16_t len)
{
    uint16_t write_num = 0;
    if(len > 0)
    {
        // IIC Start
        soft_iic_master_bsp_start(iic);
        // Write device address.
        soft_iic_master_bsp_write_byte(iic, iic->addr);
        soft_iic_master_bsp_wait_ack(iic);
        // Write register address.
        soft_iic_master_bsp_write_byte(iic, addr);
        soft_iic_master_bsp_wait_ack(iic);
        // Write data.
        uint8_t* byte_ptr = (uint8_t*)data;
        while(write_num < len)
        {
            soft_iic_master_bsp_write_byte(iic, *byte_ptr);
            if(0 == soft_iic_master_bsp_wait_ack(iic))
            {
                ++byte_ptr;
                ++write_num;
            }
            else
            {
                break;
            }
        }
        soft_iic_master_bsp_stop(iic);
    }
    return write_num;
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_fill_data               **/
/** Purpose:        Write a byte to bus repeatedly.             **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** @ [in]addr:     Device register address.                    **/
/** @ [in]data:     Byte value.                                 **/
/** @ [in]num:      Byte repeat times.                          **/
/** Return:         The number of bytes written.                **/
/*****************************************************************/
uint16_t soft_iic_master_bsp_fill_data(const soft_iic_master_t* iic, uint8_t addr, uint8_t data, uint16_t num)
{
    uint16_t write_num = 0;
    if(num > 0)
    {
        // IIC Start
        soft_iic_master_bsp_start(iic);
        // Write device address.
        soft_iic_master_bsp_write_byte(iic, iic->addr);
        soft_iic_master_bsp_wait_ack(iic);
        // Write register address.
        soft_iic_master_bsp_write_byte(iic, addr);
        soft_iic_master_bsp_wait_ack(iic);
        // Write data.
        while(write_num < num)
        {
            soft_iic_master_bsp_write_byte(iic, data);
            if(0 == soft_iic_master_bsp_wait_ack(iic))
            {
                ++write_num;
            }
            else
            {
                break;
            }
        }
        soft_iic_master_bsp_stop(iic);
    }
    return write_num;
}

/*****************************************************************/
/** Function Name:  soft_iic_master_bsp_read_data               **/
/** Purpose:        Read data from bus.                         **/
/** Params:                                                     **/
/** @ [in]iic:      IIC structure pointer.                      **/
/** @ [in]addr:     Device register address.                    **/
/** @ [in]data:     Data buffer pointer.                        **/
/** @ [in]len:      Buffer length.                              **/
/** Return:         The number of bytes actually read.          **/
/*****************************************************************/
uint16_t soft_iic_master_bsp_read_data(const soft_iic_master_t* iic, uint8_t addr, void* buffer, uint16_t len)
{
    uint8_t read_num = 0;

    if(len > 0)
    {
        // IIC start
        soft_iic_master_bsp_start(iic);
        do
        {
            // Write device address.
            soft_iic_master_bsp_write_byte(iic, iic->addr);
            if(soft_iic_master_bsp_wait_ack(iic))
            {
                break;
            }
            // Write register address.
            soft_iic_master_bsp_write_byte(iic, addr);
            if(soft_iic_master_bsp_wait_ack(iic))
            {
                break;
            }
            // IIC start
            soft_iic_master_bsp_start(iic);
            // Write device address.
            soft_iic_master_bsp_write_byte(iic, iic->addr + 1);
            if(soft_iic_master_bsp_wait_ack(iic))
            {
                break;
            }
            read_num = 1;
            uint8_t* byte_ptr = (uint8_t*)buffer;
            while(read_num < len)
            {
                *byte_ptr = soft_iic_master_bsp_read_byte(iic);
                soft_iic_master_bsp_ack(iic);
                ++byte_ptr;
                ++read_num;
            }
            /* Remaining one-byte. */
            *byte_ptr = soft_iic_master_bsp_read_byte(iic);
            soft_iic_master_bsp_nack(iic);
        }while(false);
        soft_iic_master_bsp_stop(iic);
    }
    return read_num;
}
