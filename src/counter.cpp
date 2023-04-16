#include "counter.h"
#include <Arduino.h>
#include <SPI.h>

static uint8_t buffer_size = 1;
extern SMotorsData motors[NUM_OF_MOTORS];

SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0); 

void initSPI()
{
    SPI.begin();
}

void counter_default_cfg ( void )
{
    digitalWrite(SLAVE_EN, HIGH);
    uint8_t tmp[ 4 ] = { 0 };
    buffer_size = counter_init_advanced( COUNTER_4X_QUAD | COUNTER_FREE_RUN | COUNTER_INDEX_DISABLED | COUNTER_FILTER_CLOCK_DIV1,
                                COUNTER_MODE_32 | COUNTER_ENABLE | COUNTER_FLAG_DISABLE );

    // uint8_t mdr0 = counter_read_mdr0();
    // Serial.print("mdr0 = ");
    // Serial.println(mdr0);
    // uint8_t mdr1 = counter_read_mdr1();
    // Serial.print("mdr1 = ");
    // Serial.println(mdr1);

    counter_write_dtr( buffer_size, tmp );
    counter_clear_cntr ( );
    counter_load_cntr( );
    counter_enable( );
}

void counter_write_mdr0 ( uint8_t settings )
{
    uint8_t data[1] = {settings};
    counter_write_data( COUNTER_CMD_WR | COUNTER_MDR0, data, 1 );
}

void counter_write_mdr1 ( uint8_t settings )
{
    uint8_t data[1] = {settings};
    counter_write_data( COUNTER_CMD_WR | COUNTER_MDR1, data, 1 );
}

void counter_write_dtr ( uint8_t buffer_size, uint8_t* buffer )
{
    counter_write_data( COUNTER_CMD_WR | COUNTER_DTR, buffer, buffer_size );
}

void counter_load_cntr ( )
{
    counter_write_command( COUNTER_CMD_LOAD | COUNTER_CNTR );
}

void counter_load_otr ( )
{
    counter_write_command( COUNTER_CMD_LOAD | COUNTER_OTR );
}

uint8_t counter_read_mdr0 ( )
{
    uint8_t result;

    result = counter_read_register( COUNTER_CMD_RD | COUNTER_MDR0 );

    return result;
}

uint8_t counter_read_mdr1 ( )
{
    uint8_t result;

    result = counter_read_register( COUNTER_CMD_RD | COUNTER_MDR1 );

    return result;
}

int32_t counter_read_otr ( )
{
    uint8_t data_buf[ 4 ];
    uint32_t result;

    counter_read_data( COUNTER_CMD_RD | COUNTER_OTR, data_buf, buffer_size );
    
    result = data_buf[ 0 ];
    
    for ( uint8_t cnt = 1; cnt < buffer_size; cnt++ )
    {
        result <<= 8;
        result |= data_buf[ cnt ];
    }

    return result;
}


void counter_read_cntr ( int motor)
{
//    static int32_t dbg[100] = {0};
//    static uint8_t dbgcnt = 0;
    uint8_t data_buf[ 4 ];
    int32_t result;

    counter_read_data( COUNTER_CMD_RD | COUNTER_CNTR, data_buf, buffer_size );

    result = data_buf[ 0 ];
    
    for ( uint8_t cnt = 1; cnt < buffer_size; cnt++ )
    {
        result <<= 8;
        result |= data_buf[ cnt ];
    }
    
//    dbgcnt++;
//    dbg[dbgcnt] = result;
//    if(dbgcnt >= 100)
//    {
//        dbgcnt = 0;
//        DEBUG_BREAK;
//        while(1);
//    }
    motors[motor].encoder.cnt = result;
    motors[motor].encoder.cnt_last_time_uSec = micros();
    // Serial.print("At counter_read_cntr : ");
    // Serial.println(result);
}

int32_t counter_read_dtr ( )
{
    uint8_t data_buf[ 4 ];
    uint32_t result;

    counter_read_data( COUNTER_CMD_RD | COUNTER_DTR, data_buf, buffer_size );

    result = data_buf[ 0 ];
    
    for ( uint8_t cnt = 1; cnt < buffer_size; cnt++ )
    {
        result <<= 8;
        result |= data_buf[ cnt ];
    }
    
    return result;
}

uint8_t counter_read_str ( )
{
    uint8_t result;

    result = counter_read_register( COUNTER_CMD_RD | COUNTER_STR );

    return result;
}

void counter_clear_mrd0 ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_MDR0 );
}

void counter_clead_mrd1 ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_MDR1 );
}

void counter_clear_cntr ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_CNTR );
}

void counter_clear_str ( )
{
    counter_write_command( COUNTER_CMD_CLR | COUNTER_STR );
}

void counter_initialisation ( )
{
    uint8_t mdr0_set = COUNTER_4X_QUAD | COUNTER_FREE_RUN | COUNTER_INDEX_DISABLED
                       | COUNTER_FILTER_CLOCK_DIV1;
    uint8_t mdr1_set = COUNTER_MODE_32 | COUNTER_DISABLE | COUNTER_FLAG_DISABLE;

    buffer_size = 4;
    counter_write_mdr0( mdr0_set );
    counter_write_mdr1( mdr1_set );
}

uint8_t counter_init_advanced ( uint8_t mdr0_set, uint8_t mdr1_set )
{
    static uint8_t buffer_size;
    counter_write_mdr0( mdr0_set );
    counter_write_mdr1( mdr1_set );

    switch ( mdr1_set & 0x03 )
    {
        case 0 :
        {
            buffer_size = 4;
            break;
        }
        case 1 :
        {
            buffer_size = 3;
            break;
        }
        case 2 :
        {
            buffer_size = 2;
            break;
        }
        case 3 :
        {
            buffer_size = 1;
            break;
        }
    }
    return buffer_size;
}

void counter_enable ( )
{
    uint8_t mdr1_temp = 0;

    mdr1_temp = counter_read_mdr1( ) & 0xFB;
    counter_write_mdr1( mdr1_temp );
}

void counter_disable ( )
{
    uint8_t mdr1_temp = 0;

    mdr1_temp = counter_read_mdr1( ) | 0x04;
    counter_write_mdr1( mdr1_temp );
}

//void counter_chip_enable ( counter_t *ctx )
//{
//    digital_out_high( &ctx->en );
//}

//void counter_chip_disable ( counter_t *ctx )
//{
//    digital_out_low( &ctx->en );
//}

//uint8_t get_int_state ( counter_t *ctx )
//{
//    return digital_in_read( &ctx->int_pin );
//}

// ----------------------------------------------- PRIVATE FUNCTION DEFINITIONS

void counter_write_command ( uint8_t command )
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(SLAVE_CS, LOW);
    SPI.transfer(command);
    digitalWrite(SLAVE_CS, HIGH);
    SPI.endTransaction();
}

uint8_t counter_read_register ( uint8_t command )
{   
    SPI.beginTransaction(spiSettings);
    digitalWrite(SLAVE_CS, LOW);
    SPI.transfer(command);
    uint8_t res = SPI.transfer(0);
    digitalWrite(SLAVE_CS, HIGH);
    SPI.endTransaction();
    return res;
}

void counter_write_data (uint8_t command, uint8_t *data_buff, uint8_t count)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(SLAVE_CS, LOW);
    SPI.transfer(command);
    for(int i = 0; i < count; i++)
    {
        SPI.transfer(data_buff[i]);
    }
    digitalWrite(SLAVE_CS, HIGH);
    SPI.endTransaction();
}

void counter_read_data ( uint8_t command, uint8_t *data_buff, uint8_t count )
{
    uint8_t tx_buff[ 5 ] = {0};
    tx_buff[ 0 ] = command;
    uint8_t res;

    SPI.beginTransaction(spiSettings);
    digitalWrite(SLAVE_CS, LOW);
    for(int i = 0; i < count+1; i++)
    {
        res = SPI.transfer(tx_buff[i]);
        if (i > 0)
        {
            data_buff[i - 1] = res;
        }
    }
    digitalWrite(SLAVE_CS, HIGH);
    SPI.endTransaction();
}


// ------------------------------------------------------------------------- END

