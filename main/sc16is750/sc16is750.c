#include <stdio.h>	
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>	
#include <stdlib.h>  
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c.h"
#include "I2C_Driver.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sc16is750.h"

// i2c stuff
#if CONFIG_I2C_PORT_0
#define I2C_NUM I2C_NUM_0
#elif CONFIG_I2C_PORT_1
#define I2C_NUM I2C_NUM_1
#else
#define I2C_NUM I2C_NUM_0 // if spi is selected
#endif
#define I2C_FREQ 400000 // SC16IS75x supports 400 kbit/s maximum speed

// spi stuff
#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
#define HOST_ID SPI3_HOST
#else
#define HOST_ID SPI2_HOST // If i2c is selected
#endif
#define SPI_FREQ 1000000 // SC16IS75x supports 4 Mbit/s maximum SPI clock speed

#define TAG "SC16IS752"

void SC16IS750_init(SC16IS750_t * dev, uint8_t protocol, uint8_t address, int channels)
{
	dev->protocol = protocol;
	if ( dev->protocol == SC16IS750_PROTOCOL_I2C ) {
		dev->address_i2c = address;
	} else {
		dev->address_sspin = address;
	}
	dev->peek_flag = 0;
	dev->channels = channels;
	//dev->timeout = 1000;
}

void SC16IS750_HardReset(SC16IS750_t * dev, int16_t reset)
{
	if (reset >= 0 ) {
		gpio_reset_pin( reset );
		gpio_set_direction( reset, GPIO_MODE_OUTPUT );
		gpio_set_level( reset, 1 );
		vTaskDelay( pdMS_TO_TICKS( 100 ) );
		gpio_set_level( reset, 0 );
		vTaskDelay( pdMS_TO_TICKS( 100 ) );
		gpio_set_level( reset, 1 );
	}
}

void SC16IS750_i2c(SC16IS750_t * dev, int16_t sda, int16_t scl)
{
    // NavCore integration note:
    // I2C bus is initialized elsewhere (I2C_Driver). This function is kept for API compatibility.
    (void)dev;
    (void)sda;
    (void)scl;
}


void SC16IS750_spi(SC16IS750_t * dev, int16_t mosi, int16_t miso, int16_t sclk)
{
	esp_err_t ret;
	spi_bus_config_t spi_bus_config = {
		.mosi_io_num = mosi,
		.miso_io_num = miso,
		.sclk_io_num = sclk,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize( HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO );
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg={
		.clock_speed_hz = SPI_FREQ,
		.mode = 0,
		.spics_io_num = dev->address_sspin,
#if 0
		.flags = SPI_DEVICE_NO_DUMMY,
#endif
		.queue_size = 7 
	};

	spi_device_handle_t handle;
	ret = spi_bus_add_device( HOST_ID, &devcfg, &handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	dev->handle = handle;
}

void SC16IS750_begin(SC16IS750_t * dev, uint32_t baud_A, uint32_t baud_B, long crystal_freq)
{
	dev->crystal_freq = crystal_freq;
	//SC16IS750_ResetDevice(dev);
	SC16IS750_FIFOEnable(dev, SC16IS750_CHANNEL_A, 1);
	SC16IS750_SetBaudrate(dev, SC16IS750_CHANNEL_A, baud_A);
	SC16IS750_SetLine(dev, SC16IS750_CHANNEL_A, 8, 0, 1);
	if (dev->channels == SC16IS750_DUAL_CHANNEL) {
		SC16IS750_FIFOEnable(dev, SC16IS750_CHANNEL_B, 1);
		SC16IS750_SetBaudrate(dev, SC16IS750_CHANNEL_B, baud_B);
		SC16IS750_SetLine(dev, SC16IS750_CHANNEL_B, 8, 0, 1);
	}
}

int SC16IS750_available(SC16IS750_t * dev, uint8_t channel)
{
	return SC16IS750_FIFOAvailableData(dev, channel);
}

uint8_t SC16IS750_linestate(SC16IS750_t * dev, uint8_t channel)
{
	return SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_LSR);
}

int SC16IS750_read(SC16IS750_t * dev, uint8_t channel)
{
	if ( dev->peek_flag == 0) {
		return SC16IS750_ReadByte(dev, channel);
	} else {
		dev->peek_flag = 0;
		return dev->peek_buf;
	}
}

void SC16IS750_write(SC16IS750_t * dev, uint8_t channel, uint8_t val)
{
	SC16IS750_WriteByte(dev, channel, val);
}

void SC16IS750_pinMode(SC16IS750_t * dev, uint8_t pin, uint8_t i_o)
{
	SC16IS750_GPIOSetPinMode(dev, pin, i_o);
}

void SC16IS750_digitalWrite(SC16IS750_t * dev, uint8_t pin, uint8_t value)
{
	SC16IS750_GPIOSetPinState(dev, pin, value);
}

uint8_t SC16IS750_digitalRead(SC16IS750_t * dev, uint8_t pin)
{
	return SC16IS750_GPIOGetPinState(dev, pin);
}


uint8_t SC16IS750_ReadRegister(SC16IS750_t * dev, uint8_t channel, uint8_t reg_addr)
{
    uint8_t result = 0;

    if (dev->protocol == SC16IS750_PROTOCOL_I2C) {
        // SC16IS7xx I2C register address byte: A[7:3]=reg, A[2:1]=channel (A=0,B=1) <<1, A0=0
        uint8_t addr = (uint8_t)((reg_addr << 3) | (channel << 1));
        // I2C_Driver expects 7-bit slave address in dev->address_i2c
        (void)I2C_Read(dev->address_i2c, addr, &result, 1);
        return result;
    } else if (dev->protocol == SC16IS750_PROTOCOL_SPI) {
        // Keep original SPI implementation
        uint8_t spi_addr = (uint8_t)((reg_addr << 3) | (channel << 1) | 0x80); // read
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        uint8_t txdata[2] = { spi_addr, 0xFF };
        uint8_t rxdata[2] = { 0 };
        t.length = 16;
        t.tx_buffer = txdata;
        t.rx_buffer = rxdata;
        spi_device_transmit(dev->handle, &t);
        return rxdata[1];
    }

    return result;
}


void SC16IS750_WriteRegister(SC16IS750_t * dev, uint8_t channel, uint8_t reg_addr, uint8_t val)
{
    if (dev->protocol == SC16IS750_PROTOCOL_I2C) {
        uint8_t addr = (uint8_t)((reg_addr << 3) | (channel << 1));
        (void)I2C_Write(dev->address_i2c, addr, &val, 1);
        return;
    } else if (dev->protocol == SC16IS750_PROTOCOL_SPI) {
        uint8_t spi_addr = (uint8_t)((reg_addr << 3) | (channel << 1)); // write
        spi_transaction_t t;
        memset(&t, 0, sizeof(t));
        uint8_t txdata[2] = { spi_addr, val };
        t.length = 16;
        t.tx_buffer = txdata;
        spi_device_transmit(dev->handle, &t);
        return;
    }
}


int16_t SC16IS750_SetBaudrate(SC16IS750_t * dev, uint8_t channel, uint32_t baudrate) //return error of baudrate parts per thousand
{
	uint16_t divisor;
	uint8_t prescaler;
	uint32_t actual_baudrate;
	int16_t error;
	uint8_t temp_lcr;
	if ( (SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_MCR)&0x80) == 0) { //if prescaler==1
		prescaler = 1;
	} else {
		prescaler = 4;
	}

	ESP_LOGD(TAG, "baudrate=%"PRIu32, baudrate);
	//divisor = (SC16IS750_CRYSTCAL_FREQ/prescaler)/(baudrate*16);
	uint32_t divisor1 = dev->crystal_freq/prescaler;
	ESP_LOGD(TAG, "dev->crystal_freq=%ld prescaler=%d divisor1=%"PRIu32, dev->crystal_freq, prescaler, divisor1);
	uint32_t max_baudrate = divisor1/16;
	ESP_LOGD(TAG, "max_baudrate=%"PRIu32, max_baudrate);
	uint32_t divisor2 = baudrate*16;
	ESP_LOGD(TAG, "divisor2=%"PRIu32, divisor2);
	if (divisor2 > divisor1) {
		ESP_LOGE(TAG, "This baudrate (%"PRIu32") is not support",baudrate);
		return 0;
	}
	//divisor = (dev->crystal_freq/prescaler)/(baudrate*16);
	//divisor = divisor1/divisor2;
	double wk = (double)divisor1/(double)divisor2;
	divisor = wk + 0.999;
	ESP_LOGD(TAG, "baudrate=%"PRIu32" divisor=%d",baudrate, divisor);

	temp_lcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_LCR);
	temp_lcr |= 0x80;
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_LCR, temp_lcr);
	//write to DLL
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_DLL, (uint8_t)divisor);
	//write to DLH
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_DLH, (uint8_t)(divisor>>8));
	temp_lcr &= 0x7F;
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_LCR, temp_lcr);

#if 0
	actual_baudrate = divisor1 / divisor2;
	error = ((float)actual_baudrate-baudrate)*1000/baudrate;
#endif
	actual_baudrate = (divisor1/divisor)/16;
	error = baudrate - actual_baudrate;
	ESP_LOGD(TAG, "actual_baudrate=%"PRIu32" error=%d", actual_baudrate, error);
	if (error != 0) {
		ESP_LOGW(TAG, "baudrate=%"PRIu32" actual_baudrate=%"PRIu32, baudrate, actual_baudrate);
	}

#ifdef	SC16IS750_DEBUG_PRINT
	printf("Desired baudrate: ");
	printf("%x\n", baudrate);
	printf("Calculated divisor: ");
	printf("%x\n", divisor);
	printf("Actual baudrate: ");
	printf("%x\n", actual_baudrate);
	printf("Baudrate error: ");
	printf("%x\n", error);
#endif

	return error;

}

void SC16IS750_SetLine(SC16IS750_t * dev, uint8_t channel, uint8_t data_length, uint8_t parity_select, uint8_t stop_length )
{
	uint8_t temp_lcr;
	temp_lcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_LCR);
	temp_lcr &= 0xC0; //Clear the lower six bit of LCR (LCR[0] to LCR[5]
#ifdef	SC16IS750_DEBUG_PRINT
	printf("LCR Register:0x");
	printf("%x\n", temp_lcr);
#endif
	switch (data_length) {			//data length settings
	case 5:
		break;
	case 6:
		temp_lcr |= 0x01;
		break;
	case 7:
		temp_lcr |= 0x02;
		break;
	case 8:
		temp_lcr |= 0x03;
		break;
	default:
		temp_lcr |= 0x03;
		break;
	}

	if ( stop_length == 2 ) {
	temp_lcr |= 0x04;
	}

	switch (parity_select) {		//parity selection length settings
	case 0:				//no parity
		 break;
	case 1:				//odd parity
		temp_lcr |= 0x08;
		break;
	case 2:				//even parity
		temp_lcr |= 0x18;
		break;
	case 3:				//force '1' parity
		temp_lcr |= 0x03;
		break;
	case 4:				//force '0' parity
		break;
	default:
		break;
	}

	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_LCR, temp_lcr);
}

void SC16IS750_GPIOSetPinMode(SC16IS750_t * dev, uint8_t pin_number, uint8_t i_o)
{
	uint8_t temp_iodir;

	temp_iodir = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IODIR);
	if ( i_o == OUTPUT ) {
		temp_iodir |= (0x01 << pin_number);
	} else {
		temp_iodir &= (uint8_t)~(0x01 << pin_number);
	}

	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IODIR, temp_iodir);
	return;
}

void SC16IS750_GPIOSetPinState(SC16IS750_t * dev, uint8_t pin_number, uint8_t pin_state)
{
	uint8_t temp_iostate;

	temp_iostate = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);
	if ( pin_state == 1 ) {
		temp_iostate |= (0x01 << pin_number);
	} else {
		temp_iostate &= (uint8_t)~(0x01 << pin_number);
	}

	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE, temp_iostate);
	return;
}


uint8_t SC16IS750_GPIOGetPinState(SC16IS750_t * dev, uint8_t pin_number)
{
	uint8_t temp_iostate;

	temp_iostate = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);
	if ( ( temp_iostate & (0x01 << pin_number) ) == 0 ) {
		return 0;
	}
	return 1;
}

uint8_t SC16IS750_GPIOGetPortState(SC16IS750_t * dev)
{

	return SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE);

}

void SC16IS750_GPIOSetPortMode(SC16IS750_t * dev, uint8_t port_io)
{
	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IODIR, port_io);
	return;
}

void SC16IS750_GPIOSetPortState(SC16IS750_t * dev, uint8_t port_state)
{
	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOSTATE, port_state);
	return;
}

void SC16IS750_SetPinInterrupt(SC16IS750_t * dev, uint8_t io_int_ena)
{
	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOINTENA, io_int_ena);
	return;
}

void SC16IS750_ResetDevice(SC16IS750_t * dev)
{
	uint8_t reg;

	reg = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
	reg |= 0x08;
	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, reg);

	return;
}

void SC16IS750_ModemPin(SC16IS750_t * dev, uint8_t gpio) //gpio == 0, gpio[7:4] are modem pins, gpio == 1 gpio[7:4] are gpios
{
	uint8_t temp_iocontrol;

	temp_iocontrol = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
	if ( gpio == 0 ) {
		temp_iocontrol |= 0x02;
	} else {
		temp_iocontrol &= 0xFD;
	}
	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, temp_iocontrol);

	return;
}

void SC16IS750_GPIOLatch(SC16IS750_t * dev, uint8_t latch)
{
	uint8_t temp_iocontrol;

	temp_iocontrol = SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
	if ( latch == 0 ) {
		temp_iocontrol &= 0xFE;
	} else {
		temp_iocontrol |= 0x01;
	}
	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, temp_iocontrol);

	return;
}

void SC16IS750_InterruptControl(SC16IS750_t * dev, uint8_t channel, uint8_t int_ena)
{
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_IER, int_ena);
}

uint8_t SC16IS750_InterruptPendingTest(SC16IS750_t * dev, uint8_t channel)
{
	return (SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_IIR) & 0x01);
}

int16_t SC16IS750_InterruptEventTest(SC16IS750_t * dev, uint8_t channel)
{
	uint8_t irq_src;

	irq_src = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_IIR);
	//irq_src = (irq_src >> 1);
	//irq_src &= 0x3F;
	irq_src &= 0x3E;

	switch (irq_src) {
		case 0x06:			//Receiver Line Status Error
			return SC16IS750_RECEIVE_LINE_STATUS_ERROR;
		case 0x0c:			//Receiver time-out interrupt
			return SC16IS750_RECEIVE_TIMEOUT_INTERRUPT;
		case 0x04:			//RHR interrupt
			return SC16IS750_RHR_INTERRUPT;
		case 0x02:			//THR interrupt
			return SC16IS750_THR_INTERRUPT;
		case 0x00:			//modem interrupt;
			return SC16IS750_MODEM_INTERRUPT;
		case 0x30:			//input pin change of state
			return SC16IS750_INPUT_PIN_CHANGE_STATE;
		case 0x10:			//XOFF
			return SC16IS750_RECEIVE_XOFF;
		case 0x20:			//CTS,RTS
			return SC16IS750_CTS_RTS_CHANGE;
		default:
			return -1;
	}
	return -1;
}

void SC16IS750_FIFOEnable(SC16IS750_t * dev, uint8_t channel, uint8_t fifo_enable)
{
	uint8_t temp_fcr;

	temp_fcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_FCR);

	if (fifo_enable == 0){
		temp_fcr &= 0xFE;
	} else {
		temp_fcr |= 0x01;
	}
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_FCR, temp_fcr);

	return;
}

void SC16IS750_FIFOReset(SC16IS750_t * dev, uint8_t channel, uint8_t rx_fifo)
{
	uint8_t temp_fcr;

	temp_fcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_FCR);

	if (rx_fifo == 0){
		temp_fcr |= 0x04;
	} else {
		temp_fcr |= 0x02;
	}
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_FCR, temp_fcr);

	return;

}

void SC16IS750_FIFOSetTriggerLevel(SC16IS750_t * dev, uint8_t channel, uint8_t rx_fifo, uint8_t length)
{
	uint8_t temp_reg;

	temp_reg = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_MCR);
	temp_reg |= 0x04;
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_MCR, temp_reg); //SET MCR[2] to '1' to use TLR register or trigger level control in FCR register

	temp_reg = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_EFR);
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_EFR, temp_reg|0x10); //set ERF[4] to '1' to use the  enhanced features
	if (rx_fifo == 0) {
		SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_TLR, length<<4); //Tx FIFO trigger level setting
	} else {
		SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_TLR, length);	 //Rx FIFO Trigger level setting
	}
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_EFR, temp_reg); //restore EFR register

	return;
}

uint8_t SC16IS750_FIFOAvailableData(SC16IS750_t * dev, uint8_t channel)
{
#ifdef	SC16IS750_DEBUG_PRINT
	printf("=====Available data:");
	printf("%x\n", SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_RXLVL));
#endif
	return SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_RXLVL);
	//return SC16IS750_ReadRegister(dev, SC16IS750_REG_LSR) & 0x01;
}

uint8_t SC16IS750_FIFOAvailableSpace(SC16IS750_t * dev, uint8_t channel)
{
	return SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_TXLVL);
}

void SC16IS750_WriteByte(SC16IS750_t * dev, uint8_t channel, uint8_t val)
{
	uint8_t tmp_lsr;
/*
	while ( SC16IS750_FIFOAvailableSpace(dev, channel) == 0 ){
	};
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_THR, val);
*/

	do {
		tmp_lsr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_LSR);
		delayMicroseconds(1);
	} while ((tmp_lsr&0x20) ==0);

	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_THR, val);

}

int SC16IS750_ReadByte(SC16IS750_t * dev, uint8_t channel)
{
	volatile uint8_t val;
	if (SC16IS750_FIFOAvailableData(dev, channel) == 0) {
#ifdef	SC16IS750_DEBUG_PRINT
		printf("No data available\n");
#endif
		return -1;

	} else {

#ifdef	SC16IS750_DEBUG_PRINT
		printf("***********Data available***********\n");
#endif
		val = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_RHR);
		return val;
	}


}

void SC16IS750_EnableRs485(SC16IS750_t * dev, uint8_t channel, uint8_t invert)
{
	uint8_t temp_efcr;
	temp_efcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_EFCR);
	if ( invert == NO_INVERT_RTS_SIGNAL) {
		temp_efcr |= 0x20;
	} else {
		temp_efcr |= 0x30;
	}
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_EFCR,temp_efcr);

	return;
}

void SC16IS750_EnableTransmit(SC16IS750_t * dev, uint8_t channel, uint8_t tx_enable)
{
	uint8_t temp_efcr;
	temp_efcr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_EFCR);
	if ( tx_enable == 0) {
		temp_efcr |= 0x04;
	} else {
		temp_efcr &= 0xFB;
	}
	SC16IS750_WriteRegister(dev, channel, SC16IS750_REG_EFCR, temp_efcr);

	return;
}

uint8_t SC16IS750_ping(SC16IS750_t * dev)
{
	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_A, SC16IS750_REG_SPR, 0x55);
	if (SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_A, SC16IS750_REG_SPR) !=0x55) {
		return 0;
	}

	SC16IS750_WriteRegister(dev, SC16IS750_CHANNEL_A, SC16IS750_REG_SPR, 0xAA);
	if (SC16IS750_ReadRegister(dev, SC16IS750_CHANNEL_A, SC16IS750_REG_SPR) !=0xAA) {
		return 0;
	}

	return 1;

}

void SC16IS750_setTimeout(SC16IS750_t * dev, uint32_t time_out)
{
	dev->timeout = time_out;
}


int16_t SC16IS750_readwithtimeout(SC16IS750_t * dev, uint8_t * channel)
{
	char tmp;
	uint32_t time_stamp;
	time_stamp = millis();
	do {
		if (SC16IS750_available(dev, SC16IS750_CHANNEL_A)) {
			*channel = SC16IS750_CHANNEL_A;
			tmp = SC16IS750_read(dev, SC16IS750_CHANNEL_A);
			if (tmp > 0) return tmp;
		}
		if (dev->channels == SC16IS750_DUAL_CHANNEL) {
			if (SC16IS750_available(dev, SC16IS750_CHANNEL_B)) {
				*channel = SC16IS750_CHANNEL_B;
				tmp = SC16IS750_read(dev, SC16IS750_CHANNEL_B);
				if (tmp > 0) return tmp;
			}
		}
		delay(10);
	} while(millis() - time_stamp < dev->timeout);
	return -1;	 // -1 indicates timeout
}

void SC16IS750_flush(SC16IS750_t * dev, uint8_t channel)
{
	uint8_t tmp_lsr;

	do {
		tmp_lsr = SC16IS750_ReadRegister(dev, channel, SC16IS750_REG_LSR);
		delayMicroseconds(1);
	} while ((tmp_lsr&0x20) ==0);


}

int SC16IS750_peek(SC16IS750_t * dev, uint8_t channel)
{
	if ( dev->peek_flag == 0 ) {
		dev->peek_buf = SC16IS750_ReadByte(dev, channel);
		if (	dev->peek_buf >= 0 ) {
			dev->peek_flag = 1;
		}
	}

	return dev->peek_buf;

}

