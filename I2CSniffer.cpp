#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include "I2C/I2C.h"
#include "UARTStream.h"

enum I2CEVENT {
	START, STOP, ACK, NAK, DATA, ERROR
};

enum I2CERROR {
	SIMULTANEOUS_SCK_SDA_CHANGE
};

struct I2CDATA {
	I2CEVENT event;
	uint8_t data;
};

// Which port does listen to I2C. SDA and SCL should be on the same port,
// and noth pins should be in the same PCINT group (0, 1 or 2).
#define PORT PINB;
// Pint numbers
static const uint8_t SCL_PIN = 0;
static const uint8_t SDA_PIN = 1;

static const uint8_t SCL_PCINT= 0;
static const uint8_t SDA_PCINT = 1;
static const uint8_t PCINT_GROUP = 0;

static const bool MAKE_TWI_TRAFFIC_FOR_SELF_TEST = false;

/*
 * This is not necessary for the operation of the sniffer!
 * It was for self test (MAKE_TWI_TRAFFIC_FOR_SELF_TEST).
 * An MPU6050 was connected to the TWI interface
 * and the sniffer listened to the traffic to that.
 */
#define MPU6050_ADDRESS     0x68 // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
void MPU6050_init(uint8_t stage) {
	switch (stage) {
	case 0:
		i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80); //PWR_MGMT_1    -- DEVICE_RESET 1
		break;
	case 1:
		i2c_writeReg(MPU6050_ADDRESS, 0x68, 0x7); //PWR_MGMT_1    -- RESETS
		break;
	case 2:
		i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
		break;
	case 3:
		i2c_writeReg(MPU6050_ADDRESS, 0x1A, 6); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; 5Hz BW on all.
		break;
	case 4:
		i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x10); //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 1000 deg/sec
		break;
	}
}

static const size_t BITBUFLEN = 256;
static volatile uint8_t bitBuf[BITBUFLEN];
static volatile uint8_t bitBufIn;
static volatile uint8_t bitBufOut;

/*
 * Sample the sniffer pins.
 * There is no buffer overrun check. Not time for it.
 */
ISR(PCINT0_vect) {
	bitBuf[bitBufIn] = PORT;
	bitBufIn++;
}

/*
 * The self test data transmission (MAKE_TWI_TRAFFIC_FOR_SELF_TEST).
 * Communicate normally on the TWI interface.
 * The necessary wires from TWI to the sniffer porst must be connected to self-test.
 */
void noiseTask() {
	static uint32_t makeNoise = 0;
	static uint8_t noise = 0;
	if (makeNoise++ > 500000UL) {
		makeNoise = 0;
		MPU6050_init(noise++);
		if (noise == 5)
			noise = 0;
	}
}

void processLoop() {
	static bool last_scl = true;
	static bool last_sda = true;
	static uint8_t bit;
	static uint8_t data;
	while (true) {
		if (bitBufOut != bitBufIn) {
			uint8_t port = bitBuf[bitBufOut++];
			bool scl = port & (1 << SCL_PIN);
			bool sda = port & (1 << SDA_PIN);
			if (scl && !last_scl) {
				// clock went high. Data should be stable.
				if (bit < 8) {
					data = (data << 1) | sda;
					bit++;
					if (bit == 8) {
						printf_P(PSTR("%0X\r\n"), data);
					}
				} else {
					bit = 0;
					if (sda)
						printf_P(PSTR("NAK\r\n"), data);
					else
						printf_P(PSTR("ACK\r\n"), data);
				}
			}
			else if (scl && last_scl) {
				if (sda && !last_sda) { // data change while clock high
					printf_P(PSTR("STOP\r\n"), data);
				} else if (!sda && last_sda) { // data change while clock high
					bit = 0;
					printf_P(PSTR("START\r\n"), data);
				}
			}
			last_scl = scl;
			last_sda = sda;
		}

		if (MAKE_TWI_TRAFFIC_FOR_SELF_TEST)
			noiseTask();
	}
}

int main() {
	PCMSK0 |= 1 << SCL_PCINT | 1 << SDA_PCINT;
	PCICR |= 1 << PCINT_GROUP;
	uart_init();
	fdevopen(&uart_putchar, &uart_getchar);
	if (MAKE_TWI_TRAFFIC_FOR_SELF_TEST)
		i2c_init();
	sei();
	printf("I2C Sniffer by dongfang\r\n");
	processLoop();
}
