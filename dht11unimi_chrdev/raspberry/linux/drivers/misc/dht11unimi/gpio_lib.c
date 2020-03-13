/*
 * gpio.c
 *
 *  Created on: Feb 15, 2020
 *      Author: Nunzio Castelli

 *  Reference:	Raspberry Pi Hardaware Reference
 *  			Raspberry Pi datasheet
 *  			https://sysprogs.com/VisualKernel/tutorials/raspberry/leddriver/
 *  			https://www.raspberrypi.org/forums/viewtopic.php?t=207816
 *  			https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access
 *
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/irqflags.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/stat.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/ioctl.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/cdev.h>

#include <asm/io.h>

#include "gpio_lib.h"
#include "dht11unimi.h"

/*
 * The GPIO peripheral has three dedicated interrupt lines. These lines are triggered by the
 * setting of bits in the event detect status register. Each bank has its’ own interrupt line with the
 * third line shared between all bits.
 * The Alternate function table also has the pull state (pull-up/pull-down) which is applied after
 * a power down.
 *
 * The GPIO has 41 registers. All accesses are assumed to be 32-bit.
 *
 */
#define RBUF_LEN 256
#define SUCCESS 0

#define FIXED_STANDBY_1S	1

volatile unsigned *gpio;
extern struct chrdev_device chrdev_dht11;
static spinlock_t lock;
static struct timeval tv_current_interrupt = {0, 0};
static struct timeval tv_previous_interrupt = {0, 0};


/*
 *		Interrupt handler
 *		DHT11 protocol data. Single pin bus.
 *
 */
static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs) {

	long delta_us;
	int signal;

	signal = GPIO_READ_PIN(chrdev_dht11.gpio_pin);
	// clear IRQ
	GPIO_INT_CLEAR(chrdev_dht11.gpio_pin);

//	printk(KERN_INFO "***dht11 device driver"	": exec interrupt on pin %d irq %04x voltage level %d protoc_status %d\n",
//		   chrdev_dht11.gpio_pin, chrdev_dht11.irq, signal, chrdev_dht11.protocol_status);

	do_gettimeofday(&tv_current_interrupt);

	if (chrdev_dht11.protocol_status == DHT11_PROTOCOL_STATUS_STANDBY ||
		chrdev_dht11.protocol_status == DHT11_PROTOCOL_MCU_PULL_UP ||
		chrdev_dht11.protocol_status == DHT11_PROTOCOL_MCU_PULL_DOWN) {
//		printk(KERN_INFO "Ignoring interrupt from MCU handshake. gpio pin %d\n", chrdev_dht11.gpio_pin);
		return IRQ_HANDLED;
	}

	/* sync */
	if (chrdev_dht11.protocol_status == DHT11_PROTOCOL_SENSOR_PULL_DOWN) {

		if (signal) {
			/* bad sync */
			printk(KERN_INFO "dht11 device driver"	": bad sync, wrong expected signal voltage. Voltage level %d. Restart communication.\n", signal);
			chrdev_dht11.resync = true;
		} else {
			/*
			 *		Then the programme of DHT sets Data Single-bus voltage
			 *		level from low to high and keeps it for 80us for DHT’s
			 *		preparation for sending data.
			 */
			//printk(KERN_INFO "DHT11_PROTOCOL_SENSOR_PULL_DOWN done. WAIT VOLTAGE HIGH ON PIN %d\n", chrdev_dht11.gpio_pin);
			chrdev_dht11.protocol_status = DHT11_PROTOCOL_SENSOR_PULL_UP;
		}

	} else if (chrdev_dht11.protocol_status == DHT11_PROTOCOL_SENSOR_PULL_UP) {
		//delta_us = tv_current_interrupt.tv_usec - tv_previous_interrupt.tv_usec;
		if (!signal) {
			/* bad sync */
			printk(KERN_WARNING "dht11 device driver"	": received interrupt with unchanged pin voltage level %d at us_time %lu while waiting sensor transition sequence. Force resync.\n", signal, delta_us);
			chrdev_dht11.resync = 1;
		} else {
			/* go to receveing data */
			//printk(KERN_INFO "dht11 device driver"	": DHT11_PROTOCOL_SENSOR_PULL_UP done, handshake completed. MCU now starts to buffering 40 bit data.\n");
			chrdev_dht11.protocol_status = DHT11_PROTOCOL_MCU_RECEIVE_DATA;
			chrdev_dht11.handshake_end = 1;
		}

	} else {
		/*
		 *	Buffering te 40 bit data output by the DHT11 DATA pin.
		 *	The microprocessor according to the change of I/O level receive 40 bits of data, high first out.
		 *	Data format of "0": high level and low level of 50 microseconds and 26-28 microsecond.
		 *	Data format of "1": low level 50 microsecond plus 70 microsecond high.
		 */
		delta_us = tv_current_interrupt.tv_usec - tv_previous_interrupt.tv_usec;
		//printk(KERN_INFO "dht11 device driver"	": received interrupt pin voltage level %d, us %lu\n", signal, delta_us);

		if (!signal) {

			if (chrdev_dht11.dht_start_bit > 0 && chrdev_dht11.dht_bit_cnt <= 40) {

				// store the previous bit >=65us
				if (delta_us >= 30) {
					//printk(KERN_INFO "dht11 device driver"	": bit %d voltage level %d, elapsed us %lu\n", chrdev_dht11.dht_bit_cnt, 1, delta_us);
					chrdev_dht11.dht_data[chrdev_dht11.dht_byte_pos] |= 1 << (7 - chrdev_dht11.dht_bit_pos);
				} //else
					//printk(KERN_INFO "dht11 device driver"	": bit %d voltage level %d, elapsed us %lu\n", chrdev_dht11.dht_bit_cnt, 0, delta_us);

				if (chrdev_dht11.dht_bit_pos == 7) {
					chrdev_dht11.dht_bit_pos = 0;
					chrdev_dht11.dht_byte_pos++;
				} else
					chrdev_dht11.dht_bit_pos++;

				chrdev_dht11.dht_bit_cnt++;
			}

			chrdev_dht11.dht_start_bit++;
		}

		// give control back
		if (chrdev_dht11.dht_bit_cnt >= 40)
			chrdev_dht11.sensor_busy = 0;
	}

	memcpy(&tv_previous_interrupt, &tv_current_interrupt, sizeof(struct timeval));
	return IRQ_HANDLED;
}

void gpiolib_init(void) {

	// Request to the kernel to map physical address to virtual address
	chrdev_dht11.s_pGpioRegisters = (struct GpioRegisters *)ioremap(GPIO_BASE, sizeof(struct GpioRegisters));
	gpio = chrdev_dht11.gpio = ioremap(GPIO_BASE, sizeof(struct GpioRegisters));
	memset(chrdev_dht11.dht_data, 0x00, sizeof(chrdev_dht11.dht_data));

	pr_info("GPIO memory address mapped at %x reference pointer %p\n", GPIO_BASE, chrdev_dht11.s_pGpioRegisters);
}

int gpiolib_request_pin_irq(int pin_gpio) {

	int ret;
	unsigned long flags;

	chrdev_dht11.irq = gpio_to_irq(pin_gpio);
	ret = request_irq(chrdev_dht11.irq, (irq_handler_t) irq_handler, 0, "dht11 device driver ", (void *)chrdev_dht11.gpio);

	switch (ret) {
	case -EBUSY:
		printk(KERN_ERR "dht11 device driver" ": IRQ %04x is busy\n", chrdev_dht11.irq);
		chrdev_dht11.irq = -1;
		return -EBUSY;
	case -EINVAL:
		printk(KERN_ERR "dht11 device driver" ": Bad irq number or handler\n");
		chrdev_dht11.irq = -1;
		return -EINVAL;
	default:
		//printk(KERN_INFO "dht11 device driver"	": Interrupt %04x obtained\n", chrdev_dht11.irq);
		break;
	};

	spin_lock_irqsave(&lock, flags);
	// GPREN0 GPIO Pin Rising Edge Detect Enable
	GPIO_INT_RISING(pin_gpio, 1);
	// GPFEN0 GPIO Pin Falling Edge Detect Enable
	GPIO_INT_FALLING(pin_gpio, 1);
	// clear interrupt flag
	GPIO_INT_CLEAR(pin_gpio);
	spin_unlock_irqrestore(&lock, flags);

	return 0;
}

void clear_interrupts(int pin) {

	unsigned long flags;

	//printk(KERN_INFO "dht11 device driver"	": clear irq %04x\n", chrdev_dht11.irq);

	spin_lock_irqsave(&lock, flags);
	// GPREN0 GPIO Pin Rising Edge Detect Disable
	GPIO_INT_RISING(pin, 0);
	// GPFEN0 GPIO Pin Falling Edge Detect Disable
	GPIO_INT_FALLING(pin, 0);
	spin_unlock_irqrestore(&lock, flags);

	if (chrdev_dht11.irq != -1) {
		free_irq(chrdev_dht11.irq, (void *) gpio);
		chrdev_dht11.irq = -1;
	} //else
		//printk(KERN_INFO "dht11 device driver"	": interrupt already freed\n");
}

int read_dht11(int pin) {

	int retry = 0;
	int wakeup_max = 3;
	int wakeup_n;

	do {
		wakeup_n = 0;
		memset(chrdev_dht11.buf,0x00, BUF_LEN);
		chrdev_dht11.sensor_busy = 1;
		chrdev_dht11.resync = chrdev_dht11.sensor_error = chrdev_dht11.handshake_end = 0;
		memset(chrdev_dht11.dht_data, 0x00, sizeof(chrdev_dht11.dht_data));
		chrdev_dht11.dht_bit_cnt = chrdev_dht11.dht_bit_pos = chrdev_dht11.dht_byte_pos = chrdev_dht11.dht_start_bit = 0x00;

		/*
		 *		Install IRQ
		 *		protocol data will be handled inside it
		 *		From now we start to listening data from the sensor
		 */
		//printk(KERN_INFO "installing interrupt on pin %d\n", pin);
		gpiolib_request_pin_irq(pin);

		/*
		 * 		Stable the sensor with stanb-by signal
		 * 		We do this re-sync everytime to be sure
		 *
		 * 		Reference
		 * 			DHT11’s power supply is 3-5.5V DC. When
		 * 			power is supplied to the sensor, do not
		 * 			send any instruction to the sensor in
		 * 			within one second in order to pass the unstable
		 * 			status.
		 *
		 */
		//printk(KERN_INFO "CONFIGURING PIN -> OUTPUT VOLTAGE HIGH %d\n", pin);
		chrdev_dht11.protocol_status = DHT11_PROTOCOL_STATUS_STANDBY;

		GPIO_DIR_OUTPUT(pin);		// Set pin to output
		GPIO_SET_PIN(pin);			// Voltage high
		ssleep(FIXED_STANDBY_1S);

		/*
		 *		When the communication between MCU and DHT11 begins,
		 *		the programme of MCU will set Data Single-bus voltage
		 *		level from high to low and this process must take at
		 *		least 18ms to ensure DHT’s detection of MCU's signal,
		 */
		//printk(KERN_INFO "SET PIN OUTPUT VOLTAGE LOW %d\n", pin);
		chrdev_dht11.protocol_status = DHT11_PROTOCOL_MCU_PULL_DOWN;
		GPIO_CLEAR_PIN(pin);		// Voltage low
		mdelay(DHT11_TV_MCU_PULL_DOWN_18MS);

		/*
		 * 		Then MCU will pull up voltage and wait 20-40us for
		 * 		DHT’s response
		 */
		//printk(KERN_INFO "SET PIN OUTPUT VOLTAGE HIGH %d\n", pin);
		chrdev_dht11.protocol_status = DHT11_PROTOCOL_MCU_PULL_UP;
		GPIO_SET_PIN(pin);			// Voltage high
		udelay(DHT11_TV_MCU_PULL_UP_40US);

		/*
		 *		Once DHT detects the start signal, it will send out a
		 *		low-voltage-level response signal, which lasts 80us.
		 */
		//printk(KERN_INFO "SET PIN INPUT, EXPECTED VOLTAGE LOW %d\n", pin);
		chrdev_dht11.protocol_status = DHT11_PROTOCOL_SENSOR_PULL_DOWN;
		GPIO_DIR_INPUT(pin);

		while (chrdev_dht11.sensor_busy && !chrdev_dht11.resync) {
			msleep(500);
			if(++wakeup_n >= wakeup_max) {
				/* timeout */
				//printk(KERN_WARNING "Sensor protocol error, timeout communication\n");
				chrdev_dht11.resync = 1;
			}
		}

		clear_interrupts(chrdev_dht11.gpio_pin);

		// parity checking
		if (!chrdev_dht11.resync &&
			chrdev_dht11.dht_data[4] != 0 &&
			chrdev_dht11.dht_data[4] == chrdev_dht11.dht_data[0] + chrdev_dht11.dht_data[1] + chrdev_dht11.dht_data[2] + chrdev_dht11.dht_data[3])
			break;

		chrdev_dht11.sensor_error = 1;
		/* leave the sensor alone for a while */
		msleep(1500);
		//printk(KERN_ERR "Sensor protocol error, resync the communication. Retry number %d\n", retry);


	} while (++retry < 5);

	chrdev_dht11.sensor_busy = 0;

	if (!chrdev_dht11.sensor_error) {
//		printk(KERN_INFO "Sensor data stats nbit %d nbyte %d nstartbit %d\n",
//						  chrdev_dht11.dht_bit_cnt,
//						  chrdev_dht11.dht_byte_pos,
//						  chrdev_dht11.dht_start_bit);

		chrdev_dht11.wbytes = sprintf(chrdev_dht11.buf, "Sensor data RH: %0d.%0d TEMP: %0d.%0d",
									  chrdev_dht11.dht_data[0],
									  chrdev_dht11.dht_data[1],
									  chrdev_dht11.dht_data[2],
									  chrdev_dht11.dht_data[3]);

	} else {
		printk(KERN_WARNING "DHT11 sensor not ready, retry later\n");
		chrdev_dht11.wbytes = sprintf(chrdev_dht11.buf, "DHT11 sensor not ready, retry later");
	}
	return SUCCESS;
}

void gpiolib_setGPIOFunction(int pin_gpio, int functionMode) {
    int registerIndex = pin_gpio / 10;
    int bit = (pin_gpio % 10) * 3;
    unsigned oldValue = chrdev_dht11.s_pGpioRegisters->GPFSEL[registerIndex];
    unsigned mask = 0b111 << bit;
	pr_info("configuring gpio pin %d via GPFSEL reg %d bit weight %d from %x to %x\n", pin_gpio, registerIndex, bit, (oldValue >> bit) & 0b111, functionMode);
	chrdev_dht11.s_pGpioRegisters->GPFSEL[registerIndex] = (oldValue & ~mask) | ((functionMode << bit) & mask);
}

void gpiolib_setGPIOOutputValue(int GPIO, bool outputValue) {
    if (outputValue)
    	chrdev_dht11.s_pGpioRegisters->GPSET[GPIO / 32] |= (1 << (GPIO % 32));
    else
    	chrdev_dht11.s_pGpioRegisters->GPCLR[GPIO / 32] |= (1 << (GPIO % 32));
}

void gpiolib_set_pull_resistor(int pin_gpio, int ioctl_param) {

	// Write to GPPUD to set the required control signal
	chrdev_dht11.s_pGpioRegisters->GPPUD = ioctl_param;

    // Wait 150 cycles: the maximum frequency is 125Mhz, so that is 1.2us at most
    udelay(2000);

    // Write to GPPUDCLK to clock the control signal into the GPIO pads
    chrdev_dht11.s_pGpioRegisters->GPPUDCLK[pin_gpio / 32] |= (1 << (pin_gpio % 32));

    // Wait 150 cycles: the maximum frequency is 125Mhz, so that is 1.2us at most
    udelay(2000);

    // Write to GPPUD to remove the control signal
    chrdev_dht11.s_pGpioRegisters->GPPUD = 0;

    // Write to GPPUDCLK to remove the clock
    chrdev_dht11.s_pGpioRegisters->GPPUDCLK[ pin_gpio / 32 ] = 0;
	pr_info("pin_gpio pull resistor configured to %d [0->OFF 1->DOWN 2->UP]\n", ioctl_param);

}

void gpiolib_pin_release(int pin_gpio) {

    int registerIndex = pin_gpio / 10;
    int bit = (pin_gpio % 10) * 3;

    unsigned mask = 0b111 << bit;

    //Disable all interruptions
    chrdev_dht11.s_pGpioRegisters->GPREN[ pin_gpio / 32 ] &= ~(1 << (pin_gpio % 32));    // Disable rising edge interruption
    chrdev_dht11.s_pGpioRegisters->GPFEN[ pin_gpio / 32 ] &= ~(1 << (pin_gpio % 32));    // Disable falling edge interruption
    chrdev_dht11.s_pGpioRegisters->GPHEN[ pin_gpio / 32 ] &= ~(1 << (pin_gpio % 32));    // Disable high detect interruption
    chrdev_dht11.s_pGpioRegisters->GPLEN[ pin_gpio / 32 ] &= ~(1 << (pin_gpio % 32));    // Disable low detect interruption

    chrdev_dht11.s_pGpioRegisters->GPCLR[pin_gpio / 32] |= (1 << (pin_gpio % 32));    //low level on GPIO
    chrdev_dht11.s_pGpioRegisters->GPFSEL[registerIndex] &= ~mask;            // Set pin as input
    gpiolib_set_pull_resistor(pin_gpio, PIN_PULL_OFF);            //pull_none

    free_irq(chrdev_dht11.irq, NULL );
}

void gpiolib_cleanup_module(void) {
	iounmap(chrdev_dht11.s_pGpioRegisters);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nunzio Castelli");
MODULE_DESCRIPTION("dht11 device driver - SSRI Università degli Studi di Milano");
