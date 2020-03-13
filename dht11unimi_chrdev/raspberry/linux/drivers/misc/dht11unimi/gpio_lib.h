/*
 * gpio.h
 *
 *  Created on: Feb 15, 2020
 *      Author: nc
 */

#ifndef GPIO_LIB_H_
	#define GPIO_LIB_H_

#define BCM2708_PERI_BASE 0x20000000
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)

#define BCM2708_PERI_BASE       0x20000000
#define GPIO_BASE               (BCM2708_PERI_BASE + 0x200000)    // GPIO controller

#define INP_GPIO(g)   			*(gpio.addr + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)   			*(gpio.addr + ((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) 		*(gpio.addr + (((g)/10))) |= (((a)<=3?(a) + 4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET  				*(gpio.addr + 7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR  				*(gpio.addr + 10) // clears bits which are 1 ignores bits which are 0
#define GPIO_READ(g)  			*(gpio.addr + 13) &= (1<<(g))

#define GPIO_PULL 				*(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 			*(gpio+38) // Pull up/pull down clock


// set GPIO pin g as input
#define GPIO_DIR_INPUT(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
// set GPIO pin g as output
#define GPIO_DIR_OUTPUT(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
// get logical value from gpio pin g
#define GPIO_READ_PIN(g) (*(gpio+13) & (1<<(g))) && 1
// sets   bits which are 1 ignores bits which are 0
#define GPIO_SET_PIN(g)	*(gpio+7) = 1<<g;
// clears bits which are 1 ignores bits which are 0
#define GPIO_CLEAR_PIN(g) *(gpio+10) = 1<<g;

// Clear GPIO interrupt on the pin we use
#define GPIO_INT_CLEAR(g) *(gpio+16) = (*(gpio+16) | (1<<g));
// GPREN0 GPIO Pin Rising Edge Detect Enable/Disable
#define GPIO_INT_RISING(g,v) *(gpio+19) = v ? (*(gpio+19) | (1<<g)) : (*(gpio+19) ^ (1<<g))
// GPFEN0 GPIO Pin Falling Edge Detect Enable/Disable
#define GPIO_INT_FALLING(g,v) *(gpio+22) = v ? (*(gpio+22) | (1<<g)) : (*(gpio+22) ^ (1<<g))

#define  PIN_PULL_OFF               0
#define  PIN_PULL_DOWN              1
#define  PIN_PULL_UP                2

/* From raspberry datasheet */
struct GpioRegisters {
      uint32_t GPFSEL[6];
      uint32_t Reserved0;
      uint32_t GPSET[2];
      uint32_t Reserved1;
      uint32_t GPCLR[2];
      uint32_t Reserved2;
      uint32_t GPLEV[2];
      uint32_t Reserved3;
      uint32_t GPEDS[2];
      uint32_t Reserved4;
      uint32_t GPREN[2];
      uint32_t Reserved5;
      uint32_t GPFEN[2];
      uint32_t Reserved6;
      uint32_t GPHEN[2];
      uint32_t Reserved7;
      uint32_t GPLEN[2];
      uint32_t Reserved8;
      uint32_t GPAREN[2];
      uint32_t Reserved9;
      uint32_t GPAFEN[2];
      uint32_t Reserved10;
      uint32_t GPPUD;
      uint32_t GPPUDCLK[2];
      uint32_t Reserved11[4];
};

void gpiolib_init(void);
void gpiolib_setGPIOFunction(int pin_gpio, int functionMode);
void gpiolib_setGPIOOutputValue(int GPIO, bool outputValue);
void gpiolib_set_pull_resistor(int pin_gpio, int ioctl_param);
void gpiolib_pin_release(int pin_gpio);
void gpiolib_cleanup_module(void);
int gpiolib_request_pin_irq(int pin_gpio);
extern void clear_interrupts(int pin);

int read_dht11(int pin);

#endif /* GPIO_LIB_H_ */
