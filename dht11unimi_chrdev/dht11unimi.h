/*
 * Chrdev include file
 */


#ifndef DHT11_UNIMI_H
	#define DHT11_UNIMI_H


#define MAX_DEVICES	8
#define NAME_LEN	32
#define BUF_LEN		300

#define DHT11UNIMI_COMPATIBLE	"dht11unimi"

#define DHT11_TV_STATUS_STANDBY_1S		1
#define DHT11_TV_MCU_PULL_DOWN_18MS		20
#define DHT11_TV_MCU_PULL_UP_40US		40
#define DHT11_TV_PULL_DOWN_80US			80
#define DHT11_TV_PULL_UP_80US			80
#define DHT11_TV_MCU_RECEIVE_DATA		5

#define DHT11_PROTOCOL_STATUS_STANDBY				1
#define DHT11_PROTOCOL_MCU_PULL_DOWN				2
#define DHT11_PROTOCOL_MCU_PULL_UP					3
#define DHT11_PROTOCOL_MCU_CHANGE_PIN_DIRECTION		4
#define DHT11_PROTOCOL_SENSOR_PULL_DOWN				5
#define DHT11_PROTOCOL_SENSOR_PULL_UP				6
#define DHT11_PROTOCOL_MCU_RECEIVE_DATA				7

#define DHT11_WORKS_RUN								0
#define DHT11_WORKS_SHUTDOWN						1
#define DHT11_WORK_STOPPED							2

/*
 * Chrdev basic structs
 */

/* Main struct */
struct chrdev_device {

	/* driver linux platform data and i/o buffer */
	char label[NAME_LEN];
	unsigned int busy :1;
	char buf[BUF_LEN];
	int wbytes;
	int read_only;

	unsigned int id;
	struct module *owner;
	struct cdev cdev;
	struct device *dev;

	struct mutex mux;
	struct wait_queue_head queue;

	/* scheduled job and sw-irq */
	int w_dht11_work_time;
	int work_status;

	/* data with DHT11 sensor */
	int protocol_status;
	bool handshake_end;
	bool resync;
	unsigned int sensor_busy :1;
	unsigned int sensor_error :1;
	int gpio_pin;
	int irq;
	volatile struct GpioRegisters *s_pGpioRegisters;
	volatile unsigned *gpio;
	unsigned char dht_start_bit;
	unsigned char dht_bit_cnt;
	unsigned char dht_bit_pos;
	unsigned char dht_byte_pos;
	unsigned char dht_data[5];
};

/*
 * Exported functions
 */

#define to_class_dev(obj) container_of((obj), struct class_device, kobj)
#define to_chrdev_device(obj) container_of((obj), struct chrdev_device, class)

extern int chrdev_device_register(const char *label, unsigned int id, unsigned int read_only, struct module *owner, struct device *parent);
extern int chrdev_device_unregister(const char *label, unsigned int id);

#endif
