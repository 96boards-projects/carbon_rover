/* 
# Title: main.c - Full implementation of the carbon_rover project
#                 Implements reading from HCSR04 ultrasonic sensors
#                 Implements reading from IR Edge Detection sensor
#                 Impliments controlling L9110S Motor Controller
# Author: Sahaj Sarup
# Copyright (c) 2018 Linaro Limited
#################################################################
*/

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <misc/printk.h>
#include <sys_clock.h>
#include <misc/util.h>
#include <string.h>

#define PORT	"GPIOA"
#define PORT2 "GPIOB"
#define PORT3 "GPIOC"

// Motor Controller Pins GPIOA
#define A1A	0
#define A1B	2
#define B1A	3
#define B1B	1

// IR Edge sensor Pins GPIOB
#define IRFL	12
#define IRFR	15
#define IRBL	14
#define IRBR	13

// Ultrasonic Sensor Pins
#define TRIGFL	2 //GPIOC
#define ECHOFL	4
#define TRIGFC	3 //GPIOC
#define ECHOFC	5
#define TRIGFR	6 //GPIOC
#define ECHOFR	8
#define TRIGBR	7 //GPIOC
#define ECHOBR	9
#define TRIGBC	6 //GPIOB
#define ECHOBC	7
#define TRIGBL	8 //GPIOB
#define ECHOBL	9

#define SLEEP_TIME 1000

#define STACKSIZE 1024
#define PRIORITY 7

#define DELAY K_MSEC(10)

static K_THREAD_STACK_ARRAY_DEFINE(stacks, 3, STACKSIZE);
static struct k_thread __kernel threads[3];

static struct device *gpioa;
static struct device *gpiob;
static struct device *gpioc;

static uint32_t dir = 0, irfr = 0, irfl = 0, irbr = 0, irbl = 0, us_fl = 0, us_fc = 0, us_fr = 0, us_br = 0, us_bc = 0, us_bl = 0;
static char tdir[10] = "n";


void fwd();
void bwd();
void stop();
void left();
void right();
void read_us();
void read_ir();
void run();
uint32_t get_us(uint32_t trig, uint32_t echo, struct device *dev);

void main(void)
{
	int ret;

	gpioa = device_get_binding(PORT);
	if (!gpioa) {
		printk("Cannot find %s!\n", PORT);
		return;
	}
	gpiob = device_get_binding(PORT2);
	if (!gpiob) {
		printk("Cannot find %s!\n", PORT2);
		return;
	}
	gpioc = device_get_binding(PORT3);
	if (!gpioc) {
		printk("Cannot find %s!\n", PORT3);
		return;
	}
	//Motor
	ret = gpio_pin_configure(gpioa, A1A, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PA%d!\n", A1A);
		return;
	}
	ret = gpio_pin_configure(gpioa, A1B, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PA%d!\n", A1B);
		return;
	}
	ret = gpio_pin_configure(gpioa, B1A, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PA%d!\n", B1A);
		return;
	}
	ret = gpio_pin_configure(gpioa, B1B, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PA%d!\n", B1B);
		return;
	}

	//IR
	ret = gpio_pin_configure(gpiob, IRFL, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PB%d!\n", IRFL);
		return;
	}
	ret = gpio_pin_configure(gpiob, IRFR, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PB%d!\n", IRFR);
		return;
	}
	ret = gpio_pin_configure(gpiob, IRBL, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PB%d!\n", IRBL);
		return;
	}
	ret = gpio_pin_configure(gpiob, IRBR, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PB%d!\n", IRBR);
		return;
	}

	//US
	ret = gpio_pin_configure(gpioc, TRIGFL, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PC%d!\n", TRIGFL);
		return;
	}
	ret = gpio_pin_configure(gpioc, ECHOFL, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PC%d!\n", ECHOFL);
		return;
	}
	ret = gpio_pin_configure(gpioc, TRIGFC, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PC%d!\n", TRIGFC);
		return;
	}
	ret = gpio_pin_configure(gpioc, ECHOFC, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PC%d!\n", ECHOFC);
		return;
	}
	ret = gpio_pin_configure(gpioc, TRIGFR, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PC%d!\n", TRIGFR);
		return;
	}
	ret = gpio_pin_configure(gpioc, ECHOFR, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PC%d!\n", ECHOFR);
		return;
	}
	ret = gpio_pin_configure(gpioc, TRIGBR, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PC%d!\n", TRIGBR);
		return;
	}
	ret = gpio_pin_configure(gpioc, ECHOBR, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PC%d!\n", ECHOBR);
		return;
	}
	ret = gpio_pin_configure(gpiob, TRIGBC, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PB%d!\n", TRIGBC);
		return;
	}
	ret = gpio_pin_configure(gpiob, ECHOBC, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PB%d!\n", ECHOBC);
		return;
	}
	ret = gpio_pin_configure(gpiob, TRIGBL, GPIO_DIR_OUT);
	if (ret) {
		printk("Error configuring PB%d!\n", TRIGBL);
		return;
	}
	ret = gpio_pin_configure(gpiob, ECHOBL, GPIO_DIR_IN);
	if (ret) {
		printk("Error configuring PB%d!\n", ECHOBL);
		return;
	}

	//Start threads
	k_thread_create(&threads[0], &stacks[0][0], STACKSIZE, read_us, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
	k_thread_create(&threads[1], &stacks[1][0], STACKSIZE, read_ir, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
	k_thread_create(&threads[2], &stacks[2][0], STACKSIZE, run, NULL, NULL, NULL, PRIORITY, 0, K_NO_WAIT);
}

void bwd()
{
	gpio_pin_write(gpioa, A1A, 1);
	gpio_pin_write(gpioa, A1B, 0);
	gpio_pin_write(gpioa, B1A, 1);
	gpio_pin_write(gpioa, B1B, 0);
	strcpy(tdir, "forward");
}

void fwd()
{
	gpio_pin_write(gpioa, A1A, 0);
	gpio_pin_write(gpioa, A1B, 1);
	gpio_pin_write(gpioa, B1A, 0);
	gpio_pin_write(gpioa, B1B, 1);
	strcpy(tdir, "backward");
}

void stop()
{
	gpio_pin_write(gpioa, A1A, 0);
	gpio_pin_write(gpioa, A1B, 0);
	gpio_pin_write(gpioa, B1A, 0);
	gpio_pin_write(gpioa, B1B, 0);
	strcpy(tdir, "STOP");
}

void left() {
	gpio_pin_write(gpioa, A1A, 1);
	gpio_pin_write(gpioa, A1B, 0);
	gpio_pin_write(gpioa, B1A, 0);
	gpio_pin_write(gpioa, B1B, 1);
	strcpy(tdir, "left spin");
}

void right() {
	gpio_pin_write(gpioa, A1A, 0);
	gpio_pin_write(gpioa, A1B, 1);
	gpio_pin_write(gpioa, B1A, 1);
	gpio_pin_write(gpioa, B1B, 0);
	strcpy(tdir, "right spin");
}

uint32_t get_us(uint32_t trig, uint32_t echo, struct device *dev)
{
	uint32_t cycles_spent;
	uint32_t nanseconds_spent;
	uint32_t val;
	uint32_t cm;
	uint32_t stop_time;
	uint32_t start_time;
	gpio_pin_write(dev, trig, 1);
	k_sleep(K_MSEC(10));
	gpio_pin_write(dev, trig, 0);
	start_time = k_cycle_get_32();
	do {
		gpio_pin_read(dev, echo, &val);
		stop_time = k_cycle_get_32();
		cycles_spent = stop_time - start_time;
		if (cycles_spent > 243600) //50cm for 84MHz (((MAX_RANGE * 58000) / 1000000000) * (CLOCK * 1000000))
		{
			return 50;
		}
	} while (val == 0);
	start_time = k_cycle_get_32();

	do {
		gpio_pin_read(dev, echo, &val);
		stop_time = k_cycle_get_32();
		cycles_spent = stop_time - start_time;
	} while (val == 1);
	nanseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
	cm = nanseconds_spent / 58000;
	//printk("%d\n", cm);
	return cm;
	k_sleep(DELAY);
}

void read_ir()
{
	while(1)
	{
		gpio_pin_read(gpiob, IRFR, &irfr);
		gpio_pin_read(gpiob, IRFL, &irfl);
		gpio_pin_read(gpiob, IRBR, &irbr);
		gpio_pin_read(gpiob, IRBL, &irbl);
		k_sleep(DELAY);
	}
}

void read_us()
{
	while(1)
	{
		us_fl = get_us(TRIGFL, ECHOFL, gpioc);
		us_fc = get_us(TRIGFC, ECHOFC, gpioc);
		us_fr = get_us(TRIGFR, ECHOFR, gpioc);
		us_br = get_us(TRIGBR, ECHOBR, gpioc);
		us_bc = get_us(TRIGBC, ECHOBC, gpiob);
		us_bl = get_us(TRIGBL, ECHOBL, gpiob);
		k_sleep(DELAY);
	}
}

void uart_out()
{
		printk("Front Left US: %d\tFront Center US: %d\tFront Right US: %d\nBack Right US: %d\tBack Center US: %d\tBack Letf US: %d\nFront Right IR: %d\tFront Left IR: %d\nBack Right IR: %d\tBack Letf IR: %d\nDirection: %s\n", us_fl, us_fc, us_fr, us_br, us_bc, us_bl, irfr, irfl, irbr, irbl, tdir);
}

void run()
{
	while (1) {

		if((irfr == 0 && irfl == 0 && irbr == 0 && irbl == 0) || (us_fc < 15 && us_bc < 15))
		{
			stop();
			dir = 0;
		}

		else if(dir == 0)
		{
			if((irfr == 0 && irfl == 1) || (us_fr < 15 && us_fl > 15))
			{
				left();
			}

			else if((irfl == 0 && irfr == 1) || (us_fr > 15 && us_fl < 15))
			{
				right();
			}

			else if (irfr == 1 && irfl == 1 && us_fr > 15 && us_fl > 15 && us_fc > 15)
			{
				fwd();
			}

			else if((irfr == 0 && irfl == 0) || (us_fc < 15))
			{
				bwd();
				dir = 1;
			}
		}

		else if(dir == 1)
		{
			if((irbr == 0 && irbl == 1) || (us_br < 15 && us_bl > 15))
			{
				right();
			}

			else if((irbl == 0 && irbr == 1) || (us_br > 15 && us_bl < 15))
			{
				left();
			}

			else if(irbr == 1 && irbl == 1 && us_br > 15 && us_bl > 15 && us_bc > 15)
			{
				bwd();
			}

			else if((irbr == 0 && irbr == 0) || (us_bc < 15))
			{
				fwd();
				dir = 0;
			}
		}
		uart_out();

		k_sleep(DELAY);
	}
}
