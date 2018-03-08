/*
# Title: main.c - Full implementation of the carbon_rover project
#                 Implements reading from HCSR04 ultrasonic sensors
#                 Implements reading from IR Edge Detection sensor
#                 Impliments controlling L9110S Motor Controller
#                 Impliments controliing Neopixels using I2C over Arduino
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
#include <i2c.h>
#include <uart.h>
#include <board.h>

#define PORT	"GPIOA"
#define PORT2	"GPIOB"
#define PORT3	"GPIOC"

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
#define TRIGFR	3 //GPIOC
#define ECHOFR	5
#define TRIGFC	0 //GPIOC WAS C6
#define ECHOFC	8
#define TRIGBR	1 //GPIOC WAS C7
#define ECHOBR	9
#define TRIGBC	3 //GPIOB
#define ECHOBC	10
#define TRIGBL	8 //GPIOB
#define ECHOBL	9

#define BT      6
#define PULL_UP GPIO_PUD_PULL_DOWN

#define SLEEP_TIME 1000

#define US_LIMIT_CYCLE 487200
//100cm for 84MHz (((MAX_RANGE * 58000) / 1000000000) * (CLOCK * 1000000)
#define US_LIMIT 50

#define STACKSIZE 1024
#define PRIORITY 7

#define DELAY K_MSEC(0.05)

#define I2C_DEV CONFIG_I2C_1_NAME
#define UART_DEV CONFIG_UART_STM32_PORT_6_NAME

static struct device *gpioa;
static struct device *gpiob;
static struct device *gpioc;
static struct device *i2c_dev;
static struct device *uar;

static uint32_t dir = 0, irfr = 0, irfl = 0, irbr = 0, irbl = 0, us_fl = 0,
                us_fc = 0, us_fr = 0, us_br = 0, us_bc = 0, us_bl = 0;
static char tdir[10] = "n";
static unsigned char char_in = 's';
static uint32_t ret, cnt;


void fwd();
void bwd();
void stop();
void left();
void right();
void read_us();
void read_ir();
void run_auto();
void run_controller();
void us_led();
void pixel();
void pixel_bt();
void pixel_bt_status(bool status);
void uart_out();
void controller_check();
void controller_input();
uint32_t get_us(uint32_t trig, uint32_t echo, struct device *dev);

static K_THREAD_DEFINE(read_us_id, STACKSIZE, read_us, NULL, NULL, NULL,
                        PRIORITY, 0, K_FOREVER);
static K_THREAD_DEFINE(pixel_id, STACKSIZE, pixel, NULL, NULL, NULL,
                        PRIORITY, 0, K_FOREVER);
static K_THREAD_DEFINE(pixel_bt_id, STACKSIZE, pixel_bt, NULL, NULL, NULL,
                        PRIORITY, 0, K_FOREVER);
static K_THREAD_DEFINE(run_auto_id, STACKSIZE, run_auto, NULL, NULL, NULL,
                        PRIORITY, 0, K_FOREVER);
static K_THREAD_DEFINE(run_controller_id, STACKSIZE, run_controller, NULL,
                        NULL, NULL, PRIORITY, 0, K_FOREVER);

void main(void)
{
	k_sleep(2000);

	i2c_dev = device_get_binding(I2C_DEV);

	if (!i2c_dev) {
		printk("I2C: Device driver not found.\n");
		return;
	}

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
  uar = device_get_binding(UART_DEV);
  if (!uar) {
		printk("UART6 init failed!\n");
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
  // UART and BT Disconnect Inturrupt Setup
  uart_irq_callback_set(uar, controller_input);
  uart_irq_rx_enable(uar);
  ret = gpio_pin_configure(gpioc, BT, GPIO_DIR_IN  |  PULL_UP);
  if (ret) {
    printk("Error configuring PC%d!\n", BT);
    return;
  }

  //start threads
	k_thread_start(read_us_id);
  controller_check();
}

void controller_check()
{
  // wait thread with cool animation
  k_thread_start(run_controller_id);
  k_thread_start(pixel_bt_id);
  int i = 0;
  u32_t val;
  char_in = 'S';
  while(i <= 400)
  {
	  gpio_pin_read(gpioc, BT, &val);
    if (val == 1)
	  {
      printk("Bluetooth Connected \n");
      // cool animations
      k_thread_abort(pixel_bt_id);
      pixel_bt_status(1);
      k_thread_start(pixel_id);
      return;
	  }
	  else
	  {
      printk("Bluetooth Disonnected \n");

	  }
    k_sleep(25);
    i++;
  }
  k_thread_abort(pixel_bt_id);
  k_thread_abort(run_controller_id);
  pixel_bt_status(0);
  k_thread_start(pixel_id);
  k_thread_start(run_auto_id);
}

void controller_input()
{
	cnt = uart_poll_in(uar, &char_in);
	if (cnt == 0)
	{
		printk("%c\n", char_in);
	}
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
	k_busy_wait(50);
	gpio_pin_write(dev, trig, 0);
	start_time = k_cycle_get_32();
	do {
		gpio_pin_read(dev, echo, &val);
		stop_time = k_cycle_get_32();
		cycles_spent = stop_time - start_time;
		if (cycles_spent > US_LIMIT_CYCLE)
		{
			return US_LIMIT;
		}
	} while (val == 0);
	start_time = k_cycle_get_32();

	do {
		gpio_pin_read(dev, echo, &val);
		stop_time = k_cycle_get_32();
		cycles_spent = stop_time - start_time;
		if (cycles_spent > US_LIMIT_CYCLE)
		{
			return US_LIMIT;
		}
	} while (val == 1);
	nanseconds_spent = SYS_CLOCK_HW_CYCLES_TO_NS(cycles_spent);
	cm = nanseconds_spent / 58000;
	//printk("%d\n", cm);
	return cm;
}

void read_ir()
{

	gpio_pin_read(gpiob, IRFR, &irfr);
	gpio_pin_read(gpiob, IRFL, &irfl);
	gpio_pin_read(gpiob, IRBR, &irbr);
	gpio_pin_read(gpiob, IRBL, &irbl);

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
		k_yield();
	}
}

void uart_out()
{
	printk("Front Left US: %d\tFront Center US: %d\tFront Right US: %d\n"
          "Back Right US: %d\tBack Center US: %d\tBack Letf US: %d\n"
          "Front Right IR: %d\tFront Left IR: %d\nBack Right IR: %d\t"
          "Back Letf IR: %d\nDirection: %s\n", us_fl, us_fc, us_fr,
          us_br, us_bc, us_bl, irfr, irfl, irbr, irbl, tdir);
}

void run_auto()
{
	while (1) {

		read_ir();

		if((irfr == 0 && irfl == 0 && irbr == 0 && irbl == 0) ||
        (us_fc < 15 && us_bc < 15) || (us_fc < 15 && irbr == 0 && irbl == 0) ||
        (us_bc < 15 && irfr == 0 && irfl == 0))
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
		k_yield();
	}
}

void run_controller()
{
	while (1) {

		read_ir();

		if((irfr == 0 && irfl == 0 && irbr == 0 && irbl == 0) ||
        (us_fc < 15 && us_bc < 15) || (us_fc < 15 && irbr == 0 && irbl == 0) ||
        (us_bc < 15 && irfr == 0 && irfl == 0))
		{
			stop();
			//dir = 0;
		}
		else if(char_in == 'F')
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
				//dir = 1;
			}
		}

		else if(char_in == 'B')
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
				//dir = 0;
			}
		}

    else if (char_in == 'L')
    {
      left();
    }

    else if (char_in == 'R')
    {
      right();
    }

    else if (char_in == 'S')
      {
        if((irfr == 0) || (us_fr < 15) || (irbl == 0) || (us_bl < 15))
		    {
          left();
		    }
        else if((irfl == 0) || (us_fl < 15) || (irbr == 0) || (us_br < 15))
        {
          right();
        }
        else if((irfr == 0 && irfl == 0) || (us_fc < 15))
        {
          bwd();
        }
        else if((irbr == 0 && irbr == 0) || (us_bc < 15))
        {
          fwd();
        }
        else
        {
          stop();
        }
      }
		uart_out();
		k_yield();
	}
}

void pixel_bt_status(bool status)
{
  uint8_t data[22][4];
  if (status == 1)
  {
    for (int j = 0; j < 2; j++)
    {
      for (int i = 0; i < 22; i++)
      {
        data[i][0] = i;
  	    data[i][1] = 0;
  			data[i][3] = 0;
  			data[i][2] = 255;
        ret = i2c_write(i2c_dev, data[i], 4, 13);
        k_busy_wait(50);
      }
      k_sleep(200);
      for (int i = 0; i < 22; i++)
      {
  			data[i][0] = i;
  			data[i][1] = 0;
  			data[i][3] = 0;
  			data[i][2] = 0;
        ret = i2c_write(i2c_dev, data[i], 4, 13);
        k_busy_wait(50);
      }
    k_sleep(200);
    }
  }
  else if (status == 0)
  {
    for (int j = 0; j < 2; j++)
    {
      for (int i = 0; i < 22; i++)
      {
        data[i][0] = i;
  	    data[i][1] = 255;
  			data[i][3] = 0;
  			data[i][2] = 0;
        ret = i2c_write(i2c_dev, data[i], 4, 13);
        k_busy_wait(50);
      }
      k_sleep(200);
      for (int i = 0; i < 22; i++)
      {
  			data[i][0] = i;
  			data[i][1] = 0;
  			data[i][3] = 0;
  			data[i][2] = 0;
        ret = i2c_write(i2c_dev, data[i], 4, 13);
        k_busy_wait(50);
      }
    k_sleep(200);
    }
  }
}

void pixel_bt()
{
  uint8_t data[22][4];
  while(1)
  {
    for (int j = 0; j < 22; j++)
  	{
      for (int i = 0; i < 22; i++)
    	{
    			data[i][0] = i;
    			data[i][1] = 0;
    			data[i][3] = 0;
    			data[i][2] = 0;
    	}
  			data[j][0] = j;
  			data[j][1] = 0;
  			data[j][3] = 255;
  			data[j][2] = 255;
      for (int i = 0; i < 22; i++)
    	{
    		ret = i2c_write(i2c_dev, data[i], 4, 13);
    		k_busy_wait(50);
    	}
      k_sleep(45);
  	}
	}
}

void pixel()
{
	while(1)
	{
		us_led();
		k_yield();
	}
}

void us_led()
{
 	uint8_t data[22][4];

	for (int i = 0; i < 4; i++)
	{
		if(irfl == 0)
		{
			data[i][0] = i;
			data[i][1] = 255;
			data[i][3] = 0;
			data[i][2] = 0;
		}
		else
		{	data[i][0] = i;
			data[i][1] = 255 - ((((us_fl*100)/US_LIMIT)*255)/100);
			data[i][3] = ((((us_fl*100)/US_LIMIT)*255)/100);
			data[i][2] = 0;
		}
	}

	for (int i = 4; i < 7; i++)
	{
		if(irfl == 0 && irfr == 0)
		{
			data[i][0] = i;
			data[i][1] = 255;
			data[i][3] = 0;
			data[i][2] = 0;
		}
		else
		{	data[i][0] = i;
			data[i][1] = 255 - ((((us_fc*100)/US_LIMIT)*255)/100);
			data[i][3] = ((((us_fc*100)/US_LIMIT)*255)/100);
			data[i][2] = 0;
		}
	}

	for (int i = 7; i < 11; i++)
	{
		if(irfr == 0)
		{
			data[i][0] = i;
			data[i][1] = 255;
			data[i][3] = 0;
			data[i][2] = 0;
		}
		else
		{	data[i][0] = i;
			data[i][1] = 255 - ((((us_fr*100)/US_LIMIT)*255)/100);
			data[i][3] = ((((us_fr*100)/US_LIMIT)*255)/100);
			data[i][2] = 0;
		}
	}

	for (int i = 11; i < 15; i++)
	{
		if(irbr == 0)
		{
			data[i][0] = i;
			data[i][1] = 255;
			data[i][3] = 0;
			data[i][2] = 0;
		}
		else
		{	data[i][0] = i;
			data[i][1] = 255 - ((((us_br*100)/US_LIMIT)*255)/100);
			data[i][3] = ((((us_br*100)/US_LIMIT)*255)/100);
			data[i][2] = 0;
		}
	}

	for (int i = 15; i < 18; i++)
	{
		if(irbr == 0 && irbl == 0)
		{
			data[i][0] = i;
			data[i][1] = 255;
			data[i][3] = 0;
			data[i][2] = 0;
		}
		else
		{	data[i][0] = i;
			data[i][1] = 255 - ((((us_bc*100)/US_LIMIT)*255)/100);
			data[i][3] = ((((us_bc*100)/US_LIMIT)*255)/100);
			data[i][2] = 0;
		}
	}


	for (int i = 18; i < 22; i++)
	{
		if(irbl == 0)
		{
			data[i][0] = i;
			data[i][1] = 255;
			data[i][3] = 0;
			data[i][2] = 0;
		}
		else
		{	data[i][0] = i;
			data[i][1] = 255 - ((((us_bl*100)/US_LIMIT)*255)/100);
			data[i][3] = ((((us_bl*100)/US_LIMIT)*255)/100);
			data[i][2] = 0;
		}
	}

	for (int i = 0; i < 22; i++)
	{
		ret = i2c_write(i2c_dev, data[i], 4, 13);
		k_busy_wait(50);
	}


}
