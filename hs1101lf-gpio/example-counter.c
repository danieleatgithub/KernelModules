/*
 * hs1101lf-gpio.c
 *
 *  Created on: 23/giu/2017
 *      Author: daniele
 *
 *      From: https://gist.github.com/withattribution/ac2336a49fb48836b18e
 *        From: https://www.mail-archive.com/beagleboard@googlegroups.com/msg00436.html
 */
/*
 *  hello-1.c - The simplest kernel module.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/uaccess.h>  // copy_from/to_user
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/param.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <asm/io.h>

//#define _DEBUG  // print messages into message log, see: dmesg
#define DEBUG_LED // blinks a LED at the output GPIO

// kernel module information
#define DRV_NAME "km_gpiotest"
#define MAX_AVERAGE_DEPTH  20
MODULE_LICENSE("Dual BSD/GPL");
int majornummer = 60;

// function prototypes
static void counter_exit(void);
static int counter_init(void);
static int setup_pinmux(void);
static void timer_fcounter(unsigned long arg);

// Declaration of the init and exit functions
module_init (counter_init);
module_exit (counter_exit);

// this structure holds information about our GPIOs
struct myios {
  u16 irq;
  u16 input_pin;
  u16 output_pin;
  u32 fcounter;  // counts pulses of the GPIO input line
  u32 pcounter;  // counts ptimer pulses per one periode of the input signal
};

static struct myios gpio_data;

// kernel timers for measurement purposes
struct timer_list timer_frequency_measurement;
struct timer_list timer_periodic_measurement;

// Variables for measurement counter and results
//int fcounter = 0;
int frequency = 0;  // holds the pulses per second (Hz)
int periodes = 0;   // result of periodic measurement
int timespan = 2;   // measurement time
int ptimespan = 1;  // periodic counter timespan

// ========================================================================================================================
// File IO functions for communication with the user space program
// ========================================================================================================================

// user space program opens this device: fopen("/dev/fcounter",.....
static int counter_open(struct inode *inode, struct file *filp) {
#ifdef _DEBUG
  printk("<1>counter: Device opened by user space program\n");
#endif

  return 0;
}

// user space program closes this device: fclose(...
static int counter_release(struct inode *inode, struct file *filp) {
#ifdef _DEBUG
  printk("<1>counter: Device closed by user space program\n");
#endif

  return 0;
}

// user space program writes some values to this module:  i.e.: fwrite(databuffer,size_of_item,number_of_items,filepointer);
// the parameter count = size_of_item * number_of_items since we are a character driver which handles byte by byte
static ssize_t counter_write(struct file *filp, const char *buf, size_t count,
                             loff_t *f_pos) {
  int res;
  int value;

#ifdef _DEBUG
  printk("<1>counter: userspace program sent %d bytes\n",count);
#endif

  if (count != 4)
    return count;  // ignore all value with other length then 4

  res = copy_from_user(&value, buf, count);

#ifdef _DEBUG
  printk("<1>counter: userspace program sent value: %x [hex]\n",value);
#endif

  // now the data are in value, we can process this data
  // data format of value for this module: 0xrrssttuu
  // rr = 0x55 (fixed value)
  // ss = 0x37 (fixed value)
  // tt = depth of averaging
  // uu = timespan for measurement
  if (((value >> 24) & 0xff) == 0xaa && ((value >> 16) & 0xff) == 0x37
      && ((value >> 8) & 0xff) == 0x00) {
    timespan = value & 0xff;

    // security
    if (timespan > 20)
      timespan = 20;  // max. 20s measurement period
  }

  // the return value show how many bytes have been processed by this function
  // if all bytes have been processed that the return value = count
  return count;
}

// the user space program requests data from this module. i.e.: fread ( databuffer, size_of_item, number_of_items count, filepointer);
static ssize_t counter_read(struct file *filp, char *buf, size_t count,
                            loff_t *f_pos) {
  int res;
  u32 val[2];

#ifdef _DEBUG
  printk("<1>counter: %d bytes requested by user. Offset:%d\n",count,*f_pos);
#endif

  // copy the measured frequency to the user space program as a 4-byte integer value
  // since the user space program runs on the same processor architecture as this driver, the value can be read directly by: fread(&frequency,4,1,filepointer);
  val[0] = frequency;
  val[1] = periodes;
  res = copy_to_user(buf, val, 8);

  if (*f_pos == 0)
    return 8;
  //if(*f_pos == 0) return 4;

  return 0;
}

// Structure that declares the usual file access functions
struct file_operations counter_fileops = {
  read: counter_read,
  write: counter_write,
  open: counter_open,
  release: counter_release
};

// ========================================================================================================================
// IRQ: Interrupt routines called by the kernel in case of an interrupt
// ========================================================================================================================

// timer IRQ, will be called by the kernel if the timer expires (i.e. after one second)
static void timer_fcounter(unsigned long arg) {
  /*#ifdef DEBUG_LED
   // read the source of the IRQ
   static char stat=0;

   // change status of the ext. LED
   if(stat == 0)
   {
   stat = 1;
   gpio_set_value(gpio_data.output_pin,1);
   }
   else
   {
   stat = 0;
   gpio_set_value(gpio_data.output_pin,0);
   }
   #endif
   */
  // store the number of detected edges of the input signal within the last time period
  frequency = gpio_data.fcounter;
  gpio_data.fcounter = 0;     // reset counter

  // restart timer for the next period
  // the "-1" was required to correct the time span
  mod_timer(&timer_frequency_measurement, jiffies + timespan * HZ - 1);
}

// timer IRQ running at 512 Hz (if ptimespan is 1) for periodic measurement
static void timer_pcounter(unsigned long arg) {
#ifdef DEBUG_LED
// read the source of the IRQ
  static char stat = 0;

  // change status of the ext. LED
  if (stat == 0) {
    stat = 1;
    gpio_set_value(gpio_data.output_pin, 1);
  } else {
    stat = 0;
    gpio_set_value(gpio_data.output_pin, 0);
  }
#endif

  // count periodic timer pulses
  gpio_data.pcounter++;

  mod_timer(&timer_periodic_measurement, jiffies + ptimespan);
}

// GPIO Interrupt Routine. Is called by the kernel for every edge detected on the input pin
static irqreturn_t input_irq_handler(int irq, void* dev_id) {
  struct myios* data = (struct myios*) dev_id;

  // count the detected edges on the input pin
  data->fcounter++;

  // store periodic measurement
  periodes = gpio_data.pcounter;
  gpio_data.pcounter = 0;

  return IRQ_HANDLED;  // this IRQ was handled
}

// ========================================================================================================================
// Setup, Init and Exit functions for this kernel module
// ========================================================================================================================

// Pinmux: as each pin has several functions, we need to tell which function we need
// for this definitions see the uC data sheet
#define AM33XX_CONTROL_BASE   0x44e10000
#define OUTPIN          (AM33XX_CONTROL_BASE + 0x878) // DebugLED pin (60)    : gpio1_28 (beaglebone p9/12)
#define INPIN         (AM33XX_CONTROL_BASE + 0x840) // Counter Input pin (48) : gpio1_16 (beaglebone p9/15)
#define OUTPIN_MODE       (0x7 | (2 << 3))                // mode 7 (gpio), PULLUP, OUTPUT
#define INPIN_MODE        (0x7 | (2 << 3) | (1 << 5))     // mode 7 (gpio), PULLUP, INPUT

static int setup_pinmux(void) {
  void *addr;

  // ioremap returns the physical address, this depends on the CPU used
#ifdef DEBUG_LED
  addr = ioremap(OUTPIN, 4);   // get physical port address
  if (!addr)
    return -EBUSY;

  iowrite32(OUTPIN_MODE, addr);  // and set the port mode
  gpio_data.output_pin = 60;    // and store the pin numbers in our structure
#endif

  addr = ioremap(INPIN, 4);    // get physical port address
  if (!addr)
    return -EBUSY;

  iowrite32(INPIN_MODE, addr);   // and set the port mode
  gpio_data.input_pin = 48;   // and store the pin numbers in our structure

  return 0;
}

// init function: will be called when driver is loaded with: insmod conter_driver.ko
static int counter_init(void) {
  int res, err;

  // register dev, verlinke es mit dem /dev/complete das vorher angelegt sein muss
  // register the device, link it to the major number (which was assigned when the device was
  // created with: mknod /dev/fcounter c 60 0
  res = register_chrdev(majornummer, "counter_module", &counter_fileops);
  if (res < 0) {
    printk("<1> counter: cannot find major number, is the device created ?\n");
    return res;
  }

  // Kernel-Timer: 1 second
  // this timer is used for frequency measurement
  // the pulses at the GPIO input are counted during this time, which results in pulses per second = Hz
  init_timer(&timer_frequency_measurement);         // init timer
  timer_frequency_measurement.function = timer_fcounter;  // this function is called when the timer expires
  timer_frequency_measurement.expires = jiffies + timespan * HZ;  // timespan*1 second (jiffies is the current system time and HZ are the ticks per second)
  add_timer(&timer_frequency_measurement);

  init_timer(&timer_periodic_measurement);          // init timer
  timer_periodic_measurement.function = timer_pcounter;  // this function is called when the timer expires
  timer_periodic_measurement.expires = jiffies + ptimespan;
  add_timer(&timer_periodic_measurement);

  // setup the GPIO ports
  // in this driver we use
  // P9-Pin12 as Output (ext. LED) and
  // P9-Pin15 as Input which generates an interrupt for each falling edge
  // this sets the function of the multiplexed GPIOs
  err = setup_pinmux();
  if (err < 0) {
    printk("<1>counter: failed to apply pinmux settings.\n");
    return err;
  }

  // request access to the GPIO pins. The kernel checks if these pins are already in use or not
#ifdef DEBUG_LED
  // first request access to the output line for our LED
  err = gpio_request_one(gpio_data.output_pin, GPIOF_OUT_INIT_HIGH,
  DRV_NAME " gpio");
  if (err < 0) {
    printk("<1> counter: failed to request GPIO output pin %d.\n",
           gpio_data.output_pin);
    return err;
  }
#endif

  // and for the input line
  err = gpio_request_one(gpio_data.input_pin, GPIOF_IN, DRV_NAME " irq");
  if (err < 0) {
    printk("<1> counter: failed to request GPIO input pin %d.\n",
           gpio_data.input_pin);
    gpio_free(gpio_data.output_pin);
    return err;
  }

  // now assign an IRQ for the input line
  res = gpio_to_irq(gpio_data.input_pin);
  if (res < 0) {
    printk("<1> counter : failed to get IRQ for input pin %d.\n",
           gpio_data.input_pin);
    gpio_free(gpio_data.input_pin);
    gpio_free(gpio_data.output_pin);
    return res;
  } else {
    // the kernel assigned irq number "res"
    gpio_data.irq = (u16) res;
    res = 0;
  }

  // now enable the IRQ
  err = request_any_context_irq(
      gpio_data.irq,
      input_irq_handler,
      IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME
          | IRQF_NO_THREAD,
      DRV_NAME, (void*) &gpio_data);  // (void*)&gpio_data ... will be the parameter *dev_id of the IRQ routine
  if (err < 0) {
    printk("<1> counter: failed to enable IRQ %d for pin %d.\n", gpio_data.irq,
           gpio_data.input_pin);
    gpio_free(gpio_data.input_pin);
    gpio_free(gpio_data.output_pin);
    return err;
  } else {
    // IRQ is enabled
    //gpio_data.irq_enabled = 1;
  }

#ifdef _DEBUG
  printk("<1> counter_driver: driver loaded sucessfully\n");
#endif

  return 0;
}

// exit function: will be called when driver is unloaded with: rmmod conter_driver.ko
// it is extremely important to release all reserved hardware, memory, timers ...
// or the kernel may hang up after removing the driver with rmmod !
static void counter_exit(void) {
  // remove the timers
  del_timer(&timer_frequency_measurement);
  del_timer(&timer_periodic_measurement);

  // release GPIO interrupt
  free_irq(gpio_data.irq, (void*) &gpio_data);

  // release GPIO pins
  gpio_free(gpio_data.input_pin);
  gpio_free(gpio_data.output_pin);

  // release Majornumber
  unregister_chrdev(majornummer, "complete_demo_module");

#ifdef _DEBUG
  printk("<1> counter_driver: driver unloaded\n");
#endif
}
