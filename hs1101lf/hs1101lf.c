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
#include <linux/module.h> /* Needed by all modules */
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/init.h>   /* Needed for the macros */

#define HS1101LF_DESCRIPTION   "Frequency hygrometer based on HS1101LF sensor"

static int debug = 1;
module_param(debug, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(debug, "debug mode");

static int __init hs1101lf_init(void)
{
  printk(KERN_INFO "hs1101lf-gpio init debug=%d\n", debug);
  return 0;
}

static void __exit hs1101lf_exit(void)
{
  printk(KERN_INFO "hs1101lf-gpio terminated\n");
}
module_init (hs1101lf_init);
module_exit (hs1101lf_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniele Colombo");
MODULE_DESCRIPTION(HS1101LF_DESCRIPTION);
MODULE_SUPPORTED_DEVICE("hs1101lf-gpio");
