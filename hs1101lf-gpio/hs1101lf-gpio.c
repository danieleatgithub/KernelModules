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

int init_module(void) {
  printk(KERN_INFO "Hello world 1.\n");

  /* 
   * A non 0 return means init_module failed; module can't be loaded. 
   */
  return 0;
}

void cleanup_module(void) {
printk(KERN_INFO "Goodbye world 1.\n");
}
