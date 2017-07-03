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
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/of.h>
#include "hs1101lf_gpio.h"

#define HS1101LF_DESCRIPTION   "Frequency hygrometer based on HS1101LF sensor"

#if defined(CONFIG_OF)
static const struct of_device_id hs1101lf_gpio_dt_ids[] = {
  { .compatible = "hs1101lf-gpio"},
  {}
};
MODULE_DEVICE_TABLE(of, hs1101lf_gpio_dt_ids);
#endif

RIVEDERE CON IL DEVICE INA219 anche per uso attribute, group e mutex
il mutex è da usare per la misura e la lettura

static int hs1101lf_gpio_probe_dt(struct platform_device *pdev) {
  struct hs1101lf_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);
  struct device_node *np = pdev->dev.of_node;
  int gpio;

  /*
   * Allocate memory for platform data
   */
  pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
  if (!pdata)
  return -ENOMEM;

  /*
   * Getting data from device tree
   */
  if (of_get_property(np, "linux,open-drain", NULL))
  pdata->is_open_drain = 1;

  gpio = of_get_gpio(np, 0);
  if (gpio < 0) {
    if (gpio != -EPROBE_DEFER)
    dev_err(&pdev->dev, "Failed to parse gpio property for data pin (%d)\n",
        gpio);

    return gpio;
  }
  pdata->pin = gpio;

  gpio = of_get_gpio(np, 1);
  if (gpio == -EPROBE_DEFER)
  return gpio;

  pdev->dev.platform_data = pdata;

  dev_info(&pdev->dev, "HS1101LF PROBED: Pin=%d Drain=%d", pdata->pin,
      pdata->is_open_drain);
  return 0;
}

static int hs1101lf_gpio_probe(struct platform_device *pdev) {
  struct hs1101lf_gpio_platform_data *pdata;
  int err;

  /*
   * Device tree is ok probe
   */
  if (of_have_populated_dt()) {
    err = hs1101lf_gpio_probe_dt(pdev);
    if (err < 0)
      return err;
  }

  pdata = dev_get_platdata(&pdev->dev);

  if (!pdata) {
    dev_err(&pdev->dev, "No configuration data\n");
    return -ENXIO;
  }

  err = devm_gpio_request(&pdev->dev, pdata->pin, "hs1101lf");
  if (err) {
    dev_err(&pdev->dev, "gpio_request (pin) failed\n");
    return err;
  }

  if (pdata->is_open_drain) {
    gpio_direction_output(pdata->pin, 1);
  } else {
    gpio_direction_input(pdata->pin);
  }

  return 0;
}

static int hs1101lf_gpio_remove(struct platform_device *pdev) {

  return 0;
}

/*
 * Power optimization
 */
static int __maybe_unused hs1101lf_gpio_suspend(struct device *dev)
{

  return 0;
}

static int __maybe_unused hs1101lf_gpio_resume(struct device *dev)
{

  return 0;
}

static SIMPLE_DEV_PM_OPS(hs1101lf_gpio_pm_ops, hs1101lf_gpio_suspend, hs1101lf_gpio_resume);

static struct platform_driver hs1101lf_gpio_driver = {
  .driver = {
    .name = "hs1101lf-gpio",
    .pm = &hs1101lf_gpio_pm_ops,
    .of_match_table = of_match_ptr(hs1101lf_gpio_dt_ids),
  },
  .probe = hs1101lf_gpio_probe,
  .remove = hs1101lf_gpio_remove,
};

module_platform_driver (hs1101lf_gpio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniele Colombo");
MODULE_DESCRIPTION(HS1101LF_DESCRIPTION);
MODULE_SUPPORTED_DEVICE("hs1101lf-gpio");
