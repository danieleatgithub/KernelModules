/*
 * hs1101lf.c
 *
 *  Created on: 23/giu/2017
 *      Author: daniele
 *
 *  Skeleton from /drivers/iio/frequency/ad9523.c
 *
 */
#include <linux/module.h> /* Needed by all modules */
#include <linux/kernel.h> /* Needed for KERN_INFO */
#include <linux/init.h>   /* Needed for the macros */
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "hs1101lf.h"

struct hs1101lf_state {

  int gpio;
  int irq;
  int humidity;
  unsigned long counter;

};

static ssize_t hs1101lf_counter_show(struct device *dev,
                                     struct device_attribute *attr, char *buf) {
//  struct iio_dev *indio_dev = dev_to_iio_dev(dev);
//  struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
  int ret;

  ret = sprintf(buf, "%d\n", 10);
  return ret;
}
static IIO_DEVICE_ATTR(counter, S_IRUGO,
    hs1101lf_counter_show,
    NULL,
    0);

static struct attribute *hs1101lf_attributes[] = { &iio_dev_attr_counter
    .dev_attr.attr, NULL, };

static const struct attribute_group hs1101lf_attribute_group = {
  .attrs = hs1101lf_attributes,
};
static int hs1101lf_read_raw(struct iio_dev *indio_dev,
                             struct iio_chan_spec const *chan, int *val,
                             int *val2, long m) {
  struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
  int ret;

  mutex_lock(&indio_dev->mlock);
  hs1101lf_state->humidity = 1234;

  ret = IIO_VAL_INT;
  if (chan->type == IIO_HUMIDITYRELATIVE)
    *val = hs1101lf_state->humidity;
  else
    ret = -EINVAL;

  mutex_unlock(&indio_dev->mlock);
  dev_info(&indio_dev->dev, "hs1101lf_read_raw ret=%d val=%d", ret, *val);
  return ret;
}
static int hs1101lf_reg_access(struct iio_dev *indio_dev, unsigned int reg,
                               unsigned int writeval, unsigned int *readval) {
  mutex_lock(&indio_dev->mlock);
  dev_info(&indio_dev->dev, "hs1101lf_reg_access reg=%d writeval=%d readval=%p",
           reg, writeval, readval);
  mutex_unlock(&indio_dev->mlock);

  return 0;
}
static const struct iio_info hs1101lf_info = {
  .read_raw = &hs1101lf_read_raw,
  .debugfs_reg_access = &hs1101lf_reg_access,
  .attrs = &hs1101lf_attribute_group,
  .driver_module = THIS_MODULE,
};
static const struct iio_chan_spec hs1101lf_chan_spec[] = {
  { .type = IIO_HUMIDITYRELATIVE,
    .info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),}
};

static int hs1101lf_probe(struct platform_device *pdev) {
  struct device *dev = &pdev->dev;
  struct device_node *node = dev->of_node;
  struct iio_dev *indio_dev;
  struct hs1101lf_state *hs1101lf_state;
  int ret;

  indio_dev = devm_iio_device_alloc(dev, sizeof(*hs1101lf_state));
  if (indio_dev == NULL)
    return -ENOMEM;

  ret = of_get_gpio(node, 0);
  if (ret < 0)
    return ret;

  hs1101lf_state = iio_priv(indio_dev);

  hs1101lf_state->gpio = ret;

  ret = devm_gpio_request_one(dev, hs1101lf_state->gpio, GPIOF_IN, pdev->name);
  if (ret)
    return ret;

  hs1101lf_state = iio_priv(indio_dev);

  platform_set_drvdata(pdev, indio_dev);

  indio_dev->name = pdev->name;
  indio_dev->dev.parent = &pdev->dev;
  indio_dev->info = &hs1101lf_info;
  indio_dev->modes = INDIO_DIRECT_MODE;
  indio_dev->channels = hs1101lf_chan_spec;
  indio_dev->num_channels = ARRAY_SIZE(hs1101lf_chan_spec);
  dev_info(&pdev->dev, "HS1101LF PROBED: Pin=%d", hs1101lf_state->gpio);
  return devm_iio_device_register(dev, indio_dev);

}
static int hs1101lf_remove(struct platform_device *pdev) {

  dev_info(&pdev->dev, "hs1101lf_remove");
  return 0;
}
static const struct of_device_id hs1101lf_dt_ids[] = {
  { .compatible = "hs1101lf"},
  {}
};
MODULE_DEVICE_TABLE(of, hs1101lf_dt_ids);

static struct platform_driver hs1101lf_driver = {
  .driver = {
    .name = "hs1101lf",
    .of_match_table = of_match_ptr(hs1101lf_dt_ids),
  },
  .probe = hs1101lf_probe,
  .remove = hs1101lf_remove,
};

module_platform_driver (hs1101lf_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Daniele Colombo");
MODULE_DESCRIPTION("Frequency hygrometer based on HS1101LF sensor");
MODULE_SUPPORTED_DEVICE("hs1101lf");
