/*
 * hs1101lf.c
 *
 *  Created on: 23/giu/2017
 *      Author: daniele
 *
 *  Skeleton from http://elixir.free-electrons.com/linux/latest/source/drivers/iio/frequency/ad9523.c
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
#include <linux/gpio.h>      // GPIO functions/macros
#include <linux/interrupt.h> // interrupt functions/macros

#include "hs1101lf.h"

#define  HS1101LF_FATAL(X)  \
	do { \
  	  line = __LINE__; \
  	  goto X; \
        } while(0)

struct hs1101lf_state {

	int gpio_data;
	enum of_gpio_flags gpio_data_flags;

	int gpio_power;
	enum of_gpio_flags gpio_power_flags;

	int irq;
	int humidity;
	volatile unsigned long counter;

};

static ssize_t hs1101lf_counter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
//  struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret;
	ret = sprintf(buf, "%lu\n", hs1101lf_state->counter);
	return ret;
}
static IIO_DEVICE_ATTR(counter, S_IRUGO,
		hs1101lf_counter_show,
		NULL,
		0);

static struct attribute *hs1101lf_attributes[] = {
		&iio_dev_attr_counter.dev_attr.attr, NULL, };

static const struct attribute_group hs1101lf_attribute_group = {
	.attrs = hs1101lf_attributes,
};
static int hs1101lf_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2, long m)
{
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	hs1101lf_state->humidity = gpio_get_value(hs1101lf_state->gpio_data);

	ret = IIO_VAL_INT;
	if(chan->type == IIO_HUMIDITYRELATIVE)
		*val = hs1101lf_state->humidity;
	else
		ret = -EINVAL;

	mutex_unlock(&indio_dev->mlock);
	dev_info(&indio_dev->dev, "hs1101lf_read_raw ret=%d val=%d", ret, *val);
	return ret;
}
static int hs1101lf_reg_access(struct iio_dev *indio_dev, unsigned int reg,
		unsigned int writeval, unsigned int *readval)
{
	mutex_lock(&indio_dev->mlock);
	dev_info(&indio_dev->dev,
			"hs1101lf_reg_access reg=%d writeval=%d readval=%p",
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
	{	.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),}
};
static irqreturn_t hs1101lf_gpio_irq_handler(int irq, void *data)
{
// FIXME: check if we have to lock mutex
	struct iio_dev *indio_dev = data;
	struct hs1101lf_state *hs1101lf_state;
	hs1101lf_state = iio_priv(indio_dev);
	hs1101lf_state->counter++;
//	dev_info(&indio_dev->dev, "hs1101lf_gpio_irq_handler irq=%d data=%p",
//	irq, data
//	);
	return IRQ_HANDLED;
}
static int hs1101lf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct iio_dev *indio_dev;
	struct hs1101lf_state *hs1101lf_state;
	int err = 0;
	int line = 0;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*hs1101lf_state));
	if(indio_dev == NULL)
		return -ENOMEM;

	hs1101lf_state = iio_priv(indio_dev);
	hs1101lf_state->gpio_data = of_get_named_gpio_flags(node, "data", 0,
			&hs1101lf_state->gpio_data_flags);
	if(!gpio_is_valid(hs1101lf_state->gpio_data))
		HS1101LF_FATAL(err_free_device);
	hs1101lf_state->gpio_power = of_get_named_gpio_flags(node, "power_save",
			0, &hs1101lf_state->gpio_power_flags);

	platform_set_drvdata(pdev, indio_dev);
	indio_dev->name = pdev->name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &hs1101lf_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = hs1101lf_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(hs1101lf_chan_spec);

	if((err = devm_gpio_request(dev, hs1101lf_state->gpio_data, "hs1101lf")))
		HS1101LF_FATAL(err_free_device);
	if((err = gpio_direction_input(hs1101lf_state->gpio_data)))
		HS1101LF_FATAL(err_free_device);
// FIXME: Verify that on at91 chip debounce is not supported really
	if((err = gpio_set_debounce(hs1101lf_state->gpio_data, 0))
			&& err != -ENOTSUPP)
		HS1101LF_FATAL(err_free_device);
	if((err = gpio_export(hs1101lf_state->gpio_data, false)))
		HS1101LF_FATAL(err_free_device);
	hs1101lf_state->irq = gpio_to_irq(hs1101lf_state->gpio_data);
	if(devm_request_irq(dev, hs1101lf_state->irq,
			(irq_handler_t)hs1101lf_gpio_irq_handler,
			IRQF_TRIGGER_RISING, "hs1101lf", indio_dev))
		HS1101LF_FATAL(err_free_device);

	dev_info(&pdev->dev, "hs1101lf probe use data=%d power=%d",
			hs1101lf_state->gpio_data, hs1101lf_state->gpio_power);
	return devm_iio_device_register(dev, indio_dev);

	err_free_device:
// formatter wa
	devm_iio_device_free(dev, indio_dev);

	return err;

}
static int hs1101lf_remove(struct platform_device *pdev)
{

	dev_info(&pdev->dev, "hs1101lf_remove");
	return 0;
}
static const struct of_device_id hs1101lf_dt_ids[] = {
	{	.compatible = "hs1101lf"},
	{}
};
MODULE_DEVICE_TABLE(of, hs1101lf_dt_ids);

static int __maybe_unused hs1101lf_suspend(struct device *dev)
{
//	struct w1_gpio_platform_data *pdata = dev_get_platdata(dev);

	return 0;
}

static int __maybe_unused hs1101lf_resume(struct device *dev)
{
//	struct w1_gpio_platform_data *pdata = dev_get_platdata(dev);

	return 0;
}

static SIMPLE_DEV_PM_OPS(hs1101lf_pm_ops, hs1101lf_suspend, hs1101lf_resume);

static struct platform_driver hs1101lf_driver = {
	.driver = {
		.name = "hs1101lf",
		.pm = &hs1101lf_pm_ops,
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
