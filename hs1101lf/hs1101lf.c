/*
 * hs1101lf.c
 *
 *  Created on: 23/giu/2017
 *      Author: daniele
 *
 *  Skeleton from http://elixir.free-electrons.com/linux/latest/source/drivers/iio/frequency/ad9523.c
 *
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/ktime.h>
#include <asm/div64.h>
#include <linux/sched.h>
#include <linux/init.h>   /* Needed for the macros */
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/gpio.h>      // GPIO functions/macros
#include <linux/delay.h>

#define  HS1101LF_FATAL(X,Y)  \
	do { \
  	  dev_err(dev,"hs1101lf.ko error line=%d err=%d\n",__LINE__,X); \
  	  goto Y;\
	}while(0)

struct hs1101lf_state {

	int gpio_data;
	enum of_gpio_flags gpio_data_flags;

	int gpio_power;
	enum of_gpio_flags gpio_power_flags;

	int irq;
	unsigned int humidity;
	volatile unsigned long counter;
	volatile unsigned long frequency;
	volatile int running;
	unsigned long sample_ms;
	wait_queue_head_t queue;
};

static ssize_t hs1101lf_counter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
	int ret;
	ret = sprintf(buf, "%lu\n", hs1101lf_state->counter);
	return ret;
}
static ssize_t hs1101lf_frequency_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
	int ret;
// Frequency in Hertz
	ret = sprintf(buf, "%lu\n", hs1101lf_state->frequency);
	return ret;
}
static ssize_t hs1101lf_sample_ms_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
	int ret;
	dev_info(&indio_dev->dev,
			"hs1101lf_sample_ms_show  hs1101lf_state->sample_ms=%lu",
			hs1101lf_state->sample_ms);
	ret = sprintf(buf, "%lu\n", hs1101lf_state->sample_ms);
	return ret;
}
static ssize_t hs1101lf_sample_ms_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
	int ret;
	unsigned long value;

	/*
	 FIXME: deal with locking and mutual ex or EAGAIN
	 mutex_lock(&st->lock);
	 */
	dev_info(&indio_dev->dev, "hs1101lf_sample_ms_store  buf=%s", buf);

	ret = kstrtoul(buf, 10, &value);
	if(ret)
		HS1101LF_FATAL(0, error);
	if(value < 10 || value > 10000)
		HS1101LF_FATAL(-EINVAL, error);

	hs1101lf_state->sample_ms = value;
	dev_info(&indio_dev->dev,
			"hs1101lf_sample_ms_store  hs1101lf_state->sample_ms=%lu",
			hs1101lf_state->sample_ms);

//	mutex_unlock(&st->lock);

	return len;

	error:
// wa formatter
//	mutex_unlock(&st->lock);
	return ret;
}
static IIO_DEVICE_ATTR(counter, S_IRUGO,
		hs1101lf_counter_show,
		NULL,
		0);
static IIO_DEVICE_ATTR(frequency, S_IRUGO,
		hs1101lf_frequency_show,
		NULL,
		0);
static IIO_DEVICE_ATTR(sample_ms, S_IRUGO | S_IWUSR,
		hs1101lf_sample_ms_show,
		hs1101lf_sample_ms_store,
		0);

static struct attribute *hs1101lf_attributes[] = {
		&iio_dev_attr_counter.dev_attr.attr,
		&iio_dev_attr_frequency.dev_attr.attr,
		&iio_dev_attr_sample_ms.dev_attr.attr,

		NULL, };

static const struct attribute_group hs1101lf_attribute_group = {
	.attrs = hs1101lf_attributes,
};
static irqreturn_t hs1101lf_gpio_irq_handler(int irq, void *data)
{
// FIXME: check if we have to lock mutex
	struct iio_dev *indio_dev = data;
	struct hs1101lf_state *hs1101lf_state;
	hs1101lf_state = iio_priv(indio_dev);
	if(hs1101lf_state->running)
		hs1101lf_state->counter++;
	return IRQ_HANDLED;
}
/**
 * hs1101lf_read_raw() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */

static int hs1101lf_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan, int *val, int *val2, long m)
{
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
	struct device *dev = &indio_dev->dev;
	ktime_t a, b, c;
	s64 d;
	s64 tmp;
	s64 timestamp;
// FIXME: USATO per ora come trigger
	int ret = 0;

	if(chan->type != IIO_HUMIDITYRELATIVE)
		return (-EINVAL);

	if(mutex_trylock(&indio_dev->mlock)) {
		hs1101lf_state->counter = 0;
		hs1101lf_state->humidity = 0;
		hs1101lf_state->running = 1;

		a = ktime_get();

		ret = devm_request_irq(dev, hs1101lf_state->irq,
				(irq_handler_t)hs1101lf_gpio_irq_handler,
				IRQF_TRIGGER_RISING, "hs1101lf", indio_dev);
		if(ret)
			HS1101LF_FATAL(ret, hs1101lf_error_free_mutex);
		wait_event_interruptible_timeout(hs1101lf_state->queue,
				(hs1101lf_state->running == 0),
				msecs_to_jiffies(hs1101lf_state->sample_ms));
		b = ktime_get();
		c = ktime_sub(b, a);
		d = ktime_to_ms(c);
		dev_info(&indio_dev->dev,
				"hs1101lf_read_raw  %llu %llu %llu elapsed=%llu",
				a.tv64, b.tv64, c.tv64, d);

		hs1101lf_state->running = 0;
		devm_free_irq(dev, hs1101lf_state->irq, indio_dev);
		tmp = hs1101lf_state->counter * 1000;
		hs1101lf_state->frequency = (unsigned int)do_div(tmp, d);
		hs1101lf_state->humidity = hs1101lf_state->frequency; // FIXME: frequency/humidity conversion

		timestamp = iio_get_time_ns();
		iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(IIO_HUMIDITYRELATIVE, 0,
						IIO_EV_TYPE_THRESH,
						IIO_EV_DIR_RISING), timestamp);

		dev_info(&indio_dev->dev,
				"hs1101lf_read_raw  data=%p indio_dev->event_interface=%p",
				hs1101lf_state, indio_dev->event_interface);
	}

	else
		HS1101LF_FATAL(-EAGAIN, hs1101lf_error);

	*val = hs1101lf_state->humidity;
	ret = IIO_VAL_INT;
	mutex_unlock(&indio_dev->mlock);
	dev_info(&indio_dev->dev, "hs1101lf_read_raw ret=%d val=%d", ret, *val);
	return ret;

	hs1101lf_error_free_mutex:
// formatter wa
	mutex_unlock(&indio_dev->mlock);
	hs1101lf_error:
// formatter wa
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

static int hs1101lf_read_event_config(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir)
{
//	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
//	unsigned int mask;
	dev_info(&indio_dev->dev, "hs1101lf_read_event_config  channel=%d",
			chan->channel);

	switch(chan->type) {
	case IIO_HUMIDITYRELATIVE:
		return 1;
	default:
		return -EINVAL;
	}

//	return !!(data->status_mask & mask);
}
static int hs1101lf_read_thresh(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int *val, int *val2)
{
	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);
//	int ret;
//	u8 reg;

	dev_info(&indio_dev->dev, "hs1101lf_read_thresh chan->type=%d",
			chan->type);

	if(chan->type != IIO_HUMIDITYRELATIVE)
		return (-EINVAL);

	*val = hs1101lf_state->humidity;

	return IIO_VAL_INT;
}
static const struct iio_info hs1101lf_info = {
	.read_raw = &hs1101lf_read_raw,
	.debugfs_reg_access = &hs1101lf_reg_access,
	.attrs = &hs1101lf_attribute_group,
	.read_event_config = hs1101lf_read_event_config,
	.read_event_value = hs1101lf_read_thresh,
	.driver_module = THIS_MODULE,
}
;

static const struct iio_event_spec hs1101lf_obj_event[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
		BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec hs1101lf_chan_spec[] = {
	{	.type = IIO_HUMIDITYRELATIVE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.event_spec = hs1101lf_obj_event,
		.num_event_specs = ARRAY_SIZE(hs1101lf_obj_event),
	}
};
static int hs1101lf_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct iio_dev *indio_dev;
	struct hs1101lf_state *hs1101lf_state;
	int ret = 0;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*hs1101lf_state));
	if(indio_dev == NULL)
		return -ENOMEM;

	hs1101lf_state = iio_priv(indio_dev);

	hs1101lf_state->sample_ms = 1000;
	init_waitqueue_head(&hs1101lf_state->queue);
	hs1101lf_state->gpio_data = of_get_named_gpio_flags(node, "data", 0,
			&hs1101lf_state->gpio_data_flags);
	if(!gpio_is_valid(hs1101lf_state->gpio_data))
		HS1101LF_FATAL(-EINVAL, error);
	hs1101lf_state->gpio_power = of_get_named_gpio_flags(node, "power-save",
			0, &hs1101lf_state->gpio_power_flags);

	platform_set_drvdata(pdev, indio_dev);
	indio_dev->name = pdev->name;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &hs1101lf_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = hs1101lf_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(hs1101lf_chan_spec);

	if((ret = devm_gpio_request(dev, hs1101lf_state->gpio_data, "hs1101lf")))
		HS1101LF_FATAL(ret, error);
	if((ret = gpio_direction_input(hs1101lf_state->gpio_data)))
		HS1101LF_FATAL(ret, error);
// FIXME: Verify that on at91 chip debounce is not supported really
	if((ret = gpio_set_debounce(hs1101lf_state->gpio_data, 0))
			&& ret != -ENOTSUPP)
		HS1101LF_FATAL(ret, error);
	if((ret = gpio_export(hs1101lf_state->gpio_data, false)))
		HS1101LF_FATAL(ret, error);
	hs1101lf_state->irq = gpio_to_irq(hs1101lf_state->gpio_data);
// devm_request_threaded_irq

	dev_info(&pdev->dev, "hs1101lf probe use data=%d power=%d",
			hs1101lf_state->gpio_data, hs1101lf_state->gpio_power);
	return devm_iio_device_register(dev, indio_dev);

	error:
// formatter wa
	devm_iio_device_free(dev, indio_dev);

	return ret;

}
static int hs1101lf_remove(struct platform_device *pdev)
{
//	struct device *dev = &pdev->dev;
//	struct iio_dev *indio_dev = dev_get_drvdata(dev);
//	struct hs1101lf_state *hs1101lf_state = iio_priv(indio_dev);

//	iio_device_unregister(indio_dev);
//	devm_iio_device_free(dev, indio_dev);

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
