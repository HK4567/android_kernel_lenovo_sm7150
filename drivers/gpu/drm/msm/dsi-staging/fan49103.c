#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#ifdef CONFIG_DRM
#include <linux/msm_drm_notify.h>
#endif


#define FAN49103_I2C_NAME              "Onsemi-fan49103"

/* Register Map */
#define SOFT_RESET_REG		0x00
#define VOUT_REF_REG			0x01
#define CONTROL_REG			0x02
#define MANUFACTURER_REG	0x40
#define DEVICE_ID_REG			0x41

#define FAN_INFO(fmt,arg...)    	printk("Fan49103-INFO "fmt"\n",##arg)
#define FAN_ERROR(fmt,arg...)   	printk("Fan49103-ERROR "fmt"\n",##arg)

struct fan49103_pdata {
	int pwrgood_gpio;
};

struct fan49103_data {
	struct fan49103_pdata *pdata;
	struct i2c_client *client;
#if defined(CONFIG_DRM)
	struct notifier_block drm_notifier;
#endif

	int irq;
	bool power_status;
};

//static struct fan49103_data *g_data;
static struct i2c_client* g_i2c_client = NULL;

static ssize_t fan49103_pwr_good_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf) {
	struct fan49103_data *data = dev_get_drvdata(dev);
	struct fan49103_pdata *pdata = data->pdata;
	int ret;

	if(pdata != NULL) {
		if( gpio_is_valid(pdata->pwrgood_gpio)) {
			ret = gpio_get_value(pdata->pwrgood_gpio);
		}
	} else
		FAN_ERROR("data is null\n");

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);
}

static DEVICE_ATTR(pwr_good, S_IRUGO, fan49103_pwr_good_status_show, NULL);

static struct attribute *fan49103_attributes[] = {
	&dev_attr_pwr_good.attr,
	NULL,
};

static const struct attribute_group fan49103_attr_group = {
	.attrs = fan49103_attributes,
};

int fan49103_init_regs() {
	int ret;

	if(!g_i2c_client) {
		FAN_ERROR("g_i2c_client is NULL");
		return -EINVAL;
	}

	ret = i2c_smbus_write_byte_data(g_i2c_client, VOUT_REF_REG, 0x5F);	//0x5F:4.0v
	if(ret) {
		dev_err(&g_i2c_client->dev, "Fail to set reg[0x%x], ret = %d\n", VOUT_REF_REG, ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(g_i2c_client, CONTROL_REG, 0x04);	//0x04:PWM Mode
	if(ret) {
		dev_err(&g_i2c_client->dev, "Fail to set reg[0x%x], ret = %d\n", CONTROL_REG, ret);
		return ret;
	}

	return 0;
}

static int fan49103_request_io_port(struct fan49103_data *data)
{
	struct i2c_client *client = data->client;
	struct fan49103_pdata *pdata = data->pdata;
	int ret;

	if (gpio_is_valid(pdata->pwrgood_gpio)) {	/* pm6150a_gpio1 */
		ret = gpio_request_one(pdata->pwrgood_gpio, GPIOF_DIR_IN, "fan49103_pwrgood_gpio");
		if (ret) {
			pdata->pwrgood_gpio = -1;
			dev_err(&client->dev, "Fail to request pwrgood gpio [%d]\n",
				pdata->pwrgood_gpio);
			return -EINVAL;
		}
	} else {
		dev_err(&client->dev, "Invalid pwrgood gpio [%d]!\n", pdata->pwrgood_gpio);
		return -EINVAL;
	}

	data->irq = gpio_to_irq(pdata->pwrgood_gpio);

	return 0;
}

static void fan49103_free_io_port(struct fan49103_pdata *pdata) {
	if (gpio_is_valid(pdata->pwrgood_gpio))
		gpio_free(pdata->pwrgood_gpio);
}

static int fan49103_parse_dt(struct device *dev,
			struct fan49103_pdata *pdata)
{
	struct device_node *np = dev->of_node;

	pdata->pwrgood_gpio = of_get_named_gpio(np, "onsemi,pwrgood-gpio", 0);
	if (!gpio_is_valid(pdata->pwrgood_gpio)) {
		FAN_ERROR("pwrgood gpio is not specified.");
		return -ENODEV;
	}

	return 0;
}

#ifdef FAN_ENABLE_IRQ
static irqreturn_t fan49103_irq_handler(int irq, void *dev_id)
{
	struct fan49103_data *data = dev_id;
	struct fan49103_pdata *pdata = data->pdata;
	int ret;

	if(gpio_is_valid(pdata->pwrgood_gpio)) {
		ret = gpio_get_value(pdata->pwrgood_gpio);
		if(!ret)
			FAN_ERROR("Error on pwrgood pin");
	}

	return IRQ_HANDLED;
}
#endif

static int fan49103_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fan49103_pdata *pdata;
	struct fan49103_data *data;
	int ret;

	FAN_INFO("enter %s\n\n", __func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		FAN_INFO("I2C check functionality failed.");
		return -ENODEV;
   	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct fan49103_pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		ret = fan49103_parse_dt(&client->dev, pdata);
		if (ret) {
			dev_err(&client->dev, "Failed to parse dts.\n");
			return -EINVAL;
		}
	} else {
		pdata = client->dev.platform_data;
	}

	if (!pdata) {
		dev_err(&client->dev, "fan49103 invalid pdata\n");
		return -EINVAL;
	}

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	memset(data, 0, sizeof(*data));
	data->client = client;
	data->pdata = pdata;

	dev_set_drvdata(&client->dev, data);

	ret = fan49103_request_io_port(data);
	if (ret) {
		dev_err(&client->dev, "fan49103 request IO port failed, ret = %d\n", ret);
		goto exit_free_client_data;
	}

	/* Read the chip id */
	ret = i2c_smbus_read_byte_data(client, DEVICE_ID_REG);
	if (ret < 0) {
		dev_err(&client->dev, "I2C communication ERROR!, ret = %d\n", ret);
		goto exit_free_io_port;
	}
	FAN_INFO("device id: 0x%x", ret);

	g_i2c_client = client;

	ret = fan49103_init_regs();
	if(ret) {
		dev_err(&client->dev, "fan49103 init regs failed, ret = %d\n", ret);
		goto exit_free_io_port;
	}

	ret = sysfs_create_group(&client->dev.kobj, &fan49103_attr_group);
	if (ret) {
		dev_err(&client->dev, "failed to register sysfs. err: %d\n", ret);
		goto exit_free_io_port;
	}

#ifdef FAN_ENABLE_IRQ
	ret = request_threaded_irq(data->irq, NULL,
			fan49103_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			client->name, data);
	if (ret) {
		goto exit_destroy_group;
	}
#endif

	return 0;

#ifdef FAN_ENABLE_IRQ
exit_destroy_group:
	sysfs_remove_group(&client->dev.kobj, &fan49103_attr_group);
#endif
exit_free_io_port:
	fan49103_free_io_port(pdata);
exit_free_client_data:
	i2c_set_clientdata(client, NULL);
	g_i2c_client = NULL;

	return ret;
}

static int fan49103_remove(struct i2c_client *client)
{
	struct fan49103_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &fan49103_attr_group);
#ifdef CONFIG_DRM
	msm_drm_unregister_client(&data->drm_notifier);
#endif
	fan49103_free_io_port(data->pdata);
#ifdef FAN_ENABLE_IRQ
	free_irq(client->irq, data);
#endif
	i2c_set_clientdata(client, NULL);
	g_i2c_client = NULL;
	kfree(data);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id fan49103_match_table[] = {
		{.compatible = "onsemi,fan49103",},
		{ },
};
#endif

static const struct i2c_device_id fan49103_id[] = {
    { FAN49103_I2C_NAME, 0 },
    { }
};

static struct i2c_driver fan49103_driver = {
    .probe      = fan49103_probe,
    .remove     = fan49103_remove,
    .id_table   = fan49103_id,
    .driver = {
        .name     = FAN49103_I2C_NAME,
        .owner    = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = fan49103_match_table,
#endif
    },
};

static int __init fan49103_init(void)
{
    s32 ret;
    ret = i2c_add_driver(&fan49103_driver);

    return ret;
}

static void __exit fan49103_exit(void)
{
    i2c_del_driver(&fan49103_driver);
}

module_init(fan49103_init);
module_exit(fan49103_exit);

MODULE_DESCRIPTION("Onsemi fan49103 Driver");
MODULE_LICENSE("GPL");
