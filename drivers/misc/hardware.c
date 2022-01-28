#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/iio/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/soc/qcom/smem.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <misc/hardware.h>

#define HARDWARE_MAX_ITEM_LONGTH 64

const char *sku_info[] = {
	[SKU_LTE_ROW] = "lte_row",
	[SKU_LTE_PRC] = "lte_prc",
	[SKU_WIFI_ROW] = "wifi_row",
	[SKU_WIFI_PRC] = "wifi_prc",
	[SKU_INVALID] = "unknow",
};

struct hw_info {
	int min;
	int max;
	enum HW_INFO hw;
};

struct hw_info menu[] = {
	{0, 100000, HW_EVB},
	{161500, 178500, HW_EVT},
	{570000, 630000, HW_DVT1},
	{893000, 987000, HW_DVT2},
	{1187500, 1312500, HW_PVT},
};
const char *hw_info_str[] = {
	[HW_EVB] = "evb",
	[HW_EVT] = "evt",
	[HW_DVT1] = "dvt1",
	[HW_DVT2] = "dvt2",
	[HW_PVT] = "pvt",
	[HW_INVALID] = "unknow",
};

struct hardware {
	struct list_head node;
	char name[HARDWARE_MAX_ITEM_LONGTH];
	char model[HARDWARE_MAX_ITEM_LONGTH];
};

struct hardwareinfo {
	int gpio6;
	int gpio7;
	struct iio_channel	*gpio1;
};

/*#define SMEM_HW_SW_BUILD_ID             137 socinfo.h*/
#define SMEM_ID_VENDOR2         136
#define NV_WIFI_ADDR_SIZE	6
#define NV_BT_ADDR_SIZE		6
#define NV_FACTORY_SIZE		128
#define NV_MAX_SIZE		(NV_WIFI_ADDR_SIZE + NV_BT_ADDR_SIZE + NV_WIFI_ADDR_SIZE)

#define MAC_ADDR_ARRAY(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MAC_ADDR_STRING "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC_ADDR_LENGTH 18

/*follow V2.14*/
#define PSN_OFFSET 0
#define PSN_SIZE 23
#define GSN_OFFSET 30
#define GSN_SIZE 8
#define FLAG_OFFSET 40
#define FLAG_SIZE 20
#define CCODE_OFFSET 60
#define CCODE_SIZE 4

struct smem_nv {
	unsigned char nv_wifi[NV_WIFI_ADDR_SIZE];
	unsigned char nv_bt[NV_BT_ADDR_SIZE];
	unsigned char nv_factory[NV_FACTORY_SIZE];
};

/* List of hardware handles (struct hardware) */
static LIST_HEAD(hardware_list);
static DEFINE_MUTEX(hardware_list_mutex);
static enum HW_INFO hw = HW_INVALID;
static enum SKU_INFO sku = SKU_INVALID;

struct smem_nv * gsmem_nv = NULL;

static struct smem_nv * smem_read_nv(void)
{
	struct smem_nv * buf;
	size_t size;

	if (gsmem_nv != NULL)
		return gsmem_nv;

	buf = qcom_smem_get(QCOM_SMEM_HOST_ANY, SMEM_ID_VENDOR2, &size);
	if (IS_ERR_OR_NULL(buf)) {
		printk(KERN_ERR "SMEM_ID_VENDOR_READ_NV smem_alloc failed\n");
		buf = NULL;
	}

	gsmem_nv = buf;

	return buf;
}

int register_hardware_info(const char *name, const char *model)
{
	struct hardware *h;
	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (h == NULL) {
		pr_info("failed to alloc struct hardware\n");
		return -1;
	}

	if (strlen(name) > HARDWARE_MAX_ITEM_LONGTH || strlen(model) > HARDWARE_MAX_ITEM_LONGTH) {
		pr_info("name or model length is high %d\n", HARDWARE_MAX_ITEM_LONGTH);
		return -1;
	}
	strcpy(h->name, name);
        strcpy(h->model, model);

	mutex_lock(&hardware_list_mutex);
	list_add_tail(&h->node, &hardware_list);
	mutex_unlock(&hardware_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(register_hardware_info);

int unregister_hardware_info(const char *name)
{
	struct hardware *h, *tmp;

	mutex_lock(&hardware_list_mutex);
	list_for_each_entry_safe(h, tmp, &hardware_list, node) {
		if (strcmp(h->name, name) == 0) {
			list_del(&h->node);
			kfree(h);
		}
	}
	mutex_unlock(&hardware_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(unregister_hardware_info);

enum HW_INFO hwid_get_stage(void)
{
	return hw;
}

enum SKU_INFO hwid_get_sku(void)
{
	return sku;
}

static ssize_t hardwareinfo_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	char temp_buffer[HARDWARE_MAX_ITEM_LONGTH];
	int buf_size = 0;
	struct hardware *h;

	mutex_lock(&hardware_list_mutex);
	list_for_each_entry(h, &hardware_list, node) {
		memset(temp_buffer, 0, HARDWARE_MAX_ITEM_LONGTH);
		sprintf(temp_buffer, "%s: %s\n", h->name, h->model);
		strcat(buf, temp_buffer);
		buf_size +=strlen(temp_buffer);
	}
	mutex_unlock(&hardware_list_mutex);

	return buf_size;
}

static ssize_t hwid_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int buf_size = 0;

	mutex_lock(&hardware_list_mutex);
	buf_size = sprintf(buf, "%s_%s\n", sku_info[sku], hw_info_str[hw]);
	mutex_unlock(&hardware_list_mutex);

	return buf_size;
}

/*psn*/
static ssize_t psn_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct smem_nv *psmem_nv = smem_read_nv();

	mutex_lock(&hardware_list_mutex);
	if (psmem_nv != NULL)
		memcpy(buf, &psmem_nv->nv_factory[PSN_OFFSET], PSN_SIZE);
	else
		memset(buf, 0, PSN_SIZE);
	mutex_unlock(&hardware_list_mutex);

	return PSN_SIZE;
}
/*gsn*/
static ssize_t gsn_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct smem_nv *psmem_nv = smem_read_nv();

	mutex_lock(&hardware_list_mutex);
	if (psmem_nv != NULL)
		memcpy(buf, &psmem_nv->nv_factory[GSN_OFFSET], GSN_SIZE);
	else
		memset(buf, 0x0, GSN_SIZE);
	mutex_unlock(&hardware_list_mutex);

	return GSN_SIZE;
}
/*ccode*/
static ssize_t ccode_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct smem_nv *psmem_nv = smem_read_nv();

	mutex_lock(&hardware_list_mutex);
	if (psmem_nv != NULL)
		memcpy(buf, &psmem_nv->nv_factory[CCODE_OFFSET], CCODE_SIZE);
	else
		memset(buf, 0, CCODE_SIZE);
	mutex_unlock(&hardware_list_mutex);

	return CCODE_SIZE;
}
/*flag*/
static ssize_t flag_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct smem_nv *psmem_nv = smem_read_nv();

	mutex_lock(&hardware_list_mutex);
	if (psmem_nv != NULL)
		memcpy(buf, &psmem_nv->nv_factory[FLAG_OFFSET], FLAG_SIZE);
	else
		memset(buf, 0, FLAG_SIZE);
	mutex_unlock(&hardware_list_mutex);

	return FLAG_SIZE;
}
/*wifi*/
static ssize_t wifi_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int buf_size = 0;
	struct smem_nv *psmem_nv = smem_read_nv();

	mutex_lock(&hardware_list_mutex);
	if (psmem_nv != NULL)
		buf_size = snprintf(buf, MAC_ADDR_LENGTH, MAC_ADDR_STRING, MAC_ADDR_ARRAY(psmem_nv->nv_wifi));
	else
		buf_size = sprintf(buf, "00:00:00:00");
	mutex_unlock(&hardware_list_mutex);

	return buf_size;
}
/*bt*/
static ssize_t bt_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int buf_size = 0;
	struct smem_nv *psmem_nv = smem_read_nv();

	mutex_lock(&hardware_list_mutex);
	if (psmem_nv != NULL)
		buf_size = snprintf(buf, MAC_ADDR_LENGTH, MAC_ADDR_STRING, MAC_ADDR_ARRAY(psmem_nv->nv_bt));
	else
		buf_size = sprintf(buf, "00:00:00:00");
	mutex_unlock(&hardware_list_mutex);

	return buf_size;
}

static ssize_t nvvalid_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int buf_size = 0;
	struct smem_nv *psmem_nv = smem_read_nv();

	mutex_lock(&hardware_list_mutex);
	if (psmem_nv != NULL)
		buf_size = sprintf(buf, "1");
	else
		buf_size = sprintf(buf, "0");
	mutex_unlock(&hardware_list_mutex);

	return buf_size;
}

/*ddr*/
static ssize_t ddr_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int buf_size = 0;
	struct sysinfo val;
        int temp_size;

	mutex_lock(&hardware_list_mutex);
	si_meminfo(&val);
        temp_size = ((val.totalram << (PAGE_SHIFT - 10)) / (1024*1024) + 1);
        buf_size = sprintf(buf, "%d", temp_size);
	mutex_unlock(&hardware_list_mutex);

	return buf_size;
}

static struct kobj_attribute hw_info_attr = __ATTR_RO(hardwareinfo);
static struct kobj_attribute hw_id_attr = __ATTR_RO(hwid);
static struct kobj_attribute psn_attr = __ATTR_RO(psn);
static struct kobj_attribute gsn_attr = __ATTR_RO(gsn);
static struct kobj_attribute ccode_attr = __ATTR_RO(ccode);
static struct kobj_attribute flag_attr = __ATTR_RO(flag);
static struct kobj_attribute wifi_attr = __ATTR_RO(wifi);
static struct kobj_attribute bt_attr = __ATTR_RO(bt);
static struct kobj_attribute ddr_attr = __ATTR_RO(ddr);
static struct kobj_attribute valid_attr = __ATTR_RO(nvvalid);

struct kobject *hwinfo_kobj;
EXPORT_SYMBOL(hwinfo_kobj);

static struct attribute *hwinfo_attrs[] = {
	&hw_info_attr.attr,
	&hw_id_attr.attr,
	&psn_attr.attr,
	&gsn_attr.attr,
	&ccode_attr.attr,
	&flag_attr.attr,
	&wifi_attr.attr,
	&bt_attr.attr,
	&ddr_attr.attr,
	&valid_attr.attr,
	NULL
};

static struct attribute_group hwinfo_attr_group = {
	.attrs = hwinfo_attrs,
};


#ifdef CONFIG_OF
static int hardware_parse_dt(struct device *dev, struct hardwareinfo *data)
{
	int rc;

	data->gpio6 = of_get_named_gpio_flags(dev->of_node, "hwid-gpio6", 0, NULL);
	if (!gpio_is_valid(data->gpio6)) {
		dev_err(dev, "hall gpio is not valid\n");
		return -EINVAL;
	}

	data->gpio7 = of_get_named_gpio_flags(dev->of_node, "hwid-gpio7", 0, NULL);
	if (!gpio_is_valid(data->gpio7)) {
		dev_err(dev, "hall gpio is not valid\n");
		return -EINVAL;
	}

	rc = of_property_match_string(dev->of_node, "io-channel-names", "hwid_gpio1");
	if (rc >= 0) {
		data->gpio1 = iio_channel_get(dev, "hwid_gpio1");
		if (IS_ERR(data->gpio1)) {
			rc = PTR_ERR(data->gpio1);
			if (rc != -EPROBE_DEFER)
				dev_err(dev,
					"hwid_gpio1 channel unavailable %d\n", rc);
			data->gpio1 = NULL;
			return rc;
		}
	} else {
		return rc;
	}

	return 0;
}
#else
static int hardware_parse_dt(struct device *dev, struct hall_data *data)
{
	return -EINVAL;
}
#endif


static int hardware_info_probe(struct platform_device *dev)
{
	int rc = 0, adc_hw, i = 0;
	struct hardwareinfo *data;

	printk("%s: start\n", __func__);

	//hardware id
	data = devm_kzalloc(&dev->dev, sizeof(struct hardwareinfo), GFP_KERNEL);
	if (data == NULL) {
		rc = -ENOMEM;
		dev_err(&dev->dev, "failed to allocate memory %d\n", rc);
		goto exit;
	}
	dev_set_drvdata(&dev->dev, data);
	if (dev->dev.of_node) {
		rc = hardware_parse_dt(&dev->dev, data);
		if (rc < 0) {
			dev_err(&dev->dev, "Failed to parse device tree rc=%d\n", rc);
			goto exit;
		}
	} else if (dev->dev.platform_data != NULL) {
		memcpy(data, dev->dev.platform_data, sizeof(*data));
	} else {
		dev_err(&dev->dev, "No valid platform data.\n");
		rc = -ENODEV;
		goto exit;
	}

	rc = gpio_request(data->gpio6, "hwid-gpio6");
	if (rc < 0) {
		dev_err(&dev->dev, "hwid-gpio6 request failed\n");
		goto exit;
	}

	rc = gpio_request(data->gpio7, "hwid-gpio7");
	if (rc < 0) {
		dev_err(&dev->dev, "hwid-gpio7 request failed\n");
		goto gpio_fail6;
	}

	//hardware info
	register_hardware_info("CPU", "SM7150");
	register_hardware_info("Mic", "Knowles_SPH0655LM4H-1");
	register_hardware_info("SmartPA", "CIRRUS_CS35L41-B2");
        register_hardware_info("Speaker", "JBL_QS271223AW00");

	hwinfo_kobj = kobject_create_and_add("hwinfo", NULL);
	if (!hwinfo_kobj)
		goto gpio_fail7;

	rc = sysfs_create_group(hwinfo_kobj, &hwinfo_attr_group);
	if (rc)
		kobject_put(hwinfo_kobj);

	sku = (gpio_get_value(data->gpio6) << 1) | gpio_get_value(data->gpio7);
	rc = iio_read_channel_processed(data->gpio1, &adc_hw);
	if (rc < 0) {
		pr_err("Couldn't read hwid_gpio1 chan, rc = %d\n", rc);
		goto gpio_fail7;
	}

	iio_channel_release(data->gpio1);

	for(i = 0; i < ARRAY_SIZE(menu); i++) {
		pr_err("min:%d, max=%d, adc=%d, ARRAY_SIZE:%d", menu[i].min, menu[i].max, adc_hw, ARRAY_SIZE(menu));
		if ( menu[i].min <= adc_hw && adc_hw <= menu[i].max) {
			hw=menu[i].hw;
			break;
		}
	}

	pr_err("sku=%d, hw=%d, adc_hw=%d\n", sku, hw, adc_hw);

	return 0;
gpio_fail7:
	gpio_free(data->gpio7);
gpio_fail6:
	gpio_free(data->gpio6);
exit:
	return rc;
}

static int hardware_info_remove(struct platform_device *pdev)
{
	kobject_put(hwinfo_kobj);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hardware_info_match[] = {
	{.compatible = "lenovo,hardware_info",},
	{},
};

MODULE_DEVICE_TABLE(of, hardware_info_match);
#endif

static struct platform_driver hardware_info_driver = {
	.probe = hardware_info_probe,
	.remove = hardware_info_remove,
	.driver = {
		   .name = "hardware_info",
#ifdef CONFIG_OF
		   .of_match_table = hardware_info_match,
#endif
		   },
};

static int __init hardware_info_init(void)
{
	int ret;

	printk("hardware_info_init\n");

	ret = platform_driver_register(&hardware_info_driver);
	if (ret) {
		printk("Unable to register driver (%d)\n", ret);
		return ret;
	}
		return 0;
}

static void __exit hardware_info_exit(void)
{
}

module_init(hardware_info_init);
module_exit(hardware_info_exit);

