/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, controlling GPIOs such as sensor reset
 * line, sensor IRQ line.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node.
 *
 * This driver will NOT send any commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sysfs.h>
#include <linux/device.h>

#define FPC_GPIO_NO_DEFAULT         -1
#define FPC_TTW_HOLD_TIME           1000

#define RESET_LOW_SLEEP_MIN_US      5000
#define RESET_LOW_SLEEP_MAX_US      (RESET_LOW_SLEEP_MIN_US + 100)
#define RESET_HIGH_SLEEP1_MIN_US    100
#define RESET_HIGH_SLEEP1_MAX_US    (RESET_HIGH_SLEEP1_MIN_US + 100)
#define RESET_HIGH_SLEEP2_MIN_US    5000
#define RESET_HIGH_SLEEP2_MAX_US    (RESET_HIGH_SLEEP2_MIN_US + 100)
#define PWR_ON_SLEEP_MIN_US         100
#define PWR_ON_SLEEP_MAX_US         (PWR_ON_SLEEP_MIN_US + 900)

#define NUM_PARAMS_REG_ENABLE_SET    2

#define RELEASE_WAKELOCK_W_V "release_wakelock_with_verification"
#define RELEASE_WAKELOCK     "release_wakelock"
#define START_IRQS_RECEIVED_CNT "start_irqs_received_counter"

static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_irq_active",
};

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
	int gpio;
};

static const struct vreg_config vreg_conf[] = {
	{ "vdd_ana", 1800000UL, 1800000UL, 6000, FPC_GPIO_NO_DEFAULT },
};

struct fpc1020_data {
	struct device         *dev;
	struct pinctrl        *fingerprint_pinctrl;
	struct pinctrl_state  *pinctrl_state[ARRAY_SIZE(pctl_names)];
	struct regulator      *vreg[ARRAY_SIZE(vreg_conf)];
	struct wakeup_source  *ttw_ws;
	int                    irq_gpio;
	int                    nbr_irqs_received;
	int                    nbr_irqs_received_counter_start;
	struct mutex           lock;
	bool                   prepared;
	bool                   irq_requested;
	bool                   gpios_requested;
	atomic_t               wakeup_enabled;
	int                    irqf;
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle);
static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
				       const char *label, int *gpio);
static int hw_reset(struct fpc1020_data *fpc1020);

static int request_vreg_gpio(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;

	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->gpios_requested) {
		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq", &fpc1020->irq_gpio);
		if (rc)
			goto out;

		rc = devm_request_threaded_irq(fpc1020->dev,
			gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, fpc1020->irqf,
			dev_name(fpc1020->dev), fpc1020);
		if (rc) {
			dev_err(fpc1020->dev, "could not request irq %d\n", gpio_to_irq(fpc1020->irq_gpio));
			goto out;
		}

		enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
		fpc1020->irq_requested = true;
		fpc1020->gpios_requested = true;
	} else if (!enable && fpc1020->gpios_requested) {
		if (fpc1020->irq_requested) {
			devm_free_irq(fpc1020->dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
			fpc1020->irq_requested = false;
		}

		if (gpio_is_valid(fpc1020->irq_gpio)) {
			devm_gpio_free(fpc1020->dev, fpc1020->irq_gpio);
			fpc1020->irq_gpio = FPC_GPIO_NO_DEFAULT;
		}
		fpc1020->gpios_requested = false;
	}
out:
	mutex_unlock(&fpc1020->lock);
	return rc;
}

static ssize_t request_vreg_set(struct device *device,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int status = 0;
	int voltage, enabled;
	struct regulator *vreg;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	struct device *dev = fpc1020->dev;

	vreg = devm_regulator_get(dev, "vdd_ana");
	if (IS_ERR_OR_NULL(vreg)) {
		dev_err(dev, "Unable to get %s\n", "vdd_ana");
		return PTR_ERR(vreg);
	}

	if (!strncmp(buf, "enable", strlen("enable"))) {
		request_vreg_gpio(fpc1020, true);
		enabled = regulator_is_enabled(vreg);
		if (enabled <= 0) {
			status = regulator_enable(vreg);
			usleep_range(100 * 1000, 200 * 1000);
		}
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		request_vreg_gpio(fpc1020, false);
		enabled = regulator_is_enabled(vreg);
		if (enabled > 0)
			status = regulator_disable(vreg);
	} else if (!strncmp(buf, "force_disable", strlen("force_disable"))) {
		request_vreg_gpio(fpc1020, false);
		status = regulator_force_disable(vreg);
	} else if (!strncmp(buf, "power_on_reset", strlen("power_on_reset"))) {
		enabled = regulator_is_enabled(vreg);
		if (enabled <= 0) {
			status = regulator_enable(vreg);
			usleep_range(100 * 1000, 200 * 1000);
		}
		status = hw_reset(fpc1020);
	} else {
		status = -EINVAL;
	}

	voltage = regulator_get_voltage(vreg);
	enabled = regulator_is_enabled(vreg);
	dev_info(dev, "%s: cmd=%s, status=%d, enabled=%d, voltage=%d\n",
		 __func__, buf, status, enabled, voltage);

	return status ? status : count;
}
static DEVICE_ATTR(request_vreg, 0200, NULL, request_vreg_set);

/**
 * sysfs node for controlling clocks.
 *
 * This is disabled in platform variant of this driver but kept for
 * backwards compatibility.
 *
 */
static ssize_t clk_enable_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}
static DEVICE_ATTR(clk_enable, 0200, NULL, clk_enable_set);


/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	int i, rc = -EINVAL;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
		if (!strncmp(pctl_names[i], name, strlen(pctl_names[i]))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
						  fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			return rc;
		}
	}

	dev_err(dev, "%s: '%s' not found\n", __func__, name);
	return rc;
}
static ssize_t pinctl_set(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&fpc1020->lock);
	rc = select_pin_ctl(fpc1020, buf);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);


static ssize_t regulator_enable_set(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc;
	bool enable;

	if (NUM_PARAMS_REG_ENABLE_SET != sscanf(buf, "%15[^,],%c", name, &op))
		return -EINVAL;
	enable = (op == 'e') ? true : (op == 'd') ? false : false;

	mutex_lock(&fpc1020->lock);
	{
		size_t i;
		struct regulator *vreg = NULL;
		for (i = 0; i < ARRAY_SIZE(vreg_conf); i++) {
			if (!strncmp(vreg_conf[i].name, name, strlen(vreg_conf[i].name))) {
				vreg = fpc1020->vreg[i];
				if (enable) {
					if (!vreg) {
						vreg = regulator_get(dev, name);
						if (IS_ERR(vreg)) {
							rc = PTR_ERR(vreg);
							goto out;
						}
						rc = regulator_set_load(vreg, vreg_conf[i].ua_load);
						if (rc < 0)
							dev_err(dev, "Unable to set current on %s, %d\n", name, rc);
						rc = regulator_enable(vreg);
						if (rc) {
							dev_err(dev, "error enabling %s: %d\n", name, rc);
							regulator_put(vreg);
							vreg = NULL;
							goto out;
						}
						fpc1020->vreg[i] = vreg;
					}
				} else {
					if (vreg) {
						if (regulator_is_enabled(vreg))
							regulator_disable(vreg);
						regulator_put(vreg);
						fpc1020->vreg[i] = NULL;
					}
				}
				rc = 0;
				goto out;
			}
		}
		dev_err(dev, "Regulator %s not found\n", name);
		rc = -EINVAL;
	}
out:
	mutex_unlock(&fpc1020->lock);
	return rc ? rc : count;
}
static DEVICE_ATTR(regulator_enable, 0200, NULL, regulator_enable_set);

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int rc;

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		return rc;
	usleep_range(RESET_HIGH_SLEEP1_MIN_US, RESET_HIGH_SLEEP1_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		return rc;
	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		return rc;
	usleep_range(RESET_HIGH_SLEEP2_MIN_US, RESET_HIGH_SLEEP2_MAX_US);

	return rc;
}
static ssize_t hw_reset_set(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset"))) {
		mutex_lock(&fpc1020->lock);
		rc = hw_reset(fpc1020);
		mutex_unlock(&fpc1020->lock);
	} else {
		rc = -EINVAL;
	}

	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, 0200, NULL, hw_reset_set);

/**
 * Will setup GPIOs, and regulators to correctly initialize the touch sensor to
 * be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, and reset line, all to set the sensor in a
 * correct power on or off state "electrical" wise.
 *
 * @see  device_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;

	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		fpc1020->prepared = true;
		rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
		if (rc)
			goto out;
		rc = vreg_conf[0].ua_load;
		fpc1020->vreg[0] = devm_regulator_get(fpc1020->dev, "vdd_ana");
		if (IS_ERR(fpc1020->vreg[0])) {
			rc = PTR_ERR(fpc1020->vreg[0]);
			dev_err(fpc1020->dev, "Unable to get vdd_ana\n");
			goto out;
		}
		rc = regulator_enable(fpc1020->vreg[0]);
		if (rc) {
			dev_err(fpc1020->dev, "Failed to enable vdd_ana\n");
			goto out;
		}
		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);
		rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
		if (rc)
			goto out;
	} else if (!enable && fpc1020->prepared) {
		rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);
		if (fpc1020->vreg[0])
			regulator_disable(fpc1020->vreg[0]);
		fpc1020->prepared = false;
	}
out:
	mutex_unlock(&fpc1020->lock);
	return rc;
}

/**
 * sysfs node to enable/disable (power up/power down) the touch sensor
 *
 * @see device_prepare
 */
static ssize_t device_prepare_set(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = device_prepare(fpc1020, false);
	else
		rc = -EINVAL;

	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, 0200, NULL, device_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 *
*/
static ssize_t wakeup_enable_set(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, "enable", strlen("enable")))
		atomic_set(&fpc1020->wakeup_enabled, 1);
	else if (!strncmp(buf, "disable", strlen("disable")))
		atomic_set(&fpc1020->wakeup_enabled, 0);
	else
		ret = -EINVAL;
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR(wakeup_enable, 0200, NULL, wakeup_enable_set);

/**
 * sysfs node for controlling the wakelock.
 */
static ssize_t handle_wakelock_cmd(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, RELEASE_WAKELOCK_W_V, min(count, strlen(RELEASE_WAKELOCK_W_V)))) {
		if (fpc1020->nbr_irqs_received_counter_start ==
		    fpc1020->nbr_irqs_received)
			__pm_relax(fpc1020->ttw_ws);
		else
			dev_dbg(dev, "Wakelock not released: counter mismatch (%d != %d)\n",
				fpc1020->nbr_irqs_received_counter_start,
				fpc1020->nbr_irqs_received);
	} else if (!strncmp(buf, RELEASE_WAKELOCK, min(count, strlen(RELEASE_WAKELOCK)))) {
		__pm_relax(fpc1020->ttw_ws);
	} else if (!strncmp(buf, START_IRQS_RECEIVED_CNT, min(count, strlen(START_IRQS_RECEIVED_CNT)))) {
		fpc1020->nbr_irqs_received_counter_start = fpc1020->nbr_irqs_received;
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&fpc1020->lock);
	return ret;
}
static DEVICE_ATTR(handle_wakelock, S_IWUSR, NULL, handle_wakelock_cmd);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 *
 */
static ssize_t irq_get(struct device *dev,
		       struct device_attribute *attr,
		       char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int irq_val = gpio_get_value(fpc1020->irq_gpio);

	return scnprintf(buf, PAGE_SIZE, "%i\n", irq_val);
}

static ssize_t irq_ack(struct device *dev,
		       struct device_attribute *attr,
		       const char *buf, size_t count)
{
	dev_dbg(dev, "%s\n", __func__);
	return count;
}
static DEVICE_ATTR(irq, 0600, irq_get, irq_ack);

static ssize_t fingerdown_wait_set(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return count;
}
static DEVICE_ATTR(fingerdown_wait, 0200, NULL, fingerdown_wait_set);
static DEVICE_ATTR(power_cfg, 0200, NULL, fingerdown_wait_set);

/* Create attribute list */
static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_handle_wakelock.attr,
	&dev_attr_request_vreg.attr,
	&dev_attr_fingerdown_wait.attr,
	&dev_attr_power_cfg.attr,
	NULL,
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	if (atomic_read(&fpc1020->wakeup_enabled)) {
		fpc1020->nbr_irqs_received++;
		__pm_wakeup_event(fpc1020->ttw_ws, FPC_TTW_HOLD_TIME);
	}
	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);
	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
				       const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s: gpio %d\n", label, *gpio);
	return 0;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0, i;
	struct device_node *np = dev->of_node;
	struct fpc1020_data *fpc1020;

	fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev, "failed to allocate memory for fpc1020_data\n");
		return -ENOMEM;
	}
	fpc1020->dev = dev;
	platform_set_drvdata(pdev, fpc1020);

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		dev_err(dev, "failed to get pinctrl\n");
		return PTR_ERR(fpc1020->fingerprint_pinctrl);
	}
	for (i = 0; i < ARRAY_SIZE(pctl_names); i++) {
		fpc1020->pinctrl_state[i] = pinctrl_lookup_state(fpc1020->fingerprint_pinctrl,
								 pctl_names[i]);
		if (IS_ERR(fpc1020->pinctrl_state[i])) {
			dev_err(dev, "cannot find '%s'\n", pctl_names[i]);
			return -EINVAL;
		}
		dev_info(dev, "found pin control '%s'\n", pctl_names[i]);
	}
	mutex_init(&fpc1020->lock);
	atomic_set(&fpc1020->wakeup_enabled, 0);
	fpc1020->irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_SUSPEND;
	fpc1020->gpios_requested = false;
	device_init_wakeup(dev, 1);
	fpc1020->ttw_ws = wakeup_source_register(dev, "fpc_ttw_ws");
	if (!fpc1020->ttw_ws)
		return -ENOMEM;
	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs group\n");
		return rc;
	}
	if (of_property_read_bool(np, "fpc,enable-on-boot")) {
		dev_info(dev, "Enabling hardware on boot\n");
		(void)device_prepare(fpc1020, true);
	}
	rc = hw_reset(fpc1020);
	if (rc) {
		dev_err(dev, "hardware reset failed\n");
		return rc;
	}
	dev_info(dev, "FPC1020 probe successful\n");
	return 0;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020_data *fpc1020 = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);
	wakeup_source_unregister(fpc1020->ttw_ws);
	(void)device_prepare(fpc1020, false);
	dev_info(&pdev->dev, "FPC1020 removed\n");
	return 0;
}

static const struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{ },
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name           = "fpc1020",
		.of_match_table = fpc1020_of_match,
	},
	.probe  = fpc1020_probe,
	.remove = fpc1020_remove,
};

module_platform_driver(fpc1020_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
