/*
 * Driver for ELAN eKTF2127 i2c touchscreen controller
 *
 * For this driver the layout of the Chipone icn8318 i2c
 * touchscreencontroller is used.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Author:
 * Michel Verlaan <michel.verl@gmail.com>
 * Siebren Vroegindeweij <siebren.vroegindeweij@hotmail.com>
 *
 * Original chipone_icn8318 driver:
 * Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>

#define EKTF2127_REG_POWER		4
#define EKTF2127_REG_TOUCHDATA		16

#define EKTF2127_POWER_ACTIVE		0
#define EKTF2127_POWER_MONITOR		1
#define EKTF2127_POWER_HIBERNATE	2

#define EKTF2127_MAX_TOUCHES		5

/* The difference between 2 and 3 is unclear */
#define EKTF2127_EVENT_NO_DATA	1 /* No finger seen yet since wakeup */
#define EKTF2127_EVENT_UPDATE1	2 /* New or updated coordinates */
#define EKTF2127_EVENT_UPDATE2	3 /* New or updated coordinates */
#define EKTF2127_EVENT_END	4 /* Finger lifted */

struct ektf2127_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *power_gpios;
	struct touchscreen_properties prop;
};

static void retrieve_coordinates(struct input_mt_pos *touches,
				int touch_count, char *buf)
{
	int index = 0;
	int i = 0;

	for (i = 0; i < touch_count; i++) {
		index = 2 + i * 3;

		touches[i].x = (buf[index] & 0x0f);
		touches[i].x <<= 8;
		touches[i].x |= buf[index + 2];

		touches[i].y = (buf[index] & 0xf0);
		touches[i].y <<= 4;
		touches[i].y |= buf[index + 1];
	}
}

static irqreturn_t ektf2127_irq(int irq, void *dev_id)
{
	struct ektf2127_data *data = dev_id;
	struct device *dev = &data->client->dev;
	struct input_mt_pos touches[EKTF2127_MAX_TOUCHES];
	int touch_count;
	int slots[EKTF2127_MAX_TOUCHES];
	char buff[25];
	int i, ret;

	ret = i2c_master_recv(data->client, buff, 25);
	if (ret != 25) {
		dev_err(dev, "Error reading touch data");
		return IRQ_HANDLED;
	}

	touch_count = buff[1] & 0x07;

	if (touch_count > EKTF2127_MAX_TOUCHES) {
		dev_err(dev, "Too many touches %d > %d\n",
			touch_count, EKTF2127_MAX_TOUCHES);
		touch_count = EKTF2127_MAX_TOUCHES;
	}

	retrieve_coordinates(touches, touch_count, buff);
	input_mt_assign_slots(data->input, slots, touches,
		touch_count, 0);

	for (i = 0; i < touch_count; i++) {

		input_mt_slot(data->input, slots[i]);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(data->input, &data->prop, touches[i].x,
			touches[i].y, true);
	}

	input_mt_sync_frame(data->input);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static int ektf2127_start(struct input_dev *dev)
{
	struct ektf2127_data *data = input_get_drvdata(dev);

	enable_irq(data->client->irq);
	gpiod_set_value_cansleep(data->power_gpios, 1);

	return 0;
}

static void ektf2127_stop(struct input_dev *dev)
{
	struct ektf2127_data *data = input_get_drvdata(dev);

	disable_irq(data->client->irq);
	gpiod_set_value_cansleep(data->power_gpios, 0);
}

static int ektf2127_suspend(struct device *dev)
{
	struct ektf2127_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (data->input->users)
		ektf2127_stop(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int ektf2127_resume(struct device *dev)
{
	struct ektf2127_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (data->input->users)
		ektf2127_start(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static SIMPLE_DEV_PM_OPS(ektf2127_pm_ops, ektf2127_suspend,
			ektf2127_resume);

static int ektf2127_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ektf2127_data *data;
	struct input_dev *input;
	char buff[25];
	int error, ret = 0;

	if (!client->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* This requests the gpio *and* turns on the touchscreen controller */
	data->power_gpios = devm_gpiod_get(dev, "power", GPIOD_OUT_HIGH);
	if (IS_ERR(data->power_gpios)) {
		error = PTR_ERR(data->power_gpios);
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Error getting power gpio: %d\n", error);
		return error;
	}

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->open = ektf2127_start;
	input->close = ektf2127_stop;
	input->dev.parent = dev;

	data->client = client;

	/* read hello */
	msleep(20);
	i2c_master_recv(data->client, buff, 4);

	/* Read resolution from chip */

	/* Request height */
	buff[0] = 0x53; /* REQUEST */
	buff[1] = 0x63; /* HEIGHT */
	buff[2] = 0x00;
	buff[3] = 0x00;
	ret = i2c_master_send(data->client, buff, 4);
	if (ret != 4) {
		dev_err(dev, "Error requesting height");
		return ret < 0 ? ret : -EIO;
	}

	msleep(20);

	/* Read response */
	ret = i2c_master_recv(data->client, buff, 4);
	if (ret != 4) {
		dev_err(dev, "Error receiving height");
		return ret < 0 ? ret : -EIO;
	}


	if ((buff[0] == 0x52) && (buff[1] == 0x63)) {
		data->max_y = ((buff[3] & 0xf0) << 4) | buff[2];
	} else {
		dev_err(dev, "Error receiving height data from"
			" wrong register");
		return ret < 0 ? ret : -EIO;
	}

	/* Request width */
	buff[0] = 0x53; /* REQUEST */
	buff[1] = 0x60; /* WIDTH */
	buff[2] = 0x00;
	buff[3] = 0x00;
	ret = i2c_master_send(data->client, buff, 4);
	if (ret != 4) {
		dev_err(dev, "Error requesting width");
		return ret < 0 ? ret : -EIO;
	}

	msleep(20);

	/* Read response */
	ret = i2c_master_recv(data->client, buff, 4);
	if (ret != 4) {
		dev_err(dev, "Error receiving width");
		return ret < 0 ? ret : -EIO;
	}


	if ((buff[0] == 0x52) && (buff[1] == 0x60)) {
		data->max_x = (((buff[3] & 0xf0) << 4) | buff[2]);
	} else {
		dev_err(dev, "Error receiving width data from"
			" wrong register");
		return ret < 0 ? ret : -EIO;
	}


	/* Touchscreen resolution can be overruled by devicetree*/
	of_property_read_u32(np, "touchscreen-size-x", &data->max_x);
	of_property_read_u32(np, "touchscreen-size-y", &data->max_y);
				
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);
	touchscreen_parse_properties(input, true, &data->prop);
	if (!input_abs_get_max(input, ABS_MT_POSITION_X) ||
	    !input_abs_get_max(input, ABS_MT_POSITION_Y)) {
	    	dev_err(dev, "Error touchscreen-size-x and/or -y missing\n");
	    	return -EINVAL;	    
	    }

	error = input_mt_init_slots(input, EKTF2127_MAX_TOUCHES,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED
				    | INPUT_MT_TRACK);
	if (error)
		return error;

	data->input = input;
	input_set_drvdata(input, data);

	error = devm_request_threaded_irq(dev, client->irq, NULL, ektf2127_irq,
					  IRQF_ONESHOT, client->name, data);
	if (error) {
		dev_err(dev, "Error requesting irq: %d\n", error);
		return error;
	}

	/* Stop device till opened */
	ektf2127_stop(data->input);

	error = input_register_device(input);
	if (error)
		return error;

	i2c_set_clientdata(client, data);

	return 0;
}

static const struct of_device_id ektf2127_of_match[] = {
	{ .compatible = "elan,ektf2127" },
	{ }
};
MODULE_DEVICE_TABLE(of, ektf2127_of_match);

static const struct i2c_device_id ektf2127_i2c_id[] = {
	{"ektf2127", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ektf2127_i2c_id);

static struct i2c_driver ektf2127_driver = {
	.driver = {
		.name	= "elan_ektf2127",
		.pm	= &ektf2127_pm_ops,
		.of_match_table = of_match_ptr(ektf2127_of_match),
	},
	.probe = ektf2127_probe,
	.id_table = ektf2127_i2c_id,
};

module_i2c_driver(ektf2127_driver);

MODULE_DESCRIPTION("ELAN eKTF2127 I2C Touchscreen Driver");
MODULE_AUTHOR("Michel Verlaan");
MODULE_AUTHOR("Siebren Vroegindeweij");
MODULE_LICENSE("GPL");
