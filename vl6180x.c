#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "vl6180x_registers.h"

//sys/devices/iio:device0/
//https://elixir.bootlin.com/linux/v4.7/source/drivers/iio/dummy/iio_simple_dummy.c
//https://elixir.bootlin.com/linux/v4.7/source/drivers/iio/proximity/pulsedlight-lidar-lite-v2.c
//https://github.com/pololu/vl6180x-arduino/blob/master/VL6180X.cpp


#define VL6180X_DRV_NAME "vl6180x"

//--------------------------------------------------------------------------------------

struct vl6180x_data {
	struct iio_dev *indio_dev;
	struct i2c_client *client;
};

static const struct iio_chan_spec vl6180x_channels[] = {
   //entry for distance
	{
		.type = IIO_DISTANCE,
      .address = VL6180X_RESULT__RANGE_VAL,
      //below not managed yet
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_type = {
			.sign = 'u',
			.realbits = 8,
			.storagebits = 8,
		},
      .datasheet_name = "PROX",
	},
   //entry for built-in ambient light sensor (ALS)
	{
		.type = IIO_LIGHT,
      .address = VL6180X_RESULT__ALS_VAL,
      //below not managed yet
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
		},
      .datasheet_name = "ALS",
	},
};

static int vl6180x_write_u8(struct vl6180x_data *data, u16 reg, u8 value)
{
	struct i2c_msg msg[1];
   u8 wbuf[3];
	int ret;

   msg[0].addr = data->client->addr;
   msg[0].flags = 0;
   msg[0].buf = wbuf;
   msg[0].buf[0] = reg >> 8;
   msg[0].buf[1] = reg;
   msg[0].buf[2] = value;
   msg[0].len = 3;

   ret = i2c_transfer(data->client->adapter, msg, 1);
   if(ret == 1)
      return 0;

   return -ETIMEDOUT;
}

//static int vl6180x_write_u16(struct vl6180x_data *data, u16 reg, u16 value)
//{
//	struct i2c_msg msg[1];
//   u8 wbuf[3];
//	int ret;
//
//   msg[0].addr = data->client->addr;
//   msg[0].flags = 0;
//   msg[0].buf = wbuf;
//   msg[0].buf[0] = (u8)reg >> 8;
//   msg[0].buf[1] = (u8)reg;
//   msg[0].buf[2] = (u8)(value >> 8);
//   msg[0].buf[3] = (u8)value;
//   msg[0].len = 3;
//
//   ret = i2c_transfer(data->client->adapter, msg, 1);
//   if(ret == 1)
//      return 0;
//
//   return -ETIMEDOUT;
//}

static int vl6180x_read_u8(struct vl6180x_data *data, u16 reg, u8* rd_data_u8)
{
	struct i2c_msg msg[2];
   u8 wbuf[2];
	int ret;

   msg[0].addr = data->client->addr;
   msg[0].flags = 0;
   msg[0].buf = wbuf;
   msg[0].buf[0] = reg >> 8;
   msg[0].buf[1] = reg;
   msg[0].len = 2;

   msg[1].addr = data->client->addr;
   msg[1].flags = I2C_M_RD;
   msg[1].buf = rd_data_u8;
   msg[1].len = 1;

   ret = i2c_transfer(data->client->adapter, msg, 2);
   if(ret == 2)
      return 0;

   return -ETIMEDOUT;
}

static int vl6180x_read_u16(struct vl6180x_data *data, u16 reg, u16 *rd_data_u16)
{
	struct i2c_msg msg[2];
   u8 wbuf[2];
   u8 rbuf[2];
	int ret;

   msg[0].addr = data->client->addr;
   msg[0].flags = 0;
   msg[0].buf = wbuf;
   msg[0].buf[0] = reg >> 8;
   msg[0].buf[1] = reg;
   msg[0].len = 2;

   msg[1].addr = data->client->addr;
   msg[1].flags = I2C_M_RD;
   msg[1].buf = rbuf;
   msg[1].len = 2;

   ret = i2c_transfer(data->client->adapter, msg, 2);
   if(ret == 2) {
      *rd_data_u16 = (rbuf[1] << 8) | rbuf[0];
      return 0;
   }

   return -ETIMEDOUT;
}

static int vl6180x_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct vl6180x_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;
   u8 rd_data_u8;
   u16 rd_data_u16;
   int timeout=100;

	mutex_lock(&indio_dev->mlock);

   if(chan->type == IIO_DISTANCE) {

      printk("Starting ranging.\n");

      //start ranging system (one-shot)
      vl6180x_write_u8(data, VL6180X_SYSRANGE__START, 0x01);

      printk("Waiting for interrupt status.\n");
      rd_data_u8 = 0;
      while(rd_data_u8) {
         msleep(10);
         vl6180x_read_u8(data, VL6180X_RESULT__INTERRUPT_STATUS_GPIO, &rd_data_u8);
         if(timeout == 0) {
            printk("Interrupt status timeout.\n");
            ret = -EINVAL;
            goto timeoutTrue;
         }
         timeout--;
      }

      ret = vl6180x_read_u8(data, chan->address, &rd_data_u8);
      if (!ret) {
         printk("vl6180x_read_raw: 0x%02X@%lu\n", rd_data_u8, chan->address);
         *val = rd_data_u8;
         ret = IIO_VAL_INT;
      }
   } else if(chan->type == IIO_LIGHT) {
      printk("Starting ALS.\n");

      //start ALS system (one-shot)
      vl6180x_write_u8(data, VL6180X_SYSALS__START, 0x01);

      msleep(300); //250 is fine, 200 too low

      // Interrupt not functioning for ALS - reason unknown
      // printk("Waiting for interrupt status.\n");
      // rd_data_u8 = 0;
      // while(rd_data_u8) {
      //    msleep(10);
      //    vl6180x_read_u8(data, VL6180X_RESULT__INTERRUPT_STATUS_GPIO, &rd_data_u8);
      //    if(timeout == 0) {
      //       printk("Interrupt status timeout.\n");
      //       ret = -EINVAL;
      //       goto timeoutTrue;
      //    }
      //    timeout--;
      // }

      ret = vl6180x_read_u16(data, chan->address, &rd_data_u16);
      if (!ret) {
         printk("vl6180x_read_raw: 0x%04X@%lu\n", rd_data_u16, chan->address);
         *val = rd_data_u16;
         ret = IIO_VAL_INT;
      }
   }

   //clear interrupts
   vl6180x_write_u8(data, VL6180X_SYSTEM__INTERRUPT_CLEAR, 0x07);
timeoutTrue:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info vl6180x_info = {
	.driver_module = THIS_MODULE,
	.read_raw = vl6180x_read_raw,
};

static int vl6180x_init_per_AN4545(struct vl6180x_data *data) {
   u8 rd_data_u8;

   //check to see if device has already been initialized
   vl6180x_read_u8(data, VL6180X_SYSTEM__FRESH_OUT_OF_RESET, &rd_data_u8);
   if(rd_data_u8 != 1) {
      printk("Skipping initialization - vl6180x has already been configured\n");
      return 0;
   }

   // Mandatory: Private registers
   vl6180x_write_u8(data, 0x0207, 0x01);
   vl6180x_write_u8(data, 0x0208, 0x01);
   vl6180x_write_u8(data, 0x0096, 0x00);
   vl6180x_write_u8(data, 0x0097, 0xFD);
   vl6180x_write_u8(data, 0x00E3, 0x00);
   vl6180x_write_u8(data, 0x00E4, 0x04);
   vl6180x_write_u8(data, 0x00E5, 0x02);
   vl6180x_write_u8(data, 0x00E6, 0x01);
   vl6180x_write_u8(data, 0x00E7, 0x03);
   vl6180x_write_u8(data, 0x00F5, 0x02);
   vl6180x_write_u8(data, 0x00D9, 0x05);
   vl6180x_write_u8(data, 0x00DB, 0xCE);
   vl6180x_write_u8(data, 0x00DC, 0x03);
   vl6180x_write_u8(data, 0x00DD, 0xF8);
   vl6180x_write_u8(data, 0x009F, 0x00);
   vl6180x_write_u8(data, 0x00A3, 0x3C);
   vl6180x_write_u8(data, 0x00B7, 0x00);
   vl6180x_write_u8(data, 0x00BB, 0x3C);
   vl6180x_write_u8(data, 0x00B2, 0x09);
   vl6180x_write_u8(data, 0x00CA, 0x09);
   vl6180x_write_u8(data, 0x0198, 0x01);
   vl6180x_write_u8(data, 0x01B0, 0x17);
   vl6180x_write_u8(data, 0x01AD, 0x00);
   vl6180x_write_u8(data, 0x00FF, 0x05);
   vl6180x_write_u8(data, 0x0100, 0x05);
   vl6180x_write_u8(data, 0x0199, 0x05);
   vl6180x_write_u8(data, 0x01A6, 0x1B);
   vl6180x_write_u8(data, 0x01AC, 0x3E);
   vl6180x_write_u8(data, 0x01A7, 0x1F);
   vl6180x_write_u8(data, 0x0030, 0x00);

   return 0;
}

static int vl6180x_public_registers(struct vl6180x_data *data) {
   // Recommended : Public registers - See data sheet for more detail

   // Enables polling for New Sample ready when measurement completes
   vl6180x_write_u8(data, VL6180X_SYSTEM__MODE_GPIO1, 0x10);
   // Set the averaging sample period (compromise between lower noise and increased execution time)
   vl6180x_write_u8(data, VL6180X_READOUT__AVERAGING_SAMPLE_PERIOD, 0x30);
   // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
   vl6180x_write_u8(data, VL6180X_SYSALS__ANALOGUE_GAIN, 0x46);
   // sets the # of range measurements after which auto calibration of system is performed
   vl6180x_write_u8(data, VL6180X_SYSRANGE__VHV_REPEAT_RATE, 0xFF);
   // Set ALS integration time to 100ms
   vl6180x_write_u8(data, VL6180X_SYSALS__INTEGRATION_PERIOD, 0x63);
   // perform a single temperature calibration of the ranging sensor
   vl6180x_write_u8(data, VL6180X_SYSRANGE__VHV_RECALIBRATE, 0x01);

   // Configure GPIO Interrupt behavior
   vl6180x_write_u8(data, VL6180X_SYSTEM__INTERRUPT_CONFIG_GPIO, 0x24);

   //set register indicating configuration complete (don't need to re-init on driver load)
   vl6180x_write_u8(data, VL6180X_SYSTEM__FRESH_OUT_OF_RESET, 0x00);

   return 0;
}

static int vl6180x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
   struct vl6180x_data *data;
	struct iio_dev *indio_dev;
	int ret;
   u8 rd_data_u8;

   //Allocate an IIO device
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	data = iio_priv(indio_dev);

   //verify bus supports standard I2C (needed for 16-bit Addresses)
   if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
      printk("vl6180x - i2c_check_functionality(I2C_FUNC_I2C) failed.\n");
      ret = -EOPNOTSUPP;
      goto error_free_device;
   }

   //init indio_dev
	indio_dev->info = &vl6180x_info;
	indio_dev->name = VL6180X_DRV_NAME;
	indio_dev->channels = vl6180x_channels;
	indio_dev->num_channels = ARRAY_SIZE(vl6180x_channels);
   //set device operating mode (INDIO_DIRECT_MODE, INDIO_BUFFER_TRIGGERED, INDIO_BUFFER_SOFTWARE, INDIO_BUFFER_HARDWARE, INDIO_EVENT_TRIGGERED, INDIO_HARDWARE_TRIGGERED)
	indio_dev->modes = INDIO_DIRECT_MODE;

   //load indio_dev into *client
   i2c_set_clientdata(client, indio_dev);

   //assign pointers for ease of access
   data->client = client;
   data->indio_dev = indio_dev;

   //Debug
   {
      mutex_lock(&indio_dev->mlock);
      ret = vl6180x_read_u8(data, VL6180X_IDENTIFICATION__MODEL_ID, &rd_data_u8);
      if(!ret) {
         if(rd_data_u8 == 0xB4)
            printk("Valid VL6180X ID: 0x%02X (expected 0xB4)\n", rd_data_u8);
         else
            printk("Invalid VL6180X ID: 0x%02X (expected 0xB4)\n", rd_data_u8);
      }
      mutex_unlock(&indio_dev->mlock);
   }

   //init per ST application note AN4545, section 9 - "Mandatory : private registers"
   vl6180x_init_per_AN4545(data);

   //init public registers
   vl6180x_public_registers(data);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_unreg_buffer;

	return 0;

error_unreg_buffer:
error_free_device:
	devm_iio_device_free(&client->dev, indio_dev);
	return ret;
}

static int vl6180x_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	devm_iio_device_free(&client->dev, indio_dev);

   return 0;
}

//--------------------------------------------------------------------------------------

static const struct of_device_id vl6180x_dt_ids[] = {
	{ .compatible = "stm,vl6180x" },
	{ },
};
MODULE_DEVICE_TABLE(of, vl6180x_dt_ids);

static const struct i2c_device_id vl6180x_id[] = {
	{ "vl6180x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vl6180x_id);

static struct i2c_driver vl6180x_driver = {
   .driver = {
      .name  = "VL6180X",
      .of_match_table = of_match_ptr(vl6180x_dt_ids),
   },
   .probe    = vl6180x_probe,
   .remove   = vl6180x_remove,
   .id_table = vl6180x_id,
};

//Using I2C init macro to simplify code
module_i2c_driver(vl6180x_driver);

//--------------------------------------------------------------------------------------

MODULE_AUTHOR("Michael Wilson <mgwilson271@gmail.com>");
MODULE_DESCRIPTION("STMicroelectronics VL6180X IIO Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.2");


