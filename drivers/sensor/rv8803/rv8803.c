#define DT_DRV_COMPAT mc_rv8803

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/logging/log.h>

#include "rv8803.h"

LOG_MODULE_REGISTER(RV8803, CONFIG_SENSOR_LOG_LEVEL);

struct rv8803_data {
    uint8_t channel_states;
};

struct rv8803_dev_config {
    struct i2c_dt_spec i2c;
};

static int rv8803_sample_fetch(const struct device *dev, 
                                enum sensor_channel chan) {
	
    return 0;
}

bool rv8803_convert_time_to_reg_values(int32_t hours, int32_t minutes, uint8_t* hours_reg, uint8_t* minutes_reg) {
	if(hours < 0 || minutes < 0) {
		printk("invalid hours or minutes value \r\n");
		return false;
	}

	#define NUM_MIN_INCREMENTS 7
	#define NUM_HOUR_INCREMENTS 6
	uint8_t minutes_to_bitshift_map[NUM_MIN_INCREMENTS] = {1, 2, 4, 8, 10, 20, 40};
	uint8_t hours_to_bitshift_map[NUM_HOUR_INCREMENTS] = {1, 2, 4, 8, 10, 20};

	uint8_t minutes_byte_value = 0;
	for(int i = NUM_MIN_INCREMENTS - 1; i >= 0; i--) {
		if(minutes >= minutes_to_bitshift_map[i]) {
			minutes -= minutes_to_bitshift_map[i];
			minutes_byte_value |= (1u << i);
			printk("adding: %d minutes \r\n", minutes_to_bitshift_map[i]);
		}
	}

	uint8_t hours_byte_value = 0;
	for(int i = NUM_HOUR_INCREMENTS - 1; i >= 0; i--) {
		if(hours >= hours_to_bitshift_map[i]) {
			hours -= hours_to_bitshift_map[i];
			hours_byte_value |= (1u << i);
			printk("adding: %d hours \r\n", hours_to_bitshift_map[i]);
		}
	}

	if(hours != 0 || minutes != 0) {
		printk("couldn't convert hours or minutes value to reg\r\n");
		return false;
	}

	*minutes_reg = minutes_byte_value;
	*hours_reg = hours_byte_value;
	
	return true;
}

void rv8803_convert_reg_values_to_time(uint8_t seconds_reg, uint8_t minutes_reg, uint8_t hours_reg, 
								       uint8_t* seconds, uint8_t* minutes, uint8_t* hours) {
	
	#define NUM_SEC_MIN_INCREMENTS 7
	#define NUM_HOUR_INCREMENTS 6
	uint8_t seconds_minutes_to_bitshift_map[NUM_MIN_INCREMENTS] = {1, 2, 4, 8, 10, 20, 40};
	uint8_t hours_to_bitshift_map[NUM_HOUR_INCREMENTS] = {1, 2, 4, 8, 10, 20};

	uint8_t seconds_result = 0;
	uint8_t minutes_result = 0;
	uint8_t hours_result = 0;

	for(int i = 0; i < NUM_SEC_MIN_INCREMENTS; i++) {
		if(seconds_reg == 0) {
			break;
		}
		if((seconds_reg & 1u) == 1u) {
			seconds_result += seconds_minutes_to_bitshift_map[i];
		}
		seconds_reg = seconds_reg >> 1;
	}

	for(int i = 0; i < NUM_SEC_MIN_INCREMENTS; i++) {
		if(minutes_reg == 0) {
			break;
		}
		if((minutes_reg & 1u) == 1u) {
			minutes_result += seconds_minutes_to_bitshift_map[i];
		}
		minutes_reg = minutes_reg >> 1;
	}

	for(int i = 0; i < NUM_HOUR_INCREMENTS; i++) {
		if(hours_reg == 0) {
			break;
		}
		if((hours_reg & 1u) == 1u) {
			hours_result += hours_to_bitshift_map[i];
		}
		hours_reg = hours_reg >> 1;
	}
	
	*seconds = seconds_result;
	*minutes = minutes_result;
	*hours = hours_result;
}

int rv8803_get_time_from_registers(const struct device* dev, uint8_t* seconds, uint8_t* minutes, uint8_t* hours) {
	const struct rv8803_dev_config *config = dev->config;
	
	uint8_t seconds_reg = 0;
	int i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, RV8803_SECONDS_REG, &seconds_reg);
	if(i2c_operation_error) {
		printk("trouble reading rv8803 seconds \r\n");
		return -EIO;
	}
	
	uint8_t minutes_reg = 0;
	i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, RV8803_MINUTES_REG, &minutes_reg);
	if(i2c_operation_error) {
		printk("trouble reading rv8803 minutes \r\n");
		return -EIO;
	}
	
	uint8_t hours_reg = 0;
	i2c_operation_error = i2c_reg_read_byte_dt(&config->i2c, RV8803_HOURS_REG, &hours_reg);
	if(i2c_operation_error) {
		printk("trouble reading rv8803 hours \r\n");
		return -EIO;
	}

	uint8_t converted_seconds = 0;
	uint8_t converted_minutes = 0;
	uint8_t converted_hours = 0;

	rv8803_convert_reg_values_to_time(seconds_reg, minutes_reg, hours_reg,
										&converted_seconds, &converted_minutes, & converted_hours);

	*seconds = converted_seconds;
	*minutes = converted_minutes;
	*hours = converted_hours;

	return 0;
}

static int rv8803_channel_get(const struct device *dev, 
                               enum sensor_channel chan, 
                               struct sensor_value *val) {

	if(chan != RV8803_GET_TIME) {
		printk("invalid channel for rv8803 channel_get \r\n");
		return -ENODEV;
	}

	// 1). read the required time
	// 2). if seconds is 59, read again
	// 2a). if seconds is 59 again, previous reading is good
	// 2b). otherwise, read required time again
	
	uint8_t seconds = 0;
	uint8_t minutes = 0;
	uint8_t hours = 0;

	int time_retrieval_error = rv8803_get_time_from_registers(dev, &seconds, &minutes, &hours);
	if(time_retrieval_error) {
		printk("trouble retriving rv8803 time \r\n");
		return -EIO;
	}

	if(seconds != 59){
		val->val1 = hours;
		val->val2 = minutes;
		return 0;
	}

	time_retrieval_error = rv8803_get_time_from_registers(dev, &seconds, &minutes, &hours);
	if(time_retrieval_error) {
		printk("trouble retriving rv8803 time \r\n");
		return -EIO;
	}

	val->val1 = hours;
	val->val2 = minutes;
		
    return 0;
}


static int rv8803_attr_set(const struct device *dev, 
                            enum sensor_channel chan, 
                            enum sensor_attribute attr, 
                            const struct sensor_value *val) {

	const struct rv8803_dev_config *config = dev->config;

	switch(chan) {
		case RV8803_INITIALIZE : {
			// what do we need to do here?
			// 1). turn off interrupts (off by default)
			// 2). perform offset calibration here
			// 2a). set OFFSET field to 0
			// 2b). select 1 Hz at CLKOUT by setting FD field of register 0x0D to 10
			// 2c). turn on the CLKOUT pin into output mode by setting CLKOE to high
			// 2d). compute offset in ppm and in steps
			switch(attr) {
				case RV8803_INITIALIZE_START: {
					uint8_t offset_reg_byte_zero = 0b00000000;
					int i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_OFFSET_REG, offset_reg_byte_zero);
					if(i2c_operation_error) {
						printk("trouble clearing RV8803 offset register \r\n");
						return -EIO;
					}

					uint8_t extension_reg_byte = 0b00001000;
					i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_EXTENSION_REG, extension_reg_byte);
					if(i2c_operation_error) {
						printk("trouble setting CLKOUT frequency\r\n");
						return -EIO;
					}

					printk("finished setting offset registers \r\n");

					break;					
				}
				case RV8803_INITIALIZE_COMPUTE_OFFSET: {
					
					uint8_t offset_reg_byte;
					if(val->val1 >= 0 && val->val1 <= 31) {
						offset_reg_byte = val->val1;
					} else if(val->val1 >= -32 && val->val1 <= -1) {
						offset_reg_byte = val->val1 + 64;
					} else {
						printk("offset can't be set \r\n");
						return -EIO;
					}

					int i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_OFFSET_REG, offset_reg_byte);
					if(i2c_operation_error) {
						printk("trouble setting RV8803 offset register \r\n");
						return -EIO;
					}
					break;
				}
			}
			break;
		}
		case RV8803_SET_TIME : {
			// 1). set RESET bit to 1
			// 2). set the hours and minutes
			// 3). release the reset bit to 0
			uint8_t control_reg_byte_enable_reset = 0b00000001;
			int i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_CONTROL_REG, control_reg_byte_enable_reset);
			if(i2c_operation_error) {
				printk("trouble setting RV8803 RESET bit \r\n");
				return -EIO;
			}

			uint8_t seconds = 0;
			uint8_t hours = 0;
			uint8_t minutes = 0;

			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_SECONDS_REG, seconds);
			if(i2c_operation_error) {
				printk("trouble setting RV8803 seconds reg \r\n");
				return -EIO;
			}

			bool convert_val_to_reg = rv8803_convert_time_to_reg_values(val->val1, val->val2, &hours, &minutes);
			if(!convert_val_to_reg) {
				printk("trouble converting RV8803 val to reg \r\n");
				return -ENODEV;
			}

			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_MINUTES_REG, minutes);
			if(i2c_operation_error) {
				printk("trouble setting RV8803 minutes reg \r\n");
				return -EIO;
			}

			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_HOURS_REG, hours);
			if(i2c_operation_error) {
				printk("trouble setting RV8803 hours reg \r\n");
				return -EIO;
			}

			uint8_t control_reg_byte_disable_reset = 0b00000000;
			i2c_operation_error = i2c_reg_write_byte_dt(&config->i2c, RV8803_CONTROL_REG, control_reg_byte_disable_reset);
			if(i2c_operation_error) {
				printk("trouble clearing RV8803 RESET bit \r\n");
				return -EIO;
			}

			break;
		}
	}
    return 0;
}

int rv8803_init(const struct device *dev) {

    const struct rv8803_dev_config *config = dev->config;
    
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("failure with i2c bus \r\n");
		return -ENODEV;
	}

	return 0;
}

static const struct sensor_driver_api rv8803_api_funcs = {
	.sample_fetch = rv8803_sample_fetch,
	.channel_get = rv8803_channel_get,
	.attr_set = rv8803_attr_set,
};

#define rv8803_DEVICE_INIT(inst)					\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,				\
			      rv8803_init,				        \
			      NULL,					            \
			      &rv8803_data_##inst,			    \
			      &rv8803_config_##inst,			\
			      POST_KERNEL,				        \
			      CONFIG_SENSOR_INIT_PRIORITY,		\
			      &rv8803_api_funcs);

#define rv8803_CONFIG(inst)								\
    {                                                       \
		.i2c = I2C_DT_SPEC_INST_GET(inst),					\
    }                                                       \

#define rv8803_DEFINE(inst)                                        \
    static struct rv8803_data rv8803_data_##inst;                 \
    static const struct rv8803_dev_config rv8803_config_##inst =	\
        rv8803_CONFIG(inst);				                        \
    rv8803_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(rv8803_DEFINE)
