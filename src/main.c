#include <zephyr.h>
#include <device.h>
#include <drivers/uart.h>
#include <string.h>

#include <usb/usb_device.h>
#include <usb/class/usb_hid.h>
#include <usb/class/usb_cdc.h>

#include <drivers/adc.h>
#include <hal/nrf_saadc.h>

#define ADC_DEVICE_NAME		DT_ADC_0_NAME
#define ADC_RESOLUTION		12
#define ADC_GAIN			ADC_GAIN_1
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID	0
#define ADC_1ST_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN7
#define ADC_2ND_CHANNEL_ID	2
#define ADC_2ND_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN2

#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);




/* HID */
static const u8_t hid_kbd_report_desc[] = HID_KEYBOARD_REPORT_DESC();

static K_SEM_DEFINE(usb_sem, 1, 1);	/* starts off "available" */

static char data_buf_kbd[64];
static u8_t chr_ptr_kbd;


static volatile int data_over_cdc = 0;

static void in_ready_cb(void)
{
	k_sem_give(&usb_sem);
}

static const struct hid_ops ops = {
	.int_in_ready = in_ready_cb,
};



static int ascii_to_hid(u8_t ascii)
{
	if (ascii < 32) {
		/* Character not supported */
		return -1;
	} else if (ascii < 48) {
		/* Special characters */
		switch (ascii) {
		case 32:
			return HID_KEY_SPACE;
		case 33:
			return HID_KEY_1;
		case 34:
			return HID_KEY_APOSTROPHE;
		case 35:
			return HID_KEY_3;
		case 36:
			return HID_KEY_4;
		case 37:
			return HID_KEY_5;
		case 38:
			return HID_KEY_7;
		case 39:
			return HID_KEY_APOSTROPHE;
		case 40:
			return HID_KEY_9;
		case 41:
			return HID_KEY_0;
		case 42:
			return HID_KEY_8;
		case 43:
			return HID_KEY_EQUAL;
		case 44:
			return HID_KEY_COMMA;
		case 45:
			return HID_KEY_MINUS;
		case 46:
			return HID_KEY_DOT;
		case 47:
			return HID_KEY_SLASH;
		default:
			return -1;
		}
	} else if (ascii < 58) {
		/* Numbers */
		if (ascii == 48U) {
			return HID_KEY_0;
		} else {
			return ascii - 19;
		}
	} else if (ascii < 65) {
		/* Special characters #2 */
		switch (ascii) {
		case 58:
			return HID_KEY_SEMICOLON;
		case 59:
			return HID_KEY_SEMICOLON;
		case 60:
			return HID_KEY_COMMA;
		case 61:
			return HID_KEY_EQUAL;
		case 62:
			return HID_KEY_DOT;
		case 63:
			return HID_KEY_SLASH;
		case 64:
			return HID_KEY_2;
		default:
			return -1;
		}
	} else if (ascii < 91) {
		/* Uppercase characters */
		return ascii - 61U;
	} else if (ascii < 97) {
		/* Special characters #3 */
		switch (ascii) {
		case 91:
			return HID_KEY_LEFTBRACE;
		case 92:
			return HID_KEY_BACKSLASH;
		case 93:
			return HID_KEY_RIGHTBRACE;
		case 94:
			return HID_KEY_6;
		case 95:
			return HID_KEY_MINUS;
		case 96:
			return HID_KEY_GRAVE;
		default:
			return -1;
		}
	} else if (ascii < 123) {
		/* Lowercase letters */
		return ascii - 93;
	} else if (ascii < 128) {
		/* Special characters #4 */
		switch (ascii) {
		case 123:
			return HID_KEY_LEFTBRACE;
		case 124:
			return HID_KEY_BACKSLASH;
		case 125:
			return HID_KEY_RIGHTBRACE;
		case 126:
			return HID_KEY_GRAVE;
		case 127:
			return HID_KEY_DELETE;
		default:
			return -1;
		}
	}

	return -1;
}

static bool needs_shift(u8_t ascii)
{
	if ((ascii < 33) || (ascii == 39U)) {
		return false;
	} else if ((ascii >= 33U) && (ascii < 44)) {
		return true;
	} else if ((ascii >= 44U) && (ascii < 58)) {
		return false;
	} else if ((ascii == 59U) || (ascii == 61U)) {
		return false;
	} else if ((ascii >= 58U) && (ascii < 91)) {
		return true;
	} else if ((ascii >= 91U) && (ascii < 94)) {
		return false;
	} else if ((ascii == 94U) || (ascii == 95U)) {
		return true;
	} else if ((ascii > 95) && (ascii < 123)) {
		return false;
	} else if ((ascii > 122) && (ascii < 127)) {
		return true;
	} else {
		return false;
	}
}

/* CDC ACM */
static volatile bool data_transmitted;
static volatile bool data_arrived;


static void write_data(struct device *dev, const char *buf, int len)
{
	uart_irq_tx_enable(dev);

	while (len) {
		int written;

		data_transmitted = false;
		written = uart_fifo_fill(dev, (const u8_t *)buf, len);
		while (data_transmitted == false) {
			k_yield();
		}

		len -= written;
		buf += written;
	}

	uart_irq_tx_disable(dev);
}

static void cdc_kbd_int_handler(struct device *dev)
{
	uart_irq_update(dev);

	if (uart_irq_tx_ready(dev)) {
		data_transmitted = true;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}
	u32_t bytes_read;

	while ((bytes_read = uart_fifo_read(dev, (u8_t *)data_buf_kbd, 1))) {
		chr_ptr_kbd += bytes_read;
		if (data_buf_kbd[0] == 'g') {
			data_over_cdc = 1;
		} else if (data_buf_kbd[0] == ' ') {
			data_over_cdc = 0;
		}
	}
}



#define BUFFER_SIZE  128
static s16_t m_sample_buffer[BUFFER_SIZE];

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};


static struct device *init_adc(void)
{
	int ret;
	struct device *adc_dev = device_get_binding(ADC_DEVICE_NAME);

	ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);

	(void)memset(m_sample_buffer, 0, sizeof(m_sample_buffer));

	return adc_dev;
}


static s16_t read_adc(struct device *adc_dev)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer[0]),
		.resolution  = ADC_RESOLUTION,
	};

	ret = adc_read(adc_dev, &sequence);

	// printk("ADC: %d\r\n", m_sample_buffer[0]);

	return m_sample_buffer[0];
}

static s16_t* read_adc_sequence(struct device *adc_dev)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	ret = adc_read(adc_dev, &sequence);

	// printk("ADC: %d\r\n", m_sample_buffer[0]);

	return m_sample_buffer;
}

static s16_t analyze(s16_t *buf, int size, s16_t *pmax, s16_t *pmin, s16_t *pdiff)
{
	s32_t sum = 0;
	s16_t max = buf[0];
	s16_t min = buf[0];
	s32_t diff = 0;
	s16_t avg;
	
	for (int i=0; i<size; i++) {
		s16_t value = buf[i];
		sum += value;
		if (value > max) {
			max = value;
		} else if (value < min) {
			min = value;
		}
	}

	avg = sum / size;

	for (int i=0; i<size; i++) {
		s16_t value = buf[i] - avg;
		diff += value >= 0 ? value : -value;
	}

	*pdiff = diff;
	*pmax = max;
	*pmin = min;

	return avg;
}

void main(void)
{
	struct device *adc0_dev, *hid0_dev, *cdc0_dev;

	/* Configure devices */
	adc0_dev = init_adc();

	hid0_dev = device_get_binding("HID_0");
	if (hid0_dev == NULL) {
		LOG_ERR("Cannot get USB HID 0 Device");
		return;
	}

	cdc0_dev = device_get_binding("CDC_ACM_0");
	if (cdc0_dev == NULL) {
		LOG_ERR("Cannot get USB CDC 0 Device");
		return;
	}

	/* Initialize HID */
	usb_hid_register_device(hid0_dev, hid_kbd_report_desc,
				sizeof(hid_kbd_report_desc), &ops);
	usb_hid_init(hid0_dev);

	/* Initialize CDC ACM */

	// LOG_INF("Wait for DTR on CDC ACM 0");
	// while (1) {
	// 	uart_line_ctrl_get(cdc0_dev, LINE_CTRL_DTR, &dtr);
	// 	if (dtr) {
	// 		break;
	// 	}
	// }
	// LOG_INF("DTR on CDC ACM 0 set");


	// /* Wait 1 sec for the host to do all settings */
	// k_busy_wait(K_SECONDS(1) * USEC_PER_MSEC);

	uart_irq_callback_set(cdc0_dev, cdc_kbd_int_handler);
	uart_irq_rx_enable(cdc0_dev);

	s16_t samples[128];
	s16_t max[2];
	s16_t min[2];
	s16_t avg[2];
	s16_t diff[2];

	while (true) {
		for (int16_t i=0; i<sizeof(samples); i++) {
			samples[i] = read_adc(adc0_dev);
			k_usleep(100);
		}

		avg[0] = analyze(samples, 64, &max[0], &min[0], &diff[0]);
		
		avg[1] = analyze(samples + 64, 64, &max[1], &min[1], &diff[1]);

		if ((avg[1] + diff[1]) > avg[0] && (avg[1] - diff[1]) < avg[0]) {
			break;
		}
	}


	s16_t threshold = avg[1] + 3 *(max[1] - avg[1]);
	s16_t touch = 0;
	while (true) {
		s16_t value = read_adc(adc0_dev);
		if (value > threshold) {
			if (!touch) {
				u8_t rep[] = {0x00, 0x00, 0x00, 0x00,
						      0x00, 0x00, 0x00, 0x00};
				rep[7] = ascii_to_hid(' ');
				hid_int_ep_write(hid0_dev, rep, sizeof(rep), NULL);
				
				touch = 100;
			}
		} else {
			if (touch) {
				touch--;
				if (touch == 0) {
					u8_t rep[] = {0x00, 0x00, 0x00, 0x00,
						      0x00, 0x00, 0x00, 0x00};
					hid_int_ep_write(hid0_dev, rep,
							sizeof(rep), NULL);
				}
			}
		}

		if (data_over_cdc) {
			write_data(cdc0_dev, &value, 2);
		}
		k_usleep(100);
	}
}
