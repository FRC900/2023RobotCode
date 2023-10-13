// orignal comments :
/*!
 * @file Adafruit_AS726x.h
 *
 * This is a library for the Adafruit AS726x breakout board
 * ----> https://www.adafruit.com/products/3779
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Dean Miller for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef AS726X_INC__
#define AS726X_INC__

#include <cstdint>
#include <memory>
#include <mutex>
#include "ros/console.h"
#include "frc/I2C.h"

namespace as726x
{
/*=========================================================================
  I2C ADDRESS/BITS
  -----------------------------------------------------------------------*/
constexpr int AS726x_ADDRESS = 0x49; ///< default I2C address
/*=========================================================================*/


/**************************************************************************/
/*!
  @brief  virtual registers
  */
/**************************************************************************/
constexpr uint8_t AS726X_HW_VERSION	=	0x00;
constexpr uint8_t AS726X_FW_VERSION	=	0x02;
constexpr uint8_t AS726X_CONTROL_SETUP =	0x04;
constexpr uint8_t AS726X_INT_T		=	0x05;
constexpr uint8_t AS726X_DEVICE_TEMP	 =	0x06;
constexpr uint8_t AS726X_LED_CONTROL	 =	0x07;

//for reading sensor data
constexpr uint8_t AS7262_V_HIGH	=		0x08;
constexpr uint8_t AS7262_V_LOW		=	0x09;
constexpr uint8_t AS7262_B_HIGH		=	0x0A;
constexpr uint8_t AS7262_B_LOW		=	0x0B;
constexpr uint8_t AS7262_G_HIGH		=	0x0C;
constexpr uint8_t AS7262_G_LOW		=	0x0D;
constexpr uint8_t AS7262_Y_HIGH		=	0x0E;
constexpr uint8_t AS7262_Y_LOW		=	0x0F;
constexpr uint8_t AS7262_O_HIGH		=	0x10;
constexpr uint8_t AS7262_O_LOW		=	0x11;
constexpr uint8_t AS7262_R_HIGH		=	0x12;
constexpr uint8_t AS7262_R_LOW		=	0x13;

constexpr uint8_t AS7262_V_CAL		=	0x14;
constexpr uint8_t AS7262_B_CAL		=	0x18;
constexpr uint8_t AS7262_G_CAL		=	0x1C;
constexpr uint8_t AS7262_Y_CAL		=	0x20;
constexpr uint8_t AS7262_O_CAL		=	0x24;
constexpr uint8_t AS7262_R_CAL		=	0x28;

/**************************************************************************/
/*!
  @brief  hardware registers
  */
/**************************************************************************/
constexpr uint8_t AS726X_SLAVE_STATUS_REG = 0x00;
constexpr uint8_t AS726X_SLAVE_WRITE_REG = 0x01;
constexpr uint8_t AS726X_SLAVE_READ_REG = 0x02;
constexpr uint8_t AS726X_SLAVE_TX_VALID = 0x02;
constexpr uint8_t AS726X_SLAVE_RX_VALID = 0x01;

/**************************************************************************/
/*!
  @brief  color registers
  */
/**************************************************************************/
constexpr uint8_t AS7262_VIOLET = 0x08;
constexpr uint8_t AS7262_BLUE = 0x0A;
constexpr uint8_t AS7262_GREEN = 0x0C;
constexpr uint8_t AS7262_YELLOW = 0x0E;
constexpr uint8_t AS7262_ORANGE = 0x10;
constexpr uint8_t AS7262_RED = 0x12;
constexpr uint8_t AS7262_VIOLET_CALIBRATED = 0x14;
constexpr uint8_t AS7262_BLUE_CALIBRATED = 0x18;
constexpr uint8_t AS7262_GREEN_CALIBRATED = 0x1C;
constexpr uint8_t AS7262_YELLOW_CALIBRATED = 0x20;
constexpr uint8_t AS7262_ORANGE_CALIBRATED = 0x24;
constexpr uint8_t AS7262_RED_CALIBRATED = 0x28;

/**************************************************************************/
/*!
  @brief  conversion modes. Default is Mode 2
  */
/**************************************************************************/
enum conversion_types{
	MODE_0 = 0b00,
	MODE_1 = 0b01,
	MODE_2 = 0b10, //default
	ONE_SHOT = 0b11,
};

/**************************************************************************/
/*!
  @brief gain settings. Default is 1x gain
  */
/**************************************************************************/
enum channel_gain {
	GAIN_1X = 0b00, //default
	GAIN_3X7 = 0b01,
	GAIN_16X = 0b10,
	GAIN_64X = 0b11,
};

/**************************************************************************/
/*!
  @brief  indicator LED current limit settings. Default is 1mA
  */
/**************************************************************************/
enum ind_led_current_limits {
	LIMIT_1MA = 0b00, //default
	LIMIT_2MA = 0b01,
	LIMIT_4MA = 0b10,
	LIMIT_8MA = 0b11,
};

/**************************************************************************/
/*!
  @brief  Driver LED current limit settings. Default is 12.5 mA
  */
/**************************************************************************/
enum drv_led_current_limits {
	LIMIT_12MA5 = 0b00, //default
	LIMIT_25MA = 0b01,
	LIMIT_50MA = 0b10,
	LIMIT_100MA = 0b11,
};


/*=========================================================================*/

constexpr float AS726x_INTEGRATION_TIME_MULT = 2.8; ///< multiplier for integration time
constexpr uint8_t AS726x_NUM_CHANNELS = 6; ///< number of sensor channels

/**************************************************************************/
/*!
  @brief  Color definitions used by the library
  */
/**************************************************************************/
enum {
	AS726x_VIOLET = 0,
	AS726x_BLUE,
	AS726x_GREEN,
	AS726x_YELLOW,
	AS726x_ORANGE,
	AS726x_RED,
};

/**************************************************************************/
/*!
  @brief  Class that stores state and functions for interacting with AS726x spectral sensors
  */
/**************************************************************************/
class AS726x {
	public:
		/*!
		  @brief  Class constructor
		  @param addr Optional I2C address the sensor can be found on. Defaults to 0x49.
		  */
		AS726x(void)
			: mutex_{std::make_shared<std::mutex>()}
		{}
		AS726x(const AS726x &) = delete;
		AS726x(AS726x &&) = default;
		virtual ~AS726x(void) {}

        AS726x &operator=(const AS726x &) = default;
		AS726x &operator=(AS726x &&) = default;

		bool begin(void);

		/*========= LED STUFF =========*/

		// Set indicator LED current
		void setIndicateCurrent(uint8_t current);
		// turn on/off indicator
		void indicateLED(bool on);

		//turn on the drv led
		void drvOn();
		//turn off the drv led
		void drvOff();

		//set current through drv led
		void setDrvCurrent(uint8_t current);

		/*===== END LED STUFF ======*/

		void setConversionType(uint8_t type);
		void setGain(uint8_t gain);
		void setIntegrationTime(uint8_t time);
		void enableInterrupt();
		void disableInterrupt();

		/*====== MEASUREMENTS ========*/

		//read sensor data
		void startMeasurement();

		/*!
		  @brief  Check if the sensor is ready to return data
		  @return true if data is ready to be read, false otherwise.
		  */
		bool dataReady() { return (virtualRead(AS726X_CONTROL_SETUP) & 0x02) ? true : false; }

		/*!
		  @brief  Read the on-board temperature sensor
		  @return the temperature in Centigrade.
		  */
		uint8_t readTemperature() { return virtualRead(AS726X_DEVICE_TEMP); }
		uint16_t readChannel(uint8_t channel);

		/*!
		  @brief  Read raw violet color value (AS7262 only)
		  @return the violet reading as an unsigned 16-bit integer
		  */
		uint16_t readViolet() { return(readChannel(AS7262_VIOLET)); }
		/*!
		  @brief  Read raw blue color value (AS7262 only)
		  @return the blue reading as an unsigned 16-bit integer
		  */
		uint16_t readBlue() { return(readChannel(AS7262_BLUE)); }
		/*!
		  @brief  Read raw green color value (AS7262 only)
		  @return the green reading as an unsigned 16-bit integer
		  */
		uint16_t readGreen() { return(readChannel(AS7262_GREEN)); }
		/*!
		  @brief  Read raw yellow color value (AS7262 only)
		  @return the yellow reading as an unsigned 16-bit integer
		  */
		uint16_t readYellow() { return(readChannel(AS7262_YELLOW)); }
		/*!
		  @brief  Read raw orange color value (AS7262 only)
		  @return the orange reading as an unsigned 16-bit integer
		  */
		uint16_t readOrange() { return(readChannel(AS7262_ORANGE)); }
		/*!
		  @brief  Read raw red color value (AS7262 only)
		  @return the red reading as an unsigned 16-bit integer
		  */
		uint16_t readRed() { return(readChannel(AS7262_RED)); }

		void readRawValues(uint16_t *buf, uint8_t num = AS726x_NUM_CHANNELS);

		float readCalibratedValue(uint8_t channel);

		/*!
		  @brief  Read calibrated violet color value (AS7262 only)
		  @return the violet reading as a 32-bit floating point number
		  */
		float readCalibratedViolet() { return(readCalibratedValue(AS7262_VIOLET_CALIBRATED)); }
		/*!
		  @brief  Read calibrated blue color value (AS7262 only)
		  @return the blue reading as a 32-bit floating point number
		  */
		float readCalibratedBlue() { return(readCalibratedValue(AS7262_BLUE_CALIBRATED)); }
		/*!
		  @brief  Read calibrated green color value (AS7262 only)
		  @return the green reading as a 32-bit floating point number
		  */
		float readCalibratedGreen() { return(readCalibratedValue(AS7262_GREEN_CALIBRATED)); }
		/*!
		  @brief  Read calibrated yellow color value (AS7262 only)
		  @return the yellow reading as a 32-bit floating point number
		  */
		float readCalibratedYellow() { return(readCalibratedValue(AS7262_YELLOW_CALIBRATED)); }
		/*!
		  @brief  Read calibrated orange color value (AS7262 only)
		  @return the orange reading as a 32-bit floating point number
		  */
		float readCalibratedOrange() { return(readCalibratedValue(AS7262_ORANGE_CALIBRATED)); }
		/*!
		  @brief  Read calibrated red color value (AS7262 only)
		  @return the red reading as a 32-bit floating point number
		  */
		float readCalibratedRed() { return(readCalibratedValue(AS7262_RED_CALIBRATED)); }

		void readCalibratedValues(float *buf, uint8_t num = AS726x_NUM_CHANNELS);

		std::shared_ptr<std::mutex> getMutex(void)
		{
			return mutex_;
		}
		/*==== END MEASUREMENTS =====*/

	private:
		void      write8(uint8_t reg, uint8_t value);
		uint8_t   read8(uint8_t reg);

		uint8_t virtualRead(uint8_t addr);
		void virtualWrite(uint8_t addr, uint8_t value);

		virtual void read(uint8_t reg, uint8_t *buf, uint8_t num) = 0;
		virtual void write(uint8_t reg, uint8_t *buf, uint8_t num) = 0;
		void _i2c_init();

		struct control_setup {
			control_setup()
				: unused(0)
				, DATA_RDY(0)
				, BANK(0)
				, GAIN(0)
				, INT(0)
				, RST(0)
			{
			}

			uint8_t unused : 1;

			/* 1: data ready to be read, sets int active if int is enabled */
			uint8_t DATA_RDY : 1;

			/* conversion type
			 *  0b00 = Mode 0
			 *  0b01 = Mode 1
			 *  0b10 = Mode 2
			 *  0b11 = Mode 3 One shot
			 */
			uint8_t BANK : 2;

			/* Channel gain setting (all channels)
			 *  0b00 = 1x
			 *  0b01 = 3.7x
			 *  0b10 = 16x
			 *  0b11 = 64x
			 */
			uint8_t GAIN : 2;

			/* enable or disable interrupt */
			uint8_t INT : 1;
			uint8_t RST : 1;

			uint8_t get() {
				return ( (DATA_RDY << 1) | (BANK << 2) | (GAIN << 4) | (INT << 6) | (RST << 7) );
			};
		};
		control_setup _control_setup;

		struct int_time {
			int_time()
				: INT_T(0)
			{
			}

			//integration time (multiplied by INTEGRATION_TIME_MULT) in ms
			uint8_t INT_T : 8;

			uint8_t get() {
				return INT_T;
			};
		};
		int_time _int_time;

		struct led_control {
			led_control()
				: LED_IND(0)
				, ICL_IND(0)
				, LED_DRV(0)
				, ICL_DRV(0)
			{
			}
			// enable or disable indicator LED
			uint8_t LED_IND : 1;

			//indicator led current limit
			uint8_t ICL_IND : 2;

			//enable or disable led_drv
			uint8_t LED_DRV : 1;

			uint8_t ICL_DRV : 2;

			uint8_t get() {
				return ( LED_IND | (ICL_IND << 1) | (LED_DRV << 3) | (ICL_DRV << 4) );
			};
		};
		led_control _led_control;

		std::shared_ptr<std::mutex> mutex_;
};

template <bool SIMFLAG>
class roboRIO_AS726x : public AS726x
{
	public:
		roboRIO_AS726x(const frc::I2C::Port &port, int deviceAddress = AS726x_ADDRESS);
		roboRIO_AS726x(const roboRIO_AS726x &);
		roboRIO_AS726x(roboRIO_AS726x &&) noexcept;
		virtual ~roboRIO_AS726x() = default;
		roboRIO_AS726x &operator=(const roboRIO_AS726x &);
		roboRIO_AS726x &operator=(roboRIO_AS726x &&) noexcept;
	private:
		virtual void read(uint8_t reg, uint8_t *buf, uint8_t num) override;
		virtual void write(uint8_t reg, uint8_t *buf, uint8_t num) override;
};

template <>
class roboRIO_AS726x<false> : public AS726x
{
	public:
		roboRIO_AS726x(const frc::I2C::Port &port, int deviceAddress = AS726x_ADDRESS)
		: AS726x()
		, i2c_{std::make_shared<frc::I2C>(port, deviceAddress)}
		{
		}
		roboRIO_AS726x(const roboRIO_AS726x &) = delete;
		roboRIO_AS726x(roboRIO_AS726x &&) noexcept = delete;
		virtual ~roboRIO_AS726x() = default;
		roboRIO_AS726x &operator=(const roboRIO_AS726x &) = delete;
		roboRIO_AS726x &operator=(roboRIO_AS726x &&) noexcept = delete;
	private:
		virtual void read(uint8_t reg, uint8_t *buf, uint8_t num)
		{
			auto rc = i2c_->Read(reg, num, buf);
			// ROS_INFO_STREAM("roboRIO_AS726x::" << __FUNCTION__ << ":" << __LINE__ << " reg=" << std::hex << static_cast<int>(reg) << std::dec << static_cast<int>(num) << " *buf=" << std::hex << static_cast<int>(*buf) << " rc=" << rc);
		}
		virtual void write(uint8_t reg, uint8_t *buf, uint8_t num)
		{
			if (num > 1)
			{
				ROS_ERROR_STREAM("Invalid write count " << num << " in " << __FUNCTION__ << ":" << __LINE__);
			}
			auto rc = i2c_->Write(reg, *buf);
			// ROS_INFO_STREAM("roboRIO_AS726x::" << __FUNCTION__ << ":" << __LINE__ << " reg=" << std::hex << static_cast<int>(reg) << " num=" << std::dec << static_cast<int>(num) << " *buf=" << std::hex << static_cast<int>(*buf) << " rc=" << rc);
		}
		std::shared_ptr<frc::I2C> i2c_;
};

template <>
class roboRIO_AS726x<true> : public AS726x
{
	public:
		roboRIO_AS726x(const frc::I2C::Port &port, int deviceAddress = AS726x_ADDRESS) 
		    :AS726x()
		{}
		roboRIO_AS726x(const roboRIO_AS726x &) = delete;
		roboRIO_AS726x(roboRIO_AS726x &&) noexcept = delete;
		virtual ~roboRIO_AS726x() = default;
		roboRIO_AS726x &operator=(const roboRIO_AS726x &) = delete;
		roboRIO_AS726x &operator=(roboRIO_AS726x &&) noexcept = delete;

	private:
		virtual void read(uint8_t reg, uint8_t *buf, uint8_t num)
		{
			*buf = AS726X_SLAVE_RX_VALID;
			// ROS_INFO_STREAM("roboRIO_AS726x::" << __FUNCTION__ << ":" << __LINE__ << " reg=" << std::hex << static_cast<int>(reg) << std::dec << static_cast<int>(num) << " *buf=" << std::hex << static_cast<int>(*buf));
		}
		virtual void write(uint8_t reg, uint8_t *buf, uint8_t num)
		{
			// ROS_INFO_STREAM("roboRIO_AS726x::" << __FUNCTION__ << ":" << __LINE__ << " reg=" << std::hex << static_cast<int>(reg) << " num=" << std::dec << static_cast<int>(num) << " *buf=" << std::hex << static_cast<int>(*buf));
		}
};
} //namespace

#endif