#ifndef I2CIP_PCA9685_H_
#define I2CIP_PCA9685_H_

#include <Arduino.h>
#include <I2CIP.h>

// MACROS

// REGISTER ADDRESSES
#define PCA9685_MODE1 (uint8_t)0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 (uint8_t)0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 (uint8_t)0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 (uint8_t)0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 (uint8_t)0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR (uint8_t)0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L (uint8_t)0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H (uint8_t)0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L (uint8_t)0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H (uint8_t)0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L (uint8_t)0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H (uint8_t)0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L (uint8_t)0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H (uint8_t)0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE (uint8_t)0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE (uint8_t)0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL (uint8_t)0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 (uint8_t)0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 (uint8_t)0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 (uint8_t)0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP (uint8_t)0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI (uint8_t)0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK (uint8_t)0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART (uint8_t)0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 (uint8_t)0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1 (uint8_t)0x02 /**< Active (uint8_t)LOW output enable input - high impedience */
#define MODE2_OUTDRV (uint8_t)0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH (uint8_t)0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT (uint8_t)0x10  /**< Output logic state inverted */

#define I2CIP_PCA9685_OSCFREQ 27000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */
#define I2CIP_PCA9685_PRESCALE (uint8_t)(((I2CIP_PCA9685_OSCFREQ / (I2CIP_PCA9685_FREQ * 4096.0)) + 0.5) - 1)

// Settings
#define I2CIP_PCA9685_ADDRESS 0x40 // Default
#define I2CIP_PCA9685_DELAY 10 // Write/Read Sensing Delay
// #define I2CIP_PCA9685_FREQ 1600 // Default PWM Frequency
#define I2CIP_PCA9685_FREQ 490 // Default PWM Frequency

#ifdef I2CIP_USE_GUARANTEES
#define I2CIP_GUARANTEE_PCA9685 9685
class PCA9685;
I2CIP_GUARANTEE_DEFINE(PCA9685, I2CIP_GUARANTEE_PCA9685);
#endif

typedef uint16_t i2cip_pca9685_t;

typedef enum {
  PCA9685_CH0 = 0,
  PCA9685_CH1,
  PCA9685_CH2,
  PCA9685_CH3,
  PCA9685_CH4,
  PCA9685_CH5,
  PCA9685_CH6,
  PCA9685_CH7,
  PCA9685_CH8,
  PCA9685_CH9,
  PCA9685_CH10,
  PCA9685_CH11,
  PCA9685_CH12,
  PCA9685_CH13,
  PCA9685_CH14,
  PCA9685_CH15,
  PCA9685_NONE = 0xFF
} i2cip_pca9685_chsel_t;

// const char i2cip_pca9685_id_progmem[] PROGMEM = {"PCA9685"};

// Interface class for the PCA9685 air temperature and humidity sensor
class PCA9685 : public I2CIP::Device, public I2CIP::OutputInterface<i2cip_pca9685_t, i2cip_pca9685_chsel_t>
  #ifdef I2CIP_USE_GUARANTEES
  , public Guarantee<PCA9685>
  #endif
  {
  I2CIP_DEVICE_CLASS_BUNDLE(PCA9685);
  #ifdef I2CIP_USE_GUARANTEES
  I2CIP_CLASS_USE_GUARANTEE(PCA9685, I2CIP_GUARANTEE_PCA9685);
  #endif

  private:
    const i2cip_pca9685_t _failsafe = 0x0000;
    const i2cip_pca9685_chsel_t _failsafe_args = PCA9685_NONE;

    bool initialized = false;

    #ifdef MAIN_CLASS_NAME
    friend class MAIN_CLASS_NAME;
    #endif

  protected:

    // i2cip_errorlevel_t reset(bool setbus = true);
    // i2cip_errorlevel_t sleep(bool setbus = true);
    // i2cip_errorlevel_t wakeup(bool setbus = true);

    i2cip_errorlevel_t setExtClk(uint8_t prescale = I2CIP_PCA9685_PRESCALE, bool setbus = true);
    i2cip_errorlevel_t setPWMFreq(float freq = I2CIP_PCA9685_FREQ, bool setbus = true);
    i2cip_errorlevel_t setOutputMode(bool totempole, bool setbus = true);

    i2cip_errorlevel_t setPWM(i2cip_pca9685_chsel_t num, uint16_t on, uint16_t off, bool setbus = true);
    i2cip_errorlevel_t setPin(i2cip_pca9685_chsel_t num, uint16_t val, bool invert = false, bool setbus = true);
  public:
    PCA9685(i2cip_fqa_t fqa, const i2cip_id_t& id);

    i2cip_errorlevel_t set(const i2cip_pca9685_t& value, const i2cip_pca9685_chsel_t& args) override;

    const i2cip_pca9685_chsel_t& getDefaultB(void) const override { return _failsafe_args; }
    void resetFailsafe(void) override { setValue(_failsafe); }
};

#endif