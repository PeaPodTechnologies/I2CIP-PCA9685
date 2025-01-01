#include "PCA9685.h"

#include <debug.h>

I2CIP_DEVICE_INIT_STATIC_ID(PCA9685);
I2CIP_OUTPUT_INIT_FAILSAFE(PCA9685, i2cip_pca9685_t, 0x0000, i2cip_pca9685_chsel_t, PCA9685_NONE);

using namespace I2CIP;

PCA9685::PCA9685(i2cip_fqa_t fqa, const i2cip_id_t& id) : Device(fqa, id), OutputInterface<i2cip_pca9685_t, i2cip_pca9685_chsel_t>((Device*)this) { }

i2cip_errorlevel_t PCA9685::set(const i2cip_pca9685_t& value, const i2cip_pca9685_chsel_t& args) {
  i2cip_errorlevel_t errlev = I2CIP_ERR_NONE;
  if(args == PCA9685_NONE) return I2CIP_ERR_SOFT; // What?
  if(!this->initialized) {
    // set the default internal frequency
    // errlev = setOscillatorFrequency(I2CIP_PCA9685_OSCFREQ, false); // default args
    // I2CIP_ERR_BREAK(errlev);

    // if (prescale) {
    //   setExtClk(prescale);
    // } else {
      // set a default frequency
    errlev = setPWMFreq(I2CIP_PCA9685_FREQ, false); // default args
    I2CIP_ERR_BREAK(errlev);
    // }

    delay(I2CIP_PCA9685_DELAY);

    errlev = setOutputMode(false, false); // totempole
    I2CIP_ERR_BREAK(errlev);

    delay(I2CIP_PCA9685_DELAY);
    this->initialized = true;
  }

  // uint8_t num, uint16_t val, bool invert, bool setbus
  // return setPin(args, (value > 4096) ? (value % 4096) : value, value > 4095, false); // Easter egg invert on truncate
  errlev = setPin(args, value, false, false); // Never invert
  if(errlev != I2CIP_ERR_NONE) { this->initialized = false; }
  return errlev;
}



// i2cip_errorlevel_t PCA9685::reset(bool setbus) {
//   i2cip_errorlevel_t errlev = writeRegister(PCA9685_MODE1, MODE1_RESTART, setbus);
//   I2CIP_ERR_BREAK(errlev);
//   delay(I2CIP_PCA9685_DELAY);
//   return errlev;
// }

/*!
 *  @brief  Puts board into sleep mode
 */
// i2cip_errorlevel_t PCA9685::sleep(bool setbus) {
//   uint8_t mode = 0;
//   i2cip_errorlevel_t errlev = readRegisterByte(PCA9685_MODE1, mode, false, setbus);
//   I2CIP_ERR_BREAK(errlev);

//   mode |= MODE1_SLEEP; // set sleep bit high
//   errlev = writeRegister(PCA9685_MODE1, sleep, false);
//   I2CIP_ERR_BREAK(errlev);

//   delay(I2CIP_PCA9685_DELAY);
//   return errlev;
// }

/*!
 *  @brief  Wakes board from sleep
 */
// i2cip_errorlevel_t PCA9685::wakeup(bool setbus) {
//   uint8_t mode = 0;
//   i2cip_errorlevel_t errlev = readRegisterByte(PCA9685_MODE1, mode, false, setbus);
//   I2CIP_ERR_BREAK(errlev);

//   mode &= ~MODE1_SLEEP; // set sleep bit low
//   errlev = writeRegister(PCA9685_MODE1, sleep, false);
//   I2CIP_ERR_BREAK(errlev);

//   delay(I2CIP_PCA9685_DELAY);
//   return errlev;
// }

/*!
 *  @brief  Sets EXTCLK pin to use the external clock
 *  @param  prescale
 *          Configures the prescale value to be used by the external clock
 */
i2cip_errorlevel_t PCA9685::setExtClk(uint8_t prescale, bool setbus) {
  uint8_t mode = 0;
  i2cip_errorlevel_t errlev = readRegisterByte(PCA9685_MODE1, mode, false, setbus);
  I2CIP_ERR_BREAK(errlev);

  mode &= ~MODE1_RESTART; // clear restart bit
  mode |= MODE1_SLEEP; // set sleep bit
  errlev = writeRegister(PCA9685_MODE1, mode, false); // go to sleep, turn off internal oscillator
  I2CIP_ERR_BREAK(errlev);

  // This sets both the SLEEP and EXTCLK bits of the MODE1 register to switch to
  // use the external clock.
  mode |= MODE1_EXTCLK;
  errlev = writeRegister(PCA9685_MODE1, mode, false);
  I2CIP_ERR_BREAK(errlev);

  errlev = writeRegister(PCA9685_PRESCALE, prescale, false); // set the prescaler
  I2CIP_ERR_BREAK(errlev);

  delay(I2CIP_PCA9685_DELAY);

  mode &= ~MODE1_SLEEP; // clear sleep bit
  mode |= MODE1_RESTART | MODE1_AI; // set restart, autoincrement bits
  return writeRegister(PCA9685_MODE1, mode);
}

/*!
 *  @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
 *  @param  freq Floating point frequency that we will attempt to match
 */
i2cip_errorlevel_t PCA9685::setPWMFreq(float freq, bool setbus) {
  // Range output modulation frequency is dependant on oscillator
  freq = min(max(freq, 1.0f), 1800.0f); // constrain the frequency

  float prescaleval = ((I2CIP_PCA9685_OSCFREQ / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN) prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX) prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;

  #ifdef I2CIP_DEBUG_SERIAL
    I2CIP_DEBUG_SERIAL.print("[");
    I2CIP_DEBUG_SERIAL.print(PCA9685::getStaticID());
    I2CIP_DEBUG_SERIAL.print(F(" | RESTART] FREQ "));
    I2CIP_DEBUG_SERIAL.print(freq, DEC);
    I2CIP_DEBUG_SERIAL.print(F("hz, PRESCALE "));
    I2CIP_DEBUG_SERIAL.println(prescale, DEC);
  #endif

  uint8_t oldmode = 0;
  i2cip_errorlevel_t errlev = readRegisterByte(PCA9685_MODE1, oldmode, false, setbus);
  I2CIP_ERR_BREAK(errlev);

  errlev = writeRegister(PCA9685_MODE1, (oldmode & ~MODE1_RESTART) | MODE1_SLEEP, false); // go to sleep
  I2CIP_ERR_BREAK(errlev);

  errlev = writeRegister(PCA9685_PRESCALE, prescale, false); // set the prescaler
  I2CIP_ERR_BREAK(errlev);

  errlev = writeRegister(PCA9685_MODE1, oldmode, false);
  I2CIP_ERR_BREAK(errlev);

  delay(I2CIP_PCA9685_DELAY);

  return writeRegister(PCA9685_MODE1, (oldmode | MODE1_RESTART | MODE1_AI), false); // Wake up (autoincrement enabled)
}

/*!
 *  @brief  Sets the output mode of the PCA9685 to either
 *  open drain or push pull / totempole.
 *  Warning: LEDs with integrated zener diodes should
 *  only be driven in open drain mode.
 *  @param  totempole Totempole if true, open drain if false.
 */
i2cip_errorlevel_t PCA9685::setOutputMode(bool totempole, bool setbus) {
  uint8_t mode = 0;
  i2cip_errorlevel_t errlev = readRegisterByte(PCA9685_MODE2, mode, false, setbus);
  I2CIP_ERR_BREAK(errlev);
  if (totempole) {
    mode |= MODE2_OUTDRV;
  } else {
    mode &= ~MODE2_OUTDRV;
  }
  return writeRegister(PCA9685_MODE2, mode, false);
}

/*!
 *  @brief  Reads set Prescale from PCA9685
 *  @return prescale value
 */
// i2cip_errorlevel_t PCA9685::readPrescale(uint8_t& dest) {
//   return read8(PCA9685_PRESCALE);
// }

/*!
 *  @brief  Gets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  off If true, returns PWM OFF value, otherwise PWM ON
 *  @return requested PWM output value
 */
// i2cip_errorlevel_t PCA9685::getPWM(uint16_t& dest, uint8_t num, bool off) {
//   uint8_t buffer[2] = {uint8_t(PCA9685_LED0_ON_L + 4 * num), 0};
//   if (off)
//     buffer[0] += 2;
//   i2c_dev->write_then_read(buffer, 1, buffer, 2);
//   return uint16_t(buffer[0]) | (uint16_t(buffer[1]) << 8);
// }

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  on At what point in the 4096-part cycle to turn the PWM output ON
 *  @param  off At what point in the 4096-part cycle to turn the PWM output OFF
 *  @return 0 if successful, otherwise 1
 */
i2cip_errorlevel_t PCA9685::setPWM(i2cip_pca9685_chsel_t num, uint16_t on, uint16_t off, bool setbus) {
  // size_t len = 4; uint8_t buffer[4] = { (uint8_t)(on), (uint8_t)(on >> 8), (uint8_t)(off), (uint8_t)(off >> 8) };

  // i2cip_errorlevel_t errlev = writeRegister((uint8_t)(PCA9685_LED0_ON_L + (4 * (uint8_t)num)), (uint8_t*)buffer, len, setbus);
  // I2CIP_ERR_BREAK(errlev);
  // if(len != 4) return I2CIP_ERR_SOFT;
  // return errlev;

  i2cip_errorlevel_t errlev = writeRegister((uint8_t)(PCA9685_LED0_ON_L + (4 * num)), (uint8_t)(on & 0xFF), setbus);
  I2CIP_ERR_BREAK(errlev);
  errlev = writeRegister((uint8_t)(PCA9685_LED0_ON_H + (4 * num)), (uint8_t)(on >> 8), false);
  I2CIP_ERR_BREAK(errlev);
  errlev = writeRegister((uint8_t)(PCA9685_LED0_OFF_L + (4 * num)), (uint8_t)(off & 0xFF), false);
  I2CIP_ERR_BREAK(errlev);
  return writeRegister((uint8_t)(PCA9685_LED0_OFF_H + (4 * num)), (uint8_t)(off >> 8), false);
}

/*!
 *   @brief  Helper to set pin PWM output. Sets pin without having to deal with
 * on/off tick placement and properly handles a zero value as completely off and
 * 4095 as completely on.  Optional invert parameter supports inverting the
 * pulse for sinking to ground.
 *   @param  num One of the PWM output pins, from 0 to 15
 *   @param  val The number of ticks out of 4096 to be active, should be a value
 * from 0 to 4095 inclusive.
 *   @param  invert If true, inverts the output, defaults to 'false'
 */
i2cip_errorlevel_t PCA9685::setPin(i2cip_pca9685_chsel_t num, uint16_t val, bool invert, bool setbus) {
  #ifdef I2CIP_DEBUG_SERIAL
    I2CIP_DEBUG_SERIAL.print("[");
    I2CIP_DEBUG_SERIAL.print(PCA9685::getStaticID());
    I2CIP_DEBUG_SERIAL.print(F("] CH #"));
    I2CIP_DEBUG_SERIAL.print(num, HEX);
    I2CIP_DEBUG_SERIAL.print(F(" SET: "));
    if (val > 4095) {
      I2CIP_DEBUG_SERIAL.println(F("ON"));
    } else if (val == 0) {
      I2CIP_DEBUG_SERIAL.println(F("OFF"));
    } else {
      I2CIP_DEBUG_SERIAL.println(val);
    }
  #endif


  if (val == 0x0000) {
    // OFF
    return setPWM(num, 0, 0x1000, setbus);
  } else if (val > 0x0FFF) { // Clip
    // ON
    return setPWM(num, 0x1000, 0, setbus); // Special "Never ON"
  } else {
    return setPWM(num, 0, val, setbus);
  }
  // else {
  //   return (invert ? setPWM(num, 0, val, setbus) : setPWM(num, 0, 4095 - val, setbus));
  // }
}

/*!
 *  @brief  Sets the PWM output of one of the PCA9685 pins based on the input
 * microseconds, output is not precise
 *  @param  num One of the PWM output pins, from 0 to 15
 *  @param  Microseconds The number of Microseconds to turn the PWM output ON
 */
// i2cip_errorlevel_t PCA9685::writeMicroseconds(uint8_t num,
//                                                 uint16_t Microseconds) {
// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print("Setting PWM Via Microseconds on output");
//   Serial.print(num);
//   Serial.print(": ");
//   Serial.print(Microseconds);
//   Serial.println("->");
// #endif

//   double pulse = Microseconds;
//   double pulselength;
//   pulselength = 1000000; // 1,000,000 us per second

//   // Read prescale
//   uint16_t prescale = readPrescale();

// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print(prescale);
//   Serial.println(" PCA9685 chip prescale");
// #endif

//   // Calculate the pulse for PWM based on Equation 1 from the datasheet section
//   // 7.3.5
//   prescale += 1;
//   pulselength *= prescale;
//   pulselength /= _oscillator_freq;

// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print(pulselength);
//   Serial.println(" us per bit");
// #endif

//   pulse /= pulselength;

// #ifdef ENABLE_DEBUG_OUTPUT
//   Serial.print(pulse);
//   Serial.println(" pulse for PWM");
// #endif

//   setPWM(num, 0, pulse);
// }