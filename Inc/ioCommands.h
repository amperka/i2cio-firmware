#define TEMP_SENSOR_CALIB_VAL_ADDR 0x1FFFF7B8U

uint32_t getUID(void);

enum IOcommand {
  // Basic functions
      WHO_AM_I = 0x00
      /*
      * command     (0x00)
      * argument    no
      * answer      u32
      
      Return 32 bit unic id stm32f030f4p6
      (temperature and analog reference calibration values)
      */

    , RESET_ME
      /*
      * command     (0x01)
      * argument    no
      * answer      no
      
      Reset chip 
      */

    , CHANGE_I2C_ADDR
      /*
      * command     (0x02)
      * argument    u8
      * answer      no

      Set new I2C address on chip. Restart I2C peripheral with new slave address.
      After power off or reset, device will start with old I2C address
      */

    , SAVE_I2C_ADDR
      /*
      * command     (0x03)
      * argument    no
      * answer      no

      Save current I2C address on flash, if it was changed by 
      CHANGE_I2C_ADDR, or CHANGE_I2C_ADDR_IF_UID_OK command
      After power off or reset, device will start with new I2C address
      */

    , PORT_MODE_INPUT
      /*
      * command     (0x04)
      * argument    u16
      * answer      no

      Set input mode on virtual port 0 pins. If argument is 
      0b0000000000000101, virtual pins 0 and 2 will be set on input mode
      */

    , PORT_MODE_PULLUP
      /*
      * command     (0x05)
      * argument    u16
      * answer      no

      Set input pullup mode on virtual port 0 pins. If argument is 
      0b0000000000000101, virtual pins 0 and 2 will be set on input pullup mode
      */

    , PORT_MODE_PULLDOWN
      /*
      * command     (0x06)
      * argument    u16
      * answer      no

      Set input pulldown mode on virtual port 0 pins. If argument is 
      0b0000000000000101, virtual pins 0 and 2 will be set on input pulldown mode
      */

    , PORT_MODE_OUTPUT
      /*
      * command     (0x07)
      * argument    u16
      * answer      no

      Set output mode on virtual port 0 pins. If argument is 
      0b0000000000000101, virtual pins 0 and 2 will be set on output mode with low value
      */

    , DIGITAL_READ
      /*
      * command     (0x08)
      * argument    no
      * answer      u16

      Return digital value of virtual port 0. Answer 
      0b0000000000000101 means virtual pins 0 and 2 is high, and all other pins is low.
      Not change pin mode
      */

    , DIGITAL_WRITE_HIGH
      /*
      * command     (0x09)
      * argument    u16
      * answer      no

      Set high digital value of virtual port 0 with change pin mode to output. If argument is 
      0b0000000000000101, virtual pins 0 and 2 will be output with high value.
      All other pins value is not change.
      Change pin mode to output
      */

    , DIGITAL_WRITE_LOW
      /*
      * command     (0x0A)
      * argument    u16
      * answer      no

      Set low digital value of virtual port 0 with change pin mode to output. If argument is 
      0b0000000000000101, virtual pins 0 and 2 will be output with low value.
      All other pins value is not change.
      Change pin mode to output
      */

    , ANALOG_WRITE
      /*
      * command     (0x0B)
      * arguments   u8    - pin
                    u16   - value
      * answer      no

      Writes an analog value (PWM wave) to a pin.
      The pin will generate a steady square wave of the specified duty cycle
      Default frequency of the PWM signal 1 kHz
      Change pin mode to output
      */

    , ANALOG_READ // Считать значениие с АЦП
      /*
      * command     (0x0C)
      * arguments   u8    - pin
      * answer      u16

      Return analog values of pin (TODO - adc max value). ADC conversion is never stop,
      so value can be readed immediately
      Not change pin mode
      */

    , PWM_FREQ
      /*
      * command     (0x0D)
      * arguments   u16   - value
      * answer      no

      Set the PWM frequency on all pins at the same time. The PWM filling factor does not change.
      */

    , ADC_SPEED
      /*
      * command     (0x0E)
      * arguments   u8   - value in range 0..7
      * answer      no

      Set ADC conversion speed. Value must be in range 0..7.
      Default value - 6 (TODO - fix it. Need to filtering off and set 7 as default)
    
      0: Sampling time 1.5 ADC clock cycle
      1: Sampling time 7.5 ADC clock cycles
      2: Sampling time 13.5 ADC clock cycles
      3: Sampling time 28.5 ADC clock cycles
      4: Sampling time 41.5 ADC clock cycles
      5: Sampling time 55.5 ADC clock cycles
      6: Sampling time 71.5 ADC clock cycles
      7: Sampling time 239.5 ADC clock cycles
      */

    , GET_MASTER_READED_UID
      /*
      * command     (0x0F)
      * arguments   u32   - UID
      * answer      no

      When many I2Cadio devices have the same I2C address, I2C master can read UID of devices
      with this address (WHO_AM_I command). Only one device can send correct UID 
      (smallest UID. See I2C arbitration). To set new addres on that device, I2C master must
      send readed UID to I2C slaves with command GET_MASTER_READED_UID. If UID is belongs to slave,
      that slave device can change i2c address with the command CHANGE_I2C_ADDR_IF_UID_OK
      */

    , CHANGE_I2C_ADDR_IF_UID_OK
      /*
      * command     (0x10)
      * arguments   u8   - new I2C address
      * answer      no

      Set new I2C address on slave device, if slave recieve his UID on GET_MASTER_READED_UID command
      */

    , SAY_SLOT
      /*
      * command     (0x11)
      * arguments   no
      * answer      u32   - "slot"

      Command to identify of I2Cadio device on I2C address. If slave answer is "slot", then we can addressing it
      with UID (see GET_MASTER_READED_UID)
      */

// TODO:
    // 0x1f - Advanced ADC functions
    , ADC_LOWPASS_FILTER_ON = 0x1f  // command
      /*
      * command     (0x1F)
      * arguments   no
      * answer      no
      
      turning on ADC low pass filter
      */

    , ADC_LOWPASS_FILTER_OFF        // command
      /*
      * command     (0x1F)
      * arguments   no
      * answer      no
      
      turning of ADC low pass filter. (TODO: After start device, filter is off)
      */

//    , 
    // 0x2f -Advanced PWM functions
    , ANALOG_WRITE_U8 = 0x2f        // 1b in
    // 0x3f -Advanced Digital functions
    // 0x4f -Software interfaces
    // ... 8 groups, 32 commands each

    // etc - start at 0xdf
    , VERSION = 0xdf                // command, 4b answer
    , ACT_LED_ENABLE                 // command
    , ACT_LED_DISABLE                // command
    , ACT_LED_BLINK_WITH_COUNTER // 1b in
};

uint16_t concat2U8toU16(uint8_t highVal, uint8_t lowVal){
  uint16_t result = highVal;
  result <<= 8;
  result |= lowVal;
  return result;
}

void setAnswerBuf_16(uint8_t *answerBuf, uint16_t val){
  answerBuf[1] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[0] = val & (uint8_t)0xFF;
}

void setAnswerBuf_32(uint8_t *answerBuf, uint32_t val){
  answerBuf[3] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[2] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[1] = val & (uint8_t)0xFF;
  val >>= 8;
  answerBuf[0] = val & (uint8_t)0xFF;
}

uint32_t getBufData_32(uint8_t *dataBuf){
  uint32_t result = dataBuf[4] | (dataBuf[3]<<8) | (dataBuf[2]<<16) | (dataBuf[1]<<24);
  return result;
}

uint32_t getUID(){
  uint32_t result;
  uint32_t* calibVal = (uint32_t*)TEMP_SENSOR_CALIB_VAL_ADDR;
  result = *calibVal;
  return  result;
}