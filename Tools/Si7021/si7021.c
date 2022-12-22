/*
* Si7021 Temperature Sensor Driver 
* Based on https://github.com/Csatacsibe/Si7021_driver_STM32/blob/master/driver
*
 */

#include "i2c_driver.h"
#include "si7021.h"



static const uint16_t I2C_ADDR = (0x40);           // Si7021 I2C address
static const uint8_t  HEATER_CURRENT_OFFSET = 3;      // current value in mA for register value 0
static const uint8_t  HEATER_CURRENT_STEP   = 6;      // mA/LSB

static uint8_t user_register_1 = 0b00111010;
static uint8_t heater_control_register = 0b00000000;

static float process_temp_code(uint16_t temp_code);
static float process_humi_code(uint16_t humi_code);
static uint16_t convert_to_uint16(uint8_t bytes[]);
static int8_t w_reg(uint8_t value, Si7021_registers_t reg);
static int8_t r_reg(Si7021_registers_t reg);

void init_Si7021()
{
    initI2C();
}

static float process_temp_code(uint16_t temp_code)
{
  return (float)(((175.72 * temp_code) / 65536.0) - 46.85);
}

static float process_humi_code(uint16_t humi_code)
{
  float value = (float)(((125.0 * humi_code) / 65536.0) - 6.0);

  if(value < 0)
    return 0;
  else if(value > 100)
    return 100;
  else
    return (float)value;
}

static uint16_t convert_to_uint16(uint8_t bytes[])
{
  return (uint16_t)((bytes[0]<<8) | bytes[1]);
}

static int8_t r_reg(Si7021_registers_t reg)
{
  uint8_t cmd;
  uint8_t* data;

  if(reg == User_Register_1)
  {
    cmd = R_RHT_U_reg;
    data = &(user_register_1);
  }
  else if(reg == Heater_Control_Register)
  {
    cmd = R_Heater_C_reg;
    data = &(heater_control_register);
  }
  else
    return -1;

  readI2C(I2C_ADDR, cmd, data, 1);

  return 0;
}

static int8_t w_reg(uint8_t value, Si7021_registers_t reg)
{
  uint8_t cmd;


  if(reg == User_Register_1)
  {
    cmd = W_RHT_U_reg;
  }
  else if(reg == Heater_Control_Register)
  {
    cmd = W_Heater_C_reg;
  }
  else
    return -1;

  writeI2C(I2C_ADDR, cmd, &value, 1);

  return 0;

}

int8_t r_single_Si7021(float* data, Si7021_measurement_type_t type)
{
  uint8_t cmd;
  uint8_t buffer[2];
  uint16_t code;

  if(type == Humidity)
    cmd = Humi_HM;
  else if(type == Temperature)
    cmd = Temp_HM;
  else
    return -1;

  readBurstI2C(I2C_ADDR, cmd, buffer, 2);


  code = convert_to_uint16(buffer);

  if(type == Humidity)
    *data = process_humi_code(code);
  else if(type == Temperature)
    *data = process_temp_code(code);

  return 0;
}

int8_t r_both_Si7021(float* humidity, float* temperature)
{
  uint8_t cmd = Humi_HM;
  uint8_t buffer[2];
  uint16_t code;

 readBurstI2C(I2C_ADDR, cmd, buffer, 2);


  code = convert_to_uint16(buffer);
  *humidity = process_humi_code(code);

  /* There is a temperature measurement with each RH measurement */
  cmd = Temp_HM;

 readBurstI2C(I2C_ADDR, cmd, buffer, 2);

 code = convert_to_uint16(buffer);
   *temperature = process_temp_code(code);

  return 0;
}


int8_t set_resolution_Si7021(Si7021_resolution_t resolution)
{
  int8_t rv;
  uint8_t temp = user_register_1;

  switch(resolution)
  {
    case H12_T14:
    {
      user_register_1 &= (uint8_t)(~(1<<RES1) & ~(1<<RES0));
      rv = w_reg(user_register_1, User_Register_1);
      break;
    }
    case H8_T12:
    {
      user_register_1 &= (uint8_t)(~(1<<RES1));
      user_register_1 |= (1<<RES0);
      rv = w_reg(user_register_1, User_Register_1);
      break;
    }
    case H10_T13:
    {
      user_register_1 &= ~(1<<RES0);
      user_register_1 |= (1<<RES1);
      rv = w_reg(user_register_1, User_Register_1);
      break;
    }
    case H11_T11:
    {
      user_register_1 |= (1<<RES1) | (1<<RES0);
      rv = w_reg(user_register_1, User_Register_1);
      break;
    }
    default: return -1;
  }

  /* in case of write error restore local copy of the register value */
  if(rv < 0)
    user_register_1 = temp;

  return rv;
}

Si7021_resolution_t r_resolution_Si7021()
{
  if(r_reg(User_Register_1) < 0)
    return -1;

  return (user_register_1 & ((1<<RES1) | (1<<RES0)));
}

int8_t set_heater_current_Si7021(uint8_t current)
{
  uint8_t reg_val = (current - HEATER_CURRENT_OFFSET)/HEATER_CURRENT_STEP;

  if(reg_val > 0x0F)
    reg_val = 0x0F;

  if(w_reg(reg_val, Heater_Control_Register) < 0)
    return -1;

  /* in case of write success update local copy of the register value */
  heater_control_register = reg_val;

  return 0;
}

int8_t r_heater_current_Si7021()
{
  if(r_reg(Heater_Control_Register) < 0)
    return -1;

  return ((heater_control_register & (0x0F)) * HEATER_CURRENT_STEP) + HEATER_CURRENT_OFFSET;
}

int8_t VDD_warning_Si7021()
{
  if(r_reg(User_Register_1) < 0)
    return -1;

  if(user_register_1 & (1<<VDDS))
    return 1;
  else
    return 0;
}

int8_t enable_heater_Si7021(uint8_t val)
{
  int8_t rv;
  uint8_t temp = user_register_1;

  if(val == 0)
  {
    user_register_1 &= ~(1<<HTRE);
    rv = w_reg(user_register_1, User_Register_1);
  }
  else
  {
    user_register_1 |= (1<<HTRE);
    rv = w_reg(user_register_1, User_Register_1);
  }

  /* in case of write error restore local copy of the register value */
  if(rv < 0)
    user_register_1 = temp;

  return rv;
}

int8_t rst_Si7021()
{
  uint8_t cmd = Si7021_Reset;

  writeI2C(I2C_ADDR, cmd, 0, 0);
  return 0;
}

int8_t get_register(Si7021_registers_t reg, uint8_t* rv)
{
  if(r_reg(reg) < 0)
    return -1;

  if(reg == User_Register_1)
    *rv = user_register_1;
  else
    *rv = heater_control_register;

  return 0;
}