#include "BMI160Gen.h"
#include "SPI.h"
#include "Wire.h"

//#define DEBUG

bool BMI160GenClass::begin(const int spi_cs_pin, const int intr_pin)
{
    return begin(1, spi_cs_pin, intr_pin);
}
//bmi_s_array[i].bmi160.begin(1 << entry_array[i],TCAADDR,BMI160GenClass::I2C_MODE, i2c_addr);
bool BMI160GenClass::begin(int mux_entry, int mux_add, int mode, const int arg1, const int arg2)
{
    this->mode = mode;
    switch (this->mode) {
    //case INVALID_MODE:
	case -1:
        return false;
    //case I2C_MODE:
	case 1:
        i2c_addr = arg1;
        break;
    //case SPI_MODE:
	case 2:
        spi_ss = arg1;
        break;
    default:
        return false;
    }
    if (0 <= arg2) {
        interrupt_pin = digitalPinToInterrupt(arg2);
#ifdef DEBUG
        Serial.print("BMI160GenClass::begin(): pin#=");
        Serial.print(arg2);
        Serial.print(" -> interrupt=");
        Serial.println(interrupt_pin);
#endif
    }
    return CurieIMUClass::begin(mux_entry, mux_add);
}

void BMI160GenClass::attachInterrupt(void (*callback)(void))
{
    CurieIMUClass::attachInterrupt(NULL);
    if (0 <= interrupt_pin) {
        ::attachInterrupt(interrupt_pin, callback, FALLING); 
    } else {
        Serial.println("BMI160GenClass::attachInterrupt: No valid interruption pin.");
    }
}

void BMI160GenClass::ss_init()
{
    switch (this->mode) {
    //case I2C_MODE:
	case 1:
        i2c_init();
        break;
    //case SPI_MODE:
	case 2:
        spi_init();
        break;
    default:
        break;
    }
}

int BMI160GenClass::ss_xfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt)
{
    switch (this->mode) {
    //case I2C_MODE:
	case 1:
        return i2c_xfer(buf, tx_cnt, rx_cnt);
    //case SPI_MODE:
	case 2:
        if (rx_cnt) /* For read transfers, assume 1st byte contains register address */
            buf[0] |= (1 << BMI160_SPI_READ_BIT);
        return spi_xfer(buf, tx_cnt, rx_cnt);
    default:
        return 0;
    }
}

void BMI160GenClass::i2c_init()
{
#ifdef DEBUG
  Serial.println("BMI160GenClass::i2c_init()...");
#endif // DEBUG

  //Wire.begin();
  /*
  Serial.println("Selecting address ");
  Serial.print("mux_entry- ");
  Serial.print(this->mux_entry);
  Serial.print(", mux_address = ");
  Serial.print(this->mux_add);
  Serial.print(", i2c address = ");
  Serial.println(i2c_addr);
  */
  Wire.begin();
  Wire.beginTransmission(this->mux_add);
  Wire.write(this->mux_entry);
  Wire.endTransmission();
 
  Wire.begin();
  Wire.beginTransmission(i2c_addr);
  int end_status = Wire.endTransmission();
  if( end_status != 0 )
      Serial.println("BMI160GenClass::i2c_init(): I2C failed.");
  //Serial.print("end_status = ");
  //Serial.println(end_status);
#ifdef DEBUG
  int id = getDeviceID();
  Serial.print("BMI160GenClass::i2c_init(): CHIP ID = 0x");
  Serial.println(id, HEX);
  Serial.println("BMI160GenClass::i2c_init()...done");
#endif // DEBUG
}

int BMI160GenClass::i2c_xfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt)
{
  uint8_t *p;

#ifdef DEBUG
  Serial.print("i2c_xfer(offs=0x");
  Serial.print(*buf, HEX);
  Serial.print(", tx=");
  Serial.print(tx_cnt);
  Serial.print(", rx=");
  Serial.print(rx_cnt);
  Serial.print("):");
#endif // DEBUG

  Wire.begin();
  Wire.beginTransmission(this->mux_add);
  Wire.write(this->mux_entry);
  Wire.endTransmission();
  
  Wire.begin();
  Wire.beginTransmission(i2c_addr);
  
  p = buf;
  while (0 < tx_cnt) {
    tx_cnt--;
    Wire.write(*p++);
  }
  if( Wire.endTransmission() != 0 ) {
      Serial.println("Wire.endTransmission() failed.");
  }
  if (0 < rx_cnt) {
    Wire.requestFrom(i2c_addr, rx_cnt);
    p = buf;
    while ( Wire.available() && 0 < rx_cnt) {
      rx_cnt--;
#ifdef DEBUG
      int t = *p++ = Wire.read();
      Serial.print(" ");
      Serial.print(t, HEX);
#else
      *p++ = Wire.read();;
#endif // DEBUG
    }
  }

#ifdef DEBUG
  Serial.println("");
#endif // DEBUG

  return (0);
}

void BMI160GenClass::spi_init()
{
#ifdef DEBUG
  Serial.println("BMI160GenClass::spi_init()...");
#endif // DEBUG

  // start the SPI library:
  SPI.begin();
  if (0 <= spi_ss) {
    pinMode(spi_ss, OUTPUT);
  } else {
    Serial.println("BMI160GenClass::spi_init(): WARNING: No chip select pin specified.");
  }

#ifdef DEBUG
  Serial.println("BMI160GenClass::spi_init()...done");
#endif // DEBUG
}

int BMI160GenClass::spi_xfer(uint8_t *buf, unsigned tx_cnt, unsigned rx_cnt)
{
  uint8_t *p;

#ifdef DEBUG
  Serial.print("BMI160GenClass::spi_xfer(");
  Serial.print((unsigned long)buf, HEX);
  Serial.print("=");
  Serial.print(*buf, HEX);
  Serial.print(",");
  Serial.print(tx_cnt);
  Serial.print(",");
  Serial.print(rx_cnt);
  Serial.print("):");
#endif // DEBUG

  if (0 <= spi_ss)
    digitalWrite(spi_ss, LOW);
  p = buf;
  while (0 < tx_cnt) {
    tx_cnt--;
    SPI.transfer(*p++);
  }
  p = buf;
  while (0 < rx_cnt) {
    rx_cnt--;
#ifdef DEBUG
    int t = *p++ = SPI.transfer(0);
    Serial.print(" ");
    Serial.print(t, HEX);
#else
    *p++ = SPI.transfer(0);
#endif // DEBUG
  }
  if (0 <= spi_ss)
    digitalWrite(spi_ss, HIGH);

#ifdef DEBUG
  Serial.println("");
#endif // DEBUG

  return (0);
}

BMI160GenClass BMI160;
