
#include <Wire.h>
#include <BMI160Gen.h>

#define NUM_GYROS (5)
#define TCAADDR 0x70
#define BMI160_RA_GYRO_X_L          0x0C
#define interrupt_lock() (0)
#define interrupt_unlock(flags) while (0) {}


typedef struct BMI_s {
  BMI160GenClass bmi160;
  float gyroXangle;
  float gyroYangle;
  float gyroZangle;
  float CFangleX;
  float CFangleY;
  float CFangleZ;
} BMI;

BMI bmi_s_0;
BMI bmi_s_1;
BMI bmi_s_2;
BMI bmi_s_3;
BMI bmi_s_4;

BMI bmi_s_array[] = {bmi_s_0, bmi_s_1, bmi_s_2, bmi_s_3, bmi_s_4};
int entry_array[] = {2, 3, 5, 6, 7};

byte oppos_num_bit[]     = {7, 6, 5, 4, 3, 2, 1, 0};
byte shifted_num_bit[]     = {0 << 3, 1 << 3, 2 << 3, 3 << 3, 4 << 3, 5 << 3, 6 << 3, 7 << 3};
byte shifted_oppos_num_bit[]     = {7 << 3, 6 << 3, 5 << 3, 4 << 3, 3 << 3, 2 << 3, 1 << 3, 0};
byte end_num_bit[]       = {0 << 6, 1 << 6, 0 << 6, 1 << 6, 2 << 6, 3 << 6, 2 << 6, 3 << 6};
byte oppos_end_num_bit[] = {3 << 6, 2 << 6, 3 << 6, 2 << 6, 1 << 6, 0 << 6, 1 << 6, 0};


const int i2c_addr = 0x69;

float G_GAIN = 0.0076;
float DT = 0.00001; //20ms, baud = 304bits/DT     b = 38400, dt = 0.02
float AA = 0.99;

int g_curr_bmi160_index = 0;
byte g_i = 0;


byte float_to_byte(float num)
{
  float abs_num = (num >=0) ? num : -num;
  byte b_abs_num = round(abs_num*100);
  byte ret = b_abs_num & 0x7F;
  if (num < 0)
  {
    ret = (ret | 0x80); 
  }
  return ret;
}

byte* float_to_ulong(float num, byte* outArray)
{
  float abs_num = (num >=0) ? num : -num;
  unsigned long b_abs_num = round(abs_num*100000000);
  unsigned long toSend = b_abs_num & 0x7FFFFFFF;
  if (num < 0)
  {
    toSend = (toSend | 0x80000000); 
  }

  outArray[0] = (toSend & 0xFF000000) >> 24;
  outArray[1] = (toSend & 0xFF0000) >> 16;
  outArray[2] = (toSend & 0xFF00) >> 8;
  outArray[3] = (toSend & 0xFF);

  return outArray;
}


void setup(void) 
{
  unsigned long startTime = millis();
  Serial.begin(115200);
  
  for (int i=0; i<NUM_GYROS; i++)
  {
    bmi_s_array[i].bmi160.begin(1 << entry_array[i],TCAADDR,BMI160GenClass::I2C_MODE, i2c_addr);
    bmi_s_array[i].gyroXangle = 0;
    bmi_s_array[i].gyroYangle = 0;
    bmi_s_array[i].gyroZangle = 0;
    bmi_s_array[i].CFangleX = 0;
    bmi_s_array[i].CFangleY = 0;
    bmi_s_array[i].CFangleZ = 0;
    bmi_s_array[i].bmi160.autoCalibrateGyroOffset();
  }
  while(Serial.available() >0)
  {
    Serial.read();
  }
  while(millis() - startTime < 5000)
  {
  }
}



void loop(void) 
{
  int ax, ay, az, gx, gy, gz;         // raw values
  unsigned long startTime = millis();

  bmi_s_array[g_curr_bmi160_index].bmi160.readMotionSensor(ax, ay, az, gx, gy, gz);
  
  // angular velocity
  float rateX = gx*G_GAIN;
  float rateY = gy*G_GAIN;
  float rateZ = gz*G_GAIN;

  float accXangle = (atan2(ay,az)+PI)*RAD_TO_DEG;
  float accYangle = (atan2(az,ax)+PI)*RAD_TO_DEG;
  float accZangle = (atan2(ax,ay)+PI)*RAD_TO_DEG;

  if(accXangle>180) accXangle-=360;
  if(accYangle>180) accYangle-=360;
  if(accZangle>180) accZangle-=360;

  /*
  //no acc
  bmi_s_array[g_curr_bmi160_index].CFangleX = bmi_s_array[g_curr_bmi160_index].CFangleX + rateX*DT*NUM_GYROS;
  bmi_s_array[g_curr_bmi160_index].CFangleY = bmi_s_array[g_curr_bmi160_index].CFangleY + rateY*DT*NUM_GYROS;
  bmi_s_array[g_curr_bmi160_index].CFangleZ = bmi_s_array[g_curr_bmi160_index].CFangleZ + rateZ*DT*NUM_GYROS;
  */

  // original
  bmi_s_array[g_curr_bmi160_index].CFangleX = AA*(bmi_s_array[g_curr_bmi160_index].CFangleX + rateX*DT*NUM_GYROS) + (1-AA)*accXangle;
  bmi_s_array[g_curr_bmi160_index].CFangleY = AA*(bmi_s_array[g_curr_bmi160_index].CFangleY + rateY*DT*NUM_GYROS) + (1-AA)*accYangle;
  bmi_s_array[g_curr_bmi160_index].CFangleZ = AA*(bmi_s_array[g_curr_bmi160_index].CFangleZ + rateZ*DT*NUM_GYROS) + (1-AA)*accZangle;

  /* neg
  float cos_yaw   = cos(bmi_s_array[g_curr_bmi160_index].CFangleZ*DEG_TO_RAD);
  float sin_yaw   = sin(bmi_s_array[g_curr_bmi160_index].CFangleZ*DEG_TO_RAD);
  float cos_roll  = cos(bmi_s_array[g_curr_bmi160_index].CFangleY*DEG_TO_RAD);
  float sin_roll  = sin(bmi_s_array[g_curr_bmi160_index].CFangleY*DEG_TO_RAD);
  float sin_pitch = sin(bmi_s_array[g_curr_bmi160_index].CFangleX*DEG_TO_RAD);
  float cos_pitch = cos(bmi_s_array[g_curr_bmi160_index].CFangleX*DEG_TO_RAD);
  */

  byte delta = millis() - startTime;

  float cos_yaw = cos(bmi_s_array[g_curr_bmi160_index].CFangleZ*DEG_TO_RAD);
  float cos_roll = cos(bmi_s_array[g_curr_bmi160_index].CFangleY*DEG_TO_RAD);
  float cos_pitch = cos(bmi_s_array[g_curr_bmi160_index].CFangleX*DEG_TO_RAD);
  float sin_yaw = sin(bmi_s_array[g_curr_bmi160_index].CFangleZ*DEG_TO_RAD);
  float sin_roll = sin(bmi_s_array[g_curr_bmi160_index].CFangleY*DEG_TO_RAD);
  float sin_pitch = sin(bmi_s_array[g_curr_bmi160_index].CFangleX*DEG_TO_RAD);
  
  byte arr[4];
  byte start_seq = entry_array[g_curr_bmi160_index] | shifted_oppos_num_bit[entry_array[g_curr_bmi160_index]] | end_num_bit[entry_array[g_curr_bmi160_index]];
  byte end_seq = oppos_num_bit[entry_array[g_curr_bmi160_index]] | shifted_num_bit[entry_array[g_curr_bmi160_index]] | oppos_end_num_bit[entry_array[g_curr_bmi160_index]];
  //Serial.print(start_seq);
  //Serial.print('\t');
  //Serial.println(end_seq);
  //byte start_seq = g_curr_bmi160_index | shifted_oppos_num_bit[g_curr_bmi160_index] | end_num_bit[g_curr_bmi160_index];
  //byte end_seq = oppos_num_bit[g_curr_bmi160_index] | shifted_num_bit[g_curr_bmi160_index] | oppos_end_num_bit[g_curr_bmi160_index];
  
  Serial.flush();
  Serial.write(start_seq);
  
  Serial.write(float_to_ulong(cos_yaw*cos_pitch,arr), 4);
  Serial.write(float_to_ulong(cos_yaw*sin_pitch,arr), 4);
  Serial.write(float_to_ulong(-sin_yaw,arr), 4);

  Serial.write(float_to_ulong(sin_roll*sin_yaw*cos_pitch-cos_roll*sin_pitch,arr), 4);
  Serial.write(float_to_ulong(sin_roll*sin_yaw*sin_pitch+cos_roll*cos_pitch,arr), 4);
  Serial.write(float_to_ulong(sin_roll*cos_yaw,arr), 4);

  Serial.write(float_to_ulong(cos_roll*sin_yaw*cos_pitch+sin_roll*sin_pitch,arr), 4);
  Serial.write(float_to_ulong(cos_roll*sin_yaw*sin_pitch-sin_roll*cos_pitch,arr), 4);
  Serial.write(float_to_ulong(cos_roll*cos_yaw,arr), 4);
  
  Serial.write(end_seq);

  g_curr_bmi160_index = (g_curr_bmi160_index + 1) % NUM_GYROS;
  
  while(millis() - startTime < DT*1000)
  {
  }
  
}
