/*
 * Si570 Library for Arduino
 *
 * GNU GPLv3 License
 *
 * Copyright Thomas Sarlandie - 2014
 * Based on previous work by Ashar Farhan
 */

#include <Arduino.h>
#include <Wire.h>

#include "Si570.h"
#include "debug.h"

Si570::Si570(uint8_t si570_address, uint32_t calibration_frequency) {
  i2c_address = si570_address;

  Wire.begin();

  // Disable internal pullups - You will need external 3.3v pullups.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);

  f_center = 0;
  frequency = 0;

  // Force Si570 to reset to initial freq
  i2c_write(135,0x01);
  delay(20);
  if (read_si570()) {
    debug("Successfully initialized Si570");

    freq_xtal = (uint64_t)calibration_frequency * getHsDiv() * getN1() / getRfReqDouble();
    status = SI570_READY;

    //debugSi570();
  }
  else {
    debug("Unable to properly initialize Si570");
    status = SI570_ERROR;
    // Use the factory default if we were unable to talk to the chip
    freq_xtal = 114285000l;
  }
}

/*
 * This constructor is used for tests. It allows us to manually set the
 * registers and then test the other functions.
 */
Si570::Si570(uint8_t registers[], uint32_t calibration_frequency) {
  for (int i = 0; i <= 6; i++) {
    dco_reg[7 + i] = registers[i];
  }
  freq_xtal = (uint64_t)calibration_frequency * getHsDiv() * getN1() / getRfReqDouble();

  f_center = 0;
  frequency = 0;
  status = SI570_READY;
}

void Si570::debugSi570() {
  debug(" --- Si570 Debug Info ---");
  debug("Crystal frequency calibrated at: %lu", freq_xtal);
  debug("Status: %i", status);
  for (int i = 7; i < 13; i++) {
    debug("Register[%i] = %02x", i, dco_reg[i]);
  }
  debug("HsDivider = %i  N1 = %i", getHsDiv(), getN1());
  debug("Reference Frequency (hex)   : %04lx%04lx", (uint32_t)(getRfReq() >> 32), (uint32_t)(getRfReq() & 0xffffffff));

  char freq_string[10];
  dtostrf(getRfReqDouble(), -8, 3, freq_string);
  debug("Reference Frequency (double): %s", freq_string);
}

uint8_t Si570::getHsDiv() {
  uint8_t hs_reg_value = this->dco_reg[7] >> 5;

  return 4 + hs_reg_value;
}

uint8_t Si570::getN1() {
  uint8_t n_reg_value = ((this->dco_reg[7] & 0x1F) << 2) + (this->dco_reg[8] >> 6);
  return n_reg_value + 1;
}

uint64_t Si570::getRfReq() {
  uint64_t dcoFrequency = 0;

  dcoFrequency = (uint64_t)(dco_reg[8] & 0x3F);
  dcoFrequency = (dcoFrequency << 8) | (uint64_t) dco_reg[9];
  dcoFrequency = (dcoFrequency << 8) | (uint64_t) dco_reg[10];
  dcoFrequency = (dcoFrequency << 8) | (uint64_t) dco_reg[11];
  dcoFrequency = (dcoFrequency << 8) | (uint64_t) dco_reg[12];

  return dcoFrequency;
}

double Si570::getRfReqDouble() {
  return ((double) getRfReq() / ((uint64_t)1 << 28));
}

unsigned long Si570::getFreqXtal() {
  return freq_xtal;
}

void Si570::i2c_write(uint8_t reg_address, uint8_t data) {
  Wire.beginTransmission(this->i2c_address);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}

int Si570::i2c_write(uint8_t reg_address, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(this->i2c_address);
  Wire.write(reg_address);
  Wire.write(data, length);

  int error = Wire.endTransmission();
  if (error != 0) {
    debug("Error writing %i bytes to register %i: %i", length, reg_address, error);
    return -1;
  }
  return length;
}

uint8_t Si570::i2c_read(uint8_t reg_address) {
  uint8_t rdata = 0xFF;
  Wire.beginTransmission(this->i2c_address);
  Wire.write(reg_address);
  Wire.beginTransmission(this->i2c_address);
  Wire.write(reg_address);
  Wire.endTransmission();
  Wire.requestFrom(this->i2c_address, (uint8_t)1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

int Si570::i2c_read(uint8_t reg_address, uint8_t *output, uint8_t length) {
  Wire.beginTransmission(this->i2c_address);
  Wire.write(reg_address);

  int error = Wire.endTransmission();
  if (error != 0) {
    debug("Error reading %i bytes from register %i. endTransmission() returned %i", length, reg_address, error);
    return 0;
  }

  int len = Wire.requestFrom(this->i2c_address,length);
  if (len != length) {
    debug("Requested %i bytes and only got %i bytes", length, len);
  }
  for (int i = 0; i < len && Wire.available(); i++)
    output[i] = Wire.read();

  return len;
}

bool Si570::read_si570(){
  // Try 3 times to read the registers
  for (int i = 0; i < 3; i++) {
    //we have to read six consecutive registers starting at register 7
    if (i2c_read(7, &(dco_reg[7]), 6) == 6) {
      return true;
    }
    debug("Error reading Si570 registers... Retrying.");
    delay(50);
  }
  return false;
}

void Si570::write_si570()
{
  int idco;

  // Freeze DCO
  idco = i2c_read(137);
  i2c_write(137, idco | 0x10 );

  i2c_write(7, &dco_reg[7], 6);

  // Unfreeze DCO
  i2c_write(137, idco & 0xEF);

  // Set new freq
  i2c_write(135,0x40);
}

void Si570::qwrite_si570()
{
  // FIXME: Datasheet recommends freezing the "M Control Word" while writing registers
  // to avoid small changes in between each write.
  // See page 24 - bit5 of register 135.

  i2c_write(7, &dco_reg[7], 6);
}

void Si570::setBitvals(void){
  //set the rfreq values for each bit of the rfreq (integral)
  bitval[28] = (this->freq_xtal) / (hs * n1);
  bitval[29] = bitval[28] << 1;
  bitval[30] = bitval[29] << 1;
  bitval[31] = bitval[30] << 1;
  bitval[32] = bitval[31] << 1;
  bitval[33] = bitval[32] << 1;
  bitval[34] = bitval[33] << 1;
  bitval[35] = bitval[34] << 1;
  bitval[36] = bitval[35] << 1;
  bitval[37] = bitval[36] << 1;

  //set the rfreq values for each bit of the rfreq (integral)
  bitval[27] = bitval[28] >> 1;
  bitval[26] = bitval[27] >> 1;
  bitval[25] = bitval[26] >> 1;
  bitval[24] = bitval[25] >> 1;
  bitval[23] = bitval[24] >> 1;
  bitval[22] = bitval[23] >> 1;
  bitval[21] = bitval[22] >> 1;
  bitval[20] = bitval[21] >> 1;
  bitval[19] = bitval[20] >> 1;
  bitval[18] = bitval[19] >> 1;
  bitval[17] = bitval[18] >> 1;
  bitval[16] = bitval[17] >> 1;
  bitval[15] = bitval[16] >> 1;
  bitval[14] = bitval[15] >> 1;
  bitval[13] = bitval[14] >> 1;
  bitval[12] = bitval[13] >> 1;
  bitval[11] = bitval[12] >> 1;
  bitval[10] = bitval[11] >> 1;
  bitval[9] = bitval[10] >> 1;
  bitval[8] = bitval[9] >> 1;
  bitval[7] = bitval[8] >> 1;
  bitval[6] = bitval[7] >> 1;
  bitval[5] = bitval[6] >> 1;
  bitval[4] = bitval[5] >> 1;
  bitval[3] = bitval[4] >> 1;
  bitval[2] = bitval[3] >> 1;
  bitval[1] = bitval[2] >> 1;
  bitval[0] = bitval[1] >> 1;
}

//select reasonable dividers for a frequency
//in order to avoid overflow, the frequency is scaled by 10
void Si570::setDividers (unsigned long f){
  unsigned int i, j;
  unsigned long f_dco;

  for (i = 2; i <= 127; i+= 2) {
    for (j = 4; j <= 11; j++){
      //skip 8 and 10 as unused
      if (j == 8 || j == 10)
        continue;
      f_dco = (f/10) * i * j;
      if (480000000L < f_dco && f_dco < 560000000L){
        if (hs != j || n1 != i){
          hs = j; n1 = i;
          setBitvals();
        }
        //f_dco = fnew/10 * n1 * hs;
        return;
      }
    }
  }
}

void Si570::setRfreq (unsigned long fnew){
  int i, bit, ireg, byte;
  unsigned long rfreq;

  //reset all the registers
  for (i = 7; i <= 12; i++)
    this->dco_reg[i] = 0;

  //set up HS
  this->dco_reg[7] = (hs - 4) << 5;
  this->dco_reg[7] = this->dco_reg[7] | ((n1 - 1) >> 2);
  this->dco_reg[8] = ((n1-1) & 0x3) << 6;

  ireg = 8; //registers go from 8 to 12 (five of them)
  bit = 5; //the bits keep walking down
  byte = 0;
  rfreq = 0;
  for (i = 37; i >= 0; i--){
    //skip if the bitvalue is set to zero, it means, we have hit the bottom of the bitval table
    if (bitval[i] == 0)
      break;

    if (fnew >= bitval[i]){
      fnew = fnew - bitval[i];
      byte = byte | (1 << bit);
    }
    //else{
    // putchar('0');
    //}

    bit--;
    if (bit < 0){
      bit = 7;
      //use OR instead of = as register[7] has N1 bits already set into it
      this->dco_reg[ireg] |= byte;
      byte = 0;
      ireg++;
    }
  }
}

Si570_Status Si570::setFrequency(unsigned long newfreq) {
  //check that we are not wasting our time here
  if (this->frequency == newfreq)
    return this->status;

  unsigned long delta_freq;
  if (newfreq > this->f_center)
    delta_freq = newfreq - this->f_center;
  else
    delta_freq = this->f_center - newfreq;

  //if the jump is small enough, we don't have to fiddle with the dividers
  if (delta_freq < 50000L) {
    setRfreq(newfreq);
    this->frequency = newfreq;
    qwrite_si570();
    this->status = SI570_SUCCESS_SMALLJUMP;
  }
  else {
    //else it is a big jump
    setDividers(newfreq);
    setRfreq(newfreq);
    this->f_center = this->frequency = newfreq;
    write_si570();
    this->status = SI570_SUCCESS_BIGJUMP;
  }
  return this->status;
}
