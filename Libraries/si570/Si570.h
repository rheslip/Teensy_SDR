typedef enum {
  SI570_ERROR = 0,
  SI570_READY,
  SI570_SUCCESS_BIGJUMP,
  SI570_SUCCESS_SMALLJUMP
} Si570_Status;

class Si570
{
public:
  /* Will initialize from the give i2c_address */
  Si570(uint8_t i2c_address, uint32_t calibration_frequency);
  /* Manually initialize with those register values -- For testing only. */
  Si570(uint8_t registers[], uint32_t calibration_frequency);

  Si570_Status setFrequency(unsigned long newfreq);
  unsigned long getFreqXtal();
  void debugSi570();

  Si570_Status status;

private:
  uint8_t i2c_address;
  uint8_t dco_reg[13];
  unsigned long bitval[38];
  unsigned long f_center;
  unsigned long frequency;
  unsigned int hs, n1;
  unsigned long freq_xtal;

  uint8_t i2c_read(uint8_t reg_address);
  int i2c_read(uint8_t reg_address, uint8_t *output, uint8_t length);

  void i2c_write(uint8_t reg_address, uint8_t data);
  int i2c_write(uint8_t reg_address, uint8_t *data, uint8_t length);

  bool read_si570();
  void write_si570();
  void qwrite_si570();

  uint8_t getHsDiv();
  uint8_t getN1();
  uint64_t getRfReq();
  double getRfReqDouble();

  void setRfreq(unsigned long fnew);
  void setDividers (unsigned long f);
  void setBitvals(void);
};
