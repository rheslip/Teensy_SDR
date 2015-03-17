// Number of coefficients
#define HILBERT_COEFFS 100
extern short RX_hilbert45[];
extern short RX_hilbertm45[];
extern short TX_hilbert45[];
extern short TX_hilbertm45[];

#define BPF_COEFFS 100
extern short firbpf_usb[];
extern short firbpf_lsb[];

#define COEFF_LPF  54
extern short postfir_700[];
#define COEFF_700 120
extern short postfir_lpf[];
