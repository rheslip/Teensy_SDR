#include "filters.h"

short RX_hilbert45[HILBERT_COEFFS] = {
#include "RX_hilbert_45.h" 
};

short RX_hilbertm45[HILBERT_COEFFS] = {
#include "RX_hilbert_m45.h" 
};

short TX_hilbert45[HILBERT_COEFFS] = {
#include "TX_hilbert_45.h" 
};

short TX_hilbertm45[HILBERT_COEFFS] = {
#include "TX_hilbert_m45.h" 
};

short firbpf_usb[BPF_COEFFS] = {
#include "fir_usb.h" 
};

short firbpf_lsb[BPF_COEFFS] = {
#include "fir_lsb.h" 
};

short postfir_700[COEFF_700] = {
#include "postfir_700hz.h" 
};

short postfir_lpf[COEFF_LPF] = {
#include "postfir_lpf.h" 
};



