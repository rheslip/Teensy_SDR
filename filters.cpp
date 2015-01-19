#include "filters.h"

short hilbert45[HILBERT_COEFFS] = {
#include "hilbert_45.h" 
};

short hilbertm45[HILBERT_COEFFS] = {
#include "hilbert_m45.h" 
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



