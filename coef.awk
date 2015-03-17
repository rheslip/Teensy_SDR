# awk script to convert FIR filter coefficients to q15 format as used in the Teensy Audio Library
# R Heslip Feb 12 /14

BEGIN {

}

{

if ($1 != "") {
	NRECS = NRECS +1

	print "32768 * " $1,","
	}

}

END {
# calculate summary and print 
#	printf "\n"
#	printf "%ld Coefficients processed\n", NRECS

}
