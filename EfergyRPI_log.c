
/*---------------------------------------------------------------------

EFERGY E2 CLASSIC RTL-SDR DECODER via rtl_fm

Copyright 2013 Nathaniel Elijah

Permission is hereby granted to use this Software for any purpose
including combining with commercial products, creating derivative
works, and redistribution of source or binary code, without
limitation or consideration. Any redistributed copies of this
Software must include the above Copyright Notice.

THIS SOFTWARE IS PROVIDED "AS IS". THE AUTHOR OF THIS CODE MAKES NO
WARRANTIES REGARDING THIS SOFTWARE, EXPRESS OR IMPLIED, AS TO ITS
SUITABILITY OR FITNESS FOR A PARTICULAR PURPOSE.

Compile:

gcc -lm -o EfergyRPI_001 EfergyRPI_001.c

Execute using the following parameters:

rtl_fm -f 433550000 -s 200000 -r 96000 -g 19.7 2>/dev/null | ./EfergyRPI_001  
--------------------------------------------------------------------- */
// Trivial Modifications for Data Logging by Gough (me@goughlui.com)
//
// Suggested compilation command is:
// gcc -O3 -o EfergyRPI_001 EfergyRPI_001.c -lm
// Note: -lm coming last avoids error from compiler which complains of
// undefined reference to function `pow'. The addition of -O3 turns on
// compiler optimizations which may or may not improve performance.
//
// Logging filename defined by additional command line argument.
//
// Added logging to file by append, with adjustable "samples to flush".
// This can avoid SSD wear while also avoiding too much lost data in case
// of power loss or crash.
//
// Also have ability to change line endings to DOS \r\n format.
//
// Consider changing the Voltage to match your local line voltage for 
// best results.
//
// Poor signals seem to cause rtl_fm and this code to consume CPU and slow
// down. Best results for me seem to be with the E4000 tuner. My R820T
// requires higher gain but sees some decode issues. Sometimes it stops
// altogether, but that's because rtl_fm seems to starve in providing
// samples.
//
// Should build and run on anything where rtl-fm works. You'll need to grab
// build, and install rtl-sdr from http://sdr.osmocom.org/trac/wiki/rtl-sdr
// first.
//
// Execute similarly
// rtl_fm -f 433550000 -s 200000 -r 96000 -g 19.7 2>/dev/null | ./EfergyRPI_log
// but play with the value for gain (in this case, 19.7) to achieve best result.
// -------------------------------------------------------------------------------------------------
//
// 08/12/2014 - Some debugging and sample analysis code added by github user magellannh
//
// Bug Fix -	Changed frame bytearray to unsigned char and added cast on  byte used in pow() function
//	to explicitly make it signed char.  Default signed/unsigned for char is compiler dependent
//	and this resulted in various problems.
// 
// New Feature  - Added frame analysis feature that dumps debug information to help characterize the FSK
//	sample data received by rtl_fm.  The feature is invoked using a "-a" option followed by
//	an optional verbosity level (0 to 3).  The output is sent to stdout. 
//
// Usage Examples:
//
//	rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -a 0
//		This mode shows the least information which is just the best guess at the decoded frame and a KW calculation
//		using bytes 4, 5, and 6.  The checksum is computed and displayed, but not validated.
//	rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -a 1
//		This  mode shows average plus and minus sample values and centering which can help with finding the best frequency. 
//		Adjust frequency to get wave center close to  0 .  If center is too high, lower frequency, otherwise increase it.
//	rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -a 2
//		This mode outputs a summary with counts of consecutive positive or negative samples.  These consecutive pulse counts
//		are what the main code uses to decode 0 and 1 data bits.  Seeing the actual pulse counts can help debug decode issues.
//	rtl_fm -f 433.51e6 -s 200000 -r 96000 -A fast  | ./EfergyRPI_log -a 3
//		This mode shows everything in modes 0..2 plus a raw dump of the sample data received from rtl_fm.
//
//	*Notice the "-A fast" option on  rtl_fm.  This cut Raspberry Pi cpu load from 50% to 25% and decode still worked fine.
//	Also, with an R820T USB dongle, leaving  rtl_fm gain in 'auto' mode  produced the best results.
//
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include <stdlib.h> // For exit function
#include <string.h>

// Standard definitions for  Efergy E2 classic decoding
#define MINLOWBIT 		3 	/* Min number of positive samples for a logic 0 */
#define MINHIGHBIT 		8	/* Min number of positive samples for a logic 1 */
#define VOLTAGE			240	/* Refernce Volatage */

// Alternate  definitions for the Efergy Elite 3.0 TPM (if used, comment out duplicates above)
//#define MINLOWBIT 		3 	/* Min number of positive samples for a logic 0 */
//#define MINHIGHBIT 		9	/* Min number of positive samples for a logic 1 */
//#define VOLTAGE			1	/* For Efergy Elite 3.0 TPM,  set to 1 */

#define E2BYTECOUNT		8	/* Efergy RF Message Byte Count */
#define PREAMBLE_COUNT		40	/* Number of positive samples for a valid preamble */
#define CENTERSAMP		100	/* Number of samples needed to compute for the wave center */
#define FRAMEBITCOUNT	(E2BYTECOUNT*8)	/* Number of bits for the entire frame (not including preamble) */

#define LOGTYPE			1	// Allows changing line-endings - 0 is for Unix /n, 1 for Windows /r/n
#define SAMPLES_TO_FLUSH	10	// Number of samples taken before writing to file.
					// Setting this too low will cause excessive wear to flash due to updates to
					// filesystem! You have been warned! Set to 10 samples for 6 seconds = every min.
								
int loggingok;		// Global var indicating logging on or off
int samplecount;	// Global var counter for samples taken since last flush
FILE *fp;	 		// Global var file handle

// This next part is for debug/analysis mode  which can be used to figure out frame formats and to tune frequency.
// Debug/analysis mode runs as a completely separate loop instead of the standard decode loop.  In this mode, all samples received
// from rtl_fm between a detected Efergy preamble and end of frame are saved off and then post-processed.  The size of the
// sample store is an estimate with some padding to (hopefully) comfortably store a full frame and then some.
// 
// It seems that with most Efergy formats, each data bit is encoded  using some combination of about 18-20 rtl_fm samples.
// zero bits are usually received as 10-13 negative samples followed by 4-7 positive samples, while 0 bits
// come in as 4-7 negative samples followed by 10-13 positive samples ( these #s may have wide tolerences)
// If the signal has excessive noise, it's theoretically possible to fill up this storage and still have more data coming in.
// The code handles this overflow by not storing any more.  When this happens, it usually means the sample data is  junk to begin with.
// 
// To skip over noise frames, analysis mode checks for a sequence with both a positive and negative preamble back to back (can be pos-neg or neg-pos)
//  From empirical testing, the preamble is usually about 180 negative samples followed by 45-50 positive samples.
// 
// At some frequencies the sign of the sample data becomes perfectly inverted.  When this happens,  frame data can still be decoded by keying off
// negative pulse sequences instead of positive pulse sequences.  This inversion condition can be detected by looking at the sign of the first samples
// received after the preamble.  If the first set of samples is negative, the data can get decoded from the positive sample pulses.  If the first set of
//samples after the preamble is  greater than 0 then the data probably can be found by decoding negative samples using the same rules that are usually
// used when decoding with the positive samples.  The analysis code automatically checks for this 'inverted' signal condition and decodes from either
// the positive or negative pulse stream depending on that check.
//
// NOTE THAT THE OBSERVATIONS ABOVE ARE FROM AN EFERGY ELITE 3.0 TPM MONITOR.  Other devices probably behave differently
//
#define MIN_POSITIVE_PREAMBLE_SAMPLES	40 	/* Number of positive samples in  an efergy  preamble */
#define MIN_NEGATIVE_PREAMBLE_SAMPLES	40 	/* Number of negative samples for a valid preamble - set to 0 if not needed/used for particular device */
#define ANALYZEBYTECOUNT		9	/* Attempt to decode up to this many bytes.  E2 Classic is 8 bytes.  Elite 3.0 TPM is 9 bytes */
#define ANALYZEBITCOUNT	(ANALYZEBYTECOUNT*8)	/* Number of bits for the entire frame (not including preamble) */
#define SAMPLES_PER_BIT			19
#define SAMPLE_STORE_SIZE		(ANALYZEBITCOUNT*SAMPLES_PER_BIT)	
int sample_storage[SAMPLE_STORE_SIZE];			
int sample_store_index;
int sample_store_overrun;
long analysis_wavecenter;	//  In analysis mode, center is defined as global so it can be changed in  the debug/analysis code.

int decode_bytes_from_pulse_counts(int pulse_store[], int pulse_store_index, unsigned char bytes[]) {
	int i;
	int dbit=0;
	int bitpos=0;
	unsigned char bytedata=0;
	int bytecount=0;
	
	for (i=0;i<ANALYZEBYTECOUNT;i++)
		bytes[i]=0;
		
	for (i=0;i<pulse_store_index;i++) {
		if (pulse_store[i] > MINLOWBIT) {
			dbit++;
			bitpos++;	
			bytedata = bytedata << 1;
			if (pulse_store[i] > MINHIGHBIT)
				bytedata = bytedata | 0x1;
			if (bitpos > 7) {
				bytes[bytecount] = bytedata;
				bytedata = 0;
				bitpos = 0;
				bytecount++;
				if (bytecount == ANALYZEBYTECOUNT) {
					return bytecount;
				}
			}
		}
	}
	return bytecount;
}

void display_frame_data(char *msg, unsigned char bytes[], int bytecount) {
	int i;
	
	// Assume message size is unknown and calculate checksums up to bytes7, 8,  and 9 to see if anything looks like a match
	unsigned char tbyte=0x00;
	for(i=0;i<(bytecount-1);i++)
		tbyte += bytes[i];
	
	// Take a shot at calculating current...
	double current_adc = (bytes[4] * 256) + bytes[5];
	double result  = (VOLTAGE * current_adc) / ((double) 32768 / (double) pow(2,bytes[6]));
	printf( msg);
	for(i=0;i<bytecount;i++) 
	  printf("%02x ",bytes[i]);
	printf("chk: %02x ",tbyte);
	if (result>0 && result<5000)
	  printf(" kW: %4.3f\n", result);
	else
	  printf(" kW: <out of range>\n");
}

// Verbosity level (from 0 to 3) controls amount of debug output 
void analyze_efergy_message(int verbosity_level) {
	int i;	

	// See how balanced/centered the sample data is.  Best case is  avg_neg + avg_pos = 0
	// If abs(avg neg) is greater than avg pos, try increasing frequency
	// If avg pos is greater than abs(avg neg), try decreasing frequency
	double avg_neg=0;
	double avg_pos=0;
	int pos_count=0;
	int neg_count=0;
	for (i=0;i<sample_store_index;i++)
		if (sample_storage[i] >=0) {
			avg_pos += sample_storage[i];
			pos_count++;
		} else {
			avg_neg += sample_storage[i];
			neg_count++;
		}
	if (pos_count!=0) 
		avg_pos /= pos_count;
	if (neg_count!=0)
		avg_neg /= neg_count;
	double difference = avg_neg + ((avg_pos-avg_neg)/2);
 
 	time_t ltime; 
	char buffer[80];
	time( &ltime );
	struct tm *curtime = localtime( &ltime );
	strftime(buffer,80,"%x,%X", curtime); 
	if (verbosity_level > 0) {
		printf("\nAnalysis of rtl_fm sample data for frame received on %s\n", buffer);
		printf("     Number of Samples: %6d\n", sample_store_index);
		printf("    Avg. Sample Values: %6.0f (negative)   %6.0f (positive)\n", avg_neg, avg_pos);
		printf("           Wave Center: %6.0f (this frame) %6d (last frame)\n", difference, analysis_wavecenter);
	} else
		printf("%s ", buffer);
	analysis_wavecenter = difference; // Use the calculated wave center from this sample to process next frame
	
	if (verbosity_level==3) { // Raw Sample Dump only in highest verbosity level
		int wrap_count=0;
		printf("\nShowing raw rtl_fm sample data received between start of frame and end of frame\n");
		for(i=0;i<sample_store_index;i++) {
			printf("%6d ", sample_storage[i] - analysis_wavecenter);
			wrap_count++;
			if (wrap_count >= 16) {
				printf("\n");
				wrap_count=0;
			}
		}
		printf("\n\n");
	}

	int display_pulse_details = (verbosity_level >= 2?1:0);
	if (display_pulse_details) printf("\nPulse stream for this frame (P-Consecutive samples > center, N-Consecutive samples < center)\n");
	int wrap_count=0;
	int pulse_count=0;
	int space_count=0;
	int pulse_count_storage[SAMPLE_STORE_SIZE];
	int space_count_storage[SAMPLE_STORE_SIZE];
	int pulse_store_index=0;
	int space_store_index=0;
	int display_pulse_info=1;
	for(i=0;i<sample_store_index;i++) {
		int samplec = sample_storage[i] - analysis_wavecenter;
		if (samplec < 0) {
			if (pulse_count > 0) {
				pulse_count_storage[pulse_store_index++]=pulse_count;
				if (display_pulse_details) printf("%2dP ", pulse_count);
				wrap_count++;
			}
			pulse_count=0;
			space_count++;
		} else {
			if (space_count > 0) {
				space_count_storage[space_store_index++]=space_count;
				if (display_pulse_details) printf("%2dN ", space_count);
				wrap_count++;
			}
			space_count=0;
			pulse_count++;
		}
		if (wrap_count >= 16) {
			if (display_pulse_details) printf("\n");
			wrap_count=0;
		}
	}
	if (display_pulse_details) printf("\n\n");

	// From empirical analysis with an Elite 3.0 TPM transmitter, sometimes the data can be decoded by counting negative pulses rather than positive
	// pulses.  It seems that if the first sequence after the preamble is positive pulses, the data can be decoded by parsing using the negative pulse counts.
	// The code below tests it both ways.
	unsigned char bytearray[ANALYZEBYTECOUNT];
	int bytecount;
	if (sample_storage[2] < analysis_wavecenter) {
		bytecount=decode_bytes_from_pulse_counts(pulse_count_storage, pulse_store_index, bytearray);
		display_frame_data("Decode from positive pulses: ", bytearray, bytecount);
	} else {
		bytecount = decode_bytes_from_pulse_counts(space_count_storage, space_store_index, bytearray);
		display_frame_data("Decode from negative pulses: ", bytearray, bytecount);
	}
	
	if (verbosity_level>0) printf("\n");
}

void  run_in_analysis_mode(int verbosity_level) {
	unsigned char bytearray[ANALYZEBYTECOUNT]; // Explicitly declare as unsigned char because depending on compiler, char may be unsigned or signed
	int prvsamp;
	
	sleep(1);
	
	printf("\nEfergy Power Monitor Decoder - Running in analysis mode using verbosity level %d\n\n", verbosity_level);
	analysis_wavecenter = 0;
	
	while( !feof(stdin) ) {

		// Look for a valid Efergy Preamble sequence which we'll define as
		// a sequence of at least MIN_PEAMBLE_SIZE positive and negative or negative and positive pulses. eg 50N+50P or 50P+50N
		int negative_preamble_count=0;
		int positive_preamble_count=0;
		prvsamp = 0;
		while ( !feof(stdin) ) {	
			int cursamp  = (int16_t) (fgetc(stdin) | fgetc(stdin)<<8);			
			// Check for preamble 
			if ((prvsamp >= analysis_wavecenter) && (cursamp >= analysis_wavecenter)) {
				positive_preamble_count++;
			} else if ((prvsamp < analysis_wavecenter) && (cursamp < analysis_wavecenter)) {
				negative_preamble_count++;				
			} else if ((prvsamp >= analysis_wavecenter) && (cursamp < analysis_wavecenter)) {
				if ((positive_preamble_count > MIN_POSITIVE_PREAMBLE_SAMPLES) &&
					(negative_preamble_count > MIN_NEGATIVE_PREAMBLE_SAMPLES))
					break;
				negative_preamble_count=0;
			} else if ((prvsamp < analysis_wavecenter) && (cursamp >= analysis_wavecenter)) {
				if ((positive_preamble_count > MIN_POSITIVE_PREAMBLE_SAMPLES) &&
					(negative_preamble_count > MIN_NEGATIVE_PREAMBLE_SAMPLES))
					break;
				positive_preamble_count=0;
			}	
			prvsamp = cursamp;
		} // end of find preamble while loop
			
		sample_store_index=0;
		while( !feof(stdin) ) {
			int cursamp  = (int16_t) (fgetc(stdin) | fgetc(stdin)<<8);
			sample_storage[sample_store_index] = cursamp;
			if (sample_store_index < (SAMPLE_STORE_SIZE-1))
				sample_store_index++;
			else {
				analyze_efergy_message(verbosity_level);
				break;
			}
		} // Frame processing while 
	} // outermost while 
	
	exit(0);
}

int calculate_watts(unsigned char bytes[])
{

unsigned char tbyte;
double current_adc;
double result;
int i;

time_t ltime; 
struct tm *curtime;
char buffer[80];

	/* add all captured bytes and mask lower 8 bits */

	tbyte = 0;

	for(i=0;i<7;i++)
		tbyte += bytes[i];

	tbyte &= 0xff;

	/* if checksum matches get watt data */

	if (tbyte == bytes[7])
	{
		time( &ltime );
		curtime = localtime( &ltime );
		strftime(buffer,80,"%x,%X", curtime);

		current_adc = (bytes[4] * 256) + bytes[5];
		result	= (VOLTAGE * current_adc) / ((double) 32768 / (double) pow(2,(signed char) bytes[6]));
		printf("%s,%f\n",buffer,result);
		if(loggingok) {
		  if(LOGTYPE) {
		    fprintf(fp,"%s,%f\r\n",buffer,result);
		  } else {
		    fprintf(fp,"%s,%f\n",buffer,result);
		  }
		  samplecount++;
		  if(samplecount==SAMPLES_TO_FLUSH) {
		    samplecount=0;
		    fflush(fp);
		  }
		}
		fflush(stdout);
		return 1;
	}
	printf("Checksum Error.  Try running program using -a [1-3] to analyze sample data\n");
	return 0;
}

void  main (int argc, char**argv) 
{

char bytearray[E2BYTECOUNT+1];
char bytedata;

int prvsamp;
int hctr;
int cursamp;
int bitpos;
int bytecount;

int i;
int preamble;
int frame;
int dcenter;
int dbit;

long center;

	if ((argc==2) && (strncmp(argv[1], "-h", 2)==0)) {
	  printf("\nUsage: %s              - Normal mode\n",argv[0]);
	  printf("       %s <filename>   - Normal mode plus log samples to output file\n", argv[0]);
	  printf("       %s -a [0,1,2,3] - Run in debug/analysis mode.  Verbosity level (0-3) is optional\n",argv[0]);
	  exit(0);
	} else if ((argc==3) && (strncmp(argv[1], "-a", 2)==0)) {
	  long verbosity_level = strtol(argv[2], NULL, 0);
	  run_in_analysis_mode(verbosity_level);
	} else if ((argc==2) && (strcmp(argv[1], "-a")==0))
	  run_in_analysis_mode(2);
	else if (argc==2) {
	  fp = fopen(argv[1], "a"); // Log file opened in append mode to avoid destroying data
	  samplecount=0; // Reset sample counter
	  loggingok=1;
	  if (fp == NULL) {
	      perror("Failed to open log file!"); // Exit if file open fails
	      exit(EXIT_FAILURE);
	  }
	} else {
	  loggingok=0;
	}

	printf("Efergy E2 Classic decode \n\n");

	/* initialize variables */
	
	cursamp = 0;
	prvsamp = 0;

	bytedata = 0;
	bytecount = 0;
	hctr = 0;
	bitpos = 0;
	dbit = 0;
	preamble = 0;
	frame = 0;

	dcenter = CENTERSAMP;
	center = 0;

	while( !feof(stdin) ) 
	{

		cursamp  = (int16_t) (fgetc(stdin) | fgetc(stdin)<<8);

		/* initially capture CENTERSAMP samples for wave center computation */
		
		if (dcenter > 0)
		{
			dcenter--;
			center = center + cursamp;	/* Accumulate FSK wave data */ 

			if (dcenter == 0)
			{
				/* compute for wave center and re-initialize frame variables */

				center = (long) (center/CENTERSAMP);

				hctr  = 0;
				bytedata = 0;
				bytecount = 0;
				bitpos = 0;
				dbit = 0;
				preamble = 0;
				frame = 0;
			}

		}
		else
		{
			if ((cursamp > center) && (prvsamp < center))		/* Detect for positive edge of frame data */
				hctr = 0;
			else 
				if ((cursamp > center) && (prvsamp > center))		/* count samples at high logic */
				{
					hctr++;
					if (hctr > PREAMBLE_COUNT)	
						preamble = 1;
				}
				else 
					if (( cursamp < center) && (prvsamp > center))
					{
						/* at negative edge */

						if ((hctr > MINLOWBIT) && (frame == 1))
						{
							dbit++;
							bitpos++;	
							bytedata = bytedata << 1;
							if (hctr > MINHIGHBIT)
								bytedata = bytedata | 0x1;

							if (bitpos > 7)
							{
								bytearray[bytecount] = bytedata;
								bytedata = 0;
								bitpos = 0;

								bytecount++;

								if (bytecount == E2BYTECOUNT)
								{

									/* at this point check for checksum and calculate watt data */
									/* if there is a checksum mismatch compute for a new wave center */

									if (calculate_watts(bytearray) == 0)
										dcenter = CENTERSAMP;	/* make dcenter non-zero to trigger center resampling */
								}
							}
							
							if (dbit > FRAMEBITCOUNT)
							{	
								/* reset frame variables */

								bitpos = 0;
								bytecount = 0;
								dbit = 0;
								frame = 0;
								preamble = 0;
								bytedata = 0;
							}
						}

						hctr = 0;

					} 
					else
						hctr = 0;

			if ((hctr == 0) && (preamble == 1))
			{
				/* end of preamble, start of frame data */
				preamble = 0;
				frame = 1;
			}

		} /* dcenter */

		prvsamp = cursamp;

	} /* while */
	if(loggingok) {
	    fclose(fp); // If rtl-fm gives EOF and program terminates, close file gracefully.
	}
}

