
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



#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

#define VOLTAGE			240	/* Refernce Voltage */
#define CENTERSAMP 		100	/* Number of samples needed to compute for the wave center */
#define PREAMBLE_COUNT		40 	/* Number of high(1) samples for a valid preamble */
#define MINLOWBIT 		3 	/* Number of high(1) samples for a logic 0 */
#define MINHIGHBIT 		8	/* Number of high(1) samples for a logic 1 */
#define E2BYTECOUNT		8	/* Efergy E2 Message Byte Count */
#define FRAMEBITCOUNT		64	/* Number of bits for the entire frame (not including preamble) */


int calculate_watts(char bytes[])
{

char tbyte;
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
        	result  = (VOLTAGE * current_adc) / ((double) 32768 / (double) pow(2,bytes[6]));
        	
       		printf("%s,%f\n",buffer,result);
       /*		printf("%X %X %X %X %X %X %X %X %X %X\n",bytes[0],bytes[1],bytes[2],bytes[3],bytes[4],bytes[5],bytes[6],bytes[7],bytes[8],bytes[9]); */
		fflush(stdout);
			return 1;
	}

	//printf("Checksum Error \n");

	return 0;
}

void  main (int argc, char**argv) 
{

char bytearray[9];
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

}


