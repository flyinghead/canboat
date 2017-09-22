/*

Convert can-utils/candump output format to the analyzer's RAWFORMAT_PLAIN input format.

Many Linux distributions now implement SocketCAN support and further include
the can-utils for monitoring and exercising CAN bus interfaces.

Further info re: SocketCAN and the can-utils can be viewed here...

http://en.wikipedia.org/wiki/SocketCAN

This file is part of CANboat.

CANboat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CANboat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with CANboat.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <stdio.h>
#include <time.h>
#include "common.h"

#define MSG_BUF_SIZE 		2000
#define CANDUMP_DATA_INC	3
#define MAX_DATA_BYTES		223

// There are at least two variations in candump output
// format which are currently handled...
//
#define FMT_TBD			0
#define FMT_1			1	// Angstrom ex:	"<0x18eeff01> [8] 05 a0 be 1c 00 a0 a0 c0"
#define FMT_2			2	// Debian ex:	"   can0  09F8027F   [8]  00 FC FF FF 00 00 FF FF"

int main(int argc, char ** argv)
{
	char msg[MSG_BUF_SIZE];
	FILE * infile = stdin;
	FILE * outfile = stdout;

	if (argc > 1) {
	    infile = fopen(argv[1], "r");
	    if (! infile) {
		fprintf(stderr, "Could not open input file '%s' (%s)\n",
			argv[1], strerror(errno));
		return 1;
	    }
	}

	// For every line in the candump file...
	//
	int format = FMT_TBD;
	while(fgets(msg, sizeof(msg) - 1, infile))
	{
		// Ignore empty and comment lines within the candump input.
		//
		if (*msg == 0 || *msg == '\n' || *msg == '#')
    		{
		      continue;
		}

		// Process the CAN ID
		//
		unsigned int canid;
		int size;

		// Determine which candump format is being used.
		//
		if (format == FMT_TBD)
		{
			// Format not yet detected.
			// See if we can match one.
			//
			if (sscanf(msg, "<%x> [%d] ", &canid, &size) == 2) format = FMT_1;
			else if (sscanf(msg, " %*s %x [%d] ", &canid, &size) == 2) format = FMT_2;
			else continue;
		}
		else if (format == FMT_1)
		{
			if (sscanf(msg, "<%x> [%d] ", &canid, &size) != 2) continue;
		}
		else if (format == FMT_2)
		{
			if (sscanf(msg, " %*s %x [%d] ", &canid, &size) != 2) continue;
		}

		unsigned int pri;
		unsigned int src;
		unsigned int dst;
		unsigned int pgn;

		getISO11783BitsFromCanId(canid, &pri, &pgn, &src, &dst);

		// Get current time.
		// Note that we can't get fractional seconds from gmtime().
		// It would be more practical if candump provided a timestamp
		// capability, with this utility just performing a format
		// conversion.
		//
		time_t currentTime;
		struct tm * utc;
		time(&currentTime);
		utc = gmtime(&currentTime);

		// Output all but the data bytes.
		//
		fprintf(outfile, "%04d-%02d-%02d-%02d:%02d:%02d.000,%d,%d,%d,%d,%d",
				utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
				utc->tm_hour, utc->tm_min, utc->tm_sec,
				pri, pgn, src, dst, size);

		// Now process the data bytes.
		//
		int i;
		char *p;
		unsigned int data[MAX_DATA_BYTES];
		for (p = msg; p < msg + sizeof(msg) && *p != 0 && *p != ']'; ++p);
		if (*p == ']') {
			while (*(++p) == ' ');
			for (i = 0; i < size; i++, p += CANDUMP_DATA_INC)
			{
				sscanf(p, "%2x", &data[i]);
				fprintf(outfile, ",%02x", data[i]);
			}
		}
		fprintf(outfile, "\n");
		fflush(outfile);
	}
}

