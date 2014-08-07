/*
Copyright (c) 2009, NODA Intelligent Systems
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the {organization} nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DESCRIPTION "MegaLoad firmware flasher"
#define LONG_VERSION "1.1.0"

#define DEFAULT_DEVICE "/dev/ttyUSB0"
#define DEFAULT_BAUDRATE B19200
#define DEFAULT_TIMEOUT 20

char VERBOSE = 0;

static void usage(void)
{
	extern char *__progname;

	fprintf(stderr,
		"%s - %s, %s\n"
		"Usage: %s [-pbVhtsv] file.hex\n"
		"Options:\n"
		"  -p       Device port for communication (default: %s)\n"
		"  -b       Baud rate (default: %d)\n"
		"  -V       Print verbose progress reports\n"
		"  -h       Display this help\n"
		"  -t       How long to wait before giving up (default: %i seconds)\n"
		"  -s       Hex2char string (without 0x) to send to invoke bootloader\n"
		"  -v       Print version info and exit\n",
		__progname, DESCRIPTION, LONG_VERSION, __progname, DEFAULT_DEVICE,
		DEFAULT_BAUDRATE, DEFAULT_TIMEOUT);
}

int write_segment(const int fd, unsigned char *data, unsigned int ptr, unsigned int addrmax, int segmentsize)
{
	unsigned int checksum = 0;
	unsigned char c = 0xFF;
	unsigned int i = 0;
	unsigned int j = 0;

	if(!segmentsize) {
		return -1;
	}

	if(ptr*segmentsize <= addrmax) {
		unsigned char stream[512];
		int written = 0;

		memset(stream, 0, sizeof(stream));

		stream[0] = (ptr>>8) & 255; // Memhigh
		stream[1] = ptr & 255;      // Memlow


		for(i = ptr*segmentsize, j = 0; i < (ptr+1)*segmentsize; i++, j++) {
			stream[2+j] = data[i];
			checksum += data[i];
		}

		stream[2+j] = (unsigned char)(checksum & 255); // should be the last element

		written = write(fd, stream, (sizeof(unsigned char)*3)+segmentsize);

		if(VERBOSE) {
			printf("Wrote page #%d\n", ptr);
		}

		return written;
	}

	write(fd, &c, 1);
	write(fd, &c, 1); // Yes twice
	printf("flashing is done.\n");

	return 0;
}

/* Do not forget to add the time(NULL) thing to keep alive while writing */
int write_flash(const int fd, unsigned char *data, unsigned int addrmin, unsigned int addrmax, int memsize, const int timeout)
{
	int nfd;
	int tic = time(NULL);
	int retry = 0;
	unsigned int segmentptr = 0;

	int segmentsize = 0;
	int bootsize = 0;
	int flashsize = 0;
	char triggered = 0;

	struct pollfd pfd[1];

	if(fd) {
		pfd[0].fd = fd;
		pfd[0].events = POLLIN|POLLPRI;
	}
	else {
		return -1;
	}

	while((time(NULL)-tic) < timeout || triggered) {
		unsigned char c = 0;
		static int segmentresult = 1;

		if( segmentresult == 0) {
			break; // done!
		}

		if((nfd = poll(pfd, 1, 1000)) == -1 && errno != EINTR) {
			sleep(60);
		}

		if(pfd[0].revents & POLLHUP) {
			pfd[0].fd = -1;
			continue;
		}

		if (read( fd, &c , 1 ) && c)
		{
			switch(c) {
				case 'U':
					write(fd, &c, 1);
				break;

				case '>':
					c = '<';
					write(fd, &c, 1);
					triggered = 1;
					printf("writing flash... please wait!\n");
				break;

				case '!':
					segmentresult = write_segment(fd, data, segmentptr, addrmax, segmentsize);
					segmentptr++;
				break;

				case '@':
					retry++;
					segmentptr--;
					segmentresult = write_segment(fd, data, segmentptr, addrmax, segmentsize);
					segmentptr++;
				break;

				case '%': // Lockbits
					c = 0x00;
					write(fd, &c, 1);
					c = 0xFF;
					write(fd, &c, 1);
				break;

				default:
				break;
			}

			if(retry > 3) {
				printf("Programming failed!\n");
				c = 0xff;
				write(fd, &c, 1); 
				write(fd, &c, 1); // Yes twice!
				return 0;
			}

			switch(c) { // Segmentsize
				case 'Q': segmentsize = 32; break;
				case 'R': segmentsize = 64; break;
				case 'S': segmentsize = 128; break;
				case 'T': segmentsize = 256; break;
				case 'V': segmentsize = 512; break;
			}

			switch(c) { // Bootsize
				case 'a': bootsize = 128; break;
				case 'b': bootsize = 256; break;
				case 'c': bootsize = 512; break;
				case 'd': bootsize = 1024; break;
				case 'e': bootsize = 2048; break;
				case 'f': bootsize = 4096; break;
			}

			switch(c) { // Flashsize
				case 'g': flashsize = 1024; break;
				case 'h': flashsize = 2048; break;
				case 'i': flashsize = 4096; break;
				case 'l': flashsize = 8196; break;
				case 'm': flashsize = 16384; break;
				case 'n': flashsize = 32768; break;
				case 'o': flashsize = 65536; break;
				case 'p': flashsize = 131072; break;
				case 'q': flashsize = 262144; break;
				case 'r': flashsize = 40960; break;
			}
		}
	}

	return 0;
}

int load_flash(const char *path, unsigned char *data, unsigned int *addrmin, unsigned int *addrmax)
{
	int hfd, linenum = 0;
	char c;
	unsigned int fmin = 0xFFFF, fmax = 0x0000;
	unsigned int memsize = 0;

	if((hfd = open(path, O_RDONLY)) == -1) {
		return(-1);
	}

	while(read(hfd, &c, 1)) {
		int i, count = 0;
		char buf[256];
		unsigned char checksum = 0;
		unsigned int hdrdata = 0, memhigh = 0, memlow = 0, record = -1, memaddr = 0, memoffs = 0, uichk = 0;

		memset(buf, 0, sizeof(buf));

		if(c == ':') { // dataline
			for(i = 0; i < sizeof(buf); i++) {
				if(read(hfd, &c, 1) <= 0 || c == '\n') {
					break;
				}
				buf[i] = c;
			}
			buf[255] = '\0';

			sscanf(buf, "%2x", &hdrdata);
			if( (hdrdata*2 + 11) != strlen(buf) ) {
				printf("line length does not match byte count %i %ld\n",((char)(hdrdata)>>4)*2 + 11,strlen(buf));
				close(hfd);
				return -1;
			}

			sscanf(&buf[2],"%2x%2x%2x%n", &memhigh, &memlow, &record, &count);
			if(count != 6) {
				return -1; // error parsing file
			}

			memaddr = (memhigh*256)+memlow;

			checksum = record + hdrdata + memaddr + (memaddr >> 8);

			if(record == 0) {
				if (fmin > (memaddr + memoffs)) {
					fmin = (memaddr + memoffs);
				}
				for(i = 0; i < hdrdata; i++) {
					unsigned int d = 0;
					sscanf(&buf[8+(i*2)], "%2x", &d);
					data[memaddr + memoffs + i] = (unsigned char)(d & 255);

					checksum += data[memaddr + memoffs + i];

					if(fmax < (memaddr + memoffs + i)) {
						fmax = memaddr + memoffs + i;
					}
				}
			}
			else if(record == 1) { // protocol based end of data
				memsize = fmax - fmin + 1;
				break;
			}
			else if(record == 2 || record == 4) {
				/* Unhandled */
			}

			checksum = (256 - (checksum & 255));
			sscanf(&buf[8+(i*2)], "%2x%n", &uichk, &count);

			if(checksum != uichk) {
				return -1; // Checksum failed
			}
		}
		linenum++;
	}

	if(!memsize) {
		memsize = fmax - fmin + 1;
	}

	*addrmin = fmin;
	*addrmax = fmax;

	close(hfd);

	return memsize;
}

int main(int argc, char* argv[])
{
	extern char *__progname;

	int fd = 0, timeout = DEFAULT_TIMEOUT, ch = 0;
	unsigned int fmin, fmax;
	int memsize = 0;
	unsigned char flashdata[262144];
	char *devicepath = DEFAULT_DEVICE;
	char *flashpath = NULL;
	char hexbootstr[512];
	int baudrate = DEFAULT_BAUDRATE;

	memset(hexbootstr, 0, sizeof(hexbootstr));

	struct termios tio;

	while((ch = getopt(argc, argv, "p:b:Vht:s:v")) != -1) {
		switch (ch) {
			case 'p':
				devicepath = optarg;
				break;
			case 'b':
				sscanf(optarg, "%i", &baudrate);

				switch(baudrate) {
					case 300: baudrate = B300; break;
					case 1200: baudrate = B1200; break;
					case 2400: baudrate = B2400; break;
					case 4800: baudrate = B4800; break;
					case 9600: baudrate = B9600; break;
					case 19200: baudrate = B19200; break;
					case 38400: baudrate = B38400; break;
					case 57600: baudrate = B57600; break;
					case 115200: baudrate = B115200; break;
					case 230400: baudrate = B230400; break;
					case 460800: baudrate = B460800; break;
					case 500000: baudrate = B500000; break;
					case 576000: baudrate = B576000; break;
					case 921600: baudrate = B921600; break;
					case 1000000: baudrate = B1000000; break;
					case 1152000: baudrate = B1152000; break;
					default:
						fprintf(stderr, "unknown baudrate '%i'\n", baudrate);
						exit(1);
					break;
				}

				break;
			case 'V':
				VERBOSE = 1;
				break;
			case 'h':
				usage();
				exit(0);
			case 't':
				sscanf(optarg, "%i", &timeout);
				break;
			case 's':
				sscanf(optarg, "%s", hexbootstr);
				break;
			case 'v':
				printf("%s - %s, %s\n", __progname, DESCRIPTION, LONG_VERSION);
				exit(1);
				break;
			default:
				usage();
				exit(1);
		}
	}

	if(optind >= argc) {
		fprintf(stderr, "please supply the flash file\n");
		exit(1);
	}
	
	flashpath = argv[optind];

	if((fd = open(devicepath, O_RDWR | O_NOCTTY | O_NDELAY, 0)) == -1) { // O_NONBLOCK
		fprintf(stderr, "error opening '%s'\n", devicepath);
		exit(-1);
	}

	tcgetattr(fd, &tio);
	cfsetispeed(&tio, baudrate);
	cfsetospeed(&tio, baudrate);
	cfmakeraw(&tio);
	tcsetattr(fd, TCSAFLUSH, &tio);

	if((memsize = load_flash(flashpath, flashdata, &fmin, &fmax)) > 0) {
		/* Send reboot string */
		
		if(strlen(hexbootstr)) {
			int i;
			for(i = 0; i < strlen(hexbootstr); i+=2) {
				char hex[5], *stop, c;
				hex[0] = '0';
				hex[1] = 'x';
				hex[2] = hexbootstr[i];
				hex[3] = hexbootstr[i+1];
				hex[4] = 0;
				c = (char)strtol(hex, &stop, 16);
				write(fd, &c, 1);
			}
			
		}

		write_flash(fd, flashdata, fmin, fmax, memsize, timeout);
	}
	else {
		fprintf(stderr, "error loading flash file '%s'\n", flashpath);
	}
	close(fd);

	return 0;
}

