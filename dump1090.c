// dump1090, a Mode S messages decoder for RTLSDR devices.
//
// Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//  *  Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//  *  Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include "coaa.h"
#include "dump1090.h"
#include "mirsdrapi-rsp.h"
//
// ============================= Utility functions ==========================
//
void sigintHandler(int dummy) {
    MODES_NOTUSED(dummy);
    signal(SIGINT, SIG_DFL);  // reset signal handler - bit extra safety
    Modes.exit = 1;           // Signal to threads that we are done
}
//
// =============================== Terminal handling ========================
//
#ifndef _WIN32
// Get the number of rows after the terminal changes size.
int getTermRows() { 
    struct winsize w; 
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w); 
    return (w.ws_row); 
} 

// Handle resizing terminal
void sigWinchCallback() {
    signal(SIGWINCH, SIG_IGN);
    Modes.interactive_rows = getTermRows();
    interactiveShowData();
    signal(SIGWINCH, sigWinchCallback); 
}
#else 
int getTermRows() { return MODES_INTERACTIVE_ROWS;}
#endif
//
// =============================== Initialization ===========================
//
void modesInitConfig(void) {
    // Default everything to zero/NULL
    memset(&Modes, 0, sizeof(Modes));

    // Now initialise things that should not be 0/NULL to their defaults
    Modes.use_rtlsdr              = 1;
    Modes.gain                    = MODES_MAX_GAIN;
    Modes.freq                    = MODES_DEFAULT_FREQ;
    Modes.ppm_error               = MODES_DEFAULT_PPM;
    Modes.check_crc               = 1;
    Modes.net_heartbeat_rate      = MODES_NET_HEARTBEAT_RATE;
    Modes.net_output_sbs_port     = MODES_NET_OUTPUT_SBS_PORT;
    Modes.net_output_raw_port     = MODES_NET_OUTPUT_RAW_PORT;
    Modes.net_input_raw_port      = MODES_NET_INPUT_RAW_PORT;
    Modes.net_output_beast_port   = MODES_NET_OUTPUT_BEAST_PORT;
    Modes.net_input_beast_port    = MODES_NET_INPUT_BEAST_PORT;
    Modes.net_http_port           = MODES_NET_HTTP_PORT;
    Modes.interactive_rows        = getTermRows();
    Modes.interactive_delete_ttl  = MODES_INTERACTIVE_DELETE_TTL;
    Modes.interactive_display_ttl = MODES_INTERACTIVE_DISPLAY_TTL;
    Modes.fUserLat                = MODES_USER_LATITUDE_DFLT;
    Modes.fUserLon                = MODES_USER_LONGITUDE_DFLT;
}
//
//=========================================================================
//
void modesInit(void) {
    int i, q;

    pthread_mutex_init(&Modes.pDF_mutex,NULL);
    pthread_mutex_init(&Modes.data_mutex,NULL);
    pthread_cond_init(&Modes.data_cond,NULL);

    // Allocate the various buffers used by Modes
    if ( ((Modes.icao_cache = (uint32_t *) malloc(sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2)                  ) == NULL) ||
         ((Modes.pFileData  = (uint16_t *) malloc(MODES_ASYNC_BUF_SIZE)                                         ) == NULL) ||
         ((Modes.magnitude  = (uint16_t *) malloc(MODES_ASYNC_BUF_SIZE+MODES_PREAMBLE_SIZE+MODES_LONG_MSG_SIZE) ) == NULL) ||
         ((Modes.maglut     = (uint16_t *) malloc(sizeof(uint16_t) * 256 * 256)                                 ) == NULL) ||
         ((Modes.beastOut   = (char     *) malloc(MODES_RAWOUT_BUF_SIZE)                                        ) == NULL) ||
         ((Modes.rawOut     = (char     *) malloc(MODES_RAWOUT_BUF_SIZE)                                        ) == NULL) ) 
    {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }

    // Clear the buffers that have just been allocated, just in-case
    memset(Modes.icao_cache, 0,   sizeof(uint32_t) * MODES_ICAO_CACHE_LEN * 2);
    memset(Modes.pFileData,127,   MODES_ASYNC_BUF_SIZE);
    memset(Modes.magnitude,  0,   MODES_ASYNC_BUF_SIZE+MODES_PREAMBLE_SIZE+MODES_LONG_MSG_SIZE);

    // Validate the users Lat/Lon home location inputs
    if ( (Modes.fUserLat >   90.0)  // Latitude must be -90 to +90
      || (Modes.fUserLat <  -90.0)  // and 
      || (Modes.fUserLon >  360.0)  // Longitude must be -180 to +360
      || (Modes.fUserLon < -180.0) ) {
        Modes.fUserLat = Modes.fUserLon = 0.0;
    } else if (Modes.fUserLon > 180.0) { // If Longitude is +180 to +360, make it -180 to 0
        Modes.fUserLon -= 360.0;
    }
    // If both Lat and Lon are 0.0 then the users location is either invalid/not-set, or (s)he's in the 
    // Atlantic ocean off the west coast of Africa. This is unlikely to be correct. 
    // Set the user LatLon valid flag only if either Lat or Lon are non zero. Note the Greenwich meridian 
    // is at 0.0 Lon,so we must check for either fLat or fLon being non zero not both. 
    // Testing the flag at runtime will be much quicker than ((fLon != 0.0) || (fLat != 0.0))
    Modes.bUserFlags &= ~MODES_USER_LATLON_VALID;
    if ((Modes.fUserLat != 0.0) || (Modes.fUserLon != 0.0)) {
        Modes.bUserFlags |= MODES_USER_LATLON_VALID;
    }

    // Limit the maximum requested raw output size to less than one Ethernet Block 
    if (Modes.net_output_raw_size > (MODES_RAWOUT_BUF_FLUSH))
      {Modes.net_output_raw_size = MODES_RAWOUT_BUF_FLUSH;}
    if (Modes.net_output_raw_rate > (MODES_RAWOUT_BUF_RATE))
      {Modes.net_output_raw_rate = MODES_RAWOUT_BUF_RATE;}
    if (Modes.net_sndbuf_size > (MODES_NET_SNDBUF_MAX))
      {Modes.net_sndbuf_size = MODES_NET_SNDBUF_MAX;}

    // Initialise the Block Timers to something half sensible
    ftime(&Modes.stSystemTimeBlk);
    for (i = 0; i < MODES_ASYNC_BUF_NUMBER; i++)
      {Modes.stSystemTimeRTL[i] = Modes.stSystemTimeBlk;}

    // Each I and Q value varies from 0 to 255, which represents a range from -1 to +1. To get from the 
    // unsigned (0-255) range you therefore subtract 127 (or 128 or 127.5) from each I and Q, giving you 
    // a range from -127 to +128 (or -128 to +127, or -127.5 to +127.5)..
    //
    // To decode the AM signal, you need the magnitude of the waveform, which is given by sqrt((I^2)+(Q^2))
    // The most this could be is if I&Q are both 128 (or 127 or 127.5), so you could end up with a magnitude 
    // of 181.019 (or 179.605, or 180.312)
    //
    // However, in reality the magnitude of the signal should never exceed the range -1 to +1, because the 
    // values are I = rCos(w) and Q = rSin(w). Therefore the integer computed magnitude should (can?) never 
    // exceed 128 (or 127, or 127.5 or whatever)
    //
    // If we scale up the results so that they range from 0 to 65535 (16 bits) then we need to multiply 
    // by 511.99, (or 516.02 or 514). antirez's original code multiplies by 360, presumably because he's 
    // assuming the maximim calculated amplitude is 181.019, and (181.019 * 360) = 65166.
    //
    // So lets see if we can improve things by subtracting 127.5, Well in integer arithmatic we can't
    // subtract half, so, we'll double everything up and subtract one, and then compensate for the doubling 
    // in the multiplier at the end.
    //
    // If we do this we can never have I or Q equal to 0 - they can only be as small as +/- 1.
    // This gives us a minimum magnitude of root 2 (0.707), so the dynamic range becomes (1.414-255). This 
    // also affects our scaling value, which is now 65535/(255 - 1.414), or 258.433254
    //
    // The sums then become mag = 258.433254 * (sqrt((I*2-255)^2 + (Q*2-255)^2) - 1.414)
    //                   or mag = (258.433254 * sqrt((I*2-255)^2 + (Q*2-255)^2)) - 365.4798
    //
    // We also need to clip mag just incaes any rogue I/Q values somehow do have a magnitude greater than 255.
    //

    for (i = 0; i <= 255; i++) {
        for (q = 0; q <= 255; q++) {
            int mag, mag_i, mag_q;

            mag_i = (i * 2) - 255;
            mag_q = (q * 2) - 255;

            mag = (int) round((sqrt((mag_i*mag_i)+(mag_q*mag_q)) * 258.433254) - 365.4798);

            Modes.maglut[(i*256)+q] = (uint16_t) ((mag < 65535) ? mag : 65535);
        }
    }

    // Prepare error correction tables
    modesInitErrorInfo();
}
//
// =============================== RTLSDR handling ==========================
//
void modesInitRTLSDR(void) {
    int j;
    int device_count;
    char vendor[256], product[256], serial[256];

    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported RTLSDR devices found.\n");
        exit(1);
    }

    fprintf(stderr, "Found %d device(s):\n", device_count);
    for (j = 0; j < device_count; j++) {
        rtlsdr_get_device_usb_strings(j, vendor, product, serial);
        fprintf(stderr, "%d: %s, %s, SN: %s %s\n", j, vendor, product, serial,
            (j == Modes.dev_index) ? "(currently selected)" : "");
    }

    if (rtlsdr_open(&Modes.dev, Modes.dev_index) < 0) {
        fprintf(stderr, "Error opening the RTLSDR device: %s\n",
            strerror(errno));
        exit(1);
    }

    // Set gain, frequency, sample rate, and reset the device
    rtlsdr_set_tuner_gain_mode(Modes.dev,
        (Modes.gain == MODES_AUTO_GAIN) ? 0 : 1);
    if (Modes.gain != MODES_AUTO_GAIN) {
        if (Modes.gain == MODES_MAX_GAIN) {
            // Find the maximum gain available
            int numgains;
            int gains[100];

            numgains = rtlsdr_get_tuner_gains(Modes.dev, gains);
            Modes.gain = gains[numgains-1];
            fprintf(stderr, "Max available gain is: %.2f\n", Modes.gain/10.0);
        }
        rtlsdr_set_tuner_gain(Modes.dev, Modes.gain);
        fprintf(stderr, "Setting gain to: %.2f\n", Modes.gain/10.0);
    } else {
        fprintf(stderr, "Using automatic gain control.\n");
    }
    rtlsdr_set_freq_correction(Modes.dev, Modes.ppm_error);
    if (Modes.enable_agc) rtlsdr_set_agc_mode(Modes.dev, 1);
    rtlsdr_set_center_freq(Modes.dev, Modes.freq);
    rtlsdr_set_sample_rate(Modes.dev, MODES_DEFAULT_RATE);
    rtlsdr_reset_buffer(Modes.dev);
    fprintf(stderr, "Gain reported by device: %.2f\n",
        rtlsdr_get_tuner_gain(Modes.dev)/10.0);
}

#ifdef SDRPLAY

/* =============================== SDRplay handling ========================== */
int modesInitSDRplay(void) {

    mir_sdr_ErrT err;
    float ver;

    /* Check API version */
    err = mir_sdr_ApiVersion(&ver);
    if (err ||  (ver != MIR_SDR_API_VERSION)) {
            fprintf(stderr, "Incorrect API version %f\n", ver);
            return (1);
    }       

    mir_sdr_SetParam(201,1);
    mir_sdr_SetParam(202,0);

    /* Initialize SDRplay device */
    err = mir_sdr_Init (9, 8.000, 1090.048, mir_sdr_BW_1_536, mir_sdr_IF_2_048, &Modes.sdrplaySamplesPerPacket);

    if (err){
            fprintf(stderr, "Unable to initialize RSP\n");
            return (1);
    }  
    /* Allocate 16-bit I and Q buffers */

    Modes.sdrplay_i = malloc ((Modes.sdrplaySamplesPerPacket >> 2) * sizeof(short));
    Modes.sdrplay_q = malloc ((Modes.sdrplaySamplesPerPacket >> 2) * sizeof(short));
    Modes.sdrplay_ii = malloc (Modes.sdrplaySamplesPerPacket * sizeof(short));
    Modes.sdrplay_qq = malloc (Modes.sdrplaySamplesPerPacket * sizeof(short));
    Modes.sdrplay_data = malloc (MODES_ASYNC_BUF_SIZE * MODES_ASYNC_BUF_NUMBER);

    if ((Modes.sdrplay_i == NULL) || (Modes.sdrplay_q == NULL)
        || (Modes.sdrplay_ii == NULL) || (Modes.sdrplay_qq == NULL)
        || (Modes.sdrplay_data == NULL)){
            fprintf(stderr, "Insufficient memory for buffers\n");
            return (1);
    }  

    /* Configure DC tracking in tuner */
    err = mir_sdr_SetDcMode(4,0);
    err |= mir_sdr_SetDcTrackTime(63);
    if (err){
            fprintf(stderr, "Set DC tracking failed, %d\n", err);
            return (1);
    }  

    return (0);
}

#endif

//
//=========================================================================
//
// We use a thread reading data in background, while the main thread
// handles decoding and visualization of data to the user.
//
// The reading thread calls the RTLSDR API to read data asynchronously, and
// uses a callback to populate the data buffer.
//
// A Mutex is used to avoid races with the decoding thread.
//
void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx) {

    MODES_NOTUSED(ctx);

    // Lock the data buffer variables before accessing them
    pthread_mutex_lock(&Modes.data_mutex);

    Modes.iDataIn &= (MODES_ASYNC_BUF_NUMBER-1); // Just incase!!!

    // Get the system time for this block
    ftime(&Modes.stSystemTimeRTL[Modes.iDataIn]);

    if (len > MODES_ASYNC_BUF_SIZE) {len = MODES_ASYNC_BUF_SIZE;}

    // Queue the new data
    Modes.pData[Modes.iDataIn] = (uint16_t *) buf;
    Modes.iDataIn    = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn + 1);
    Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn - Modes.iDataOut);   

    if (Modes.iDataReady == 0) {
      // Ooooops. We've just received the MODES_ASYNC_BUF_NUMBER'th outstanding buffer
      // This means that RTLSDR is currently overwriting the MODES_ASYNC_BUF_NUMBER+1
      // buffer, but we havent yet processed it, so we're going to lose it. There
      // isn't much we can do to recover the lost data, but we can correct things to
      // avoid any additional problems.
      Modes.iDataOut   = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataOut+1);
      Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1);   
      Modes.iDataLost++;
    }
 
    // Signal to the other thread that new data is ready, and unlock
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
}
//
//=========================================================================
//
// This is used when --ifile is specified in order to read data from file
// instead of using an RTLSDR device
//
void readDataFromFile(void) {
    pthread_mutex_lock(&Modes.data_mutex);
    while(Modes.exit == 0) {
        ssize_t nread, toread;
        unsigned char *p;

        if (Modes.iDataReady) {
            pthread_cond_wait(&Modes.data_cond, &Modes.data_mutex);
            continue;
        }

        if (Modes.interactive) {
            // When --ifile and --interactive are used together, slow down
            // playing at the natural rate of the RTLSDR received.
            pthread_mutex_unlock(&Modes.data_mutex);
            usleep(64000);
            pthread_mutex_lock(&Modes.data_mutex);
        }

        toread = MODES_ASYNC_BUF_SIZE;
        p = (unsigned char *) Modes.pFileData;
        while(toread) {
            nread = read(Modes.fd, p, toread);
            if (nread <= 0) {
                Modes.exit = 1; // Signal the other threads to exit.
                break;
            }
            p += nread;
            toread -= nread;
        }
        if (toread) {
            // Not enough data on file to fill the buffer? Pad with no signal.
            memset(p,127,toread);
        }

        Modes.iDataIn &= (MODES_ASYNC_BUF_NUMBER-1); // Just incase!!!

        // Get the system time for this block
        ftime(&Modes.stSystemTimeRTL[Modes.iDataIn]);

        // Queue the new data
        Modes.pData[Modes.iDataIn] = Modes.pFileData;
        Modes.iDataIn    = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn + 1);
        Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn - Modes.iDataOut);   

        // Signal to the other thread that new data is ready
        pthread_cond_signal(&Modes.data_cond);
    }
}

#ifdef SDRPLAY

#define ACC_SHIFT 14					// sets time constant of averaging filter
#define MIN_GAIN_THRESH 6				// increase gain if peaks below this 
#define MAX_GAIN_THRESH 9				// decrease gain if peaks above this

/*
	Data from RSP at 8MHz, 2MHz IF, is downsampled to 2MHz, zero IF. It is reduced 
    to 8 bits and interleaved into a circular buffer. Each time the pointer passes
    a multiple of MODES_ASYNC_BUF_SIZE, that segment of buffer is handed off to the
    routine which normally receives data from the RTL device.

    For each packet from the RSP, the maximum I signal value is recorded. This is
    entered into a slow, exponentially decaying filter. The output from this filter
    is occasionally checked and a decision made whether to step the RSP gain by
    plus or minus 1dB.
*/


int sdrplay_start_rx(void) {

    unsigned int firstSampleNum;
    int grChanged, rfChanged, fsChanged;
    mir_sdr_ErrT err = 0;
    int i, count1, count2, new_buf_flag;
    int sig_i, sig_q, max_sig, max_sig_acc = MIN_GAIN_THRESH << ACC_SHIFT;

    unsigned char *dptr = Modes.sdrplay_data;
	unsigned int end, input_index, data_index = 0;    

    while(1)
    {
		/* read real signal values at 8MHz */

        err = mir_sdr_ReadPacket (Modes.sdrplay_ii, Modes.sdrplay_qq, 
                         &firstSampleNum, &grChanged, &rfChanged, &fsChanged);
        if (err)
        {
            fprintf(stderr, "sdrplay data read failed\n");
            Modes.exit = 1; /* Signal the other thread to exit. */
            break;
		}  

		/* convert to quadrature at 2MHz */

        err = mir_sdr_DownConvert(Modes.sdrplay_ii, Modes.sdrplay_i,
                        Modes.sdrplay_q, Modes.sdrplaySamplesPerPacket,
                        mir_sdr_IF_2_048, 4, 0);
        if (err)
        {
            fprintf(stderr, "sdrplay downconvert failed\n");
            Modes.exit = 1; /* Signal the other thread to exit. */
            break;
        }

        // Assumptions; sdrplaySamplesPP/2 is smaller than MODES_ASYNC_BUF_SIZE, so either 0 or 1 buffers handed off
        //              sdrplaySamplesPP is divisible by 4

        // Think about what's going to happen, will we overrun end, will we fill a buffer?
	
		/* count1 is lesser of input samples and samples to end of buffer */
        /* count2 is the remainder, generally zero */

    	end = data_index + (Modes.sdrplaySamplesPerPacket >> 1);
        count2 = end - (MODES_ASYNC_BUF_SIZE * MODES_ASYNC_BUF_NUMBER);
        if (count2 < 0) count2 = 0; 						/* count2 is samples wrapping around to start of buf */
        count1 = (Modes.sdrplaySamplesPerPacket >> 1) - count2;   /* count1 is samples fitting before the end of buf */

		/* flag is set if this packet takes us past a multiple of MODES_ASYNC_BUF_SIZE */

        new_buf_flag = ((data_index & (MODES_ASYNC_BUF_SIZE-1)) < (end & (MODES_ASYNC_BUF_SIZE-1)))? 0 : 1;

		/* now interleave data from I/Q into circular buffer, and note max I value */

        input_index = 0;
        max_sig = 0;

        for (i = 0; i < count1 >> 1; i++)
        {
		    sig_i = (Modes.sdrplay_i[input_index] >> 8) + 127;
            if (sig_i < 0) sig_i = 0;
		    dptr[data_index++] = (unsigned char) sig_i;

		    sig_q = (Modes.sdrplay_q[input_index++] >> 8) + 127;
            if (sig_q < 0) sig_q = 0;
		    dptr[data_index++] = (unsigned char) sig_q;

            if (sig_i > max_sig) max_sig = sig_i;
        }

		/* apply slowly decaying filter to max signal value */

        max_sig -= 127;
        max_sig_acc += max_sig;
        max_sig = max_sig_acc >> ACC_SHIFT;
        max_sig_acc -= max_sig;

		/* this code is triggered as we reach the end of our circular buffer */

        if (data_index >= (MODES_ASYNC_BUF_SIZE * MODES_ASYNC_BUF_NUMBER))
        {
            data_index = 0;  // pointer back to start of buffer */

            /* adjust gain if required */
            if (max_sig > MAX_GAIN_THRESH) mir_sdr_SetGr (1, 0, 0);
            if (max_sig < MIN_GAIN_THRESH) mir_sdr_SetGr (-1, 0, 0);
        }

		/* insert any remaining signal at start of buffer */

        for (i = 0; i < count2 >> 1; i++)
        {
		    sig_i = (Modes.sdrplay_i[input_index] >> 8) + 127;
            if (sig_i < 0) sig_i = 0;
		    dptr[data_index++] = (unsigned char) sig_i;

		    sig_q = (Modes.sdrplay_q[input_index++] >> 8) + 127;
            if (sig_q < 0) sig_q = 0;
		    dptr[data_index++] = (unsigned char) sig_q;
        }

		/* send buffer downstream if enough available */

        if (new_buf_flag)
        {
			/* go back by one buffer length, then round down further to start of buffer */
            end = data_index + MODES_ASYNC_BUF_SIZE * (MODES_ASYNC_BUF_NUMBER-1);
            end &= MODES_ASYNC_BUF_SIZE * MODES_ASYNC_BUF_NUMBER - 1;
            end &= ~(MODES_ASYNC_BUF_SIZE-1);

			/* now pretend this came from an rtlsdr device */
            rtlsdrCallback(&Modes.sdrplay_data[end], MODES_ASYNC_BUF_SIZE, NULL);
		}
    }
	return 0;
}

#endif


//
//=========================================================================
//
// We read data using a thread, so the main thread only handles decoding
// without caring about data acquisition
//
void *readerThreadEntryPoint(void *arg) {
    MODES_NOTUSED(arg);

    if (Modes.filename == NULL)
    {
        if (Modes.use_rtlsdr)
            rtlsdr_read_async(Modes.dev, rtlsdrCallback, NULL,
                              MODES_ASYNC_BUF_NUMBER,
                              MODES_ASYNC_BUF_SIZE);
#ifdef SDRPLAY
        else // Modes.use_sdrplay
		{
            int status = sdrplay_start_rx();
            if (status != 0) {
                fprintf(stderr, "sdrplay_start_rx failed");
                mir_sdr_Uninit();
            }
        }
#endif
    } 
    else {
        readDataFromFile();
    }
    // Signal to the other thread that new data is ready - dummy really so threads don't mutually lock
    pthread_cond_signal(&Modes.data_cond);
    pthread_mutex_unlock(&Modes.data_mutex);
#ifndef _WIN32
    pthread_exit(NULL);
#else
    return NULL;
#endif
}
//
// ============================== Snip mode =================================
//
// Get raw IQ samples and filter everything is < than the specified level
// for more than 256 samples in order to reduce example file size
//
void snipMode(int level) {
    int i, q;
    uint64_t c = 0;

    while ((i = getchar()) != EOF && (q = getchar()) != EOF) {
        if (abs(i-127) < level && abs(q-127) < level) {
            c++;
            if (c > MODES_PREAMBLE_SIZE) continue;
        } else {
            c = 0;
        }
        putchar(i);
        putchar(q);
    }
}
//
// ================================ Main ====================================
//
void showHelp(void) {
    printf(
"-----------------------------------------------------------------------------\n"
"|                        dump1090 ModeS Receiver         Ver : " MODES_DUMP1090_VERSION " |\n"
"-----------------------------------------------------------------------------\n"
"--device-index <index>   Select RTL device (default: 0)\n"
#ifdef SDRPLAY
"--dev-sdrplay            use RSP device instead of RTL device (default: RTL).\n"
#endif
"--gain <db>              Set gain (default: max gain. Use -10 for auto-gain)\n"
"--enable-agc             Enable the Automatic Gain Control (default: off)\n"
"--freq <hz>              Set frequency (default: 1090 Mhz)\n"
"--ifile <filename>       Read data from file (use '-' for stdin)\n"
"--interactive            Interactive mode refreshing data on screen\n"
"--interactive-rows <num> Max number of rows in interactive mode (default: 15)\n"
"--interactive-ttl <sec>  Remove from list if idle for <sec> (default: 60)\n"
"--interactive-rtl1090    Display flight table in RTL1090 format\n"
"--raw                    Show only messages hex values\n"
"--net                    Enable networking\n"
"--modeac                 Enable decoding of SSR Modes 3/A & 3/C\n"
"--net-beast              TCP raw output in Beast binary format\n"
"--net-only               Enable just networking, no RTL device or file used\n"
"--net-bind-address <ip>  IP address to bind to (default: Any; Use 127.0.0.1 for private)\n"
"--net-http-port <port>   HTTP server port (default: 8080)\n"
"--net-ri-port <port>     TCP raw input listen port  (default: 30001)\n"
"--net-ro-port <port>     TCP raw output listen port (default: 30002)\n"
"--net-sbs-port <port>    TCP BaseStation output listen port (default: 30003)\n"
"--net-bi-port <port>     TCP Beast input listen port  (default: 30004)\n"
"--net-bo-port <port>     TCP Beast output listen port (default: 30005)\n"
"--net-ro-size <size>     TCP raw output minimum size (default: 0)\n"
"--net-ro-rate <rate>     TCP raw output memory flush rate (default: 0)\n"
"--net-heartbeat <rate>   TCP heartbeat rate in seconds (default: 60 sec; 0 to disable)\n"
"--net-buffer <n>         TCP buffer size 64Kb * (2^n) (default: n=0, 64Kb)\n"
"--lat <latitude>         Reference/receiver latitude for surface posn (opt)\n"
"--lon <longitude>        Reference/receiver longitude for surface posn (opt)\n"
"--fix                    Enable single-bits error correction using CRC\n"
"--no-fix                 Disable single-bits error correction using CRC\n"
"--no-crc-check           Disable messages with broken CRC (discouraged)\n"
"--phase-enhance          Enable phase enhancement\n"
"--aggressive             More CPU for more messages (two bits fixes, ...)\n"
"--mlat                   display raw messages in Beast ascii mode\n"
"--stats                  With --ifile print stats at exit. No other output\n"
"--stats-every <seconds>  Show and reset stats every <seconds> seconds\n"
"--onlyaddr               Show only ICAO addresses (testing purposes)\n"
"--metric                 Use metric units (meters, km/h, ...)\n"
"--snip <level>           Strip IQ file removing samples < level\n"
"--debug <flags>          Debug mode (verbose), see README for details\n"
"--quiet                  Disable output to stdout. Use for daemon applications\n"
"--ppm <error>            Set receiver error in parts per million (default 0)\n"
"--help                   Show this help\n"
"\n"
"Debug mode flags: d = Log frames decoded with errors\n"
"                  D = Log frames decoded with zero errors\n"
"                  c = Log frames with bad CRC\n"
"                  C = Log frames with good CRC\n"
"                  p = Log frames with bad preamble\n"
"                  n = Log network debugging info\n"
"                  j = Log frames to frames.js, loadable by debug.html\n"
    );
}

#ifdef _WIN32
void showCopyright(void) {
    uint64_t llTime = time(NULL) + 1;

    printf(
"-----------------------------------------------------------------------------\n"
"|                        dump1090 ModeS Receiver         Ver : " MODES_DUMP1090_VERSION " |\n"
"-----------------------------------------------------------------------------\n"
"\n"
" Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>\n"
" Copyright (C) 2014 by Malcolm Robb <support@attavionics.com>\n"
"\n"
" All rights reserved.\n"
"\n"
" THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n"
" ""AS IS"" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n"
" LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR\n"
" A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT\n"
" HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,\n"
" SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT\n"
" LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,\n"
" DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY\n"
" THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT\n"
" (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\n"
" OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n"
"\n"
" For further details refer to <https://github.com/MalcolmRobb/dump1090>\n" 
"\n"
    );

  // delay for 1 second to give the user a chance to read the copyright
  while (llTime >= time(NULL)) {}
}
#endif


static void display_stats(void) {
    int j;
    time_t now = time(NULL);

    printf("\n\n");
    if (Modes.interactive)
        interactiveShowData();

    printf("Statistics as at %s", ctime(&now));

    printf("%d sample blocks processed\n",                    Modes.stat_blocks_processed);
    printf("%d sample blocks dropped\n",                      Modes.stat_blocks_dropped);

    printf("%d ModeA/C detected\n",                           Modes.stat_ModeAC);
    printf("%d valid Mode-S preambles\n",                     Modes.stat_valid_preamble);
    printf("%d DF-?? fields corrected for length\n",          Modes.stat_DF_Len_Corrected);
    printf("%d DF-?? fields corrected for type\n",            Modes.stat_DF_Type_Corrected);
    printf("%d demodulated with 0 errors\n",                  Modes.stat_demodulated0);
    printf("%d demodulated with 1 error\n",                   Modes.stat_demodulated1);
    printf("%d demodulated with 2 errors\n",                  Modes.stat_demodulated2);
    printf("%d demodulated with > 2 errors\n",                Modes.stat_demodulated3);
    printf("%d with good crc\n",                              Modes.stat_goodcrc);
    printf("%d with bad crc\n",                               Modes.stat_badcrc);
    printf("%d errors corrected\n",                           Modes.stat_fixed);

    for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
        printf("   %d with %d bit %s\n", Modes.stat_bit_fix[j], j+1, (j==0)?"error":"errors");
    }

    if (Modes.phase_enhance) {
        printf("%d phase enhancement attempts\n",                 Modes.stat_out_of_phase);
        printf("%d phase enhanced demodulated with 0 errors\n",   Modes.stat_ph_demodulated0);
        printf("%d phase enhanced demodulated with 1 error\n",    Modes.stat_ph_demodulated1);
        printf("%d phase enhanced demodulated with 2 errors\n",   Modes.stat_ph_demodulated2);
        printf("%d phase enhanced demodulated with > 2 errors\n", Modes.stat_ph_demodulated3);
        printf("%d phase enhanced with good crc\n",               Modes.stat_ph_goodcrc);
        printf("%d phase enhanced with bad crc\n",                Modes.stat_ph_badcrc);
        printf("%d phase enhanced errors corrected\n",            Modes.stat_ph_fixed);

        for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
            printf("   %d with %d bit %s\n", Modes.stat_ph_bit_fix[j], j+1, (j==0)?"error":"errors");
        }
    }

    printf("%d total usable messages\n",                      Modes.stat_goodcrc + Modes.stat_ph_goodcrc + Modes.stat_fixed + Modes.stat_ph_fixed);
    fflush(stdout);

    Modes.stat_blocks_processed =
        Modes.stat_blocks_dropped = 0;

    Modes.stat_ModeAC =
        Modes.stat_valid_preamble =
        Modes.stat_DF_Len_Corrected =
        Modes.stat_DF_Type_Corrected =
        Modes.stat_demodulated0 =
        Modes.stat_demodulated1 =
        Modes.stat_demodulated2 =
        Modes.stat_demodulated3 =
        Modes.stat_goodcrc =
        Modes.stat_badcrc =
        Modes.stat_fixed = 0;

    Modes.stat_out_of_phase =
        Modes.stat_ph_demodulated0 =
        Modes.stat_ph_demodulated1 =
        Modes.stat_ph_demodulated2 =
        Modes.stat_ph_demodulated3 =
        Modes.stat_ph_goodcrc =
        Modes.stat_ph_badcrc =
        Modes.stat_ph_fixed = 0;

    for (j = 0;  j < MODES_MAX_BITERRORS;  j++) {
        Modes.stat_ph_bit_fix[j] = 0;
        Modes.stat_bit_fix[j] = 0;
    }
}


//
//=========================================================================
//
// This function is called a few times every second by main in order to
// perform tasks we need to do continuously, like accepting new clients
// from the net, refreshing the screen in interactive mode, and so forth
//
void backgroundTasks(void) {
    static time_t next_stats;

    if (Modes.net) {
        modesReadFromClients();
    }    

    // If Modes.aircrafts is not NULL, remove any stale aircraft
    if (Modes.aircrafts) {
        interactiveRemoveStaleAircrafts();
    }

    // Refresh screen when in interactive mode
    if (Modes.interactive) {
        interactiveShowData();
    }

    if (Modes.stats > 0) {
        time_t now = time(NULL);
        if (now > next_stats) {
            if (next_stats != 0)
                display_stats();
            next_stats = now + Modes.stats;
        }
    }
}
//
//=========================================================================
//
int verbose_device_search(char *s)
{
	int i, device_count, device, offset;
	char *s2;
	char vendor[256], product[256], serial[256];
	device_count = rtlsdr_get_device_count();
	if (!device_count) {
		fprintf(stderr, "No supported devices found.\n");
		return -1;
	}
	fprintf(stderr, "Found %d device(s):\n", device_count);
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		fprintf(stderr, "  %d:  %s, %s, SN: %s\n", i, vendor, product, serial);
	}
	fprintf(stderr, "\n");
	/* does string look like raw id number */
	device = (int)strtol(s, &s2, 0);
	if (s2[0] == '\0' && device >= 0 && device < device_count) {
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string exact match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strcmp(s, serial) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string prefix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		if (strncmp(s, serial, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	/* does string suffix match a serial */
	for (i = 0; i < device_count; i++) {
		rtlsdr_get_device_usb_strings(i, vendor, product, serial);
		offset = strlen(serial) - strlen(s);
		if (offset < 0) {
			continue;}
		if (strncmp(s, serial+offset, strlen(s)) != 0) {
			continue;}
		device = i;
		fprintf(stderr, "Using device %d: %s\n",
			device, rtlsdr_get_device_name((uint32_t)device));
		return device;
	}
	fprintf(stderr, "No matching devices found.\n");
	return -1;
}
//
//=========================================================================
//
int main(int argc, char **argv) {
    int j;

    // Set sane defaults
    modesInitConfig();
    signal(SIGINT, sigintHandler); // Define Ctrl/C handler (exit program)

    // Parse the command line options
    for (j = 1; j < argc; j++) {
        int more = j+1 < argc; // There are more arguments

        if (!strcmp(argv[j],"--device-index") && more) {
            Modes.dev_index = verbose_device_search(argv[++j]);
#ifdef SDRPLAY
        } else if (!strcmp(argv[j],"--dev-sdrplay")) {
            Modes.use_sdrplay = 1; Modes.use_rtlsdr = 0;
#endif
        } else if (!strcmp(argv[j],"--gain") && more) {
            Modes.gain = (int) (atof(argv[++j])*10); // Gain is in tens of DBs
        } else if (!strcmp(argv[j],"--enable-agc")) {
            Modes.enable_agc++;
        } else if (!strcmp(argv[j],"--freq") && more) {
            Modes.freq = (int) strtoll(argv[++j],NULL,10);
        } else if (!strcmp(argv[j],"--ifile") && more) {
            Modes.filename = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--fix")) {
            Modes.nfix_crc = 1;
        } else if (!strcmp(argv[j],"--no-fix")) {
            Modes.nfix_crc = 0;
        } else if (!strcmp(argv[j],"--no-crc-check")) {
            Modes.check_crc = 0;
        } else if (!strcmp(argv[j],"--phase-enhance")) {
            Modes.phase_enhance = 1;
        } else if (!strcmp(argv[j],"--raw")) {
            Modes.raw = 1;
        } else if (!strcmp(argv[j],"--net")) {
            Modes.net = 1;
        } else if (!strcmp(argv[j],"--modeac")) {
            Modes.mode_ac = 1;
        } else if (!strcmp(argv[j],"--net-beast")) {
            Modes.beast = 1;
        } else if (!strcmp(argv[j],"--net-only")) {
            Modes.net = 1;
            Modes.net_only = 1;
       } else if (!strcmp(argv[j],"--net-heartbeat") && more) {
            Modes.net_heartbeat_rate = atoi(argv[++j]) * 15;
       } else if (!strcmp(argv[j],"--net-ro-size") && more) {
            Modes.net_output_raw_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-rate") && more) {
            Modes.net_output_raw_rate = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ro-port") && more) {
            if (Modes.beast) // Required for legacy backward compatibility
                {Modes.net_output_beast_port = atoi(argv[++j]);;}
            else
                {Modes.net_output_raw_port = atoi(argv[++j]);}
        } else if (!strcmp(argv[j],"--net-ri-port") && more) {
            Modes.net_input_raw_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bo-port") && more) {
            Modes.net_output_beast_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bi-port") && more) {
            Modes.net_input_beast_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-bind-address") && more) {
            Modes.net_bind_address = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--net-http-port") && more) {
            Modes.net_http_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-sbs-port") && more) {
            Modes.net_output_sbs_port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-buffer") && more) {
            Modes.net_sndbuf_size = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--onlyaddr")) {
            Modes.onlyaddr = 1;
        } else if (!strcmp(argv[j],"--metric")) {
            Modes.metric = 1;
        } else if (!strcmp(argv[j],"--aggressive")) {
            Modes.nfix_crc = MODES_MAX_BITERRORS;
        } else if (!strcmp(argv[j],"--interactive")) {
            Modes.interactive = 1;
        } else if (!strcmp(argv[j],"--interactive-rows") && more) {
            Modes.interactive_rows = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--interactive-ttl") && more) {
            Modes.interactive_display_ttl = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--lat") && more) {
            Modes.fUserLat = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--lon") && more) {
            Modes.fUserLon = atof(argv[++j]);
        } else if (!strcmp(argv[j],"--debug") && more) {
            char *f = argv[++j];
            while(*f) {
                switch(*f) {
                case 'D': Modes.debug |= MODES_DEBUG_DEMOD; break;
                case 'd': Modes.debug |= MODES_DEBUG_DEMODERR; break;
                case 'C': Modes.debug |= MODES_DEBUG_GOODCRC; break;
                case 'c': Modes.debug |= MODES_DEBUG_BADCRC; break;
                case 'p': Modes.debug |= MODES_DEBUG_NOPREAMBLE; break;
                case 'n': Modes.debug |= MODES_DEBUG_NET; break;
                case 'j': Modes.debug |= MODES_DEBUG_JS; break;
                default:
                    fprintf(stderr, "Unknown debugging flag: %c\n", *f);
                    exit(1);
                    break;
                }
                f++;
            }
        } else if (!strcmp(argv[j],"--stats")) {
            Modes.stats = -1;
        } else if (!strcmp(argv[j],"--stats-every") && more) {
            Modes.stats = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--snip") && more) {
            snipMode(atoi(argv[++j]));
            exit(0);
        } else if (!strcmp(argv[j],"--help")) {
            showHelp();
            exit(0);
        } else if (!strcmp(argv[j],"--ppm") && more) {
            Modes.ppm_error = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--quiet")) {
            Modes.quiet = 1;
        } else if (!strcmp(argv[j],"--mlat")) {
            Modes.mlat = 1;
        } else if (!strcmp(argv[j],"--interactive-rtl1090")) {
            Modes.interactive = 1;
            Modes.interactive_rtl1090 = 1;
        } else {
            fprintf(stderr,
                "Unknown or not enough arguments for option '%s'.\n\n",
                argv[j]);
            showHelp();
            exit(1);
        }
    }

#ifdef _WIN32
    // Try to comply with the Copyright license conditions for binary distribution
    if (!Modes.quiet) {showCopyright();}
#endif

#ifndef _WIN32
    // Setup for SIGWINCH for handling lines
    if (Modes.interactive) {signal(SIGWINCH, sigWinchCallback);}
#endif

    // Initialization
    modesInit();

    if (Modes.net_only) {
        fprintf(stderr,"Net-only mode, no RTL device or file open.\n");
    } else if (Modes.filename == NULL) {
        if (Modes.use_rtlsdr)
            modesInitRTLSDR();
#ifdef SDRPLAY
        else
            modesInitSDRplay();
#endif
    } else {
        if (Modes.filename[0] == '-' && Modes.filename[1] == '\0') {
            Modes.fd = STDIN_FILENO;
        } else if ((Modes.fd = open(Modes.filename,
#ifdef _WIN32
                                    (O_RDONLY | O_BINARY)
#else
                                    (O_RDONLY)
#endif
                                    )) == -1) {
            perror("Opening data file");
            exit(1);
        }
    }
    if (Modes.net) modesInitNet();

    // If the user specifies --net-only, just run in order to serve network
    // clients without reading data from the RTL device
    while (Modes.net_only) {
        if (Modes.exit) exit(0); // If we exit net_only nothing further in main()
        backgroundTasks();
        usleep(100000);
    }

    // Create the thread that will read the data from the device.
    pthread_create(&Modes.reader_thread, NULL, readerThreadEntryPoint, NULL);
    pthread_mutex_lock(&Modes.data_mutex);

    while (Modes.exit == 0) {

        if (Modes.iDataReady == 0) {
            pthread_cond_wait(&Modes.data_cond,&Modes.data_mutex); // This unlocks Modes.data_mutex, and waits for Modes.data_cond 
            continue;                                              // Once (Modes.data_cond) occurs, it locks Modes.data_mutex
        }

        // Modes.data_mutex is Locked, and (Modes.iDataReady != 0)
        if (Modes.iDataReady) { // Check we have new data, just in case!!
 
            Modes.iDataOut &= (MODES_ASYNC_BUF_NUMBER-1); // Just incase

            // Translate the next lot of I/Q samples into Modes.magnitude
            computeMagnitudeVector(Modes.pData[Modes.iDataOut]);

            Modes.stSystemTimeBlk = Modes.stSystemTimeRTL[Modes.iDataOut];

            // Update the input buffer pointer queue
            Modes.iDataOut   = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataOut + 1); 
            Modes.iDataReady = (MODES_ASYNC_BUF_NUMBER-1) & (Modes.iDataIn - Modes.iDataOut);   

            // If we lost some blocks, correct the timestamp
            if (Modes.iDataLost) {
                Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES * 6 * Modes.iDataLost);
                Modes.stat_blocks_dropped += Modes.iDataLost;
                Modes.iDataLost = 0;
            }

            // It's safe to release the lock now
            pthread_cond_signal (&Modes.data_cond);
            pthread_mutex_unlock(&Modes.data_mutex);

            // Process data after releasing the lock, so that the capturing
            // thread can read data while we perform computationally expensive
            // stuff at the same time.
            detectModeS(Modes.magnitude, MODES_ASYNC_BUF_SAMPLES);

            // Update the timestamp ready for the next block
            Modes.timestampBlk += (MODES_ASYNC_BUF_SAMPLES*6);
            Modes.stat_blocks_processed++;
        } else {
            pthread_cond_signal (&Modes.data_cond);
            pthread_mutex_unlock(&Modes.data_mutex);
        }

        backgroundTasks();
        pthread_mutex_lock(&Modes.data_mutex);
    }

    // If --stats were given, print statistics
    if (Modes.stats) {
        display_stats();
    }

    if (Modes.filename == NULL) {
        if (Modes.use_rtlsdr) {
            rtlsdr_cancel_async(Modes.dev);  // Cancel rtlsdr_read_async will cause data input thread to terminate cleanly
            rtlsdr_close(Modes.dev);
        } 
#ifdef SDRPLAY
        else { //Modes.use_sdrplay
            mir_sdr_Uninit();
        }
#endif
    }
    pthread_cond_destroy(&Modes.data_cond);     // Thread cleanup
    pthread_mutex_destroy(&Modes.data_mutex);
    pthread_join(Modes.reader_thread,NULL);     // Wait on reader thread exit
#ifndef _WIN32
    pthread_exit(0);
#else
    return (0);
#endif
}
//
//=========================================================================
//
