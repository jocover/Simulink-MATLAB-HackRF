/*
 * Copyright 2012 Communications Engineering Lab, KIT
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING. If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#define S_FUNCTION_NAME		hackrf_source
#define S_FUNCTION_LEVEL	2
/* Simulink */
#include "simstruc.h"

/* misc */
#include "utilities.h"
#include <stdint.h>
#include "hackrf.h"
#include <mutex>
#include <condition_variable>

/* defines */

#define BUF_SIZE		(16 * 32 * 512)
#define BUF_NUM			15
#define BUF_SKIP		3       /* buffers to skip due to initial garbage */
#define BYTES_PER_SAMPLE	2       /* hackrf device delivers 8 bit signed IQ data */

struct SampleBuffer
{ unsigned char **buf;
  unsigned int	num;
  unsigned int	head;
  unsigned int	used;
  unsigned int	offset;
  int		samp_avail;
  unsigned int	skipped;
  bool		overrun;
  bool		overran_before; };

/* S-function params */
enum SFcnParamsIndex
{
	DEVICE_INDEX=0,
	SAMPLE_RATE,
	BANDWIDTH,
	FREQUENCY,
	LNAGAIN,
	RXVGA,
	AMP,
	FRAME_LENGTH,
	USE_FRAMES,


	/* NUM_PARAMS must be the last in this enum to correctly set the number
	 * of expected parameters.
	 */
	NUM_PARAMS
};
enum PWorkIndex
{
	DEVICE,                 /* HackRF object */
	GAINS,                  /* list of possible gain values */

	THREAD,                 /* boost thread object for reading samples from dev */
	MUTEX,                  /* manages access to SBUF */
	COND_VAR,
	SBUF,                   /* sample buffer struct */

	P_WORK_LENGTH
};
enum IWorkIndex
{
	FREQUENCY_PORT_INDEX,   /* port index of FREQUENCY signal, 0 if none */
	GAIN_PORT_INDEX,        /* port index of LNAGAIN signal, 0 if none */

	I_WORK_LENGTH
};

enum RWorkIndex
{
	LAST_FREQUENCY,         /* holds current FREQUENCY (for port based setting) */
	LAST_GAIN,              /* holds current LNAGAIN (for port based setting) */

	R_WORK_LENGTH
};
/* ======================================================================== */
#if defined(MATLAB_MEX_FILE)
#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters( SimStruct *S )
/* ======================================================================== */
{
	NUMERIC_NOTEMPTY_OR_DIE( S, DEVICE_INDEX );
	NUMERIC_NOTEMPTY_OR_DIE( S, SAMPLE_RATE );
	NUMERIC_NOTEMPTY_OR_DIE( S, BANDWIDTH );
	NUMERIC_NOTEMPTY_OR_DIE( S, FREQUENCY );
	NUMERIC_NOTEMPTY_OR_DIE( S, LNAGAIN );
	NUMERIC_NOTEMPTY_OR_DIE( S, RXVGA );
	NUMERIC_NOTEMPTY_OR_DIE( S, AMP );
	NUMERIC_NOTEMPTY_OR_DIE( S, FRAME_LENGTH );
	NUMERIC_NOTEMPTY_OR_DIE( S, USE_FRAMES );
}


#endif /* MDL_CHECK_PARAMETERS */
/* ======================================================================== */
#define MDL_INITIAL_SIZES
static void mdlInitializeSizes( SimStruct *S )
/* ======================================================================== */
{
	int_T port;

	/* set number of expected parameters and check for a mismatch. */
	ssSetNumSFcnParams( S, NUM_PARAMS );
#if defined(MATLAB_MEX_FILE)
	if ( ssGetNumSFcnParams( S ) == ssGetSFcnParamsCount( S ) )
	{
		mdlCheckParameters( S );
		if ( ssGetErrorStatus( S ) != NULL )
			return;
	} else {
		return;
	}
#endif

	/* sampling */
	ssSetNumSampleTimes( S, PORT_BASED_SAMPLE_TIMES );

	/* Set number of input ports and tunability */
	ssSetSFcnParamTunable( S, DEVICE_INDEX, SS_PRM_NOT_TUNABLE );

	/* check whether frequency or gain is tunable */
	int_T num_input_ports = 0;
	if ( mxIsEmpty( ssGetSFcnParam( S, FREQUENCY ) ) )
	{
		ssSetSFcnParamTunable( S, FREQUENCY, SS_PRM_NOT_TUNABLE );
		num_input_ports++;
	}


	/* set the resulting number of ports */
	if ( !ssSetNumInputPorts( S, num_input_ports ) )
		return;

	/* port properties */
	for ( port = 0; port < num_input_ports; port++ )
	{
		/* RF_FREQ and GAIN both have the same port spec */
		ssSetInputPortMatrixDimensions( S, port, 1, 1 );
		ssSetInputPortComplexSignal( S, port, COMPLEX_NO );
		ssSetInputPortDataType( S, port, SS_DOUBLE );
		ssSetInputPortFrameData( S, port, FRAME_INHERITED );
		ssSetInputPortDirectFeedThrough( S, port, 1 );
		ssSetInputPortSampleTime( S, port, INHERITED_SAMPLE_TIME );
		ssSetInputPortOffsetTime( S, port, 0.0 );
	}

	/* Set number of output ports */
	if ( !ssSetNumOutputPorts( S, 1 ) )
		return;
	/* data port properties */
	port = 0;
	{
		double sample_time = 1 / mxGetScalar( ssGetSFcnParam( S, SAMPLE_RATE ) );

		/* get data port properties */
		const int_T	buf_length	= (int_T) (double) mxGetScalar( ssGetSFcnParam( S, FRAME_LENGTH ) );
		const Frame_T	outputsFrames	= ( (double) mxGetScalar( ssGetSFcnParam( S, USE_FRAMES ) ) > 0.0) ? FRAME_YES : FRAME_NO;
		const time_T	period		= (time_T) (sample_time * buf_length);

		/* set data port properties */
		ssSetOutputPortMatrixDimensions( S, port, buf_length, 1 );
		ssSetOutputPortComplexSignal( S, port, COMPLEX_YES );
		ssSetOutputPortDataType( S, port, SS_DOUBLE );
		ssSetOutputPortFrameData( S, port, outputsFrames );
		ssSetOutputPortSampleTime( S, port, period );
		ssSetOutputPortOffsetTime( S, port, 0.0 );
	}

	/* Prepare work Vectors */
	ssSetNumPWork( S, P_WORK_LENGTH );
	ssSetNumIWork( S, I_WORK_LENGTH );
	ssSetNumRWork( S, R_WORK_LENGTH );
	ssSetNumModes( S, 0 );
	ssSetNumNonsampledZCs( S, 0 );

	/* Specify the sim state compliance to be same as a built-in block */
	ssSetSimStateCompliance( S, USE_DEFAULT_SIM_STATE );

	ssSetOptions( S, 0 );
}


/* ======================================================================== */
static int rx_callback( hackrf_transfer * transfer )
/* ======================================================================== */
{
	unsigned char	* buf	= transfer->buffer;
	int		len	= transfer->valid_length;
	/* get access to Simulink stuff */
	SimStruct	*S	= (SimStruct *) transfer->rx_ctx;
	SampleBuffer	*sbuf	= (SampleBuffer *) ssGetPWorkValue( S, SBUF );
	if ( sbuf->skipped < BUF_SKIP )
	{
		sbuf->skipped++;
		return(0);
	}
	/*
	 * write received samples to sample buffer
	 */
	std::mutex * mutex = (std::mutex *) ssGetPWorkValue( S, MUTEX );
	std::unique_lock<std::mutex> lock( *mutex );


	/* get pos write pos in buffers */
	int buf_tail = (sbuf->head + sbuf->used) % sbuf->num;
	memcpy( sbuf->buf[buf_tail], buf, len );
	if ( sbuf->used == sbuf->num )
	{
		sbuf->overrun		= true;
		sbuf->overran_before	= true;
		sbuf->head		= (sbuf->head + 1) % sbuf->num;
	} else {
		sbuf->used++;
	}
	std::condition_variable * cond_var = (std::condition_variable *) ssGetPWorkValue( S, COND_VAR );
	cond_var->notify_one();
	/*
	 * notify output function
	 */


	return(0);
}


/* ======================================================================== */
static void mdlInitializeSampleTimes( SimStruct *S )
/* ======================================================================== */
{
	/* PORT_BASED_SAMPLE_TIMES */
}


/* ======================================================================== */
#define MDL_START
static void mdlStart( SimStruct *S )
/* ======================================================================== */
{
	int			ret = HACKRF_SUCCESS;
	hackrf_device_list_t	*list;
	struct hackrf_device	*device;
	const uint32_t		device_index	= (uint32_t) mxGetScalar( ssGetSFcnParam( S, DEVICE_INDEX ) );
	const uint64_t		frequency	= (uint64_t) mxGetScalar( ssGetSFcnParam( S, FREQUENCY ) );
	const double		sample_rate	= mxGetScalar( ssGetSFcnParam( S, SAMPLE_RATE ) );
	const uint32_t		lna_gain	= (uint32_t) mxGetScalar( ssGetSFcnParam( S, LNAGAIN ) ) & ~0x7;
	const uint32_t		bandwidth	= (uint32_t) mxGetScalar( ssGetSFcnParam( S, BANDWIDTH ) );
	const uint32_t		rxvga		= (uint32_t) mxGetScalar( ssGetSFcnParam( S, RXVGA ) ) & ~0x1;
	const uint32_t		amp		= (uint32_t) mxGetScalar( ssGetSFcnParam( S, AMP ) );

	/* Set options of this Block */
	ssSetOptions( S, ssGetOptions( S ) | SS_OPTION_CALL_TERMINATE_ON_EXIT );
	/* give handle to PWork vector */
	ssSetPWorkValue( S, DEVICE, NULL );
	/* init HackRF device */
	ret = hackrf_init();
	if ( ret < 0 )
	{
		ssSetErrorStatusf( S, "Failed to init HackRF device #%d", device_index );
		return;
	}
	list = hackrf_device_list();
	if ( list->devicecount < 1 )
	{
		ssSetErrorStatusf( S, "No HackRF boards found.\n" );
		return;
	}
	/* open HackRF device */
	ret = hackrf_device_list_open( list, device_index, &device );
	if ( ret < 0 )
	{
		ssSetErrorStatusf( S, "Failed to open HackRF device #%d", device_index );
		return;
	}

	/* give handle to PWork vector */
	ssSetPWorkValue( S, DEVICE, (struct hackrf_device *) device );

	ret = hackrf_set_sample_rate( device, sample_rate );
	if ( ret < 0 )
	{
		ssSetErrorStatusf( S, "Failed to Set HackRF Sample Rate #%d", device_index );
	}


	ret = hackrf_set_freq( device, frequency );
	if ( ret < 0 )
	{
		ssSetErrorStatusf( S, "Failed to Set HackRF frequency #%d", device_index );
	}

	ret = hackrf_set_baseband_filter_bandwidth( device, bandwidth );
	if ( ret < 0 )
	{
		ssSetErrorStatusf( S, "Failed to Set HackRF bandwidth #%d", device_index );
	}

	ret	= hackrf_set_vga_gain( device, lna_gain );
	ret	|= hackrf_set_vga_gain( device, rxvga );
	ret	|= hackrf_set_amp_enable( device, amp );
	if ( ret < 0 )
	{
		ssSetErrorStatusf( S, "Failed to Set HackRF gain #%d", device_index );
	}
	/* create mutex for sample thread */
	std::mutex * mutex = new std::mutex();
	ssSetPWorkValue( S, MUTEX, mutex );

	/* create condition variable for sample thread */
	/*
	 *  pthread_cond_t *cond_var = (pthread_cond_t *) malloc(sizeof(pthread_cond_t));
	 *  pthread_cond_init(cond_var,NULL);
	 */
	std::condition_variable * cond_var = new std::condition_variable();
	ssSetPWorkValue( S, COND_VAR, cond_var );

	/* allocate memory for sample buffer */
	SampleBuffer *sbuf = (SampleBuffer *) malloc( sizeof(SampleBuffer) );
	sbuf->num		= BUF_NUM;
	sbuf->head		= sbuf->used = sbuf->offset = sbuf->skipped = 0;
	sbuf->samp_avail	= BUF_SIZE / BYTES_PER_SAMPLE;
	sbuf->overrun		= sbuf->overran_before = false;
	sbuf->buf		= (unsigned char * *) malloc( sbuf->num * sizeof(unsigned char *) );
	if ( sbuf->buf )
	{
		for ( unsigned int i = 0; i < sbuf->num; ++i )
			sbuf->buf[i] = (unsigned char *) malloc( BUF_SIZE * sizeof(unsigned char) );
	}
	ssSetPWorkValue( S, SBUF, sbuf );

	ret = hackrf_start_rx( device, rx_callback, (void *) S );
	if ( ret < 0 )
	{
		ssSetErrorStatusf( S, "Failed to Start HackRF #%d", device_index );
	}
}


/* ======================================================================== */
#define MDL_OUTPUTS
static void mdlOutputs( SimStruct *S, int_T tid )
/* ======================================================================== */
{
	const int_T		frame_length	= (int_T) (double) mxGetScalar( ssGetSFcnParam( S, FRAME_LENGTH ) );
	struct  hackrf_device	*_device	= (hackrf_device *) ssGetPWorkValue( S, DEVICE );

	/* get sample buffer work vars */
	SampleBuffer *sbuf = (SampleBuffer *) ssGetPWorkValue( S, SBUF );

	/* output buffer */
	double* out = (double *) ssGetOutputPortSignal( S, 0 );

	std::mutex * mutex = (std::mutex *) ssGetPWorkValue( S, MUTEX );

	std::unique_lock <std::mutex> lock( *mutex );


	std::condition_variable * cond_var = (std::condition_variable *) ssGetPWorkValue( S, COND_VAR );

	/* wait for sample buffers to fill */
	while ( sbuf->used < 3 ) /* collect at least 3 buffers */
	{
		cond_var->wait( lock );
	}
	/* output overrun status (not in callback, due to issues with Simulink) */
	if ( sbuf->overrun )
	{
		ssPrintf( "O" );
		sbuf->overrun = false;
	}

	/* get next samples to be read (select buffer + offset) */
	char *buf = (char *) sbuf->buf[sbuf->head] + BYTES_PER_SAMPLE * (sbuf->offset);

	/* enough samples in selected buffer for a whole frame? */
	if ( frame_length <= sbuf->samp_avail )
	{
		/* read entire frame from current sample buffer */
		for ( int k = 0; k < BYTES_PER_SAMPLE * frame_length; ++k )
			out[k] = ( (double) (buf[k]) ) / 128.0;

		sbuf->offset		+= frame_length;
		sbuf->samp_avail	-= frame_length;
	} else {
		/* read as many as possible */
		for ( int k = 0; k < BYTES_PER_SAMPLE * sbuf->samp_avail; ++k )
			out[k] = ( (double) (buf[k]) ) / 128.0;

		/* mark current buffer as used / empty */
		sbuf->head = (sbuf->head + 1) % sbuf->num;
		sbuf->used--;


		/* set to start of the next sample buffer */
		buf = (char *) sbuf->buf[sbuf->head];

		int remaining = frame_length - sbuf->samp_avail;

		/* get the rest of samples */
		for ( int k = 0; k < BYTES_PER_SAMPLE * (remaining); ++k )
			out[k + BYTES_PER_SAMPLE * sbuf->samp_avail] = ( (double) (buf[k]) ) / 128.0;

		/* update offset with the current buffer and the number of sample remaining */
		sbuf->offset		= remaining;
		sbuf->samp_avail	= (BUF_SIZE / BYTES_PER_SAMPLE) - remaining;
	}
}


/* ======================================================================== */
static void mdlTerminate( SimStruct *S )
/* ======================================================================== */
{
	/* check if HackRF object has been created */
	if ( ssGetPWorkValue( S, DEVICE ) )
	{
		struct hackrf_device *device = (struct hackrf_device *) ssGetPWorkValue( S, DEVICE );

		hackrf_stop_rx( device );
		hackrf_close( device );
		hackrf_exit();
	}


	/* release thread stuff */
	if ( ssGetPWorkValue( S, MUTEX ) )
	{
		std::mutex	* mutex = (std::mutex *) ssGetPWorkValue( S, MUTEX );
		delete		mutex;
	}
	if ( ssGetPWorkValue( S, COND_VAR ) )
	{
		std::condition_variable * cond_var = (std::condition_variable *) ssGetPWorkValue( S, COND_VAR );
		delete			cond_var;
	}
	/* destroy sample buffer struct */
	if ( ssGetPWorkValue( S, SBUF ) )
	{
		SampleBuffer *sbuf = (SampleBuffer *) ssGetPWorkValue( S, SBUF );

		if ( sbuf->overran_before )
		{
			ssPrintf( "\n" );
		}

		if ( sbuf->buf )
		{
			for ( unsigned int i = 0; i < sbuf->num; ++i )
			{
				if ( sbuf->buf[i] )
					free( sbuf->buf[i] );
			}
			free( sbuf->buf );
		}
		free( sbuf );
	}
}


#ifdef  MATLAB_MEX_FILE         /* Is this file being compiled as a MEX-file? */
#include "simulink.c"           /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"            /* Code generation registration function */
#endif
