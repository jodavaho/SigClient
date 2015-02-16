#pragma once
// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the BB60_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// BB60_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef BB60_EXPORTS
#define BB60_API __declspec(dllexport)
#else
#define BB60_API __declspec(dllimport)
#endif

#define BB_MAX_DEVICES           8
#define BB_SAMPLERATE            80000000     // 80 Msamples per second
#define BB_MIN_FREQ              9.0e3        // 9 kHz (final?) min freq
#define BB_MAX_FREQ              6.5e9        // 6 GHz max freq
#define BB_MIN_RT_SPAN           200.0e3      // 200 kHz min real-time span
#define BB_MAX_RT_SPAN           20.0e6       // 20 MHz max real-time span
#define BB_MIN_RT_RBW            2465.0       // 2.4kHz min real-time rbw
#define BB_MAX_RT_RBW            631250.0     // 631kHz max real-time rbw
#define BB_MIN_TG_SPAN           200.0e3      // 200 kHz min time-gate span
#define BB_MAX_TG_SPAN           20.0e6       // 20 MHz max time-gate span
#define BB_MIN_SPAN              20.0         // 20 HZ min span, sweeping
#define BB_MAX_SPAN              (6.5e9 - 9.0e3) 
#define BB_MIN_SWEEP_TIME        0.001        // 1 millisecond
#define BB_MIN_BW                0.301003456  // Hz
#define BB_MAX_BW                10100000     // Hz
#define BB_RAW_PACKET_SIZE       299008       // For RAW_PIPE mode only
#define BB_MIN_USB_VOLTAGE       4.4f

#define BB_IDLE                           -1
#define BB_SWEEPING                       0x0
#define BB_REAL_TIME                      0x1      
#define BB_ZERO_SPAN                      0x2
#define BB_TIME_GATE                      0x3
#define BB_RAW_PIPE                       0x4
#define BB_RAW_SWEEP                      0x5
#define BB_RAW_SWEEP_LOOP                 0x6
#define BB_AUDIO_DEMOD                    0x7

#define BB_NO_SPUR_REJECT                 0x0     
#define BB_SPUR_REJECT                    0x1     
#define BB_BYPASS_RF                      0x2     

#define BB_LOG_SCALE                      0x0
#define BB_LIN_SCALE                      0x1
#define BB_LOG_FULL_SCALE                 0x2
#define BB_LIN_FULL_SCALE                 0x3

#define BB_NATIVE_RBW                     0x0
#define BB_NON_NATIVE_RBW                 0x1      
#define BB_6DB_RBW				          0x2 // Uses 6 dB cutoffs instead of 3 dB

#define BB_MIN_AND_MAX                    0x0  
#define BB_AVERAGE                        0x1
#define BB_MIN_ONLY                       0x2
#define BB_MAX_ONLY                       0x3
#define BB_QUASI_PEAK					  0x4 // For EMC

#define BB_LOG                            0x0 
#define BB_VOLTAGE                        0x1
#define BB_POWER                          0x2
#define BB_BYPASS                         0x3

#define BB_NUTALL                         0x0
#define BB_BLACKMAN                       0x1  
#define BB_HAMMING                        0x2
#define BB_FLAT_TOP                       0x3
#define BB_FLAT_TOP_EMC_9KHZ              0x4
#define BB_FLAT_TOP_EMC_120KHZ            0x5

#define BB_AUTO_GAIN                      -1
#define BB_NO_GAIN                        0x0
#define BB_LOW_GAIN                       0x1
#define BB_MED_GAIN                       0x2
#define BB_HIGH_GAIN                      0x3

#define BB_DEMOD_AM                       0x0
#define BB_DEMOD_FM                       0x1  
#define BB_DEMOD_USB                      0x2  
#define BB_DEMOD_LSB                      0x3  
#define BB_DEMOD_CW                       0x4  

#define BB_DEFAULT                        0x0
#define BB_TIME_STAMP                     0x10

#define BB_NO_TRIGGER                     0x0      
#define BB_VIDEO_TRIGGER                  0x1      
#define BB_EXTERNAL_TRIGGER               0x2

#define BB_TRIGGER_RISING                 0x0
#define BB_TRIGGER_FALLING                0x1

#define BB_TWENTY_MHZ                     0x0
#define BB_TEN_MHZ                        0x1
#define BB_SEVEN_MHZ                      0x2

#define BB_ENABLE                         0x0
#define BB_DISABLE                        0x1

// Port 1 IO
#define BB_PORT1_AC_COUPLED               0x00
#define BB_PORT1_DC_COUPLED               0x04
#define BB_PORT1_INT_REF_OUT              0x00
#define BB_PORT1_EXT_REF_IN               0x08
#define BB_PORT1_OUT_LOGIC_LOW            0x14
#define BB_PORT1_OUT_LOGIC_HIGH           0x1C
// Port 2 IO
#define BB_PORT2_OUT_LOGIC_LOW            0x00
#define BB_PORT2_OUT_LOGIC_HIGH           0x20
#define BB_PORT2_IN_TRIGGER_RISING_EDGE   0x40
#define BB_PORT2_IN_TRIGGER_FALLING_EDGE  0x60

// Status Codes
// Errors are negative and suffixed with 'Err'
// Errors stop the flow of execution, warnings do not
enum bbStatus {
	bbPacketFramingErr           = -13,
	bbGPSErr                     = -12,
	bbGainNotSetErr              = -11,
	bbDeviceNotIdleErr           = -10,
	bbDeviceInvalidErr           = -9,
	bbBufferTooSmallErr          = -8,
	bbNullPtrErr                 = -7,
	bbAllocationLimitErr         = -6,
	bbDeviceAlreadyStreamingErr  = -5,
	bbInvalidParameterErr        = -4,
	bbDeviceNotConfiguredErr     = -3,
	bbDeviceNotStreamingErr      = -2,
	bbDeviceNotOpenErr           = -1,
	bbNoError                    = 0,
	bbAdjustedParameter          = 1,
	bbInvalidBandwidth           = 2,
	bbADCOverflow                = 3,
	bbNoTriggerFound             = 4
};

#ifdef __cplusplus
extern "C" {
#endif

BB60_API bbStatus bbOpenDevice(
	int *device
	);	

BB60_API bbStatus bbCloseDevice(
	int device
	);	

BB60_API bbStatus bbConfigureAcquisition(
	int device, 
	unsigned int detectorType, 
	unsigned int verticalScale  
	);    

BB60_API bbStatus bbConfigureCenterSpan(
	int device, 
	double center, 
	double span
	);

BB60_API bbStatus bbConfigureLevel(
	int device, 
	double ref, 
	double atten
	);

BB60_API bbStatus bbConfigureGain(
	int device,
	int gain
	);

BB60_API bbStatus bbConfigureSweepCoupling(
	int device, 
	double rbw, 
	double vbw, 
	double sweepTime, 
	unsigned int rbwType,
	unsigned int rejection
	); 

BB60_API bbStatus bbConfigureWindow(
	int device,
	unsigned int window
	);

BB60_API bbStatus bbConfigureProcUnits(
	int device,
	unsigned int units 
	); 

BB60_API bbStatus bbConfigureTrigger(
	int device,
	unsigned int type,
	unsigned int edge,
	double level,
	double timeout
	);

BB60_API bbStatus bbConfigureTimeGate(
	int device,
	double gateDelay,
	double gateLength,
	double timeout
	);

BB60_API bbStatus bbConfigureRawSweep(
	int device,
	unsigned int start, // In MHz
	unsigned int ppf,
	unsigned int steps,
	unsigned int stepsize
	);

BB60_API bbStatus bbConfigureIO(
	int device,
	unsigned int port1,
	unsigned int port2
	);

BB60_API bbStatus bbConfigureDemod(
	int device,
	int modulationType,
	double freq,		  // in Hz
	float IFBW,               // 3 kHz to 300 kHz
	float audioLowPassFreq,   // 1 kHz to 12 kHz
	float audioHighPassFreq,  // 20 Hz to 1 kHz
	float FMDeemphasis        // in usec, typically 500 or 750
	);

BB60_API bbStatus bbInitiate(
	int device,
	unsigned int mode, 
	unsigned int flag  // demod AM/FM for zero-span
	);

BB60_API bbStatus bbQueryTraceInfo(
	int device, 
	unsigned int *traceSize,
	double *binSize,
	double *startFreq
	);

BB60_API bbStatus bbFetchTrace(            
	int device, 
	int arraySize,
	double *min, 
	double *max
	);

BB60_API bbStatus bbFetchAudio(            
	int device, 
	float *audio 
	);

BB60_API bbStatus bbFetchRawCorrections(
	int device,
	float *corrections,
	int *index,
	double *startFreq
	);

BB60_API bbStatus bbFetchRaw(
	int device,
	float *buffer,
	int *triggers
	);

BB60_API bbStatus bbFetchRawSweep(
	int device,
	short *buffer
	);

BB60_API bbStatus bbStartRawSweepLoop(
	int device,
	short *buffer,
	unsigned int bufferLength,
	unsigned int *bufferIndex,
	unsigned int *bufferCounter
	);

BB60_API bbStatus bbQueryTimestamp(
	int device,
	unsigned int *seconds,
	unsigned int *nanoseconds
	);

BB60_API bbStatus bbSyncCPUtoGPS(
	int comPort, 
	int baudRate
	);

BB60_API bbStatus bbAbort(
	int device
	);

BB60_API bbStatus bbPreset(
	int device
	);

BB60_API bbStatus bbSetUSBTimeout(
	int device,
	unsigned int mode
	);

BB60_API bbStatus bbQueryDiagnostics(
 	int device,
	float *temperature,
	float *voltage1_8,
	float *voltage1_2,
	float *voltageUSB,
	float *currentUSB
	);

BB60_API bbStatus bbSelfCal(
	int device 
	);

BB60_API bbStatus bbGetSerialNumber(
	int device,
	unsigned int *sid
	);

BB60_API const char *bbGetErrorString(
	bbStatus status
	);

BB60_API bbStatus bbFetchAudio(            
	int device, 
	float *audio 
	);

#ifdef __cplusplus
} /* Extern "C" */
#endif

