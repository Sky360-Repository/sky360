#pragma once

#include <stdint.h>

#include "qhyccderr.h"
#include "qhyccdcamdef.h"
#include "config.h"

#if defined (_WIN32)
    #ifndef EXPORTFUNC
        #define EXPORTFUNC extern "C" __declspec(dllexport)
    #endif
    #ifndef STDCALL
        #define STDCALL __stdcall
    #endif
    #ifndef EXPORTC
        #define EXPORTC extern "C"
    #endif
#else
    #define EXPORTFUNC extern "C"
    #define STDCALL
    #define EXPORTC extern "C"
#endif


enum CONTROL_ID
{
/*0*/  CONTROL_BRIGHTNESS = 0, //!< image brightness
/*1*/  CONTROL_CONTRAST,       //!< image contrast
/*2*/  CONTROL_WBR,            //!< the red of white balance
/*3*/  CONTROL_WBB,            //!< the blue of white balance
/*4*/  CONTROL_WBG,            //!< the green of white balance
/*5*/  CONTROL_GAMMA,          //!< screen gamma
/*6*/  CONTROL_GAIN,           //!< camera gain
/*7*/  CONTROL_OFFSET,         //!< camera offset
/*8*/  CONTROL_EXPOSURE,       //!< expose time (us)
/*9*/  CONTROL_SPEED,          //!< transfer speed
/*10*/  CONTROL_TRANSFERBIT,    //!< image depth bits
/*11*/  CONTROL_CHANNELS,       //!< image channels
/*12*/  CONTROL_USBTRAFFIC,     //!< hblank
/*13*/  CONTROL_ROWNOISERE,     //!< row denoise
/*14*/  CONTROL_CURTEMP,        //!< current cmos or ccd temprature
/*15*/  CONTROL_CURPWM,         //!< current cool pwm
/*16*/  CONTROL_MANULPWM,       //!< set the cool pwm
/*17*/  CONTROL_CFWPORT,        //!< control camera color filter wheel port
/*18*/  CONTROL_COOLER,         //!< check if camera has cooler
/*19*/  CONTROL_ST4PORT,        //!< check if camera has st4port
/*20*/  CAM_COLOR,              /// FIXME!  CAM_IS_COLOR CAM_COLOR conflict
/*21*/  CAM_BIN1X1MODE,         //!< check if camera has bin1x1 mode
/*22*/  CAM_BIN2X2MODE,         //!< check if camera has bin2x2 mode
/*23*/  CAM_BIN3X3MODE,         //!< check if camera has bin3x3 mode
/*24*/  CAM_BIN4X4MODE,         //!< check if camera has bin4x4 mode
/*25*/  CAM_MECHANICALSHUTTER,                   //!< mechanical shutter
/*26*/  CAM_TRIGER_INTERFACE,                    //!< check if camera has triger interface
/*27*/  CAM_TECOVERPROTECT_INTERFACE,            //!< tec overprotect
/*28*/  CAM_SINGNALCLAMP_INTERFACE,              //!< singnal clamp
/*29*/  CAM_FINETONE_INTERFACE,                  //!< fine tone
/*30*/  CAM_SHUTTERMOTORHEATING_INTERFACE,       //!< shutter motor heating
/*31*/  CAM_CALIBRATEFPN_INTERFACE,              //!< calibrated frame
/*32*/  CAM_CHIPTEMPERATURESENSOR_INTERFACE,     //!< chip temperaure sensor
/*33*/  CAM_USBREADOUTSLOWEST_INTERFACE,         //!< usb readout slowest

/*34*/  CAM_8BITS,                               //!< 8bit depth
/*35*/  CAM_16BITS,                              //!< 16bit depth
/*36*/  CAM_GPS,                                 //!< check if camera has gps

/*37*/  CAM_IGNOREOVERSCAN_INTERFACE,            //!< ignore overscan area

/*38*/  //QHYCCD_3A_AUTOBALANCE,					 //!< auto white balance//lyl move to 1024
/*39*/  QHYCCD_3A_AUTOEXPOSURE=39,					 //!< auto exposure
/*40*/  QHYCCD_3A_AUTOFOCUS,
/*41*/  CONTROL_AMPV,                            //!< ccd or cmos ampv
/*42*/  CONTROL_VCAM,                            //!< Virtual Camera on off
/*43*/  CAM_VIEW_MODE,

/*44*/  CONTROL_CFWSLOTSNUM,         //!< check CFW slots number
/*45*/  IS_EXPOSING_DONE,
/*46*/  ScreenStretchB,
/*47*/  ScreenStretchW,
/*48*/  CONTROL_DDR,
/*49*/  CAM_LIGHT_PERFORMANCE_MODE,

/*50*/  CAM_QHY5II_GUIDE_MODE,
/*51*/  DDR_BUFFER_CAPACITY,
/*52*/  DDR_BUFFER_READ_THRESHOLD,
/*53*/  DefaultGain,
/*54*/  DefaultOffset,
/*55*/  OutputDataActualBits,
/*56*/  OutputDataAlignment,

/*57*/  CAM_SINGLEFRAMEMODE,
/*58*/  CAM_LIVEVIDEOMODE,
 /*59*/ CAM_IS_COLOR,
/*60*/  hasHardwareFrameCounter,
/*61*/  CONTROL_MAX_ID_Error, //** No Use , last max index */
/*62*/  CAM_HUMIDITY,			//!<check if camera has	 humidity sensor  20191021 LYL Unified humidity function
/*63*/  CAM_PRESSURE,             //check if camera has pressure sensor
/*64*/  CONTROL_VACUUM_PUMP,        /// if camera has VACUUM PUMP
/*65*/  CONTROL_SensorChamberCycle_PUMP, ///air cycle pump for sensor drying
/*66*/  CAM_32BITS,
/*67*/  CAM_Sensor_ULVO_Status, /// Sensor working status [0:init  1:good  2:checkErr  3:monitorErr 8:good 9:powerChipErr]  410 461 411 600 268 [Eris board]
/*68*/  CAM_SensorPhaseReTrain, /// 2020,4040/PRO，6060,42PRO
/*69*/  CAM_InitConfigFromFlash, /// 2410 461 411 600 268 for now
/*70*/  CAM_TRIGER_MODE, //check if camera has multiple triger mode
/*71*/  CAM_TRIGER_OUT, //check if camera support triger out function
/*72*/  CAM_BURST_MODE, //check if camera support burst mode
/*73*/  CAM_SPEAKER_LED_ALARM, // for OEM-600
/*74*/  CAM_WATCH_DOG_FPGA, // for _QHY5III178C Celestron, SDK have to feed this dog or it go reset

/*75*/  CAM_BIN6X6MODE,         //!< check if camera has bin6x6 mode
/*76*/  CAM_BIN8X8MODE,         //!< check if camera has bin8x8 mode
/*77*/  CAM_GlobalSensorGPSLED,         ///Show GPS LED tab on sharpCap
/*78*/  CONTROL_ImgProc,   /// Process image
/*79*/  CONTROL_RemoveRBI,   /// Remove single RBI


/* Do not Put Item after  CONTROL_MAX_ID !! This should be the max index of the list */
/*Last One */  CONTROL_MAX_ID,

//TEST id name list
/*1024*/ CONTROL_AUTOWHITEBALANCE=1024, //!<auto white balance  eg.CONTROL_TEST=1024
/*1025*/ CONTROL_AUTOEXPOSURE			//!<auto exposure
};

enum BAYER_ID
{
  BAYER_GB = 1,
  BAYER_GR,
  BAYER_BG,
  BAYER_RG
};

typedef struct _QHYCamMinMaxStepValue
{
  const char *name;
  double min;
  double max;
  double step;
}
QHYCamMinMaxStepValue;

typedef uint64_t QHYDWORD;

typedef QHYDWORD  (*QHYCCDProcCallBack) (void *handle,
    QHYDWORD message,
    QHYDWORD wParam,
    QHYDWORD lParam);

typedef void qhyccd_handle;

EXPORTC void STDCALL OutputQHYCCDDebug(char *strOutput);
EXPORTC void STDCALL SetQHYCCDAutoDetectCamera(bool enable);
EXPORTC void STDCALL SetQHYCCDLogLevel(uint8_t logLevel);
EXPORTC void STDCALL EnableQHYCCDMessage(bool enable);
EXPORTC void STDCALL set_histogram_equalization(bool enable);
EXPORTC void STDCALL EnableQHYCCDLogFile(bool enable);
EXPORTC uint32_t STDCALL SetQHYCCDSingleFrameTimeOut(qhyccd_handle *h, uint32_t time);  
EXPORTC const char* STDCALL GetTimeStamp();
EXPORTC uint32_t STDCALL InitQHYCCDResource();
EXPORTC uint32_t STDCALL ReleaseQHYCCDResource();
EXPORTC uint32_t STDCALL ScanQHYCCD();
EXPORTC uint32_t STDCALL GetQHYCCDId(uint32_t index, char *id);
EXPORTC uint32_t STDCALL GetQHYCCDModel(char *id, char *model);
EXPORTC qhyccd_handle * STDCALL OpenQHYCCD(char *id);
EXPORTC uint32_t STDCALL CloseQHYCCD(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL SetQHYCCDStreamMode(qhyccd_handle *handle, uint8_t mode);
EXPORTC uint32_t STDCALL InitQHYCCD(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL IsQHYCCDControlAvailable(qhyccd_handle *handle, CONTROL_ID controlId);
EXPORTC uint32_t STDCALL GetQHYCCDControlName(qhyccd_handle *handle, CONTROL_ID controlId, char *IDname);
EXPORTC uint32_t STDCALL SetQHYCCDParam(qhyccd_handle *handle, CONTROL_ID controlId, double value);
EXPORTC double STDCALL GetQHYCCDParam(qhyccd_handle *handle, CONTROL_ID controlId);
EXPORTC uint32_t STDCALL GetQHYCCDParamMinMaxStep(qhyccd_handle *handle,CONTROL_ID controlId,double *min,double *max,double *step);
EXPORTC uint32_t STDCALL SetQHYCCDResolution(qhyccd_handle *handle,uint32_t x,uint32_t y,uint32_t xsize,uint32_t ysize);
EXPORTC uint32_t STDCALL GetQHYCCDMemLength(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL ExpQHYCCDSingleFrame(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL GetQHYCCDSingleFrame(qhyccd_handle *handle,uint32_t *w,uint32_t *h,uint32_t *bpp,uint32_t *channels,uint8_t *imgdata);
EXPORTC uint32_t STDCALL CancelQHYCCDExposing(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL CancelQHYCCDExposingAndReadout(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL BeginQHYCCDLive(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL GetQHYCCDLiveFrame(qhyccd_handle *handle,uint32_t *w,uint32_t *h,uint32_t *bpp,uint32_t *channels,uint8_t *imgdata);
EXPORTC uint32_t STDCALL StopQHYCCDLive(qhyccd_handle *handle);
EXPORTFUNC uint32_t STDCALL QHYCCDPcieRecv(qhyccd_handle *handle, void * data, int len,uint64_t timeout);
EXPORTFUNC uint32_t STDCALL GetQHYCCDPcieDDRNum(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL SetQHYCCDBinMode(qhyccd_handle *handle,uint32_t wbin,uint32_t hbin);
EXPORTC uint32_t STDCALL SetQHYCCDBitsMode(qhyccd_handle *handle,uint32_t bits);
EXPORTC uint32_t STDCALL ControlQHYCCDTemp(qhyccd_handle *handle,double targettemp);
EXPORTC uint32_t STDCALL ControlQHYCCDGuide(qhyccd_handle *handle,uint32_t direction,uint16_t duration);
EXPORTC uint32_t STDCALL SendOrder2QHYCCDCFW(qhyccd_handle *handle,char *order,uint32_t length);
EXPORTC	uint32_t STDCALL GetQHYCCDCFWStatus(qhyccd_handle *handle,char *status);
EXPORTC	uint32_t STDCALL IsQHYCCDCFWPlugged(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL GetQHYCCDTrigerInterfaceNumber(qhyccd_handle *handle, uint32_t *modeNumber);
EXPORTC uint32_t STDCALL GetQHYCCDTrigerInterfaceName(qhyccd_handle *handle, uint32_t modeNumber, char *name);
EXPORTC uint32_t STDCALL SetQHYCCDTrigerInterface(qhyccd_handle *handle, uint32_t trigerMode);
EXPORTC uint32_t STDCALL SetQHYCCDTrigerFunction(qhyccd_handle *h, bool value);
EXPORTC uint32_t STDCALL SetQHYCCDTrigerMode(qhyccd_handle *handle, uint32_t trigerMode);
EXPORTC uint32_t STDCALL EnableQHYCCDTrigerOut(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL EnableQHYCCDTrigerOutA(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL SendSoftTriger2QHYCCDCam(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL SetQHYCCDTrigerFilterOnOff(qhyccd_handle *handle, bool onoff);
EXPORTC uint32_t STDCALL SetQHYCCDTrigerFilterTime(qhyccd_handle *handle, uint32_t time);
EXPORTC void STDCALL Bits16ToBits8(qhyccd_handle *h, uint8_t *InputData16, uint8_t *OutputData8, uint32_t imageX, uint32_t imageY, uint16_t B, uint16_t W);
EXPORTC void  STDCALL HistInfo192x130(qhyccd_handle *h, uint32_t x, uint32_t y, uint8_t *InBuf, uint8_t *OutBuf);
EXPORTC uint32_t STDCALL OSXInitQHYCCDFirmware(char *path);
EXPORTC uint32_t STDCALL OSXInitQHYCCDFirmwareArray();
EXPORTC uint32_t STDCALL OSXInitQHYCCDAndroidFirmwareArray(int idVendor, int idProduct, qhyccd_handle *handle);
EXPORTC uint32_t STDCALL GetQHYCCDChipInfo(qhyccd_handle *h, double *chipw, double *chiph, uint32_t *imagew, uint32_t *imageh, double *pixelw, double *pixelh, uint32_t *bpp);
EXPORTC uint32_t STDCALL GetQHYCCDEffectiveArea(qhyccd_handle *h,uint32_t *startX, uint32_t *startY, uint32_t *sizeX, uint32_t *sizeY);
EXPORTC uint32_t STDCALL GetQHYCCDOverScanArea(qhyccd_handle *h,uint32_t *startX, uint32_t *startY, uint32_t *sizeX, uint32_t *sizeY);
EXPORTC uint32_t STDCALL GetQHYCCDCurrentROI(qhyccd_handle *handle, uint32_t *startX, uint32_t *startY, uint32_t *sizeX, uint32_t *sizeY);
EXPORTC uint32_t STDCALL SetQHYCCDFocusSetting(qhyccd_handle *h,uint32_t focusCenterX, uint32_t focusCenterY);
EXPORTC uint32_t STDCALL GetQHYCCDExposureRemaining(qhyccd_handle *h);
EXPORTC uint32_t STDCALL GetQHYCCDFWVersion(qhyccd_handle *h,uint8_t *buf);
EXPORTC uint32_t STDCALL GetQHYCCDFPGAVersion(qhyccd_handle *h, uint8_t fpga_index, uint8_t *buf);
EXPORTC uint32_t STDCALL SetQHYCCDInterCamSerialParam(qhyccd_handle *h, uint32_t opt);
EXPORTC uint32_t STDCALL QHYCCDInterCamSerialTX(qhyccd_handle *h, char *buf, uint32_t length);
EXPORTC uint32_t STDCALL QHYCCDInterCamSerialRX(qhyccd_handle *h, char *buf);
EXPORTC uint32_t STDCALL QHYCCDInterCamOledOnOff(qhyccd_handle *handle, uint8_t onoff);
EXPORTC uint32_t STDCALL SetQHYCCDInterCamOledBrightness(qhyccd_handle *handle, uint8_t brightness);
EXPORTC uint32_t STDCALL SendFourLine2QHYCCDInterCamOled(qhyccd_handle *handle, char *messagetemp, char *messageinfo, char *messagetime, char *messagemode);
EXPORTC uint32_t STDCALL SendTwoLine2QHYCCDInterCamOled(qhyccd_handle *handle, char *messageTop, char *messageBottom);
EXPORTC uint32_t STDCALL SendOneLine2QHYCCDInterCamOled(qhyccd_handle *handle, char *messageTop);
EXPORTC uint32_t STDCALL GetQHYCCDCameraStatus(qhyccd_handle *h, uint8_t *buf);
EXPORTC uint32_t STDCALL GetQHYCCDShutterStatus(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL ControlQHYCCDShutter(qhyccd_handle *handle, uint8_t status);
EXPORTC uint32_t STDCALL GetQHYCCDPressure(qhyccd_handle *handle, double *pressure);
EXPORTC uint32_t STDCALL GetQHYCCDHumidity(qhyccd_handle *handle, double *hd);
EXPORTC uint32_t STDCALL QHYCCDI2CTwoWrite(qhyccd_handle *handle, uint16_t addr, uint16_t value);
EXPORTC uint32_t STDCALL QHYCCDI2CTwoRead(qhyccd_handle *handle, uint16_t addr);
EXPORTC double STDCALL GetQHYCCDReadingProgress(qhyccd_handle *handle);
EXPORTC uint32_t STDCALL TestQHYCCDPIDParas(qhyccd_handle *h, double p, double i, double d);
EXPORTC uint32_t STDCALL DownloadFX3FirmWare(uint16_t vid, uint16_t pid, char *imgpath);
EXPORTC uint32_t STDCALL GetQHYCCDType(qhyccd_handle *h);
EXPORTC uint32_t STDCALL SetQHYCCDDebayerOnOff(qhyccd_handle *h, bool onoff);
EXPORTC uint32_t STDCALL SetQHYCCDFineTone(qhyccd_handle *h, uint8_t setshporshd, uint8_t shdloc, uint8_t shploc, uint8_t shwidth);
EXPORTC uint32_t STDCALL SetQHYCCDGPSVCOXFreq(qhyccd_handle *handle, uint16_t i);
EXPORTC uint32_t STDCALL SetQHYCCDGPSLedCalMode(qhyccd_handle *handle, uint8_t i);
EXPORTC void STDCALL SetQHYCCDGPSLedCal(qhyccd_handle *handle, uint32_t pos, uint8_t width);
EXPORTC void STDCALL SetQHYCCDGPSPOSA(qhyccd_handle *handle, uint8_t is_slave, uint32_t pos, uint8_t width);
EXPORTC void STDCALL SetQHYCCDGPSPOSB(qhyccd_handle *handle, uint8_t is_slave, uint32_t pos, uint8_t width);
EXPORTC uint32_t STDCALL SetQHYCCDGPSMasterSlave(qhyccd_handle *handle, uint8_t i);
EXPORTC void STDCALL SetQHYCCDGPSSlaveModeParameter(qhyccd_handle *handle, uint32_t target_sec, uint32_t target_us, uint32_t deltaT_sec, uint32_t deltaT_us, uint32_t expTime);
EXPORTC void STDCALL SetQHYCCDQuit();
EXPORTC uint32_t STDCALL QHYCCDVendRequestWrite(qhyccd_handle *h, uint8_t req, uint16_t value, uint16_t index1, uint32_t length, uint8_t *data);
EXPORTC uint32_t STDCALL QHYCCDVendRequestRead(qhyccd_handle *h, uint8_t req, uint16_t value, uint16_t index1, uint32_t length, uint8_t *data);
EXPORTC uint32_t STDCALL QHYCCDReadUSB_SYNC(qhyccd_handle *pDevHandle, uint8_t endpoint, uint32_t length, uint8_t *data, uint32_t timeout);
EXPORTC uint32_t STDCALL QHYCCDLibusbBulkTransfer(qhyccd_handle *pDevHandle, uint8_t endpoint, uint8_t *data, uint32_t length, int32_t *transferred, uint32_t timeout);
EXPORTC uint32_t STDCALL GetQHYCCDSDKVersion(uint32_t *year, uint32_t *month, uint32_t *day, uint32_t *subday);
EXPORTC uint32_t STDCALL GetQHYCCDNumberOfReadModes(qhyccd_handle *h, uint32_t *numModes);
EXPORTC uint32_t STDCALL GetQHYCCDReadModeResolution(qhyccd_handle *h, uint32_t modeNumber, uint32_t* width, uint32_t* height);
EXPORTC uint32_t STDCALL GetQHYCCDReadModeName(qhyccd_handle *h,uint32_t modeNumber, char* name);
EXPORTC uint32_t STDCALL SetQHYCCDReadMode(qhyccd_handle *h, uint32_t modeNumber);
EXPORTC uint32_t STDCALL GetQHYCCDReadMode(qhyccd_handle *h, uint32_t* modeNumber);
EXPORTC uint32_t STDCALL GetQHYCCDBeforeOpenParam(QHYCamMinMaxStepValue *p, CONTROL_ID controlId);
EXPORTC uint32_t STDCALL EnableQHYCCDBurstMode(qhyccd_handle *h, bool i);
EXPORTC uint32_t STDCALL SetQHYCCDBurstModeStartEnd(qhyccd_handle *h, unsigned short start, unsigned short end);
EXPORTC uint32_t STDCALL EnableQHYCCDBurstCountFun(qhyccd_handle *h, bool i);
EXPORTC uint32_t STDCALL ResetQHYCCDFrameCounter(qhyccd_handle *h);
EXPORTC uint32_t STDCALL SetQHYCCDBurstIDLE(qhyccd_handle *h);
EXPORTC uint32_t STDCALL ReleaseQHYCCDBurstIDLE(qhyccd_handle *h);
EXPORTC uint32_t STDCALL SetQHYCCDBurstModePatchNumber(qhyccd_handle *h, uint32_t value);
EXPORTC uint32_t STDCALL SetQHYCCDEnableLiveModeAntiRBI(qhyccd_handle *h, uint32_t value);
EXPORTC uint32_t STDCALL SetQHYCCDWriteFPGA(qhyccd_handle *h, uint8_t number, uint8_t regindex, uint8_t regvalue);
EXPORTC uint32_t STDCALL SetQHYCCDWriteCMOS(qhyccd_handle *h, uint8_t number, uint16_t regindex, uint16_t regvalue);
EXPORTC uint32_t STDCALL SetQHYCCDTwoChannelCombineParameter(qhyccd_handle *handle, double x, double ah, double bh, double al, double bl);
EXPORTC uint32_t STDCALL EnableQHYCCDImageOSD(qhyccd_handle *h, uint32_t i);
EXPORTC uint32_t STDCALL GetQHYCCDPreciseExposureInfo(qhyccd_handle *h,
                                                         uint32_t *PixelPeriod_ps,
                                                         uint32_t *LinePeriod_ns,
                                                         uint32_t *FramePeriod_us,
                                                         uint32_t *ClocksPerLine,
                                                         uint32_t *LinesPerFrame,
                                                         uint32_t *ActualExposureTime,
                                                         uint8_t  *isLongExposureMode);
EXPORTFUNC uint32_t STDCALL GetQHYCCDRollingShutterEndOffset(qhyccd_handle *h, uint32_t row, double *offset);                                                        
EXPORTC void STDCALL QHYCCDQuit();
EXPORTC QHYDWORD STDCALL SetQHYCCDCallBack(QHYCCDProcCallBack ProcCallBack, int32_t Flag);
EXPORTFUNC void RegisterPnpEventIn(void (*in_pnp_event_in_func)(char *id));
EXPORTFUNC void RegisterPnpEventOut(void (*in_pnp_event_out_func)(char *id));
EXPORTFUNC uint32_t STDCALL resetDev(char *deviceID, uint32_t readModeIndex, uint8_t streamMode,qhyccd_handle* devHandle, uint32_t* imageWidth, uint32_t* imageHigh, uint32_t bitDepth);
EXPORTFUNC void RegisterDataEventSingle(void (*in_data_event_single_func)(char *id, uint8_t *imgdata));
EXPORTFUNC void RegisterDataEventLive(void (*in_data_event_live_func)(char *id, uint8_t *imgdata));
EXPORTFUNC void RegisterTransferEventError(void (*transfer_event_error_func)());
EXPORTFUNC uint32_t STDCALL PCIEClearDDR(qhyccd_handle *handle);
EXPORTFUNC uint32_t STDCALL GetReadModesNumber(char* deviceID, uint32_t* numModes);
EXPORTFUNC uint32_t STDCALL GetReadModeName(char* deviceID, uint32_t modeIndex, char* modeName);
EXPORTFUNC void STDCALL QHYCCDSensorPhaseReTrain(qhyccd_handle *handle);
EXPORTFUNC void STDCALL QHYCCDReadInitConfigFlash(qhyccd_handle *handle, char* configString_raw64);
EXPORTFUNC void STDCALL QHYCCDEraseInitConfigFlash(qhyccd_handle *handle);
EXPORTFUNC void STDCALL QHYCCDResetFlashULVOError(qhyccd_handle *handle);
EXPORTFUNC void STDCALL QHYCCDTestFlashULVOError(qhyccd_handle *handle);
EXPORTFUNC void STDCALL QHYCCDSetFlashInitPWM(qhyccd_handle *handle, uint8_t pwm);
EXPORTFUNC void STDCALL QHYCCDGetDebugDataD3(qhyccd_handle *handle, char* debugData_raw64);
EXPORTFUNC uint32_t STDCALL QHYCCDSolve(int timeout_s, float scale_l, float  scale_h,float center_ra, float center_dec, float center_r, float& s_ra, float& s_dec, float& s_size_x, float& s_size_y, float& s_rotation);
EXPORTFUNC void STDCALL QHYCCDEqualizeHistogram(uint8_t * pdata, int width, int height, int bpp);
void  QHYCCDGetDebugControlID(CONTROL_ID controlId, bool hasValue, bool isSetValue, double value);
EXPORTFUNC int STDCALL QHYCCD_fpga_list(struct fpga_info_list &list);
EXPORTFUNC uint32_t STDCALL QHYCCD_fpga_open(int id);
EXPORTFUNC void STDCALL QHYCCD_fpga_close();
EXPORTFUNC int STDCALL QHYCCD_fpga_send(int chnl, void * data, int len, int destoff, int last, uint64_t timeout);
EXPORTFUNC int STDCALL QHYCCD_fpga_recv(int chnl, void * data, int len, uint64_t timeout);
EXPORTFUNC void STDCALL QHYCCD_fpga_reset();
