/*!
 * \file novatel/novatel.h
 * \author David Hodo <david.hodo@gmail.com>
 * \version 1.0
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo - Integrated Solutions for Systems (IS4S)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides an interface for OEM 4, V, and 6 series of Novatel GPS receivers
 *
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 * This library depends on Serial: https://github.com/wjwwood/serial
 *
 */

#ifndef NOVATEL_H
#define NOVATEL_H

#include <string>
#include <cstring> // for size_t

// Structure definition headers
#include "novatel/novatel_enums.h"
#include "novatel/novatel_structures.h"
// Boost Headers
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
//#include <boost/condition_variable.hpp>
// Serial Headers
#include "serial/serial.h"

namespace novatel {

// used to convert lat and long to UTM coordinates
#define GRAD_A_RAD(g) ((g)*0.0174532925199433)
#define CRC32_POLYNOMIAL 0xEDB88320L

// Constants for unpacking RANGECMP
#define CMP_MAX_VALUE         8388608.0
#define CMP_GPS_WAVELENGTH_L1 0.1902936727984
#define CMP_GPS_WAVELENGTH_L2 0.2442102134246

typedef boost::function<double()> GetTimeCallback;
typedef boost::function<void()> HandleAcknowledgementCallback;

// Messaging callbacks
typedef boost::function<void(const std::string&)> LogMsgCallback;
typedef boost::function<void(unsigned char *)> RawMsgCallback;

// INS Specific Callbacks
typedef boost::function<void(InsPositionVelocityAttitude&, double&)> InsPositionVelocityAttitudeCallback;
typedef boost::function<void(InsPositionVelocityAttitudeShort&, double&)> InsPositionVelocityAttitudeShortCallback;
typedef boost::function<void(VehicleBodyRotation&, double&)> VehicleBodyRotationCallback;
typedef boost::function<void(InsSpeed&, double&)> InsSpeedCallback;
typedef boost::function<void(RawImu&, double&)> RawImuCallback;
typedef boost::function<void(RawImuShort&, double&)> RawImuShortCallback;
typedef boost::function<void(Position&, double&)> BestGpsPositionCallback;
typedef boost::function<void(BestLeverArm&, double&)> BestLeverArmCallback;
typedef boost::function<void(InsCovariance&, double&)> InsCovarianceCallback;
typedef boost::function<void(InsCovarianceShort&, double&)> InsCovarianceShortCallback;

// GPS Callbacks
typedef boost::function<void(UtmPosition&, double&)> BestUtmPositionCallback;
typedef boost::function<void(Velocity&, double&)> BestVelocityCallback;
typedef boost::function<void(PositionEcef&, double&)> BestPositionEcefCallback;
typedef boost::function<void(Dop&, double&)> PseudorangeDopCallback;
typedef boost::function<void(Dop&, double&)> RtkDopCallback;
typedef boost::function<void(BaselineEcef&, double&)> BaselineEcefCallback;
typedef boost::function<void(IonosphericModel&, double&)> IonosphericModelCallback;
typedef boost::function<void(RangeMeasurements&, double&)> RangeMeasurementsCallback;
typedef boost::function<void(CompressedRangeMeasurements&, double&)> CompressedRangeMeasurementsCallback;
typedef boost::function<void(GpsEphemeris&, double&)> GpsEphemerisCallback;
typedef boost::function<void(RawEphemeris&, double&)> RawEphemerisCallback;
typedef boost::function<void(RawAlmanac&, double&)> RawAlmanacCallback;
typedef boost::function<void(Almanac&, double&)> AlmanacCallback;
typedef boost::function<void(SatellitePositions&, double&)> SatellitePositionsCallback;
typedef boost::function<void(SatelliteVisibility&, double&)> SatelliteVisibilityCallback;
typedef boost::function<void(TimeOffset&, double&)> TimeOffsetCallback;
typedef boost::function<void(TrackStatus&, double&)> TrackingStatusCallback;
typedef boost::function<void(ReceiverHardwareStatus&, double&)> ReceiverHardwareStatusCallback;
typedef boost::function<void(Position&, double&)> BestPositionCallback;
typedef boost::function<void(Position&, double&)> BestPseudorangePositionCallback;
typedef boost::function<void(Position&, double&)> RtkPositionCallback;
typedef boost::function<void(Inspvax&,double&)>InspvaxCallback;//add by wendao
typedef boost::function<void(BestGnss&,double&)>BestGnssCallback;//add by wendao
typedef boost::function<void(CorrImuShort&,double&)>CorrImuShortCallback;//add by wendao


const uint32_t crc32Table[]={
	         0,0x77073096,0xee0e612c,0x990951ba, 0x76dc419,0x706af48f,0xe963a535,0x9e6495a3,
	 0xedb8832,0x79dcb8a4,0xe0d5e91e,0x97d2d988, 0x9b64c2b,0x7eb17cbd,0xe7b82d07,0x90bf1d91,
	0x1db71064,0x6ab020f2,0xf3b97148,0x84be41de,0x1adad47d,0x6ddde4eb,0xf4d4b551,0x83d385c7,
	0x136c9856,0x646ba8c0,0xfd62f97a,0x8a65c9ec,0x14015c4f,0x63066cd9,0xfa0f3d63,0x8d080df5,
	0x3b6e20c8,0x4c69105e,0xd56041e4,0xa2677172,0x3c03e4d1,0x4b04d447,0xd20d85fd,0xa50ab56b,
	0x35b5a8fa,0x42b2986c,0xdbbbc9d6,0xacbcf940,0x32d86ce3,0x45df5c75,0xdcd60dcf,0xabd13d59,
	0x26d930ac,0x51de003a,0xc8d75180,0xbfd06116,0x21b4f4b5,0x56b3c423,0xcfba9599,0xb8bda50f,
	0x2802b89e,0x5f058808,0xc60cd9b2,0xb10be924,0x2f6f7c87,0x58684c11,0xc1611dab,0xb6662d3d,
	0x76dc4190, 0x1db7106,0x98d220bc,0xefd5102a,0x71b18589, 0x6b6b51f,0x9fbfe4a5,0xe8b8d433,
	0x7807c9a2, 0xf00f934,0x9609a88e,0xe10e9818,0x7f6a0dbb, 0x86d3d2d,0x91646c97,0xe6635c01,
	0x6b6b51f4,0x1c6c6162,0x856530d8,0xf262004e,0x6c0695ed,0x1b01a57b,0x8208f4c1,0xf50fc457,
	0x65b0d9c6,0x12b7e950,0x8bbeb8ea,0xfcb9887c,0x62dd1ddf,0x15da2d49,0x8cd37cf3,0xfbd44c65,
	0x4db26158,0x3ab551ce,0xa3bc0074,0xd4bb30e2,0x4adfa541,0x3dd895d7,0xa4d1c46d,0xd3d6f4fb,
	0x4369e96a,0x346ed9fc,0xad678846,0xda60b8d0,0x44042d73,0x33031de5,0xaa0a4c5f,0xdd0d7cc9,
	0x5005713c,0x270241aa,0xbe0b1010,0xc90c2086,0x5768b525,0x206f85b3,0xb966d409,0xce61e49f,
	0x5edef90e,0x29d9c998,0xb0d09822,0xc7d7a8b4,0x59b33d17,0x2eb40d81,0xb7bd5c3b,0xc0ba6cad,
	0xedb88320,0x9abfb3b6, 0x3b6e20c,0x74b1d29a,0xead54739,0x9dd277af, 0x4db2615,0x73dc1683,
	0xe3630b12,0x94643b84, 0xd6d6a3e,0x7a6a5aa8,0xe40ecf0b,0x9309ff9d, 0xa00ae27,0x7d079eb1,
	0xf00f9344,0x8708a3d2,0x1e01f268,0x6906c2fe,0xf762575d,0x806567cb,0x196c3671,0x6e6b06e7,
	0xfed41b76,0x89d32be0,0x10da7a5a,0x67dd4acc,0xf9b9df6f,0x8ebeeff9,0x17b7be43,0x60b08ed5,
	0xd6d6a3e8,0xa1d1937e,0x38d8c2c4,0x4fdff252,0xd1bb67f1,0xa6bc5767,0x3fb506dd,0x48b2364b,
	0xd80d2bda,0xaf0a1b4c,0x36034af6,0x41047a60,0xdf60efc3,0xa867df55,0x316e8eef,0x4669be79,
	0xcb61b38c,0xbc66831a,0x256fd2a0,0x5268e236,0xcc0c7795,0xbb0b4703,0x220216b9,0x5505262f,
	0xc5ba3bbe,0xb2bd0b28,0x2bb45a92,0x5cb36a04,0xc2d7ffa7,0xb5d0cf31,0x2cd99e8b,0x5bdeae1d,
	0x9b64c2b0,0xec63f226,0x756aa39c, 0x26d930a,0x9c0906a9,0xeb0e363f,0x72076785, 0x5005713,
	0x95bf4a82,0xe2b87a14,0x7bb12bae, 0xcb61b38,0x92d28e9b,0xe5d5be0d,0x7cdcefb7, 0xbdbdf21,
	0x86d3d2d4,0xf1d4e242,0x68ddb3f8,0x1fda836e,0x81be16cd,0xf6b9265b,0x6fb077e1,0x18b74777,
	0x88085ae6,0xff0f6a70,0x66063bca,0x11010b5c,0x8f659eff,0xf862ae69,0x616bffd3,0x166ccf45,
	0xa00ae278,0xd70dd2ee,0x4e048354,0x3903b3c2,0xa7672661,0xd06016f7,0x4969474d,0x3e6e77db,
	0xaed16a4a,0xd9d65adc,0x40df0b66,0x37d83bf0,0xa9bcae53,0xdebb9ec5,0x47b2cf7f,0x30b5ffe9,
	0xbdbdf21c,0xcabac28a,0x53b39330,0x24b4a3a6,0xbad03605,0xcdd70693,0x54de5729,0x23d967bf,
	0xb3667a2e,0xc4614ab8,0x5d681b02,0x2a6f2b94,0xb40bbe37,0xc30c8ea1,0x5a05df1b,0x2d02ef8d};

class Novatel
{
public:
	Novatel();
	~Novatel();

	/*!
	 * Connects to the Novatel receiver given a serial port.
	 *
	 * @param port Defines which serial port to connect to in serial mode.
	 * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
	 *
	 * @throws ConnectionFailedException connection attempt failed.
	 * @throws UnknownErrorCodeException unknown error code returned.
	 */
	 bool Connect(std::string port, int baudrate=115200, bool search=true);

   /*!
    * Disconnects from the serial port
    */
    void Disconnect();

  //! Indicates if a connection to the receiver has been established.
  bool IsConnected() {return is_connected_;}

  /*!

     * Pings the GPS to determine if it is properly connected
     *
     * This method sends a ping to the GPS and waits for a response.
     *
     * @param num_attempts The number of times to ping the device
     * before giving up
     * @param timeout The time in milliseconds to wait for each reponse
     *
     * @return True if the GPS was found, false if it was not.
     */
     bool Ping(int num_attempts=5);


     /*!
      * Pings the GPS to determine if it is properly connected
      *
      * This method sends a ping to the GPS and waits for a response.
      *
      * @param num_attempts The number of times to ping the device
      * before giving up
      * @param timeout The time in milliseconds to wait for each reponse
      *
      * @return True if the GPS was found, false if it was not.
      */
     void set_time_handler(GetTimeCallback time_handler) {
         this->time_handler_ = time_handler;
     }

    void setLogDebugCallback(LogMsgCallback debug_callback){log_debug_=debug_callback;};
    void setLogInfoCallback(LogMsgCallback info_callback){log_info_=info_callback;};
    void setLogWarningCallback(LogMsgCallback warning_callback){log_warning_=warning_callback;};
    void setLogErrorCallback(LogMsgCallback error_callback){log_error_=error_callback;};

    /*!
     * Request the given list of logs from the receiver.
     * Format: "[LOGNAME][MESSAGETYPE] [PORT] [LOGTYPE] [PERIOD] ..."
     * [MESSAGETYPE] - [A]=ASCII
     *               - [B]=Binary
     *               . [empty]=Abreviated ASCII
     * log_string format: "BESTUTMB ONTIME 1.0; BESTVELB ONTIME 1.0"
     */
    void ConfigureLogs(std::string log_string);
    void Unlog(std::string log); //!< Stop logging a specified log
    void UnlogAll(); //!< Stop logging all logs that aren't set with HOLD parameter

    /*!
     * SaveConfiguration() saves the current receiver configuration
     * in nonvolatile memory
     */
    void SaveConfiguration();

    void ConfigureInterfaceMode(std::string com_port,  
      std::string rx_mode, std::string tx_mode);

    void ConfigureBaudRate(std::string com_port, int baudrate);

    void SetBaudRate(int baudrate, std::string com_port="COM1");

    bool SendCommand(std::string cmd_msg, bool wait_for_ack=true);
    bool SendMessage(uint8_t* msg_ptr, size_t length);

    /*!
     * SetSvElevationCutoff
     * Sets the elevation cut-off angle. Svs below this angle
     * are not automatically searched for and are not used
     * in the position calculation. Angles < 5 deg are not
     * recommended except in specific situations
     * (Angle = +-90 deg)
     */
    bool SetSvElevationAngleCutoff(float angle);

    /*!
     * Pseudrange/Delta-Phase filter (PDPFILTER)- smooths positions
     * and bridges gaps in GPS coverage. Enabled by default on
     * OEMStar receiver.
     */
    void PDPFilterDisable();
    void PDPFilterEnable();
    void PDPFilterReset();
    void PDPModeConfigure(PDPMode mode, PDPDynamics dynamics);

    /*!
     * SetPositionTimeout (POSTIMEOUT) sets the timeout value for the
     * position calculation. In position logs, the position_type field
     * is set to NONE when this timeout expires (0 - 86400 sec)
     */
    void SetPositionTimeout(uint32_t seconds);

    bool SetInitialPosition(double latitude, double longitude, double height);
    bool SetInitialTime(uint32_t gps_week, double gps_seconds);
    bool InjectAlmanac(Almanac almanac);
    /*!
     * SetL1CarrierSmoothing sets the amount of smoothing to be performed on
     * code measurements. L2 smoothing is available in OEMV receivers, but
     * NOT in OEMStar Firmaware receivers.
     *      l1_time_constant : 2<= time constant <= 2000 [sec]
     *      l2_time_constant : 5<= time constant <= 2000 [sec] (firmware default = 100)
     */
    bool SetCarrierSmoothing(uint32_t l1_time_constant, uint32_t l2_time_constant);

    bool HardwareReset();
    /*!
     * HotStartReset
     * Restarts the GPS receiver, initialized with
     * Ephemeris, Almanac, Position, Time, etc.
     */
    bool HotStartReset();
    /*!
     * WarmStartReset
     * Restarts the GPS receiver, initialized with
     * Ephemeris, Almanac, NOT Position and Time info
     */
    bool WarmStartReset();
    /*!
     * ColdStartReset
     * Restarts the GPS receiver, initialized without
     * any initial or aiding data.
     */
    bool ColdStartReset();

    void SendRawEphemeridesToReceiver(RawEphemerides raw_ephemerides);

    /*!
     * Requests version information from the receiver
     *
     * This requests the VERSION message from the receiver and
     * uses the result to populate the receiver capapbilities
     *
     * @return True if the GPS was found, false if it was not.
     */
	bool UpdateVersion();

    bool ConvertLLaUTM(double Lat, double Long, double *northing, double *easting, int *zone, bool *north);

    void ReadFromFile(unsigned char* buffer, unsigned int length);

    // Set data callbacks
    void set_best_gps_position_callback(BestGpsPositionCallback handler){
        best_gps_position_callback_=handler;};
    void set_best_lever_arm_callback(BestLeverArmCallback handler){
        best_lever_arm_callback_=handler;};
    void set_best_position_callback(BestPositionCallback handler){
        best_position_callback_=handler;};
    void set_best_utm_position_callback(BestUtmPositionCallback handler){
        best_utm_position_callback_=handler;};
    void set_best_velocity_callback(BestVelocityCallback handler){
        best_velocity_callback_=handler;};
    void set_best_position_ecef_callback(BestPositionEcefCallback handler){
        best_position_ecef_callback_=handler;};
    void set_ins_position_velocity_attitude_callback(InsPositionVelocityAttitudeCallback handler){
        ins_position_velocity_attitude_callback_=handler;};
    void set_ins_position_velocity_attitude_short_callback(InsPositionVelocityAttitudeShortCallback handler){
        ins_position_velocity_attitude_short_callback_=handler;};
   //add by wendao   
    void set_inspvax_callback(InspvaxCallback handler){
        inspvax_callback_=handler;};
    void set_bestgnss_callback(BestGnssCallback handler){
        bestgnss_callback_=handler;};
        
    void set_corrImu_short_callback(CorrImuShortCallback handler){
    	corrImu_short_callback_=handler;}
           
        
        
    void set_vehicle_body_rotation_callback(VehicleBodyRotationCallback handler){
        vehicle_body_rotation_callback_=handler;};
    void set_ins_speed_callback(InsSpeedCallback handler){
        ins_speed_callback_=handler;};
    void set_raw_imu_callback(RawImuCallback handler){
        raw_imu_callback_=handler;};
    void set_raw_imu_short_callback(RawImuShortCallback handler){
        raw_imu_short_callback_=handler;};
    void set_ins_covariance_callback(InsCovarianceCallback handler){
        ins_covariance_callback_=handler;};
    void set_ins_covariance_short_callback(InsCovarianceShortCallback handler){
        ins_covariance_short_callback_=handler;};
    void set_pseudorange_dop_callback(PseudorangeDopCallback handler){
        pseudorange_dop_callback_=handler;};
    void set_rtk_dop_callback(RtkDopCallback handler){
        rtk_dop_callback_=handler;};
    void set_baseline_ecef_callback(BaselineEcefCallback handler){
        baseline_ecef_callback_=handler;};
    void set_ionospheric_model_callback(IonosphericModelCallback handler){
        ionospheric_model_callback_=handler;};
    void set_range_measurements_callback(RangeMeasurementsCallback handler){
        range_measurements_callback_=handler;};
    void set_compressed_range_measurements_callback(CompressedRangeMeasurementsCallback handler){
        compressed_range_measurements_callback_=handler;};
    void set_gps_ephemeris_callback(GpsEphemerisCallback handler){
        gps_ephemeris_callback_=handler;};
    void set_raw_ephemeris_callback(RawEphemerisCallback handler){
        raw_ephemeris_callback_=handler;};
    void set_raw_almanc_callback(RawAlmanacCallback handler){
        raw_almanac_callback_=handler;};
    void set_almanac_callback(AlmanacCallback handler){
        almanac_callback_=handler;};
    void set_satellite_positions_callback(SatellitePositionsCallback handler){
        satellite_positions_callback_=handler;};
    void set_satellite_visibility_callback(SatelliteVisibilityCallback handler){
        satellite_visibility_callback_=handler;};
    void set_time_offset_callback(TimeOffsetCallback handler){
        time_offset_callback_=handler;};
    void set_tracking_status_callback(TrackingStatusCallback handler){
        tracking_status_callback_=handler;};
    void set_receiver_hardware_status_callback(ReceiverHardwareStatusCallback handler){
        receiver_hardware_status_callback_=handler;};
    void set_best_pseudorange_position_callback(BestPseudorangePositionCallback handler){
        best_pseudorange_position_callback_=handler;};
    void set_rtk_position_callback(RtkPositionCallback handler){
        rtk_position_callback_=handler;};

    void set_raw_msg_callback(RawMsgCallback handler) {
        raw_msg_callback_=handler;};
    RawEphemerides test_ephems_;
private:

  bool Connect_(std::string port, int baudrate);


	/*!
	 * Starts a thread to continuously read from the serial port.
	 *
	 * Starts a thread that runs 'ReadSerialPort' which constatly reads
	 * from the serial port.  When valid data is received, parse and then
	 *  the data callback functions are called.
	 *
	 * @see xbow440::DataCallback, xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StopReading
	 */
	void StartReading();

	/*!
	 * Starts the thread that reads from the serial port
	 *
	 * @see xbow440::XBOW440::ReadSerialPort, xbow440::XBOW440::StartReading
	 */
	void StopReading();

	/*!
	 * Method run in a seperate thread that continuously reads from the
	 * serial port.  When a complete packet is received, the parse
	 * method is called to process the data
	 *
	 * @see xbow440::XBOW440::Parse, xbow440::XBOW440::StartReading, xbow440::XBOW440::StopReading
	 */
	void ReadSerialPort();

	void BufferIncomingData(unsigned char *message, unsigned int length);

	/*!
	 * Parses a packet of data from the GPS.  The
	 */
	void ParseBinary(unsigned char *message, size_t length, BINARY_LOG_TYPE message_id);

	bool ParseVersion(std::string packet);

	void UnpackCompressedRangeData(const CompressedRangeData &cmp,
	                                     RangeData           &rng);

	double UnpackCompressedPsrStd(const uint16_t &val) const;

	double UnpackCompressedAccumulatedDoppler(
	    const CompressedRangeData &cmp,
	    const double              &uncmpPsr) const;

    bool SendBinaryDataToReceiver(uint8_t* msg_ptr, size_t length);

    unsigned long CRC32Value(int i);
    unsigned long CalculateBlockCRC32 ( unsigned long ulCount, /* Number of bytes in the data block */
                                        unsigned char *ucBuffer ); /* Data block */
	unsigned long CalculateBlockCRC32ByTable ( unsigned long ulCount, /* Number of bytes in the data block */
											const unsigned char *ucBuffer, /* Data block */
											const uint32_t* table = crc32Table);
    //////////////////////////////////////////////////////
    // Serial port reading members
    //////////////////////////////////////////////////////
	//! Serial port object for communicating with sensor
	serial::Serial *serial_port_;
	//! shared pointer to Boost thread for listening for data from novatel
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;  //!< True if the read thread is running, false otherwise.

    //////////////////////////////////////////////////////
    // Diagnostic Callbacks
    //////////////////////////////////////////////////////
    HandleAcknowledgementCallback handle_acknowledgement_;
    LogMsgCallback log_debug_;
    LogMsgCallback log_info_;
    LogMsgCallback log_warning_;
    LogMsgCallback log_error_;

    GetTimeCallback time_handler_; //!< Function pointer to callback function for timestamping


    //////////////////////////////////////////////////////
    // New Data Callbacks
    //////////////////////////////////////////////////////
    RawMsgCallback raw_msg_callback_;

    BestGpsPositionCallback best_gps_position_callback_;
    BestLeverArmCallback best_lever_arm_callback_;
    BestPositionCallback best_position_callback_;
    BestUtmPositionCallback best_utm_position_callback_;
    BestVelocityCallback best_velocity_callback_;
    BestPositionEcefCallback best_position_ecef_callback_;
    InsPositionVelocityAttitudeCallback ins_position_velocity_attitude_callback_;
    InsPositionVelocityAttitudeShortCallback ins_position_velocity_attitude_short_callback_;
    
    InspvaxCallback inspvax_callback_; //add by wendao
    BestGnssCallback bestgnss_callback_;
    CorrImuShortCallback corrImu_short_callback_; //add by wendao
    
    VehicleBodyRotationCallback vehicle_body_rotation_callback_;
    InsSpeedCallback ins_speed_callback_;
    RawImuCallback raw_imu_callback_;
    RawImuShortCallback raw_imu_short_callback_;
    InsCovarianceCallback ins_covariance_callback_;
    InsCovarianceShortCallback ins_covariance_short_callback_;

    // GPS Callbacks
    PseudorangeDopCallback pseudorange_dop_callback_;
    RtkDopCallback rtk_dop_callback_;
    BaselineEcefCallback baseline_ecef_callback_;
    IonosphericModelCallback ionospheric_model_callback_;
    RangeMeasurementsCallback range_measurements_callback_;
    CompressedRangeMeasurementsCallback compressed_range_measurements_callback_;
    GpsEphemerisCallback gps_ephemeris_callback_;
    RawEphemerisCallback raw_ephemeris_callback_;
    AlmanacCallback almanac_callback_;
    RawAlmanacCallback raw_almanac_callback_;
    SatellitePositionsCallback satellite_positions_callback_;
    SatelliteVisibilityCallback satellite_visibility_callback_;
    TimeOffsetCallback time_offset_callback_;
    TrackingStatusCallback tracking_status_callback_;
    ReceiverHardwareStatusCallback receiver_hardware_status_callback_;
    BestPseudorangePositionCallback best_pseudorange_position_callback_;
    RtkPositionCallback rtk_position_callback_;



	//////////////////////////////////////////////////////
	// Incoming data buffers
	//////////////////////////////////////////////////////
	unsigned char data_buffer_[MAX_NOUT_SIZE];	//!< data currently being buffered to read
	unsigned char* data_read_;		//!< used only in BufferIncomingData - declared here for speed
	size_t bytes_remaining_;	//!< bytes remaining to be read in the current message
	size_t short_message_length_;
	size_t buffer_index_;		//!< index into data_buffer_
	size_t header_length_;	//!< length of the current header being read
	bool reading_acknowledgement_;	//!< true if an acknowledgement is being received
    bool reading_reset_complete_;   //!< true if an {COM#} message confirming receiver reset if complete
	double read_timestamp_; 		//!< time stamp when last serial port read completed
	double parse_timestamp_;		//!< time stamp when last parse began
	BINARY_LOG_TYPE message_id_;	//!< message id of message currently being buffered

    //////////////////////////////////////////////////////
    // Mutex's
    //////////////////////////////////////////////////////
    boost::condition_variable ack_condition_;
    boost::mutex ack_mutex_;
    bool ack_received_;     //!< true if an acknowledgement has been received from the GPS
    boost::condition_variable reset_condition_;
    boost::mutex reset_mutex_;
    bool waiting_for_reset_complete_;     //!< true if GPS has finished resetting and is ready for input

    bool is_connected_; //!< indicates if a connection to the receiver has been established
	//////////////////////////////////////////////////////
    // Receiver information and capabilities
	//////////////////////////////////////////////////////
	std::string protocol_version_;		//!< Receiver version, OEM4, OEMV, OEM6, or UNKNOWN
	std::string serial_number_; //!< Receiver serial number
	std::string hardware_version_; //!< Receiver hardware version
	std::string software_version_; //!< Receiver software version
	std::string model_;				//!< Receiver model number

	bool l2_capable_; //!< Can the receiver handle L1 and L2 or just L1?
	bool raw_capable_; //!< Can the receiver output raw measurements?
	bool rtk_capable_; //!< Can the receiver compute RT2 and/or RT20 positions?
	bool glonass_capable_; //!< Can the receiver receive GLONASS frequencies?
	bool span_capable_;  //!< Is the receiver a SPAN unit?
	
	bool is_short_header;


};
}
#endif
