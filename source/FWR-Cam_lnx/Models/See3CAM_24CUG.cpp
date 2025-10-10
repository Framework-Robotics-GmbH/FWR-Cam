//------------------------------------------------------------------------------
// FWR-Cam Library
// Copyright (c) 2025 Framework Robotics GmbH
// 
// Authors: Michael Lampert
// 
// Licensed under the BSD 3-Clause License.
// You may use, modify, and redistribute this file under those terms.
// 
// See the LICENSE file in the project root for full license information.
//------------------------------------------------------------------------------

#include "FWR-Cam_lnx/Models/See3CAM_24CUG.hpp"

// needs flags: -std=c++20 -ludev -pthread

#include <iostream>        // For clog, cout
#include <string_view>
#include <thread>          // For this_thread::sleep_for, this_thread::yield
#include <chrono>          // For chrono::milliseconds, chrono::steady_clock
#include <utility>         // For exchange
#include <algorithm>       // For min
#include <cstring>         // For C string functions like memset, strcmp, strerror
#include <cstdio>          // For printf
// #include <cstdlib>         // For atoi

#include <fcntl.h>         // For open, O_RDWR, O_NONBLOCK
#include <unistd.h>        // For close, read, write
#include <sys/select.h>    // For select, fd_set, struct timeval
#include <sys/ioctl.h>     // For FIONREAD
#include <cerrno>          // For errno
#include <sys/types.h>     // For various data types (optional, sometimes included by other headers)
#include <sys/stat.h>      // For file status (optional, sometimes included by <fcntl.h>)

#include <libudev.h>       // For udev functions and structures

#include <magic_enum/magic_enum.hpp>





namespace FWR::Cam_lnx::Models
{


using namespace std;
using namespace magic_enum;


static constexpr string_view ECON_VID                    = "2560";
static constexpr string_view CAMERA_PID                  = "c128";

static constexpr uint8_t CAMERA_CONTROL_24CUG            = 0xA8;

static constexpr uint8_t GET_SPECIALEFFECT_24CUG         = 0x03;
static constexpr uint8_t SET_SPECIALEFFECT_24CUG         = 0x04;

static constexpr uint8_t GET_DENOISE_24CUG               = 0x05;
static constexpr uint8_t SET_DENOISE_24CUG               = 0x06;

static constexpr uint8_t GET_EXP_ROI_MODE_24CUG          = 0x07;
static constexpr uint8_t SET_EXP_ROI_MODE_24CUG          = 0x08;

static constexpr uint8_t GET_Q_FACTOR_24CUG              = 0x09;
static constexpr uint8_t SET_Q_FACTOR_24CUG              = 0x0A;

static constexpr uint8_t GET_FLIP_MODE_24CUG             = 0x0B;
static constexpr uint8_t SET_FLIP_MODE_24CUG             = 0x0C;

static constexpr uint8_t SET_FLIP_BOTHFLIP_ENABLE_24CUG  = 0x03;
static constexpr uint8_t SET_FLIP_HORZFLIP_24CUG         = 0x01;
static constexpr uint8_t SET_FLIP_VERTFLIP_24CUG         = 0x02;
static constexpr uint8_t SET_FLIP_BOTHFLIP_DISABLE_24CUG = 0x00;

static constexpr uint8_t GET_FACE_DETECT_RECT_24CUG      = 0x0D;
static constexpr uint8_t SET_FACE_DETECT_RECT_24CUG      = 0x0E;

static constexpr uint8_t GET_SMILE_DETECTION             = 0x0F;
static constexpr uint8_t SET_SMILE_DETECTION             = 0x10;

static constexpr uint8_t GET_EXPOSURE_COMPENSATION_24CUG = 0x11;
static constexpr uint8_t SET_EXPOSURE_COMPENSATION_24CUG = 0x12;

static constexpr uint8_t GET_FRAME_RATE_24CUG            = 0x13;
static constexpr uint8_t SET_FRAME_RATE_24CUG            = 0x14;

static constexpr uint8_t GET_STROBE_CONTROL_24CUG        = 0x15;
static constexpr uint8_t SET_STROBE_CONTROL_24CUG        = 0x16;

static constexpr uint8_t GET_FLICKER_CONRTOL_24CUG       = 0x17;
static constexpr uint8_t SET_FLICKER_CONTROL_24CUG       = 0x18;

static constexpr uint8_t GET_BURST_LENGTH_24CUG          = 0x19;
static constexpr uint8_t SET_BURST_LENGTH_24CUG          = 0x1A;

static constexpr uint8_t GET_STREAM_MODE_24CUG           = 0x1B;
static constexpr uint8_t SET_STREAM_MODE_24CUG           = 0x1C;

static constexpr uint8_t SET_TO_DEFAULT_24CUG            = 0xFF;

static constexpr uint8_t SET_FAIL                        = 0x00;
static constexpr uint8_t SET_SUCCESS                     = 0x01;
static constexpr uint8_t GET_FAIL                        = 0x00;
static constexpr uint8_t GET_SUCCESS                     = 0x01;




See3CAM_24CUG:: See3CAM_24CUG(string const& serialNo) noexcept
 :  V4L2Cam(serialNo)
{}

See3CAM_24CUG::~See3CAM_24CUG() {}


string_view const& See3CAM_24CUG::produceVendorID()  noexcept { return   ECON_VID; }
string_view const& See3CAM_24CUG::produceProductID() noexcept { return CAMERA_PID; }


bool See3CAM_24CUG::gatherSerialNumbers(vector<string>& serials) noexcept
{
    return V4L2Cam::gatherSerialNumbers(   ECON_VID
                                       , CAMERA_PID
                                       , serials
                                       );
}


bool See3CAM_24CUG::_locateDeviceNodeAndInitialize( udev       * uDev
                                                  , udev_device* parentDev
                                                  )
{
    bool initialized{false};
    
    udev_enumerate* enumerate = udev_enumerate_new(uDev);
    if ( !enumerate ) {
        clog << "See3CAM_24CUG::_locateDeviceNodeAndInitialize: "
                "udev_enumerate_new failed"
             << endl;
        
        return false;
    }
    
    udev_enumerate_add_match_subsystem(enumerate, "hidraw");
    udev_enumerate_add_match_parent(enumerate, parentDev);
    udev_enumerate_scan_devices(enumerate);
    
    udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    
    udev_list_entry* dev_list_entry;
    udev_list_entry_foreach(dev_list_entry, devices)
    {
        const char *path = udev_list_entry_get_name(dev_list_entry);
        if ( !path ) [[unlikely]]
            continue;
        
        udev_device* dev = udev_device_new_from_syspath(uDev, path);
        if ( !dev )
            continue;
        
        char const* dev_path = udev_device_get_devnode(dev);
        if ( !dev_path ) {
            udev_device_unref(dev);
            continue;
        }
        
        FD_t fd{xopen(dev_path, O_RDWR | O_NONBLOCK)};
        if ( fd < 0 ) {
            clog << "See3CAM_24CUG::_locateDeviceNodeAndInitialize: Could not "
                    "open hidraw device path"
                 << endl;
            
            udev_device_unref(dev);
            continue;
        }
        
        hidPath = dev_path;
        
        hidFD = make_shared<FD_t>(move(fd));
        
        if ( !drainHidInput(BUFFER_LENGTH) ) [[unlikely]]
            clog << "See3CAM_24CUG::_locateDeviceNodeAndInitialize: Could not "
                    "ensure HIDraw device interface is ready to be used!"
                 << endl;
        else
        {
            initializeSettings();
            initialized = true;
        }
        
        closeHIDFD();
        
        udev_device_unref(dev);
        break;
    }
    
    udev_enumerate_unref(enumerate);
    
    if ( !initialized )
            clog << "See3CAM_24CUG::_locateDeviceNodeAndInitialize: Could not "
                    "find hidraw device node"
                 << endl;
    
    return initialized;
    
}

void See3CAM_24CUG::initializeSettings() {
    fetchEffectMode();
    fetchDeNoiseValue();
    fetchAutoExpoModeAndROI();
    fetchExposureCompensation();
    fetchBurstLength();
    fetchQFactor();
    fetchMirrorMode();
    fetchFramerate();
    fetchFaceDetectMode();
    fetchSmileDetectMode();
    fetchFlickerDetectMode();
    fetchFlashMode();
    fetchStreamMode();
}


shared_ptr<V4L2Cam::FD_t> See3CAM_24CUG::produceHIDFD()
{
    if ( hidPath.empty() ) [[unlikely]]
        return shared_ptr<FD_t>();
    if ( hidFD && *hidFD )
        return hidFD;
    else
        return make_shared<FD_t>(xopen( hidPath.c_str()
                                      , O_RDWR | O_NONBLOCK
                                      )
                                );
}

bool See3CAM_24CUG::openHIDFD() {
    if ( hidPath.empty() ) [[unlikely]]
        return false;
    if ( !hidFD || !*hidFD )
        hidFD = make_shared<FD_t>(xopen( hidPath.c_str()
                                       , O_RDWR | O_NONBLOCK
                                       )
                                 );
    
    return (bool)hidFD && (bool)*hidFD;
}

void See3CAM_24CUG::closeHIDFD() {
    if ( hidFD && *hidFD )
        hidFD->close_fd();
    
    hidFD.reset();
}


void See3CAM_24CUG::_uninitialize() {
    closeHIDFD();
    hidPath.clear();
    
    effectModeSource = ssrc::UNKNOWN;
    effectMode.reset();
    
    deNoiseValueSource = ssrc::UNKNOWN;
    deNoiseValue.reset();
    
    autoExpoModeAndROISource = ssrc::UNKNOWN;
    autoExpoMode.reset();
    autoExpoROIxCoord.reset();
    autoExpoROIyCoord.reset();
    autoExpoROISize.reset();
    
    exposureCompensationSource = ssrc::UNKNOWN;
    exposureCompensation.reset();
    
    burstLengthSource = ssrc::UNKNOWN;
    burstLength.reset();
    
    qFactorSource = ssrc::UNKNOWN;
    qFactor.reset();
    
    mirrorModeSource = ssrc::UNKNOWN;
    mirrorMode.reset();
    
    framerateSource = ssrc::UNKNOWN;
    framerate.reset();
    
    faceDetectModeSource = ssrc::UNKNOWN;
    faceRectMode.reset();
    faceEmbedMode.reset();
    faceOverlayRectMode.reset();
    
    smileDetectModeSource = ssrc::UNKNOWN;
    smileDetectMode.reset();
    smileEmbedMode.reset();
    
    flickerDetectModeSource = ssrc::UNKNOWN;
    flickerDetectMode.reset();
    
    flashModeSource = ssrc::UNKNOWN;
    flashMode.reset();
    
    streamModeSource = ssrc::UNKNOWN;
    streamMode.reset();
    streamModeFunctionLock.reset();
    
    initialized = false;
}


void See3CAM_24CUG::initializeBuffers()
{
    // Initialize input and output buffers
    memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));
    memset( g_in_packet_buf, 0x00, sizeof( g_in_packet_buf));
}


/**
 * brief sendHidCmd - Sending hid command and get reply back
 * param outBuf - Buffer that fills to send into camera
 * param inBuf  - Buffer to get reply back
 * param len    - Buffer length
 * return success/failure
 * */
bool See3CAM_24CUG::sendHidCmd( uint8_t* outBuf
                              , uint8_t*  inBuf
                              , uint32_t    len
                              )
{
    using namespace chrono;
    
    auto fd_ptr = produceHIDFD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
    {
        errno = 0;
        return false;
    }
    
//     if ( !drainHidInput(len) ) [[unlikely]]
//     {
//         clog << "See3CAM_24CUG::sendHidCmd: couldn't pre-drain HID interface!"
//              << endl;
//         
//         return false;
//     }
    
    ssize_t written_bytes = 0;
    
    while ( written_bytes < static_cast<ssize_t>(len) )
    {
        ssize_t result = write( *fd_ptr
                              , outBuf + written_bytes
                              ,    len - written_bytes
                              );
        
        if ( result < 0 )
        {
            int const errNo = errno;
            
            if ( errNo == EINTR || errNo == EAGAIN )
            {
                this_thread::sleep_for(milliseconds(1));
                continue;
            }
            else if ( errNo == EIO )
            {
                // Optionally implement a retry mechanism here
                clog << "See3CAM_24CUG::sendHidCmd: Write I/O error: "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
            else
            {
                clog << "See3CAM_24CUG::sendHidCmd: Write error: "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
        }
        
        written_bytes += result;
    }
    
//     auto const end_time = steady_clock::now() + seconds(1);
//     
//     // Wait for data availability using select
//     while ( true )
//     {
//         auto now = steady_clock::now();
//         
//         if ( now >= end_time ) {
//             clog << "See3CAM_24CUG::sendHidCmd: Timeout waiting for data from "
//                     "HID device."
//                  << endl;
//             
//             errno = 0;
//             return false;
//         }
//         
//         timeval tv{};
//         fd_set  rfds;
//         FD_ZERO(&rfds);
//         FD_SET(static_cast<int>(*fd_ptr), &rfds);
//         
//         auto const remaining_time = duration_cast<microseconds>(end_time - now);
//         tv.tv_sec  = static_cast<     time_t>(remaining_time.count() / 1'000'000);
//         tv.tv_usec = static_cast<suseconds_t>(remaining_time.count() % 1'000'000);
//         
//         clog << "See3CAM_24CUG::sendHidCmd: try to `select` ..."
//              << endl;
//         
//         int32_t select_result = select( *fd_ptr + 1
//                                       , &rfds
//                                       , nullptr
//                                       , nullptr
//                                       , &tv
//                                       );
//         
//         if ( select_result < 0 )
//         {
//             int const errNo = errno;
//             
//             if ( errNo == EINTR )
//             {
//                 this_thread::sleep_for(milliseconds(1));
//                 
//                 continue;
//             }
//             
//             clog << "See3CAM_24CUG::sendHidCmd: Select error: "
//                  << strerror(errNo)
//                  << endl;
//             
//             errno = errNo;
//             return false;
//         }
//         else if ( select_result == 0 )
//             continue; // just run into timeout test at loop start
//         
//         // Data is available for reading
//         break;
//     }
    
    auto const end_time = steady_clock::now() + seconds(1);
    
    while ( true )
    {
        auto now = steady_clock::now();
        
        if ( now >= end_time ) [[unlikely]]
        {
            clog << "See3CAM_24CUG::sendHidCmd: Timeout waiting for response from "
                    "HID device!"
                 << endl;
            
            errno = 0;
            return false;
        }
        
        ssize_t const read_result = read( *fd_ptr
                                        , inBuf
                                        , len
                                        );
        
        if ( read_result < 0 )
        {
            int const errNo = errno;
            
            if ( errNo == EINTR || errNo == EAGAIN || EWOULDBLOCK )
            {
                this_thread::sleep_for(milliseconds(1));
                
                continue;
            }
            
            if ( errNo == EIO )
            {
                // Optionally implement a retry mechanism here
                clog << "See3CAM_24CUG::sendHidCmd: Read I/O error: "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
            else
            {
                clog << "See3CAM_24CUG::sendHidCmd: Read error: "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
        }
        else if ( read_result == 0 )
        {
            clog << "See3CAM_24CUG::sendHidCmd: HID device disconnected "
                    "during read."
                 << endl;
            
            errno = 0;
            return false;
        }
        
        break;
    }
    
    return true;
}


bool See3CAM_24CUG::drainHidInput(size_t reportLen)
{
    using namespace chrono;
    
    auto fd_ptr = produceHIDFD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
    {
        errno = 0;
        return true; // nothing to drain if we don't have a fd yet
    }
    
    
    vector<uint8_t> buf;
    buf.resize(reportLen);
    
    while ( true )
    {
        ssize_t n = read(*fd_ptr, buf.data(), buf.size());
        
        if ( n > 0 )
            // drained one report, loop again
            continue;
        
        if ( n == 0 )
            // device disappeared
            return false;
        
        if ( errno == EAGAIN || errno == EWOULDBLOCK )
            // nothing left to read → drained
            break;
        
        if ( errno == EINTR )
            continue;
        
        // hard error
        return false;
    }
    
    // Budget expired: we drained as much as we could. Treat as success.
    errno = 0;
    return true;
}




/* ************************** */
/* Resolution and PixelFormat */
/* ************************** */

bool See3CAM_24CUG::_checkResolution(uint32_t const _width, uint32_t const _height)
{
    for ( auto const& res : RESOLUTIONS )
        if ( _width == res[0] && _height == res[1] )
            return true;
    
    return false;
}

bool See3CAM_24CUG::_checkPixelFormat(uint32_t const _pixelFormat)
{
    switch(static_cast<SupportedPixelFormat>(_pixelFormat))
    {
        // TODO keep in-sync with SupportedPixelFormat
        case SupportedPixelFormat::UYVY : return true;
        case SupportedPixelFormat::MJPEG: return true;
        default                         : return false;
    }
}



/* ********** */
/* EffectMode */
/* ********** */

V4L2Cam::ssrc See3CAM_24CUG::tellEffectModeSource() {
    return effectModeSource;
}

bool See3CAM_24CUG::giveEffectMode(uint8_t& _effectMode)
{
    if ( !effectMode.has_value() ) [[unlikely]]
        return false;
    
    _effectMode = enum_integer(effectMode.value());
    
    return true;
}

bool See3CAM_24CUG::takeEffectMode(uint8_t const _effectMode)
{
    auto mode = enum_cast<EffectMode>(_effectMode);
    
    if ( !mode.has_value() ) [[unlikely]]
        return false;
    
    effectMode = mode;
    
    effectModeSource = ssrc::GIVEN;
    
    return true;
}

// no checkEffectMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchEffectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_SPECIALEFFECT_24CUG;
    
    effectModeSource = ssrc::FETCHED;

    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_SPECIALEFFECT_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        effectMode = enum_cast<EffectMode>(g_in_packet_buf[2]);
        
        return true;
    } else {
        effectMode.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyEffectMode()
{
    if ( !effectMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_SPECIALEFFECT_24CUG;
    g_out_packet_buf[3] = enum_integer(effectMode.value());
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_SPECIALEFFECT_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ************ */
/* DeNoiseValue */
/* ************ */

V4L2Cam::ssrc See3CAM_24CUG::tellDeNoiseValueSource() {
    return deNoiseValueSource;
}

bool See3CAM_24CUG::giveDeNoiseValue(uint8_t &     _deNoiseValue)
{
    if ( !checkDeNoiseValue() ) [[unlikely]]
        return false;
    
    _deNoiseValue = deNoiseValue.value();
    
    return true;
}

bool See3CAM_24CUG::takeDeNoiseValue(uint8_t const _deNoiseValue)
{
    if (    _deNoiseValue < DENOISEVALUE_MIN
         || _deNoiseValue > DENOISEVALUE_MAX
       )
        return false;
    
    deNoiseValue = _deNoiseValue;
    
    deNoiseValueSource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkDeNoiseValue()
{
    return    deNoiseValue.has_value()
           && deNoiseValue.value() >= DENOISEVALUE_MIN
           && deNoiseValue.value() <= DENOISEVALUE_MAX;
}

bool See3CAM_24CUG::fetchDeNoiseValue()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_DENOISE_24CUG;
    
    deNoiseValueSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_DENOISE_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        deNoiseValue = g_in_packet_buf[2];
        if ( !checkDeNoiseValue() )
            deNoiseValue.reset();
        
        return true;
    } else {
        deNoiseValue.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyDeNoiseValue()
{
    if ( !checkDeNoiseValue() )
        return false;
    
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_DENOISE_24CUG;
    g_out_packet_buf[3] = deNoiseValue.value();
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_DENOISE_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ************ */
/* AutoExpoMode */
/* ************ */

V4L2Cam::ssrc See3CAM_24CUG::tellAutoExpoModeAndROISource() {
    return autoExpoModeAndROISource;
}

bool See3CAM_24CUG::giveAutoExpoModeAndROI( uint8_t & _autoExpoMode )
{
    if ( !checkAutoExpoModeAndROI() ) [[unlikely]]
        return false;
    
    _autoExpoMode = enum_integer(autoExpoMode.value());
    
    return true;
}

bool See3CAM_24CUG::giveAutoExpoModeAndROI( uint8_t  &     _autoExpoMode
                                          , uint16_t const _frameWidth
                                          , uint16_t const _frameHeight
                                          , uint16_t &     _roiXCoord
                                          , uint16_t &     _roiYCoord
                                          , uint8_t  &     _roiSize
                                          )
{
    if ( !checkAutoExpoModeAndROI() ) [[unlikely]]
        return false;
    
    if ( autoExpoMode.value() != AutoExpoMode::MANUAL )
    {
        _autoExpoMode = enum_integer(autoExpoMode.value());
        _roiXCoord    = uint16_t{};
        _roiYCoord    = uint16_t{};
        _roiSize      = uint8_t {};
        
        return true;
    }
    
    if (    _frameWidth  <= 1
         || _frameHeight <= 1
       )
        return false;
    
    constexpr double camLow  = 0;
    constexpr double camHigh = 255;
    
    // TODO someone review this reverse calculations
    constexpr double roiXLow   = 0;
    double    const  roiXHigh  = _frameWidth - 1;
    uint16_t  const  roiXCoord = (   ((double)autoExpoROIxCoord.value() - camLow)
                                   / (camHigh - camLow)                           )
                               * (roiXHigh - roiXLow)
                               +  roiXLow;
    
    constexpr double roiYLow   = 0;
    double    const  roiYHigh  = _frameWidth - 1;
    uint16_t  const  roiYCoord = (   ((double)autoExpoROIxCoord.value() - camLow)
                                   / (camHigh - camLow)                           )
                               * (roiYHigh - roiYLow)
                               +  roiYLow;
    
    _autoExpoMode = enum_integer(AutoExpoMode::MANUAL);
    _roiXCoord    = roiXCoord;
    _roiYCoord    = roiYCoord;
    _roiSize      = autoExpoROISize.value();
    
    return true;
}

bool See3CAM_24CUG::takeAutoExpoModeAndROI( uint8_t  const _autoExpoMode )
{
    auto mode = enum_cast<AutoExpoMode>(_autoExpoMode);
    
    if (    !mode.has_value()
         ||  mode.value() == AutoExpoMode::MANUAL
       ) [[unlikely]]
        return false;
    
    autoExpoMode     = mode;
    autoExpoROIxCoord.reset();
    autoExpoROIyCoord.reset();
    autoExpoROISize  .reset();
    
    autoExpoModeAndROISource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::takeAutoExpoModeAndROI( uint8_t  const _autoExpoMode
                                          , uint16_t const _frameWidth
                                          , uint16_t const _frameHeight
                                          , uint16_t const _roiXCoord
                                          , uint16_t const _roiYCoord
                                          , uint8_t  const _roiSize
                                          )
{
    auto mode = enum_cast<AutoExpoMode>(_autoExpoMode);
    
    if ( !mode.has_value() ) [[unlikely]]
        return false;
    
    if ( mode.value() != AutoExpoMode::MANUAL ) {
        autoExpoMode     = mode;
        autoExpoROIxCoord.reset();
        autoExpoROIyCoord.reset();
        autoExpoROISize  .reset();
        
        autoExpoModeAndROISource = ssrc::GIVEN;
        
        return true;
    }
    
    //((Input - InputLow) / (InputHigh - InputLow)) * (OutputHigh - OutputLow) + OutputLow
    // map frame width and height to [0, 255]
    
    if (    _frameWidth  <= 1
         || _frameHeight <= 1
         || _frameWidth  <= _roiXCoord
         || _frameHeight <= _roiYCoord
         || _roiSize     == 0   // guess-work TODO would 0 be ok?
       ) [[unlikely]]
        return false;
    
    constexpr double camLow  = 0;
    constexpr double camHigh = 255;
    
    constexpr double roiXLow   = 0;
    double    const  roiXHigh  = _frameWidth - 1;
    uint8_t   const  camXCoord = (   (_roiXCoord - roiXLow)
                                   / ( roiXHigh  - roiXLow) )
                               * (camHigh - camLow)
                               +  camLow;
    
    constexpr double roiYLow   = 0;
    double    const  roiYHigh  = _frameHeight - 1;
    uint8_t   const  camYCoord = (   (_roiYCoord - roiYLow)
                                   / ( roiYHigh  - roiYLow) )
                               * (camHigh - camLow)
                               +  camLow;
    
    autoExpoMode      = AutoExpoMode::MANUAL;
    autoExpoROIxCoord = camXCoord;
    autoExpoROIyCoord = camYCoord;
    autoExpoROISize   = _roiSize;
    
    autoExpoModeAndROISource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkAutoExpoModeAndROI()
{
    if ( !autoExpoMode.has_value() ) [[unlikely]]
        return false;
    
    if ( autoExpoMode.value() != AutoExpoMode::MANUAL )
    {
        autoExpoROIyCoord.reset();
        autoExpoROISize  .reset();
        autoExpoROIxCoord.reset();
        
        return true;
    }
    
    if (    !autoExpoROIxCoord.has_value()
         || !autoExpoROIyCoord.has_value()
         || !autoExpoROISize  .has_value()
         ||  autoExpoROISize.value() == 0   // guess-work TODO would 0 be ok?
       )
        return false;
    
    return true;
}

bool See3CAM_24CUG::fetchAutoExpoModeAndROI()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_EXP_ROI_MODE_24CUG;
    
    autoExpoModeAndROISource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_EXP_ROI_MODE_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        // HACK not tested, if the cam actually returns the coords
        // (weren't read from the buffer in e-con's example code)
        // TODO someone test it
        autoExpoMode      = enum_cast<AutoExpoMode>(g_in_packet_buf[2]);
        autoExpoROIxCoord = g_in_packet_buf[3];
        autoExpoROIyCoord = g_in_packet_buf[4];
        autoExpoROISize   = g_in_packet_buf[5];
        if ( !checkAutoExpoModeAndROI() )
        {
            autoExpoMode     .reset();
            autoExpoROIxCoord.reset();
            autoExpoROIyCoord.reset();
            autoExpoROISize  .reset();
        }
        
        return true;
    } else {
        autoExpoMode     .reset();
        autoExpoROIxCoord.reset();
        autoExpoROIyCoord.reset();
        autoExpoROISize  .reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyAutoExpoModeAndROI()
{
    if ( !checkAutoExpoModeAndROI() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_EXP_ROI_MODE_24CUG;
    g_out_packet_buf[3] = enum_integer(autoExpoMode.value());
    
    if ( autoExpoMode.value() == AutoExpoMode::MANUAL )
    {
        g_out_packet_buf[4] = autoExpoROIxCoord.value();
        g_out_packet_buf[5] = autoExpoROIyCoord.value();
        g_out_packet_buf[6] = autoExpoROISize  .value();
    }
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_EXP_ROI_MODE_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ******************** */
/* ExposureCompensation */
/* ******************** */

V4L2Cam::ssrc See3CAM_24CUG::tellExposureCompensationSource() {
    return exposureCompensationSource;
}

bool See3CAM_24CUG::giveExposureCompensation(uint32_t &     _exposureCompensation)
{
    if ( !checkExposureCompensation() ) [[unlikely]]
        return false;
    
    _exposureCompensation = exposureCompensation.value();
    
    return true;
}

bool See3CAM_24CUG::takeExposureCompensation(uint32_t const _exposureCompensation)
{
    if (    _exposureCompensation < EXPOSURECOMP_MIN
         || _exposureCompensation > EXPOSURECOMP_MAX
       )
        return false;
    
    exposureCompensation = _exposureCompensation;
    
    exposureCompensationSource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkExposureCompensation()
{
    return    exposureCompensation.has_value()
           && exposureCompensation.value() >= EXPOSURECOMP_MIN
           && exposureCompensation.value() <= EXPOSURECOMP_MAX;
}

bool See3CAM_24CUG::fetchExposureCompensation()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_EXPOSURE_COMPENSATION_24CUG;
    
    exposureCompensationSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG 
         && g_in_packet_buf[1] == GET_EXPOSURE_COMPENSATION_24CUG 
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        exposureCompensation = (((uint32_t)g_in_packet_buf[2]) << 24)
                             | (((uint32_t)g_in_packet_buf[3]) << 16)
                             | (((uint32_t)g_in_packet_buf[4]) <<  8)
                             | (((uint32_t)g_in_packet_buf[5]) <<  0);
        if ( !checkExposureCompensation() )
            exposureCompensation.reset();
        
        return true;
    } else {
        exposureCompensation.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyExposureCompensation()
{
    if ( !checkExposureCompensation() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_EXPOSURE_COMPENSATION_24CUG;
    g_out_packet_buf[3] = (uint8_t)((exposureCompensation.value() >> 24) & 0xFF);
    g_out_packet_buf[4] = (uint8_t)((exposureCompensation.value() >> 16) & 0xFF);
    g_out_packet_buf[5] = (uint8_t)((exposureCompensation.value() >>  8) & 0xFF);
    g_out_packet_buf[6] = (uint8_t)((exposureCompensation.value() >>  0) & 0xFF);
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_EXPOSURE_COMPENSATION_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* *********** */
/* BurstLength */
/* *********** */

V4L2Cam::ssrc See3CAM_24CUG::tellBurstLengthSource() {
    return burstLengthSource;
}

bool See3CAM_24CUG::giveBurstLength(uint8_t &     _burstLength)
{
    if ( !burstLength.has_value() ) [[unlikely]]
        return false;
    
    _burstLength = burstLength.value();
    
    return true;
}

bool See3CAM_24CUG::takeBurstLength(uint8_t const _burstLength)
{
    burstLength = _burstLength;
    
    burstLengthSource = ssrc::GIVEN;
    
    return true;
}

// no chechBurstLength() - we know of no limits
// - e-con's example code didn't provide any

bool See3CAM_24CUG::fetchBurstLength()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_BURST_LENGTH_24CUG;
    
    burstLengthSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_BURST_LENGTH_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        burstLength = g_in_packet_buf[2];
        
        return true;
    } else {
        burstLength.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyBurstLength()
{
    if ( !burstLength.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_BURST_LENGTH_24CUG;
    g_out_packet_buf[3] = burstLength.value();
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_BURST_LENGTH_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ******* */
/* QFactor */
/* ******* */

V4L2Cam::ssrc See3CAM_24CUG::tellQFactorSource() {
    return qFactorSource;
}

bool See3CAM_24CUG::giveQFactor(uint8_t &     _qFactor)
{
    if ( !checkQFactor() ) [[unlikely]]
        return false;
    
    _qFactor = qFactor.value();
    
    return true;
}

bool See3CAM_24CUG::takeQFactor(uint8_t const _qFactor)
{
    if (    _qFactor < Q_FACTOR_MIN
         || _qFactor > Q_FACTOR_MAX
       )
        return false;
    
    qFactor = _qFactor;
    
    qFactorSource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkQFactor()
{
    return    qFactor.has_value()
           && qFactor.value() >= Q_FACTOR_MIN
           && qFactor.value() <= Q_FACTOR_MAX;
}

bool See3CAM_24CUG::fetchQFactor()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_Q_FACTOR_24CUG;
    
    qFactorSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_Q_FACTOR_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
    {
        qFactor = g_in_packet_buf[2];
        if ( !checkQFactor() )
            qFactor.reset();
        
        return true;
    } else {
        qFactor.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyQFactor()
{
    if ( !checkQFactor() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_Q_FACTOR_24CUG;
    g_out_packet_buf[3] = qFactor.value();
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_Q_FACTOR_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ********** */
/* MirrorMode */
/* ********** */

V4L2Cam::ssrc See3CAM_24CUG::tellMirrorModeSource() {
    return mirrorModeSource;
}

bool See3CAM_24CUG::giveMirrorMode( bool &     _horizontal
                                  , bool &     _vertical
                                  )
{
    if ( !mirrorMode.has_value() ) [[unlikely]]
        return false;
    
    _horizontal = enum_integer(mirrorMode.value())
                & enum_integer(MirrorMode::HORZ  );
    _vertical   = enum_integer(mirrorMode.value())
                & enum_integer(MirrorMode::VERT  );
    
    return true;
}

bool See3CAM_24CUG::takeMirrorMode( bool const _horizontal
                                  , bool const _vertical
                                  )
{
    mirrorMode = enum_cast<MirrorMode>(
          enum_integer( _horizontal ? MirrorMode::HORZ
                                                : MirrorMode::NONE )
        | enum_integer( _vertical   ? MirrorMode::VERT
                                                : MirrorMode::NONE )
    );
    
    mirrorModeSource = ssrc::GIVEN;
    
    return true;
}

// no checkMirrorMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchMirrorMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_FLIP_MODE_24CUG;
    
    mirrorModeSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_FLIP_MODE_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        mirrorMode = enum_cast<MirrorMode>(g_in_packet_buf[2]);
        
        return true;
    } else {
        mirrorMode.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyMirrorMode()
{
    if ( !mirrorMode.has_value() )
        return false;
    
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FLIP_MODE_24CUG;
    g_out_packet_buf[3] = enum_integer(mirrorMode.value());
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_FLIP_MODE_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ********* */
/* Framerate */
/* ********* */

V4L2Cam::ssrc See3CAM_24CUG::tellFramerateSource() {
    return framerateSource;
}

bool See3CAM_24CUG::giveFramerate(uint8_t &     _framerate)
{
    if ( !checkFramerate() ) [[unlikely]]
        return false;
    
    _framerate = framerate.value();
    
    return true;
}

bool See3CAM_24CUG::takeFramerate(uint8_t const _framerate)
{
    if (    _framerate < FRAMERATE_MIN
         || _framerate > FRAMERATE_MAX
       )
        return false;
    
    framerate = _framerate;
    
    framerateSource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkFramerate()
{
    return    framerate.has_value()
           && framerate.value() >= FRAMERATE_MIN
           && framerate.value() <= FRAMERATE_MAX;
}

bool See3CAM_24CUG::fetchFramerate()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_FRAME_RATE_24CUG;
    
    framerateSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_FRAME_RATE_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        framerate = g_in_packet_buf[2];
        if ( !checkFramerate() )
            framerate.reset();
        
        return true;
    } else {
        framerate.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyFramerate()
{
    if ( !checkFramerate() )
        return false;
    
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FRAME_RATE_24CUG;
    g_out_packet_buf[3] = framerate.value();
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_FRAME_RATE_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ************** */
/* FaceDetectMode */
/* ************** */

V4L2Cam::ssrc See3CAM_24CUG::tellFaceDetectModeSource() {
    return faceDetectModeSource;
}

bool See3CAM_24CUG::giveFaceDetectMode( bool &     _faceDetect
                                      , bool &     _embedData
                                      , bool &     _overlayRect
                                      )
{
    if ( !checkFaceDetectMode() ) [[unlikely]]
        return false;
    
    _faceDetect  = faceRectMode       .value() == FaceRectMode       ::ENABLE;
    _embedData   = faceEmbedMode      .value() == FaceEmbedMode      ::ENABLE;
    _overlayRect = faceOverlayRectMode.value() == FaceOverlayRectMode::ENABLE;
    
    return true;
}

bool See3CAM_24CUG::takeFaceDetectMode( bool const _faceDetect
                                      , bool const _embedData
                                      , bool const _overlayRect
                                      )
{
    faceRectMode        = _faceDetect
                        ? FaceRectMode::ENABLE
                        : FaceRectMode::DISABLE;
    faceEmbedMode       = _embedData
                        ? FaceEmbedMode::ENABLE
                        : FaceEmbedMode::DISABLE;
    faceOverlayRectMode = _overlayRect
                        ? FaceOverlayRectMode::ENABLE
                        : FaceOverlayRectMode::DISABLE;
    
    faceDetectModeSource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkFaceDetectMode()
{
    if (    !faceRectMode       .has_value()
         || !faceEmbedMode      .has_value()
         || !faceOverlayRectMode.has_value()
       )
    {
        faceRectMode       .reset();
        faceEmbedMode      .reset();
        faceOverlayRectMode.reset();
        
        return false;
    }
    
    return true;
}

bool See3CAM_24CUG::fetchFaceDetectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_FACE_DETECT_RECT_24CUG;
    
    faceDetectModeSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_FACE_DETECT_RECT_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        faceRectMode        = enum_cast<FaceRectMode       >(g_in_packet_buf[2]);
        faceEmbedMode       = enum_cast<FaceEmbedMode      >(g_in_packet_buf[3]);
        faceOverlayRectMode = enum_cast<FaceOverlayRectMode>(g_in_packet_buf[4]);
        checkFaceDetectMode();
        
        return true;
    } else {
        faceRectMode       .reset();
        faceEmbedMode      .reset();
        faceOverlayRectMode.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyFaceDetectMode()
{
    if ( !checkFaceDetectMode() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FACE_DETECT_RECT_24CUG;
    g_out_packet_buf[3] = enum_integer(faceRectMode       .value());
    g_out_packet_buf[4] = enum_integer(faceEmbedMode      .value());
    g_out_packet_buf[5] = enum_integer(faceOverlayRectMode.value());
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_FACE_DETECT_RECT_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* *************** */
/* SmileDetectMode */
/* *************** */

V4L2Cam::ssrc See3CAM_24CUG::tellSmileDetectModeSource() {
    return smileDetectModeSource;
}

bool See3CAM_24CUG::giveSmileDetectMode( bool &     _smileDetect
                                       , bool &     _embedData
                                       )
{
    if ( !checkSmileDetectMode() ) [[unlikely]]
        return false;
    
    _smileDetect = smileDetectMode.value() == SmileDetectMode::ENABLE;
    _embedData   = smileEmbedMode .value() == SmileEmbedMode ::ENABLE;
    
    return true;
}

bool See3CAM_24CUG::takeSmileDetectMode( bool const _smileDetect
                                       , bool const _embedData
                                       )
{
    smileDetectMode = _smileDetect
                    ? SmileDetectMode::ENABLE
                    : SmileDetectMode::DISABLE;
    smileEmbedMode  = _embedData
                    ? SmileEmbedMode::ENABLE
                    : SmileEmbedMode::DISABLE;
    
    smileDetectModeSource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkSmileDetectMode()
{
    if (    !smileDetectMode.has_value()
         || !smileEmbedMode .has_value()
       )
    {
        smileDetectMode.reset();
        smileEmbedMode .reset();
        
        return false;
    }
    
    return true;
}

bool See3CAM_24CUG::fetchSmileDetectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_SMILE_DETECTION;
    
    smileDetectModeSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_SMILE_DETECTION
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        smileDetectMode = enum_cast<SmileDetectMode>(g_in_packet_buf[2]);
        smileEmbedMode  = enum_cast<SmileEmbedMode >(g_in_packet_buf[4]);
        checkSmileDetectMode();
        
        return true;
    } else {
        smileDetectMode.reset();
        smileEmbedMode .reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applySmileDetectMode()
{
    if ( !checkSmileDetectMode() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_SMILE_DETECTION;
    g_out_packet_buf[3] = enum_integer(smileDetectMode.value());
    g_out_packet_buf[5] = enum_integer(smileEmbedMode .value());
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_SMILE_DETECTION
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ***************** */
/* FlickerDetectMode */
/* ***************** */

V4L2Cam::ssrc See3CAM_24CUG::tellFlickerDetectModeSource() {
    return flickerDetectModeSource;
}

bool See3CAM_24CUG::giveFlickerDetectMode(uint8_t &     _flickerMode)
{
    if ( !flickerDetectMode.has_value() ) [[unlikely]]
        return false;
    
    _flickerMode = enum_integer(flickerDetectMode.value());
    
    return true;
}

bool See3CAM_24CUG::takeFlickerDetectMode(uint8_t const _flickerMode)
{
    auto mode = enum_cast<FlickerDetectMode>(_flickerMode);
    
    if ( !mode.has_value() ) [[unlikely]]
        return false;
    
    flickerDetectMode = mode;
    
    flickerDetectModeSource = ssrc::GIVEN;
    
    return true;
}

// no checkFlickerDetectMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchFlickerDetectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_FLICKER_CONRTOL_24CUG;
    
    flickerDetectModeSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_FLICKER_CONRTOL_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        flickerDetectMode = enum_cast<FlickerDetectMode>(g_in_packet_buf[2]);
        
        return true;
    } else {
        flickerDetectMode.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyFlickerDetectMode()
{
    if ( !flickerDetectMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FLICKER_CONTROL_24CUG;
    g_out_packet_buf[3] = enum_integer(flickerDetectMode.value());
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_FLICKER_CONTROL_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ********* */
/* FlashMode */
/* ********* */

V4L2Cam::ssrc See3CAM_24CUG::tellFlashModeSource() {
    return flashModeSource;
}

bool See3CAM_24CUG::giveFlashMode(uint8_t &     _flashMode)
{
    if ( !flashMode.has_value() ) [[unlikely]]
        return false;
    
    _flashMode = enum_integer(flashMode.value());
    
    return true;
}

bool See3CAM_24CUG::takeFlashMode(uint8_t const _flashMode)
{
    auto mode = enum_cast<FlashMode>(_flashMode);
    
    if ( !mode.has_value() ) [[unlikely]]
        return false;
    
    flashMode = mode;
    
    flashModeSource = ssrc::GIVEN;
    
    return true;
}

// no checkFlashMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchFlashMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_STROBE_CONTROL_24CUG;
    
    flashModeSource = ssrc::FETCHED;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_STROBE_CONTROL_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        flashMode = enum_cast<FlashMode>(g_in_packet_buf[2]);
        
        return true;
    } else {
        flashMode.reset();
        
        return false;
    }
}

bool See3CAM_24CUG::applyFlashMode()
{
    if ( !flashMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_STROBE_CONTROL_24CUG;
    g_out_packet_buf[3] = enum_integer(flashMode.value());
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_STROBE_CONTROL_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}


/* ********** */
/* StreamMode */
/* ********** */

V4L2Cam::ssrc See3CAM_24CUG::tellStreamModeSource() {
    return streamModeSource;
}

bool See3CAM_24CUG::giveStreamMode( uint8_t &     _streamMode
                                  , bool    &     _autoFunctionLock
                                  )
{
    if ( !checkStreamMode() ) [[unlikely]]
        return false;
    
    _streamMode       = enum_integer(streamMode.value());
    _autoFunctionLock = streamModeFunctionLock.value();
    
    return true;
}

bool See3CAM_24CUG::takeStreamMode( uint8_t const _streamMode
                                  , bool    const _autoFunctionLock
                                  )
{
    auto mode = enum_cast<StreamMode>(_streamMode);
    
    if ( !mode.has_value() ) [[unlikely]]
        return false;
    
    streamMode             = mode;
    streamModeFunctionLock = _autoFunctionLock;
    
    streamModeSource = ssrc::GIVEN;
    
    return true;
}

bool See3CAM_24CUG::checkStreamMode()
{
    if (    !streamMode            .has_value()
         || !streamModeFunctionLock.has_value()
       )
    {
        streamMode            .reset();
        streamModeFunctionLock.reset();
        
        return false;
    }
    
    return true;
}

bool See3CAM_24CUG::fetchStreamMode()
{
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_STREAM_MODE_24CUG;
    
    streamModeSource = ssrc::FETCHED;

    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == GET_STREAM_MODE_24CUG
         && g_in_packet_buf[6] == GET_SUCCESS
       )
    {
        streamMode             = enum_cast
                                 <StreamMode>(g_in_packet_buf[2]);
        streamModeFunctionLock = (bool)g_in_packet_buf[3];
        checkStreamMode();
        
        return true;
    } else {
        streamMode            .reset();
        streamModeFunctionLock.reset();
        
        return true;
    }
}

bool See3CAM_24CUG::applyStreamMode()
{
    if ( !streamMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_STREAM_MODE_24CUG;
    g_out_packet_buf[3] = enum_integer(streamMode.value());
    g_out_packet_buf[4] = streamModeFunctionLock.value();
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_STREAM_MODE_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
        return true;
    else
        return false;
}



bool See3CAM_24CUG::setToDefault()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_TO_DEFAULT_24CUG;
    
    if (    sendHidCmd(g_out_packet_buf, g_in_packet_buf, BUFFER_LENGTH)
         && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
         && g_in_packet_buf[1] == SET_TO_DEFAULT_24CUG
         && g_in_packet_buf[6] == SET_SUCCESS
       )
    {
        initializeSettings();
        return true;
    }
    else
        return false;
}


} // namespace FWR::Cam_lnx::Models
