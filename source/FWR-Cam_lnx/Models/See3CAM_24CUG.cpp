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


static constexpr string_view VID_ECON                    = "2560";
static constexpr string_view PID_24CUG                   = "c128";

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

static constexpr uint8_t GET_SMILE_DETECTION_24CUG       = 0x0F;
static constexpr uint8_t SET_SMILE_DETECTION_24CUG       = 0x10;

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

static constexpr uint8_t GET_FIRMWARE_VERSION_24CUG	     = 0x40;

static constexpr uint8_t SET_FAIL                        = 0x00;
static constexpr uint8_t SET_SUCCESS                     = 0x01;
static constexpr uint8_t GET_FAIL                        = 0x00;
static constexpr uint8_t GET_SUCCESS                     = 0x01;




See3CAM_24CUG:: See3CAM_24CUG(string const& serialNo) noexcept
 :  V4L2Cam(serialNo)
{}

See3CAM_24CUG::~See3CAM_24CUG() {}


string_view const& See3CAM_24CUG::produceVendorID()  noexcept { return VID_ECON ; }
string_view const& See3CAM_24CUG::produceProductID() noexcept { return PID_24CUG; }


bool See3CAM_24CUG::gatherSerialNumbers(vector<string>& serials) noexcept
{
    return V4L2Cam::gatherSerialNumbers( VID_ECON
                                       , PID_24CUG
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
//         static int32_t thisRunsFD = -1;
//         
//         FD_t fd;
//         if ( thisRunsFD == -1 ) fd = thisRunsFD = xopen(dev_path, O_RDWR | O_NONBLOCK);
//         else                    fd = thisRunsFD;

        if ( fd < 0 ) {
            clog << "See3CAM_24CUG::_locateDeviceNodeAndInitialize: Could not "
                    "open hidraw device path: " << dev_path << "!"
                 << endl;
            
            udev_device_unref(dev);
            continue;
        }
        
        hidPath = dev_path;
        
        hidFD = make_shared<FD_t>(move(fd));
        
        string fwVersion{"<unknown>"};
        
        if ( fetchFWVersion() )
        {
            fwVersion = to_string(fwVersionMajor .value()) + "."
                      + to_string(fwVersionMinor1.value()) + "."
                      + to_string(fwVersionMinor2.value()) + "."
                      + to_string(fwVersionMinor3.value());
        }
        
        
        clog << "See3CAM_24CUG::_locateDeviceNodeAndInitialize: found ..."
             << "\n    hidFD           : " << to_string((int32_t)*hidFD)
             << "\n    hidPath         : " << hidPath
             << "\n    firmware version: " << fwVersion
             << endl;
        
        if ( !drainHidInput(BUFFER_LENGTH) ) [[unlikely]]
            clog << "See3CAM_24CUG::_locateDeviceNodeAndInitialize: Could not "
                    "ensure HIDraw device interface is ready to be used!"
                 << endl;
        else if ( state == State::UNINITIALIZED )
        {
            initializeSettings();
            initialized = true;
        }
        else /* state == State::DEVICE_KNOWN */
        {
            reapplySettings();
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

void See3CAM_24CUG::initializeSettings() noexcept
{
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

void See3CAM_24CUG::reapplySettings() noexcept
{
    if (           effectModeSource == ssrc::DEVICE ) applyEffectMode();
    if (         deNoiseValueSource == ssrc::DEVICE ) applyDeNoiseValue();
    if (   autoExpoModeAndROISource == ssrc::DEVICE ) applyAutoExpoModeAndROI();
    if ( exposureCompensationSource == ssrc::DEVICE ) applyExposureCompensation();
    if (          burstLengthSource == ssrc::DEVICE ) applyBurstLength();
    if (              qFactorSource == ssrc::DEVICE ) applyQFactor();
    if (           mirrorModeSource == ssrc::DEVICE ) applyMirrorMode();
    if (            framerateSource == ssrc::DEVICE ) applyFramerate();
    if (       faceDetectModeSource == ssrc::DEVICE ) applyFaceDetectMode();
    if (      smileDetectModeSource == ssrc::DEVICE ) applySmileDetectMode();
    if (    flickerDetectModeSource == ssrc::DEVICE ) applyFlickerDetectMode();
    if (            flashModeSource == ssrc::DEVICE ) applyFlashMode();
    if (           streamModeSource == ssrc::DEVICE ) applyStreamMode();
}


void See3CAM_24CUG::_logSetup(ostream& out) noexcept
{
    out << "\n    stream mode   : ";
    
    if ( !fetchStreamMode() ) [[unlikely]]
        out << "couldn't fetch stream mode from device";
    else
    {
        uint8_t sm{};
        bool    _{};
        
        if ( !giveStreamMode(sm, _) ) [[unlikely]]
            out << "no valid value!";
        else if ( static_cast<StreamMode>(sm) == StreamMode::MASTER )
            out << "MASTER";
        else if ( static_cast<StreamMode>(sm) == StreamMode::TRIGGER )
            out << "TRIGGER";
    }
    
    out << "\n    exposure comp.: ";
    uint32_t expComp{};
    if ( !fetchExposureCompensation() ) [[unlikely]]
        out << "couldn't fetch exposure compensation value from device";
    else if ( !giveExposureCompensation(expComp) ) [[unlikely]]
        out << "no valid value!";
    else
        out << to_string(expComp);
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
    
//     effectModeSource = ssrc::NONE;
//     effectMode.reset();
//     
//     deNoiseValueSource = ssrc::NONE;
//     deNoiseValue.reset();
//     
//     autoExpoModeAndROISource = ssrc::NONE;
//     autoExpoMode.reset();
//     autoExpoROIxCoord.reset();
//     autoExpoROIyCoord.reset();
//     autoExpoROISize.reset();
//     
//     exposureCompensationSource = ssrc::NONE;
//     exposureCompensation.reset();
//     
//     burstLengthSource = ssrc::NONE;
//     burstLength.reset();
//     
//     qFactorSource = ssrc::NONE;
//     qFactor.reset();
//     
//     mirrorModeSource = ssrc::NONE;
//     mirrorMode.reset();
//     
//     framerateSource = ssrc::NONE;
//     framerate.reset();
//     
//     faceDetectModeSource = ssrc::NONE;
//     faceRectMode.reset();
//     faceEmbedMode.reset();
//     faceOverlayRectMode.reset();
//     
//     smileDetectModeSource = ssrc::NONE;
//     smileDetectMode.reset();
//     smileEmbedMode.reset();
//     
//     flickerDetectModeSource = ssrc::NONE;
//     flickerDetectMode.reset();
//     
//     flashModeSource = ssrc::NONE;
//     flashMode.reset();
//     
//     streamModeSource = ssrc::NONE;
//     streamMode.reset();
//     streamModeFunctionLock.reset();
    
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
bool See3CAM_24CUG::sendHidCmd( std::string_view opName
                              , uint8_t*         outBuf
                              , uint8_t*          inBuf
                              , uint32_t         len
                              )
{
    using namespace chrono;
    
    auto fd_ptr = produceHIDFD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
    {
        clog << "See3Cam24CugNode::sendHidCmd: could not open device file to \""
             << opName << "\"!"
             << endl;
        
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
                clog << "See3CAM_24CUG::sendHidCmd: Write I/O error for \""
                     << opName << "\": "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
            else
            {
                clog << "See3CAM_24CUG::sendHidCmd: Write error for \""
                     << opName << "\": "
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
                    "HID device for \"" << opName << "\"!"
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
                clog << "See3CAM_24CUG::sendHidCmd: Read I/O error for \""
                     << opName << "\": "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
            else
            {
                clog << "See3CAM_24CUG::sendHidCmd: Read error for \""
                     << opName << "\": "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
        }
        else if ( read_result == 0 )
        {
            clog << "See3CAM_24CUG::sendHidCmd: HID device disconnected "
                    "during read for \""
                 << opName << "\"."
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
        int errNo = errno;
        
        if ( n > 0 )
            // drained one report, loop again
            continue;
        
        if ( n == 0 )
            // device disappeared
            return false;
        
        if ( errNo == EAGAIN || errNo == EWOULDBLOCK )
            // nothing left to read -> drained
            break;
        
        if ( errNo == EINTR )
            continue;
        
        clog << "See3CAM_24CUG::drainHidInput: error: " << strerror(errNo)
             << endl;
        
        // hard error
        return false;
    }
    
    // Budget expired: we drained as much as we could. Treat as success.
    errno = 0;
    return true;
}




/* ********* */
/* Framerate */
/* ********* */

bool See3CAM_24CUG::_requestFramerate(uint8_t fps) noexcept
{
    if ( !takeFramerate(fps) ) [[unlikely]]
        clog << "See3CAM_24CUG::_requestFramerate: didn't accept framerate."
             << endl;
    else if ( !applyFramerate() ) [[unlikely]]
        clog << "See3CAM_24CUG::_requestFramerate: couldn't apply set framerate "
                "to device."
             << endl;
    else
        return true;
    
    return false;
}

optional<uint8_t> See3CAM_24CUG::_produceFramerate() noexcept
{
    uint8_t fps{};
    
    if ( !fetchFramerate() ) [[unlikely]]
        clog << "See3CAM_24CUG::_produceFramerate: couldn't query device for "
                "framerate"
             << endl;
    else if ( !giveFramerate(fps) ) [[unlikely]]
        clog << "See3CAM_24CUG::_produceFramerate: don't have valid framerate "
                "to return"
             << endl;
    else
        return fps;
    
    return nullopt;
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

bool See3CAM_24CUG::_checkPixelFormat(PixelFormat const _pixelFormat)
{
    using  pfInt = std::underlying_type_t<PixelFormat>;
    using spfInt = std::underlying_type_t<SupportedPixelFormat>;
    
    return       static_cast< pfInt>(_pixelFormat)
              == static_cast<spfInt>(SupportedPixelFormat::UYVY)
           ||    static_cast< pfInt>(_pixelFormat)
              == static_cast<spfInt>(SupportedPixelFormat::UYVY);
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
    
    effectModeSource = ssrc::USER;
    
    return true;
}

// no checkEffectMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchEffectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_SPECIALEFFECT_24CUG;

    bool fetched =    sendHidCmd( "GET_SPECIALEFFECT_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_SPECIALEFFECT_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
        effectMode = enum_cast<EffectMode>(g_in_packet_buf[2]);
    else
        effectMode.reset();
    
    if ( effectMode ) effectModeSource = ssrc::DEVICE;
    else              effectModeSource = ssrc::NONE;
    
    if ( fetched && !effectMode ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchEffectMode: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return effectModeSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyEffectMode()
{
    if ( effectModeSource == ssrc::NONE || !effectMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_SPECIALEFFECT_24CUG;
    g_out_packet_buf[3] = enum_integer(effectMode.value());
    
    bool applied =    sendHidCmd( "SET_SPECIALEFFECT_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_SPECIALEFFECT_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;  
    
    if ( !applied ) [[unlikely]]
    {
        effectMode.reset();
        effectModeSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyEffectMode: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchEffectMode();
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
    
    deNoiseValueSource = ssrc::USER;
    
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
    
    bool fetched =    sendHidCmd( "GET_DENOISE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_DENOISE_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) { deNoiseValue       = (decltype(deNoiseValue)::value_type)
                                          (g_in_packet_buf[2]);
                     deNoiseValueSource = ssrc::DEVICE;
                   }
    else           { deNoiseValue.reset();
                     deNoiseValueSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkDeNoiseValue() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchDeNoiseValue: value reported by device not "
                "valid according to our domain knowledge!"
             << endl;
    
    return deNoiseValueSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyDeNoiseValue()
{
    if ( deNoiseValueSource == ssrc::NONE || !checkDeNoiseValue() )
        return false;
    
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_DENOISE_24CUG;
    g_out_packet_buf[3] = deNoiseValue.value();
    
    bool applied =    sendHidCmd( "SET_DENOISE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_DENOISE_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        deNoiseValue.reset();
        deNoiseValueSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyDeNoiseValue: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchDeNoiseValue();
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
    
    autoExpoModeAndROISource = ssrc::USER;
    
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
    
    if ( mode.value() != AutoExpoMode::MANUAL )
    {
        autoExpoMode     = mode;
        autoExpoROIxCoord.reset();
        autoExpoROIyCoord.reset();
        autoExpoROISize  .reset();
        
        autoExpoModeAndROISource = ssrc::USER;
        
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
    
    autoExpoModeAndROISource = ssrc::USER;
    
    return true;
}

bool See3CAM_24CUG::checkAutoExpoModeAndROI()
{
    if ( !autoExpoMode.has_value() ) [[unlikely]]
        return false;
    
    if ( autoExpoMode.value() != AutoExpoMode::MANUAL )
        return true;
    
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
    
    bool fetched =    sendHidCmd( "GET_EXP_ROI_MODE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_EXP_ROI_MODE_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
        autoExpoMode = enum_cast<AutoExpoMode>(g_in_packet_buf[2]);
    else
        autoExpoMode.reset();
    
    if ( autoExpoMode ) { // HACK not tested, if the cam actually returns the coords
                          // (weren't read from the buffer in e-con's example code)
                          // TODO someone test it
                          autoExpoROIxCoord = (decltype(autoExpoROIxCoord)::value_type)
                                              (g_in_packet_buf[3]);
                          autoExpoROIyCoord = (decltype(autoExpoROIyCoord)::value_type)
                                              (g_in_packet_buf[4]);
                          autoExpoROISize   = (decltype(autoExpoROISize  )::value_type)
                                              (g_in_packet_buf[5]);
                          autoExpoModeAndROISource = ssrc::DEVICE;
                        }
    else                { autoExpoROIxCoord.reset();
                          autoExpoROIyCoord.reset();
                          autoExpoROISize  .reset();
                          autoExpoModeAndROISource = ssrc::NONE;
                        }
    
    if ( fetched && !checkAutoExpoModeAndROI() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchAutoExpoModeAndROI: values reported by "
                "device not valid according to our domain knowledge!"
             << endl;
    
    return autoExpoModeAndROISource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyAutoExpoModeAndROI()
{
    if ( autoExpoModeAndROISource == ssrc::NONE || !checkAutoExpoModeAndROI() )
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
    
    bool applied =    sendHidCmd( "SET_EXP_ROI_MODE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_EXP_ROI_MODE_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        autoExpoMode     .reset();
        autoExpoROIxCoord.reset();
        autoExpoROIyCoord.reset();
        autoExpoROISize  .reset();
        autoExpoModeAndROISource = ssrc::NONE;
        
        clog << "V4L2Cam::applyAutoExpoModeAndROI: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchAutoExpoModeAndROI();
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
    
    exposureCompensationSource = ssrc::USER;
    
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
    
    bool fetched =    sendHidCmd( "GET_EXPOSURE_COMPENSATION_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG 
                   && g_in_packet_buf[1] == GET_EXPOSURE_COMPENSATION_24CUG 
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) { exposureCompensation = (decltype(exposureCompensation)::value_type)
                                            (   (((uint32_t)g_in_packet_buf[2]) << 24)
                                              | (((uint32_t)g_in_packet_buf[3]) << 16)
                                              | (((uint32_t)g_in_packet_buf[4]) <<  8)
                                              | (((uint32_t)g_in_packet_buf[5]) <<  0)
                                            );
                     exposureCompensationSource = ssrc::DEVICE;
                   }
    else           { exposureCompensation.reset();
                     exposureCompensationSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkExposureCompensation() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchExposureCompensation: value reported by "
                "device not valid according to our domain knowledge!"
             << endl;
    
    return exposureCompensationSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyExposureCompensation()
{
    if ( exposureCompensationSource == ssrc::NONE || !checkExposureCompensation() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_EXPOSURE_COMPENSATION_24CUG;
    g_out_packet_buf[3] = (uint8_t)((exposureCompensation.value() >> 24) & 0xFF);
    g_out_packet_buf[4] = (uint8_t)((exposureCompensation.value() >> 16) & 0xFF);
    g_out_packet_buf[5] = (uint8_t)((exposureCompensation.value() >>  8) & 0xFF);
    g_out_packet_buf[6] = (uint8_t)((exposureCompensation.value() >>  0) & 0xFF);
    
    bool applied =    sendHidCmd( "SET_EXPOSURE_COMPENSATION_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_EXPOSURE_COMPENSATION_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        exposureCompensation.reset();
        exposureCompensationSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyExposureCompensation: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchExposureCompensation();
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
    
    burstLengthSource = ssrc::USER;
    
    return true;
}

// no chechBurstLength() - we know of no limits
// - e-con's example code didn't provide any

bool See3CAM_24CUG::fetchBurstLength()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_BURST_LENGTH_24CUG;
    
    bool fetched =    sendHidCmd( "GET_BURST_LENGTH_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_BURST_LENGTH_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) { burstLength       = (decltype(burstLength)::value_type)
                                         (g_in_packet_buf[2]);
                     burstLengthSource = ssrc::DEVICE;
                   }
    else           { burstLength.reset();
                     burstLengthSource = ssrc::NONE;
                   }
    
    return burstLengthSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyBurstLength()
{
    if ( burstLengthSource == ssrc::NONE || !burstLength.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_BURST_LENGTH_24CUG;
    g_out_packet_buf[3] = burstLength.value();
    
    bool applied =    sendHidCmd( "SET_BURST_LENGTH_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_BURST_LENGTH_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        burstLength.reset();
        burstLengthSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyBurstLength: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchBurstLength();
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
    
    qFactorSource = ssrc::USER;
    
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
    
    bool fetched =    sendHidCmd( "GET_Q_FACTOR_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_Q_FACTOR_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( fetched ) { qFactor       = (decltype(qFactor)::value_type)
                                     (g_in_packet_buf[2]);
                     qFactorSource = ssrc::DEVICE;
                   }
    else           { qFactor.reset();
                     qFactorSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkQFactor() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchQFactor: value reported by device not "
                "valid according to our domain knowledge!"
             << endl;
    
    return qFactorSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyQFactor()
{
    if ( qFactorSource == ssrc::NONE || !checkQFactor() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_Q_FACTOR_24CUG;
    g_out_packet_buf[3] = qFactor.value();
    
    bool applied =    sendHidCmd( "SET_Q_FACTOR_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_Q_FACTOR_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        qFactor.reset();
        qFactorSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyQFactor: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchQFactor();
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
    
    mirrorModeSource = ssrc::USER;
    
    return true;
}

// no checkMirrorMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchMirrorMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_FLIP_MODE_24CUG;
    
    bool fetched =    sendHidCmd( "GET_FLIP_MODE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_FLIP_MODE_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
        mirrorMode = enum_cast<MirrorMode>(g_in_packet_buf[2]);
    else
        mirrorMode.reset();
    
    if ( mirrorMode ) mirrorModeSource = ssrc::DEVICE;
    else              mirrorModeSource = ssrc::NONE;
    
    if ( fetched && !mirrorMode ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchMirrorMode: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return mirrorModeSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyMirrorMode()
{
    if ( mirrorModeSource == ssrc::NONE || !mirrorMode.has_value() )
        return false;
    
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FLIP_MODE_24CUG;
    g_out_packet_buf[3] = enum_integer(mirrorMode.value());
    
    bool applied =    sendHidCmd( "SET_FLIP_MODE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_FLIP_MODE_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        mirrorMode.reset();
        mirrorModeSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyMirrorMode: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchMirrorMode();
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
    
    framerateSource = ssrc::USER;
    
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
    
    bool fetched =    sendHidCmd( "GET_FRAME_RATE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_FRAME_RATE_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) { framerate       = (decltype(framerate)::value_type)
                                       (g_in_packet_buf[2]);
                     framerateSource = ssrc::DEVICE;
                   }
    else           { framerate.reset();
                     framerateSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkFramerate() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchFramerate: value reported by device not "
                "valid according to our domain knowledge!"
             << endl;
    
    return framerateSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyFramerate()
{
    if ( framerateSource == ssrc::NONE || !checkFramerate() )
        return false;
    
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FRAME_RATE_24CUG;
    g_out_packet_buf[3] = framerate.value();
    
    bool applied =    sendHidCmd( "SET_FRAME_RATE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_FRAME_RATE_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        framerate.reset();
        framerateSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyFramerate: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchFramerate();
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
    
    faceDetectModeSource = ssrc::USER;
    
    return true;
}

bool See3CAM_24CUG::checkFaceDetectMode()
{
    return    faceRectMode       .has_value()
           && faceEmbedMode      .has_value()
           && faceOverlayRectMode.has_value();
}

bool See3CAM_24CUG::fetchFaceDetectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_FACE_DETECT_RECT_24CUG;
    
    bool fetched =    sendHidCmd( "GET_FACE_DETECT_RECT_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_FACE_DETECT_RECT_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
    {
        faceRectMode        = enum_cast<FaceRectMode       >(g_in_packet_buf[2]);
        faceEmbedMode       = enum_cast<FaceEmbedMode      >(g_in_packet_buf[3]);
        faceOverlayRectMode = enum_cast<FaceOverlayRectMode>(g_in_packet_buf[4]);
    }
    
    if ( !fetched || !checkFaceDetectMode() )
    {
        faceRectMode       .reset();
        faceEmbedMode      .reset();
        faceOverlayRectMode.reset();
    }
    
    if ( checkFaceDetectMode() ) faceDetectModeSource = ssrc::DEVICE;
    else                         faceDetectModeSource = ssrc::NONE;
    
    if ( fetched && !checkFaceDetectMode() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchFaceDetectMode: values reported by device "
                "not valid according to our domain knowledge!"
             << endl;
    
    return faceDetectModeSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyFaceDetectMode()
{
    if ( faceDetectModeSource == ssrc::NONE || !checkFaceDetectMode() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FACE_DETECT_RECT_24CUG;
    g_out_packet_buf[3] = enum_integer(faceRectMode       .value());
    g_out_packet_buf[4] = enum_integer(faceEmbedMode      .value());
    g_out_packet_buf[5] = enum_integer(faceOverlayRectMode.value());
    
    bool applied =    sendHidCmd( "SET_FACE_DETECT_RECT_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_FACE_DETECT_RECT_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        faceRectMode       .reset();
        faceEmbedMode      .reset();
        faceOverlayRectMode.reset();
        faceDetectModeSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyFaceDetectMode: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchFaceDetectMode();
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
    
    smileDetectModeSource = ssrc::USER;
    
    return true;
}

bool See3CAM_24CUG::checkSmileDetectMode()
{
    return    smileDetectMode.has_value()
           && smileEmbedMode .has_value();
}

bool See3CAM_24CUG::fetchSmileDetectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_SMILE_DETECTION_24CUG;
    
    bool fetched =    sendHidCmd( "GET_SMILE_DETECTION_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_SMILE_DETECTION_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
    {
        smileDetectMode = enum_cast<SmileDetectMode>(g_in_packet_buf[2]);
        smileEmbedMode  = enum_cast<SmileEmbedMode >(g_in_packet_buf[4]);
    }
    
    if ( !fetched || !checkSmileDetectMode() )
    {
        smileDetectMode.reset();
        smileEmbedMode .reset();
    }
    
    if ( checkSmileDetectMode() ) smileDetectModeSource = ssrc::DEVICE;
    else                          smileDetectModeSource = ssrc::NONE;
    
    if ( fetched && !checkSmileDetectMode() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchSmileDetectMode: values reported by device "
                "not valid according to our domain knowledge!"
             << endl;
    
    return smileDetectModeSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applySmileDetectMode()
{
    if ( smileDetectModeSource == ssrc::NONE || !checkSmileDetectMode() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_SMILE_DETECTION_24CUG;
    g_out_packet_buf[3] = enum_integer(smileDetectMode.value());
    g_out_packet_buf[5] = enum_integer(smileEmbedMode .value());
    
    bool applied =    sendHidCmd( "SET_SMILE_DETECTION_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_SMILE_DETECTION_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        smileDetectMode.reset();
        smileEmbedMode .reset();
        smileDetectModeSource = ssrc::NONE;
        
        clog << "V4L2Cam::applySmileDetectMode: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchSmileDetectMode();
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
    
    flickerDetectModeSource = ssrc::USER;
    
    return true;
}

// no checkFlickerDetectMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchFlickerDetectMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_FLICKER_CONRTOL_24CUG;
    
    bool fetched =    sendHidCmd( "GET_FLICKER_CONRTOL_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_FLICKER_CONRTOL_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
        flickerDetectMode = enum_cast<FlickerDetectMode>(g_in_packet_buf[2]);
    else
        flickerDetectMode.reset();
    
    if ( flickerDetectMode ) flickerDetectModeSource = ssrc::DEVICE;
    else                     flickerDetectModeSource = ssrc::NONE;
    
    if ( fetched && !flickerDetectMode ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchFlickerDetectMode: value reported by device "
                "not valid according to our domain knowledge!"
             << endl;
    
    return flickerDetectModeSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyFlickerDetectMode()
{
    if ( flickerDetectModeSource == ssrc::NONE || !flickerDetectMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_FLICKER_CONTROL_24CUG;
    g_out_packet_buf[3] = enum_integer(flickerDetectMode.value());
    
    bool applied =    sendHidCmd( "SET_FLICKER_CONTROL_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_FLICKER_CONTROL_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        flickerDetectMode.reset();
        flickerDetectModeSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyFlickerDetectMode: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchFlickerDetectMode();
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
    
    flashModeSource = ssrc::USER;
    
    return true;
}

// no checkFlashMode() - using magic_enum suffices

bool See3CAM_24CUG::fetchFlashMode()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_STROBE_CONTROL_24CUG;
    
    bool fetched =    sendHidCmd( "GET_STROBE_CONTROL_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_STROBE_CONTROL_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
        flashMode = enum_cast<FlashMode>(g_in_packet_buf[2]);
    else
        flashMode.reset();
    
    if ( flashMode ) flashModeSource = ssrc::DEVICE;
    else             flashModeSource = ssrc::NONE;
    
    if ( fetched && !flashMode ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchFlashMode: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return flashModeSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyFlashMode()
{
    if ( flashModeSource == ssrc::NONE || !flashMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_STROBE_CONTROL_24CUG;
    g_out_packet_buf[3] = enum_integer(flashMode.value());
    
    bool applied =    sendHidCmd( "SET_STROBE_CONTROL_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_STROBE_CONTROL_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        flashMode.reset();
        flashModeSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyFlashMode: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchFlashMode();
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
    
    streamModeSource = ssrc::USER;
    
    return true;
}

bool See3CAM_24CUG::checkStreamMode()
{
    return    streamMode            .has_value()
           && streamModeFunctionLock.has_value();
}

bool See3CAM_24CUG::fetchStreamMode()
{
    initializeBuffers();

    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = GET_STREAM_MODE_24CUG;

    bool fetched =    sendHidCmd( "GET_STREAM_MODE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == GET_STREAM_MODE_24CUG
                   && g_in_packet_buf[6] == GET_SUCCESS;
    
    if ( fetched ) [[  likely]]
        streamMode = enum_cast<StreamMode>(g_in_packet_buf[2]);
    else
        streamMode.reset();
    
    if ( streamMode ) { streamModeFunctionLock = (decltype(streamModeFunctionLock)::value_type)
                                                 (g_in_packet_buf[3]);
                        streamModeSource = ssrc::DEVICE;
                      }
    else              { streamModeFunctionLock.reset();
                        streamModeSource = ssrc::NONE;
                      }
    
    if ( fetched && !checkStreamMode() ) [[unlikely]]
        clog << "See3CAM_24CUG::fetchStreamMode: values reported by "
                "device not valid according to our domain knowledge!"
             << endl;
    
    return streamModeSource == ssrc::DEVICE;
}

bool See3CAM_24CUG::applyStreamMode()
{
    if ( streamModeSource == ssrc::NONE || !streamMode.has_value() )
        return false;
    
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_STREAM_MODE_24CUG;
    g_out_packet_buf[3] = enum_integer(streamMode.value());
    g_out_packet_buf[4] = streamModeFunctionLock.value();
    
    bool applied =    sendHidCmd( "SET_STREAM_MODE_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_STREAM_MODE_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
    {
        streamMode            .reset();
        streamModeFunctionLock.reset();
        streamModeSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyStreamMode: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchStreamMode();
}



bool See3CAM_24CUG::giveFWVersion( uint16_t& major
                                 , uint16_t& minor1
                                 , uint16_t& minor2
                                 , uint16_t& minor3
                                 )
{
    if (    !fwVersionMajor .has_value()
         || !fwVersionMinor1.has_value()
         || !fwVersionMinor2.has_value()
         || !fwVersionMinor3.has_value()
       ) [[unlikely]]
        return false;
    
    major  = fwVersionMajor .value();
    minor1 = fwVersionMinor1.value();
    minor2 = fwVersionMinor2.value();
    minor3 = fwVersionMinor3.value();
    
    return true;
}

bool See3CAM_24CUG::fetchFWVersion()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = GET_FIRMWARE_VERSION_24CUG;
    
    bool fetched =    sendHidCmd( "GET_FIRMWARE_VERSION_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == GET_FIRMWARE_VERSION_24CUG;
    
    if ( fetched ) { fwVersionMajor  =  g_in_packet_buf[1];
                     fwVersionMinor1 =  g_in_packet_buf[2];
                     fwVersionMinor2 = (g_in_packet_buf[3]<<8)+g_in_packet_buf[4];
                     fwVersionMinor3 = (g_in_packet_buf[5]<<8)+g_in_packet_buf[6];
                   }
    else           { fwVersionMajor .reset();
                     fwVersionMinor1.reset();
                     fwVersionMinor2.reset();
                     fwVersionMinor3.reset();
                   }
    
    return fetched;
}



bool See3CAM_24CUG::setToDefault()
{
    initializeBuffers();
    
    g_out_packet_buf[1] = CAMERA_CONTROL_24CUG;
    g_out_packet_buf[2] = SET_TO_DEFAULT_24CUG;
    
    bool applied =    sendHidCmd( "SET_TO_DEFAULT_24CUG"
                                , g_out_packet_buf
                                , g_in_packet_buf
                                , BUFFER_LENGTH
                                )
                   && g_in_packet_buf[0] == CAMERA_CONTROL_24CUG
                   && g_in_packet_buf[1] == SET_TO_DEFAULT_24CUG
                   && g_in_packet_buf[6] == SET_SUCCESS;
    
    if ( !applied ) [[unlikely]]
        return false;
    else
    {
        initializeSettings();
        return true;
    }
}


} // namespace FWR::Cam_lnx::Models
