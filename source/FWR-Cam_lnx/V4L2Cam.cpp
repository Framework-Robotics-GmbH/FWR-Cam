//------------------------------------------------------------------------------
// FWR-Cam Library
// Copyright (c) 2026 Framework Robotics GmbH
// 
// Authors: Michael Lampert
// 
// Licensed under the BSD 3-Clause License.
// You may use, modify, and redistribute this file under those terms.
// 
// See the LICENSE file in the project root for full license information.
//------------------------------------------------------------------------------

#include "FWR-Cam_lnx/V4L2Cam.hpp"

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <cstring>
#include <algorithm>
#include <filesystem>

#include <fcntl.h> /* low-level i/o */
// #include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <libusb-1.0/libusb.h>
#include <linux/usbdevice_fs.h>
// #include <stdbool.h>

#include <libudev.h>    // For udev functions and structures

#include <magic_enum/magic_enum.hpp>
#include <magic_enum/magic_enum_flags.hpp>





namespace magic_enum::customize
{


template <>
struct enum_range<FWR::Cam_lnx::V4L2CamData::XIOCTL_FLAGS>
{
    static constexpr int min = 0;
    static constexpr int max = 4;
};


} // namespace magic_enum::customize



namespace FWR::Cam_lnx
{


using namespace std;
using namespace magic_enum;
using namespace magic_enum::bitwise_operators;

namespace fs = std::filesystem;


std::string_view to_string_view(ErrorAction ea) noexcept
{
    return enum_name(ea);
}

std::string_view to_string_view(PixelFormat pf) noexcept
{
    switch ( pf )
    {
        case PixelFormat::INVALID: return "INVALID";
        case PixelFormat::RGB332 : return "RGB332" ;
        case PixelFormat::RGB565 : return "RGB565" ;
        case PixelFormat::RGB24  : return "RGB24"  ;
        case PixelFormat::RGB32  : return "RGB32"  ;
        case PixelFormat::GREY   : return "GREY"   ;
        case PixelFormat::YUYV   : return "YUYV"   ;
        case PixelFormat::UYVY   : return "UYVY"   ;
        case PixelFormat::MJPEG  : return "MJPEG"  ;
        case PixelFormat::H264   : return "H264"   ;
        case PixelFormat::NV12   : return "NV12"   ;
        case PixelFormat::NV21   : return "NV21"   ;
        case PixelFormat::YUV420 : return "YUV420" ;
        case PixelFormat::YVU420 : return "YVU420" ;
        
        default:
            return "<unknown enumerator>";
    }
}

uint32_t to_integer(PixelFormat pf) noexcept
{
    uint32_t ui = static_cast<uint32_t>(pf);
    
    switch ( pf )
    {
        case PixelFormat::INVALID: [[fallthrough]];
        case PixelFormat::RGB332 : [[fallthrough]];
        case PixelFormat::RGB565 : [[fallthrough]];
        case PixelFormat::RGB24  : [[fallthrough]];
        case PixelFormat::RGB32  : [[fallthrough]];
        case PixelFormat::GREY   : [[fallthrough]];
        case PixelFormat::YUYV   : [[fallthrough]];
        case PixelFormat::UYVY   : [[fallthrough]];
        case PixelFormat::MJPEG  : [[fallthrough]];
        case PixelFormat::H264   : [[fallthrough]];
        case PixelFormat::NV12   : [[fallthrough]];
        case PixelFormat::NV21   : [[fallthrough]];
        case PixelFormat::YUV420 : [[fallthrough]];
        case PixelFormat::YVU420 : return ui;
        
        default:
            return uint32_t{};
    }
}

PixelFormat to_PixelFormat(uint32_t ui) noexcept
{
    PixelFormat pf = static_cast<PixelFormat>(ui);
    
    switch ( pf )
    {
        case PixelFormat::INVALID: [[fallthrough]];
        case PixelFormat::RGB332 : [[fallthrough]];
        case PixelFormat::RGB565 : [[fallthrough]];
        case PixelFormat::RGB24  : [[fallthrough]];
        case PixelFormat::RGB32  : [[fallthrough]];
        case PixelFormat::GREY   : [[fallthrough]];
        case PixelFormat::YUYV   : [[fallthrough]];
        case PixelFormat::UYVY   : [[fallthrough]];
        case PixelFormat::MJPEG  : [[fallthrough]];
        case PixelFormat::H264   : [[fallthrough]];
        case PixelFormat::NV12   : [[fallthrough]];
        case PixelFormat::NV21   : [[fallthrough]];
        case PixelFormat::YUV420 : [[fallthrough]];
        case PixelFormat::YVU420 : return pf;
        
        default:
            return PixelFormat::INVALID;
    }
}

// only bumps up
static inline void update(ErrorAction& ea, ErrorAction val) noexcept
{
    if ( enum_integer(val) > enum_integer(ea) )
        ea = val;
}



static string retrieve_fd_path(int const fd)
{
    array<char, PATH_MAX> buf{};
    array<char, 64      > link{};
    
    int len = ::snprintf(link.data(), link.size(), "/proc/self/fd/%d", fd);
    
    if (    len <= 0
         || len >= static_cast<int>(link.size())
       ) [[unlikely]]
        return {};
    
    ssize_t r = ::readlink(link.data(), buf.data(), buf.size() - 1);
    
    if ( r < 0 ) [[unlikely]]
        return {};
    
    buf[r] = '\0';
    
    return string(buf.data());
}

void V4L2CamData::FD_t::close_fd()
{
    constexpr int32_t retry_count = 10;
    
    if ( value == -1 )
        return;
    
    // const string path = retrieve_fd_path(value);
    decSFSCfor(*this);
    
    int32_t delay_ms = 1;
    int32_t result;
    
    for ( int32_t attempt = 0
        ;    value   > -1
          && attempt < retry_count
        ; ++attempt
        )
    {
        errno = 0;
        
        result = ::close(value);
        
        if ( result == 0 )
        {
            value = -1;
            
            // clog << "V4L2CamData::FD_t::close_fd: fd closed (\"" << path << "\")"
            //      << endl;
        }
        else if ( errno == EINTR )
        {
            if ( attempt < retry_count - 1 )
            {
                this_thread::sleep_for(chrono::milliseconds(delay_ms++));
                continue;
            }
            else
                value = -1;
        }
        else
            value = -1;
    }
    
    if ( value > -1 ) [[unlikely]]
    {
        clog << "V4L2CamData::FD_t::close_fd: could not close the file descriptor. "
                "Just dropping it."
             << endl;
        
        value = -1;
    }
}


void V4L2CamData::FD_t::logSFSC() noexcept
{
    lock_guard<mutex> lck(sFSC_mtx);
    
    clog << "V4L2CamData::FD_t::logSFSC: [open FDs count] [file]";
    
    if ( sFSC.empty() )
    {
        clog << "\n  <no open and tracked FDs>"
             << endl;
        
        return;
    }
    
    for ( auto const& [path, cnt] : sFSC )
        clog << "\n  " << to_string(cnt) << "  " << path;
    
    clog << endl;
}

bool V4L2CamData::FD_t::warnSFSC() noexcept
{
    lock_guard<mutex> lck(sFSC_mtx);
    
    if ( sFSC.empty() )
        return false;
    
    clog << "V4L2CamData::FD_t::warnSFSC: there are files still open!!!"
         << endl;
    
    return true;
}

void V4L2CamData::FD_t::incSFSCfor(FD_t const& fd) noexcept
{
    lock_guard<mutex> lck(sFSC_mtx);
    
    const string path = retrieve_fd_path(fd);
    
    // clog << "V4L2CamData::FD_t::incSFSCfor: " << path << " adopted"
    //      << endl;
    
    ++sFSC[path];
    
    erase_if( sFSC
            , [] (auto const& kv) { return kv.second == 0; }
            );
}

void V4L2CamData::FD_t::decSFSCfor(FD_t const& fd) noexcept
{
    lock_guard<mutex> lck(sFSC_mtx);
    
    const string path = retrieve_fd_path(fd);
    
    // clog << "V4L2CamData::FD_t::decSFSCfor: " << path << " closing"
    //      << endl;
    
    --sFSC[path];
    
    erase_if( sFSC
            , [] (auto const& kv) { return kv.second == 0; }
            );
}


unordered_map< string
             , int32_t > V4L2CamData::FD_t::sFSC     = unordered_map< string
                                                                    , int32_t
                                                                    >{};
mutex                    V4L2CamData::FD_t::sFSC_mtx = mutex{};


V4L2CamData::V4L2CamData(string const& sNo) noexcept
 :  serialNo(sNo)
 ,  evntFD(make_shared<FD_t>(eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)))
{
    if ( !evntFD ) [[unlikely]]
        clog << "V4L2CamData::V4L2CamData: Failed to create eventfd!!!"
             << endl;
}


bool V4L2Cam::hubCanPowerCyclePerPort = false;


bool V4L2Cam::goIntoInitializedState() noexcept
{
    if ( errorAction == ErrorAction::ForgetDevice ) [[unlikely]]
    {
        
        
        clog << "V4L2Cam::goIntoInitializedState: last known device state: "
                "\"seems to be gone\". destruct this V4L2Cam object. make a new "
                "one, if you feel like it."
             << endl;
        
        return false;
    }
    
    if ( isJustInitialized() )
        return true;
    
    if (    state == State::UNINITIALIZED
         || state == State::DEVICE_KNOWN
       )
    {
        if ( !locateDeviceNodeAndInitialize() ) [[unlikely]]
        {
            clog << "V4L2Cam::goIntoInitializedState: could not reach State::"
                    "INITIALIZED!"
                 << endl;
            
            return false;
        }
        
        return true;
    }
    
    [[unlikely]]
    
    if (    state == State::DEQUEUEING
         || state == State::STREAMING
       )
        if ( !tryAndStopStreaming(false) ) [[unlikely]]
        {
            clog << "V4L2Cam::goIntoInitializedState: cannot stop streaming!"
                 << endl;
            
            return false;
        }
    
    if ( state == State::BUFFER_QUEUE_PREPPED )
    {
        if ( !releaseBufferQueue() ) [[unlikely]]
        {
            clog << "V4L2Cam::goIntoInitializedState: could not release device's "
                    "buffer queue!"
                 << endl;
            
            return false;
        }
    }
    
    if ( !isJustInitialized() ) [[unlikely]]
    {
        clog << "V4L2Cam::goIntoInitializedState: current state not known to "
                "this here function!"
             << endl;
        
        return false;
    }
    
    return true;
}

void V4L2Cam::goIntoUninitializedState() noexcept
{
    if ( state == State::UNINITIALIZED ) [[unlikely]]
    {
        clog << "V4L2Cam::goIntoUninitializedState: already in uninitalized state"
             << endl;
        
        return;
    }
    
    if ( !tryAndStopStreaming() ) [[unlikely]]
        clog << "V4L2Cam::goIntoUninitializedState: couldn't stop streaming. "
                "will still procede with uninialization."
             << endl;
    
    uninitialize(true);
}

bool V4L2Cam::locateDeviceNodeAndInitialize()
{
    if (    state != State::UNINITIALIZED
         && state != State::DEVICE_KNOWN
       ) [[unlikely]]
    {
        clog << "V4L2Cam::locateDeviceNodeAndInitialize: already initialized. "
                "no extrawurst!"
             << endl;
        
        return true;
    }
    
    string_view const&  vendorID = _produceVendorID ();
    string_view const& productID = _produceProductID();
    
    clog << "V4L2Cam::locateDeviceNodeAndInitialize: looking for "
         << "\n    vID: " << vendorID
         << "\n    pID: " << productID
         << "\n    sNo: " << serialNo
         << endl;
    
    bool   product_found{false};
    bool    serial_found{false};
    bool   capture_found{false};
    bool streaming_found{false};
    
    udev* uDev = udev_new();
    if ( !uDev ) [[unlikely]]
    {
        clog << "V4L2Cam::locateDeviceNodeAndInitialize: udev does not work"
             << endl;
        
        return false;
    }
    
    udev_enumerate* enumerate = udev_enumerate_new(uDev);
    if ( !enumerate ) [[unlikely]]
    {
        clog << "V4L2Cam::locateDeviceNodeAndInitialize: udev_enumerate_new "
                "failed"
             << endl;
        
        udev_unref(uDev);
        return false;
    }
    
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);
    
    udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    
    udev_device    * dev{};
    udev_list_entry* dev_list_entry;
    
    udev_list_entry_foreach( dev_list_entry, devices )
    {
        if ( dev )
            udev_device_unref(dev);
        
        const char *path = udev_list_entry_get_name(dev_list_entry);
        
        if ( !path ) [[unlikely]]
            continue;
        
        string_view svPath(path);
        // clog << "V4L2Cam::locateDeviceNodeAndInitialize: looking at"
        //         "\n    path: " << svPath.substr(0, 70);
        // for ( size_t p{70}; p < svPath.size(); p += 70 )
        //     clog << "\n          " << svPath.substr(p, 70);
        // clog << endl;
        
        dev = udev_device_new_from_syspath(uDev, path);
        
        if ( !dev )
            continue;
        
        udev_device*
        pdev = udev_device_get_parent_with_subsystem_devtype( dev
                                                            , "usb"
                                                            , "usb_device"
                                                            );
        
        if ( !pdev )
            continue;
        
        const char *vendor  = udev_device_get_sysattr_value(pdev, "idVendor");
        const char *product = udev_device_get_sysattr_value(pdev, "idProduct");
        
        if (                !vendor ||              !product
             || vendorID !=  vendor || productID !=  product
           )
        {
            // clog << "V4L2Cam::locateDeviceNodeAndInitialize: wrong vendor and/or "
            //         "product id"
            //      << endl;
            
            continue;
        }
        
        product_found = true;
        
        const char *sn = udev_device_get_sysattr_value(pdev, "serial");
        
        if ( !sn || serialNo != sn )
        {
            // clog << "V4L2Cam::locateDeviceNodeAndInitialize: wrong serial No"
            //      << endl;
        
            continue;
        }
        
        serial_found = true;
        
        char const* dev_path = udev_device_get_devnode(dev);
        
        if ( !dev_path )
            continue;
        
        // clog << "V4L2Cam::locateDeviceNodeAndInitialize: looking at"
        //         "\n    device path: " << dev_path
        //      << endl;
        
        FD_t fd{xopen(dev_path, O_RDWR | O_NONBLOCK)};
        
//         static int32_t thisRunsFD = -1;
//         
//         FD_t fd;
//         if ( thisRunsFD == -1 ) fd = thisRunsFD = xopen(dev_path, O_RDWR | O_NONBLOCK);
//         else                    fd = thisRunsFD;
        
        if ( !fd )
        {
            int const errNo = errno;
            
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Could not open "
                    "v4l2 device path! "
                    "\n  path: " << dev_path <<
                    "\n  err : " << strerror(errNo)
                 << endl;
            
            continue;
        }
        
        v4l2_capability cap{};
        
        if ( !xioctl(fd, VIDIOC_QUERYCAP, "VIDIOC_QUERYCAP", &cap) )
            continue;
        
        if ( !(   cap.capabilities
                & V4L2_CAP_VIDEO_CAPTURE )
           )
        {
            // clog << "V4L2Cam::locateDeviceNodeAndInitialize: not a capture device"
            //      << endl;
            
            continue;
        }
        
        capture_found = true;
        
        if ( !(   cap.capabilities
                & V4L2_CAP_STREAMING )
           )
        {
            // clog << "V4L2Cam::locateDeviceNodeAndInitialize: not a streaming "
            //         "capture device"
            //      << endl;
            
            continue;
        }
        
        streaming_found = true;
        
        v4l2Path = dev_path;
        
        const char *kernel_name = udev_device_get_sysname(pdev);
        if ( kernel_name ) USBKernelName = kernel_name;
        else               USBKernelName.clear();
        
        const char *busnum = udev_device_get_sysattr_value(pdev, "busnum");
        if ( busnum ) USBBusNumber = busnum;
        else          USBBusNumber.clear();
        
        const char *devnum = udev_device_get_sysattr_value(pdev, "devnum");
        if ( devnum ) USBDeviceAddress = devnum;
        else          USBDeviceAddress.clear();
        
        if ( !USBKernelName.empty() ) [[  likely]]
        {
            const char* syspath = udev_device_get_syspath(pdev);
            string_view sv_path = syspath;
            
            auto const pos = sv_path.find(USBKernelName);
            
            string_view const pPath = pos != string_view::npos
                                    ? string_view(syspath).substr(0, pos)
                                    : "/sys/bus/usb/devices";
            
            findDisableFSNode(pPath, USBKernelName);
        }
        
        // resetCrop(fd);
        
        v4l2FD = make_shared<FD_t>(move(fd));
        
        auto manufacturer = udev_device_get_sysattr_value(pdev, "manufacturer");
        auto product_name = udev_device_get_sysattr_value(pdev, "product");
        
        clog << "V4L2Cam::locateDeviceNodeAndInitialize: found ..."
             // << "\n    fd              : " << to_string((int32_t)*v4l2FD)
             << "\n    v4l2Path        : " << v4l2Path
             << "\n    USBKernelName   : " << USBKernelName
             << "\n    USBBusNumber    : " << USBBusNumber
             << "\n    USBDeviceAddress: " << USBDeviceAddress
             << "\n    manufacturer    : " << (   manufacturer != nullptr
                                                ? manufacturer
                                                : "<not in descriptor>"
                                              )
             << "\n    product name    : " << (   product_name != nullptr
                                                ? product_name
                                                : "<not in descriptor>"
                                              )
             << endl;
        
        auto const ea = errorAction;
        errorAction   = ErrorAction::None;
        
        if ( state == State::UNINITIALIZED )
        {
            determineSettingDomains(*v4l2FD);
            
            if ( errorAction != ErrorAction::None ) [[unlikely]]
            {
                clog << "V4L2Cam::locateDeviceNodeAndInitialize: could not "
                        "determine setting value domains!"
                     << endl;
                
                break;
            }
            
            if ( !determineMaxBufferSizeNeeded(*v4l2FD) ) [[unlikely]]
            {
                clog << "V4L2Cam::locateDeviceNodeAndInitialize: could not "
                        "determine max buffer size needed!"
                     << endl;
                
                break;
            }
            
            initializeSettings();
            
            state = State::DEVICE_KNOWN;
        }
        else /* state == State::DEVICE_KNOWN */
            reapplySettings();
        
        if ( _locateDeviceNodeAndInitialize(uDev, pdev) )
        {
            state = State::INITIALIZED;
        }
        else if (    errorAction != ErrorAction::CheckPermissions
                  && errorAction != ErrorAction::ForgetDevice
                ) [[unlikely]]
        {
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Model specific "
                    "derived class object couldn't initialize properly.\n    "
                    "At this stage probably bad device state. Marking "
                    "for need to reset device."
                 << endl;
            
            // uninitialize();
            state = State::UNINITIALIZED;
            
            errorAction = ErrorAction::ResetDevice;
            
            break;
        }
        
        update(errorAction, ea);
        
        break;
    }
    if ( dev )
        udev_device_unref(dev);
    
    udev_enumerate_unref(enumerate);
    udev_unref(uDev);
    
    if ( state != State::INITIALIZED )
    {
        if ( !product_found )
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Could not find "
                    "product w/ vendor ID: " <<  vendorID <<
                    " and product ID: "      << productID
                 << endl;
        else if ( !serial_found )
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Found correct "
                    "product(s), but not serial No: " << serialNo
                 << endl;
        else if ( !capture_found )
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Found correct "
                    "product and serial No, but no video capture device node"
                 << endl;
        else if ( !streaming_found )
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Found correct "
                    "product, serial No, and video capture device node, but "
                    "does not support streaming i/o"
                 << endl;
        else
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Found correct "
                    "product, serial No, and video capture device node "
                    "supporting streaming i/o, but could not fully initialize"
                 << endl;
    }
    
    if ( errorAction == ErrorAction::CheckLogic ) [[unlikely]]
    {
        clog << "V4L2Cam::locateDeviceNodeAndInitialize: Communication with "
                "device (driver) yielded errors indicating logical errors.\n    "
                "At this stage probably bad device state. Unitializing. Marking "
                "for need to reset device."
             << endl;
        
        // uninitialize();
        state = State::UNINITIALIZED;
        
        errorAction = ErrorAction::ResetDevice;
    }
    
    if ( errorAction == ErrorAction::ForgetDevice ) [[unlikely]]
    {
        clog << "V4L2Cam::locateDeviceNodeAndInitialize: Device seems to be gone!"
             << endl;
        
        uninitialize();
        state = State::UNINITIALIZED;
    }
    
    return state == State::INITIALIZED;
}

bool V4L2Cam::splitUsbSysname( string_view const  cur
                             , string           & parent
                             , int              & port
                             )
{
    // cur formats: "1-2" or "1-2.3.4"
    // parent:      "usb1" (for root hub case) or "1-2.3"
    // port:        last hop port number
    
    auto const dash = cur.find('-');
    
    if ( dash == string_view::npos )
        return false;
    
    auto const dot = cur.rfind('.');
    
    if ( dot != string_view::npos )
    {
        // parent is everything before last dot
        parent = cur.substr(0, dot);
        port   = stoi(string(cur.substr(dot + 1)));
        
        return true;
    }
    
    // No dot => form "bus-port"
    // parent hub is the root hub "usb<bus>"
    parent = string("usb") + string(cur.substr(0, dash));
    port   = stoi(string(cur.substr(dash + 1)));
    
    return true;
}

void V4L2Cam::findDisableFSNode( fs::path      sysBusUSBDevices
                               , string const& usbSysName
                               )
{
    USBPortDisablePathSelf.clear();
    USBPortDisablePathPeer.clear();
    
    error_code ec{};
    
    // sysBusUSBDevices should be "/sys/bus/usb/devices"
    // usbSysName e.g. "1-2.3.4"
    
    string cur = usbSysName;
    
    while ( true )
    {
        string parent;
        int port = -1;
        
        if ( !splitUsbSysname(cur, parent, port))
            break;
        
        fs::path hubInterfaceDir;
        
        if ( parent.rfind("usb", 0) == 0)
        {
            // Root hub interface is typically "<bus>-0:1.0", bus is after "usb"
            string const busStr = parent.substr(3);
            hubInterfaceDir = sysBusUSBDevices / ( busStr + "-0:1.0" );
        }
        else
        {
            hubInterfaceDir = sysBusUSBDevices / ( parent + ":1.0" );
        }
        
        fs::path const disablePath = hubInterfaceDir
                                   / ( parent + "-port" + to_string(port) )
                                   / "disable";
        
        if ( fs::exists(disablePath) )
        {
            USBPortDisablePathSelf = fs::canonical(disablePath, ec);
            
            if ( ec ) [[unlikely]] USBPortDisablePathSelf = disablePath;
            
            break;
        }
        
        cur = parent;
    }
    
    if ( USBPortDisablePathSelf.empty() )
        return;
    
    fs::path peerPath = USBPortDisablePathSelf.parent_path() / "peer";
    
    if ( fs::exists(peerPath) )
    {
        peerPath /= "disable";
        
        if ( fs::exists(peerPath) ) [[  likely]]
        {
            USBPortDisablePathPeer = fs::canonical(peerPath, ec);
            
            if ( ec ) [[unlikely]] USBPortDisablePathPeer = peerPath;
            
            return;
        }
        
        clog << "V4L2Cam::findDisableFSNode: found parent port with disable "
                "attribute and peer, but peer don't have disable!"
             << endl;
    }
}


bool V4L2Cam::usingMemoryType(MemoryType mt) noexcept
{
    if (    state != State::UNINITIALIZED
         && state != State::DEVICE_KNOWN
         && state != State::INITIALIZED
       ) [[unlikely]]
    {
        clog << "V4L2Cam::usingMemoryType: not in a correct state "
                "(UNINITIALIZED|DEVICE_KNOWN|INITIALIZED) to set or change the "
                "memory type to use (for buffers)!"
             << endl;
        
        return false;
    }
    
    if (    mt == MemoryType::UNKNOWN
         || mt == MemoryType::MMAP
       ) [[unlikely]]
    {
        memoryType = MemoryType::UNKNOWN;
        
        clog << "V4L2Cam::usingMemoryType: user-given memory type not supported. "
                "Reset to UNKNOWN, internally!"
             << endl;
        
        return false;
    }
    
    
    MemoryType oldMemTyp = memoryType;
    memoryType = mt;
    
    if ( !isSetMemoryTypeSupported() ) [[unlikely]]
    {
        memoryType = oldMemTyp;
        
        clog << "V4L2Cam::usingMemoryType: could not verify, that the device "
                "supports the given buffer type (v4l2_memory)!"
             << endl;
        
        return false;
    }
    return true;
}


bool V4L2Cam::requestBufferQueue(uint32_t count) noexcept
{
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    if ( state != State::INITIALIZED ) [[unlikely]]
    {
        clog << "V4L2Cam::requestBufferQueue: not in the correct state "
                "(INITIALIZED) to prep a v4l2 buffer queue!"
             << endl;
        
        return false;
    }
    
    
    uint32_t bufType{};
    uint32_t memType{};
    
    
    if (    !decideBufferType(bufType)
         || !bufType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::requestBufferQueue: could not decide v4l2 buffer type!"
             << endl;
        
        return false;
    }
    
    if (    !decideMemoryType(memType)
         || !memType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::requestBufferQueue: memory type to use unknown!"
             << endl;
        
        return false;
    }
    
    
    {
        v4l2_requestbuffers req{};
        req.count  = 0;
        req.type   = bufType;
        req.memory = memType;
        
        if ( !xioctl(*fd_ptr, VIDIOC_REQBUFS, "VIDIOC_REQBUFS", &req) ) [[unlikely]]
        {
            int const errNo = errno;
            
            clog << "V4L2Cam::requestBufferQueue: Could not pre-cleanup buffer queue! "
                 << strerror(errNo)
                 << endl;
            
            return false;
        }
    }
    
    
    v4l2_requestbuffers req{};
    req.count  = count;
    req.type   = bufType;
    req.memory = memType;
    
    if ( !xioctl(*fd_ptr, VIDIOC_REQBUFS, "VIDIOC_REQBUFS", &req) ) [[unlikely]]
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::requestBufferQueue: Could not have the device prepare "
                "a buffer queue for " << count << " buffers! "
             << strerror(errNo) << endl;
        
        return false;
    }
    
    bufferCount = req.count;
    buffersQueued.clear();
    state       = State::BUFFER_QUEUE_PREPPED;
        
    return count == req.count; // TODO do sth. more explicit?
}


bool V4L2Cam::produceUnqueuedMask(decltype(V4L2CamData::buffersQueued)& mask) const noexcept
{
    if (    state != State::BUFFER_QUEUE_PREPPED
         && state != State::STREAMING
       ) [[unlikely]]
    {
        clog << "V4L2Cam::produceUnqueuedMask: not in a correct state "
                "(BUFFER_QUEUE_PREPPED|STREAMING) - there's no queue for buffers "
                "to not be queued in!"
             << endl;
        
        return false;
    }
    
    
    using value_t = decltype(buffersQueued)::value_t;
    
    size_t m = buffersQueued.raw();
    
    m = ~m
      &  ((size_t{1} << bufferCount) - 1);
    
    mask.raw(static_cast<value_t>(m));
    
    return true;
}

bool V4L2Cam::prepBuffer(v4l2_buffer& buf) noexcept
{
    if (    state != State::BUFFER_QUEUE_PREPPED
         && state != State::STREAMING
         && state != State::DEQUEUEING
       ) [[unlikely]]
    {
        clog << "V4L2Cam::prepBuffer: not in a correct state "
                "(BUFFER_QUEUE_PREPPED|STREAMING|DEQUEUEING) to prep a buffer descriptor!"
             << endl;
        
        return false;
    }
    
    
    uint32_t bufType{};
    uint32_t memType{};
    
    
    if (    !decideBufferType(bufType)
         || !bufType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::prepBuffer: could not decide v4l2 buffer type!"
             << endl;
        
        return false;
    }
    
    if (    !decideMemoryType(memType)
         || !memType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::prepBuffer: memory type to use unknown!"
             << endl;
        
        return false;
    }
    
    
    buf = {};
    
    buf.type   = bufType;
    buf.memory = memType;
    
    if ( apiToUse == APIToUse::MULTI )
    {
        bufferPlanes = {};
        buf.m.planes = bufferPlanes.data();
        buf.length   = bufferPlanes.size();
    }
    
    return true;
}


bool V4L2Cam::queueBuffer(v4l2_buffer& buf) noexcept
{
    shared_ptr<FD_t> fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    
    if (    state != State::BUFFER_QUEUE_PREPPED
         && state != State::STREAMING
       ) [[unlikely]]
    {
        clog << "V4L2Cam::queueBuffer: not in a correct state "
                "(BUFFER_QUEUE_PREPPED|STREAMING) to queue buffers!"
             << endl;
        
        return false;
    }
    
    
    if ( buf.index >= bufferCount ) [[unlikely]]
    {
        clog << "V4L2Cam::queueBuffer: given buffer has out-of-bounds index!!!"
             << endl;
        
        return false;
    }
    
    
    if ( buffersQueued.test(buf.index) ) [[unlikely]]
        clog << "V4L2Cam::queueBuffer: given buffer queued, but there's already "
                "one queued at that index! (I'll try to queue it anyway. "
                "Wanna see what happens.)"
             << endl;
    
    
    if ( !xioctl(*fd_ptr, VIDIOC_QBUF, "VIDIOC_QBUF", &buf) ) [[unlikely]]
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::queueBuffer: could not queue buffer! "
             << strerror(errNo) << endl;
        
        return false;
    }
    
    buffersQueued.set(buf.index);
    
    return true;
}


void V4L2Cam::logSetup(ostream& out) noexcept
{
    if ( state != State::BUFFER_QUEUE_PREPPED ) [[unlikely]]
    {
        out << "    not in the correct state (BUFFER_QUEUE_PREPPED) for setup "
               " report!"
            << endl;
        
        return;
    }
    
    out << "    serial No.    : " << serialNo;
    
    bool apiKnown;
    
    out << "\n    buffer API    : ";
    switch ( apiToUse )
    {
        case APIToUse::UNKNOWN: out << "UNKNOWN"; apiKnown = false; break;
        case APIToUse::MULTI  : out << "MULTI"  ; apiKnown = true ; break;
        case APIToUse::SINGLE : out << "SINGLE" ; apiKnown = true ; break;
        default               : out << "???"    ; apiKnown = false;
    }
    
    auto fmt_opt = apiKnown
                 ? giveV4L2Format()
                 : decltype(giveV4L2Format()){};
    
    if ( fmt_opt ) [[  likely]]
    {
        uint32_t width{}, height{}, pxFmt{}, field{};
        
        if ( apiToUse == APIToUse::MULTI )
        {
            auto& fmt   = (*fmt_opt).fmt.pix_mp;
                  width = fmt.width;       height = fmt.height;
                  pxFmt = fmt.pixelformat; field  = fmt.field;
        }
        else // APIToUse::SINGLE
        {
            auto& fmt   = (*fmt_opt).fmt.pix;
                  width = fmt.width;       height = fmt.height;
                  pxFmt = fmt.pixelformat; field  = fmt.field;
        }
        
        out << "\n    width         : " << to_string(width)
            << "\n    heigth        : " << to_string(height)
            << "\n    pixel format  : "
            <<      '\'' 
            <<      static_cast<char>((pxFmt >>  0) & 0xFF)
            <<      static_cast<char>((pxFmt >>  8) & 0xFF)
            <<      static_cast<char>((pxFmt >> 16) & 0xFF)
            <<      static_cast<char>((pxFmt >> 24) & 0xFF)
            <<      '\''
            << "\n    field order   : ";
        
        switch( field )
        {
            case V4L2_FIELD_ANY          : out << "V4L2_FIELD_ANY <illegal!>"; break;
            case V4L2_FIELD_NONE         : out << "V4L2_FIELD_NONE"          ; break;
            case V4L2_FIELD_TOP          : out << "V4L2_FIELD_TOP"           ; break;
            case V4L2_FIELD_BOTTOM       : out << "V4L2_FIELD_BOTTOM"        ; break;
            case V4L2_FIELD_INTERLACED   : out << "V4L2_FIELD_INTERLACED"    ; break;
            case V4L2_FIELD_SEQ_TB       : out << "V4L2_FIELD_SEQ_TB"        ; break;
            case V4L2_FIELD_SEQ_BT       : out << "V4L2_FIELD_SEQ_BT"        ; break;
            case V4L2_FIELD_ALTERNATE    : out << "V4L2_FIELD_ALTERNATE"     ; break;
            case V4L2_FIELD_INTERLACED_TB: out << "V4L2_FIELD_INTERLACED_TB" ; break;
            case V4L2_FIELD_INTERLACED_BT: out << "V4L2_FIELD_INTERLACED_BT" ; break;
            
            default: clog << "<not recognized!>";
        }
    }
    else if ( apiKnown ) // , but no fmt
        out << "\n    could not get v4l2_format from device!";
    
    out << "\n    memory type   : ";
    switch ( memoryType )
    {
        case MemoryType::UNKNOWN: out << "UNKNOWN"; break;
        case MemoryType::MMAP   : out << "MMAP"   ; break;
        case MemoryType::USERPTR: out << "USERPTR"; break;
        case MemoryType::DMABUF : out << "DMABUF" ; break;
        default                 : out << "???"    ;
    }
    out << "\n    buffers queued: " << to_string(buffersQueued.count());
     
    out << "\n    framerate     : ";
    auto fps_opt = _produceFramerate();
    if ( !fps_opt ) [[unlikely]]
        out << "can't tell";
    else
        out << to_string(*fps_opt);
    
    out << "\n    exposure      : ";
    int32_t exp{};
    if ( !fetchExposure() ) [[unlikely]]
        out << "couldn't get value from device";
    else if ( !giveExposure(exp) ) [[unlikely]]
        out << "have no valid value to provide";
    else
        out << to_string(exp);
    
    
    _logSetup(out);
}


bool V4L2Cam::startStreaming() noexcept
{
    shared_ptr<FD_t> fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    
    if ( state != State::BUFFER_QUEUE_PREPPED ) [[unlikely]]
    {
        clog << "V4L2Cam::startStreaming: not in the correct state "
                "(BUFFER_QUEUE_PREPPED) to start streaming!"
             << endl;
        
        return false;
    }
    
    if ( buffersQueued.count() < bufferCount ) [[unlikely]]
    {
        clog << "V4L2Cam::startStreaming: user hasn't filled the buffer queue to "
                "the brim. I cannot work like that!"
             << endl;
        
        return false;
    }
    
    
    uint32_t bufType{};
    
    if (    !decideBufferType(bufType)
         || !bufType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::startStreaming: could not decide v4l2 buffer type!"
             << endl;
        
        return false;
    }

    
    if ( !xioctl(*fd_ptr, VIDIOC_STREAMON, "VIDIOC_STREAMON", &bufType) )
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::startCapturing: couldn't start streaming! "
             << strerror(errNo) << endl;
        
        return false;
    }
    
    state = State::STREAMING;
    
    return true;
}

bool V4L2Cam::fillBuffer(v4l2_buffer& buf) noexcept
{
    shared_ptr<FD_t> fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
    {
        clog << "V4L2Cam::fillBuffer: no v4l2 dev fd!!!"
             << endl;
        
        return false;
    }
    
    
    if ( state != State::STREAMING ) [[unlikely]]
    {
        clog << "V4L2Cam::fillBuffer: not in the correct state "
                "(STREAMING) to try to fetch a frame!"
             << endl;
        
        return false;
    }
    
    
    state = State::DEQUEUEING;
    
    int errNo{};
    
    pollfd fds[2]{};
    fds[0].fd     = *fd_ptr;
    fds[0].events = ( POLLIN | POLLRDNORM /*| POLLERR | POLLHUP | POLLNVAL*/ );
    
    if ( evntFD ) [[  likely]]
    {
        fds[1].fd     = *evntFD;
        fds[1].events = POLLIN | POLLRDNORM;
    }
    
    
    while ( true )
    {
        int ret = poll(fds, evntFD ? 2 : 1, FRAME_POLL_TIMEOUT);
        errNo   = errno;
        
        if ( ret == -1 ) [[unlikely]]
        {
            if ( errNo == EINTR ) [[  likely]]
            {
                clog << "V4L2Cam::fillBuffer: poll got interupted"
                     << endl;
                
                continue;
            }
            
            clog << "V4L2Cam::fillBuffer: poll errored out! "
                 << strerror(errNo) << endl;
            
            state = State::STREAMING;
            
            return false;
        }
        else if ( ret == 0 ) [[unlikely]]
        {
            update(errorAction, ErrorAction::ResetDevice);
            state = State::STREAMING;
            clog << "V4L2Cam::fillBuffer: poll timed out. marking for need to "
                    "reset cam device!"
                 << endl;
            
            return false;
        }
        
        if (    evntFD
             && fds[1].revents & ( POLLIN | POLLRDNORM )
           ) [[unlikely]]
        {
            uint64_t dummy;
            dummy = read(*evntFD, &dummy, sizeof(dummy));
            
            clog << "V4L2Cam::fillBuffer: so. wrote to my eventfd to wake me!?"
                 << endl;
        }
        
        if ( fds[0].revents & ( POLLERR | POLLHUP | POLLNVAL ) ) [[unlikely]]
        {
            clog << "V4L2Cam::fillBuffer: polled ";
            
            if ( fds[0].revents & POLLERR ) [[unlikely]]
            {
                update(errorAction, ErrorAction::ResetDevice);
                clog << "POLLERR";
            }
            else if ( fds[0].revents & POLLHUP ) [[unlikely]]
            {
                update(errorAction, ErrorAction::ForgetDevice);
                clog << "POLLHUP";
            }
            else if ( fds[0].revents & POLLNVAL ) [[unlikely]]
            {
                update(errorAction, ErrorAction::CheckLogic);
                clog << "POLLNVAL";
            }
            
            clog << " on v4l2 interface!"
                 << endl;
            
            state = State::STREAMING;
            
            return false;
        }
        else if ( !( fds[0].revents & ( POLLIN | POLLRDNORM ) ) ) [[unlikely]]
        {
            state = State::STREAMING;
            
            return false;
        }
        
        // we've got a frame, so the last reset measure - if any taken at all -
        // seems to have been fruitful -> reset for handling of the next error
        lastResetMeasure = ResetMeasure::NONE;
        
        break;
    }
        
    
    buf = {};
    
    if ( !prepBuffer(buf) ) [[unlikely]]
    {
        clog << "V4L2Cam::fillBuffer: sth. went wrong prepping the "
                "v4l2_buffer structure for the fetch. "
                ;
        clog << "Will try anyways. Might work." // fucks up cam, but want to learn to rehabilitate
             << endl;
        
        update(errorAction, ErrorAction::CheckLogic);
        
        return false;
    }
    
    bool succ = xioctl( *fd_ptr
                      , VIDIOC_DQBUF
                      , "VIDIOC_DQBUF"
                      , &buf
                      , XIOCTL_FLAGS::QUASI_BLOCKING
                      );
    errNo     = errno;
    
    state = State::STREAMING;
    
    if ( !succ ) [[unlikely]]
    {
        clog << "V4L2Cam::fillBuffer: dequeueing a buffer with a frame didn't work! "
             << strerror(errNo) << endl;
        
        return false;
    }
    
    
    if ( buf.index >= bufferCount ) [[unlikely]]
    {
        clog << "V4L2Cam::fillBuffer: retrieved buffer has out-of-bounds index!!!"
             << endl;
        
        return false;
    }
    
    
    buffersQueued.reset(buf.index);
    
    return true;
}

bool V4L2Cam::wake() noexcept
{
    if ( !evntFD ) [[unlikely]]
    {
        clog << "V4L2Cam::wake: there's no valid eventfd!!!"
             << endl;
        
        return false;
    }
    
    
    if (    state != State::STREAMING
         && state != State::DEQUEUEING
       ) [[unlikely]]
    {
        clog << "V4L2Cam::wake: not in working state; no thread to wake"
             << endl;
        
        return false;
    }
    
    
    uint64_t one = 1;
    auto bytesWritten = write(*evntFD, &one, sizeof(one));
    
    if ( bytesWritten != sizeof(one) ) [[unlikely]]
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::wake: write to eventfd failed with errno "
             << errNo << " (" << strerror(errNo) << ")"
             << endl;
        
        return false;
    }
    
    return true;
}

bool V4L2Cam::stopStreaming() noexcept
{
    shared_ptr<FD_t> fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    
    if ( state != State::STREAMING ) [[unlikely]]
    {
        clog << "V4L2Cam::stopStreaming: not in the correct state "
                "(STREAMING) to stop streaming!"
             << endl;
        
        return false;
    }
    
    
    state = State::DEQUEUEING;
    
    v4l2_buffer buf{};
    
    if ( !prepBuffer(buf) ) [[unlikely]]
    {
        clog << "V4L2Cam::stopStreaming: sth. went wrong prepping the "
                "v4l2_buffer structure for dequeueing whatever's left to dequeue!"
             << endl;
        
        return false;
    }
    
    
    if ( !xioctl(*fd_ptr, VIDIOC_STREAMOFF, "VIDIOC_STREAMOFF", &buf.type) )
    [[unlikely]]
    {
        int const errNo = errno;
        
        state = State::STREAMING;
        
        clog << "V4L2Cam::stopStreaming: operation failed! "
             << strerror(errNo) << endl;
        
        return false;
    }
    
    // clog << "V4L2Cam::stopStreaming: successfully set VIDIOC_STREAMOFF"
    //      << endl;
    
    while (    buffersQueued.count() > 0
            && xioctl( *fd_ptr
                     , VIDIOC_DQBUF
                     , "VIDIOC_DQBUF"
                     , &buf
                     , XIOCTL_FLAGS::EXPECT_EINVAL
                     )
          )
    {
        if ( buf.index >= bufferCount ) [[unlikely]]
            clog << "V4L2Cam::stopStreaming: retrieved buffer has out-of-bounds "
                    "index!!!"
                 << endl;
        else
            buffersQueued.reset(buf.index);
        
        prepBuffer(buf);
    }
    
    // driver might have cleared queue with STREAMOFF, instead of playing the
    // dequeue game with us.
    buffersQueued.clear();
    
    state = State::BUFFER_QUEUE_PREPPED;
    
    if ( errorAction == ErrorAction::StopStreaming )
         errorAction =  ErrorAction::None;
    
    return true;
}


bool V4L2Cam::releaseBufferQueue() noexcept
{
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    if ( state != State::BUFFER_QUEUE_PREPPED ) [[unlikely]]
    {
        clog << "V4L2Cam::releaseBufferQueue: not in the correct state "
                "(BUFFER_QUEUE_PREPPED) to un-prep a v4l2 buffer queue!"
        << endl;
        
        return false;
    }
    
    
    uint32_t bufType{};
    uint32_t memType{};
    
    
    if (    !decideBufferType(bufType)
         || !bufType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::releaseBufferQueue: could not decide v4l2 buffer type!"
             << endl;
        
        return false;
    }
    
    if (    !decideMemoryType(memType)
         || !memType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::releaseBufferQueue: memory type to use unknown!"
             << endl;
        
        return false;
    }
    
    
    v4l2_requestbuffers req{};
    req.count  = 0;
    req.type   = bufType;
    req.memory = memType;
    
    if ( !xioctl(*fd_ptr, VIDIOC_REQBUFS, "VIDIOC_REQBUFS", &req) ) [[unlikely]]
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::releaseBufferQueue: Could not un-prep buffer queue! "
             << strerror(errNo) << endl;
        
        return false;
    }
    
    bufferCount = 0;
    buffersQueued.clear();
    state = State::INITIALIZED;
    
    return true;
}


bool V4L2Cam::reinitialize() noexcept
{
    if ( state == State::UNINITIALIZED ) [[unlikely]]
    {
        clog << "V4L2Cam::reinitialize: not in a correct state (anything but "
                "UNINITIALIZED) to reset device!"
             << endl;
        
        return false;
    }
    
    if ( errorAction != ErrorAction::Reinitialize ) [[unlikely]]
    {
        clog << "V4L2Cam::reinitialize: not marked with ErrorAction::Reinitialize!"
             << endl;
        
        return false;
    }
    
    
    tryAndStopStreaming();
    uninitialize(true);
    
    this_thread::sleep_for(chrono::milliseconds(100));
    
    clog << "V4L2Cam::reinitialize: trying to re-init ..."
         << endl;
    
    if ( !locateDeviceNodeAndInitialize() )
    {
        clog << "V4L2Cam::reinitialize: ... no success!"
             << endl;
        
        return false;
    }
    else
    {
        clog << "V4L2Cam::reinitialize: ... successfully"
             << endl;
        
        errorAction = ErrorAction::None;
        this_thread::sleep_for(chrono::milliseconds(100)); // grace period
        
        return true;
    }
}

bool V4L2Cam::resetDevice() noexcept
{
    if ( state == State::UNINITIALIZED ) [[unlikely]]
        clog << "V4L2Cam::resetDevice: resetting device from UNINITIALIZED state "
                "will not restore settings, since none known!"
             << endl;
    
    if ( errorAction != ErrorAction::ResetDevice ) [[unlikely]]
    {
        clog << "V4L2Cam::resetDevice: not marked with ErrorAction::ResetDevice!"
             << endl;
        
        return false;
    }
    
    if (    lastResetMeasure == ResetMeasure::USB_PORT_POWER_CYCLE
         || lastResetMeasure == ResetMeasure::USB_PORT_POWER_CYCLE_REQUESTED
       ) [[unlikely]]
    {
        clog << "V4L2Cam::resetDevice: measures already exhausted!"
             << endl;
        
        errorAction = ErrorAction::ForgetDevice;
        
        return false;
    }
    
    if (    USBKernelName   .empty()
         || USBBusNumber    .empty()
         || USBDeviceAddress.empty()
       ) [[unlikely]]
    {
        clog << "V4L2Cam::resetDevice: USB kernel name, bus number, or device "
                "address is not set."
             << endl;
        
        return false;
    }
    
    clog << "V4L2Cam::resetDevice: see what we can do ..."
         << endl;
    
    storedUSBID.kernelName             = USBKernelName;
    storedUSBID.busNumber              = USBBusNumber;
    storedUSBID.deviceAddress          = USBDeviceAddress;
    storedUSBID.portDisablePathSelf = USBPortDisablePathSelf;
    storedUSBID.portDisablePathPeer = USBPortDisablePathPeer;
    
    tryAndStopStreaming();
    
RESET_DEVICE_ESCALATE_MEASURES:
    
    uninitialize(true);
    
    bool didReset{false}, didReinit{false};
    chrono::milliseconds reinitInterval{};
    chrono::milliseconds reinitTimeout {};
    
    // lastResetMeasure gets reset on successfull frame fetch
    
    // can't seem to get it to work for now
    if (     lastResetMeasure == ResetMeasure::NONE )
    {
        clog << "V4L2Cam::resetDevice: trying to rebind USB interfaces ..."
             << endl;
        
        lastResetMeasure = ResetMeasure::USB_IFACE_REBIND;
        reinitInterval   = chrono::milliseconds{ 500};
        reinitTimeout    = chrono::milliseconds{2000};
        // didReset         = rebindUSBDevice();
    }
    if (    !didReset
         &&  lastResetMeasure == ResetMeasure::USB_IFACE_REBIND
       )
    {
        clog << "V4L2Cam::resetDevice: trying to reset USB device (host state) ..."
             << endl;
        
        lastResetMeasure = ResetMeasure::USB_DEVFS_RESET;
        reinitInterval   = chrono::milliseconds{1250};
        reinitTimeout    = chrono::milliseconds{5000};
        didReset         = resetUSBDevice();
    }
//     if (    !didReset
//          &&  lastResetMeasure == ResetMeasure::USB_DEVFS_RESET
//        )
//     {
//         clog << "V4L2Cam::resetDevice: trying to reset USB device (itself) ..."
//              << endl;
//         
//         lastResetMeasure = ResetMeasure::USB_PORT_RESET;
//         reinitInterval   = chrono::milliseconds{ 400};
//         reinitTimeout    = chrono::milliseconds{5000};
//         didReset         = resetAtUSBHubPort();
//     }
    if (    !didReset
         // &&  lastResetMeasure == ResetMeasure::USB_PORT_RESET
         &&  lastResetMeasure == ResetMeasure::USB_DEVFS_RESET
       )
    {
        if ( hubCanPowerCyclePerPort )
        {
            clog << "V4L2Cam::resetDevice: trying to power cycle USB port ..."
                 << endl;
            
            lastResetMeasure = ResetMeasure::USB_PORT_POWER_CYCLE;
            reinitInterval   = chrono::milliseconds{2000};
            reinitTimeout    = chrono::milliseconds{8000};
            // TODO *untested*
            didReset         = powerCycleUSBDevice();
            // TODO *untested*
            // didReset         = resetAtUSBHubPort();
        }
        else // can only request higher-ups to do the power cycling
        {
            clog << "V4L2Cam::resetDevice: requesting power cycling of USB port "
                    "by higher-ups"
                 << endl;
            
            errorAction      = ErrorAction::USBPowerCycle;
            lastResetMeasure = ResetMeasure::USB_PORT_POWER_CYCLE_REQUESTED;
        }
    }
    
    
    if ( lastResetMeasure != ResetMeasure::USB_PORT_POWER_CYCLE_REQUESTED )
    {
        if ( !didReset
           ) [[unlikely]]
        {
            clog << "V4L2Cam::resetDevice: ... but failed!"
                 << endl;
            
            errorAction = ErrorAction::ForgetDevice;
            
            return false;
        }
        
        clog << "V4L2Cam::resetDevice: ... successfully"
             << endl;
        clog << "V4L2Cam::resetDevice: trying to re-init ..."
             << endl;
        
        auto const deadline = chrono::steady_clock::now() + reinitTimeout;
        
        do
        {
            if ( locateDeviceNodeAndInitialize() )
                didReinit = true;
            else
                this_thread::sleep_for(reinitInterval);
        }
        while (    !didReinit
                &&  chrono::steady_clock::now() < deadline
              );
        
        if ( !didReinit )
        {
            clog << "V4L2Cam::resetDevice: ... but failed!"
                 << endl;
            
            if ( lastResetMeasure == ResetMeasure::USB_PORT_POWER_CYCLE )
            {
                clog << "V4L2Cam::resetDevice: reset measures exhausted. marking "
                        "myself unfixable!"
                     << endl;
                
                errorAction = ErrorAction::ForgetDevice;
            }
            else
            {
                clog << "V4L2Cam::resetDevice: immediately going back to try the "
                        "next measure in line"
                     << endl;
                
                goto RESET_DEVICE_ESCALATE_MEASURES;
            }
        }
        else
        {
            clog << "V4L2Cam::resetDevice: ... successfully"
                 << endl;
            
            if ( lastResetMeasure != ResetMeasure::USB_PORT_POWER_CYCLE_REQUESTED )
            {
                errorAction = ErrorAction::None;
                this_thread::sleep_for(chrono::milliseconds(100)); // grace period
            }
        }
    }
    
    
    return didReset && didReinit;
}

void V4L2Cam::powerCyclingConducted() noexcept
{
    if (    errorAction      != ErrorAction ::USBPowerCycle
         || lastResetMeasure != ResetMeasure::USB_PORT_POWER_CYCLE_REQUESTED
       ) [[unlikely]]
        return;
    
    
    errorAction      = ErrorAction::None;
    lastResetMeasure = ResetMeasure::USB_PORT_POWER_CYCLE;
}


V4L2Cam::V4L2Cam(string const& sNo) noexcept
 :  V4L2CamData(sNo)
{
    superObjectCannotExist = false;
}

V4L2Cam::~V4L2Cam() noexcept
{
    superObjectCannotExist = true;
    
    tryAndStopStreaming(true); // `std::terminate`s, if it doesn't work out!
    
    if ( state == State::BUFFER_QUEUE_PREPPED ) [[unlikely]]
        releaseBufferQueue();
    
    uninitialize();
    
    evntFD->close_fd();
}


bool V4L2Cam::gatherSerialNumbers( string_view      vendorID
                                 , string_view     productID
                                 , vector<string>& serials
                                 ) noexcept
{
    serials.clear();

    udev *uDev = udev_new();
    
    if ( !uDev ) [[unlikely]]
    {
        clog << "V4L2Cam::gatherSerialNumbers: udev does not work"
             << endl;
        
        return false;
    }
    
    udev_enumerate *enumerate = udev_enumerate_new(uDev);
    
    if ( !enumerate ) [[unlikely]]
    {
        clog << "V4L2Cam::gatherSerialNumbers: udev_enumerate_new failed"
             << endl;
        
        udev_unref(uDev);
        return false;
    }
    
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);
    
    udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
    
    udev_device    * dev{};
    udev_list_entry* dev_list_entry;
    
    udev_list_entry_foreach( dev_list_entry, devices )
    {
        if ( dev )
            udev_device_unref(dev);
        
        const char *path = udev_list_entry_get_name(dev_list_entry);
        
        if ( !path )
            continue;
        
        dev = udev_device_new_from_syspath(uDev, path);
        
        if ( !dev )
            continue;
        
        udev_device*
        pdev = udev_device_get_parent_with_subsystem_devtype( dev
                                                            , "usb"
                                                            , "usb_device"
                                                            );
        
        if ( !pdev )
            continue;
        
        const char *vendor  = udev_device_get_sysattr_value(pdev, "idVendor");
        const char *product = udev_device_get_sysattr_value(pdev, "idProduct");
        
        if (                !vendor ||              !product
             || vendorID !=  vendor || productID !=  product
           )
            continue;
        
        const char *sn = udev_device_get_sysattr_value(pdev, "serial");
        
        if ( sn && *sn )
            serials.emplace_back(sn);
    }
    clog << flush;
    
    if ( dev )
        udev_device_unref(dev);
    
    
    udev_enumerate_unref(enumerate);
    udev_unref(uDev);
    
    std::ranges::sort(serials);
    auto new_end = std::ranges::unique(serials).begin();
    serials.erase(new_end, serials.end());
    
    return !serials.empty();
}


bool V4L2Cam::xioctl( FD_t const&       fd
                    , uint64_t          request
                    , string_view const requestStr
                    , void*             arg
                    , XIOCTL_FLAGS      callFlags
                    )
{
    using namespace std::chrono;
    using namespace std::chrono_literals;
    
    bool quasi_blocking =    ( callFlags & XIOCTL_FLAGS::QUASI_BLOCKING )
                          != XIOCTL_FLAGS::NONE;
    
    constexpr uint8_t retry_count = 8;
    uint8_t           busy_count{};
    auto const        deadline    = steady_clock::now() + 1s;
    
    int r;
    int errNo;
    
    do {
        errno = 0;
        
        r     = ioctl( static_cast<int>(fd)
                     , static_cast<unsigned long>(request)
                     , arg
                     );
        errNo = errno;
        
        if ( r == -1 )
        {
            if ( steady_clock::now() >= deadline ) [[unlikely]]
                break;
            
            if ( errNo == EBUSY )
            {
                if      ( busy_count == 0 )
                    this_thread::yield();
                else if (    quasi_blocking
                          || busy_count  < retry_count
                        )
                    this_thread::sleep_for
                    (milliseconds(1u << std::min<uint8_t>(busy_count, 7)));
                else
                    break;
                
                busy_count++;
            }
            else if ( errNo == EAGAIN && quasi_blocking )
                this_thread::sleep_for(1ms);
        }
    } while ( r == -1 && (      errNo == EINTR
                           ||   errNo == EBUSY
                           || ( errNo == EAGAIN && quasi_blocking )
                         )
            );
    
    if ( r == -1 ) [[unlikely]]
    {
        // Handle specific errors and set action flags
        switch ( errNo )
        {
            // Errors indicating that forgetting the device is appropriate
            case ENODEV:    [[fallthrough]]; // No such device
            case ENXIO:     [[fallthrough]]; // No such device or address
            case EBADF:                      // Bad file descriptor
                clog << "V4L2Cam::xioctl: Error: Device is no longer available ("
                     << strerror(errNo)
                     << "). Forgetting all about the device is indicated."
                     << endl;
                
                update(errorAction, ErrorAction::ForgetDevice);
                break;
            
            // Errors indicating that resetting the camera abstraction class is appropriate
            case EBUSY:                      // Device or resource busy after retries
                if ( state == State::STREAMING )
                // I interpret this as "what you're trying to do won't work
                // while streaming". Am I right/wrong?
                {
                    clog << "V4L2Cam::xioctl: ioctl failed with EBUSY while in "
                            "streaming mode"
                         << endl;
                    
                    update(errorAction, ErrorAction::StopStreaming);
                    break;
                }
                else
                    [[fallthrough]];
            case ETIMEDOUT:                  // Connection timed out
                clog << "V4L2Cam::xioctl: Error: Device is busy or timed out ("
                     << strerror(errNo)
                     << "). Resetting camera class is indicated."
                     << endl;
                
                update(errorAction, ErrorAction::Reinitialize);
                break;
            
            // Errors indicating that power-cycling the USB camera is appropriate
            case EIO:       [[fallthrough]]; // Input/output error
            case EFAULT:                     // Bad address
                clog << "V4L2Cam::xioctl: Error: Hardware failure ("
                     << strerror(errNo)
                     << "). Power-cycling the USB camera is indicated."
                     << endl;
                
                update(errorAction, ErrorAction::ResetDevice);
                break;
            
            // Errors indicating permission issues
            case EPERM:     [[fallthrough]]; // Operation not permitted
            case EACCES:                     // Permission denied
                clog << "V4L2Cam::xioctl: Error: Permission denied ("
                     << strerror(errNo)
                     << "). Check permissions is indicated."
                     << endl;
                
                update(errorAction, ErrorAction::CheckPermissions);
                break;
            
            // Errors indicating programming errors or misconfigurations
            case EINVAL:    if (    ( callFlags & XIOCTL_FLAGS::EXPECT_EINVAL )
                                 != XIOCTL_FLAGS::NONE
                               )
                                return false;
                            else
                                goto XIOCTL_WRONG_USAGE; // Invalid argument
            case EDOM  :    if (    ( callFlags & XIOCTL_FLAGS::EXPECT_EDOM )
                                 != XIOCTL_FLAGS::NONE
                               )
                                return false;
                            else
                                goto XIOCTL_WRONG_USAGE; // Argument out of domain
            case ENOTTY:    if (    ( callFlags & XIOCTL_FLAGS::EXPECT_ENOTTY )
                                 != XIOCTL_FLAGS::NONE
                               )
                                return false;
                            else
                                goto XIOCTL_WRONG_USAGE; // Inappropriate ioctl for device
            case EOVERFLOW:                  // Value too large for defined data type
XIOCTL_WRONG_USAGE:
                clog << "V4L2Cam::xioctl: Error: Invalid request or argument ("
                     << strerror(errNo)
                     << "). Check configuration is indicated."
                     << endl;
                
                update(errorAction, ErrorAction::CheckLogic);
                break;
            
            case ENOMEM:                     // Out of memory
                clog << "V4L2Cam::xioctl: Error: Out of memory ("
                     << strerror(errNo) << ")."
                     << endl;
                
                update(errorAction, ErrorAction::FreeMemory);
                break;
            
            // not of import
            case EINTR:     [[fallthrough]]; // Interrupted system call
            case EAGAIN:                     // Resource temporarily unavailable
                break;
            
            default:
                clog << "V4L2Cam::xioctl: Error: ioctl failed with errNo "
                     << to_string(errNo) << " (" << strerror(errNo) << ")."
                     << endl;
                
                break;
        }
    }
    
    if ( r == -1 )
        clog << "V4L2Cam::xioctl: error occured. given request: "
             << requestStr
             << endl;
    
    errno = errNo;
    return r != -1;
}

int32_t V4L2Cam::xopen( char    const* pathname
                      , int32_t const  flags
                      )
{
    constexpr int32_t retry_count = 10;
    int32_t delay_ms = 1;
    
    int32_t   fd{-1};
    int errNo;
    
    for ( int32_t attempt = 0; attempt < retry_count; ++attempt )
    {
        errno = 0;
        
        fd    = open(pathname, flags);
        errNo = errno;
        
        if ( fd >= 0 )
            break;
        
        if ( errNo == EINTR || errNo == EAGAIN || errNo == EBUSY )
        {
            if ( attempt < retry_count - 1 )
            {
                this_thread::sleep_for(chrono::milliseconds(delay_ms *= 2));
                
                continue;
            }
            else
            {
                clog << "V4L2Cam::xopen: Failed to open character device after "
                     << (attempt + 1) << " attempts: " << strerror(errNo) << ". "
                        "path:" << pathname
                     << endl;
                
                break;
            }
        }
        else if ( errNo == ENOENT || errNo == ENODEV )
        {
            clog << "V4L2Cam::xopen: character device not found: "
                 << strerror(errNo) << ". path:" << pathname
                 << endl;
            
            break;
        }
        else
        {
            clog << "V4L2Cam::xopen: Failed to open character device: "
                 << strerror(errNo) << ". path:" << pathname
                 << endl;
            
            break;
        }
    }
    
    return fd;
}

bool V4L2Cam::xwrite( int         fd
                    , char const* str
                    , size_t      len
                    )
{
    using namespace chrono;
    
    size_t written_bytes{};
    
    while ( written_bytes < len )
    {
        ssize_t result = write( fd
                              , str + written_bytes
                              , len - written_bytes
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
                clog << "V4L2Cam.cpp, xwrite(): Write I/O error: "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
            else
            {
                clog << "V4L2Cam.cpp, xwrite(): Write error: "
                     << strerror(errNo)
                     << endl;
                
                errno = errNo;
                return false;
            }
        }
        
        written_bytes += result;
    }
    
    return true;
}


// private:

void V4L2Cam::initializeSettings()
{
    fetchResolutionAndPixelFormat();
    fetchBrightness();
    fetchContrast();
    fetchSaturation();
    fetchSharpness();
    fetchGamma();
    fetchWhiteBalance();
    fetchGain();
    fetchPowerLineFrequency();
    fetchExposure();
}

void V4L2Cam::reapplySettings()
{
    auto const ea = errorAction;
    
    clog << "V4L2Cam::reapplySettings: !!! unintelligently trying to re-apply "
            "remembered settings\n"
            "                              some might yield \"Permission "
            "denied\" errors. Can safely be ignored."
         << endl;
    
    if (         resolutionSource == ssrc::DEVICE
         ||     pixelFormatSource == ssrc::DEVICE ) applyResolutionAndPixelFormat();
    if (         brightnessSource == ssrc::DEVICE ) applyBrightness();
    if (           contrastSource == ssrc::DEVICE ) applyContrast();
    if (         saturationSource == ssrc::DEVICE ) applySaturation();
    if (          sharpnessSource == ssrc::DEVICE ) applySharpness();
    if (              gammaSource == ssrc::DEVICE ) applyGamma();
    if (       whiteBalanceSource == ssrc::DEVICE ) applyWhiteBalance();
    if (               gainSource == ssrc::DEVICE ) applyGain();
    if ( powerLineFrequencySource == ssrc::DEVICE ) applyPowerLineFrequency();
    if (           exposureSource == ssrc::DEVICE ) applyExposure();
    
    if (    errorAction != ea
         && errorAction == ErrorAction::CheckPermissions
       )
    {
        errorAction = ea;
        
        initializeSettings();
    }
    
    clog << "V4L2Cam::reapplySettings: done"
         << endl;
}

// // only to be used by locateDeviceNodeAndInitialize()
// void V4L2Cam::resetCrop(FD_t const& fd)
// {
//     // if the cam can crop, reset crop
//     v4l2_cropcap cropcap{};
//     
//     cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//     
//     if ( xioctl(fd, VIDIOC_CROPCAP, &cropcap)  )
//     {
//         v4l2_crop crop{};
//         crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // should first try *_MPLANE
//         crop.c = cropcap.defrect; /* reset to default */
//         
//         if ( !xioctl(fd, VIDIOC_S_CROP, &crop) ) [[unlikely]]
//             clog << "V4L2Cam::resetCrop: VIDIOC_S_CROP failed!"
//                  << endl;
//     }
//     else [[unlikely]]
//         clog << "V4L2Cam::resetCrop: VIDIOC_CROPCAP failed!"
//              << endl;
// }

bool V4L2Cam::isSetMemoryTypeSupported() noexcept
{
    if ( state != State::INITIALIZED ) [[unlikely]]
    {
        clog << "V4L2Cam::isSetMemoryTypeSupported: not in a correct state "
                "(INITIALIZED) to check buffer type support! (needs requesting "
                "0 buffers)"
             << endl;
        
        return false;
    }
    
    shared_ptr<FD_t> fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    
    uint32_t bufType{};
    uint32_t memType{};
    
    
    if (    !decideBufferType(bufType)
         || !bufType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::isSetMemoryTypeSupported: could not decide v4l2 "
                "buffer type!"
             << endl;
        
        return false;
    }
    
    if (    !decideMemoryType(memType)
         || !memType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::isSetMemoryTypeSupported: memory type to use unknown!"
             << endl;
        
        return false;
    }
    
    
    v4l2_requestbuffers req{};
    req.count  = 0;
    req.type   = bufType;
    req.memory = memType;
    
    if ( !xioctl(*fd_ptr, VIDIOC_REQBUFS, "VIDIOC_REQBUFS", &req) ) [[unlikely]]
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::isSetMemoryTypeSupported: device does not support set "
                "buffer type (v4l2_memory)! "
             << strerror(errNo) << endl;
        
        return false;
    }
    
    return true;
}


bool V4L2Cam::determineMaxBufferSizeNeeded(FD_t const& fd)
{
    // TODO also learn supported resolutions and supported pixel formats
    maxBufferSizeNeeded.reset();
    
    uint32_t maxSizeImage = 0;
    SUPPORTS__VIDIOC_TRY_FMT = true;
    
    uint32_t bufType{};
    
    if (    !decideBufferType(bufType)
         || !bufType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::determineMaxBufferSizeNeeded: could not decide v4l2 "
                "buffer type!"
             << endl;
        
        return false;
    }

    
    // Check if VIDIOC_TRY_FMT is supported
    v4l2_format originalFmt{};
    
    originalFmt.type = bufType;
    
    if ( !xioctl(fd, VIDIOC_G_FMT, "VIDIOC_G_FMT", &originalFmt) ) [[unlikely]]
    {
        clog << "V4L2Cam::determineMaxBufferSizeNeeded: VIDIOC_G_FMT failed!"
             << endl;
        
        return false;
    }
    
    if ( !xioctl( fd
                , VIDIOC_TRY_FMT
                , "VIDIOC_TRY_FMT"
                , &originalFmt
                , XIOCTL_FLAGS::EXPECT_ENOTTY | XIOCTL_FLAGS::EXPECT_EINVAL
                )
       )
    {
        if ( errno == ENOTTY || errno == EINVAL )
            SUPPORTS__VIDIOC_TRY_FMT = false;
        else
        {
            clog << "V4L2Cam::determineMaxBufferSizeNeeded: VIDIOC_TRY_FMT failed!"
                 << endl;
            
            return false;
        }
    }
    
    // Structure to enumerate pixel formats
    v4l2_fmtdesc fmtDesc{};
    fmtDesc.type = bufType;
    
    int errNo{};
    
    // Enumerate all supported pixel formats
    while ( xioctl( fd
                  , VIDIOC_ENUM_FMT
                  , "VIDIOC_ENUM_FMT"
                  , &fmtDesc
                  , XIOCTL_FLAGS::EXPECT_EINVAL
                  )
          )
    {
        uint32_t pixelFormat = fmtDesc.pixelformat;
        
        // Structure to enumerate frame sizes
        v4l2_frmsizeenum frmSize{};
        
        frmSize.pixel_format = pixelFormat;
        
        // Enumerate all frame sizes for the pixel format
        while ( xioctl( fd
                      , VIDIOC_ENUM_FRAMESIZES
                      , "VIDIOC_ENUM_FRAMESIZES"
                      , &frmSize
                      , XIOCTL_FLAGS::EXPECT_EINVAL
                      )
              )
        {
            uint32_t width {};
            uint32_t height{};
            
            switch ( frmSize.type )
            {
                case V4L2_FRMSIZE_TYPE_DISCRETE:
                    width  = frmSize.discrete.width;
                    height = frmSize.discrete.height;
                    break;
                case V4L2_FRMSIZE_TYPE_STEPWISE:   [[fallthrough]];
                case V4L2_FRMSIZE_TYPE_CONTINUOUS:
                    width  = frmSize.stepwise.max_width;
                    height = frmSize.stepwise.max_height;
                    break;
                default:
                    clog << "V4L2Cam::determineMaxBufferSizeNeeded: Unknown "
                            "frame size type."
                         << endl;
                    
                    ++frmSize.index;
                    continue;
            }
            
            v4l2_format fmt{};
            fmt.type = bufType;
            
            if ( bufType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE )
            {
                fmt.fmt.pix_mp.pixelformat = pixelFormat;
                fmt.fmt.pix_mp.field       = V4L2_FIELD_ANY;

                fmt.fmt.pix_mp.width       = width;
                fmt.fmt.pix_mp.height      = height;
            }
            else
            {
                fmt.fmt.pix.pixelformat = pixelFormat;
                fmt.fmt.pix.field       = V4L2_FIELD_ANY;

                fmt.fmt.pix.width       = width;
                fmt.fmt.pix.height      = height;
            }
            
            // get sizeimage
            bool ret = xioctl( fd
                             ,   SUPPORTS__VIDIOC_TRY_FMT
                               ? VIDIOC_TRY_FMT
                               : VIDIOC_S_FMT
                             ,   SUPPORTS__VIDIOC_TRY_FMT
                               ? "VIDIOC_TRY_FMT"
                               : "VIDIOC_S_FMT"
                             , &fmt
                             );
            errNo = errno;
            
            if ( ret && fmt.fmt.pix.sizeimage > maxSizeImage )
                maxSizeImage = fmt.fmt.pix.sizeimage;
            
            ++frmSize.index;
        }
        errNo = errno;
        
        if ( errNo != EINVAL && errNo != 0 )
        {
            clog << "V4L2Cam::determineMaxBufferSizeNeeded: v4l2 device frame "
                    "sizes enumeration errored out (" << strerror(errNo) << ")."
                 << endl;
            
            return false;
        }
        
        ++fmtDesc.index;
    }
    errNo = errno;
    
    if ( errNo != EINVAL && errNo != 0 )
    {
        clog << "V4L2Cam::determineMaxBufferSizeNeeded: v4l2 device format "
                "enumeration errored out (" << strerror(errNo) << ")."
             << endl;
        
        return false;
    }
    
    if (    !SUPPORTS__VIDIOC_TRY_FMT
         && !xioctl(fd, VIDIOC_S_FMT, "VIDIOC_S_FMT", &originalFmt)
       ) [[unlikely]]
    {
        apiToUse = APIToUse::UNKNOWN;
        
        clog << "V4L2Cam::determineMaxBufferSizeNeeded: Restoring original "
                "format failed"
             << endl;
    }
    
    if ( maxSizeImage > 0 )
        maxBufferSizeNeeded = maxSizeImage;
    else
        clog << "V4L2Cam::determineMaxBufferSizeNeeded: Failed to determine "
                "the maximum buffer size."
             << endl;
    
    return maxBufferSizeNeeded.has_value();
}

void V4L2Cam::determineSettingDomains(FD_t const& fd)
{
    queryControlDomain( fd, V4L2_CID_BRIGHTNESS, "V4L2_CID_BRIGHTNESS"
                      , BRIGHTNESS_DOMAIN_KNOWN, BRIGHTNESS_MIN, BRIGHTNESS_MAX
                      , BRIGHTNESS_STEP);
    queryControlDomain( fd, V4L2_CID_CONTRAST, "V4L2_CID_CONTRAST"
                      , CONTRAST_DOMAIN_KNOWN, CONTRAST_MIN, CONTRAST_MAX
                      , CONTRAST_STEP);
    queryControlDomain( fd, V4L2_CID_SATURATION, "V4L2_CID_SATURATION"
                      , SATURATION_DOMAIN_KNOWN, SATURATION_MIN, SATURATION_MAX
                      , SATURATION_STEP);
    queryControlDomain( fd, V4L2_CID_SHARPNESS, "V4L2_CID_SHARPNESS"
                      , SHARPNESS_DOMAIN_KNOWN, SHARPNESS_MIN, SHARPNESS_MAX
                      , SHARPNESS_STEP);
    queryControlDomain( fd, V4L2_CID_GAMMA, "V4L2_CID_GAMMA", GAMMA_DOMAIN_KNOWN
                      , GAMMA_MIN, GAMMA_MAX, GAMMA_STEP);
    queryControlDomain( fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE
                      , "V4L2_CID_WHITE_BALANCE_TEMPERATURE"
                      , WHITE_BALANCE_DOMAIN_KNOWN, WHITE_BALANCE_MIN
                      , WHITE_BALANCE_MAX, WHITE_BALANCE_STEP);
    queryControlDomain( fd, V4L2_CID_GAIN, "V4L2_CID_GAIN", GAIN_DOMAIN_KNOWN
                      , GAIN_MIN, GAIN_MAX, GAIN_STEP);
    // queryControlDomain(fd, V4L2_CID_EXPOSURE, EXPOSURE_DOMAIN_KNOWN,
    //                    EXPOSURE_MIN, EXPOSURE_MAX, EXPOSURE_STEP);
    queryControlDomain( fd, V4L2_CID_EXPOSURE_ABSOLUTE
                      , "V4L2_CID_EXPOSURE_ABSOLUTE", EXPOSURE_DOMAIN_KNOWN
                      , EXPOSURE_MIN, EXPOSURE_MAX, EXPOSURE_STEP);
}

// TODO add param cidStr
void V4L2Cam::queryControlDomain( FD_t        const& fd
                                , uint32_t           controlID
                                , string_view const  cidStr
                                , bool             & domainKnown
                                , int32_t          & min
                                , int32_t          & max
                                , int32_t          & step
                                )
{
    v4l2_queryctrl queryctrl{};
    queryctrl.id = controlID;
    
    // Call ioctl directly so we can treat EINVAL/EDOM as "unsupported" quietly.
    if ( !xioctl( fd
                , VIDIOC_QUERYCTRL
                , "VIDIOC_QUERYCTRL"
                , &queryctrl
                , XIOCTL_FLAGS::EXPECT_EINVAL | XIOCTL_FLAGS::EXPECT_EDOM
                )
       )
    {
        int const errNo = errno;
        
        // Unsupported / not applicable / driver oddities: skip
        if ( errNo == EINVAL || errNo == EDOM )
        {
            clog << "V4L2Cam::queryControlDomain: VIDIOC_QUERYCTRL failed - for "
                    "(probably) benign reason - for "
                 << cidStr << ": " << strerror(errNo)
                 << endl;
            
            return;
        }
        
        // Otherwise, real error worth logging (I/O, bad fd, etc.).
        clog << "V4L2Cam::queryControlDomain: VIDIOC_QUERYCTRL failed for "
             << cidStr << ": " << strerror(errNo) << "!"
             << endl;
        
        errno = errNo;
        return;
    }
    
    if ( queryctrl.flags & V4L2_CTRL_FLAG_DISABLED )
        return;
    
    // Only publish a numeric domain for numeric controls.
    switch ( queryctrl.type )
    {
        case V4L2_CTRL_TYPE_INTEGER:
            domainKnown = true;
            min  = queryctrl.minimum;
            max  = queryctrl.maximum;
            step = std::max<int32_t>( 1, queryctrl.step );
            
            return;
        
#ifdef V4L2_CTRL_TYPE_INTEGER64
        case V4L2_CTRL_TYPE_INTEGER64:
            // Optional: upgrade to QUERY_EXT_CTRL to get full 64-bit min/max/step.
            // For now, don’t pretend we have a 32-bit domain.
            return;
#endif
        
        // MENU/BOOLEAN/BUTTON/STRING/etc. don’t have a numeric [min,max,step] domain.
        default:
            return;
    }
}


shared_ptr<V4L2Cam::FD_t> V4L2Cam::produceV4L2FD()
{
    if ( v4l2Path.empty() ) [[unlikely]]
        return shared_ptr<FD_t>();
    if ( v4l2FD && *v4l2FD )
        return v4l2FD;
    else
        return make_shared<FD_t>(xopen( v4l2Path.c_str()
                                      , O_RDWR | O_NONBLOCK
                                      )
                                );
}

// bool V4L2Cam::openV4L2FD() {
//     if ( v4l2Path.empty() ) [[unlikely]]
//         return false;
//     if ( !v4l2FD || !*v4l2FD )
//         v4l2FD = make_shared<FD_t>(xopen( v4l2Path.c_str()
//                                         , O_RDWR | O_NONBLOCK
//                                         )
//                                   );
//     
//     return (bool)v4l2FD && (bool)*v4l2FD;
// }

void V4L2Cam::closeV4L2FD()
{
    if ( v4l2FD && !*v4l2FD )
    {
        v4l2FD->close_fd();
        
        clog << "V4L2Cam::closeV4L2FD: device fd explicitly closed"
             << endl;
    }
    
    v4l2FD.reset();
}


bool V4L2Cam::tryAndStopStreaming(bool hard) noexcept
{
    static constexpr size_t maxTries{10};
    
    size_t tries{};
    
    while (    state == State::DEQUEUEING
            || state == State::STREAMING
          ) [[unlikely]]
    {
        wake();
        this_thread::sleep_for(chrono::milliseconds(100));
        stopStreaming();
        
        if ( errorAction == ErrorAction::ForgetDevice ) [[unlikely]]
        {
            clog << "V4L2Cam::tryAndStopStreaming: device seems to be gone!"
                 << endl;
            
            return false;
        }
        
        if ( tries++ >= maxTries ) [[unlikely]]
        {
            clog << "V4L2Cam::tryAndStopStreaming: cannot stop streaming!"
                    
                 << endl;
            
            if ( hard )
            {
                clog << " KILLING THIS PROCESS!!!"
                     << endl;
                
                std::terminate();
            }
            
            clog << endl;
            
            return false;
        }
    }
    
    return true;
}


void V4L2Cam::uninitialize(bool hard) noexcept
{
    if (     state != State::UNINITIALIZED
         &&  state != State::DEVICE_KNOWN
         &&  state != State::INITIALIZED
         &&  errorAction != ErrorAction::ForgetDevice
         && !hard
       ) [[unlikely]]
    {
        clog << "V4L2Cam::uninitialize: not in a correct state (UNINITIALIZED|"
                "DEVICE_KNOWN|INITIALIZED) to fully uninitialize!"
             << endl;
        
        return;
    }
    
    
    if ( !superObjectCannotExist ) [[  likely]]
        _uninitialize();
    
    closeV4L2FD();
    v4l2Path.clear();
    USBKernelName.clear();
    USBBusNumber.clear();
    USBDeviceAddress.clear();
    USBPortDisablePathSelf.clear();
    USBPortDisablePathPeer.clear();
    
    state = state == State::UNINITIALIZED
          ? State::UNINITIALIZED
          : State::DEVICE_KNOWN;
    
    bufferCount   = 0;
    buffersQueued = {};
    bufferPlanes  = {};
    
//     // dev and dbg phase only. non-sensical, when there are multiple V4L2Cams
//     if ( FD_t::warnSFSC() ) [[unlikely]]
//     {
//         clog << "V4L2Cam::uninitialize: there are left open FDs! report follows:"
//              << endl;
//         
//         FD_t::logSFSC();
//     }
}

bool V4L2Cam::rebindUSBDevice() noexcept
{
    namespace fs = std::filesystem;
    
    fs::path const devDir{"/sys/bus/usb/devices/" + storedUSBID.kernelName};
    
    if ( !fs::exists(devDir) )
    {
        clog << "V4L2Cam::rebindUSBDevice: sysfs device dir missing: " << devDir
             << endl;
        
        return false;
    }
    
    struct Iface
    {
        string driver{};   // e.g. "uvcvideo"
        string iface{};    // e.g. "2-2:1.0"
    };
    
    vector<Iface> ifaces{};
    
    // Enumerate <usbKernelName>:1.*
    for ( auto const& entry : fs::directory_iterator(devDir) )
    {
        if ( !entry.is_directory() ) continue;
        
        auto const name = entry.path().filename().string(); // maybe "2-2:1.0"
        
        if ( name.rfind(storedUSBID.kernelName + ":", 0) != 0 )
            continue; // not an interface
        
        fs::path driverLink = entry.path() / "driver";
        
        if ( !fs::exists(driverLink) ) [[unlikely]]
            continue; // unbound interface (rare)
        
        error_code ec{};
        fs::path   resolved = fs::read_symlink(driverLink, ec);
        
        if ( ec ) [[unlikely]]
            continue;
        
        string driver = resolved.filename().string(); // basename e.g. "uvcvideo"
        
        if ( driver.empty() ) [[unlikely]]
            continue;
        
        ifaces.push_back({ driver, name });
    }
    
    if ( ifaces.empty() ) [[unlikely]]
    {
        clog << "V4L2Cam::rebindUSBDevice: no bound interfaces found under "
             << devDir
             << endl;
        
        return false;
    }
    
    auto openAttr = [] (char const* driver, char const* attr) -> ofstream
    {
        string p = string("/sys/bus/usb/drivers/") + driver + "/" + attr;
        ofstream f(p);
        
        return f;
    };
    
    uint8_t unboundCount{}, reboundCount{};
    
    // Unbind: uvcvideo first (keeps ordering tidy), then others.
    stable_sort( ifaces.begin()
               , ifaces.end()
               , [] (Iface const& a, Iface const& b)
                 {
                     bool au = (a.driver == "uvcvideo");
                     bool bu = (b.driver == "uvcvideo");
                     
                     if ( au == bu ) return a.iface < b.iface;
                     else            return au;
                 }
               );
    
    {
        string   lastDriver;
        ofstream attr;
        
        for ( auto const& i : ifaces )
        {
            if ( i.driver != lastDriver )
            {
                attr = openAttr(i.driver.c_str(), "unbind");
                
                if ( !attr.is_open() )
                {
                    clog << "V4L2Cam::rebindUSBDevice: Failed to open unbind "
                            "for driver " << i.driver << "!"
                         << endl;
                    
                    continue;
                }
                
                lastDriver = i.driver;
            }
            
            attr << i.iface << '\n';
            attr.close();
            clog << "V4L2Cam::rebindUSBDevice: Unbound " << i.iface
                 << " from " << i.driver
                 << endl;
            
            ++unboundCount;
        }
    }
    
    this_thread::sleep_for(chrono::milliseconds(150));
    
    {
        string   lastDriver;
        ofstream attr;
        
        for ( auto const& i : ifaces )
        {
            if ( i.driver != lastDriver )
            {
                attr = openAttr(i.driver.c_str(), "bind");
                
                if ( !attr.is_open() )
                {
                    clog << "V4L2Cam::rebindUSBDevice: Failed to open bind for "
                            "driver " << i.driver
                         << endl;
                    
                    continue;
                }
                
                lastDriver = i.driver;
            }
            
            attr << i.iface << '\n';
            attr.close();
            clog << "V4L2Cam::rebindUSBDevice: Rebound " << i.iface
                 << " to   " << i.driver
                 << endl;
            
            ++reboundCount;
        }
    }
    
    if ( reboundCount > 0 )
        this_thread::sleep_for(chrono::milliseconds(1500));
    
    return    unboundCount >  0
           && reboundCount == unboundCount;
}


bool V4L2Cam::resetUSBDevice() noexcept
{
    // /dev/bus/usb/%03u/%03u (padded)
    char path[64]{};
    std::snprintf( path
                 , sizeof(path)
                 , "/dev/bus/usb/%03lu/%03lu"
                 , std::stoul(storedUSBID.busNumber)
                 , std::stoul(storedUSBID.deviceAddress)
                 );
    
    FD_t fd{xopen(path, O_RDWR)};
    
    if ( !fd ) [[unlikely]]
    {
        clog << "V4L2Cam::resetUSBDevice: open " << path << " failed"
             << endl;
        
        return false;
    }
    
    if ( ioctl(fd, USBDEVFS_RESET, 0) < 0 ) [[unlikely]]
    {
        clog << "V4L2Cam::resetUSBDevice: USBDEVFS_RESET failed"
             << endl;
        
        return false;
    }
    
    // Give the link time to debounce and enumerate a bit.
    this_thread::sleep_for(chrono::milliseconds(5000));
    
    return true;
}


// TODO even at a hub at which you effectively cannot power cycle a single port
//      this measure seems to have a resetting effect. Make use of that!
bool V4L2Cam::powerCycleUSBDevice() noexcept
{
    if ( storedUSBID.portDisablePathSelf.empty() ) [[unlikely]]
    {
        clog << "V4L2Cam::powerCycleUSBDevice: no disable path to write \"1\" to!"
             << endl;
        
        return false;
    }
    
    array<fs::path const*, 2> const usbPortDisableFiles
    {
        &storedUSBID.portDisablePathSelf,
        &storedUSBID.portDisablePathPeer
    };
    
    size_t turnedOffCount{};
    size_t      portCount = (   storedUSBID.portDisablePathPeer.empty()
                              ? 1
                              : 2
                            );
    
    for ( size_t i{}; i < portCount; ++i)
    {
        auto const* path = usbPortDisableFiles[i]->c_str();
        
        FD_t fd{xopen(path, O_WRONLY | O_CLOEXEC)};
        
        auto errNo = errno;
        
        if ( fd < 0 ) [[unlikely]]
        {
            clog << "V4L2Cam::powerCycleUSBDevice: opening \""
                 << path << "\" failed: " << strerror( errNo )
                 << endl;
            
            break;
        }
        
        char   const* val = "1\n";
        size_t const  len = strlen(val);
        
        if ( !xwrite(fd, val, len) ) [[unlikely]]
        {
            clog << "V4L2Cam::powerCycleUSBDevice: writing to \""
                 << path << "\" failed: "
                 << strerror( errNo ) << '\n';
            
            break;
        }
        
        ++turnedOffCount;
    }
    
    clog << "V4L2Cam::powerCycleUSBDevice: pulled power down on"
         << to_string(turnedOffCount) << " ports of "
         << to_string(portCount)
         << endl;
    
    
    // HACK should check power status of one of the ports to make sure, but can't
    // be bothered, right now
    
    this_thread::sleep_for(chrono::milliseconds(750));
    
    
    size_t turnedOnCount{};
    
    for ( size_t i{}; i < turnedOffCount; ++i )
    {
        auto const* path = usbPortDisableFiles[i]->c_str();
        
        FD_t fd{xopen(usbPortDisableFiles[i]->c_str(), O_WRONLY | O_CLOEXEC)};
        
        auto errNo = errno;
        
        if ( fd < 0 ) [[unlikely]]
        {
            clog << "V4L2Cam::powerCycleUSBDevice: opening \""
                 << path << "\" failed: " << strerror( errNo )
                 << endl;
            
            continue;
        }
        
        char   const* val = "0\n";
        size_t const  len = strlen(val);
        
        if ( !xwrite(fd, val, len) ) [[unlikely]]
        {
            clog << "V4L2Cam::powerCycleUSBDevice: writing to \""
                 << path << "\" failed: "
                 << strerror( errNo ) << '\n';
            
            continue;
        }
        
        ++turnedOnCount;
    }
    
    clog << "V4L2Cam::powerCycleUSBDevice: pulled power up   on "
         << to_string(turnedOnCount) << " ports of "
         << to_string(portCount)
         << endl;
    
    if ( turnedOnCount < turnedOffCount ) [[unlikely]]
    {
        clog << "V4L2Cam::powerCycleUSBDevice: left ports "
                "powered down! (logically that is)"
             << endl;
    }
    
    if (    turnedOffCount == portCount
         && turnedOnCount  == 0
       ) [[unlikely]]
       clog << "V4L2Cam::powerCycleUSBDevice: left all ports "
               "powered down! (actually)"
            << endl;
    
    
    // all pulled down := they actually lost power
    // at least one up := they all have power
    bool cycled =    turnedOffCount == portCount
                  && turnedOnCount   > 0;
    
    if ( cycled )
        this_thread::sleep_for(chrono::milliseconds(5000));
    
    return cycled;
}


// bool V4L2Cam::resetAtUSBHubPort( string const& usbBusNumber
//                                , string const& usbDeviceAddress
//                                // , bool          powerCycle
//                                ) noexcept
// {
//     libusb_context*          ctx        = nullptr;
//     libusb_device_handle*    hub_handle = nullptr;
//     // libusb_device_descriptor hub_desc{};
//     uint8_t                  port{};
//     
//     bool success{true};
//     
//     int32_t res = libusb_init(&ctx);
//     
//     if ( res < 0 )
//     {
//         clog << "V4L2Cam::resetAtUSBPort: Failed to initialize libusb: "
//              << libusb_error_name(res)
//              << endl;
//         
//         return false;
//     }
//     
//     success &= produceHubHandleAndPortNumber( usbBusNumber
//                                             , usbDeviceAddress
//                                             , ctx
//                                             , hub_handle
//                                             // , hub_desc
//                                             , port
//                                             );
//     
//     constexpr uint16_t PORT_POWER       = 8;
//     // constexpr uint16_t PORT_RESET       = 4;
//     constexpr uint8_t  REQTYPE_PORT_OUT = static_cast<uint8_t>(LIBUSB_ENDPOINT_OUT      )
//                                         | static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_CLASS)
//                                         | static_cast<uint8_t>(LIBUSB_RECIPIENT_OTHER   );
//     
//     if ( !success ) [[unlikely]]
//     {
//         clog << "V4L2Cam::resetAtUSBPort: couldn't identify device's hub and "
//                 "port!"
//              << endl;
//         
//         goto RESET_AT_USB_PORT_END;
//     }
//     
//     // if ( powerCycle )
//     {
//         res = libusb_control_transfer( hub_handle
//                                      , REQTYPE_PORT_OUT
//                                      , LIBUSB_REQUEST_CLEAR_FEATURE
//                                      , PORT_POWER
//                                      , port
//                                      , NULL
//                                      , 0
//                                      , 1000
//                                      );
//         success &= res >= 0;
//         
//         if ( !success ) [[unlikely]]
//         {
//             clog << "V4L2Cam::resetAtUSBPort: Failed to power off the device:"
//                  << libusb_error_name(res)
//                  << endl;
//             
//             goto RESET_AT_USB_PORT_END;
//         }
//         
//         this_thread::sleep_for(chrono::milliseconds(750));
//         
//         
//         static constexpr uint8_t max_attempts{10};
//         uint8_t                  attempt{};
//         auto                     sleep_base = chrono::milliseconds(200);
//         auto                     backoff    = chrono::milliseconds( 50);
//         
//         success = false;
//         
//         do {
//             this_thread::sleep_for(sleep_base + backoff);
//             
//             res = libusb_control_transfer( hub_handle
//                                          , REQTYPE_PORT_OUT
//                                          , LIBUSB_REQUEST_SET_FEATURE
//                                          , PORT_POWER
//                                          , port
//                                          , NULL
//                                          , 0
//                                          , 1000
//                                          );
//             success |= res >= 0;
//             
//             ++attempt;
//             
//             if ( backoff < chrono::milliseconds(800) )
//                 backoff *= 2;
//         }
//         while (    !success
//                 &&  attempt < max_attempts
//               );
//         
//         if ( !success )
//         {
//             clog << "V4L2Cam::resetAtUSBPort: Failed to power on the device:"
//                  << libusb_error_name(res)
//                  << endl;
//             
//             goto RESET_AT_USB_PORT_END;
//         }
//         
//         // Give the link time to debounce and enumerate a bit.
//         this_thread::sleep_for(chrono::milliseconds(1200));
//     }
// //     else /* just reset */
// //     {
// //         res = libusb_control_transfer( hub_handle
// //                                      , REQTYPE_PORT_OUT
// //                                      , LIBUSB_REQUEST_SET_FEATURE
// //                                      , PORT_RESET
// //                                      , port
// //                                      , NULL
// //                                      , 0
// //                                      , 1000
// //                                      );
// //         success &= res >= 0;
// //         
// //         if ( !success ) [[unlikely]]
// //         {
// //             clog << "V4L2Cam::resetAtUSBPort: Failed to port reset on the device: "
// //                  << libusb_error_name(res)
// //                  << endl;
// // 
// //             goto RESET_AT_USB_PORT_END;
// //         }
// //         
// //         // Give the link time to debounce and enumerate a bit.
// //         this_thread::sleep_for(chrono::milliseconds(400));
// //     }
//     
//     
// RESET_AT_USB_PORT_END:
//     
//     if ( hub_handle != nullptr )
//         libusb_close(hub_handle);
//     libusb_exit(ctx);
//     
//     return success;
// }

// bool V4L2Cam::produceHubHandleAndPortNumber( string const&           usbBusNumber
//                                            , string const&           usbDeviceAddress
//                                            , libusb_context *        ctx
//                                            , libusb_device_handle *& hub_handle
//                                            , uint8_t&                port
//                                            ) noexcept
// {
//     hub_handle = nullptr;
//     
//     bool found{false};
//     int res = 0;
//     libusb_device **dev_list = nullptr;
//     ssize_t cnt;
//     
//     uint8_t target_busnum = static_cast<uint8_t>(std::stoul(usbBusNumber));
//     uint8_t target_devnum = static_cast<uint8_t>(std::stoul(usbDeviceAddress));
//     
//     cnt = libusb_get_device_list(ctx, &dev_list);
//     
//     if ( cnt < 0 )
//     {
//         clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to get device "
//                 "list: " << libusb_error_name(static_cast<int>(cnt))
//              << endl;
//         
//         return false;
//     }
//     
//     for ( ssize_t i = 0; i < cnt; i++ )
//     {
//         libusb_device* device = dev_list[i];
//         
//         if (    libusb_get_bus_number    (device) != target_busnum
//              || libusb_get_device_address(device) != target_devnum
//            )
//             continue;
//         
//         // Get the parent device (the hub)
//         libusb_device* parent = libusb_get_parent(device);
//         
//         if ( !parent )
//         {
//             clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to get "
//                     "parent device (hub)"
//                  << endl;
//             
//             break;
//         }
//         
//         res = libusb_open(parent, &hub_handle);
//         
//         if ( res < 0 )
//         {
//             clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to open "
//                     "hub device: " << libusb_error_name(res)
//                  << endl;
//             
//             break;
//         }
//         
// //         res = libusb_get_device_descriptor(parent, &hub_desc);
// //         
// //         if ( res < 0 )
// //         {
// //             clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to get hub "
// //                     "descriptor: " << libusb_error_name(res)
// //                  << endl;
// //                  
// //             libusb_close(hub_handle);
// //             hub_handle = nullptr;
// //             
// //             break;
// //         }
//         
//         port = libusb_get_port_number(device);
//         found = true;
//         break;
//     }
//     
//     libusb_free_device_list(dev_list, 1);
//     
//     if ( !found )
//         clog << "V4L2Cam::produceHubHandleAndPortNumber: Device not found"
//              << endl;
//     
//     return found;
// }



bool V4L2Cam::decideBufferType(uint32_t& bufType) noexcept
{
    if ( apiToUse == APIToUse::UNKNOWN ) [[unlikely]]
    {
        giveV4L2Format(); // learning apiToUse is a side-effect
        
        if ( apiToUse == APIToUse::UNKNOWN )
        {
            clog << "V4L2Cam::decideBufferType: Soemthing's off. Can't learn "
                    "the API to use (single- vs multi-planar)!"
                 << endl;
            
            return false;
        }
    }
    
    
    switch ( apiToUse )
    {
        case APIToUse::MULTI : bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE; break;
        case APIToUse::SINGLE: bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE       ; break;
        
        default: [[unlikely]] // [[impossible]] rather
            bufType = 0;
            
            clog << "V4L2Cam::decideBufferType: device wants us to use an API "
                    "unknown to us!"
                 << endl;
            
            return false;
    }
    
    return true;
}

bool V4L2Cam::decideMemoryType(uint32_t& memType) noexcept
{
    if (    memoryType == MemoryType::UNKNOWN
         || memoryType == MemoryType::MMAP
       ) [[unlikely]]
    {
        clog << "V4L2Cam::decideMemoryType: not set (by user) to a supported "
                "memory type!"
             << endl;
        
        return false;
    }
    
    
    switch ( memoryType )
    {
        case MemoryType::USERPTR: memType = V4L2_MEMORY_USERPTR; break;
        case MemoryType::DMABUF : memType = V4L2_MEMORY_DMABUF ; break;
        
        default: [[unlikely]] // [[impossible]] rather(, unless so. invented new ones)
            clog << "V4L2Cam::decideMemoryType: user asks for unknown memory "
                    "type!"
                 << endl;
            
            return false;
    }
    
    return true;
}



/* ********************** */
/* ********************** */
/* Settings related Stuff */
/* ********************** */
/* ********************** */


bool V4L2Cam::fetch_control_value( shared_ptr<FD_t> fd_ptr
                                 , uint32_t         id
                                 , string_view const idStr
                                 , int32_t&         value
                                 )
{
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_control ctrl{id, {}};
    
    if ( !xioctl(*fd_ptr, VIDIOC_G_CTRL, "VIDIOC_G_CTRL", &ctrl) ) [[unlikely]]
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::fetch_control_value: VIDIOC_G_CTRL with id \'"
             << idStr << "\" failed (" << strerror(errNo) << ")."
             << endl;
        
        errno = errNo;
        return false;
    }
    
    value = ctrl.value;
    
    return true;
}

bool V4L2Cam::apply_control_value( shared_ptr<FD_t>  fd_ptr
                                 , uint32_t          id
                                 , string_view const idStr
                                 , int32_t const     value
                                 )
{
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_control ctrl{id, value};
    
    if ( !xioctl(*fd_ptr, VIDIOC_S_CTRL, "VIDIOC_S_CTRL", &ctrl) ) [[unlikely]]
    {
        int const errNo = errno;
        
        clog << "V4L2Cam::apply_control_value: VIDIOC_S_CTRL with id \""
             << idStr << "\" and value \"" << to_string(value)
             << "\" failed (" << strerror(errNo) << ")."
             << endl;
        
        errno = errNo;
        return false;
    }
    
    return true;
}

//  public:
optional<v4l2_format> V4L2Cam::giveV4L2Format()
{
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return {};
    
    v4l2_format format{};
    
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if ( xioctl( *fd_ptr
               , VIDIOC_G_FMT
               , "VIDIOC_G_FMT"
               , &format
               , XIOCTL_FLAGS::EXPECT_EINVAL
               )
       )
    {
        apiToUse = APIToUse::MULTI;
        
        return format;
    }
    
    format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if ( xioctl(*fd_ptr, VIDIOC_G_FMT, "VIDIOC_G_FMT", &format) )
    {
        apiToUse = APIToUse::SINGLE;
        
        return format;
    }
    
    apiToUse = APIToUse::UNKNOWN;
    
    return {};
}


/* ********* */
/* Framerate */
/* ********* */

bool V4L2Cam::requestFramerate(uint8_t fps) noexcept
{
    if ( !_requestFramerate(fps) ) [[unlikely]]
    {
        clog << "V4L2Cam::requestFramerate: implementation didn't accept "
                "framerate!"
             << endl;
        
        return false;
    }
    
    return true;
}

optional<uint8_t> V4L2Cam::produceFramerate() noexcept
{
    auto fps_opt = _produceFramerate();
    
    if ( !fps_opt.has_value() ) [[unlikely]]
        clog << "V4L2Cam::produceFramerate: implementation could not produce "
                "in-effect framerate!"
             << endl;
    
    return fps_opt;
}


/* ************************** */
/* Resolution and PixelFormat */
/* ************************** */

bool V4L2Cam::giveResolution(uint32_t & _width, uint32_t & _height)
{
    if ( !checkResolution() ) [[unlikely]]
        return false;
    
    _width  = width .value();
    _height = height.value();
    
    return true;
}

bool V4L2Cam::takeResolution(uint32_t const _width, uint32_t const _height)
{
    if ( !checkResolution(_width, _height) ) [[unlikely]]
        return false;
    
    width            = _width;
    height           = _height;
    resolutionSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkResolution(uint32_t const _width, uint32_t const _height)
{
    if ( RESOLUTION_DOMAIN_KNOWN ) {
        // TODO check against known value domain // NIY
        return false;
    } else {
        return _checkResolution(_width, _height);
    }
}

bool V4L2Cam::checkResolution()
{
    return    width.has_value() && height.has_value()
           && checkResolution(width.value(), height.value());
}


bool V4L2Cam::givePixelFormat(PixelFormat & _pixelFormat)
{
    if ( !checkPixelFormat() ) [[unlikely]]
        return false;
    
    _pixelFormat = pixelFormat.value();
    
    return true;
}

bool V4L2Cam::takePixelFormat(PixelFormat const _pixelFormat)
{
    if ( !checkPixelFormat(_pixelFormat) ) [[unlikely]]
        return false;
    
    pixelFormat       = _pixelFormat;
    pixelFormatSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkPixelFormat(PixelFormat const _pixelFormat)
{
    if ( PIXEL_FORMAT_DOMAIN_KNOWN ) {
        // TODO check against known value domain // NIY
        return false;
    } else {
        return _checkPixelFormat(_pixelFormat);
    }
}

bool V4L2Cam::checkPixelFormat()
{
    return    pixelFormat.has_value()
           && checkPixelFormat(pixelFormat.value());
}


bool V4L2Cam::fetchResolutionAndPixelFormat()
{
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_format format{};
                format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    
    if ( xioctl( *fd_ptr
               , VIDIOC_G_FMT
               , "VIDIOC_G_FMT"
               , &format
               , XIOCTL_FLAGS::EXPECT_EINVAL
               )
       )
        updateResNPixFmtFromDeviceInfo(format);
    else
    {
        format = {};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        
        if ( xioctl(*fd_ptr, VIDIOC_G_FMT, "VIDIOC_G_FMT", &format) )
            updateResNPixFmtFromDeviceInfo(format);
        else
            resetResNPixFmt();
    }
    
    return     resolutionSource == ssrc::DEVICE
           && pixelFormatSource == ssrc::DEVICE
           && currentBufferSizeNeeded;
}

bool V4L2Cam::applyResolutionAndPixelFormat()
{
    if (    (  resolutionSource == ssrc::NONE || !checkResolution () )
         && ( pixelFormatSource == ssrc::NONE || !checkPixelFormat() )
       )
        return false;
    
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_format format{};
    
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    
    if ( xioctl( *fd_ptr
               , VIDIOC_G_FMT
               , "VIDIOC_G_FMT"
               , &format
               , XIOCTL_FLAGS::EXPECT_EINVAL
               )
       )
    {
        apiToUse = APIToUse::MULTI;
        
        if ( resolutionSource != ssrc::NONE && checkResolution() )
        {
            format.fmt.pix_mp.width  = width .value();
            format.fmt.pix_mp.height = height.value();
        }
        if ( pixelFormatSource != ssrc::NONE && checkPixelFormat() )
            format.fmt.pix_mp.pixelformat = enum_integer(pixelFormat.value());
        
        // format.fmt.pix_mp.field = V4L2_FIELD_INTERLACED; // from e-con's example. strange
        format.fmt.pix_mp.field = V4L2_FIELD_NONE;
    }
    else
    {
        format = {};
        
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        
        if ( xioctl(*fd_ptr, VIDIOC_G_FMT, "VIDIOC_G_FMT", &format) )
        {
            apiToUse = APIToUse::SINGLE;
            
            if ( resolutionSource != ssrc::NONE && checkResolution() )
            {
                format.fmt.pix.width  = width .value();
                format.fmt.pix.height = height.value();
            }
            if ( pixelFormatSource != ssrc::NONE && checkPixelFormat() )
                format.fmt.pix.pixelformat = enum_integer(pixelFormat.value());
            
            // format.fmt.pix.field = V4L2_FIELD_INTERLACED; // from e-con's example. strange
            format.fmt.pix.field = V4L2_FIELD_NONE;
        }
        else
        {
            resetResNPixFmt();
            
            return false;
        }
    }
    
    
    if ( xioctl(*fd_ptr, VIDIOC_S_FMT, "VIDIOC_S_FMT", &format) ) [[  likely]]
        updateResNPixFmtFromDeviceInfo(format);
    else
        resetResNPixFmt();
    
    return     resolutionSource == ssrc::DEVICE
           && pixelFormatSource == ssrc::DEVICE
           && currentBufferSizeNeeded;
}

void V4L2Cam::updateResNPixFmtFromDeviceInfo(v4l2_format const& format) noexcept
{
    if ( format.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE )
    {
        apiToUse = APIToUse::MULTI;
        
        if ( checkResolution(format.fmt.pix_mp.width, format.fmt.pix_mp.height) )
        {
            width            = static_cast<uint32_t>(format.fmt.pix_mp.width);
            height           = static_cast<uint32_t>(format.fmt.pix_mp.height);
            resolutionSource = ssrc::DEVICE;
            
            if ( !checkResolution(format.fmt.pix_mp.width, format.fmt.pix_mp.height) )
            [[unlikely]]
                clog << "V4L2Cam::updateResNPixFmtFromDeviceInfo: resolution "
                        "reported by device not valid according to our domain "
                        "knowledge!"
                     << endl;
        }
        else
        {
            width .reset();
            height.reset();
            resolutionSource = ssrc::NONE;
        }
        
        pixelFormat = to_PixelFormat(format.fmt.pix_mp.pixelformat);
        
        if ( !pixelFormat && format.fmt.pix_mp.pixelformat > 0 ) [[unlikely]]
            clog << "V4L2Cam::updateResNPixFmtFromDeviceInfo: pixel format "
                    "reported by device not valid according to our domain "
                    "knowledge!"
                 << endl;
        
        if ( pixelFormat ) pixelFormatSource = ssrc::DEVICE;
        else               pixelFormatSource = ssrc::NONE;
        
        if (    format.fmt.pix_mp.             num_planes == 1 // currently, we support no more
             && format.fmt.pix_mp.plane_fmt[0].sizeimage   > 0
           ) [[  likely]]
            currentBufferSizeNeeded
             =  static_cast<uint32_t>(format.fmt.pix_mp.plane_fmt[0].sizeimage);
        else
            currentBufferSizeNeeded.reset();
    }
    else /* format.type == V4L2_BUF_TYPE_VIDEO_CAPTURE */
    {
        apiToUse = APIToUse::SINGLE;
        
        if ( checkResolution(format.fmt.pix.width, format.fmt.pix.height) ) {
            width            = format.fmt.pix.width;
            height           = format.fmt.pix.height;
            resolutionSource = ssrc::DEVICE;
            
            if ( !checkResolution(format.fmt.pix.width, format.fmt.pix.height) )
            [[unlikely]]
                clog << "V4L2Cam::updateResNPixFmtFromDeviceInfo: resolution "
                        "reported by device not valid according to our domain "
                        "knowledge!"
                     << endl;
        }
        else
        {
            width .reset();
            height.reset();
            resolutionSource = ssrc::NONE;
        }
        
        pixelFormat = to_PixelFormat(format.fmt.pix.pixelformat);
        
        if ( !pixelFormat && format.fmt.pix_mp.pixelformat > 0 ) [[unlikely]]
            clog << "V4L2Cam::updateResNPixFmtFromDeviceInfo: pixel format "
                    "reported by device not valid according to our domain "
                    "knowledge!"
                 << endl;
        
        if ( pixelFormat ) pixelFormatSource = ssrc::DEVICE;
        else               pixelFormatSource = ssrc::NONE;
        
        if ( format.fmt.pix.sizeimage > 0 )
            currentBufferSizeNeeded = format.fmt.pix.sizeimage;
        else
            currentBufferSizeNeeded.reset();
    }
}

void V4L2Cam::resetResNPixFmt() noexcept
{
    apiToUse = APIToUse::UNKNOWN;
    
    width      .reset();
    height     .reset();
    pixelFormat.reset();
    
    resolutionSource = ssrc::NONE;
    pixelFormatSource = ssrc::NONE;
    
    currentBufferSizeNeeded.reset();
}


bool V4L2Cam::giveMaxBufferSizeNeeded(uint32_t& mbs)
{
    if ( !maxBufferSizeNeeded.has_value() ) [[unlikely]]
        return false;
    
    mbs = maxBufferSizeNeeded.value();
    
    return true;
}

bool V4L2Cam::giveCurrentBufferSizeNeeded(uint32_t& cbs)
{
    if ( !currentBufferSizeNeeded.has_value() ) [[unlikely]]
        return false;
    
    cbs = currentBufferSizeNeeded.value();
    
    return true;
}



/* ********** */
/* Brightness */
/* ********** */

bool V4L2Cam::giveBrightness(int32_t& _brightness)
{
    if ( !checkBrightness() ) [[unlikely]]
        return false;
    
    _brightness = brightness.value();
    
    return true;
}

bool V4L2Cam::takeBrightness(int32_t const _brightness)
{
    if ( !checkBrightness(_brightness) ) [[unlikely]]
        return false;
    
    brightness       = _brightness;
    brightnessSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkBrightness(int32_t const _brightness)
{
    if ( !BRIGHTNESS_DOMAIN_KNOWN ) [[unlikely]]
        return true; // can't fail early, when you know too little
    
    return    _brightness >= BRIGHTNESS_MIN
           && _brightness <= BRIGHTNESS_MAX;
}

bool V4L2Cam::checkBrightness()
{
    return brightness.has_value() && checkBrightness(brightness.value());
}

bool V4L2Cam::fetchBrightness()
{
    int32_t value{};
    
    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_BRIGHTNESS
                                      , "V4L2_CID_BRIGHTNESS"
                                      , value
                                      );
    
    if ( fetched ) { brightness       = (decltype(brightness)::value_type)(value);
                     brightnessSource = ssrc::DEVICE;
                   }
    else           { brightness.reset();
                     brightnessSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkBrightness() ) [[unlikely]]
        clog << "V4L2Cam::fetchBrightness: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return brightnessSource == ssrc::DEVICE;
}

bool V4L2Cam::applyBrightness()
{
    if ( brightnessSource == ssrc::NONE || !checkBrightness() )
        return false;
    
    int32_t value = brightness.value();
    
    if ( BRIGHTNESS_DOMAIN_KNOWN && BRIGHTNESS_STEP > 1 )
        value -= (value - BRIGHTNESS_MIN) % BRIGHTNESS_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_BRIGHTNESS
                                      , "V4L2_CID_BRIGHTNESS"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        brightness.reset();
        brightnessSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyBrightness: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchBrightness();
}


/* ********* */
/* Contrast  */
/* ********* */

bool V4L2Cam::giveContrast(int32_t& _contrast)
{
    if ( !checkContrast() ) [[unlikely]]
        return false;
    
    _contrast = contrast.value();
    
    return true;
}

bool V4L2Cam::takeContrast(int32_t const _contrast)
{
    if ( !checkContrast(_contrast) ) [[unlikely]]
        return false;
    
    contrast       = _contrast;
    contrastSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkContrast(int32_t const _contrast)
{
    if ( !CONTRAST_DOMAIN_KNOWN ) [[unlikely]]
        return true;
    
    return    _contrast >= CONTRAST_MIN
           && _contrast <= CONTRAST_MAX;
}

bool V4L2Cam::checkContrast()
{
    return contrast.has_value() && checkContrast(contrast.value());
}

bool V4L2Cam::fetchContrast()
{
    int32_t value{};

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_CONTRAST
                                      , "V4L2_CID_CONTRAST"
                                      , value
                                      );
    
    if ( fetched ) { contrast       = (decltype(contrast)::value_type)(value);
                     contrastSource = ssrc::DEVICE;
                   }
    else           { contrast.reset();
                     contrastSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkContrast() ) [[unlikely]]
        clog << "V4L2Cam::fetchContrast: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return contrastSource == ssrc::DEVICE;
}

bool V4L2Cam::applyContrast()
{
    if ( contrastSource == ssrc::NONE || !checkContrast() )
        return false;
    
    int32_t value = contrast.value();
    
    if ( CONTRAST_DOMAIN_KNOWN && CONTRAST_STEP > 1 )
        value -= (value - CONTRAST_MIN) % CONTRAST_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_CONTRAST
                                      , "V4L2_CID_CONTRAST"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        contrast.reset();
        contrastSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyContrast: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchContrast();
}


/* *********** */
/* Saturation  */
/* *********** */

bool V4L2Cam::giveSaturation(int32_t& _saturation)
{
    if ( !checkSaturation() ) [[unlikely]]
        return false;
    
    _saturation = saturation.value();
    
    return true;
}

bool V4L2Cam::takeSaturation(int32_t const _saturation)
{
    if ( !checkSaturation(_saturation) ) [[unlikely]]
        return false;
    
    saturation       = _saturation;
    saturationSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkSaturation(int32_t const _saturation)
{
    if ( !SATURATION_DOMAIN_KNOWN ) [[unlikely]]
        return true;
    
    return    _saturation >= SATURATION_MIN
           && _saturation <= SATURATION_MAX;
}

bool V4L2Cam::checkSaturation()
{
    return saturation.has_value() && checkSaturation(saturation.value());
}

bool V4L2Cam::fetchSaturation()
{
    int32_t value{};

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_SATURATION
                                      , "V4L2_CID_SATURATION"
                                      , value
                                      );
    
    if ( fetched ) { saturation       = (decltype(saturation)::value_type)(value);
                     saturationSource = ssrc::DEVICE;
                   }
    else           { saturation.reset();
                     saturationSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkSaturation() ) [[unlikely]]
        clog << "V4L2Cam::fetchSaturation: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return saturationSource == ssrc::DEVICE;
}

bool V4L2Cam::applySaturation()
{
    if ( saturationSource == ssrc::NONE || !checkSaturation() )
        return false;
    
    int32_t value = saturation.value();
    
    if ( SATURATION_DOMAIN_KNOWN && SATURATION_STEP > 1 )
        value -= (value - SATURATION_MIN) % SATURATION_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_SATURATION
                                      , "V4L2_CID_SATURATION"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        saturation.reset();
        saturationSource = ssrc::NONE;
        
        clog << "V4L2Cam::applySaturation: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchSaturation();
}


/* ********** */
/* Sharpness  */
/* ********** */

bool V4L2Cam::giveSharpness(int32_t& _sharpness)
{
    if ( !checkSharpness() ) [[unlikely]]
        return false;
    
    _sharpness = sharpness.value();
    
    return true;
}

bool V4L2Cam::takeSharpness(int32_t const _sharpness)
{
    if ( !checkSharpness(_sharpness) ) [[unlikely]]
        return false;
    
    sharpness       = _sharpness;
    sharpnessSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkSharpness(int32_t const _sharpness)
{
    if ( !SHARPNESS_DOMAIN_KNOWN ) [[unlikely]]
        return true;
    
    return    _sharpness >= SHARPNESS_MIN
           && _sharpness <= SHARPNESS_MAX;
}

bool V4L2Cam::checkSharpness()
{
    return sharpness.has_value() && checkSharpness(sharpness.value());
}

bool V4L2Cam::fetchSharpness()
{
    int32_t value{};

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_SHARPNESS
                                      , "V4L2_CID_SHARPNESS"
                                      , value
                                      );
    
    if ( fetched ) { sharpness       = (decltype(sharpness)::value_type)(value);
                     sharpnessSource = ssrc::DEVICE;
                   }
    else           { sharpness.reset();
                     sharpnessSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkSharpness() ) [[unlikely]]
        clog << "V4L2Cam::fetchSharpness: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return sharpnessSource == ssrc::DEVICE;
}

bool V4L2Cam::applySharpness()
{
    if ( sharpnessSource == ssrc::NONE || !checkSharpness() )
        return false;
    
    int32_t value = sharpness.value();
    
    if ( SHARPNESS_DOMAIN_KNOWN && SHARPNESS_STEP > 1 )
        value -= (value - SHARPNESS_MIN) % SHARPNESS_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_SHARPNESS
                                      , "V4L2_CID_SHARPNESS"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        sharpness.reset();
        sharpnessSource = ssrc::NONE;
        
        clog << "V4L2Cam::applySharpness: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchSharpness();
}


/* ******* */
/* Gamma   */
/* ******* */

bool V4L2Cam::giveGamma(int32_t& _gamma)
{
    if ( !checkGamma() ) [[unlikely]]
        return false;
    
    _gamma = gamma.value();
    
    return true;
}

bool V4L2Cam::takeGamma(int32_t const _gamma)
{
    if ( !checkGamma(_gamma) ) [[unlikely]]
        return false;
    
    gamma       = _gamma;
    gammaSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkGamma(int32_t const _gamma)
{
    if ( !GAMMA_DOMAIN_KNOWN ) [[unlikely]]
        return true;
    
    return    _gamma >= GAMMA_MIN
           && _gamma <= GAMMA_MAX;
}

bool V4L2Cam::checkGamma()
{
    return gamma.has_value() && checkGamma(gamma.value());
}

bool V4L2Cam::fetchGamma()
{
    int32_t value{};

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_GAMMA
                                      , "V4L2_CID_GAMMA"
                                      , value
                                      );
    
    if ( fetched ) { gamma       = (decltype(gamma)::value_type)(value);
                     gammaSource = ssrc::DEVICE;
                   }
    else           { gamma.reset();
                     gammaSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkGamma() ) [[unlikely]]
        clog << "V4L2Cam::fetchGamma: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return gammaSource == ssrc::DEVICE;
}

bool V4L2Cam::applyGamma()
{
    if ( gammaSource == ssrc::NONE || !checkGamma() )
        return false;
    
    int32_t value = gamma.value();
    
    if ( GAMMA_DOMAIN_KNOWN && GAMMA_STEP > 1 )
        value -= (value - GAMMA_MIN) % GAMMA_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_GAMMA
                                      , "V4L2_CID_GAMMA"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        gamma.reset();
        gammaSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyGamma: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchGamma();
}


/* ************* */
/* White Balance */
/* ************* */

bool V4L2Cam::giveWhiteBalance(int32_t& _whiteBalance)
{
    if ( !checkWhiteBalance() ) [[unlikely]]
        return false;
    
    _whiteBalance = whiteBalance.value();
    
    return true;
}

bool V4L2Cam::takeWhiteBalance(int32_t const _whiteBalance)
{
    if ( !checkWhiteBalance(_whiteBalance) ) [[unlikely]]
        return false;
    
    whiteBalance       = _whiteBalance;
    whiteBalanceSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkWhiteBalance(int32_t const _whiteBalance)
{
    if ( !WHITE_BALANCE_DOMAIN_KNOWN ) [[unlikely]]
        return true;
    
    return    _whiteBalance >= WHITE_BALANCE_MIN
           && _whiteBalance <= WHITE_BALANCE_MAX;
}

bool V4L2Cam::checkWhiteBalance()
{
    return whiteBalance.has_value() && checkWhiteBalance(whiteBalance.value());
}

bool V4L2Cam::fetchWhiteBalance()
{
    int32_t value{};

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_WHITE_BALANCE_TEMPERATURE
                                      , "V4L2_CID_WHITE_BALANCE_TEMPERATURE"
                                      , value
                                      );
    
    if ( fetched ) { whiteBalance       = (decltype(whiteBalance)::value_type)(value);
                     whiteBalanceSource = ssrc::DEVICE;
                   }
    else           { whiteBalance.reset();
                     whiteBalanceSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkWhiteBalance() ) [[unlikely]]
        clog << "V4L2Cam::fetchWhiteBalance: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return whiteBalanceSource == ssrc::DEVICE;
}

bool V4L2Cam::applyWhiteBalance()
{
    if ( whiteBalanceSource == ssrc::NONE || !checkWhiteBalance() )
        return false;
    
    int32_t value = whiteBalance.value();
    
    if ( WHITE_BALANCE_DOMAIN_KNOWN && WHITE_BALANCE_STEP > 1 )
        value -= (value - WHITE_BALANCE_MIN) % WHITE_BALANCE_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_WHITE_BALANCE_TEMPERATURE
                                      , "V4L2_CID_WHITE_BALANCE_TEMPERATURE"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        whiteBalance.reset();
        whiteBalanceSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyWhiteBalance: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchWhiteBalance();
}


/* ***** */
/* Gain  */
/* ***** */

bool V4L2Cam::giveGain(int32_t& _gain)
{
    if ( !checkGain() ) [[unlikely]]
        return false;
    
    _gain = gain.value();
    
    return true;
}

bool V4L2Cam::takeGain(int32_t const _gain)
{
    if ( !checkGain(_gain) ) [[unlikely]]
        return false;
    
    gain       = _gain;
    gainSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkGain(int32_t const _gain)
{
    if ( !GAIN_DOMAIN_KNOWN ) [[unlikely]]
        return true;
    
    return    _gain >= GAIN_MIN
           && _gain <= GAIN_MAX;
}

bool V4L2Cam::checkGain()
{
    return gain.has_value() && checkGain(gain.value());
}

bool V4L2Cam::fetchGain()
{
    int32_t value{};

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_GAIN
                                      , "V4L2_CID_GAIN"
                                      , value
                                      );
    
    if ( fetched ) { gain       = (decltype(gain)::value_type)(value);
                     gainSource = ssrc::DEVICE;
                   }
    else           { gain.reset();
                     gainSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkGain() ) [[unlikely]]
        clog << "V4L2Cam::fetchGain: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return gainSource == ssrc::DEVICE;
}

bool V4L2Cam::applyGain()
{
    if ( gainSource == ssrc::NONE || !checkGain() )
        return false;
    
    int32_t value = gain.value();
    
    if ( GAIN_DOMAIN_KNOWN && GAIN_STEP > 1 )
        value -= (value - GAIN_MIN) % GAIN_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_GAIN
                                      , "V4L2_CID_GAIN"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        gain.reset();
        gainSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyGain: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchGain();
}


/* ****************** */
/* PowerLineFrequency */
/* ****************** */

bool V4L2Cam::givePowerLineFrequency(uint8_t& _powerLineFrequency)
{
    if ( !powerLineFrequency.has_value() ) [[unlikely]]
        return false;
    
    _powerLineFrequency = enum_integer(powerLineFrequency.value());
    
    return true;
}

bool V4L2Cam::takePowerLineFrequency(uint8_t const _powerLineFrequency)
{
    auto plf = enum_cast<PowerLineFrequency>(_powerLineFrequency);
    
    if ( !plf.has_value() ) [[unlikely]]
        return false;
    
    powerLineFrequency       = plf;
    powerLineFrequencySource = ssrc::USER;
    
    return true;
}

// no checkPowerLineFrequency() - using magic_enum suffices

bool V4L2Cam::fetchPowerLineFrequency()
{
    int32_t value{};
    
    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_POWER_LINE_FREQUENCY
                                      , "V4L2_CID_POWER_LINE_FREQUENCY"
                                      , value
                                      );

    if ( fetched ) [[  likely]]
        powerLineFrequency = enum_cast<PowerLineFrequency>(value);
    else
        powerLineFrequency.reset();
    
    if ( powerLineFrequency ) powerLineFrequencySource = ssrc::DEVICE;
    else                      powerLineFrequencySource = ssrc::NONE;
    
    if ( fetched && !powerLineFrequency ) [[unlikely]]
        clog << "V4L2Cam::fetchPowerLineFrequency: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return powerLineFrequencySource == ssrc::DEVICE;
}

bool V4L2Cam::applyPowerLineFrequency()
{
    if ( powerLineFrequencySource == ssrc::NONE || !powerLineFrequency.has_value() )
        return false;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_POWER_LINE_FREQUENCY
                                      , "V4L2_CID_POWER_LINE_FREQUENCY"
                                      , enum_integer(powerLineFrequency.value())
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        powerLineFrequency.reset();
        powerLineFrequencySource = ssrc::NONE;
        
        clog << "V4L2Cam::applyPowerLineFrequency: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchPowerLineFrequency();
}


/* ******** */
/* Exposure */
/* ******** */

bool V4L2Cam::giveExposure(int32_t& _exposure)
{
    if ( !checkExposure() ) [[unlikely]]
        return false;
    
    _exposure = exposure.value();
    
    return true;
}

bool V4L2Cam::takeExposure(int32_t const _exposure)
{
    if ( !checkExposure(_exposure) ) [[unlikely]]
        return false;
    
    exposure       = _exposure;
    exposureSource = ssrc::USER;
    
    return true;
}

bool V4L2Cam::checkExposure(int32_t const _exposure)
{
    if ( !EXPOSURE_DOMAIN_KNOWN ) [[unlikely]]
        return true;
    
    return    _exposure >= EXPOSURE_MIN
           && _exposure <= EXPOSURE_MAX;
}

bool V4L2Cam::checkExposure()
{
    return exposure.has_value() && checkExposure(exposure.value());
}

bool V4L2Cam::fetchExposure()
{
    int32_t value{};

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_EXPOSURE_ABSOLUTE
                                      , "V4L2_CID_EXPOSURE_ABSOLUTE"
                                      , value
                                      );
    
    if ( fetched ) { exposure       = (decltype(exposure)::value_type)(value);
                     exposureSource = ssrc::DEVICE;
                   }
    else           { exposure.reset();
                     exposureSource = ssrc::NONE;
                   }
    
    if ( fetched && !checkExposure() ) [[unlikely]]
        clog << "V4L2Cam::fetchExposure: value reported by device not valid "
                "according to our domain knowledge!"
             << endl;
    
    return exposureSource == ssrc::DEVICE;
}

bool V4L2Cam::applyExposure()
{
    if ( exposureSource == ssrc::NONE || !checkExposure() )
        return false;
    
    int32_t value = exposure.value();
    
    if ( EXPOSURE_DOMAIN_KNOWN && EXPOSURE_STEP > 1 )
        value -= (value - EXPOSURE_MIN) % EXPOSURE_STEP;
    
    bool applied = apply_control_value( produceV4L2FD()
                                      , V4L2_CID_EXPOSURE_ABSOLUTE
                                      , "V4L2_CID_EXPOSURE_ABSOLUTE"
                                      , value
                                      );
    
    if ( !applied ) [[unlikely]]
    {
        exposure.reset();
        exposureSource = ssrc::NONE;
        
        clog << "V4L2Cam::applyExposure: ioctl failed!"
             << endl;
        
        return false;
    }
    else
        return fetchExposure();
}


} // namespace FWR::Cam_lnx
