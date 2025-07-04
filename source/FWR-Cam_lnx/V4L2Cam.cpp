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

#include "FWR-Cam_lnx/V4L2Cam.hpp"

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <cstring>

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
// #include <stdbool.h>

#include <libudev.h>    // For udev functions and structures

#include <magic_enum/magic_enum.hpp>





namespace FWR::Cam_lnx
{


using namespace std;
using namespace magic_enum;


// struct buffer
// {
//     void *start;
//     size_t length;
// };


V4L2CamData::V4L2CamData(std::string const& sNo) noexcept
 :  serialNo(sNo)
 ,  evntFD(make_shared<FD_t>(eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)))
{
    if ( !evntFD ) [[unlikely]]
        clog << "V4L2CamData::V4L2CamData: Failed to create eventfd!!!"
             << endl;
}


void V4L2CamData::FD_t::close_fd()
{
    constexpr int32_t retry_count = 10;
    int32_t delay_ms = 1;
    
    int32_t result;
    
    for ( int32_t attempt = 0; attempt < retry_count; ++attempt )
    {
        errno = 0;
        
        result = ::close(value);
        
        if ( result == 0 )
            return;
        
        if ( errno == EINTR ) {
            if ( attempt < retry_count - 1 )
            {
                this_thread::sleep_for(chrono::milliseconds(delay_ms++));
                continue;
            } else
                return;
        } else
            return;
    }
}


bool V4L2Cam::goIntoInitializedState() noexcept
{
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
    
    if (     (    state == State::DEQUEUEING
               || state == State::STREAMING
             )
         && !tryAndStopStreaming(false)
       ) [[unlikely]]
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

bool V4L2Cam::locateDeviceNodeAndInitialize()
{
    if (    state != State::INITIALIZED
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
    if ( !enumerate )
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
    
    udev_device* dev{};
    udev_list_entry* dev_list_entry;
    udev_list_entry_foreach(dev_list_entry, devices)
    {
        if ( dev )
            udev_device_unref(dev);
        
        const char *path = udev_list_entry_get_name(dev_list_entry);
        if ( !path ) [[unlikely]]
            continue;
        
        dev = udev_device_new_from_syspath(uDev, path);
        if ( !dev )
            continue;
        
        udev_device* pdev = udev_device_get_parent_with_subsystem_devtype
                            (dev, "usb", "usb_device");
        if ( !pdev )
            continue;
        
        const char *vendor  = udev_device_get_sysattr_value(pdev, "idVendor");
        const char *product = udev_device_get_sysattr_value(pdev, "idProduct");
        if (                !vendor ||              !product
             || vendorID !=  vendor || productID !=  product
           )
            continue;
        
        product_found = true;
        
        const char *sn = udev_device_get_sysattr_value(pdev, "serial");
        if ( !sn || serialNo != sn )
            continue;
        
        serial_found = true;
        
        char const* dev_path = udev_device_get_devnode(dev);
        if ( !dev_path )
            continue;
        
        FD_t fd{xopen(dev_path, O_RDWR | O_NONBLOCK)};
        if ( !fd )
        {
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: Could not open "
                    "v4l2 device path! "
                 << strerror(errno) << endl;
            
            continue;
        }
        
        v4l2_capability cap{};
        if ( !xioctl(fd, VIDIOC_QUERYCAP, &cap) )
        {
            clog << "V4L2Cam::locateDeviceNodeAndInitialize: VIDIOC_QUERYCAP "
                    "failed! "
                 << strerror(errno) << endl;
            
            continue;
        }
        
        if ( !(   cap.capabilities
                & V4L2_CAP_VIDEO_CAPTURE )
           )
            continue;
        
        capture_found = true;
        
        if ( !(   cap.capabilities
                & V4L2_CAP_STREAMING )
           )
            continue;
        
        streaming_found = true;
        
        if ( _locateDeviceNodeAndInitialize(uDev, pdev) )
        {
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
            
            resetCrop(fd);
            
            v4l2FD = make_shared<FD_t>(move(fd));
            
            if ( state == State::UNINITIALIZED )
            {
                determineSettingDomains(fd);
                determineMaxBufferSizeNeeded(fd);
                
                initializeSettings();
                
                state = State::DEVICE_KNOWN;
            }
            else /* state == State::DEVICE_KNOWN */
                reapplySettings();
            
            state = State::INITIALIZED;
        }
        
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
    
    return state == State::INITIALIZED;
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
    
    
    v4l2_requestbuffers req{};
    req.count  = count;
    req.type   = bufType;
    req.memory = memType;
    
    if ( !xioctl(*fd_ptr, VIDIOC_REQBUFS, &req) ) [[unlikely]]
    {
        clog << "V4L2Cam::requestBufferQueue: Could not have the device prepare "
                "a buffer queue for " << count << " buffers! "
             << strerror(errno) << endl;
        
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
       ) [[unlikely]]
    {
        clog << "V4L2Cam::prepBuffer: not in a correct state "
                "(BUFFER_QUEUE_PREPPED|STREAMING) to prep a buffer descriptor!"
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
                "one queued for that index! (I'll try to queue it anyway. "
                "Wanna see what happens.)"
             << endl;
    
    
    if ( !xioctl(*fd_ptr, VIDIOC_QBUF, &buf) ) [[unlikely]]
    {
        clog << "V4L2Cam::queueBuffer: could not queue buffer! "
             << strerror(errno) << endl;
        
        return false;
    }
    
    buffersQueued.set(buf.index);
    
    return true;
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
        clog << "V4L2Cam::requestBufferQueue: could not decide v4l2 buffer type!"
             << endl;
        
        return false;
    }

    
    if ( !xioctl(*fd_ptr, VIDIOC_STREAMON, &bufType) )
    {
        clog << "V4L2Cam::startCapturing: couldn't start streaming! "
             << strerror(errno) << endl;
        
        return false;
    }
    
    state = State::STREAMING;
    
    return true;
}

bool V4L2Cam::fillBuffer(v4l2_buffer& buf) noexcept
{
    shared_ptr<FD_t> fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    
    if ( state != State::STREAMING ) [[unlikely]]
    {
        clog << "V4L2Cam::fillBuffer: not in the correct state "
                "(STREAMING) to try to fetch a frame!"
             << endl;
        
        return false;
    }
    
    
    state = State::DEQUEUEING;
    
    
    pollfd fds[2]{};
    fds[0].fd     = *fd_ptr;
    fds[0].events = POLLIN;
    
    if ( evntFD ) [[  likely]]
    {
        fds[1].fd     = *evntFD;
        fds[1].events = POLLIN;
    }
    
    
    while ( true )
    {
        int ret = poll(fds, evntFD ? 2 : 1, -1);
        
        if ( ret == -1 ) [[unlikely]]
        {
            if ( errno == EINTR ) [[  likely]]
                continue;
            
            clog << "V4L2Cam::fillBuffer: poll errored out! "
                 << strerror(errno) << endl;
            
            state = State::STREAMING;
            
            return false;
        }
        
        if (    evntFD
             && fds[1].revents & POLLIN
           ) [[unlikely]]
        {
            uint64_t dummy;
            dummy = read(*evntFD, &dummy, sizeof(dummy));
            
            clog << "V4L2Cam::fillBuffer: so. wrote to my eventfd to wake me!?"
                 << endl;
        }
        
        if ( !( fds[0].revents & POLLIN ) ) [[unlikely]]
        {
            state = State::STREAMING;
            
            return false;
        }
        else
            break;
    }
        
    
    buf = {};
    
    if ( !prepBuffer(buf) ) [[unlikely]]
        clog << "V4L2Cam::fillBuffer: sth. went wrong prepping the "
                "v4l2_buffer structure for the fetch. Will try anyways. Might "
                "work."
             << endl;
    
    bool succ = xioctl(*fd_ptr, VIDIOC_DQBUF, &buf);
    
    state = State::STREAMING;
    
    if ( !succ ) [[unlikely]]
    {
        clog << "V4L2Cam::fillBuffer: dequeueing a buffer with a framedidn't work! "
             << strerror(errno) << endl;
        
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
        clog << "V4L2Cam::wake: not in the correct state "
                "(STREAMING|DEQUEUEING) to wake the one in fillBuffer()!"
             << endl;
        
        return false;
    }
    
    
    uint64_t one = 1;
    auto bytesWritten = write(*evntFD, &one, sizeof(one));
    
    if ( bytesWritten != sizeof(one) ) [[unlikely]]
    {
        int err = errno;
        clog << "V4L2Cam::wake: write to eventfd failed with errno "
             << err << " (" << strerror(err) << ")"
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
    
    
    v4l2_buffer buf{};
    
    if ( !prepBuffer(buf) ) [[unlikely]]
    {
        clog << "V4L2Cam::stopStreaming: sth. went wrong prepping the "
                "v4l2_buffer structure for dequeueing whatever's left to dequeue!"
             << endl;
        
        // let's not return, but still try to dequeue
    }
    
    
    if ( !xioctl(*fd_ptr, VIDIOC_STREAMOFF, &buf.type) ) [[unlikely]]
    {
        clog << "V4L2Cam::stopStreaming: operation failed! "
             << strerror(errno) << endl;
        
        return false;
    }
    
    while ( xioctl(*fd_ptr, VIDIOC_DQBUF, &buf) )
    {
        if ( buf.index >= bufferCount ) [[unlikely]]
            clog << "V4L2Cam::stopStreaming: retrieved buffer has out-of-bounds "
                    "index!!!"
                 << endl;
        else
            buffersQueued.reset(buf.index);
        
        prepBuffer(buf);
    }
    
    
    state = State::BUFFER_QUEUE_PREPPED;
    
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
    
    if ( !xioctl( *fd_ptr
                , VIDIOC_REQBUFS
                , &req
                )
       ) [[unlikely]]
    {
        clog << "V4L2Cam::releaseBufferQueue: Could not un-prep buffer queue! "
             << strerror(errno) << endl;
        
        return false;
    }
    
    bufferCount = 0;
    buffersQueued.clear();
    state = State::INITIALIZED;
    
    return true;
}



// static bool init_userp(uint32_t buffer_size)
// {
//     v4l2_requestbuffers req{};
//     
//     req.count  = NUM_BUFFS;
//     req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//     req.memory = V4L2_MEMORY_USERPTR;
//     
//     if ( !xioctl(fd, VIDIOC_REQBUFS, &req) )
//     {
//         if ( errno == EINVAL )
//             fprintf( stderr
//             , "The device does not support user pointer i/o\n"
//             );
//         
//         return false;
//     }
//     
//     buffers = (struct buffer *)calloc(req.count, sizeof(*buffers));
//     
//     if ( !buffers )
//     {
//         clog <<  "Out of memory\n");
//         
//         return false;
//     }
//     
//     for ( n_buffers = 0; n_buffers < req.count; ++n_buffers )
//     {
//         buffers[n_buffers].length = buffer_size;
//         
//         if (    posix_memalign( &buffers[n_buffers].start
//             , getpagesize()
//             , buffer_size
//         )
//             != 0
//         )
//         {
//             // This happens only in case of ENOMEM
//             for ( int i = 0; i < n_buffers; i++ )
//                 free(buffers[i].start);
//             
//             free(buffers);
//             fprintf( stderr
//             , "Error occurred when allocating memory for buffers\n"
//             );
//             
//             return false;
//         }
//     }
//     
//     return true;
// }
// 
// bool V4L2Cam::helper_init_cam( const char *devname
//                              , uint32_t width
//                              , uint32_t height
//                              , uint32_t format
//                              )
// {
//     if ( initialized )
//     {
//         clog << "V4L2Cam::Device already initialized." << endl;
//         return false;
//     }
//     
//     fd = xopen(devname, O_RDWR /* required */ | O_NONBLOCK);
//     
//     if ( fd < 0 )
//     {
//         clog << "V4L2Cam::Error occurred when opening cam v4l2 device node" << endl;
//         return false;
//     }
//     
//     if (    init_userp(currentBufferSizeNeeded) < 0
//          || start_capturing() < 0
//        )
//     {
//         clog << "V4L2Cam::Error occurred when initialising camera" << endl;
//         return false;
//     }
//     
//     initialized = true;
//     return true;
// }
// 
// bool V4L2Cam::helper_deinit_cam()
// {
//     if ( !initialized )
//     {
//         clog <<  "Error: trying to de-initialise without initialising camera\n");
//         return false;
//     }
//     
//     /*
//      * It's better to turn off initialized even if the
//      * de-initialisation fails as it shouldn't have affect
//      * re-initialisation a lot.
//      */
//     initialized = false;
//     
//     if (    stop_capturing() < 0
//          || uninit_device()  < 0
//          || close_device()   < 0
//        )
//     {
//         clog <<  "Error occurred when de-initialising camera\n");
//         return false;
//     }
//     
//     return true;
// }
// 
// /**
//  * Untested for misusages
//  */
// 
// bool V4L2Cam::helper_change_cam_res( uint32_t width
// , uint32_t height
// // , uint32_t framerate // SB's addition
// , uint32_t format
// )
// {
//     if ( !initialized )
//     {
//         clog <<  "Error: trying to de-initialise without initialising camera\n");
//         return false;
//     }
//     
//     if (    stop_capturing() < 0
//         || uninit_device()  < 0
//     )
//     {
//         clog <<  "Error occurred when ude-initializing device to change camera resolution\n");
//         return false;
//     }
//     
//     initialized = 0;
//     
//     if (    set_io_method(io_meth)              < 0
//         || init_userp(currentBufferSizeNeeded) < 0
//         || start_capturing()                   < 0
//     )
//     {
//         clog <<  "Error occurred when changing camera resolution\n");
//         return false;
//     }
//     
//     initialized = true;
//     
//     return true;
// }
// 
// bool V4L2Cam::helper_get_cam_frame( unsigned char** pointer_to_cam_data
//                                   , int32_t* size
//                                   )
// {
//     static unsigned char max_timeout_retries = 2;
//     unsigned char timeout_retries = 0;
//     
//     if ( !initialized )
//     {
//         clog <<  "Error: trying to get frame without successfully initialising camera\n");
//         return false;
//     }
//     
//     if ( !is_released )
//     {
//         fprintf( stderr
//                , "Error: trying to get another frame without "
//                         "releasing already obtained frame\n"
//                );
//         return false;
//     }
//     
//     for ( size_t i = 0; i < 10; i++ )
//     //  (;;)
//     {
//         fd_set fds;
//         struct timeval tv;
//         int32_t r;
//         
//         FD_ZERO(&fds);
//         FD_SET(fd, &fds);
//         
//         /* Timeout. */
//         tv.tv_sec = 1;
//         tv.tv_usec = 0;
//         
//         memset(&frame_buf, 0, sizeof(frame_buf));
//         
//         frame_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//         
//         r = select(fd + 1, &fds, NULL, NULL, &tv);
//         // printf("r = %d\n", r);
//         
//         if ( r == -1 )
//         {
//             clog <<  "V4L2Cam::helper_get_cam_frame: select -1\n");
//             if ( errno == EINTR )
//                 continue;
//         }
//         
//         if ( r == 0 )
//         {
//             clog <<  "V4L2Cam::helper_get_cam_frame: select timeout\n");
//             return false;
//             // timeout_retries++;
// 
//             // if (timeout_retries == max_timeout_retries)
//             // {
//             // clog <<  "V4L2Cam::helper_get_cam_frame: Could not get frame after multiple retries\n");
//             // return false;
//             // }
//         }
//         
//         if ( !xioctl(fd, VIDIOC_DQBUF, &frame_buf) )
//         {
//             switch ( errno )
//             {
//             case EAGAIN:
//                 continue;
//             
//             case EIO:
//                 /* Could ignore EIO, see spec. */
// 
//                 [[fallthrough]]
// 
//             default:
//                 continue;
//             }
//         }
//         
//         *pointer_to_cam_data = (unsigned char *)buffers[frame_buf.index].start;
//         *size = frame_buf.bytesused;
//         break;
//         /* EAGAIN - continue select loop. */
//     }
//     
//     is_released = false;
//     return true;
// }
// 
// bool V4L2Cam::helper_release_cam_frame()
// {
//     if ( !initialized )
//     {
//         clog <<  "Error: trying to release frame without successfully initialising camera\n");
//         return false;
//     }
//     
//     if ( is_released )
//     {
//         clog <<  "Error: trying to release already released frame\n");
//         return false;
//     }
//     
//     if ( !xioctl(fd, VIDIOC_QBUF, &frame_buf) )
//     {
//         clog <<  "Error occurred when queueing frame for re-capture\n");
//         return false;
//     }
//     
//     /*
//      * We assume the frame hasn't been released if an error occurred as
//      * we couldn't queue the frame for streaming.
//      *
//      * Assuming it to be released in case an error occurs causes issues
//      * such as the loss of a buffer, etc.
//      */
//     is_released = true;
//     
//     return true;
// }


/**
 * Untested for misusages

bool V4L2Cam::helper_queryctrl( uint32_t id
                              , v4l2_queryctrl* qctrl
                              )
{
    if ( !initialized )
    {
        fprintf( stderr
               , "Error: trying to query control without initialising camera\n"
              );
        return false;
    }
    
    qctrl->id = id;
    if ( xioctl(fd, VIDIOC_QUERYCTRL, qctrl) == -1 ) {
        clog <<  "Error QUERYCTRL\n");
        return false;
    }
    
    return true;
}

*/


V4L2Cam::V4L2Cam(std::string const& sNo) noexcept
 :  V4L2CamData(sNo)
{}

V4L2Cam::~V4L2Cam() noexcept
{
    tryAndStopStreaming(true); // std::aborts, if it doesn't work out!
    
    if ( state == State::BUFFER_QUEUE_PREPPED ) [[unlikely]]
        releaseBufferQueue();
    
    uninitialize();
    
    evntFD->close_fd();
}

int32_t V4L2Cam::xioctl( FD_t const& fd
                       , uint64_t    request
                       , void*       arg
                       , bool        quasi_blocking
                       )
{
    constexpr uint8_t retry_count = 5;
    uint8_t busy_count{};
    
    int r;
    
    do {
        errno = 0;
        
        r = ioctl(fd, request, arg);
        
        if (    r == -1
             && errno == EBUSY
           )
        {
            if      ( busy_count == 0 )
                this_thread::yield();
            else if ( busy_count  < retry_count )
                this_thread::sleep_for(chrono::milliseconds(busy_count++));
            else {
                break;
            }
        }
    } while ( r == -1 && (      errno == EINTR
                           ||   errno == EBUSY
                           || ( errno == EAGAIN && quasi_blocking )
                         )
            );
    
    if ( r == -1 ) [[unlikely]]
    {
        // Handle specific errors and set action flags
        switch ( errno )
        {
            // Errors indicating that uninitializing the camera abstraction class is appropriate
            case ENODEV:    [[fallthrough]]; // No such device
            case ENXIO:     [[fallthrough]]; // No such device or address
            case EBADF:                     // Bad file descriptor
                clog << "V4L2Cam::xioctl: Error: Device is no longer available ("
                     << strerror(errno)
                     << "). Uninitializing camera class is indicated."
                     << endl;
                
                errorAction = ErrorAction::Uninitialize;
                break;
            
            // Errors indicating that resetting the camera abstraction class is appropriate
            case EBUSY:                      // Device or resource busy after retries
                if ( state == State::STREAMING )
                {
                    errorAction = ErrorAction::StopStreaming;
                    break;
                }
                else
                    [[fallthrough]];
            case ETIMEDOUT:                  // Connection timed out
                clog << "V4L2Cam::xioctl: Error: Device is busy or timed out ("
                     << strerror(errno)
                     << "). Resetting camera class is indicated."
                     << endl;
                
                errorAction = ErrorAction::ReopenDescriptors;
                break;
            
            // Errors indicating that power-cycling the USB camera is appropriate
            case EIO:       [[fallthrough]]; // Input/output error
            case EFAULT:                     // Bad address
                clog << "V4L2Cam::xioctl: Error: Hardware failure ("
                     << strerror(errno)
                     << "). Power-cycling the USB camera is indicated."
                     << endl;
                
                errorAction = ErrorAction::PowerCycle;
                break;
            
            // Errors indicating permission issues
            case EPERM:     [[fallthrough]]; // Operation not permitted
            case EACCES:                     // Permission denied
                clog << "V4L2Cam::xioctl: Error: Permission denied ("
                     << strerror(errno)
                     << "). Check permissions is indicated."
                     << endl;
                
                errorAction = ErrorAction::CheckPermissions;
                break;
            
            // Errors indicating programming errors or misconfigurations
            case EINVAL:    [[fallthrough]]; // Invalid argument
            case ENOTTY:    [[fallthrough]]; // Inappropriate ioctl for device
            case EOVERFLOW:                  // Value too large for defined data type
                clog << "V4L2Cam::xioctl: Error: Invalid request or argument ("
                     << strerror(errno)
                     << "). Check configuration is indicated."
                     << endl;
                
                errorAction = ErrorAction::CheckLogic;
                break;
            
            case ENOMEM:                     // Out of memory
                clog << "V4L2Cam::xioctl: Error: Out of memory ("
                     << strerror(errno) << ")."
                     << endl;
                
                errorAction = ErrorAction::FreeMemory;
                break;
            
            // not of import
            case EINTR:     [[fallthrough]]; // Interrupted system call
            case EAGAIN:                     // Resource temporarily unavailable
                break;
            
            default:
                clog << "V4L2Cam::xioctl: Error: ioctl failed with errno "
                     << errno << " (" << strerror(errno) << ")."
                     << endl;
                
                break;
        }
    }
    
    return r == 0;
}

int32_t V4L2Cam::xopen( char    const* pathname
                      , int32_t const  flags
                      )
{
    constexpr int32_t retry_count = 10;
    int32_t delay_ms = 1;
    
    int32_t fd{-1};
    
    for ( int32_t attempt = 0; attempt < retry_count; ++attempt ) {
        errno = 0;
        
        fd = open(pathname, flags);
        
        if ( fd >= 0 )
            break;
        
        if ( errno == EINTR || errno == EAGAIN || errno == EBUSY ) {
            if ( attempt < retry_count - 1 ) {
                this_thread::sleep_for(chrono::milliseconds(delay_ms *= 2));
                continue;
            } else {
                clog << "V4L2Cam::xopen: Failed to open character device after "
                     << (attempt + 1) << " attempts: " << strerror(errno)
                     << endl;
                
                break;
            }
        } else if ( errno == ENOENT || errno == ENODEV ) {
            clog << "V4L2Cam::xopen: character device not found: "
                 << strerror(errno)
                 << endl;
            
            break;
        } else {
            clog << "V4L2Cam::xopen: Failed to open character device: "
                 << strerror(errno)
                 << endl;
            
            break;
        }
    }
    
    return fd;
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
    applyResolutionAndPixelFormat();
    applyBrightness();
    applyContrast();
    applySaturation();
    applySharpness();
    applyGamma();
    applyWhiteBalance();
    applyGain();
    applyPowerLineFrequency();
    applyExposure();
}

// only to be used by locateDeviceNodeAndInitialize()
void V4L2Cam::resetCrop(FD_t const& fd)
{
    // if the cam can crop, reset crop
    v4l2_cropcap cropcap{};
    
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if ( xioctl(fd, VIDIOC_CROPCAP, &cropcap)  )
    {
        v4l2_crop crop{};
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        xioctl(fd, VIDIOC_S_CROP, &crop);
    }
}

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
    
    if ( !xioctl(*fd_ptr, VIDIOC_REQBUFS, &req) ) [[unlikely]]
    {
        clog << "V4L2Cam::isSetMemoryTypeSupported: device does not support set "
                "buffer type (v4l2_memory)! "
             << strerror(errno) << endl;
        
        return false;
    }
    
    return true;
}


bool V4L2Cam::determineMaxBufferSizeNeeded(FD_t const& fd) {
    // TODO also learn supported resolutions and supported pixel formats
    maxBufferSizeNeeded.reset();
    
    uint32_t maxSizeImage = 0;
    SUPPORTS__VIDIOC_TRY_FMT = true;
    
    uint32_t bufType{};
    
    if (    !decideBufferType(bufType)
         || !bufType
       ) [[unlikely]]
    {
        clog << "V4L2Cam::requestBufferQueue: could not decide v4l2 buffer type!"
             << endl;
        
        return false;
    }

    
    // Check if VIDIOC_TRY_FMT is supported
    v4l2_format originalFmt{};
    
    originalFmt.type = bufType;
    
    if ( !xioctl(fd, VIDIOC_G_FMT, &originalFmt) )
        return false;
    
    if ( !xioctl(fd, VIDIOC_TRY_FMT, &originalFmt) ) {
        if ( errno == ENOTTY || errno == EINVAL )
            SUPPORTS__VIDIOC_TRY_FMT = false;
        else
            return false;
    }
    
    // Structure to enumerate pixel formats
    v4l2_fmtdesc fmtDesc{};
    
    // TODO after this, do it again with the *_MPLANE type
    fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    // Enumerate all supported pixel formats
    while ( xioctl(fd, VIDIOC_ENUM_FMT, &fmtDesc) ) {
        uint32_t pixelFormat = fmtDesc.pixelformat;
        
        // Structure to enumerate frame sizes
        v4l2_frmsizeenum frmSize{};
        
        frmSize.pixel_format = pixelFormat;
        
        // Enumerate all frame sizes for the pixel format
        while ( xioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmSize) ) {
            v4l2_format fmt{};
            
            fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmt.fmt.pix.pixelformat = pixelFormat;
            fmt.fmt.pix.field       = V4L2_FIELD_ANY;
            
            switch ( frmSize.type ) {
                case V4L2_FRMSIZE_TYPE_DISCRETE:
                    fmt.fmt.pix.width  = frmSize.discrete.width;
                    fmt.fmt.pix.height = frmSize.discrete.height;
                    break;
                case V4L2_FRMSIZE_TYPE_STEPWISE:   [[fallthrough]];
                case V4L2_FRMSIZE_TYPE_CONTINUOUS:
                    fmt.fmt.pix.width  = frmSize.stepwise.max_width;
                    fmt.fmt.pix.height = frmSize.stepwise.max_height;
                    break;
                default:
                    clog << "V4L2Cam::determineMaxBufferSizeNeeded: Unknown "
                            "frame size type."
                         << endl;
                    
                    frmSize.index++;
                    continue;
            }
            
            // get sizeimage
            bool ret = SUPPORTS__VIDIOC_TRY_FMT
                     ? xioctl(fd, VIDIOC_TRY_FMT, &fmt)
                     : xioctl(fd, VIDIOC_S_FMT  , &fmt);
            
            if ( ret && fmt.fmt.pix.sizeimage > maxSizeImage )
                maxSizeImage = fmt.fmt.pix.sizeimage;
            
            frmSize.index++;
        }
        
        if ( errno != EINVAL && errno != 0 ) {
            clog << "V4L2Cam::determineMaxBufferSizeNeeded: v4l2 device frame "
                    "sizes enumeration errored out"
                 << endl;
            
            return false;
        }
        
        fmtDesc.index++;
    }
    
    if ( errno != EINVAL && errno != 0 ) {
        clog << "V4L2Cam::determineMaxBufferSizeNeeded: v4l2 device format "
                "enumeration errored out"
             << endl;
        
        return false;
    }
    
    if (    !SUPPORTS__VIDIOC_TRY_FMT
         &&  xioctl(fd, VIDIOC_S_FMT, &originalFmt) != 0
       )
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

void V4L2Cam::determineSettingDomains(FD_t const& fd) {
    queryControlDomain(fd, V4L2_CID_BRIGHTNESS, BRIGHTNESS_DOMAIN_KNOWN,
                       BRIGHTNESS_MIN, BRIGHTNESS_MAX, BRIGHTNESS_STEP);
    queryControlDomain(fd, V4L2_CID_CONTRAST, CONTRAST_DOMAIN_KNOWN,
                       CONTRAST_MIN, CONTRAST_MAX, CONTRAST_STEP);
    queryControlDomain(fd, V4L2_CID_SATURATION, SATURATION_DOMAIN_KNOWN,
                       SATURATION_MIN, SATURATION_MAX, SATURATION_STEP);
    queryControlDomain(fd, V4L2_CID_SHARPNESS, SHARPNESS_DOMAIN_KNOWN,
                       SHARPNESS_MIN, SHARPNESS_MAX, SHARPNESS_STEP);
    queryControlDomain(fd, V4L2_CID_GAMMA, GAMMA_DOMAIN_KNOWN,
                       GAMMA_MIN, GAMMA_MAX, GAMMA_STEP);
    queryControlDomain(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE, WHITE_BALANCE_DOMAIN_KNOWN,
                       WHITE_BALANCE_MIN, WHITE_BALANCE_MAX, WHITE_BALANCE_STEP);
    queryControlDomain(fd, V4L2_CID_GAIN, GAIN_DOMAIN_KNOWN,
                       GAIN_MIN, GAIN_MAX, GAIN_STEP);
    queryControlDomain(fd, V4L2_CID_EXPOSURE, EXPOSURE_DOMAIN_KNOWN,
                       EXPOSURE_MIN, EXPOSURE_MAX, EXPOSURE_STEP);
}

void V4L2Cam::queryControlDomain( FD_t     const& fd
                                , uint32_t        controlID
                                , bool          & domainKnown
                                , int32_t       & min
                                , int32_t       & max
                                , int32_t       & step
                                )
{
    struct v4l2_queryctrl queryctrl {};
    queryctrl.id = controlID;
    
    if ( xioctl(fd, VIDIOC_QUERYCTRL, &queryctrl) ) {
        if (!(queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
            domainKnown = true;
            min = queryctrl.minimum;
            max = queryctrl.maximum;
            step = queryctrl.step;
        }
    } else if (errno != EINVAL)
        clog << "V4L2Cam::queryControlDomain: Error querying control "
             << controlID << ": " << strerror(errno)
             << endl;
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

bool V4L2Cam::openV4L2FD() {
    if ( v4l2Path.empty() ) [[unlikely]]
        return false;
    if ( !v4l2FD || !*v4l2FD )
        v4l2FD = make_shared<FD_t>(xopen( v4l2Path.c_str()
                                        , O_RDWR | O_NONBLOCK
                                        )
                                  );
    
    return (bool)v4l2FD && (bool)*v4l2FD;
}

void V4L2Cam::closeV4L2FD() {
    if ( v4l2FD && !*v4l2FD )
        v4l2FD->close_fd();
    
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
        this_thread::yield();
        stopStreaming();
        
        if ( tries++ >= maxTries ) [[unlikely]]
        {
            clog << "V4L2Cam::tryAndStopStreaming: cannot stop streaming!"
                    
                 << endl;
            
            if ( hard )
            {
                clog << " KILLING THIS PROCESS!!!"
                     << endl;
                
                std::abort();
            }
            
            clog << endl;
            
            return false;
        }
    }
    
    return true;
}


void V4L2Cam::uninitialize()
{
    if (    state != State::UNINITIALIZED
         && state != State::DEVICE_KNOWN
         && state != State::INITIALIZED
       ) [[unlikely]]
    {
        clog << "V4L2Cam::uninitialize: not in a correct state (UNINITIALIZED|"
                "DEVICE_KNOWN|INITIALIZED) to fully uninitialize!"
             << endl;
        
        return;
    }
    
    
    _uninitialize();
    
    closeV4L2FD();
    v4l2Path.clear();
    USBKernelName.clear();
    USBBusNumber.clear();
    USBDeviceAddress.clear();
    
    state = state == State::UNINITIALIZED
          ? State::UNINITIALIZED
          : State::DEVICE_KNOWN;
    
    bufferCount       = 0;
    buffersQueued     = {};
    bufferPlanes      = {};
}

bool V4L2Cam::rebindUSBDevice() {
    if ( USBKernelName.empty() ) {
        clog << "V4L2Cam::rebindUSBDevice: USB kernel name is not set."
             << endl;
        
        return false;
    }

    // Unbind the device
    static constexpr const char* unbindPath{"/sys/bus/usb/drivers/usb/unbind"};
    ofstream unbindFile(unbindPath);
    if ( unbindFile.is_open() ) {
        unbindFile << USBKernelName;
        unbindFile.close();
        clog << "V4L2Cam::powerCycleDevice: Device " << USBKernelName <<
                " unbound."
             << endl;
    } else {
        clog << "V4L2Cam::rebindUSBDevice: Failed to open " << unbindPath
             << endl;
        
        return false;
    }

    // Wait for the device to unbind
    this_thread::sleep_for(chrono::seconds(1));

    // Rebind the device
    static constexpr const char* bindPath{"/sys/bus/usb/drivers/usb/bind"};
    ofstream bindFile(bindPath);
    if ( bindFile.is_open() ) {
        bindFile << USBKernelName;
        bindFile.close();
        
        clog << "V4L2Cam::rebindUSBDevice: Device " << USBKernelName <<
                " rebound."
             << endl;
    } else {
        clog << "V4L2Cam::rebindUSBDevice: Failed to open " << bindPath
             << endl;
        
        return false;
    }

    // Wait for the device to rebind
    this_thread::sleep_for(chrono::seconds(1));
    
    return true;
}

bool V4L2Cam::powerCycleDevice() {
    if ( USBBusNumber.empty() || USBDeviceAddress.empty() ) {
        clog << "V4L2Cam::powerCycleDevice: USB bus number or device address "
                "is not set."
             << endl;
        
        return false;
    }
    
    libusb_context*          ctx        = nullptr;
    libusb_device_handle*    hub_handle = nullptr;
    libusb_device_descriptor hub_desc{};
    uint8_t                  port{};
    
    int32_t res = 0;
    
    res = libusb_init(&ctx);
    if ( res < 0 ) {
        clog << "V4L2Cam::powerCycleDevice: Failed to initialize libusb: "
             << libusb_error_name(res)
             << endl;
        
        return false;
    }
    
    if ( !produceHubHandleAndPortNumber(ctx, hub_handle, hub_desc, port) ) {
        libusb_exit(ctx);
        
        return false;
    }
    
    // Determine if the hub is USB 2.0 or USB 3.0+
    bool is_usb2 = hub_desc.bcdUSB < 0x0300;
    
    // Send the control transfer to set port power
    uint8_t  bmRequestType = static_cast<uint8_t>(LIBUSB_REQUEST_TYPE_CLASS)
                           | static_cast<uint8_t>(LIBUSB_RECIPIENT_OTHER);
    uint8_t  bRequest      = LIBUSB_REQUEST_SET_FEATURE;
    uint16_t wValue        = is_usb2
                           ? 0x0008    // PORT_POWER
                           : 0x0005;   // PORT_LINK_STATE
    uint16_t wIndex = port;
    
    // Turn off the power
    if ( is_usb2 )
        bRequest = LIBUSB_REQUEST_CLEAR_FEATURE;
    else // USB 3.0+ hubs may not support standard per-port power switching
        wIndex = port | ( 0x03 << 8 );  // U3
    
    res = libusb_control_transfer( hub_handle
                                 , bmRequestType, bRequest
                                 , wValue, wIndex
                                 , NULL, 0
                                 , 1000
                                 );
    if ( res < 0 ) {
        clog << "V4L2Cam::powerCycleDevice: Failed to power off the device."
             << endl;
        
        libusb_close(hub_handle);
        libusb_exit(ctx);
        
        return false;
    }
    
    // Turn on the power
    if ( is_usb2 )
        bRequest = LIBUSB_REQUEST_SET_FEATURE;
    else // USB 3.0+ hubs may not support standard per-port power switching
        wIndex = port | ( 0x0 << 8 );   // U0
    
    static constexpr uint8_t max_attempts{10};
    uint8_t attempt{};
    auto sleep_base = chrono::seconds(1);
    auto backoff    = chrono::milliseconds(10);
    
    do {
        this_thread::sleep_for(sleep_base + backoff);
        res = libusb_control_transfer( hub_handle
                                     , bmRequestType, bRequest
                                     , wValue, wIndex
                                     , NULL, 0
                                     , 1000
                                     );
        attempt++;
        backoff *= 2;
    } while ( res < 0 && attempt < max_attempts );
    
    if ( res < 0 )
        clog << "V4L2Cam::powerCycleDevice: Failed to power on the device."
             << endl;
    else
        this_thread::sleep_for(chrono::seconds(1));
    
    libusb_close(hub_handle);
    libusb_exit(ctx);
    
    return res < 0;
}

bool V4L2Cam::produceHubHandleAndPortNumber( libusb_context* const     ctx
                                           , libusb_device_handle*&    hub_handle
                                           , libusb_device_descriptor& hub_desc
                                           , uint8_t&                  port
                                           )
{
    hub_handle = nullptr;
    
    bool found{false};
    int res = 0;
    libusb_device **dev_list = nullptr;
    ssize_t cnt;
    libusb_device_handle* handle = nullptr;
    
    uint8_t target_busnum = static_cast<uint8_t>(stoi(USBBusNumber));
    uint8_t target_devnum = static_cast<uint8_t>(stoi(USBDeviceAddress));
    
    cnt = libusb_get_device_list(ctx, &dev_list);
    if ( cnt < 0 ) {
        clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to get device "
                "list: " << libusb_error_name(static_cast<int>(cnt))
             << endl;
        
        return false;
    }
    
    for ( ssize_t i = 0; i < cnt; i++ ) {
        libusb_device* device = dev_list[i];
        
        if (    libusb_get_bus_number    (device) != target_busnum
             || libusb_get_device_address(device) != target_devnum
           )
            continue;
        
        res = libusb_open(device, &handle);
        if ( res < 0 ) {
            clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to open "
                    "device: " << libusb_error_name(res)
                 << endl;
            
            break;
        }
        
        // Get the parent device (the hub)
        libusb_device* parent = libusb_get_parent(device);
        if ( !parent ) {
            clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to get "
                    "parent device (hub)"
                 << endl;
            
            break;
        }
        
        res = libusb_open(parent, &hub_handle);
        if ( res < 0 ) {
            clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to open "
                    "hub device: " << libusb_error_name(res)
                 << endl;
            
            break;
        }
        
        res = libusb_get_device_descriptor(parent, &hub_desc);
        if ( res < 0 ) {
            clog << "V4L2Cam::produceHubHandleAndPortNumber: Failed to get hub "
                    "descriptor: " << libusb_error_name(res)
                 << endl;
                 
            libusb_close(hub_handle);
            hub_handle = nullptr;
            
            break;
        }
        
        port = libusb_get_port_number(device);
        found = true;
        break;
    }
    
    if ( handle )
        libusb_close(handle);
    
    libusb_free_device_list(dev_list, 1);
    
    if ( !found )
        clog << "V4L2Cam::produceHubHandleAndPortNumber: Device not found"
             << endl;
    
    return found;
}



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
                                 , int32_t&         value
                                 )
{
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_control ctrl{id, {}};
    
    if ( xioctl(*fd_ptr, VIDIOC_G_CTRL, &ctrl) != 0 )
        return false;
    
    value = ctrl.value;
    
    return true;
}

bool V4L2Cam::apply_control_value( shared_ptr<FD_t> fd_ptr
                                 , uint32_t         id
                                 , int32_t const    value
                                 )
{
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_control ctrl{id, value};
    
    return xioctl(*fd_ptr, VIDIOC_S_CTRL, &ctrl) == 0;
}

//  public:
optional<v4l2_format> V4L2Cam::giveV4L2Format()
{
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return {};
    
    v4l2_format format{};
    
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if ( xioctl(*fd_ptr, VIDIOC_G_FMT, &format))
    {
        apiToUse = APIToUse::MULTI;
        
        return format;
    }
    
    format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if ( xioctl(*fd_ptr, VIDIOC_G_FMT, &format))
    {
        apiToUse = APIToUse::SINGLE;
        
        return format;
    }
    
    apiToUse = APIToUse::UNKNOWN;
    
    clog << "V4L2Cam::giveV4L2Format: requesting format failed! "
         << strerror(errno) << endl;
    
    return {};
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
    resolutionSource = ssrc::GIVEN;
    
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


bool V4L2Cam::givePixelFormat(uint32_t & _pixelFormat)
{
    if ( !checkPixelFormat() ) [[unlikely]]
        return false;
    
    _pixelFormat = enum_integer(pixelFormat.value());
    
    return true;
}

bool V4L2Cam::takePixelFormat(uint32_t const _pixelFormat)
{
    if ( !checkPixelFormat(_pixelFormat) ) [[unlikely]]
        return false;
    
    pixelFormat       = static_cast<PixelFormat>(_pixelFormat);
    pixelFormatSource = ssrc::GIVEN;
    
    return true;
}

bool V4L2Cam::checkPixelFormat(uint32_t const _pixelFormat)
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
           && checkPixelFormat(static_cast<uint32_t>(pixelFormat.value()));
}


bool V4L2Cam::fetchResolutionAndPixelFormat()
{
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_format format{};
    
    resolutionSource  = ssrc::FETCHED;
    pixelFormatSource = ssrc::FETCHED;
    
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    
    if ( xioctl(*fd_ptr, VIDIOC_G_FMT, &format) )
    {
        apiToUse = APIToUse::MULTI;
        
        if ( checkResolution(format.fmt.pix_mp.width, format.fmt.pix_mp.height) ) {
            width  = static_cast<uint32_t>(format.fmt.pix_mp.width);
            height = static_cast<uint32_t>(format.fmt.pix_mp.height);
        } else {
            width .reset();
            height.reset();
        }
        
        if ( checkPixelFormat(format.fmt.pix_mp.pixelformat) )
            pixelFormat = static_cast<PixelFormat>(format.fmt.pix_mp.pixelformat);
        else
            pixelFormat.reset();
        
        if ( format.fmt.pix_mp.num_planes == 1 ) // currently, we support no more
            currentBufferSizeNeeded
             =  static_cast<uint32_t>(format.fmt.pix_mp.plane_fmt[0].sizeimage);
        
        return true;
    }
    
    format = {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if ( xioctl(*fd_ptr, VIDIOC_G_FMT, &format) )
    {
        apiToUse = APIToUse::SINGLE;
        
        if ( checkResolution(format.fmt.pix.width, format.fmt.pix.height) ) {
            width  = format.fmt.pix.width;
            height = format.fmt.pix.height;
        } else {
            width .reset();
            height.reset();
        }
        
        if ( checkPixelFormat(format.fmt.pix.pixelformat) )
            pixelFormat = static_cast<PixelFormat>(format.fmt.pix.pixelformat);
        else
            pixelFormat.reset();
        
        currentBufferSizeNeeded = format.fmt.pix.sizeimage;
        
        return true;
    }
    else
    {
        apiToUse = APIToUse::UNKNOWN;
        
        width      .reset();
        height     .reset();
        pixelFormat.reset();
        
        currentBufferSizeNeeded.reset();
        
        return false;
    }
}

bool V4L2Cam::applyResolutionAndPixelFormat()
{
    if ( !checkResolution() || !checkPixelFormat() )
        return false;
    
    auto fd_ptr = produceV4L2FD();
    
    if ( !fd_ptr || !*fd_ptr ) [[unlikely]]
        return false;
    
    v4l2_format format{};
    
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    
    if ( xioctl(*fd_ptr, VIDIOC_G_FMT, &format) )
    {
        apiToUse = APIToUse::MULTI;
        
        format.fmt.pix_mp.width       = width      .value();
        format.fmt.pix_mp.height      = height     .value();
        format.fmt.pix_mp.pixelformat = enum_integer(pixelFormat.value());
        // format.fmt.pix_mp.field = V4L2_FIELD_INTERLACED; // from e-con's example. strange
        format.fmt.pix_mp.field       = V4L2_FIELD_NONE;
    }
    else
    {
        format = {};
        
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        
        if ( xioctl(*fd_ptr, VIDIOC_G_FMT, &format) )
        {
            apiToUse = APIToUse::SINGLE;
            
            format.fmt.pix.width       = width      .value();
            format.fmt.pix.height      = height     .value();
            format.fmt.pix.pixelformat = enum_integer(pixelFormat.value());
            // format.fmt.pix.field = V4L2_FIELD_INTERLACED; // from e-con's example. strange
            format.fmt.pix.field       = V4L2_FIELD_NONE;
        }
        else
        {
            apiToUse = APIToUse::UNKNOWN;
            
            return false;
        }
    }
    
    
    if ( xioctl(*fd_ptr, VIDIOC_S_FMT, &format) ) {
        currentBufferSizeNeeded = format.fmt.pix.sizeimage;
        
        return true;
    } else {
        currentBufferSizeNeeded.reset();
        
        return false;
    }
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
    brightnessSource = ssrc::GIVEN;
    
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
    
    brightnessSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_BRIGHTNESS
                                      , value
                                      );
    
    brightness = fetched && checkBrightness(value)
               ? decltype(brightness)(value)
               : decltype(brightness)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applyBrightness()
{
    if ( !checkBrightness() )
        return false;
    
    int32_t value = brightness.value();
    
    if ( BRIGHTNESS_DOMAIN_KNOWN && BRIGHTNESS_STEP != 1 )
        value -= (value - BRIGHTNESS_MIN) % BRIGHTNESS_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_BRIGHTNESS
                              , value
                              );
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
    contrastSource = ssrc::GIVEN;
    
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
    
    contrastSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_CONTRAST
                                      , value
                                      );
    
    contrast = fetched && checkContrast(value)
             ? decltype(contrast)(value)
             : decltype(contrast)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applyContrast()
{
    if ( !checkContrast() )
        return false;
    
    int32_t value = contrast.value();
    
    if ( CONTRAST_DOMAIN_KNOWN && CONTRAST_STEP != 1 )
        value -= (value - CONTRAST_MIN) % CONTRAST_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_CONTRAST
                              , value
                              );
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
    saturationSource = ssrc::GIVEN;
    
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
    
    saturationSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_SATURATION
                                      , value
                                      );
    
    saturation = fetched && checkSaturation(value)
               ? decltype(saturation)(value)
               : decltype(saturation)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applySaturation()
{
    if ( !checkSaturation() )
        return false;
    
    int32_t value = saturation.value();
    
    if ( SATURATION_DOMAIN_KNOWN && SATURATION_STEP != 1 )
        value -= (value - SATURATION_MIN) % SATURATION_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_SATURATION
                              , value
                              );
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
    sharpnessSource = ssrc::GIVEN;
    
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
    
    sharpnessSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_SHARPNESS
                                      , value
                                      );
    
    sharpness = fetched && checkSharpness(value)
              ? decltype(sharpness)(value)
              : decltype(sharpness)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applySharpness()
{
    if ( !checkSharpness() )
        return false;
    
    int32_t value = sharpness.value();
    
    if ( SHARPNESS_DOMAIN_KNOWN && SHARPNESS_STEP != 1 )
        value -= (value - SHARPNESS_MIN) % SHARPNESS_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_SHARPNESS
                              , value
                              );
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
    gammaSource = ssrc::GIVEN;
    
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
    
    gammaSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_GAMMA
                                      , value
                                      );
    
    gamma = fetched && checkGamma(value)
          ? decltype(gamma)(value)
          : decltype(gamma)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applyGamma()
{
    if ( !checkGamma() )
        return false;
    
    int32_t value = gamma.value();
    
    if ( GAMMA_DOMAIN_KNOWN && GAMMA_STEP != 1 )
        value -= (value - GAMMA_MIN) % GAMMA_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_GAMMA
                              , value
                              );
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
    whiteBalanceSource = ssrc::GIVEN;
    
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
    
    whiteBalanceSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_WHITE_BALANCE_TEMPERATURE
                                      , value
                                      );
    
    whiteBalance = fetched && checkWhiteBalance(value)
                 ? decltype(whiteBalance)(value)
                 : decltype(whiteBalance)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applyWhiteBalance()
{
    if ( !checkWhiteBalance() )
        return false;
    
    int32_t value = whiteBalance.value();
    
    if ( WHITE_BALANCE_DOMAIN_KNOWN && WHITE_BALANCE_STEP != 1 )
        value -= (value - WHITE_BALANCE_MIN) % WHITE_BALANCE_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_WHITE_BALANCE_TEMPERATURE
                              , value
                              );
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
    gainSource = ssrc::GIVEN;
    
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
    
    gainSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_GAIN
                                      , value
                                      );
    
    gain = fetched && checkGain(value)
         ? decltype(gain)(value)
         : decltype(gain)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applyGain()
{
    if ( !checkGain() )
        return false;
    
    int32_t value = gain.value();
    
    if ( GAIN_DOMAIN_KNOWN && GAIN_STEP != 1 )
        value -= (value - GAIN_MIN) % GAIN_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_GAIN
                              , value
                              );
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
    powerLineFrequencySource = ssrc::GIVEN;
    
    return true;
}

// no checkPowerLineFrequency() - using magic_enum suffices

bool V4L2Cam::fetchPowerLineFrequency()
{
    int32_t value{};
    
    powerLineFrequencySource = ssrc::FETCHED;

    if ( fetch_control_value( produceV4L2FD()
                            , V4L2_CID_POWER_LINE_FREQUENCY
                            , value
                            )
       )
    {
        powerLineFrequency = enum_cast<PowerLineFrequency>(value);
        
        return true;
    } else {
        powerLineFrequency.reset();
        
        return false;
    }
}

bool V4L2Cam::applyPowerLineFrequency()
{
    if ( !powerLineFrequency.has_value() )
        return false;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_POWER_LINE_FREQUENCY
                              , enum_integer(powerLineFrequency.value())
                              );
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
    exposureSource = ssrc::GIVEN;
    
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
    
    exposureSource = ssrc::FETCHED;

    bool fetched = fetch_control_value( produceV4L2FD()
                                      , V4L2_CID_EXPOSURE
                                      , value
                                      );
    
    exposure = fetched && checkExposure(value)
             ? decltype(exposure)(value)
             : decltype(exposure)(nullopt);
    
    return fetched;
}

bool V4L2Cam::applyExposure()
{
    if ( !checkExposure() )
        return false;
    
    int32_t value = exposure.value();
    
    if ( EXPOSURE_DOMAIN_KNOWN && EXPOSURE_STEP != 1 )
        value -= (value - EXPOSURE_MIN) % EXPOSURE_STEP;
    
    return apply_control_value( produceV4L2FD()
                              , V4L2_CID_EXPOSURE
                              , value
                              );
}


} // namespace FWR::Cam_lnx
