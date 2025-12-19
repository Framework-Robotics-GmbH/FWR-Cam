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

#pragma once

#include <array>
#include <vector>
#include <string>
#include <string_view>
#include <filesystem>
#include <optional>
#include <memory>
#include <utility>
#include <cstdint>    // For fixed-width integer types like uint8_t
#include <sys/eventfd.h>
#include <linux/videodev2.h>

// #include <libusb-1.0/libusb.h>

#include "FWR-Cam_lnx/util/SmallBitset.hpp"





struct udev;
struct udev_device;
struct libusb_context;
struct libusb_device_handle;


namespace FWR::Cam_lnx
{


enum class ErrorAction : uint8_t { None
                                 , StopStreaming
                                 , Reinitialize
                                 , ResetDevice
                                 , USBPowerCycle // when higher-ups must do it
                                 , FreeMemory
                                 , CheckLogic
                                 , CheckPermissions
                                 , ForgetDevice
                                 };

std::string_view to_string_view(ErrorAction ea) noexcept;

// to store it whithin resetDevice before uninitialize(), in case we don't manage
// to reinitialize, so user code can still retrieve it
struct USBID
{
    std::string           kernelName{};
    std::string           busNumber{};
    std::string           deviceAddress{};
    std::filesystem::path portDisablePathSelf{};
    std::filesystem::path portDisablePathPeer{};
};



// For now just documentation-ish
enum class PixelFormat : uint32_t
{
    INVALID = 0, // well, INVALID
    RGB332  = ('R' | ('G' << 8) | ('B' << 16) | ('1' << 24)), // 'RGB1' in little-endian
    RGB565  = ('R' | ('G' << 8) | ('B' << 16) | ('P' << 24)), // 'RGBP'
    RGB24   = ('R' | ('G' << 8) | ('B' << 16) | ('3' << 24)), // 'RGB3'
    RGB32   = ('R' | ('G' << 8) | ('B' << 16) | ('4' << 24)), // 'RGB4'
    GREY    = ('G' | ('R' << 8) | ('E' << 16) | ('Y' << 24)), // 'GREY'
    YUYV    = ('Y' | ('U' << 8) | ('Y' << 16) | ('V' << 24)), // 'YUYV'
    UYVY    = ('U' | ('Y' << 8) | ('V' << 16) | ('Y' << 24)), // 'UYVY'
    MJPEG   = ('M' | ('J' << 8) | ('P' << 16) | ('G' << 24)), // 'MJPG'
    H264    = ('H' | ('2' << 8) | ('6' << 16) | ('4' << 24)), // 'H264'
    NV12    = ('N' | ('V' << 8) | ('1' << 16) | ('2' << 24)), // 'NV12'
    NV21    = ('N' | ('V' << 8) | ('2' << 16) | ('1' << 24)), // 'NV21'
    YUV420  = ('Y' | ('U' << 8) | ('1' << 16) | ('2' << 24)), // 'YU12'
    YVU420  = ('Y' | ('V' << 8) | ('1' << 16) | ('2' << 24))  // 'YV12'
};

std::string_view to_string_view(PixelFormat pf) noexcept;
uint32_t         to_integer    (PixelFormat pf) noexcept;
PixelFormat      to_PixelFormat(uint32_t    ui) noexcept;


enum class PowerLineFrequency : uint8_t
{
    DISABLED  = 0, // Disable anti-flicker adjustment
    FREQ_50HZ = 1, // Anti-flicker adjustment for 50 Hz
    FREQ_60HZ = 2, // Anti-flicker adjustment for 60 Hz
    AUTO      = 3  // Automatically detect and adjust
};



struct V4L2CamData
{
    static constexpr uint8_t  MAX_BUFFERS        = 8;
    static constexpr uint8_t  MAX_PLANES         = 4;
    static constexpr uint16_t FRAME_POLL_TIMEOUT = 3000; // ms
    
    // the `EXPECT_*`s mean, treat them as failures still, but not as errors
    enum class XIOCTL_FLAGS : uint8_t { NONE           = 0
                                      , QUASI_BLOCKING = 1 << 0
                                      , EXPECT_EINVAL  = 1 << 1
                                      , EXPECT_EDOM    = 1 << 2
                                      , EXPECT_ENOTTY  = 1 << 3
                                      };
    
    
    enum class SettingSource: uint8_t { NONE
                                      , USER
                                      , DEVICE
                                      };
    enum class State        : uint8_t { UNINITIALIZED
                                      , DEVICE_KNOWN
                                      , INITIALIZED
                                      , BUFFER_QUEUE_PREPPED
                                      , STREAMING
                                      , DEQUEUEING
                                      };
    enum class ResetMeasure : uint8_t { NONE
                                      , USB_IFACE_REBIND
                                      , USB_DEVFS_RESET
                                      // , USB_PORT_RESET
                                      , USB_PORT_POWER_CYCLE_REQUESTED
                                      , USB_PORT_POWER_CYCLE
                                      };
    enum class MemoryType   : uint8_t { UNKNOWN
                                      , MMAP    // NIY
                                      , USERPTR
                                      , DMABUF
                                      };
    enum class APIToUse     : uint8_t { UNKNOWN
                                      , MULTI
                                      , SINGLE
                                      };
    
    // file descriptor helper struct
    struct FD_t
    {
        FD_t() = default;
        explicit FD_t(int32_t fd) : value(fd) {}
        
        FD_t(FD_t&& other) noexcept : value(std::exchange(other.value, -1)) {}
        FD_t& operator=(FD_t&& other) noexcept
        {
            if ( this != &other )
            {
                close_fd();
                value = std::exchange(other.value, -1);
            }
            return *this;
        }
        
        FD_t(const FD_t&)            = delete;
        FD_t& operator=(const FD_t&) = delete;
        
        ~FD_t() { close_fd(); }
        
        void close_fd();
        
        operator int32_t() const { return value; }
        operator bool()    const { return value > -1; }
        
        FD_t& operator=(int32_t const new_fd)
        {
            if ( value != new_fd )
            {
                close_fd();
                value = new_fd;
            }
            return *this;
        }
        
        auto operator<=>(FD_t    const&  o) const { return value <=> o.value; }
        auto operator<=>(int32_t const  fd) const { return value <=> fd;      }
        
        private:
            int32_t value{-1};
    };
    
    V4L2CamData(std::string const& serialNo) noexcept;
    
    
    //       //
    // state //
    //       //
    
    std::string const serialNo;
    
    // data members to forget on uninitialize - begin
    std::string              v4l2Path{};
    std::string              USBKernelName{};
    std::string              USBBusNumber{};
    std::string              USBDeviceAddress{};
    std::filesystem::path    USBPortDisablePathSelf{};
    std::filesystem::path    USBPortDisablePathPeer{};
    std::shared_ptr<FD_t>    v4l2FD{};
    std::shared_ptr<FD_t>    evntFD{};
    
    State                    state{State::UNINITIALIZED};
    ResetMeasure             lastResetMeasure{ResetMeasure::NONE};
    
    uint8_t                  bufferCount{};
    SmallBitset<MAX_BUFFERS> buffersQueued{};
    // data members to forget on uninitialize - end
    
    MemoryType               memoryType{MemoryType::UNKNOWN};
    APIToUse                 apiToUse{APIToUse::UNKNOWN};
    
    // lends it to user, when prepping user's v4l2_buffer
    std::array< v4l2_plane
              , MAX_PLANES > bufferPlanes{};
    
    // Error action flag and counters for severity promotion
    ErrorAction errorAction{ErrorAction::None};
    uint8_t     reopenDescriptorsCount{};
    uint8_t     reinitializationCount{};
    
    USBID       storedUSBID{};
    
    
    //                           //
    // settings domain knowledge //
    //                           //
    
    bool SUPPORTS__VIDIOC_TRY_FMT{};
    
    bool    RESOLUTION_DOMAIN_KNOWN{};
    bool    PIXEL_FORMAT_DOMAIN_KNOWN{};
    
    bool    BRIGHTNESS_DOMAIN_KNOWN{};
    int32_t BRIGHTNESS_MIN{};
    int32_t BRIGHTNESS_MAX{};
    int32_t BRIGHTNESS_STEP{};
    bool    CONTRAST_DOMAIN_KNOWN{};
    int32_t CONTRAST_MIN{};
    int32_t CONTRAST_MAX{};
    int32_t CONTRAST_STEP{};
    bool    SATURATION_DOMAIN_KNOWN{};
    int32_t SATURATION_MIN{};
    int32_t SATURATION_MAX{};
    int32_t SATURATION_STEP{};
    bool    SHARPNESS_DOMAIN_KNOWN{};
    int32_t SHARPNESS_MIN{};
    int32_t SHARPNESS_MAX{};
    int32_t SHARPNESS_STEP{};
    bool    GAMMA_DOMAIN_KNOWN{};
    int32_t GAMMA_MIN{};
    int32_t GAMMA_MAX{};
    int32_t GAMMA_STEP{};
    bool    WHITE_BALANCE_DOMAIN_KNOWN{};
    int32_t WHITE_BALANCE_MIN{};
    int32_t WHITE_BALANCE_MAX{};
    int32_t WHITE_BALANCE_STEP{};
    bool    GAIN_DOMAIN_KNOWN{};
    int32_t GAIN_MIN{};
    int32_t GAIN_MAX{};
    int32_t GAIN_STEP{};
    bool    EXPOSURE_DOMAIN_KNOWN{};
    int32_t EXPOSURE_MIN{};
    int32_t EXPOSURE_MAX{};
    int32_t EXPOSURE_STEP{};
    
    
    //                 //
    // settings valeus //
    //                 //
    
    std::optional<uint32_t>     maxBufferSizeNeeded{};
    std::optional<uint32_t> currentBufferSizeNeeded{};
    
    SettingSource                     resolutionSource{};
    std::optional<uint32_t>           width {};
    std::optional<uint32_t>           height{};
    
    SettingSource                     pixelFormatSource{};
    std::optional<PixelFormat>        pixelFormat{};
    
    
    SettingSource                     brightnessSource{};
    std::optional<int32_t>            brightness{};
    
    SettingSource                     contrastSource{};
    std::optional<int32_t>            contrast{};
    
    SettingSource                     saturationSource{};
    std::optional<int32_t>            saturation{};
    
    SettingSource                     sharpnessSource{};
    std::optional<int32_t>            sharpness{};
    
    SettingSource                     gammaSource{};
    std::optional<int32_t>            gamma{};
    
    SettingSource                     whiteBalanceSource{};
    std::optional<int32_t>            whiteBalance{};
    
    SettingSource                     gainSource{};
    std::optional<int32_t>            gain{};
    
    SettingSource                     powerLineFrequencySource{};
    std::optional<PowerLineFrequency> powerLineFrequency{};
    
    SettingSource                     exposureSource{};
    std::optional<int32_t>            exposure{};
    
}; // struct V4L2CamData




class V4L2Cam
 :  protected V4L2CamData
{
public:
    static bool hubCanPowerCyclePerPort;
    
    // in this class there should be no data members other than the inherited
    
protected:
    using ssrc = SettingSource;
    
public:
    virtual ~V4L2Cam();
    
    
    inline std::string const& produceSerialNo() const noexcept { return serialNo; }
    inline bool               isUninitialized() const noexcept { return state ==
                                                                        State::UNINITIALIZED;
                                                               }
    
    // mind the declaration order of the following set of functions *hint* *hint* *wink*
    
    inline
    bool isJustInitialized() const noexcept { return state == State::INITIALIZED; }
    bool goIntoInitializedState() noexcept;
    void goIntoUninitializedState() noexcept;
    
    bool locateDeviceNodeAndInitialize();
    static bool splitUsbSysname( std::string_view const  cur
                               , std::string           & parent
                               , int                   & port
                               );
    void findDisableFSNode( std::filesystem::path sysBusUsbDevices
                          , std::string const&    usbSysname
                          );
    
    bool usingMemoryType(MemoryType) noexcept;
    
    bool requestBufferQueue(uint32_t count) noexcept;
    
    bool produceUnqueuedMask(decltype(V4L2CamData::buffersQueued)&) const noexcept;
    bool  prepBuffer(v4l2_buffer&) noexcept; // nulls, then sets .type and .memory
                                             // and .m.planes as necessary
    bool queueBuffer(v4l2_buffer&) noexcept; // you're expected to have set the
                                             // index correctly, too
    
    void logSetup(std::ostream& out) noexcept;
    
    bool startStreaming() noexcept;
    bool    isStreaming() noexcept { return    state == State::STREAMING
                                            || state == State::DEQUEUEING; }
    bool  fillBuffer   (v4l2_buffer&) noexcept; // nulls and preps the arg, then blocks for a frame
    bool           wake()             noexcept; // to interrupt fillBuffer
    bool  stopStreaming() noexcept;
    
    bool releaseBufferQueue() noexcept;
    
    inline
    void resetTrivialErrorAction() noexcept
    {
        if ( errorAction == ErrorAction::StopStreaming )
             errorAction =  ErrorAction::None;
    }
    bool          reinitialize() noexcept; // for realizing ErrorAction::Reinitialize
    bool          resetDevice () noexcept; // for realizing ErrorAction::ResetDevice
    void powerCyclingConducted() noexcept; // after the deed was done by higher-ups,
                                           // to adjust inner state
    inline
    USBID produceUSBID() noexcept { return storedUSBID; }
    
    
    inline // for when you got false for requestBufferQueue
    size_t produceActualQueueSize() const noexcept {return bufferCount; }
    
    ErrorAction produceErrorAction() const { return errorAction; }
    
    
/**
 * Untested for misusages
    
    bool helper_queryctrl( uint32_t id
                         , v4l2_queryctrl* qctrl
                         )
    
*/

protected:
    V4L2Cam(std::string const& serialNo) noexcept;

    V4L2Cam           (V4L2Cam const& ) = delete;
    V4L2Cam& operator=(V4L2Cam const& ) = delete;
    V4L2Cam           (V4L2Cam      &&) = delete;
    V4L2Cam& operator=(V4L2Cam      &&) = delete;
    
    
    static bool gatherSerialNumbers( std::string_view           vendorID
                                   , std::string_view          productID
                                   , std::vector<std::string>& serials
                                   ) noexcept;
    

    bool xioctl( FD_t const&            fd
               , uint64_t               request
               , std::string_view const requestStr
               , void*                  arg
               , XIOCTL_FLAGS callFlags = XIOCTL_FLAGS::NONE
               );
    static int32_t xopen( char    const* pathname
                        , int32_t const  flags
                        );
    static bool xwrite( int         fd
                      , char const* str
                      , size_t      len
                      );
    
private:
    virtual std::string_view const& _produceVendorID () noexcept = 0;
    virtual std::string_view const& _produceProductID() noexcept = 0;
    
    virtual bool _locateDeviceNodeAndInitialize( udev       * uDev
                                               , udev_device* parentDev
                                               ) = 0;
    
    virtual void _logSetup(std::ostream& out) noexcept = 0;
    
    // only to be used by locateDeviceNodeAndInitialize()
    void initializeSettings();
    void    reapplySettings();
    
    // void resetCrop(FD_t const&);
    bool isSetMemoryTypeSupported() noexcept;
    bool determineMaxBufferSizeNeeded(FD_t const&);
    void determineSettingDomains(FD_t const& fd);
    void queryControlDomain( FD_t             const& fd
                           , uint32_t                controlID
                           , std::string_view const  cidStr
                           , bool                  & domainKnown
                           , int32_t               & min
                           , int32_t               & max
                           , int32_t               & step
                           );
    
    std::shared_ptr<FD_t> produceV4L2FD();
    bool                     openV4L2FD(); // locateDevice...() has that effect, too
    void                    closeV4L2FD();
    
    bool tryAndStopStreaming(bool hard = false) noexcept;
    
    void        uninitialize(bool hard = false) noexcept;
    bool     rebindUSBDevice() noexcept;
    bool      resetUSBDevice() noexcept;
    bool powerCycleUSBDevice() noexcept;
    // bool resetAtUSBHubPort() noexcept;
    // bool produceHubHandleAndPortNumber( std::string const&      usbBusNumber
    //                                   , std::string const&      usbDeviceAddress
    //                                   , libusb_context *        ctx
    //                                   , libusb_device_handle *& hub_handle
    //                                   , uint8_t&                port
    //                                   ) noexcept;
    
    virtual void _uninitialize() = 0;
    
    
    bool decideBufferType(uint32_t& bufType) noexcept;
    bool decideMemoryType(uint32_t& memType) noexcept;
    
    bool superObjectCannotExist{true};
    
    
    // settings helpers //
    
    bool fetch_control_value( std::shared_ptr<FD_t>  fd_ptr
                            , uint32_t               id
                            , std::string_view const idStr
                            , int32_t&               value
                            );
    bool apply_control_value( std::shared_ptr<FD_t> fd_ptr
                            , uint32_t              id
                            , std::string_view const idStr
                            , int32_t const         value
                            );
    
    
    //                 //
    // settings values //
    //                 //
    
    // to/from device (not just cache)
    virtual bool                   _requestFramerate(uint8_t fps) noexcept = 0;
    virtual std::optional<uint8_t> _produceFramerate()            noexcept = 0;
    
public:
    std::optional<v4l2_format> giveV4L2Format();
    
    bool                   requestFramerate(uint8_t fps) noexcept;
    std::optional<uint8_t> produceFramerate()            noexcept;
    
    ssrc  tellResolutionSource() { return resolutionSource; }
    bool  giveResolution(uint32_t &     width, uint32_t &     height);
    bool  takeResolution(uint32_t const width, uint32_t const height);
    
    ssrc  tellPixelFormatSource() { return pixelFormatSource; }
    bool  givePixelFormat(PixelFormat &     pixelFormat);
    bool  takePixelFormat(PixelFormat const pixelFormat);
    
    bool fetchResolutionAndPixelFormat();
    bool applyResolutionAndPixelFormat();
    private:
    void updateResNPixFmtFromDeviceInfo(v4l2_format const&) noexcept;
    void  resetResNPixFmt              ()                   noexcept;
    public:
    
    bool  giveMaxBufferSizeNeeded    (uint32_t & mbs);
    bool  giveCurrentBufferSizeNeeded(uint32_t & cbs);
    
    ssrc  tellBrightnessSource() { return brightnessSource; }
    bool  giveBrightness(int32_t &     brightness);
    bool  takeBrightness(int32_t const brightness);
    bool fetchBrightness();
    bool applyBrightness();
    
    ssrc  tellContrastSource() { return contrastSource; }
    bool  giveContrast(int32_t &     contrast);
    bool  takeContrast(int32_t const contrast);
    bool fetchContrast();
    bool applyContrast();
    
    ssrc  tellSaturationSource() { return saturationSource; }
    bool  giveSaturation(int32_t &     saturation);
    bool  takeSaturation(int32_t const saturation);
    bool fetchSaturation();
    bool applySaturation();
    
    ssrc  tellSharpnessSource() { return sharpnessSource; }
    bool  giveSharpness(int32_t &     sharpness);
    bool  takeSharpness(int32_t const sharpness);
    bool fetchSharpness();
    bool applySharpness();
    
    ssrc  tellGammaSource() { return gammaSource; }
    bool  giveGamma(int32_t &     gamma);
    bool  takeGamma(int32_t const gamma);
    bool fetchGamma();
    bool applyGamma();
    
    ssrc  tellWhiteBalanceSource() { return whiteBalanceSource; }
    bool  giveWhiteBalance(int32_t &     whiteBalance);
    bool  takeWhiteBalance(int32_t const whiteBalance);
    bool fetchWhiteBalance();
    bool applyWhiteBalance();
    
    ssrc  tellGainSource() { return gainSource; }
    bool  giveGain(int32_t &     gain);
    bool  takeGain(int32_t const gain);
    bool fetchGain();
    bool applyGain();
    
    ssrc  tellPowerLineFrequencySource() { return powerLineFrequencySource; }
    bool  givePowerLineFrequency(uint8_t &     powerLineFrequency);
    bool  takePowerLineFrequency(uint8_t const powerLineFrequency);
    bool fetchPowerLineFrequency();
    bool applyPowerLineFrequency();
    
    ssrc  tellExposureSource() { return exposureSource; }
    bool  giveExposure(int32_t &     exposure);
    bool  takeExposure(int32_t const exposure);
    bool fetchExposure();
    bool applyExposure();
    
private:
    // in the .cpp, those checkers are between their respective take* and fetch*
    bool checkResolution(uint32_t const width, uint32_t const height);
    bool checkResolution();
    virtual bool _checkResolution(uint32_t const, uint32_t const) { return true; }
    
    bool checkPixelFormat(PixelFormat const pixelFormat);
    bool checkPixelFormat();
    virtual bool _checkPixelFormat(PixelFormat const) { return true; }
    
    bool checkBrightness(int32_t const brightness);
    bool checkBrightness();
    bool checkContrast(int32_t const contrast);
    bool checkContrast();
    bool checkSaturation(int32_t const saturation);
    bool checkSaturation();
    bool checkSharpness(int32_t const sharpness);
    bool checkSharpness();
    bool checkGamma(int32_t const gamma);
    bool checkGamma();
    bool checkWhiteBalance(int32_t const whiteBalance);
    bool checkWhiteBalance();
    bool checkGain(int32_t const gain);
    bool checkGain();
    bool checkExposure(int32_t const exposure);
    bool checkExposure();
    
}; // class V4L2Cam


template<class M>
concept V4L2CamModel_c
 =      std::derived_from <M, V4L2Cam>
    && !std::same_as      <M, V4L2Cam>
    && !std::is_abstract_v<M>;


} // namespace FWR::Cam_lnx
