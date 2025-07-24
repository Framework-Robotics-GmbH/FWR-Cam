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

#include <string>
#include <optional>
#include <vector>
#include <array>
#include <cstdint>    // For fixed-width integer types like uint8_t

#include "FWR-Cam_lnx/V4L2Cam.hpp"





struct udev;
struct udev_device;

constexpr size_t BUFFER_LENGTH = 65;


namespace FWR::Cam_lnx::Models
{


/* Functions split up into give*, take*, fetch*, and apply* to separate concerns
 * of
 * - mapping values between generic/external and device-specific/internal
 *   domains and value checking
 * - communication with the device
 * For every setting it gets tracked, wether the locally stored value comes from
 * the user (SettingSource::GIVEN) or from the device (SettingSource::FETCHED).
 * On successful take* the state gets set to GIVEN, on every fetch* it gets set
 * to FETCHED.
 * For local value storage `std::optional`s are used. fetch* will report true,
 * iff communication with the device succeeded - irrespective of the validity of
 * the retrieved value(s). give* will return false, iff the locally held values
 * are invalid.
 * apply* will also return false on invalid local value(s), since it won't
 * attempt device communication and thus the latter won't have succeeded.
 * 
 * TODO Define a camera base class defining all sorts of setting function sets
 *      as virtual functions, make these here virtual overrides. Type their
 *      arguments to generic value domains (where applicable).
 * TODO Move the file descriptor class from this here's .cpp to the generic base
 *      class and there define a shared_ptr<FD_t>. Keep the opening and closing
 *      of the fd on every request, but add support for leaving it open and
 *      closing it later.
 * TODO Initialization and setToDefault() do all `fetch*`s.
 * TODO In generic base class, define error enum for everything that can go
 *      wrong with value checks, device file accesses (make them specific). Make
 *      the setting functions then return such enumerators.
*/



class See3CAM_24CUG
 :  public V4L2Cam
{
public:
    See3CAM_24CUG(std::string const& serialNo) noexcept;
    ~See3CAM_24CUG();

    See3CAM_24CUG           (See3CAM_24CUG const& ) = delete;
    See3CAM_24CUG& operator=(See3CAM_24CUG const& ) = delete;
    See3CAM_24CUG           (See3CAM_24CUG      &&) = delete;
    See3CAM_24CUG& operator=(See3CAM_24CUG      &&) = delete;
    
    
    static std::string_view const& produceVendorID () noexcept;
    static std::string_view const& produceProductID() noexcept;
    
    static bool gatherSerialNumbers(std::vector<std::string>& serials) noexcept;
    
private:
    inline virtual std::string_view const& _produceVendorID () noexcept override
    {                                return produceVendorID ();                }
    inline virtual std::string_view const& _produceProductID() noexcept override
    {                                return produceProductID();                }
    
    virtual bool _locateDeviceNodeAndInitialize( udev       * uDev
                                               , udev_device* parentDev
                                               ) override;
    void initializeSettings();
    
    std::shared_ptr<FD_t> produceHIDFD();
    bool                     openHIDFD();
    void                    closeHIDFD();
    
    virtual void _uninitialize() override;
    
public:
    ssrc  tellEffectModeSource();
    bool  giveEffectMode(uint8_t &     effectMode);
    bool  takeEffectMode(uint8_t const effectMode);
    bool fetchEffectMode();
    bool applyEffectMode();
    
    ssrc  tellDeNoiseValueSource();
    bool  giveDeNoiseValue(uint8_t &     deNoiseVal);
    bool  takeDeNoiseValue(uint8_t const deNoiseVal);
    bool fetchDeNoiseValue();
    bool applyDeNoiseValue();
    
    ssrc  tellAutoExpoModeAndROISource();
    bool  giveAutoExpoModeAndROI( uint8_t  &     autoExpoMode );
    bool  giveAutoExpoModeAndROI( uint8_t  &     autoExpoMode
                                , uint16_t const frameWidth
                                , uint16_t const frameHeight
                                , uint16_t &     roiXCoord
                                , uint16_t &     roiYCoord
                                , uint8_t  &     roiSize
                                );
    bool  takeAutoExpoModeAndROI( uint8_t  const autoExpoMode );
    bool  takeAutoExpoModeAndROI( uint8_t  const autoExpoMode
                                , uint16_t const frameWidth
                                , uint16_t const frameHeight
                                , uint16_t const roiXCoord
                                , uint16_t const roiYCoord
                                , uint8_t  const roiSize
                                );
    bool fetchAutoExpoModeAndROI();
    bool applyAutoExpoModeAndROI();
    
    ssrc  tellExposureCompensationSource();
    bool  giveExposureCompensation(uint32_t &     exposureCompValue);
    bool  takeExposureCompensation(uint32_t const exposureCompValue);
    bool fetchExposureCompensation();
    bool applyExposureCompensation();
    
    ssrc  tellBurstLengthSource();
    bool  giveBurstLength(uint8_t &     burstLength);
    bool  takeBurstLength(uint8_t const burstLength);
    bool fetchBurstLength();
    bool applyBurstLength();
    
    ssrc  tellQFactorSource();
    bool  giveQFactor(uint8_t &     qFactor);
    bool  takeQFactor(uint8_t const qFactor);
    bool fetchQFactor();
    bool applyQFactor();
    
    ssrc  tellMirrorModeSource();
    bool  giveMirrorMode( bool &     horizontal
                        , bool &     vertical
                        );
    bool  takeMirrorMode( bool const horizontal
                        , bool const vertical
                        );
    bool fetchMirrorMode();
    bool applyMirrorMode();
    
    ssrc  tellFramerateSource();
    bool  giveFramerate(uint8_t &     framerate);
    bool  takeFramerate(uint8_t const framerate);
    bool fetchFramerate();
    bool applyFramerate();
    
    ssrc  tellFaceDetectModeSource();
    bool  giveFaceDetectMode( bool &     faceDetect
                            , bool &     embedData
                            , bool &     overlayRect
                            );
    bool  takeFaceDetectMode( bool const faceDetect
                            , bool const embedData
                            , bool const overlayRect
                            );
    bool fetchFaceDetectMode();
    bool applyFaceDetectMode();
    
    ssrc  tellSmileDetectModeSource();
    bool  giveSmileDetectMode( bool &     smileDetect
                             , bool &     embedData
                             );
    bool  takeSmileDetectMode( bool const smileDetect
                             , bool const embedData
                             );
    bool fetchSmileDetectMode();
    bool applySmileDetectMode();
    
    ssrc  tellFlickerDetectModeSource();
    bool  giveFlickerDetectMode(uint8_t &     flickerMode);
    bool  takeFlickerDetectMode(uint8_t const flickerMode);
    bool fetchFlickerDetectMode();
    bool applyFlickerDetectMode();
    
    ssrc  tellFlashModeSource();
    bool  giveFlashMode(uint8_t &     flashMode);
    bool  takeFlashMode(uint8_t const flashMode);
    bool fetchFlashMode();
    bool applyFlashMode();
    
    ssrc  tellStreamModeSource();
    bool  giveStreamMode( uint8_t &     streamMode
                        , bool    &     autoFunctionLock
                        );
    bool  takeStreamMode( uint8_t const streamMode
                        , bool    const autoFunctionLock
                        );
    bool fetchStreamMode();
    bool applyStreamMode();
    
    bool setToDefault();
    
private:
    virtual bool _checkResolution(uint32_t const, uint32_t const) override;
    virtual bool _checkPixelFormat(uint32_t const) override;
    
    // in the .cpp, those checkers are between their respective take* and fetch*
    bool checkDeNoiseValue();
    bool checkAutoExpoModeAndROI();
    bool checkExposureCompensation();
    bool checkQFactor();
    bool checkFramerate();
    bool checkFaceDetectMode();
    bool checkSmileDetectMode();
    bool checkStreamMode();
    
    void initializeBuffers();
    bool sendHidCmd(uint8_t* outBuf, uint8_t* inBuf, uint32_t len);
    
    
    static constexpr std::array<std::array<uint32_t, 2>, 3> RESOLUTIONS = {{
        { {1280,  720} },
        { {1920, 1080} },
        { {1920, 1200} }
    }};
    
    enum class SupportedPixelFormat : uint32_t {
        // YUYV   = ('Y' | ('U' << 8) | ('Y' << 16) | ('V' << 24)), // 'YUYV'
        UYVY   = ('U' | ('Y' << 8) | ('V' << 16) | ('Y' << 24)), // 'UYVY'
        MJPEG  = ('M' | ('J' << 8) | ('P' << 16) | ('G' << 24)), // 'MJPG'
    };
    
    
    enum class EffectMode : uint8_t
    {
        NORMAL      = 0x01,
        BLACK_WHITE = 0x04,
        GREYSCALE   = 0x07,
        NEGATIVE    = 0x08,
        SKETCH      = 0x10
    };
    
    static constexpr uint8_t DENOISEVALUE_MIN{ 0};
    static constexpr uint8_t DENOISEVALUE_MAX{15};
    
    enum class AutoExpoMode : uint8_t
    {
        FACE     = 0x00,
        FULL     = 0x01,
        MANUAL   = 0x02,
        DISABLED = 0x03
    };
    
    static constexpr uint32_t EXPOSURECOMP_MIN{       50};
    static constexpr uint32_t EXPOSURECOMP_MAX{100000000};
    
    static constexpr uint8_t Q_FACTOR_MIN{10};
    static constexpr uint8_t Q_FACTOR_MAX{96};
    
    enum class MirrorMode : uint8_t
    {
        NONE = 0b00,
        HORZ = 0b01,
        VERT = 0b11,
        BOTH = 0b11
    };
    
    static constexpr uint8_t FRAMERATE_MIN{  1};
    static constexpr uint8_t FRAMERATE_MAX{120};
    
    enum class FaceRectMode : uint8_t
    {
        ENABLE  = 0x01,
        DISABLE = 0x00
    };
    
    enum class FaceEmbedMode : uint8_t
    {
        ENABLE  = 0x01,
        DISABLE = 0x00
    };
    
    enum class FaceOverlayRectMode : uint8_t
    {
        ENABLE  = 0x01,
        DISABLE = 0x00
    };
    
    enum class SmileDetectMode : uint8_t
    {
        ENABLE  = 0x01,
        DISABLE = 0x00
    };
    
    enum class SmileEmbedMode : uint8_t
    {
        ENABLE  = 0x01,
        DISABLE = 0x00
    };
    
    enum class FlickerDetectMode : uint8_t
    {
        AUTO    = 0x00,
        Hz50    = 0x01,
        Hz60    = 0x02,
        DISABLE = 0x03
    };
    
    enum class FlashMode : uint8_t
    {
        OFF    = 0x00,
        STROBE = 0x01
    };
    
    enum class StreamMode : uint8_t
    {
        MASTER  = 0x00,
        TRIGGER = 0x01
    };
    
    unsigned char g_out_packet_buf[BUFFER_LENGTH];
    unsigned char  g_in_packet_buf[BUFFER_LENGTH];
    
    // data members to forget on uninitialize
    bool initialized{false};
    
    std::string           hidPath{};
    std::shared_ptr<FD_t> hidFD{};
    
    SettingSource                      effectModeSource{};
    std::optional<EffectMode         > effectMode{};
    
    SettingSource                      deNoiseValueSource{};
    std::optional<uint8_t            > deNoiseValue{};
    
    SettingSource                      autoExpoModeAndROISource{};
    std::optional<AutoExpoMode       > autoExpoMode{};
    std::optional<uint16_t           > autoExpoROIxCoord{};
    std::optional<uint16_t           > autoExpoROIyCoord{};
    std::optional<uint8_t            > autoExpoROISize{};
    
    SettingSource                      exposureCompensationSource{};
    std::optional<uint32_t           > exposureCompensation{};
    
    SettingSource                      burstLengthSource{};
    std::optional<uint8_t            > burstLength{};
    
    SettingSource                      qFactorSource{};
    std::optional<uint8_t            > qFactor{};
    
    SettingSource                      mirrorModeSource{};
    std::optional<MirrorMode         > mirrorMode{};
    
    SettingSource                      framerateSource{};
    std::optional<uint8_t            > framerate{};
    
    SettingSource                      faceDetectModeSource{};
    std::optional<FaceRectMode       > faceRectMode{};
    std::optional<FaceEmbedMode      > faceEmbedMode{};
    std::optional<FaceOverlayRectMode> faceOverlayRectMode{};
    
    SettingSource                      smileDetectModeSource{};
    std::optional<SmileDetectMode    > smileDetectMode{};
    std::optional<SmileEmbedMode     > smileEmbedMode{};
    
    SettingSource                      flickerDetectModeSource{};
    std::optional<FlickerDetectMode  > flickerDetectMode{};
    
    SettingSource                      flashModeSource{};
    std::optional<FlashMode          > flashMode{};
    
    SettingSource                      streamModeSource{};
    std::optional<StreamMode         > streamMode{};
    std::optional<bool               > streamModeFunctionLock{};
};


} // namespace FWR::Cam_lnx::Models
