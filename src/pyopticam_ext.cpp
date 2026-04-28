#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/list.h>
#include <cameralibrary.h>

#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdlib>

using namespace CameraLibrary;
namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(pyopticam_ext, m) {
    m.doc() = "Python bindings for the OptiTrack Camera SDK (CameraLibrary).";

    m.def("GetCameraList", [](CameraManager& manager) {
        CameraList cameras;
        manager.GetCameraList(cameras);
        return cameras;
    }, "manager"_a);

    m.def("GetFrameGroupObjectArray", [](cModuleSync* sync) {
        nb::gil_scoped_release release;

        // Layout: [camera_index, object_index, {X, Y, Radius}]
        float* data = new float[8 * 255 * 3]();
        size_t shape[3] = { 8, 255, 3 };
        nb::capsule owner(data, [](void* p) noexcept { delete[] (float*)p; });
        auto ndarray = nb::ndarray<nb::numpy, float>(data, 3, shape, owner);

        std::shared_ptr<const FrameGroup> fg = sync->GetFrameGroup();
        while (!fg || fg->Count() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            fg = sync->GetFrameGroup();
        }

        for (int i = 0; i < fg->Count(); i++) {
            auto frame = fg->GetFrame(i);
            if (frame->IsInvalid()) continue;
            int n = frame->ObjectCount();
            for (int j = 0; j < n; j++) {
                data[(i * 255 * 3) + (j * 3) + 0] = frame->Object(j)->X();
                data[(i * 255 * 3) + (j * 3) + 1] = frame->Object(j)->Y();
                data[(i * 255 * 3) + (j * 3) + 2] = frame->Object(j)->Radius();
            }
        }

        nb::gil_scoped_acquire acquire;
        return ndarray;
    });

    m.def("GetSlowFrameArray", [](nb::ndarray<int32_t, nb::ndim<2>, nb::c_contig, nb::device::cpu> serials) {
        nb::gil_scoped_release release;

        auto view = serials.view();
        int count = (int)view.shape(0);

        uint8_t* stand_in = new uint8_t[8]();
        size_t stand_in_shape[3] = { (size_t)count, 1, 1 };
        nb::capsule stand_in_owner(stand_in, [](void* p) noexcept { delete[] (uint8_t*)p; });
        auto ndarray = nb::ndarray<nb::numpy, uint8_t>(stand_in, 3, stand_in_shape, stand_in_owner);

        CameraManager* mgr = &CameraManager::X();
        uint8_t* buffer = nullptr;
        int height = 0, width = 0;
        size_t offset = 0;

        for (int i = 0; i < count; i++) {
            auto camera = mgr->GetCameraBySerial(view(i, 0));
            if (!camera->IsCameraRunning()) camera->Start();

            // LatestFrame() does not throw on failure; it returns null silently.
            std::shared_ptr<const Frame> frame = camera->LatestFrame();
            while (!frame) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                frame = camera->LatestFrame();
            }

            if (frame->IsInvalid()) {
                printf("[WARNING] Empty or invalid frame from camera %i\n", camera->Serial());
                continue;
            }

            int size = frame->GrayscaleDataSize();
            if (offset == 0) {
                stand_in_owner.release();
                height = frame->Height();
                width = frame->Width();

                buffer = (uint8_t*)malloc(size * (count + 1));
                while (!buffer) {
                    printf("[ERROR] Failed to allocate frame buffer, retrying...\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    buffer = (uint8_t*)malloc(size * (count + 1));
                }

                size_t shape[3] = { (size_t)count, (size_t)height, (size_t)width };
                nb::capsule owner(buffer, [](void* p) noexcept { free(p); });
                ndarray = nb::ndarray<nb::numpy, uint8_t>(buffer, 3, shape, owner);
            }

            if (size > width * height) {
                printf("[WARNING] Frame size mismatch: size=%i w=%i h=%i\n", size, width, height);
                break;
            }
            memcpy(buffer + offset, frame->GrayscaleData(*camera), size);
            offset += size;
        }

        if (offset == 0)
            printf("[WARNING] No valid frames retrieved\n");

        nb::gil_scoped_acquire acquire;
        return ndarray;
    });

    m.def("GetFrameArrayNoSync", [](nb::list cameras) {
        nb::gil_scoped_release release;

        int count = (int)cameras.size();

        uint8_t* stand_in = new uint8_t[8]();
        size_t stand_in_shape[3] = { (size_t)count, 1, 1 };
        nb::capsule stand_in_owner(stand_in, [](void* p) noexcept { delete[] (uint8_t*)p; });
        auto ndarray = nb::ndarray<nb::numpy, uint8_t>(stand_in, 3, stand_in_shape, stand_in_owner);

        nb::gil_scoped_acquire acquire_tmp;
        std::vector<Camera*> cam_ptrs;
        cam_ptrs.reserve(count);
        for (int i = 0; i < count; i++)
            cam_ptrs.push_back(nb::cast<Camera*>(cameras[i]));
        nb::gil_scoped_release release2;

        uint8_t* buffer = nullptr;
        int height = 0, width = 0;
        size_t offset = 0;

        for (int i = 0; i < count; i++) {
            Camera* cam = cam_ptrs[i];
            auto frame = cam->LatestFrame();
            if (!frame || frame->IsInvalid()) {
                printf("[WARNING] Camera %i: null or invalid frame\n", i);
                continue;
            }

            int size = frame->GrayscaleDataSize();
            if (offset == 0) {
                stand_in_owner.release();
                height = frame->Height();
                width = frame->Width();
                buffer = (uint8_t*)malloc(size * count);
                if (!buffer) {
                    printf("[ERROR] Failed to allocate frame buffer\n");
                    break;
                }
                memset(buffer, 0, size * count);
                size_t shape[3] = { (size_t)count, (size_t)height, (size_t)width };
                nb::capsule owner(buffer, [](void* p) noexcept { free(p); });
                ndarray = nb::ndarray<nb::numpy, uint8_t>(buffer, 3, shape, owner);
            }

            if (size <= width * height) {
                memcpy(buffer + (i * size), frame->GrayscaleData(*cam), size);
                offset += size;
            } else {
                printf("[WARNING] Camera %i: frame size mismatch\n", i);
            }
        }

        nb::gil_scoped_acquire acquire;
        return ndarray;
    });

    m.def("GetFrameGroup", [](cModuleSync* sync, int timeout_ms) -> std::shared_ptr<const FrameGroup> {
        nb::gil_scoped_release release;
        auto fg = sync->GetFrameGroup();
        for (int elapsed = 0; (!fg || fg->Count() == 0) && elapsed < timeout_ms; ++elapsed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            fg = sync->GetFrameGroup();
        }
        nb::gil_scoped_acquire acquire;
        return fg;
    }, "sync"_a, "timeout_ms"_a = 100);

    m.def("FillTensorFromFrameGroup", [](std::shared_ptr<Camera> camera,
                                          std::shared_ptr<const FrameGroup> fg,
                                          nb::ndarray<nb::numpy> ndarray) {
        nb::gil_scoped_release release;

        if (!fg || fg->Count() == 0) {
            printf("[WARNING] FrameGroup is null or empty\n");
            nb::gil_scoped_acquire acquire;
            return;
        }

        uint8_t* buffer = (uint8_t*)ndarray.data();
        size_t offset = 0;
        int height = 0, width = 0;

        for (int i = 0; i < fg->Count(); i++) {
            auto frame = fg->GetFrame(i);
            if (frame->IsInvalid()) {
                printf("[WARNING] Subframe %i is invalid\n", i);
                continue;
            }
            int size = frame->GrayscaleDataSize();
            if (offset == 0) {
                height = frame->Height();
                width = frame->Width();
            }
            if (size > width * height || (int)ndarray.shape(1) < height || (int)ndarray.shape(2) < width) {
                printf("[WARNING] Shape mismatch at frame %i: size=%i w=%i h=%i\n", i, size, width, height);
                break;
            }
            memcpy(buffer + offset, frame->GrayscaleData(*camera), size);
            offset += size;
        }

        if (offset == 0)
            printf("[WARNING] No valid frames copied\n");

        nb::gil_scoped_acquire acquire;
    });

    // -------------------------------------------------------------------------
    // Enums
    // -------------------------------------------------------------------------

    nb::enum_<Core::eVideoMode>(m, "eVideoMode", "Camera video output mode.")
        .value("SegmentMode",                Core::eVideoMode::SegmentMode)
        .value("GrayscaleMode",              Core::eVideoMode::GrayscaleMode)
        .value("ObjectMode",                 Core::eVideoMode::ObjectMode)
        .value("InterleavedGrayscaleMode",   Core::eVideoMode::InterleavedGrayscaleMode)
        .value("PrecisionMode",              Core::eVideoMode::PrecisionMode)
        .value("BitPackedPrecisionMode",     Core::eVideoMode::BitPackedPrecisionMode)
        .value("MJPEGMode",                  Core::eVideoMode::MJPEGMode)
        .value("VideoMode",                  Core::eVideoMode::VideoMode)
        .value("SynchronizationTelemetry",   Core::eVideoMode::SynchronizationTelemetry)
        .value("UnknownMode",                Core::eVideoMode::UnknownMode);

    nb::enum_<CameraLibrary::eImagerGain>(m, "eImagerGain", "Imager gain level.")
        .value("Gain_Level0", CameraLibrary::eImagerGain::Gain_Level0)
        .value("Gain_Level1", CameraLibrary::eImagerGain::Gain_Level1)
        .value("Gain_Level2", CameraLibrary::eImagerGain::Gain_Level2)
        .value("Gain_Level3", CameraLibrary::eImagerGain::Gain_Level3)
        .value("Gain_Level4", CameraLibrary::eImagerGain::Gain_Level4)
        .value("Gain_Level5", CameraLibrary::eImagerGain::Gain_Level5)
        .value("Gain_Level6", CameraLibrary::eImagerGain::Gain_Level6)
        .value("Gain_Level7", CameraLibrary::eImagerGain::Gain_Level7);

    nb::enum_<CameraLibrary::eCameraState>(m, "eCameraState", "Camera initialization state.")
        .value("Uninitialized",                   CameraLibrary::eCameraState::Uninitialized)
        .value("InitializingDevice",              CameraLibrary::eCameraState::InitializingDevice)
        .value("InitializingCamera",              CameraLibrary::eCameraState::InitializingCamera)
        .value("Initializing",                    CameraLibrary::eCameraState::Initializing)
        .value("WaitingForChildDevices",          CameraLibrary::eCameraState::WaitingForChildDevices)
        .value("WaitingForDeviceInitialization",  CameraLibrary::eCameraState::WaitingForDeviceInitialization)
        .value("Initialized",                     CameraLibrary::eCameraState::Initialized)
        .value("Disconnected",                    CameraLibrary::eCameraState::Disconnected)
        .value("Shutdown",                        CameraLibrary::eCameraState::Shutdown);

    nb::enum_<CameraLibrary::eStatusLEDs>(m, "eStatusLEDs", "Camera status LED selector.")
        .value("GreenStatusLED",  CameraLibrary::eStatusLEDs::GreenStatusLED)
        .value("RedStatusLED",    CameraLibrary::eStatusLEDs::RedStatusLED)
        .value("CaseStatusLED",   CameraLibrary::eStatusLEDs::CaseStatusLED)
        .value("IlluminationLED", CameraLibrary::eStatusLEDs::IlluminationLED);

    nb::enum_<CameraLibrary::cModuleSync::eOptimization>(m, "eOptimization")
        .value("ForceTimelyDelivery",   CameraLibrary::cModuleSync::eOptimization::ForceTimelyDelivery)
        .value("FavorTimelyDelivery",   CameraLibrary::cModuleSync::eOptimization::FavorTimelyDelivery)
        .value("ForceCompleteDelivery", CameraLibrary::cModuleSync::eOptimization::ForceCompleteDelivery)
        .value("eOptimizationCount",    CameraLibrary::cModuleSync::eOptimization::eOptimizationCount);

    nb::enum_<CameraLibrary::FrameGroup::Modes>(m, "Modes")
        .value("None",     CameraLibrary::FrameGroup::Modes::None)
        .value("Software", CameraLibrary::FrameGroup::Modes::Software)
        .value("Hardware", CameraLibrary::FrameGroup::Modes::Hardware);

    // -------------------------------------------------------------------------
    // Simple data types
    // -------------------------------------------------------------------------

    nb::class_<sStatusLightColor>(m, "sStatusLightColor")
        .def(nb::init())
        .def_rw("Red",   &sStatusLightColor::Red)
        .def_rw("Green", &sStatusLightColor::Green)
        .def_rw("Blue",  &sStatusLightColor::Blue);

    nb::class_<Core::cUID>(m, "cUID")
        .def(nb::init())
        .def("SetValue",             &Core::cUID::SetValue)
        .def("LowBits",              &Core::cUID::LowBits)
        .def("HighBits",             &Core::cUID::HighBits)
        .def("Valid",                &Core::cUID::Valid)
        .def_static("Generate",      Core::cUID::Generate)
        .def("__lt__",               &Core::cUID::operator<)
        .def("__le__",               &Core::cUID::operator<=)
        .def("__gt__",               &Core::cUID::operator>)
        .def("__ge__",               &Core::cUID::operator>=)
        .def("__eq__",               &Core::cUID::operator==)
        .def("__ne__",               &Core::cUID::operator!=);

    // -------------------------------------------------------------------------
    // Camera lists
    // -------------------------------------------------------------------------

    nb::class_<CameraEntry>(m, "CameraEntry")
        .def("UID",          &CameraEntry::UID)
        .def("Serial",       &CameraEntry::Serial)
        .def("Revision",     &CameraEntry::Revision)
        .def("Name",         &CameraEntry::Name)
        .def("State",        &CameraEntry::State)
        .def("IsVirtual",    &CameraEntry::IsVirtual)
        .def("SerialString", &CameraEntry::SerialString);

    nb::class_<CameraList>(m, "CameraList", "Snapshot list of connected cameras.")
        .def(nb::init())
        .def("get",     &CameraList::operator[], "index"_a)
        .def("Count",   &CameraList::Count)
        .def("Refresh", &CameraList::Refresh);

    nb::class_<HardwareKeyList>(m, "HardwareKeyList")
        .def(nb::init())
        .def("get",   &HardwareKeyList::operator[])
        .def("Count", &HardwareKeyList::Count);

    nb::class_<HubList>(m, "HubList")
        .def(nb::init())
        .def("get",   &HubList::operator[])
        .def("Count", &HubList::Count);

    nb::class_<HardwareDeviceList>(m, "HardwareDeviceList")
        .def(nb::init())
        .def("get",   &HardwareDeviceList::operator[])
        .def("Count", &HardwareDeviceList::Count);

    // -------------------------------------------------------------------------
    // Frame / FrameGroup
    // -------------------------------------------------------------------------

    nb::class_<Frame>(m, "Frame")
        .def("ObjectCount",            &Frame::ObjectCount)
        .def("FrameID",                &Frame::FrameID)
        .def("FrameType",              &Frame::FrameType)
        .def("MJPEGQuality",           &Frame::MJPEGQuality)
        .def("Object",                 &Frame::Object)
        .def("IsInvalid",              &Frame::IsInvalid)
        .def("IsEmpty",                &Frame::IsEmpty)
        .def("IsGrayscale",            &Frame::IsGrayscale)
        .def("Width",                  &Frame::Width)
        .def("Height",                 &Frame::Height)
        .def("Left",                   &Frame::Left)
        .def("Top",                    &Frame::Top)
        .def("Right",                  &Frame::Right)
        .def("Bottom",                 &Frame::Bottom)
        .def("Scale",                  &Frame::Scale)
        .def("TimeStamp",              &Frame::TimeStamp)
        .def("IsSynchInfoValid",       &Frame::IsSynchInfoValid)
        .def("IsExternalLocked",       &Frame::IsExternalLocked)
        .def("IsRecording",            &Frame::IsRecording)
        .def("HardwareTimeStampValue", &Frame::HardwareTimeStampValue)
        .def("HardwareTimeStamp",      &Frame::HardwareTimeStamp)
        .def("IsHardwareTimeStamp",    &Frame::IsHardwareTimeStamp)
        .def("HardwareTimeFreq",       &Frame::HardwareTimeFreq)
        .def("MasterTimingDevice",     &Frame::MasterTimingDevice)
        .def("ImageDataSize",          &Frame::ImageDataSize)
        .def("GrayscaleDataSize",      &Frame::GrayscaleDataSize)
        .def("SetObjectCount",         &Frame::SetObjectCount)
        .def("RemoveObject",           &Frame::RemoveObject);

    nb::class_<FrameGroup>(m, "FrameGroup")
        .def(nb::init())
        .def("Count",                  &FrameGroup::Count)
        .def("GetFrame",               &FrameGroup::GetFrame, nb::rv_policy::reference)
        .def("AddFrame",               &FrameGroup::AddFrame)
        .def("SetMode",                &FrameGroup::SetMode)
        .def("Mode",                   &FrameGroup::Mode)
        .def("SetTimeStamp",           &FrameGroup::SetTimeStamp)
        .def("SetTimeSpread",          &FrameGroup::SetTimeSpread)
        .def("SetEarliestTimeStamp",   &FrameGroup::SetEarliestTimeStamp)
        .def("SetLatestTimeStamp",     &FrameGroup::SetLatestTimeStamp)
        .def("TimeSpread",             &FrameGroup::TimeSpread)
        .def("TimeStamp",              &FrameGroup::TimeStamp)
        .def("EarliestTimeStamp",      &FrameGroup::EarliestTimeStamp)
        .def("LatestTimeStamp",        &FrameGroup::LatestTimeStamp)
        .def("FrameID",                &FrameGroup::FrameID)
        .def("TimeSpreadDeviation",    &FrameGroup::TimeSpreadDeviation)
        .def("DroppedFrames",          &FrameGroup::DroppedFrames);

    // -------------------------------------------------------------------------
    // Camera
    // -------------------------------------------------------------------------

    nb::class_<Camera>(m, "Camera", "Represents a connected OptiTrack camera.")
        .def("Width",                           &Camera::Width)
        .def("Height",                          &Camera::Height)
        .def("LatestFrame",                     &Camera::LatestFrame)
        .def("NextFrame",                       &Camera::NextFrame)
        .def("Name",                            &Camera::Name)
        .def("Start",                           &Camera::Start)
        .def("Stop",                            &Camera::Stop)
        .def("IsCameraRunning",                 &Camera::IsCameraRunning)
        .def("SetNumeric",                      &Camera::SetNumeric)
        .def("SetExposure",                     &Camera::SetExposure)
        .def("SetThreshold",                    &Camera::SetThreshold)
        .def("SetIntensity",                    &Camera::SetIntensity)
        .def("SetPrecisionCap",                 &Camera::SetPrecisionCap)
        .def("SetShutterDelay",                 &Camera::SetShutterDelay)
        .def("SetStrobeOffset",                 &Camera::SetStrobeOffset)
        .def("SetFrameRate",                    &Camera::SetFrameRate)
        .def("FrameRate",                       &Camera::FrameRate)
        .def("SetFrameDecimation",              &Camera::SetFrameDecimation)
        .def("FrameDecimation",                 &Camera::FrameDecimation)
        .def("GrayscaleDecimation",             &Camera::GrayscaleDecimation)
        .def("PrecisionCap",                    &Camera::PrecisionCap)
        .def("ShutterDelay",                    &Camera::ShutterDelay)
        .def("StrobeOffset",                    &Camera::StrobeOffset)
        .def("Exposure",                        &Camera::Exposure)
        .def("Threshold",                       &Camera::Threshold)
        .def("Intensity",                       &Camera::Intensity)
        .def("SetVideoType",                    &Camera::SetVideoType)
        .def("IsVideoTypeSupported",            &Camera::IsVideoTypeSupported)
        .def("IsVideoTypeSynchronous",          &Camera::IsVideoTypeSynchronous)
        .def("DataRate",                        &Camera::DataRate)
        .def("PacketSize",                      &Camera::PacketSize)
        .def("SetGrayscaleDecimation",          &Camera::SetGrayscaleDecimation)
        .def("SendEmptyFrames",                 &Camera::SendEmptyFrames)
        .def("SendInvalidFrames",               &Camera::SendInvalidFrames)
        .def("SetLateDecompression",            &Camera::SetLateDecompression)
        .def("LateDecompression",               &Camera::LateDecompression)
        .def("Serial",                          &Camera::Serial)
        .def("SerialString",                    &Camera::SerialString)
        .def("Model",                           &Camera::Model)
        .def("SubModel",                        &Camera::SubModel)
        .def("Revision",                        &Camera::Revision)
        .def("HardwareInterface",               &Camera::HardwareInterface)
        .def("CameraID",                        &Camera::CameraID)
        .def("CameraIDValid",                   &Camera::CameraIDValid)
        .def("SetIRFilter",                     &Camera::SetIRFilter)
        .def("IRFilter",                        &Camera::IRFilter)
        .def("IsFilterSwitchAvailable",         &Camera::IsFilterSwitchAvailable)
        .def("SetAGC",                          &Camera::SetAGC)
        .def("AGC",                             &Camera::AGC)
        .def("IsAGCAvailable",                  &Camera::IsAGCAvailable)
        .def("SetAEC",                          &Camera::SetAEC)
        .def("AEC",                             &Camera::AEC)
        .def("IsAECAvailable",                  &Camera::IsAECAvailable)
        .def("SetImagerGain",                   &Camera::SetImagerGain)
        .def("ImagerGain",                      &Camera::ImagerGain)
        .def("IsImagerGainAvailable",           &Camera::IsImagerGainAvailable)
        .def("ImagerGainLevels",                &Camera::ImagerGainLevels)
        .def("SetHighPowerMode",                &Camera::SetHighPowerMode)
        .def("HighPowerMode",                   &Camera::HighPowerMode)
        .def("IsHighPowerModeAvailable",        &Camera::IsHighPowerModeAvailable)
        .def("IsHighPowerModeSupported",        &Camera::IsHighPowerModeSupported)
        .def("LowPowerSetting",                 &Camera::LowPowerSetting)
        .def("ActualFrameRate",                 &Camera::ActualFrameRate)
        .def("SetMJPEGQuality",                 &Camera::SetMJPEGQuality)
        .def("MJPEGQuality",                    &Camera::MJPEGQuality)
        .def("IsMJPEGAvailable",                &Camera::IsMJPEGAvailable)
        .def("IsContinuousIRAvailable",         &Camera::IsContinuousIRAvailable)
        .def("SetContinuousIR",                 &Camera::SetContinuousIR)
        .def("ContinuousIR",                    &Camera::ContinuousIR)
        .def("SetRinglightEnabledWhileStopped", &Camera::SetRinglightEnabledWhileStopped)
        .def("RinglightEnabledWhileStopped",    &Camera::RinglightEnabledWhileStopped)
        .def("IsHardwareFiltered",              &Camera::IsHardwareFiltered)
        .def("SwitchState",                     &Camera::SwitchState)
        .def("GetDistortionModel",              &Camera::GetDistortionModel)
        .def("ResetWindow",                     &Camera::ResetWindow)
        .def("SetWindow",                       &Camera::SetWindow)
        .def("IsWindowingSupported",            &Camera::IsWindowingSupported)
        .def("CalcWindow",                      &Camera::CalcWindow)
        .def("SetLED",                          &Camera::SetLED)
        .def("SetAllLED",                       &Camera::SetAllLED)
        .def("SetStatusIntensity",              &Camera::SetStatusIntensity)
        .def("StatusRingLightCount",            &Camera::StatusRingLightCount)
        .def("SetStatusRingLights",             &Camera::SetStatusRingLights)
        .def("SetStatusRingRGB",                &Camera::SetStatusRingRGB)
        .def("IsIRIlluminationAvailable",       &Camera::IsIRIlluminationAvailable)
        .def("SetEnableBlockingMask",           &Camera::SetEnableBlockingMask)
        .def("IsBlockingMaskEnabled",           &Camera::IsBlockingMaskEnabled)
        .def("AddBlockingRectangle",            &Camera::AddBlockingRectangle)
        .def("RemoveBlockingRectangle",         &Camera::RemoveBlockingRectangle)
        .def("SetBitMaskPixel",                 &Camera::SetBitMaskPixel)
        .def("ClearBlockingMask",               &Camera::ClearBlockingMask)
        .def("UpdateBlockingMask",              &Camera::UpdateBlockingMask)
        .def("BlockingMaskWidth",               &Camera::BlockingMaskWidth)
        .def("BlockingMaskHeight",              &Camera::BlockingMaskHeight)
        .def("BlockingGrid",                    &Camera::BlockingGrid)
        .def("ImagerWidth",                     &Camera::ImagerWidth)
        .def("ImagerHeight",                    &Camera::ImagerHeight)
        .def("FocalLength",                     &Camera::FocalLength)
        .def("HardwareFrameRate",               &Camera::HardwareFrameRate)
        .def("PhysicalPixelWidth",              &Camera::PhysicalPixelWidth)
        .def("PhysicalPixelHeight",             &Camera::PhysicalPixelHeight)
        .def("SetTextOverlay",                  &Camera::SetTextOverlay)
        .def("TextOverlay",                     &Camera::TextOverlay)
        .def("SetMarkerOverlay",                &Camera::SetMarkerOverlay)
        .def("MarkerOverlay",                   &Camera::MarkerOverlay)
        .def("IsInitialized",                   &Camera::IsInitialized)
        .def("IsDisconnected",                  &Camera::IsDisconnected)
        .def("State",                           &Camera::State)
        .def("UID",                             &Camera::UID)
        .def("ConnectionType",                  &Camera::ConnectionType)
        .def("IsVirtual",                       &Camera::IsVirtual)
        .def("IsCommandQueueEmpty",             &Camera::IsCommandQueueEmpty)
        .def("AttachModule",                    &Camera::AttachModule)
        .def("RemoveModule",                    &Camera::RemoveModule)
        .def("ModuleCount",                     &Camera::ModuleCount)
        .def("AttachListener",                  &Camera::AttachListener)
        .def("RemoveListener",                  &Camera::RemoveListener)
        .def("Shutdown",                        &Camera::Shutdown)
        .def("IsCamera",                        &Camera::IsCamera)
        .def("IsHardwareKey",                   &Camera::IsHardwareKey)
        .def("IsHub",                           &Camera::IsHub)
        .def("IsUSB",                           &Camera::IsUSB)
        .def("IsEthernet",                      &Camera::IsEthernet)
        .def("IsTBar",                          &Camera::IsTBar)
        .def("IsSyncAuthority",                 &Camera::IsSyncAuthority)
        .def("IsBaseStation",                   &Camera::IsBaseStation)
        .def("SyncFeatures",                    &Camera::SyncFeatures)
        .def("SetObjectColor",                  &Camera::SetObjectColor)
        .def("ObjectColor",                     &Camera::ObjectColor)
        .def("SetEnablePayload",                &Camera::SetEnablePayload)
        .def("IsEnablePayload",                 &Camera::IsEnablePayload)
        .def("IsCameraTempValid",               &Camera::IsCameraTempValid)
        .def("CameraTemp",                      &Camera::CameraTemp)
        .def("IsRinglightTempValid",            &Camera::IsRinglightTempValid)
        .def("RinglightTemp",                   &Camera::RinglightTemp)
        .def("SetLLDPDetection",                &Camera::SetLLDPDetection)
        .def("IsLLDPDetectionAvailable",        &Camera::IsLLDPDetectionAvailable)
        .def("LLDPDetection",                   &Camera::LLDPDetection)
        .def("MinimumExposureValue",            &Camera::MinimumExposureValue)
        .def("MaximumExposureValue",            &Camera::MaximumExposureValue)
        .def("MinimumFrameRateValue",           &Camera::MinimumFrameRateValue)
        .def("MaximumFrameRateValue",           &Camera::MaximumFrameRateValue)
        .def("MaximumFullImageFrameRateValue",  &Camera::MaximumFullImageFrameRateValue)
        .def("MinimumThreshold",                &Camera::MinimumThreshold)
        .def("MaximumThreshold",                &Camera::MaximumThreshold)
        .def("MinimumIntensity",                &Camera::MinimumIntensity)
        .def("MaximumIntensity",                &Camera::MaximumIntensity)
        .def("MaximumMJPEGRateValue",           &Camera::MaximumMJPEGRateValue)
        .def("StorageMaxSize",                  &Camera::StorageMaxSize)
        .def("OptiHubConnectivity",             &Camera::OptiHubConnectivity)
        .def("IsColor",                         &Camera::IsColor)
        .def("SetColorMatrix",                  &Camera::SetColorMatrix)
        .def("SetColorGamma",                   &Camera::SetColorGamma)
        .def("SetColorPrescalar",               &Camera::SetColorPrescalar)
        .def("SetColorCompression",             &Camera::SetColorCompression)
        .def("ColorMatrix",                     &Camera::ColorMatrix)
        .def("ColorGamma",                      &Camera::ColorGamma)
        .def("ColorPrescalar",                  &Camera::ColorPrescalar)
        .def("ColorMode",                       &Camera::ColorMode)
        .def("ColorCompression",               &Camera::ColorCompression)
        .def("ColorBitRate",                    &Camera::ColorBitRate)
        .def("CameraResolutionCount",           &Camera::CameraResolutionCount)
        .def("CameraResolutionID",              &Camera::CameraResolutionID)
        .def("CameraResolution",                &Camera::CameraResolution)
        .def("SetCameraResolution",             &Camera::SetCameraResolution)
        .def("QueryHardwareTimeStampValue",     &Camera::QueryHardwareTimeStampValue)
        .def("IsHardwareTimeStampValueSupported", &Camera::IsHardwareTimeStampValueSupported)
        .def("SetColorEnhancement",             &Camera::SetColorEnhancement)
        .def("ColorEnhancement",                &Camera::ColorEnhancement);

    // -------------------------------------------------------------------------
    // CameraManager
    // -------------------------------------------------------------------------

    nb::class_<cCameraLibraryStartupSettings>(m, "cCameraLibraryStartupSettings")
        .def(nb::init())
        .def_static("X",                  cCameraLibraryStartupSettings::X)
        .def("EnableDevelopment",         &cCameraLibraryStartupSettings::EnableDevelopment)
        .def("IsDevelopmentEnabled",      &cCameraLibraryStartupSettings::IsDevelopmentEnabled);

    nb::class_<cCameraManagerListener>(m, "CameraManagerListener")
        .def("CameraConnected",                       &cCameraManagerListener::CameraConnected)
        .def("CameraRemoved",                         &cCameraManagerListener::CameraRemoved)
        .def("SyncSettingsChanged",                   &cCameraManagerListener::SyncSettingsChanged)
        .def("CameraInitialized",                     &cCameraManagerListener::CameraInitialized)
        .def("SyncAuthorityInitialized",              &cCameraManagerListener::SyncAuthorityInitialized)
        .def("SyncAuthorityRemoved",                  &cCameraManagerListener::SyncAuthorityRemoved)
        .def("CameraMessage",                         &cCameraManagerListener::CameraMessage)
        .def("RequestUnknownDeviceImplementation",    &cCameraManagerListener::RequestUnknownDeviceImplementation);

    nb::class_<CameraManager>(m, "CameraManager", "Singleton managing all connected OptiTrack cameras. Access via CameraManager.X().")
        .def_static("X",                             CameraManager::X, nb::rv_policy::reference)
        .def("WaitForInitialization",                &CameraManager::WaitForInitialization)
        .def("AreCamerasInitialized",                &CameraManager::AreCamerasInitialized)
        .def("AreCamerasShutdown",                   &CameraManager::AreCamerasShutdown)
        .def("Shutdown",                             &CameraManager::Shutdown)
        .def("GetCameraBySerial",                    &CameraManager::GetCameraBySerial)
        .def("GetCamera",                            nb::overload_cast<const Core::cUID&>(&CameraManager::GetCamera))
        .def("GetCamera",                            nb::overload_cast<>(&CameraManager::GetCamera))
        .def("GetCameraList",                        &CameraManager::GetCameraList)
        .def("GetHardwareKey",                       &CameraManager::GetHardwareKey)
        .def("GetDevice",                            &CameraManager::GetDevice)
        .def("PrepareForSuspend",                    &CameraManager::PrepareForSuspend)
        .def("ResumeFromSuspend",                    &CameraManager::ResumeFromSuspend)
        .def("TimeStamp",                            &CameraManager::TimeStamp)
        .def("RegisterListener",                     &CameraManager::RegisterListener)
        .def("UnregisterListener",                   &CameraManager::UnregisterListener)
        .def_static("CameraFactory",                 CameraManager::CameraFactory)
        .def("AddCamera",                            &CameraManager::AddCamera)
        .def("RemoveCamera",                         &CameraManager::RemoveCamera)
        .def("ScanForCameras",                       &CameraManager::ScanForCameras)
        .def("ApplySyncSettings",                    &CameraManager::ApplySyncSettings)
        .def("GetSyncSettings",                      &CameraManager::GetSyncSettings)
        .def("SyncSettings",                         &CameraManager::SyncSettings)
        .def("SoftwareTrigger",                      &CameraManager::SoftwareTrigger)
        .def("SyncMode",                             &CameraManager::SyncMode)
        .def("UpdateRecordingBit",                   &CameraManager::UpdateRecordingBit)
        .def("GetSyncFeatures",                      &CameraManager::GetSyncFeatures)
        .def("ShouldLockCameraExposures",            &CameraManager::ShouldLockCameraExposures)
        .def("ShouldForceCameraRateControls",        &CameraManager::ShouldForceCameraRateControls)
        .def("ShouldApplySyncOnExposureChange",      &CameraManager::ShouldApplySyncOnExposureChange)
        .def("SuggestCameraIDOrder",                 &CameraManager::SuggestCameraIDOrder)
        .def_static("DestroyInstance",               CameraManager::DestroyInstance)
        .def_static("IsActive",                      CameraManager::IsActive)
        .def_static("Ptr",                           CameraManager::Ptr);

    // -------------------------------------------------------------------------
    // cModuleSync
    // -------------------------------------------------------------------------

    nb::class_<CameraLibrary::cModuleSync>(m, "cModuleSync", "Frame synchronizer that groups frames from multiple cameras.")
        .def_static("Create",                CameraLibrary::cModuleSync::Create, nb::rv_policy::reference)
        .def_static("Destroy",               CameraLibrary::cModuleSync::Destroy)
        .def("PostFrame",                    &CameraLibrary::cModuleSync::PostFrame)
        .def("FrameDeliveryRate",            &CameraLibrary::cModuleSync::FrameDeliveryRate)
        .def("AddCamera",                    &CameraLibrary::cModuleSync::AddCamera)
        .def("CameraCount",                  &CameraLibrary::cModuleSync::CameraCount)
        .def("GetFrameGroup",                &CameraLibrary::cModuleSync::GetFrameGroup)
        .def("LastFrameGroupMode",           &CameraLibrary::cModuleSync::LastFrameGroupMode)
        .def("AllowIncompleteGroups",        &CameraLibrary::cModuleSync::AllowIncompleteGroups)
        .def("SetAllowIncompleteGroups",     &CameraLibrary::cModuleSync::SetAllowIncompleteGroups)
        .def("SetOptimization",              &CameraLibrary::cModuleSync::SetOptimization)
        .def("Optimization",                 &CameraLibrary::cModuleSync::Optimization)
        .def("RemoveAllCameras",             &CameraLibrary::cModuleSync::RemoveAllCameras)
        .def("SetSuppressOutOfOrder",        &CameraLibrary::cModuleSync::SetSuppressOutOfOrder);
}
