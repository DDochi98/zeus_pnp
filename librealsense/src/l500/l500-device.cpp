// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2018 Intel Corporation. All Rights Reserved.

#include "l500-device.h"

#include "context.h"
#include "stream.h"
#include "image.h"

#include "l500-depth.h"
#include "l500-color.h"
#include "l500-private.h"

#include <src/proc/decimation-filter.h>
#include <src/proc/threshold.h>
#include <src/proc/spatial-filter.h>
#include <src/proc/temporal-filter.h>
#include <src/proc/hole-filling-filter.h>
#include <src/proc/zero-order.h>
#include <src/proc/syncer-processing-block.h>
#include <src/proc/rotation-transform.h>

#include <common/fw/firmware-version.h>
#include <rsutils/time/periodic-timer.h>
#include <rsutils/time/work-week.h>
#include <common/utilities/time/l500/get-mfr-ww.h>
#include <rsutils/string/from.h>

#include <vector>


namespace librealsense
{
    std::map<uint32_t, rs2_format> l500_depth_fourcc_to_rs2_format = {
        { rs_fourcc('G','R','E','Y'), RS2_FORMAT_Y8 },
        { rs_fourcc('Z','1','6',' '), RS2_FORMAT_Z16 },
        { rs_fourcc('C',' ',' ',' '), RS2_FORMAT_RAW8 },
        { rs_fourcc('C','N','F','4'), RS2_FORMAT_RAW8 },
        { rs_fourcc('F','G',' ',' '), RS2_FORMAT_FG },
    };

    std::map<uint32_t, rs2_stream> l500_depth_fourcc_to_rs2_stream = {
        { rs_fourcc('G','R','E','Y'), RS2_STREAM_INFRARED },
        { rs_fourcc('Z','1','6',' '), RS2_STREAM_DEPTH },
        { rs_fourcc('C',' ',' ',' '), RS2_STREAM_CONFIDENCE },
        { rs_fourcc('C','N','F','4'), RS2_STREAM_CONFIDENCE },
        { rs_fourcc('F','G',' ',' '), RS2_STREAM_DEPTH },
    };

    using namespace ivcam2;

    l500_device::l500_device(std::shared_ptr<context> ctx,
        const platform::backend_device_group& group)
        :device(ctx, group), global_time_interface(), 
        _depth_stream(new stream(RS2_STREAM_DEPTH)),
        _ir_stream(new stream(RS2_STREAM_INFRARED)),
        _confidence_stream(new stream(RS2_STREAM_CONFIDENCE)),
        _temperatures()
    {
        _depth_device_idx = add_sensor(create_depth_device(ctx, group.uvc_devices));
        _pid = group.uvc_devices.front().pid;
        std::string device_name = (rs500_sku_names.end() != rs500_sku_names.find(_pid)) ? rs500_sku_names.at(_pid) : "RS5xx";

        using namespace ivcam2;

        auto&& backend = ctx->get_backend();

        auto& depth_sensor = get_depth_sensor();
        auto& raw_depth_sensor = get_raw_depth_sensor();

#ifndef HWM_OVER_XU
        if( group.usb_devices.size() > 0 )
        {
            // This use-case is mainly to support FW development & debugging on unlock units before they
            // have any XU capabilities
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>( backend.create_usb_device( group.usb_devices.front() ),
                    raw_depth_sensor ) );
        }
        else
#endif
        {
            _hw_monitor = std::make_shared<hw_monitor>(
                std::make_shared<locked_transfer>( std::make_shared<command_transfer_over_xu>(
                    raw_depth_sensor, depth_xu, L500_HWMONITOR ),
                    raw_depth_sensor ) );
        }

        std::vector<uint8_t> gvd_buff(HW_MONITOR_BUFFER_SIZE);
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);
        // fooling tests recordings - don't remove
        _hw_monitor->get_gvd(gvd_buff.size(), gvd_buff.data(), GVD);

        auto optic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_serial_offset, module_serial_size);
        auto asic_serial = _hw_monitor->get_module_serial_string(gvd_buff, module_asic_serial_offset, module_asic_serial_size);
        auto fwv = _hw_monitor->get_firmware_version_string(gvd_buff, fw_version_offset);
        _fw_version = firmware_version(fwv);

        _is_locked = _hw_monitor->get_gvd_field<uint8_t>(gvd_buff, is_camera_locked_offset) != 0;

        auto pid_hex_str = hexify(group.uvc_devices.front().pid);

        using namespace platform;

        _usb_mode = raw_depth_sensor.get_usb_specification();
        if (usb_spec_names.count(_usb_mode) && (usb_undefined != _usb_mode))
        {
            auto usb_type_str = usb_spec_names.at(_usb_mode);
            register_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR, usb_type_str);
        }

        register_info(RS2_CAMERA_INFO_NAME, device_name);
        register_info(RS2_CAMERA_INFO_SERIAL_NUMBER, optic_serial);
        register_info(RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER, asic_serial);
        register_info(RS2_CAMERA_INFO_FIRMWARE_UPDATE_ID, asic_serial);
        register_info(RS2_CAMERA_INFO_FIRMWARE_VERSION, _fw_version);
        register_info(RS2_CAMERA_INFO_DEBUG_OP_CODE, std::to_string(static_cast<int>(fw_cmd::GLD)));
        register_info(RS2_CAMERA_INFO_PHYSICAL_PORT, group.uvc_devices.front().device_path);
        register_info(RS2_CAMERA_INFO_PRODUCT_ID, pid_hex_str);
        register_info(RS2_CAMERA_INFO_PRODUCT_LINE, "L500");
        register_info(RS2_CAMERA_INFO_CAMERA_LOCKED, _is_locked ? "YES" : "NO");

        // If FW supportes the SET_AGE command, we update the age of the device in weeks to aid projection of aging
        if( ( _fw_version >= firmware_version( "1.5.4.0" ) ) )
        {
            try
            {
                auto manufacture
                    = utilities::time::l500::get_manufacture_work_week( optic_serial );
                auto age
                    = rsutils::time::get_work_weeks_since( manufacture );
                command cmd( fw_cmd::UNIT_AGE_SET, (uint8_t)age );
                _hw_monitor->send( cmd );
            }
            catch( ... )
            {
                LOG_ERROR( "Failed to set units age" );
            }
        }

        configure_depth_options();
    }


    l500_depth_sensor & l500_device::get_depth_sensor()
    {
        return dynamic_cast<l500_depth_sensor &>(get_sensor( _depth_device_idx ));
    }


    std::shared_ptr<synthetic_sensor> l500_device::create_depth_device( std::shared_ptr<context> ctx,
        const std::vector<platform::uvc_device_info>& all_device_infos )
    {
        auto&& backend = ctx->get_backend();

        std::vector<std::shared_ptr<platform::uvc_device>> depth_devices;
        for( auto&& info : filter_by_mi( all_device_infos, 0 ) ) // Filter just mi=0, DEPTH
            depth_devices.push_back( backend.create_uvc_device( info ) );

        std::unique_ptr<frame_timestamp_reader> timestamp_reader_metadata( new l500_timestamp_reader_from_metadata( backend.create_time_service() ) );
        auto enable_global_time_option = std::shared_ptr<global_time_option>( new global_time_option() );
        auto raw_depth_ep = std::make_shared<uvc_sensor>( "Raw Depth Sensor", std::make_shared<platform::multi_pins_uvc_device>( depth_devices ),
            std::unique_ptr<frame_timestamp_reader>( new global_timestamp_reader( std::move( timestamp_reader_metadata ), _tf_keeper, enable_global_time_option ) ), this );
        raw_depth_ep->register_xu( depth_xu );

        auto depth_ep = std::make_shared<l500_depth_sensor>( this, raw_depth_ep, l500_depth_fourcc_to_rs2_format, l500_depth_fourcc_to_rs2_stream );

        depth_ep->register_option( RS2_OPTION_GLOBAL_TIME_ENABLED, enable_global_time_option );
        depth_ep->get_option( RS2_OPTION_GLOBAL_TIME_ENABLED ).set( 0 );

        // NOTE: _fw_version is not yet initialized! Any additional options should get added from configure_depth_options()!
        depth_ep->register_info(RS2_CAMERA_INFO_PHYSICAL_PORT, filter_by_mi(all_device_infos, 0).front().device_path);
        return depth_ep;
    }


    /** This processing block removes all frames that are not of the given stream types
     *
     * This is only a workaround!!!
     * It seems that, when definining a processing block outputs, if any other frames exist there
     * then an issue can be exhibited where duplicate frames are produced. This solved the issue.
     */
    class filtering_processing_block : public generic_processing_block {
        std::vector< rs2_stream > _streams;

    public:
        filtering_processing_block( rs2_stream stream_to_pass )
            : generic_processing_block( "filtering_processing_block" ), _streams( 1, stream_to_pass ) {}
        filtering_processing_block( std::initializer_list< rs2_stream > const & streams )
            : generic_processing_block( "filtering_processing_block" ), _streams( streams ) {}

        rs2::frame process_frame( const rs2::frame_source & source,
                                  const rs2::frame & f ) override {
            return f;
        }

    private:
        bool should_process( const rs2::frame & f ) override {
            auto fs = f.as< rs2::frameset >();
            if( fs )
                return false;  // we'll get the individual frames back by themselves:
            auto it = std::find( _streams.begin(), _streams.end(), f.get_profile().stream_type() );
            return ( it != _streams.end() );  // keep the frame only if one of those we got
        }
        rs2::frame prepare_output( const rs2::frame_source & source, rs2::frame input,
                                   std::vector< rs2::frame > results ) override {
            if( results.empty() )
                return rs2::frame{};
            return source.allocate_composite_frame(results);
        }
    };


    void l500_device::configure_depth_options()
    {
        synthetic_sensor & depth_sensor = get_depth_sensor();

        depth_sensor.register_processing_block(
            { {RS2_FORMAT_Z16}, {RS2_FORMAT_Y8} },
            { {RS2_FORMAT_Z16, RS2_STREAM_DEPTH, 0, 0, 0, 0, &rotate_resolution} },
            [=]() {
                auto z16rot = std::make_shared<rotation_transform>(RS2_FORMAT_Z16, RS2_STREAM_DEPTH, RS2_EXTENSION_DEPTH_FRAME);
                auto y8rot = std::make_shared<rotation_transform>(RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME);
                auto sync = std::make_shared<syncer_process_unit>(nullptr, false); // disable logging on this internal syncer

                auto cpb = std::make_shared<composite_processing_block>();
                cpb->add(z16rot);
                cpb->add(y8rot);
                cpb->add(sync);
                cpb->add( std::make_shared< filtering_processing_block >( RS2_STREAM_DEPTH ) );
                return cpb;
            }
        );


        depth_sensor.register_processing_block(
            { {RS2_FORMAT_Z16}, {RS2_FORMAT_Y8}, {RS2_FORMAT_RAW8} },
            {
                {RS2_FORMAT_Z16, RS2_STREAM_DEPTH, 0, 0, 0, 0, &rotate_resolution},
                {RS2_FORMAT_RAW8, RS2_STREAM_CONFIDENCE, 0, 0, 0, 0, &l500_confidence_resolution}
            },
            [=]() {
                auto z16rot = std::make_shared<rotation_transform>(RS2_FORMAT_Z16, RS2_STREAM_DEPTH, RS2_EXTENSION_DEPTH_FRAME);
                auto y8rot = std::make_shared<rotation_transform>(RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME);
                auto conf = std::make_shared<confidence_rotation_transform>();
                auto sync = std::make_shared<syncer_process_unit>(nullptr, false); // disable logging on this internal syncer

                auto cpb = std::make_shared<composite_processing_block>();
                cpb->add(z16rot);
                cpb->add(y8rot);
                cpb->add(conf);
                cpb->add(sync);
                cpb->add( std::shared_ptr< filtering_processing_block >(
                    new filtering_processing_block{RS2_STREAM_DEPTH, RS2_STREAM_CONFIDENCE} ) );
                return cpb;
            }
        );

        depth_sensor.register_processing_block(
            { {RS2_FORMAT_Y8} },
            { {RS2_FORMAT_Y8, RS2_STREAM_INFRARED, 0, 0, 0, 0, &rotate_resolution} },
            []() { return std::make_shared<rotation_transform>(RS2_FORMAT_Y8, RS2_STREAM_INFRARED, RS2_EXTENSION_VIDEO_FRAME); }
        );

        depth_sensor.register_processing_block(
            { {RS2_FORMAT_RAW8} },
            { {RS2_FORMAT_RAW8, RS2_STREAM_CONFIDENCE, 0, 0, 0, 0, &l500_confidence_resolution} },
            []() { return std::make_shared<confidence_rotation_transform>(); }
        );

        depth_sensor.register_processing_block(
            processing_block_factory::create_id_pbf( RS2_FORMAT_FG, RS2_STREAM_DEPTH ) );

        std::shared_ptr< freefall_option > freefall_opt;
        if( _fw_version >= firmware_version( "1.3.5.0" ) )
        {
            depth_sensor.register_option(
                RS2_OPTION_FREEFALL_DETECTION_ENABLED,
                freefall_opt = std::make_shared< freefall_option >( *_hw_monitor )
            );
        }
        else
        {
            LOG_DEBUG( "Skipping Freefall control: requires FW 1.3.5" );
        }
        if( _fw_version >= firmware_version( "1.3.12.9" ) )
        {
            depth_sensor.register_option(
                RS2_OPTION_INTER_CAM_SYNC_MODE,
                std::make_shared< hw_sync_option >( *_hw_monitor, freefall_opt )
            );
        }
        else
        {
            LOG_DEBUG( "Skipping HW Sync control: requires FW 1.3.12.9" );
        }
    }

    void l500_device::force_hardware_reset() const
    {
        command cmd(ivcam2::fw_cmd::HW_RESET);
        cmd.require_response = false;
        _hw_monitor->send(cmd);
    }

    void l500_device::create_snapshot(std::shared_ptr<debug_interface>& snapshot) const
    {
        throw not_implemented_exception("create_snapshot(...) not implemented!");
    }

    void l500_device::enable_recording(std::function<void(const debug_interface&)> record_action)
    {
        throw not_implemented_exception("enable_recording(...) not implemented!");
    }

    double l500_device::get_device_time_ms()
    {
        if (!_hw_monitor)
            throw wrong_api_call_sequence_exception("_hw_monitor is not initialized yet");

        command cmd(ivcam2::fw_cmd::MRD, ivcam2::REGISTER_CLOCK_0, ivcam2::REGISTER_CLOCK_0 + 4);
        // TODO -Redirect HW Monitor commands to used atomic (UVC) transfers for faster transactions and transfer integrity
        // Disabled due to limitation in FW
        auto res = _hw_monitor->send(cmd, nullptr);

        if (res.size() < sizeof(uint32_t))
        {
            LOG_DEBUG("size(res):" << res.size());
            throw std::runtime_error("Not enough bytes returned from the firmware!");
        }
        uint32_t dt = *(uint32_t*)res.data();
        double ts = dt * TIMESTAMP_USEC_TO_MSEC;
        return ts;
    }

    void l500_device::enter_update_state() const
    {
        // Stop all data streaming/exchange pipes with HW
        stop_activity();
        using namespace std;
        using namespace std::chrono;

        try
        {
            LOG_INFO( "entering to update state, device disconnect is expected" );
            command cmd( ivcam2::DFU );
            cmd.param1 = 1;
            _hw_monitor->send( cmd );

            // We allow 6 seconds because on Linux the removal status is updated at a 5 seconds rate.
            const int MAX_ITERATIONS_FOR_DEVICE_DISCONNECTED_LOOP = DISCONNECT_PERIOD_MS / DELAY_FOR_RETRIES;
            for( auto i = 0; i < MAX_ITERATIONS_FOR_DEVICE_DISCONNECTED_LOOP; i++ )
            {
                // If the device was detected as removed we assume the device is entering update
                // mode Note: if no device status callback is registered we will wait the whole time
                // and it is OK
                if( ! is_valid() )
                {
                    this_thread::sleep_for( milliseconds( DELAY_FOR_CONNECTION ) );
                    return;
                }

                this_thread::sleep_for( milliseconds( DELAY_FOR_RETRIES ) );
            }

            if (device_changed_notifications_on())
                LOG_WARNING("Timeout waiting for device disconnect after DFU command!");
        }
        catch( std::exception & e )
        {
            LOG_ERROR( e.what() );
        }
        catch( ... )
        {
            LOG_ERROR( "Unknown error during entering DFU state" );
        }
    }

    std::vector<uint8_t> l500_device::backup_flash(update_progress_callback_ptr callback)
    {
        int flash_size = 1024 * 2048;
        int max_bulk_size = 1016;
        int max_iterations = int(flash_size / max_bulk_size + 1);

        std::vector<uint8_t> flash;
        flash.reserve(flash_size);

        get_raw_depth_sensor().invoke_powered([&](platform::uvc_device& dev)
        {
            for (int i = 0; i < max_iterations; i++)
            {
                int offset = max_bulk_size * i;
                int size = max_bulk_size;
                if (i == max_iterations - 1)
                {
                    size = flash_size - offset;
                }

                bool appended = false;

                const int retries = 3;
                for (int j = 0; j < retries && !appended; j++)
                {
                    try
                    {
                        command cmd(ivcam2::FRB);
                        cmd.param1 = offset;
                        cmd.param2 = size;
                        auto res = _hw_monitor->send(cmd);

                        flash.insert(flash.end(), res.begin(), res.end());
                        appended = true;
                    }
                    catch (...)
                    {
                        if (i < retries - 1) std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        else throw;
                    }
                }

                if (callback) callback->on_update_progress((float)i / max_iterations);
            }
            if (callback) callback->on_update_progress(1.0);
        });

        return flash;
    }

    void l500_device::update_flash_section(std::shared_ptr<hw_monitor> hwm, const std::vector<uint8_t>& image, uint32_t offset, uint32_t size, update_progress_callback_ptr callback, float continue_from, float ratio)
    {
        int sector_count = int( size / ivcam2::FLASH_SECTOR_SIZE );
        int first_sector = int( offset / ivcam2::FLASH_SECTOR_SIZE );

        if (sector_count * ivcam2::FLASH_SECTOR_SIZE != size)
            sector_count++;

        sector_count += first_sector;

        for (int sector_index = first_sector; sector_index < sector_count; sector_index++)
        {
            command cmdFES(ivcam2::FES);
            // On both FES & FWB commands We don't really need the response but we see that setting
            // false and sending many commands cause a failure. 
            // Looks like the FW is expecting it.
            cmdFES.require_response = true;
            cmdFES.param1 = int(sector_index);
            cmdFES.param2 = 1;
            auto res = hwm->send(cmdFES);

            for (int i = 0; i < ivcam2::FLASH_SECTOR_SIZE; )
            {
                auto index = sector_index * ivcam2::FLASH_SECTOR_SIZE + i;
                if (index >= offset + size)
                    break;
                int packet_size = std::min((int)(HW_MONITOR_COMMAND_SIZE - (i % HW_MONITOR_COMMAND_SIZE)), (int)(ivcam2::FLASH_SECTOR_SIZE - i));
                command cmdFWB(ivcam2::FWB);
                cmdFWB.require_response = true;
                cmdFWB.param1 = int(index);
                cmdFWB.param2 = packet_size;
                cmdFWB.data.assign(image.data() + index, image.data() + index + packet_size);
                res = hwm->send(cmdFWB);
                i += packet_size;
            }

            if (callback)
                callback->on_update_progress(continue_from + (float)sector_index / (float)sector_count * ratio);
        }
    }

    void l500_device::update_section(std::shared_ptr<hw_monitor> hwm, const std::vector<uint8_t>& merged_image, flash_section fs, uint32_t tables_size,
        update_progress_callback_ptr callback, float continue_from, float ratio)
    {
        auto first_table_offset = fs.tables.front().offset;
        float total_size = float(fs.app_size + tables_size);

        float app_ratio = fs.app_size / total_size * ratio;
        float tables_ratio = tables_size / total_size * ratio;

        update_flash_section(hwm, merged_image, fs.offset, fs.app_size, callback, continue_from, app_ratio);
        update_flash_section(hwm, merged_image, first_table_offset, tables_size, callback, app_ratio, tables_ratio);
    }

    void l500_device::update_flash_internal(std::shared_ptr<hw_monitor> hwm, const std::vector<uint8_t>& image, std::vector<uint8_t>& flash_backup, update_progress_callback_ptr callback, int update_mode)
    {
        auto flash_image_info = ivcam2::get_flash_info(image);
        auto flash_backup_info = ivcam2::get_flash_info(flash_backup);
        auto merged_image = merge_images(flash_backup_info, flash_image_info, image);

        // update read-write section
        auto first_table_offset = flash_image_info.read_write_section.tables.front().offset;
        auto tables_size = flash_image_info.header.read_write_start_address + flash_image_info.header.read_write_size - first_table_offset;
        update_section(hwm, merged_image, flash_image_info.read_write_section, tables_size, callback, 0, update_mode == RS2_UNSIGNED_UPDATE_MODE_READ_ONLY ? 0.5f : 1.f);

        if (update_mode == RS2_UNSIGNED_UPDATE_MODE_READ_ONLY)
        {
            // update read-only section
            auto first_table_offset = flash_image_info.read_only_section.tables.front().offset;
            auto tables_size = flash_image_info.header.read_only_start_address + flash_image_info.header.read_only_size - first_table_offset;
            update_section(hwm, merged_image, flash_image_info.read_only_section, tables_size, callback, 0.5, 0.5);
        }
    }

    void l500_device::update_flash(const std::vector<uint8_t>& image, update_progress_callback_ptr callback, int update_mode)
    {
        if (_is_locked)
            throw std::runtime_error("this camera is locked and doesn't allow direct flash write, for firmware update use rs2_update_firmware method (DFU)");

        get_raw_depth_sensor().invoke_powered([&](platform::uvc_device& dev)
        {
            command cmdPFD(ivcam2::PFD);
            cmdPFD.require_response = false;
            auto res = _hw_monitor->send(cmdPFD);

            switch (update_mode)
            {
            case RS2_UNSIGNED_UPDATE_MODE_FULL:
                update_flash_section(_hw_monitor, image, 0, ivcam2::FLASH_SIZE, callback, 0, 1.0);
                break;
            case RS2_UNSIGNED_UPDATE_MODE_UPDATE:
            case RS2_UNSIGNED_UPDATE_MODE_READ_ONLY:
            {
                auto flash_backup = backup_flash(nullptr);
                update_flash_internal(_hw_monitor, image, flash_backup, callback, update_mode);
                break;
            }
            default:
                throw std::runtime_error("invalid update mode value");
            }

            if (callback) callback->on_update_progress(1.0);

            command cmdHWRST(ivcam2::HW_RESET);
            res = _hw_monitor->send(cmdHWRST);
        });
    }

    command l500_device::get_firmware_logs_command() const
    {
        return command{ ivcam2::GLD, 0x1f4 };
    }

    command l500_device::get_flash_logs_command() const
    {
        return command{ ivcam2::FRB, 0x0011E000, 0x3f8 };
    }

    static void log_FW_response_first_byte(hw_monitor& hwm, const std::string& command_name, const command &cmd, size_t expected_size)
    {
        auto res = hwm.send(cmd);
        if (res.size() < expected_size)
        {
            throw invalid_value_exception( rsutils::string::from()
                                           << command_name + " FW command failed: size expected: " << expected_size
                                           << " , size received: " << res.size() );
        }

        LOG_INFO(command_name << ": " << static_cast<int>(res[0]));
    }

    std::vector< uint8_t > l500_device::send_receive_raw_data(const std::vector< uint8_t > & input)
    {
        std::string command_str(input.begin(), input.end());

        if (command_str == "GET-NEST")
        {
            auto minimal_fw_ver = firmware_version("1.5.0.0");
            if (_fw_version >= minimal_fw_ver)
            {
                // Handle extended temperature command
                LOG_INFO("Nest AVG: " << get_temperatures().nest_avg);

                // Handle other commands (all results log the first byte)
                log_FW_response_first_byte(*_hw_monitor, "Gain trim",
                    command(ivcam2::IRB, 0x6C, 0x2, 0x1),
                    sizeof(uint8_t));
                log_FW_response_first_byte(*_hw_monitor, "IPF gain",
                    command(ivcam2::MRD, 0xA003007C, 0xA0030080),
                    sizeof(uint32_t));
                log_FW_response_first_byte(*_hw_monitor, "APB VBR",
                    command(ivcam2::AMCGET, 0x4, 0x0, 0x0),
                    sizeof(uint32_t));
                return std::vector< uint8_t >();
            }
            else
                throw librealsense::invalid_value_exception(
                    rsutils::string::from() << "get-nest command requires FW version >= " << minimal_fw_ver
                                            << ", current version is: " << _fw_version );
        }

        return _hw_monitor->send(input);
    }

    std::vector<uint8_t> l500_device::build_command(uint32_t opcode,
        uint32_t param1,
        uint32_t param2,
        uint32_t param3,
        uint32_t param4,
        uint8_t const * data,
        size_t dataLength) const
    {
        return _hw_monitor->build_command(opcode, param1, param2, param3, param4, data, dataLength);
    }

    ivcam2::extended_temperatures l500_device::get_temperatures() const
    {
        ivcam2::extended_temperatures rv;
        if (_have_temperatures)
        {
            std::lock_guard<std::mutex> lock(_temperature_mutex);
            rv = _temperatures;
        }
        else
        {
            // Noise estimation was added at FW version 1.5.0.0
            auto fw_version_support_nest = (_fw_version >= firmware_version("1.5.0.0")) ? true : false;
            auto expected_size = fw_version_support_nest ? sizeof(extended_temperatures)
                : sizeof(temperatures);


            const auto res = _hw_monitor->send(command{ TEMPERATURES_GET });
            // Verify read
            if (res.size() < expected_size)
            {
                throw std::runtime_error( rsutils::string::from()
                                          << "TEMPERATURES_GET - Invalid result size! expected: " << expected_size
                                          << " bytes, "
                                             "got: "
                                          << res.size() << " bytes" );
            }

            if (fw_version_support_nest)
                rv = *reinterpret_cast<extended_temperatures const *>(res.data());
            else
                *reinterpret_cast<temperatures *>(&rv) = *reinterpret_cast<temperatures const *>(res.data());
        }
        return rv;
    }

    void l500_device::start_temperatures_reader()
    {
        LOG_DEBUG("Starting temperature fetcher thread");
        _keep_reading_temperature = true;
        _temperature_reader = std::thread( [&]() {
            try
            {
                auto fw_version_support_nest = _fw_version >= firmware_version( "1.5.0.0" );

                auto expected_size = fw_version_support_nest ? sizeof( extended_temperatures )
                                                             : sizeof( temperatures );

                rsutils::time::periodic_timer second_has_passed(std::chrono::seconds(1));
                second_has_passed.set_expired(); // Force condition true on start


                while (_keep_reading_temperature)
                {
                    if (second_has_passed) // Update temperatures every second
                    {
                        const auto res = _hw_monitor->send(command{ TEMPERATURES_GET });
                        // Verify read
                        if (res.size() < expected_size)
                        {
                            throw std::runtime_error( rsutils::string::from()
                                                      << "TEMPERATURES_GET - Invalid result size!, expected: "
                                                      << expected_size
                                                      << " bytes, "
                                                         "got: "
                                                      << res.size() << " bytes" );
                        }
               
                        std::lock_guard<std::mutex> lock(_temperature_mutex);

                        memset(&_temperatures, 0, sizeof(_temperatures));
                        if (fw_version_support_nest)
                            _temperatures = *reinterpret_cast<extended_temperatures const *>(res.data());
                        else
                            *reinterpret_cast<temperatures *>(&_temperatures) = *reinterpret_cast<temperatures const *>(res.data());
                       
                        _have_temperatures = true;
                    }
               
                    // Do not hold the device alive too long if reader thread was turned off
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                }
             }
             catch (...)
             {
                 LOG_WARNING("unable to read temperatures - closing temperatures reader");
             }
             _have_temperatures = false;
         });
     }
    
    void l500_device::stop_temperatures_reader()
    {
        if (_keep_reading_temperature)
        {
            LOG_DEBUG("Stopping temperature fetcher thread");
            _keep_reading_temperature = false;
            _have_temperatures = false;
        }

        if (_temperature_reader.joinable())
        {
            _temperature_reader.join();
        }
    }

    bool l500_device::check_fw_compatibility(const std::vector<uint8_t>& image) const
    {
        std::string fw_version = extract_firmware_version_string(image);

        auto min_max_fw_it = ivcam2::device_to_fw_min_max_version.find(_pid);
        if (min_max_fw_it == ivcam2::device_to_fw_min_max_version.end())
            throw librealsense::invalid_value_exception(
                rsutils::string::from() << "Min and Max firmware versions have not been defined for this device: "
                                        << std::hex << _pid );

        // Limit L515 to FW versions within the 1.5.1.3-1.99.99.99 range to differenciate from the other products
        bool result = (firmware_version(fw_version) >= firmware_version(min_max_fw_it->second.first)) &&
            (firmware_version(fw_version) <= firmware_version(min_max_fw_it->second.second));
        if (!result)
            LOG_ERROR("Firmware version isn't compatible" << fw_version);

        return result;

    }

    notification l500_notification_decoder::decode(int value)
    {
        // Anything listed in l500-private.h on l500_fw_error_report is an error; everything else is a warning
        if (l500_fw_error_report.find(static_cast<uint8_t>(value)) != l500_fw_error_report.end())
            return{ RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR, value, RS2_LOG_SEVERITY_ERROR, l500_fw_error_report.at(static_cast<uint8_t>(value)) };

        return { RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR,
                 value,
                 RS2_LOG_SEVERITY_WARN,
                 ( rsutils::string::from() << "L500 HW report - unresolved type " << value ) };
    }
}
