#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    // Lệch có chủ đích so với model lúc train, để sim dùng nghiệm thu được.
    // Mặc định = không lệch -> hành vi y hệt trước.
    int sim2real_delay_ms = 0;         // trễ lệnh xuống motor (ms)
    double sim2real_gain_scale = 1.0;  // gains motor thật / gains gửi xuống
    double sim2real_mass_scale = 1.0;  // khối lượng + quán tính thật / theo CAD

    // Camera recorder: render trực tiếp framebuffer MuJoCo, không quay desktop.
    std::filesystem::path record_path;
    int record_width = 1280;
    int record_height = 720;
    int record_fps = 30;
    double record_camera_distance = 5.0;
    double record_camera_azimuth = 135.0;
    double record_camera_elevation = -15.0;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }

        // Tuỳ chọn — config.yaml cũ không có mấy khoá này vẫn chạy bình thường.
        if (cfg["sim2real_delay_ms"])
            sim2real_delay_ms = cfg["sim2real_delay_ms"].as<int>();
        if (cfg["sim2real_gain_scale"])
            sim2real_gain_scale = cfg["sim2real_gain_scale"].as<double>();
        if (cfg["sim2real_mass_scale"])
            sim2real_mass_scale = cfg["sim2real_mass_scale"].as<double>();
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Unitree Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
        ("record", po::value<std::filesystem::path>(&config.record_path),
            "Record the MuJoCo camera directly to MP4")
        ("record-width", po::value<int>(&config.record_width), "Recorder width")
        ("record-height", po::value<int>(&config.record_height), "Recorder height")
        ("record-fps", po::value<int>(&config.record_fps), "Recorder frame rate")
        ("record-camera-distance", po::value<double>(&config.record_camera_distance),
            "Tracking camera distance")
        ("record-camera-azimuth", po::value<double>(&config.record_camera_azimuth),
            "Tracking camera azimuth in degrees")
        ("record-camera-elevation", po::value<double>(&config.record_camera_elevation),
            "Tracking camera elevation in degrees")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}
