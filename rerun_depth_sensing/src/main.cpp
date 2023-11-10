///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*********************************************************************
 ** This sample demonstrates how to capture a live 3D point cloud   **
 ** with the ZED SDK and display the result in an OpenGL window.    **
 *********************************************************************/

// ZED includes
#include <sl/Camera.hpp>
#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

// Sample includes
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;
using namespace rerun::demo;

void parseArgs(int argc, char **argv, sl::InitParameters& param);

int main(int argc, char **argv) {

    const auto rec = rerun::RecordingStream("rerun_zed_ptcloud");
    rec.spawn().exit_on_failure();

    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::NEURAL;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    init_parameters.sdk_verbose = 1;
    init_parameters.coordinate_units = UNIT::CENTIMETER;
    parseArgs(argc, argv, init_parameters);

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    auto camera_config = zed.getCameraInformation().camera_configuration;
    float image_aspect_ratio = camera_config.resolution.width / (1.f * camera_config.resolution.height);
    int requested_low_res_w = min(720, (int) camera_config.resolution.width);
    sl::Resolution res(requested_low_res_w, requested_low_res_w / image_aspect_ratio);


    RuntimeParameters runParameters;
    // Setting the depth confidence parameters
    runParameters.confidence_threshold = 50;
    runParameters.texture_confidence_threshold = 100;

    // Allocation of 4 channels of float on GPU
    Mat point_cloud(res, MAT_TYPE::F32_C4, sl::MEM::GPU);
    Mat point_cloud_cpu(res, MAT_TYPE::F32_C4, sl::MEM::CPU);
    std::cout << "Press on 's' for saving current .ply file" << std::endl;


    auto full_size = point_cloud.getResolution().area();

    std::vector<rerun::Position3D> ptcloud_positions(full_size);
    std::vector<rerun::Color> ptcloud_colors(full_size);

    int i = 0;

    sl::Mat img;
    std::vector<uint8_t> img_vec(full_size * 4);

    // Main Loop
    while (1) {
        // Check that a new image is successfully acquired
        if (zed.grab(runParameters) == ERROR_CODE::SUCCESS) {
            auto ts = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
            rec.set_time_sequence("sequence_time", i++);
            rec.set_time_nanos("stable_time", ts);

            zed.retrieveMeasure(point_cloud_cpu, MEASURE::XYZRGBA, MEM::CPU, res);

            sl::float4* p = point_cloud_cpu.getPtr<sl::float4>(sl::MEM::CPU);
            // Ptcloud format is: {x, y, z, color (packed)}
            
            for (int k = 0; k < full_size; k++) {
                auto pt = p[k];
                ptcloud_positions[k] = rerun::Position3D(pt.x, pt.y, pt.z);
                unsigned char *p_uc = (unsigned char*) &pt.w;
                ptcloud_colors[k] = rerun::Color(p_uc[0], p_uc[1], p_uc[2]);
            }

            rec.log(
                    "zed/ptcloud_neural",
                    rerun::Points3D(ptcloud_positions).with_colors(ptcloud_colors).with_radii({0.6f})
            );


            // Quick and dirty image (TODO: conversion to RGB, current buffer is BGRA)
            zed.retrieveImage(img, VIEW::LEFT, MEM::CPU, res);
            img_vec.assign(img.getPtr<sl::uchar1>(), img.getPtr<sl::uchar1>() + full_size * 4);

            rec.log("zed/left", rerun::Image({static_cast<size_t> (res.height),
                    static_cast<size_t> (res.width),
                    static_cast<size_t> (4)},
                std::move(img_vec)));
        }
    }
    
    // close the ZED
    zed.close();

    return EXIT_SUCCESS;
}

void parseArgs(int argc, char **argv, sl::InitParameters& param) {
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    } else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        } else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        } else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        } else if (arg.find("HD1200") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1200;
            cout << "[Sample] Using Camera in resolution HD1200" << endl;
        } else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        } else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        } else if (arg.find("SVGA") != string::npos) {
            param.camera_resolution = RESOLUTION::SVGA;
            cout << "[Sample] Using Camera in resolution SVGA" << endl;
        } else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    } else {
        // Default
    }
}
