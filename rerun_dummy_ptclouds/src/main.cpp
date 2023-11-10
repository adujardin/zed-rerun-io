// Dummy example of ptclouds logging with rerun
// This sample code is a reproduction of a pretty extreme use case of stereo ptcloud comparison across a sequence
// In our test rerun handled it very well and the RAM consumption was reasonable considering the amount of data
// However the export into rrd led to a burst in RAM usage (a few times the amount used for the initial logging)


#include <rerun.hpp>
#include <rerun/demo_utils.hpp>
#include <thread>
#include <random>
#include <iostream>

using namespace std;
using namespace rerun::demo;

void run_cam(const rerun::RecordingStream* rec, int idx) {
    std::default_random_engine gen;

    float size = 50.f;
    std::uniform_real_distribution<float> dist(-size, size);

    auto full_size = 720 * 500;
    std::vector<rerun::Position3D> ptcloud_positions(full_size);
    std::vector<rerun::Color> ptcloud_colors(full_size, rerun::Color(0, idx * 50, 255 - idx * 50));

    int i = 0;
    // Main Loop
    while (i < 500) {
        rec->set_time_sequence("sequence_time", i++);

        /* zed.retrieveMeasure(point_cloud_cpu, MEASURE::XYZRGBA, MEM::CPU, res);

        sl::float4* p = point_cloud_cpu.getPtr<sl::float4>(sl::MEM::CPU);
        // Ptcloud format is: {x, y, z, color (packed)}

        for (int k = 0; k < full_size; k++) {
            auto pt = p[k];
            ptcloud_positions[k] = rerun::Position3D(pt.x, pt.y, pt.z);
            unsigned char *p_uc = (unsigned char*) &pt.w;
            ptcloud_colors[k] = rerun::Color(p_uc[0], p_uc[1], p_uc[2]);
        }*/

        std::generate(ptcloud_positions.begin(), ptcloud_positions.end(), [&] {
            return rerun::Position3D(dist(gen) + idx * size, dist(gen), dist(gen)); });

        rec->log(
                "ptcloud_" + std::to_string(idx),
                rerun::Points3D(ptcloud_positions).with_colors(ptcloud_colors).with_radii({1.2f})
        );

        /*if (mode == DEPTH_MODE::NEURAL) {

            zed.retrieveImage(img, VIEW::LEFT, MEM::CPU, res);
            img_vec.assign(img.getPtr<sl::uchar1>(), img.getPtr<sl::uchar1>() + full_size * 4);

            rec->log("left", rerun::Image({static_cast<size_t> (res.height),
                static_cast<size_t> (res.width),
                static_cast<size_t> (4)},
            std::move(img_vec)));
        }*/
    }

    // close the ZED
    //zed.close();
}

int main(int argc, char **argv) {

    const auto rec = rerun::RecordingStream("rerun_zed_ptcloud");
    rec.spawn().exit_on_failure();


    int idx = 0;
    std::thread t(run_cam, &rec, idx++);
    std::thread t1(run_cam, &rec, idx++);
    std::thread t2(run_cam, &rec, idx++);
    std::thread t3(run_cam, &rec, idx++);
    std::thread t4(run_cam, &rec, idx++);


    t.join();
    t1.join();
    t2.join();
    t3.join();
    t4.join();


    return EXIT_SUCCESS;
}
