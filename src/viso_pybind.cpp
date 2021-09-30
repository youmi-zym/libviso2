//
// Created by yzhang on 9/25/21.
//
#include <pybind11/pybind11.h>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using std::istringstream;

#include <viso_stereo.h>

using namespace std;

vector<float> read_kitti_intrinsics(const string file)
{
    std::ifstream infile(file);
    string line;
    vector<float> intrinsics;
    while(getline(infile, line))
    {
        //K_02: 9.597910e+02 0.000000e+00 6.960217e+02 0.000000e+00 9.569251e+02 2.241806e+02 0.000000e+00 0.000000e+00 1.000000e+00
        if(line.find("K_02") != string::npos)
        {
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            string values = line.substr(6);
            istringstream ss(values);
            int cnt = 1;
            float value;
            while(ss >> value){
                if(cnt == 1) // fx
                    intrinsics.push_back(value);
                if(cnt == 3) // cx
                    intrinsics.push_back(value);
                if(cnt == 6) // cy
                {
                    intrinsics.push_back(value);
                    break;
                }
                cnt++;
            }
        }
    }

    infile.close();

    return intrinsics;
}


VisualOdometryStereo init_viso(vector<float> intrinsics, float baseline)
{

    // set most important visual odometry parameters
    // for a full parameter list, look at: viso_stereo.h
    VisualOdometryStereo::parameters param;
    param.calib.f  = intrinsics[1]; // focal length in pixels
    param.calib.cu = intrinsics[2]; // principal point (u-coordinate) in pixels
    param.calib.cv = intrinsics[3]; // principal point (v-coordinate) in pixels
    param.base     = baseline; // baseline in meters

    // init visual odometry
    VisualOdometryStereo viso(param);

    return viso;
}


// namespace py = pybind11;
// PYBIND11_MODULE(viso2, m) {
//     m.def("read_kitti_intrinsics", &read_kitti_intrinsics, "read kitti 2012/2015 intrinsic file", py::arg("file"));
// }

//py::class_<VisualOdometryStereo>(m, "VisualOdometryStereo")
//.def("getNumberOfMatches", &VisualOdometryStereo::getNumberOfMatches)
//.def("getNumberOfInliers", &VisualOdometryStereo::getNumberOfInliers);
// .def("getMotion", &VisualOdometryStereo::getMotion)
// .def("process", &VisualOdometryStereo::process)
// m.def("init_viso", &init_viso, "initialize VisualOdometryStereo", py::arg("intrinsics"), py::arg("baseline"));
