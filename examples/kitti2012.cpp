/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <fstream>
#include <sstream>
#include <iomanip>

using std::istringstream;

#include <viso_stereo.h>
#include <png++/png.hpp>

using namespace std;

#include <filesystem>
using namespace std::filesystem;

vector<string> glob_image_from_dir(string dir)
{
    vector<string> filenames;
    for (const auto & entry : std::filesystem::directory_iterator(dir))
        filenames.push_back(entry.path().filename().string());

    std::sort(filenames.begin(), filenames.end(),
              [](const auto& lhs, const auto& rhs) {
                  return lhs < rhs;
              });
    for (const auto& file : filenames) {
        std::cout << file << '\n';
    }
}

vector<float> read_intrinsics(string file)
{
    std::ifstream infile(file);
    string line;
    vector<float> intrinsics;
    while(getline(infile, line))
    {

        //P2: 7.070912e+02 0.000000e+00 6.018873e+02 4.688783e+01 0.000000e+00 7.070912e+02 1.831104e+02 1.178601e-01 0.000000e+00 0.000000e+00 1.000000e+00 6.203223e-03
        if(line.find("P2") != string::npos)
        {
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            string values = line.substr(4);
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

            cout << intrinsics[0] << " " << intrinsics[1] << " " << intrinsics[2] << endl;

        }
    }

    infile.close();

    return intrinsics;
}

int main (int argc, char** argv) {

    if (argc<2) {
        // please manually generate pose_left folder at training and testing dir
        // Attention: keep an eye on the fail.txt file to check the failure case
        // ./kitti2012  /path/to/kitti2012/training/ 194
        // ./kitti2012  /path/to/kitti2012/testing/ 195
        cerr << "Usage: ./kitti2012 path/to/sequence/ num_of_image" << endl;
        return 1;
    }

    // sequence directory
    string dir = argv[1];
    string fail_file = dir + "fail.txt";
    ofstream fail_write(fail_file);

    int maxID = std::stoi(argv[2]);
    // record the fail case
    int failures = 0;
    for(int imgID=0; imgID < maxID; imgID++)
    {
        cout << "Processing: Image ID: " << imgID << endl;

        // set most important visual odometry parameters
        // for a full parameter list, look at: viso_stereo.h
        VisualOdometryStereo::parameters param;

        // write pose to txt file
        char extrinsic_file_name[256]; sprintf(extrinsic_file_name,"%06d.txt",imgID);
        string extrinsic_file = dir + "pose_left/" + extrinsic_file_name;
        ofstream file_write(extrinsic_file);
        file_write << std::fixed;

        // read intrinsics from txt file
        char intrinsic_file_name[256]; sprintf(intrinsic_file_name,"%06d.txt",imgID);
        string intrinsic_file = dir + "calib/" + intrinsic_file_name;

        vector<float> intrinsics = read_intrinsics(intrinsic_file);
        param.calib.f  = intrinsics[1]; // focal length in pixels
        param.calib.cu = intrinsics[2]; // principal point (u-coordinate) in pixels
        param.calib.cv = intrinsics[3]; // principal point (v-coordinate) in pixels
        param.base     = 0.54; // baseline in meters

        // init visual odometry
        VisualOdometryStereo viso(param);

        // current pose (this matrix transforms a point from the current
        // frame's camera coordinates to the first frame's camera coordinates)
        Matrix pose = Matrix::eye(4);

        // loop through all frames i=0:20
        for (int32_t i=0; i<21; i++) {

            // input file names
            char base_name[256]; sprintf(base_name,"%06d_%02d.png",imgID, i);
            string left_img_file_name  = dir + "image_2/" + base_name;
            string right_img_file_name = dir + "image_3/" + base_name;

            // catch image read/write errors here
            try {

                // load left and right input image
                png::image< png::gray_pixel > left_img(left_img_file_name);
                png::image< png::gray_pixel > right_img(right_img_file_name);

                // image dimensions
                int32_t width  = left_img.get_width();
                int32_t height = left_img.get_height();

                // convert input images to uint8_t buffer
                uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
                uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
                int32_t k=0;
                for (int32_t v=0; v<height; v++) {
                    for (int32_t u=0; u<width; u++) {
                        left_img_data[k]  = left_img.get_pixel(u,v);
                        right_img_data[k] = right_img.get_pixel(u,v);
                        k++;
                    }
                }

                // status
                cout << "Processing: Frame: " << i;

                // compute visual odometry
                int32_t dims[] = {width,height,width};
                if (viso.process(left_img_data,right_img_data,dims)) {

                    // returns transformation from previous to current coordinates as a 4x4
                    // homogeneous transformation matrix Tr_delta, with the following semantics:
                    // p_t = Tr_delta * p_ {t-1} takes a point in the camera coordinate system
                    // at time t_1 and maps it to the camera coordinate system at time t.
                    // note: getMotion() returns the last transformation even when process()
                    // has failed. this is useful if you wish to linearly extrapolate occasional
                    // frames for which no correspondences have been found
                    // on success, update current pose
                    pose = pose * Matrix::inv(viso.getMotion());

                    // output some statistics
                    double num_matches = viso.getNumberOfMatches();
                    double num_inliers = viso.getNumberOfInliers();
                    cout << ", Matches: " << num_matches;
                    cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
                    cout << pose << endl << endl;

                    // write pose to file
                    if(file_write.is_open())
                    {
                        char leading[256]; sprintf(leading,"%02d to %02d: ", i-1, i);
                        file_write << leading;
                        file_write << std::fixed <<  std::setprecision(9) <<
                                   pose.val[0][0] << " " << pose.val[0][1] << " " << pose.val[0][2] << " " << pose.val[0][3] << " " <<
                                   pose.val[1][0] << " " << pose.val[1][1] << " " << pose.val[1][2] << " " << pose.val[1][3] << " " <<
                                   pose.val[2][0] << " " << pose.val[2][1] << " " << pose.val[2][2] << " " << pose.val[2][3] << " " <<
                                   pose.val[3][0] << " " << pose.val[3][1] << " " << pose.val[3][2] << " " << pose.val[3][3] << endl;
                    }
                    else
                        cerr << "ERROR: Couldn't write pose to file!" << endl;
                } else {
                    cout << " ... failed!" << endl;
                    if(i>0)
                        failures++;

                    fail_write << "Processing image ID: " << imgID << ", Frame: " << i << " failed!" << endl;
                }

                // release uint8_t buffers
                free(left_img_data);
                free(right_img_data);

                // catch image read errors here
            } catch (...) {
                cout << "ERROR: Couldn't process image " << imgID << ": " << i << " !" << endl;
                fail_write << "ERROR: Couldn't process image " << imgID << ": " << i << ", not exist!" << endl;
                // return 1;
            }
        }

        file_write.close();
    }
    fail_write.close();
    // output
    cout << "Otherwise the first image, there are " << failures << " failed cases!" <<  endl;
    cout << "KITTI Pose Computation Complete! Exiting ..." << endl;

    // exit
    return 0;
}

