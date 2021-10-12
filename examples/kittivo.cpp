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
#include <cstdint>
#include <fstream>
#include <sstream>
#include <iomanip>

using std::istringstream;

#include <viso_stereo.h>
#include <png++/png.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#include <filesystem>
using namespace std::filesystem;

vector<string> glob_item_from_dir(string dir)
{
    vector<string> filenames;
    for (const auto & entry : std::filesystem::directory_iterator(dir))
        filenames.push_back(entry.path().filename().string());

    std::sort(filenames.begin(), filenames.end(),
              [](const auto& lhs, const auto& rhs) {
                  return lhs < rhs;
              });
    /*for (const auto& file : filenames) {
        std::cout << file << '\n';
    }*/

    return filenames;
}

vector<float> read_intrinsics(string file)
{
    std::ifstream infile(file);
    string line;
    vector<float> intrinsics;
    while(getline(infile, line))
    {
        // P2:  7.188560000000e+02 0.000000000000e+00 6.071928000000e+02 4.538225000000e+01
        //      0.000000000000e+00 7.188560000000e+02 1.852157000000e+02 -1.130887000000e-01
        //      0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 3.779761000000e-03
        // P = K @ T, K is intrinsic matrix, and T is translation matrix
        if(line.find("P2") != string::npos)
        {
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            string values = line.substr(4);
            istringstream ss(values);
            int cnt = 1;
            int num_column = 4;
            float value;
            while(ss >> value){
                if(cnt == 1) // fx
                    intrinsics.push_back(value);
                if(cnt == 3) // cx
                    intrinsics.push_back(value);
                if(cnt == num_column + 3) // cy
                {
                    intrinsics.push_back(value);
                    break;
                }
                cnt++;
            }

            // cout << intrinsics[0] << " " << intrinsics[1] << " " << intrinsics[2] << endl;

        }
    }

    infile.close();

    return intrinsics;
}

int main (int argc, char** argv) {

    if (argc<2) {
        // please manually generate pose_left folder at training and testing dir
        // Attention: keep an eye on the fail.txt file to check the failure case
        // ./kittivo path/to/kittivo/
        cerr << "Usage: ./kittivo path/to/sequence/" << endl;
        return 1;
    }

    // sequence directory
    string dir = argv[1];
    string fail_dir = dir + "viso2/fail_record/";

    vector<string> time_sequences = glob_item_from_dir(dir + "sequences/");

    for (const auto& time_sequence : time_sequences) {
        cout << "Processing: time sequence: " << time_sequence << endl;

        // record the fail case
        int failures = 0;
        string fail_file = fail_dir + time_sequence + ".txt";
        ofstream fail_write(fail_file);

        // set most important visual odometry parameters
        // for a full parameter list, look at: viso_stereo.h
        VisualOdometryStereo::parameters param;

        // write pose to txt file
        string extrinsic_file = dir + "viso2/" + time_sequence + ".txt";
        ofstream file_write(extrinsic_file);
        file_write << std::fixed;

        // read intrinsics from txt file
        string intrinsic_file = dir + "sequences/" + time_sequence + "/" +  + "calib.txt";

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

        vector<string> images = glob_item_from_dir(dir + "sequences/" + time_sequence + "/" + "image_2/");

        bool is_first_image = true;
        for (const auto& base_name: images){

            // input file names
            string left_img_file_name  = dir + "sequences/" + time_sequence + "/" + "image_2/" + base_name;
            string right_img_file_name = dir + "sequences/" + time_sequence + "/" + "image_3/" + base_name;

            // catch image read/write errors here
            try {

                // load left and right input image
                Mat left_img = imread(left_img_file_name, IMREAD_UNCHANGED);
                left_img.convertTo(left_img, CV_8UC1);
                Mat right_img = imread(right_img_file_name, IMREAD_UNCHANGED);
                right_img.convertTo(right_img, CV_8UC1);

                // png::image< png::gray_pixel > left_img(left_img_file_name);
                // png::image< png::gray_pixel > right_img(right_img_file_name);

                // image dimensions
                int32_t width  = left_img.cols;
                int32_t height = left_img.rows;

                // convert input images to uint8_t buffer
                uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
                uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
                int32_t k=0;
                for (int32_t v=0; v<height; v++) {
                    for (int32_t u=0; u<width; u++) {
                        left_img_data[k]  = left_img.at<uint8_t>(v, u);
                        right_img_data[k] = right_img.at<uint8_t>(v, u);
                        k++;
                    }
                }

                // status
                cout << "Processing: Frame: " << base_name;

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
                    // pose = pose * viso.getMotion();

                    // output some statistics
                    double num_matches = viso.getNumberOfMatches();
                    double num_inliers = viso.getNumberOfInliers();
                    cout << ", Matches: " << num_matches;
                    cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
                    cout << pose << endl << endl;

                    // write pose to file
                    if(file_write.is_open())
                    {
                        string leading = base_name.substr(0, 6) + " ";
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
                    // write pose of previous frame to file
                    if(file_write.is_open())
                    {
                        string leading = base_name.substr(0, 6) + " ";
                        file_write << leading;
                        file_write << std::fixed <<  std::setprecision(9) <<
                                   pose.val[0][0] << " " << pose.val[0][1] << " " << pose.val[0][2] << " " << pose.val[0][3] << " " <<
                                   pose.val[1][0] << " " << pose.val[1][1] << " " << pose.val[1][2] << " " << pose.val[1][3] << " " <<
                                   pose.val[2][0] << " " << pose.val[2][1] << " " << pose.val[2][2] << " " << pose.val[2][3] << " " <<
                                   pose.val[3][0] << " " << pose.val[3][1] << " " << pose.val[3][2] << " " << pose.val[3][3] << endl;
                    }
                    else
                        cerr << "ERROR: Couldn't write pose to file!" << endl;

                    if(!is_first_image)
                        failures++;

                    fail_write << "Processing frame: " << base_name << " failed!" << endl;
                }

                // release uint8_t buffers
                free(left_img_data);
                free(right_img_data);

                // catch image read errors here
            } catch (...) {
                cout << "ERROR: Couldn't process image " << time_sequence << ": " << base_name << " !" << endl;
                fail_write << "ERROR: Couldn't process image: " << base_name << ", not exist!" << endl;
                // return 1;
            }

            is_first_image = false;
        }
        cout << "For sequence: " << time_sequence << endl;
        cout << "Otherwise the first image, there are " << failures << " failed cases!" <<  endl;
        cout << endl << endl << endl << endl;
        file_write.close();
        fail_write.close();
    }
    // output
    cout << "DrivingStereo Pose Computation Complete! Exiting ..." << endl;

    // exit
    return 0;
}

