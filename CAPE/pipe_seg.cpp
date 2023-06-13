/*
 * Copyright 2018 Pedro Proenza <p.proenca@surrey.ac.uk> (University of Surrey)
 *
 */

#include <iostream>
#include <cstdio>
#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "CAPE.h"
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;

bool done = false;
float COS_ANGLE_MAX = cos(M_PI/12);
float MAX_MERGE_DIST = 50.0f;
bool cylinder_detection= true;
CAPE * plane_detector;
std::vector<cv::Vec3b> color_code;

// set rgb image intrinsic
float fx_rgb = 604.3303833007812;
float fy_rgb = 604.4821166992188;
float cx_rgb = 638.0137329101562;
float cy_rgb = 363.1947937011719;

// set the image width and height
int width = 1280;
int height = 720;

// set the patch size
int PATCH_SIZE = 20;

// compute the number of horizontal and vertical cells
int nr_horizontal_cells = width/PATCH_SIZE;
int nr_vertical_cells = height/PATCH_SIZE;


void projectPointCloud(const cv::Mat & X, const cv::Mat & Y, const cv::Mat & Z, const double &z_min, Eigen::MatrixXf & cloud_array){
    // X: x coordinate of pcd
    // Y: y coordinate of pcd
    // Z: z coordinate of pcd
    // organize as a image, each pixel has a value
    int width = X.cols;
    int height = X.rows;
    
    cv::Mat U, V;
    // Project to image coordinates
    cv::divide(X,Z,U,1);    // U=X*1/Z
    cv::divide(Y,Z,V,1);    // V=Y*1/Z (Performs per-element division of two arrays or a scalar by an array.)

    // project the pcd to image plane
    // because the same position may not have depth value
    U = U * fx_rgb + cx_rgb;
    V = V * fy_rgb + cy_rgb;
    // Reusing U as cloud index
    //U = V*width + U + 0.5;

    float z, u, v;
    int id;
    
    for(int r=0; r< height; r++){
        for(int c=0; c< width; c++){
            z = Z.ptr<float>(r)[c];
            u = U.ptr<float>(r)[c];
            v = V.ptr<float>(r)[c];
            if(z>z_min && u>0 && v>0 && u<width && v<height){
                // std::floor(5.88) = 5
                id = floor(v)*width + u;
                cloud_array(id,0) = X.ptr<float>(r)[c];
                cloud_array(id,1) = Y.ptr<float>(r)[c];
                cloud_array(id,2) = Z.ptr<float>(r)[c];
            }
        }
    }
}


void organizePointCloudByCell(const Eigen::MatrixXf & cloud_in, const cv::Mat & cell_map, Eigen::MatrixXf & cloud_out){

    int width = cell_map.cols;
    int height = cell_map.rows;
    int mxn = width*height;
    int mxn2 = 2*mxn;

    int id, it(0);

    for(int r=0; r< height; r++){
        for(int c=0; c< width; c++){
            id = cell_map.ptr<int>(r)[c];
            *(cloud_out.data() + id) = *(cloud_in.data() + it);
            *(cloud_out.data() + mxn + id) = *(cloud_in.data() + mxn + it);
            *(cloud_out.data() + mxn2 + id) = *(cloud_in.data() + mxn2 + it);
            it++;
        }
    }
}


void read_tum_dataset(const std::string &assocPath, std::vector<std::string> &filesColor, std::vector<std::string> &filesDepth, std::vector<std::string> &time_stamp_depth)
{
    std::ifstream assocIn;
    assocIn.open(assocPath.c_str());
    std::string line;
    while (std::getline(assocIn, line))
    {
        if (line.empty() || line.compare(0, 1, "#") == 0)
            continue;
        std::istringstream iss(line);
        std::string timestampDepth, timestampColor;
        std::string fileDepth, fileColor;
        if (!(iss >> timestampColor >> fileColor >> timestampDepth >> fileDepth))
            break;

        filesDepth.push_back(fileDepth);
        filesColor.push_back(fileColor);
        time_stamp_depth.push_back(timestampDepth);
    }
    assocIn.close();
}


void gen_random_color()
{
    // For visualize the final results
    for(int i=0; i<100;i++){
        cv::Vec3b color;
        color[0]=rand()%255;
        color[1]=rand()%255;
        color[2]=rand()%255;
        color_code.push_back(color);
    }

    // Add specific colors for planes
    color_code[0][0] = 0; color_code[0][1] = 0; color_code[0][2] = 255;
    color_code[1][0] = 255; color_code[1][1] = 0; color_code[1][2] = 204;
    color_code[2][0] = 255; color_code[2][1] = 100; color_code[2][2] = 0;
    color_code[3][0] = 0; color_code[3][1] = 153; color_code[3][2] = 255;
    // Add specific colors for cylinders
    color_code[50][0] = 178; color_code[50][1] = 255; color_code[50][2] = 0;
    color_code[51][0] = 255; color_code[51][1] = 0; color_code[51][2] = 51;
    color_code[52][0] = 0; color_code[52][1] = 255; color_code[52][2] = 51;
    color_code[53][0] = 153; color_code[53][1] = 0; color_code[53][2] = 255;
}


int main(int argc, char ** argv){
    // step 1: set the tum dataset path
    std::string assocPath = "/home/zhy/windows_disk/datasets/pipeline_light_datasets_0315/6/associations.txt";
    std::string dataset_root = "/home/zhy/windows_disk/datasets/pipeline_light_datasets_0315/6";
    
    // step 2: read the tum dataset
    std::vector<std::string> filesDepth, filesColor, time_stamp_depth;
    read_tum_dataset(assocPath, filesColor, filesDepth, time_stamp_depth);

    // Pre-computations for backprojection
    cv::Mat_<float> X_pre(height, width);
    cv::Mat_<float> Y_pre(height, width);

    // backprojection model: x=[(u-cx)/fx] * z
    // here X_pre is [(u-cx)/fx]. if know depth value at (u_m,v_m), the X coordinate matrix X = X_pre * depth_image.
    // acclerate the computation
    for (int r = 0;r < height; r ++){
        for (int c = 0;c < width; c ++){
            // Not efficient but at this stage doesn t matter
            X_pre.at<float>(r,c) = (c-cx_rgb)/fx_rgb;
            Y_pre.at<float>(r,c) = (r-cy_rgb)/fy_rgb;
        }
    }

    // Pre-computations for maping an image point cloud to a cache-friendly array where cell's local point clouds are contiguous
    cv::Mat_<int> cell_map(height, width);

    for (int r=0;r<height; r++){
        // segment height into cell_r's cell with size PATCH_SIZE
        // 取整操作
        int cell_r = r/PATCH_SIZE;
        // remaining block size. because 640%30 may not be a int value.
        // 取余数操作
        int local_r = r%PATCH_SIZE;
        
        for (int c=0;c<width; c++){
            // segment height into cell_r's cell with size PATCH_SIZE
            int cell_c = c/PATCH_SIZE;
            int local_c = c%PATCH_SIZE;

            // 什么意思 ???? 
            cell_map.at<int>(r,c) = (cell_r * nr_horizontal_cells + cell_c) * PATCH_SIZE * PATCH_SIZE 
                                    + local_r*PATCH_SIZE 
                                    + local_c;
        }
    }

    cv::Mat_<float> X(height,width);
    cv::Mat_<float> Y(height,width);

    Eigen::MatrixXf cloud_array(width*height,3);
    Eigen::MatrixXf cloud_array_organized(width*height,3);


    // Populate with random color codes
    gen_random_color();

    // Initialize CAPE
    plane_detector = new CAPE(height, width, PATCH_SIZE, PATCH_SIZE, cylinder_detection, COS_ANGLE_MAX, MAX_MERGE_DIST);
    
    // visualize the results
    cv::namedWindow("Seg");

    // run the cape in tum dataset
    for(int i = 0; i < filesColor.size(); i++)
    {
        std::string rgb_path = dataset_root + "/" + filesColor[i];
        std::string dep_path = dataset_root + "/" + filesDepth[i];
        cv::Mat rgb_img = cv::imread(rgb_path, cv::IMREAD_COLOR);
        cv::Mat d_img = cv::imread(dep_path, cv::IMREAD_ANYDEPTH);

        d_img.convertTo(d_img, CV_32F);

        // check the exist of the image
        if ( rgb_img.empty() || d_img.empty() )
        {
            std::cout<<"no image founded" << std::endl;
            break;
        }
        
        // Backproject to point cloud
        // pcd: (x_i, y_i, z_i), i \in {1,2,...N}
        X = X_pre.mul(d_img);    // x coordinate of pcd
        Y = Y_pre.mul(d_img);    // y coordinate of pcd
        cloud_array.setZero();    // set zero to process next frame
        
        // back-project to 3D space and generate cloud_array
        projectPointCloud(X, Y, d_img, 0, cloud_array);

        cv::Mat_<cv::Vec3b> seg_rz = cv::Mat_<cv::Vec3b>(height,width,cv::Vec3b(0,0,0));
        cv::Mat_<uchar> seg_output = cv::Mat_<uchar>(height,width,uchar(0));

        // Run CAPE
        int nr_planes, nr_cylinders;
        vector<PlaneSeg> plane_params;
        vector<CylinderSeg> cylinder_params;
        double t1 = cv::getTickCount();

        // ???
        organizePointCloudByCell(cloud_array, cell_map, cloud_array_organized);
        plane_detector->process(cloud_array_organized, nr_planes, nr_cylinders, seg_output, plane_params, cylinder_params);
        
        double t2 = cv::getTickCount();
        double time_elapsed = (t2-t1)/(double)cv::getTickFrequency();
        // cout<<"Total time elapsed: "<<time_elapsed<<endl;

        // Map segments with color codes and overlap segmented image w/ RGB
        uchar * sCode;
        uchar * dColor;
        uchar * srgb;
        int code;
        for(int r=0; r<  height; r++){
            dColor = seg_rz.ptr<uchar>(r);
            sCode = seg_output.ptr<uchar>(r);
            srgb = rgb_img.ptr<uchar>(r);
            for(int c=0; c< width; c++){
                code = *sCode;
                if (code>0){
                    dColor[c*3] =   color_code[code-1][0]/2 + srgb[0]/2;
                    dColor[c*3+1] = color_code[code-1][1]/2 + srgb[1]/2;
                    dColor[c*3+2] = color_code[code-1][2]/2 + srgb[2]/2;;
                }else{
                    dColor[c*3] =  srgb[0];
                    dColor[c*3+1] = srgb[1];
                    dColor[c*3+2] = srgb[2];
                }
                sCode++; srgb++; srgb++; srgb++;
            }
        }

        // Show frame rate and labels
        cv::rectangle(seg_rz,  cv::Point(0,0),cv::Point(width,20), cv::Scalar(0,0,0),-1);
        std::stringstream fps;
        fps<<(int)(1/time_elapsed+0.5)<<" fps";
        cv::putText(seg_rz, fps.str(), cv::Point(15,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));
        cout<<"Nr cylinders:"<<nr_cylinders<<endl;
        int cylinder_code_offset = 50;
        // show cylinder labels
        if (nr_cylinders>0){
            std::stringstream text;
            text<<"Cylinders:";
            cv::putText(seg_rz, text.str(), cv::Point(width/2,15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,1));
            for(int j=0;j<nr_cylinders;j++){
                cv::rectangle(seg_rz,  cv::Point(width/2 + 80+15*j,6),cv::Point(width/2 + 90+15*j,16), cv::Scalar(color_code[cylinder_code_offset+j][0],color_code[cylinder_code_offset+j][1],color_code[cylinder_code_offset+j][2]),-1);
            }
        }
        cv::imshow("Seg", seg_rz);

        // save the result
        stringstream ss;
        ss << setw(5) << setfill('0') << i;
        std::string img_num = ss.str();
        cv::imwrite("../results/"+ img_num + ".jpg", seg_rz);
        cv::waitKey(1);
    }
    return 0;
}

