#ifndef _COMMON_MARKER_DETECTION_
#define _COMMON_MARKER_DETECTION_

//C++ Libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Core>
using namespace Eigen;

typedef struct t_marker{ //tracked marker structure
    const char* name;
    MatrixXd T;
    int imx0,imy0,imxf,imyf;
    int imx0_LED,imy0_LED,imxf_LED,imyf_LED;
    int state;
    int frequencies[10];
    int count;
    double filter_count,max;
    int current_freq;
    int failures;
    double t_old;
    double old_size_local;
}marker;

std::vector<marker>* get_markers();
void initialize_markers(std::string param_name);
int detect_markers(int no);
int track_markers(unsigned char* buf,unsigned int step, int width, int height);

#endif