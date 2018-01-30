#ifndef KCF_HEADER
#define KCF_HEADER

#include <opencv2/opencv.hpp>
#include <vector>
#include "piotr_fhog/fhog.hpp"
#include "complexmat.hpp"

struct BBox_c
{
    double cx, cy, w, h;

    inline void scale(double factor){
        cx *= factor;
        cy *= factor;
        w  *= factor;
        h  *= factor;
    }
};

class KCF_Tracker
{
public:

    /*
    padding             ... extra area surrounding the target           (1.5)
    kernel_sigma        ... gaussian kernel bandwidth                   (0.5)
    lambda              ... regularization                              (1e-4)
    interp_factor       ... linear interpolation factor for adaptation  (0.02)
    output_sigma_factor ... spatial bandwidth (proportional to target)  (0.1)
    cell_size           ... hog cell size                               (4)
    */
    KCF_Tracker(double padding, double kernel_sigma, double lambda, double interp_factor, double output_sigma_factor, int cell_size) :
        p_padding(padding), p_output_sigma_factor(output_sigma_factor), p_kernel_sigma(kernel_sigma),
        p_lambda(lambda), p_interp_factor(interp_factor), p_cell_size(cell_size) {}
    KCF_Tracker() {}

    // Init/re-init methods
    void init(cv::Mat & img);
    void setTrackerPose(cv::Mat & img);
    void updateTrackerPosition(BBox_c & bbox);

    // frame-to-frame object tracking
    void track(cv::Mat & img);
    BBox_c getBBox();

    // kernel update
    void updateKernel(cv::Mat img);
    cv::Point2f getError();
    double correlation;

private:
    cv::Point2f error;

    BBox_c p_pose;
    bool p_resize_image = false;

    double p_padding = 1.5;
    double p_output_sigma_factor = 0.1;
    double p_output_sigma;
    double p_kernel_sigma = 0.5;    //def = 0.5
    double p_lambda = 1e-4;         //regularization in learning step
    double p_interp_factor = 0.02;  //def = 0.02, linear interpolation factor for adaptation
    int p_cell_size = 4;            //4 for hog (= bin_size)
    int p_windows_size[2];
    cv::Mat p_cos_window;

    FHoG p_fhog;                    //class encapsulating hog feature extraction

    //model
    ComplexMat p_yf;
    ComplexMat p_model_alphaf;
    ComplexMat p_model_alphaf_num;
    ComplexMat p_model_alphaf_den;
    ComplexMat p_model_xf;

    //helping functions
    cv::Mat get_subwindow(const cv::Mat & input, int cx, int cy, int size_x, int size_y);
    cv::Mat gaussian_shaped_labels(double sigma, int dim1, int dim2);
    ComplexMat gaussian_correlation(const ComplexMat & xf, const ComplexMat & yf, double sigma, bool auto_correlation = false);
    cv::Mat circshift(const cv::Mat & patch, int x_rot, int y_rot);
    cv::Mat cosine_window_function(int dim1, int dim2);
    ComplexMat fft2(const cv::Mat & input);
    ComplexMat fft2(const std::vector<cv::Mat> & input, const cv::Mat & cos_window);
    cv::Mat ifft2(const ComplexMat & inputf);

    //tests
    friend void run_tests(KCF_Tracker & tracker, const std::vector<bool> & tests);
};

#endif //KCF_HEADER
