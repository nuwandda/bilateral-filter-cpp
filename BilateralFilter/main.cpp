#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "BilateralFilter.h"

void blur_gaussian(const cv::Mat& input, cv::Mat& output, const int kernel_size, const double sigma) {
    BilateralFilter filter;
    std::vector<double> kernel = filter.get_gaussian_kernel(kernel_size, sigma);

    CV_Assert(input.channels() == output.channels());

    unsigned char* data_in = (unsigned char*)(input.data);
    unsigned char* data_out = (unsigned char*)(output.data);

    for (int row = 0; row < input.rows; row++) {
        for (int col = 0; col < (input.cols * input.channels()); col += input.channels()) {
            for (int channel_index = 0; channel_index < input.channels(); channel_index++) {

                if (row <= kernel_size / 2 || row >= input.rows - kernel_size / 2 ||
                    (input.cols * input.channels()) <= kernel_size / 2 ||
                    col >= (input.cols * input.channels()) - kernel_size / 2) {
                    data_out[output.step * row + col + channel_index] = data_in[output.step * row + col + channel_index];
                    continue;
                }

                int k_ind = 0;
                double sum = 0;
                for (int k_row = -kernel_size / 2; k_row <= kernel_size / 2; ++k_row) {
                    for (int k_col = -kernel_size / 2; k_col <= kernel_size / 2; ++k_col) {
                        // The operation should be done on images with range [0,1] so we convert the pixel value
                        // from [0-255] to [0-1] before the operation.
                        sum += kernel[k_ind] * (data_in[input.step * (row + k_row) + col + (k_col * input.channels()) + channel_index] / 255.0);
                        k_ind++;
                    }
                }
                // Do not forget to convert back to [0-255] by multiplying the pixel value by 255.
                data_out[output.step * row + col + channel_index] = (unsigned int)(std::max(std::min(sum, 1.0), 0.0) * 255.0);
            }
        }
    }
}

int main(int argc, char** argv)
{
    /// Please change the path below
    cv::Mat frame = cv::imread("C:/Users/donme/Downloads/laden.jpg");
    cv::Mat gray_scaled;
    cv::Mat original;
    frame.copyTo(original);
    cv::cvtColor(frame, gray_scaled, cv::COLOR_BGR2GRAY);
    cv::imshow("BGR Original", frame);

    cv::Mat output_guassian_filter(original.rows, original.cols, CV_8UC1);
    cv::Mat output_guassian_filter_rgb(original.rows, original.cols, CV_8UC3);


    blur_gaussian(gray_scaled, output_guassian_filter, 15, 0);
    cv::imshow("15-Gray", output_guassian_filter);

    blur_gaussian(original, output_guassian_filter_rgb, 15, 0);
    cv::imshow("15-RGB", output_guassian_filter_rgb);

    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}