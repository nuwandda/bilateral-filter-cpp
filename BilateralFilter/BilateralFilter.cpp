#include <opencv2/opencv.hpp>
#define _USE_MATH_DEFINES
#include <math.h>
#include "BilateralFilter.h"

std::vector<double> BilateralFilter::get_gaussian_kernel(const int kernel_size, double sigma)
{
	std::setprecision(8);
	std::vector<double> kernel(kernel_size * kernel_size, 0);

	if (sigma <= 0) 
	{
		sigma = 0.3 * ((kernel_size - 1) * 0.5 - 1) + 0.8;
	}
	double r, s = 2.0 * sigma * sigma;

	/// Sum is for normalization
	double sum = 0.0;


    /// Generating nxn kernel
    int i, j;
    double mean = kernel_size / 2;
    for (i = 0; i < kernel_size; i++) {
        for (j = 0; j < kernel_size; j++) {
            kernel[(i * kernel_size) + j] = exp(-0.5 * (pow((i - mean) / sigma, 2.0) + pow((j - mean) / sigma, 2.0)))
                / (2 * M_PI * sigma * sigma);
            sum += kernel[(i * kernel_size) + j];
        }
    }

    // Normalising the Kernel
    for (int i = 0; i < kernel.size(); ++i) {
        kernel[i] /= sum;
    }

    return kernel;
}
