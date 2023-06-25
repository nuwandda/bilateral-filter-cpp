#ifndef BILATERALFILTER_H
#define BILATERALFILTER_H
#include <opencv2/opencv.hpp>

class BilateralFilter {
public:

	/// Get gaussian kernel
	/// \param Kernel size for blur
	/// \param Sigma value
	/// \return output vector
	std::vector<double> get_gaussian_kernel(const int kernel_size, double sigma);

};

#endif // !BILATERALFILTER_H
