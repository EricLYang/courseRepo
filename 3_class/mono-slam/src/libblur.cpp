/*
 * libblur.cpp
 *
 *  Created on: Jul 9, 2013
 *      Author: ludovico
 */
#include "libblur.h"
#include <math.h>

// Input: parametri del kernel (parametri predetti da kalman)
// Output: kernel stimato

// This function is to evaluate the kernel
// (the smallest one possible inferable from EKF)
// knowing initial and final point after camera shake

cv::Mat evaluateKernel(cv::Point2f one, cv::Point2f two) {
	int width, height;

    height  = abs(one.x - two.x ) + 1;
	width = abs(one.y - two.y) + 1;

	cv::Mat kernel = cv::Mat::zeros(width, height, cv::DataType<double>::type);
	double length, theta, value;
	theta = atan2((one.y - two.y),(one.x - two.x));

	length = norm(one - two);
    double c = cos(theta);
    double s = sin(theta);

    int x0 = (s < 0 ? -s*length: 0);
    int y0 = (c < 0 ? -c*length: 0);


	for(int i=0; i<length; ++i) {
        int x = i*s + x0;
        int y = i*c + y0;
		kernel.at<double>(x,y) = 1;
    }
    //std::cout << "Kernel Built" <<  std::endl;
    kernel = kernel/sum(kernel).val[0];


   // //std::cout << kernel <<  std::endl;

	return kernel.clone();
}


// Input: patch non blurrata + parametri del kernel (parametri predetti da kalman)
// Output: patch blurrata

// This function is to blur the patch with the predicted kernel
// (the smallest one possible inferable from EKF)

cv::Mat blurPatch(const cv::Mat &_patch, cv::Point2f one, cv::Point2f two) {
    cv::Mat blurredWindow = _patch.clone();
    //std::cout << "Start Blurring" << std::endl;

    cv::Mat patch = _patch.clone();
    patch.convertTo(patch, cv::DataType<double>::type);

	cv::Mat kernel = evaluateKernel(one,two);

	cv::Point2f anchor = cv::Point( -1, -1 );

	int delta = 0, ddepth = -1;
	filter2D(patch, patch, ddepth, kernel, anchor, delta, cv::BORDER_DEFAULT);

    patch.convertTo(patch, CV_8U);
    //std::cout << "Stop Blurring" << std::endl;

  //  hconcat(blurredWindow, patch, blurredWindow);
  //  imshow("Blurring", blurredWindow);
  //  cv::waitKey(1);

	return patch;
}



// Input: nuova patch blurrata + parametri del kernel (parametri stimati)
// Output: patch deblurrata

// This function is to deblur the patch with the estimated kernel
// LR deconvolution is used in the spatial domain - Improved algorithm
// (see Fish et al. - Blind deconvolution by means of the RichardsonñLucy algorithm )

// Input kernel is supposed to be normalised (sum of its element equal to 1)

// If the deblurring process is not giving high quality outcomes, try to
// augment the number of iterations or to pre-process the kernel taping its edges
// (using http://read.pudn.com/downloads112/sourcecode/graph/texture_mapping/464953/images/edgetaper.m__.htm )

cv::Mat deblurPatch(const cv::Mat &_patch, cv::Point2f one, cv::Point2f two) {
    ////std::cout << "Start Deblurring" << one << std::endl << two << std::endl;
    cv::Mat blurredWindow = _patch.clone();
    cv::Mat patch = _patch.clone();
    patch.convertTo(patch, cv::DataType<double>::type);
    cv::Mat kernel = evaluateKernel(one,two);
//    //std::cout << " kernel" << kernel << std::endl;
	cv::Mat kernel_hat = kernel.clone(), result = patch.clone();
	cv::Mat est_conv = result.clone(), relative_blur = patch.clone(), error_est = patch.clone();
	cv::Mat patch_hat = patch.clone(), k_est_conv = kernel.clone(), k_relative_blur = kernel.clone(), k_error_est = kernel.clone();

	int rows = kernel.rows, cols = kernel.cols, maxIter = 3;
	int pRows = patch.rows, pCols = patch.cols;
	cv::Point2f anchor = cv::Point( -1, -1 );
	int delta = 0, ddepth = -1;

    double eps = DBL_MIN;
	for(int l=0; l<maxIter; l++) {

		//Improvement
		for(int i=0; i<pRows; i++)
			for(int j=0; j<pCols; j++)
				patch_hat.at<double>(i,j) = patch.at<double>(pRows-i-1,pCols-j-1);
		filter2D(kernel, k_est_conv, ddepth, patch, anchor, delta, cv::BORDER_DEFAULT);
		for(int i=0; i<rows; i++)
			for(int j=0; j<cols; j++) {
                //if(est_conv.at<double>(i,j) < eps)
				//	est_conv.at<double>(i,j) = 10*eps;
				k_relative_blur.at<double>(i,j) = kernel.at<double>(i,j) / k_est_conv.at<double>(i,j);
            }
		filter2D(k_relative_blur, k_error_est, ddepth, patch_hat);
		for(int i=0; i<rows; i++)
			for(int j=0; j<cols; j++) {
				kernel.at<double>(i,j) = kernel.at<double>(i,j) * k_error_est.at<double>(i,j);
            }

		// Classical LR
		for(int i=0; i<rows; i++)
			for(int j=0; j<cols; j++)
				kernel_hat.at<double>(i,j) = kernel.at<double>(rows-i-1,cols-j-1);
		filter2D(result, est_conv, ddepth, kernel);
		for(int i=0; i<pRows; i++)
			for(int j=0; j<pCols; j++) {
                //if(est_conv.at<double>(i,j) < eps)
				//	est_conv.at<double>(i,j) = 10*eps;
				relative_blur.at<double>(i,j) = patch.at<double>(i,j) / est_conv.at<double>(i,j);

            }
		filter2D(relative_blur, error_est, ddepth, kernel_hat, anchor, delta, cv::BORDER_DEFAULT);
		for(int i=0; i<pRows; i++)
			for(int j=0; j<pCols; j++) {
				result.at<double>(i,j) = result.at<double>(i,j) * error_est.at<double>(i,j);
            }
	}



    result.convertTo(result, CV_8U);

    hconcat(blurredWindow, result, blurredWindow);
    imshow("Deblurring", blurredWindow);


    //std::cout << "Stop Deblurring" << std::endl;

	return result;
}



/*

// Input: nuova patch blurrata + parametri del kernel (parametri stimati)
// Output: patch deblurrata

// This function is to deblur the patch with the estimated kernel
// LR deconvolution is used in the spatial domain
// (for a quick reference see http://en.wikipedia.org/wiki/RichardsonñLucy_deconvolution )

// Input kernel is supposed to be normalised (sum of its element equal to 1)

// If the deblurring process is not giving high quality outcomes, try to
// augment the number of iterations or to pre-process the kernel taping its edges
// (using http://read.pudn.com/downloads112/sourcecode/graph/texture_mapping/464953/images/edgetaper.m__.htm )

cv::Mat deblurPatch(cv::Mat patch, cv::Point2f one, cv::Point2f two) {
    //std::cout << "Start Deblurring" << one << std::endl << two << std::endl;

    imshow("predublurring", patch);

    patch.convertTo(patch, cv::DataType<double>::type);
    patch = patch/255;//(patch.rows*patch.cols*cv::mean(patch).val[0]);

    cv::Mat kernel = evaluateKernel(one,two);
    //std::cout << " kernel" << kernel << std::endl;
	cv::Mat kernel_hat = kernel.clone(), result = patch.clone();
	cv::Mat est_conv = result.clone(), relative_blur = patch.clone(), error_est = patch.clone();

	int rows = kernel.rows, cols = kernel.cols, maxIter = 30;
	int pRows = patch.rows, pCols = patch.cols;
	cv::Point2f anchor = cv::Point( -1, -1 );
	int delta = 0, ddepth = -1;

	for(int i=0; i<rows; i++)
		for(int j=0; j<cols; j++)
			kernel_hat.at<double>(i,j) = kernel.at<double>(rows-i-1,cols-j-1);

    double eps = DBL_MIN;
	for(int l=0; l<maxIter; l++) {
		filter2D(result, est_conv, ddepth, kernel, anchor, delta, cv::BORDER_DEFAULT);
		for(int i=0; i<pRows; i++)
			for(int j=0; j<pCols; j++) {
                if(est_conv.at<double>(i,j) < eps) est_conv.at<double>(i,j) = 10*eps;
				relative_blur.at<double>(i,j) = patch.at<double>(i,j) / est_conv.at<double>(i,j);
                ////std::cout << patch.at<double>(i,j) << " " << est_conv.at<double>(i,j) << " "<< relative_blur.at<double>(i,j) << std::endl;

            }
		filter2D(relative_blur, error_est, ddepth, kernel_hat, anchor, delta, cv::BORDER_DEFAULT);
		for(int i=0; i<pRows; i++)
			for(int j=0; j<pCols; j++) {
				result.at<double>(i,j) = result.at<double>(i,j) * error_est.at<double>(i,j);
                ////std::cout << "iters = " << l << " " << i << " " << j << std::endl;
            }

	}



    result = result*255;
    result.convertTo(result, CV_8U);
    imshow("postdublurring", result);

    //std::cout << "Stop Deblurring" << std::endl;

	return result;
}

*/
