/*
 * libblur.h
 *
 *  Created on: Jul 9, 2013
 *      Author: Ludovico
 */

#ifndef LIBBLUR_H_
#define LIBBLUR_H_


#include <opencv2/opencv.hpp>

// Input: parametri del kernel (parametri predetti da kalman)
// Output: kernel stimato

// This function is to evaluate the kernel
// (the smallest one possible inferable from EKF)
// knowing initial and final point after camera shake

cv::Mat evaluateKernel(cv::Point2f one, cv::Point2f two);




// Input: patch non blurrata + parametri del kernel (parametri predetti da kalman)
// Output: patch blurrata

// This function is to blur the patch with the predicted kernel
// (the smallest one possible inferable from EKF)

cv::Mat blurPatch(const cv::Mat &patch, cv::Point2f one, cv::Point2f two);





// Input: nuova patch blurrata + parametri del kernel (parametri stimati)
// Output: patch deblurrata


// This function is to deblur the patch with the estimated kernel
// LR deconvolution is used in the spatial domain
// (for a quick reference see http://en.wikipedia.org/wiki/RichardsonñLucy_deconvolution )


// Input kernel is supposed to be normalised (sum of its element equal to 1)


// If the deblurring process is not giving high quality outcomes, try to
// augment the number of iterations or to pre-process the kernel taping its edges
// (using http://read.pudn.com/downloads112/sourcecode/graph/texture_mapping/464953/images/edgetaper.m__.htm )


cv::Mat deblurPatch(const cv::Mat &patch, cv::Point2f one, cv::Point2f two);

#endif /* LIBBLUR_H_ */
