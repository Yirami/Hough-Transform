/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    hough_transform_wrapper.h
 * @brief
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-06-26
 * @version 1.0
 * @note
 * @history
**/

#ifndef _HOUGH_TRANSFORM_WRAPPER_H_
#define _HOUGH_TRANSFORM_WRAPPER_H_


#ifdef __linux__

#else // windows
// #define MAKE_LIB
#ifdef MAKE_LIB
#ifdef HOUGHTRANSFORMVSDLL_EXPORTS
#define HOUGHTRANSFORMVSDLL_API __declspec(dllexport)
#else
#define HOUGHTRANSFORMVSDLL_API __declspec(dllimport)
#endif
#endif  // MAKE_LIB
#endif  // __linux__


#ifdef __cplusplus
#include <array>
#include <string.h>
// include c++ code headers here
#include <vector>
#include <array>
#include "./standard_hough_transform.hpp"
#include "./directional_coded_hough_transform.hpp"

extern "C" {
#endif

#include <stddef.h>

// write wrapper functions invoked by c projects here

#ifdef __linux__

#else // windows
#ifdef MAKE_LIB
HOUGHTRANSFORMVSDLL_API void CCall_SHT(const unsigned char *img, const int img_sz[2], const int max_lines, double *lines_mat);
HOUGHTRANSFORMVSDLL_API void CCall_DCHT(const unsigned char *img, const int img_sz[2], const int max_lines, double *lines_mat);
#else
void CCall_SHT(const unsigned char *img, const int img_sz[2], const int max_lines, double *lines_mat);
void CCall_DCHT(const unsigned char *img, const int img_sz[2], const int max_lines, double *lines_mat);
#endif
#endif //  __linux__

#ifdef __cplusplus
}
#endif


#endif
