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


#ifdef __cplusplus
#include <array>
#include <string.h>
// include c++ code headers here
#include <vector>
#include <array>
#include "./standard_hough_transform.hpp"
extern "C" {
#endif

#include <stddef.h>

// write wrapper functions invoked by c projects here

void CCall_SHT(const unsigned char *img, const unsigned int img_sz[2], const unsigned int max_lines, size_t *vote, unsigned int vote_sz[2], double *lines_mat);

#ifdef __cplusplus
}
#endif


#endif
