/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    test_opencv.cpp
 * @brief
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-06-26
 * @version 1.0
 * @note
 * @history
**/

#include <stddef.h>
#include <stdlib.h>

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/hough_transform_wrapper.h"

int main() {

  cv::Mat cv_img = cv::imread("../media/line.bmp");

  cv::Mat cv_gray, cv_edge;
  cv::cvtColor(cv_img, cv_gray, cv::COLOR_BGR2GRAY);
  cv::Canny(cv_gray, cv_edge, 3, 9);

  // cv::imshow("Test Line", cv_edge);
  // cv::waitKey(0);

  unsigned char *img = cv_edge.data;
  size_t vote[31*180] = {0};
  double *lines_mat = new double[10*6];
  unsigned int img_sz[2] = {cv_edge.size().width, cv_edge.size().height};
  unsigned int vote_sz[2] = {0};

  CCall_SHT(img, img_sz, 10, vote, vote_sz, lines_mat);

  cv::Mat vote_cv(vote_sz[0], vote_sz[1], CV_64FC1, cv::Scalar::all(0));
  if (vote_cv.isContinuous()) {
    double *st_ptr = vote_cv.ptr<double>(0);
    for (size_t i=0;i<(size_t)vote_sz[0]*(size_t)vote_sz[1];i++)
      st_ptr[i] = (double)vote[i];
  }

  system("pause>nul");
  delete [] vote;
  delete [] lines_mat;
}
