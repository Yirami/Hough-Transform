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

#include <windows.h>

#include <cstring>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "include/hough_transform_wrapper.h"

static const std::string hmat_csv="D:/GitRepo/hough-transform/media/hmat.csv";

// #define USE_DLL

int main() {

  std::ofstream result_f(hmat_csv);

  #ifdef USE_DLL
  HINSTANCE hDllInst;
  hDllInst = LoadLibrary("hough_transform_vs_dll.dll");
  typedef int(*DLLFUNC)(const unsigned char *img, const int img_sz[2], const int max_lines, double *lines_mat);
  DLLFUNC Get_C_SHT = (DLLFUNC)GetProcAddress(hDllInst, "CCall_SHT");
  #endif  // USE_DLL

  // cv::Mat cv_img = cv::imread("D:/GitRepo/hough-transform/media/debug.png");
  //
  // cv::Mat cv_gray, cv_edge;
  // cv::cvtColor(cv_img, cv_gray, cv::COLOR_BGR2GRAY);
  // cv::Canny(cv_gray, cv_edge, 100, 150);
  // cv::imwrite("D:/GitRepo/hough-transform/media/debug_edge.png", cv_edge);

  cv::Mat cv_edge = cv::imread("D:/GitRepo/hough-transform/media/debug_edge.png", CV_LOAD_IMAGE_GRAYSCALE);

  unsigned char *img = cv_edge.data;
  double lines_mat[10*6] = {0};
  double lines_mat2[10*6] = {0};
  int img_sz[2] = {cv_edge.size().width, cv_edge.size().height};

  #ifdef USE_DLL
  Get_C_SHT(img, img_sz, 10, lines_mat);
  #else
  CCall_SHT(img, img_sz, 10, lines_mat);
  CCall_DCHT(img, img_sz, 10, lines_mat2);
  #endif

  system("pause>nul");
  return 0;
}
