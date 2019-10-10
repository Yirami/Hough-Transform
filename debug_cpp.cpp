/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    debug_cpp.cpp
 * @brief   debug class interface with opencv
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-09-22
 * @version 1.0
 * @note
 * @history
**/

#include <vector>
#include <cstring>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "include/timer.hpp"
#include "include/standard_hough_transform.hpp"
#include "include/directional_coded_hough_transform.hpp"

static const std::string hmat_csv="D:/GitRepo/hough-transform/media/hmat.csv";

int main() {
  YUtils::Timer<double> sht_timer, dcht_timer;
  // std::ofstream result_f(hmat_csv);

  constexpr size_t max_lines = 10;

  cv::Mat cv_edge = cv::imread("D:/GitRepo/hough-transform/media/debug_edge.png", CV_LOAD_IMAGE_GRAYSCALE);

  YHoughTransform::SHT<double> sht;
  YHoughTransform::DCHT<double> dcht;
  std::array<size_t, 2> img_wh = {(size_t)cv_edge.size().width, (size_t)cv_edge.size().height};
  sht.FeedImage(cv_edge.data, img_wh);
  dcht.FeedImage(cv_edge.data, img_wh);

  // sht.Vote();
  // const size_t row_num = sht.GetRhoDiv();
  // const size_t col_num = sht.GetThetaDiv();
  // size_t *vote = new size_t[row_num*col_num];
  // std::memcpy(vote, sht.GetVotePtr(), row_num*col_num*sizeof(size_t));
  //
  // for (int r=0;r<row_num;r++) {
  //   for (int c=0;c<col_num;c++)
  //     result_f<<(int)vote[r*col_num+c]<<",";
  //   result_f<<std::endl;
  // }
  // delete [] vote;

  std::vector<YHoughTransform::HoughLine<double>> lines2;
  dcht_timer.Start();
  dcht.Vote();
  dcht.FindPeaks(max_lines, lines2);
  dcht.FindLines(lines2);
  dcht.Radian2Degree(lines2);
  dcht_timer.Stop();
  std::vector<YHoughTransform::HoughLine<double>> lines;
  std::vector<size_t> filt;
  for (size_t i=0; i<180; i++)
    filt.push_back(i);
  sht_timer.Start();
  sht.Vote();
  // sht.FindPeaks(max_lines, lines);
  sht.FindPeaksS(filt, max_lines, lines);
  sht.FindLines(lines);
  sht.Radian2Degree(lines);
  sht_timer.Stop();
  double sht_t = sht_timer.Elapsed_ms();
  double dcht_t = dcht_timer.Elapsed_ms();
}
