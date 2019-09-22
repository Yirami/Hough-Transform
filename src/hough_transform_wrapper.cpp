/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    hough_transform_wrapper.c
 * @brief
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-06-26
 * @version 1.0
 * @note
 * @history
**/

#include "../include/hough_transform_wrapper.h"

void CCall_SHT(const unsigned char *img, const int img_sz[2], const int max_lines, double *lines_mat) {
  YHoughTransform::SHT<double> sht;
  std::array<size_t, 2> img_wh = {(size_t)img_sz[0], (size_t)img_sz[1]};
  sht.FeedImage(img, img_wh);
  std::vector<YHoughTransform::HoughLine<double>> lines;
  sht.GetLines(max_lines, lines);

  for (size_t i=0; i<lines.size(); i++) {
    lines_mat[i*6] = lines[i].rho;
    lines_mat[i*6+1] = lines[i].theta;
    lines_mat[i*6+2] = (double)lines[i].start_pt[0];
    lines_mat[i*6+3] = (double)lines[i].start_pt[1];
    lines_mat[i*6+4] = (double)lines[i].end_pt[0];
    lines_mat[i*6+5] = (double)lines[i].end_pt[1];
  }

}

void CCall_DCHT(const unsigned char *img, const int img_sz[2], const int max_lines, double *lines_mat) {
  YHoughTransform::DCHT<double> dcht;
  std::array<size_t, 2> img_wh = {(size_t)img_sz[0], (size_t)img_sz[1]};
  dcht.FeedImage(img, img_wh);
  std::vector<YHoughTransform::HoughLine<double>> lines;
  dcht.GetLines(max_lines, lines);

  for (size_t i=0; i<lines.size(); i++) {
    lines_mat[i*6] = lines[i].rho;
    lines_mat[i*6+1] = lines[i].theta;
    lines_mat[i*6+2] = (double)lines[i].start_pt[0];
    lines_mat[i*6+3] = (double)lines[i].start_pt[1];
    lines_mat[i*6+4] = (double)lines[i].end_pt[0];
    lines_mat[i*6+5] = (double)lines[i].end_pt[1];
  }
}
