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

void CCall_SHT(const unsigned char *img, const unsigned int img_sz[2], const unsigned int max_lines, size_t *vote, unsigned int vote_sz[2], double *lines_mat) {
  YHoughTransform::SHT<double> sht;
  std::array<size_t, 2> img_wh = {(size_t)img_sz[0], (size_t)img_sz[1]};
  sht.FeedImage(img, img_wh);
  std::vector<YHoughTransform::HoughLine<double>> lines;
  sht.GetLines(max_lines, lines);

  size_t row_num = sht.rho_div_;
  size_t col_num = sht.theta_div_;
  std::memcpy(vote, sht.vote_map_, row_num*col_num*sizeof(size_t));

  vote_sz[0] = (unsigned int)row_num;
  vote_sz[1] = (unsigned int)col_num;

  for (size_t i=0; i<lines.size(); i++) {
    lines_mat[i*6] = lines[i].rho;
    lines_mat[i*6+1] = lines[i].theta;
    lines_mat[i*6+2] = (double)lines[i].start_pt[0];
    lines_mat[i*6+3] = (double)lines[i].start_pt[1];
    lines_mat[i*6+4] = (double)lines[i].end_pt[0];
    lines_mat[i*6+5] = (double)lines[i].end_pt[1];
  }

}
