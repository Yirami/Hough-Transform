/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    standard_hough_transform.hpp
 * @brief   Template class for Standard Hough Transform(SHT) .
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-06-25
 * @version 1.0
 * @note
 * @history
**/

#ifndef _STANDARD_HOUGH_TRANSFORM_HPP_
#define _STANDARD_HOUGH_TRANSFORM_HPP_

#include <cmath>
#include <array>
#include <vector>
#include <cstring>

#include "./singleton.hpp"

namespace YHoughTransform {
  using std::vector;
  using std::array;

  template <typename T>
  struct TriMap {
    T angle;
    T sin;
    T cos;
  };

  template <typename T>
  struct HoughLine {
    T rho;    // 0 is top-left corner
    T theta;  // [-pi/2, pi/2)@[-90, 90) 0 for verticala and negative for left
    array<size_t, 2> start_pt;  // [column, row]
    array<size_t, 2> end_pt;
  };

  template <typename T, size_t PI_DIV=180>  // [-pi/2, pi/2)
  class SHT {
  public:
    SHT();
    ~SHT() {if (vote_map_) delete [] vote_map_;};
    void FeedImage(const unsigned char *img, array<size_t, 2> &size_wh);
    void SetAngleFilter(vector<size_t> filt) {theta_filter_ = filt;};
    void GetLines(size_t max_lines, vector<HoughLine<T>> &lines);
    void Vote();
    void FindPeaks(size_t max_lines, vector<HoughLine<T>> &lines);
    void FindLines(vector<HoughLine<T>> &lines);
  public:
    constexpr static T PI = 3.14159265358979;
    T NOISE_SCALE = 0.1;  // give up short lines [scale*height]
    T SUPPRESS_THETA = 2;
    T SUPPRESS_RHO = 3;
    size_t LINE_GAP = 5;
  public:
    const unsigned char *img_ = nullptr;
    array<size_t, 2> img_size_ = {0}; // [columns, rows]

    vector<size_t> theta_filter_;
    T theta_res_ = PI/PI_DIV;   // resolution of theta
    T rho_res_ = 1; // resolution of rho
    size_t theta_div_ = PI_DIV;
    size_t rho_div_ = 0;

    size_t *vote_map_ = nullptr;

    static array<TriMap<T>, PI_DIV> *tri_map_;
  };

  template <typename T, size_t PI_DIV>
  array<TriMap<T>, PI_DIV> * SHT<T, PI_DIV>::tri_map_=YPattern::Singleton<array<TriMap<T>, PI_DIV>>::Get();

  template <typename T, size_t PI_DIV>
  SHT<T, PI_DIV>::SHT() {
    for (size_t i=0; i<PI_DIV-0; i++)
    // for (size_t i=30; i<PI_DIV-30; i++)
      theta_filter_.push_back(i);
    static bool tri_map_init_ = false;
    if (!tri_map_init_) {
      for (size_t i=0; i<PI_DIV; i++) {
        tri_map_->at(i).angle = theta_res_*i-PI/2;
        tri_map_->at(i).sin = sin(tri_map_->at(i).angle);
        tri_map_->at(i).cos = cos(tri_map_->at(i).angle);
      }
      tri_map_init_ = true;
    }
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::FeedImage(const unsigned char *img, array<size_t, 2> &size_wh) {
    img_ = img;
    img_size_ = size_wh;
    T rho_max = (T)sqrt(pow(size_wh[0]-1,2)+pow(size_wh[1]-1,2));
    rho_div_ = (size_t)floor(rho_max/rho_res_+0.5)*2+1; // rho may be negative
    if (vote_map_) delete [] vote_map_;
    vote_map_ = new size_t[rho_div_*theta_div_];
    std::memset(vote_map_, 0, rho_div_*theta_div_*sizeof(size_t));
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::GetLines(size_t max_lines, vector<HoughLine<T>> &lines) {
    Vote();
    FindPeaks(max_lines, lines);
    FindLines(lines);
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::Vote() {
    double rho_shift = (double)(rho_div_-1)/2;
    for (size_t c=0; c<img_size_[0]; c++)
      for (size_t r=0; r<img_size_[1]; r++)
        if (img_[r*img_size_[0]+c])
          for (auto th:theta_filter_) {
            const double rho_c = floor((r*tri_map_->at(th).sin+c*tri_map_->at(th).cos)/rho_res_+0.5);
            const size_t rho_c_shift = (size_t)(rho_c + rho_shift);
            vote_map_[PI_DIV*rho_c_shift+th]++;
          }
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::FindPeaks(size_t max_lines, vector<HoughLine<T>> &lines) {
    array<size_t, 2> suppress = {(size_t)ceil(SUPPRESS_THETA/theta_res_), (size_t)ceil(SUPPRESS_RHO/rho_res_)};
    lines.clear();
    for (size_t i=0; i<max_lines; i++) {
      // search maximum vote
      size_t max_vote = 0, max_r = 0, max_c = 0;
      for (auto c:theta_filter_)
        for (size_t r=0; r<rho_div_; r++)
          if (vote_map_[r*theta_div_+c]>max_vote) {
            max_vote = vote_map_[r*theta_div_+c];
            max_r = r;
            max_c = c;
          }
      if (max_vote<size_t(img_size_[0]*NOISE_SCALE)) break;
      HoughLine<T> line = {0};
      line.rho = rho_res_*(max_r-(rho_div_-1)/2);
      line.theta = tri_map_->at(max_c).angle;
      lines.push_back(line);
      // suppress in vote space
      size_t start_r = max_r>suppress[1]?(max_r-suppress[1]):0;
      size_t end_r = max_r+suppress[1]<rho_div_?(max_r+suppress[1]+1):rho_div_;
      vector<size_t> c_list;
      c_list.push_back(max_c);
      for (size_t i=1; i<=suppress[1]; i++) {
        if (max_c<i)
          c_list.push_back(theta_div_+max_c-i);
        else
          c_list.push_back(max_c-i);
        if (max_c+i>=theta_div_)
          c_list.push_back(max_c+i-theta_div_);
        else
          c_list.push_back(max_c+i);
      }
      for (size_t c:c_list)
        for (size_t r=start_r; r<rho_div_; r++)
          vote_map_[r*theta_div_+c] = 0;
    }
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::FindLines(vector<HoughLine<T>> &lines) {
    constexpr int shift = 16;
    for (auto line:lines) {
      const T rho = line.rho;
      const T theta = line.theta;
      const T curr_sin = sin(theta);
      const T curr_cos = cos(theta);

      int rStart = 0;
  		int cStart = (int)floor((rho-(T)rStart*curr_sin)/curr_cos);
  		// determine whether the cStart overflows the boundary
  		if (cStart<0) {
  			cStart = 0;
  			rStart = (int)floor((rho-(T)cStart*curr_cos)/curr_sin);
  		}
  		else if (cStart>=(int)img_size_[0]) {
  			cStart = (int)img_size_[0]-1;
  			rStart = (int)floor((rho-(T)cStart*curr_cos)/curr_sin);
  		}
  		int biasFlag = 0;
  		int r0 = rStart, c0 = cStart, dr0, dc0;
  		if (abs(theta)<=45) {
  			biasFlag = 1;
  			dr0 = 1;
  			dc0 = (int)floor(fabs(curr_sin)*((int)1 << shift)/curr_cos+0.5);
  			c0 = (c0 << shift)+((int)1 << (shift-1));
  		}
  		else {
  			dc0 = 1;
  			dr0 = (int)floor(curr_cos*(1 << shift)/fabs(curr_sin)+0.5);
  			r0 = (r0 << shift)+(1 << (shift-1));
  		}
  		if (theta>0)
  			dc0 = -dc0;
  		// walk along the line using fixed-point arithmetics,
  		// ... stop at the image border
  		int lastLength = 0, lineStart[2] = {0}, lineEnd[2] = {0};
  		for (int gap=0, c=c0, r=r0, startPFlag =1; ; c+=dc0, r+=dr0) {
  			int r1, c1;
  			if (biasFlag) {
  				c1 = c >> shift;
  				r1 = r;
  			}
  			else {
  				c1 = c;
  				r1 = r >> shift;
  			}
  			if (c1 < 0||c1 >= (int)img_size_[0]||r1 < 0||r1 >= (int)img_size_[1]) {
  				// ensure lastLength has been update before  exit
          int thisLength = (int)floor(sqrt(pow((double)lineStart[0]-(double)lineEnd[0],2) + pow((double)lineStart[1]-(double)lineEnd[1],2)));
  				if (thisLength > lastLength) {
            lastLength = thisLength;
            line.start_pt.at(0) = (size_t)lineStart[0];
            line.start_pt.at(1) = (size_t)lineStart[1];
            line.end_pt.at(0) = (size_t)lineEnd[0];
            line.end_pt.at(1) = (size_t)lineEnd[1];
  				}
  				break;
  			}
  			if (img_[c1*img_size_[1]+r1]) {
  				gap = 0;
  				if (startPFlag) {
  					startPFlag = 0;
  					lineStart[0] = c1;
  					lineStart[1] = r1;
  					lineEnd[0] = c1;
  					lineEnd[1] = r1;
  				}
  				else {
  					lineEnd[0] = c1;
  					lineEnd[1] = r1;
  				}
  			}
  			else if (!startPFlag) {
  				// fuzzy processing the points beside line
  				int leftPFlag = 0, rightPFlag = 0;
  				if (c1>0 && img_[(c1-1)*img_size_[1]+r1])
  					leftPFlag = 1;
  				if (c1<(int)img_size_[0]-1 && img_[(c1+1)*img_size_[1]+r1])
  					rightPFlag = 1;
  				if (leftPFlag || rightPFlag) {
  					gap = 0;
            lineEnd[0] = c1;
  					lineEnd[1] = r1;
  					continue;
  				}
  				// end of line segment
  				if (++gap > (int)LINE_GAP) {
  					startPFlag = 1;
  					int thisLength = (int)floor(sqrt(pow((double)lineStart[0]-(double)lineEnd[0],2) + pow((double)lineStart[1]-(double)lineEnd[1],2)));
  					if (thisLength > lastLength) {
  						lastLength = thisLength;
  						// coordinate transformation
              line.start_pt.at(0) = (size_t)lineStart[0];
              line.start_pt.at(1) = (size_t)lineStart[1];
              line.end_pt.at(0) = (size_t)lineEnd[0];
              line.end_pt.at(1) = (size_t)lineEnd[1];
  					}
  				}
  			}
  		}
  		// enhanced the robustness: none line segment are detected
  		if (!lastLength) {
  			line.start_pt.at(0) = (size_t)rStart;
  			line.start_pt.at(1) = (size_t)cStart;
  			line.end_pt.at(0) = (size_t)rStart;
  			line.end_pt.at(1) = (size_t)cStart;
  		}
    }
  }

}

#endif
