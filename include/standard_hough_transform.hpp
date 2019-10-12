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

  struct Point2D {
    int x;  // along columns
    int y;  // along rows
  };

  template <typename T>
  struct HoughLine {
    T rho;    // 0 is top-left corner
    T theta;  // [-pi/2, pi/2) 0 for horizontal(right), positive(anti-clockwise)
    Point2D start_pt;
    Point2D end_pt;
  };

  template <typename T, size_t PI_DIV=180>  // [-pi/2, pi/2)
  class SHT {
  public:
    SHT();
    ~SHT() {if (vote_map_) delete [] vote_map_;};
    virtual void FeedImage(const unsigned char *img, array<size_t, 2> &size_wh);
    inline void SetAngleFilter();
    void SetAngleFilter(vector<size_t> &filt) {theta_filter_ = filt;};
    virtual void Vote();  // the only one to be invoked while debug vote map
    size_t GetThetaDiv() const {return theta_div_;};  // for debug vote map
    size_t GetRhoDiv() const {return rho_div_;};  // for debug vote map
    const size_t *GetVotePtr() const {return vote_map_;}; // for debug vote map
    void FindPeaks(const vector<size_t> &angle_filter,
                   size_t max_lines,
                   vector<HoughLine<T>> &lines);
    inline void FindPeaks(size_t max_lines, vector<HoughLine<T>> &lines);
    inline void FindPeaksS(const vector<size_t> &angle_filter,
                           size_t max_lines,
                           vector<HoughLine<T>> &lines);
    void FindLines(vector<HoughLine<T>> &lines) const;
    inline void Radian2Degree(vector<HoughLine<T>> &lines) const;
  protected:

    T Rad2Deg_(T radian) const{return 180*radian/PI;};
    T Deg2Rad_(T degree) const{return PI*degree/180;};
  public:
    constexpr static T PI = 3.14159265358979;
    T NOISE_SCALE = 0.1;  // give up short lines [scale*height]
    T SUPPRESS_THETA = 2; // degree
    T SUPPRESS_RHO = 3;
    size_t LINE_GAP = 5;
  protected:
    const unsigned char *img_ = nullptr;
    array<size_t, 2> img_size_wh_ = {0}; // [columns, rows]

    vector<size_t> theta_filter_;
    T theta_res_ = PI/PI_DIV;   // resolution of theta
    T rho_res_ = 1; // resolution of rho
    size_t theta_div_ = PI_DIV;
    size_t rho_div_ = 0;

    size_t *vote_map_ = nullptr;  // FindPeaks() will modify vote map !!!

    static bool tri_map_init_;
    static array<TriMap<T>, PI_DIV> *tri_map_;
  };

  template <typename T, size_t PI_DIV>
  bool SHT<T, PI_DIV>::tri_map_init_ = false;

  template <typename T, size_t PI_DIV>
  array<TriMap<T>, PI_DIV> * SHT<T, PI_DIV>::tri_map_ = \
    YPattern::Singleton<array<TriMap<T>, PI_DIV>>::Get();

  template <typename T, size_t PI_DIV>
  SHT<T, PI_DIV>::SHT() {
    SetAngleFilter();
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
  void SHT<T, PI_DIV>::SetAngleFilter() {
    for (size_t i=0; i<PI_DIV; i++)
    // for (size_t i=30; i<PI_DIV-30; i++)
      theta_filter_.push_back(i);
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::FeedImage(const unsigned char *img,
                                 array<size_t, 2> &size_wh) {
    img_ = img;
    img_size_wh_ = size_wh;
    T rho_max = (T)sqrt(pow(size_wh[0]-1,2)+pow(size_wh[1]-1,2));
    rho_div_ = (size_t)floor(rho_max/rho_res_+0.5)*2+1; // rho may be negative
    if (vote_map_) {delete [] vote_map_; vote_map_ = nullptr;}
    vote_map_ = new size_t[rho_div_*theta_div_];
    std::memset(vote_map_, 0, rho_div_*theta_div_*sizeof(size_t));
  }

  template <typename T, size_t PI_DIV>
  inline void SHT<T, PI_DIV>::Radian2Degree(vector<HoughLine<T>> &lines) const{
    for (auto &line:lines)
      line.theta = Rad2Deg_(line.theta);
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::Vote() {
    T rho_shift = ((T)rho_div_-1)/2;
    for (size_t c=0; c<img_size_wh_[0]; c++) {
      for (size_t r=0; r<img_size_wh_[1]; r++) {
        if (img_[r*img_size_wh_[0]+c])
          for (auto th:theta_filter_) {
            const T rho_c = floor((r*tri_map_->at(th).cos+\
                                   c*tri_map_->at(th).sin)/rho_res_+0.5);
            const size_t rho_c_shift = (size_t)(rho_c + rho_shift);
            vote_map_[PI_DIV*rho_c_shift+th]++;
          }
      }
    }
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::FindPeaks(const vector<size_t> &angle_filter,
                                 size_t max_lines,
                                 vector<HoughLine<T>> &lines) {
    array<int, 2> suppress = {(int)ceil(Deg2Rad_(SUPPRESS_THETA)/theta_res_),
                              (int)ceil(SUPPRESS_RHO/rho_res_)};
    lines.clear();
    for (size_t i=0; i<max_lines; i++) {
      // search maximum vote
      size_t max_vote = 0;
      int max_r = 0, max_c = 0;
      for (auto c:angle_filter) {
        for (int r=0; r<(int)rho_div_; r++)
          if (vote_map_[(size_t)r*PI_DIV+c]>max_vote) {
            max_vote = vote_map_[(size_t)r*PI_DIV+c];
            max_r = r;
            max_c = (int)c;
          }
      }
      if (max_vote<size_t(img_size_wh_[0]*NOISE_SCALE)) break;
      HoughLine<T> line = {0};
      line.rho = rho_res_*(max_r-((T)rho_div_-1)/2);
      line.theta = tri_map_->at(max_c).angle;
      lines.push_back(line);
      // suppress in vote space
      const int start_r = max_r>suppress[1]?max_r-suppress[1]:0;
      const int end_r = max_r+suppress[1]<(int)rho_div_?max_r+suppress[1]+1:\
                                                       (int)rho_div_;
      // columns are handled differently because of angle's head-tail closure
      vector<int> theta_list;
      theta_list.push_back(max_c);
      for (int i=1; i<=suppress[0]; i++) {
        if (max_c<i)
          theta_list.push_back((int)theta_div_+max_c-i);
        else
          theta_list.push_back(max_c-i);
        if (max_c+i>=(int)theta_div_)
          theta_list.push_back(max_c+i-(int)theta_div_);
        else
          theta_list.push_back(max_c+i);
      }
      for (int c:theta_list)
        for (int r=start_r; r<end_r; r++)
          vote_map_[r*(int)theta_div_+c] = 0;
    }
  }

  template <typename T, size_t PI_DIV>
  inline void SHT<T, PI_DIV>::FindPeaks(size_t max_lines,
                                        vector<HoughLine<T>> &lines) {
    FindPeaks(theta_filter_, max_lines, lines);
  }

  template <typename T, size_t PI_DIV>
  inline void SHT<T, PI_DIV>::FindPeaksS(const vector<size_t> &angle_filter,
                                         size_t max_lines,
                                         vector<HoughLine<T>> &lines) {
    // puppet for vote_map_
    size_t *vote_map_puppet = new size_t[rho_div_*theta_div_];
    std::memcpy(vote_map_puppet, vote_map_, rho_div_*theta_div_*sizeof(size_t));
    size_t *vote_map_swap = vote_map_;
    vote_map_ = vote_map_puppet;
    FindPeaks(angle_filter, max_lines, lines);
    vote_map_ = vote_map_swap;
    delete [] vote_map_puppet;
  }

  template <typename T, size_t PI_DIV>
  void SHT<T, PI_DIV>::FindLines(vector<HoughLine<T>> &lines) const{
    constexpr int shift = 16;
    for (auto &line:lines) {
      const T rho = line.rho;
      const T theta = Rad2Deg_(line.theta);
      const T curr_sin = sin(line.theta);
      const T curr_cos = cos(line.theta);

      int r_start = 0;
  		int c_start = (int)floor((rho-(T)r_start*curr_cos)/curr_sin);
  		// determine whether the c_start overflows the boundary
  		if (c_start<0) {
  			c_start = 0;
  			r_start = (int)floor((rho-(T)c_start*curr_sin)/curr_cos);
  		}
  		else if (c_start>=(int)img_size_wh_[0]) {
  			c_start = (int)img_size_wh_[0]-1;
  			r_start = (int)floor((rho-(T)c_start*curr_sin)/curr_cos);
  		}
  		bool bias_flag = false;
  		int r0 = r_start, c0 = c_start, dr0, dc0;
  		if (abs(theta)>45) {
  			bias_flag = true;
  			dr0 = 1;
  			dc0 = (int)floor(curr_cos*(T)(1 << shift)/fabs(curr_sin)+0.5);
  			c0 = (c0 << shift)+(1 << (shift-1));
  		}
  		else {
  			dc0 = 1;
  			dr0 = (int)floor(fabs(curr_sin)*(T)(1 << shift)/curr_cos+0.5);
  			r0 = (r0 << shift)+(1 << (shift-1));
  		}
  		if (theta>0)
  			dc0 = -dc0;
  		// walk along the line using fixed-point arithmetics,
  		// ... stop at the image border
  		int last_length = 0;
      Point2D line_start = {0}, line_end = {0};
  		for (int gap=0, c=c0, r=r0, start_p_flag =1; ; c+=dc0, r+=dr0) {
  			int r1, c1;
  			if (bias_flag) {
  				c1 = c >> shift;
  				r1 = r;
  			}
  			else {
  				c1 = c;
  				r1 = r >> shift;
  			}
  			if (c1<0||c1>=(int)img_size_wh_[0]||r1<0||r1>=(int)img_size_wh_[1]) {
  				// ensure last_length has been update before exit
          int this_length = (int)floor(sqrt(
                                      pow((T)line_start.x-(T)line_end.x,2) +
                                      pow((T)line_start.y-(T)line_end.y,2)));
  				if (this_length > last_length) {
            last_length = this_length;
            line.start_pt.x = line_start.x;
            line.start_pt.y = line_start.y;
            line.end_pt.x = line_end.x;
            line.end_pt.y = line_end.y;
  				}
  				break;
  			}
  			if (img_[r1*img_size_wh_[0]+c1]) {
  				gap = 0;
  				if (start_p_flag) {
  					start_p_flag = 0;
  					line_start.x = c1;
  					line_start.y = r1;
  					line_end.x = c1;
  					line_end.y = r1;
  				}
  				else {
  					line_end.x = c1;
  					line_end.y = r1;
  				}
  			}
  			else if (!start_p_flag) {
  				// fuzzy processing the points beside line
  				int left_p_flag = 0, right_p_flag = 0;
  				if (c1>0 && img_[r1*img_size_wh_[0]+(c1-1)])
  					left_p_flag = 1;
  				if (c1<(int)img_size_wh_[0]-1 && img_[r1*img_size_wh_[0]+(c1+1)])
  					right_p_flag = 1;
  				if (left_p_flag || right_p_flag) {
  					gap = 0;
            line_end.x = c1;
  					line_end.y = r1;
  					continue;
  				}
  				// end of line segment
  				if (++gap > (int)LINE_GAP) {
  					start_p_flag = 1;
  					int this_length = (int)floor(sqrt(
                                      pow((T)line_start.x-(T)line_end.x,2) +
                                      pow((T)line_start.y-(T)line_end.y,2)));
  					if (this_length > last_length) {
  						last_length = this_length;
  						// coordinate transformation
              line.start_pt.x = line_start.x;
              line.start_pt.y = line_start.y;
              line.end_pt.x = line_end.x;
              line.end_pt.y = line_end.y;
  					}
  				}
  			}
  		}
  		// enhanced the robustness: none line segment are detected
  		if (!last_length) {
  			line.start_pt.x = r_start;
  			line.start_pt.y = c_start;
  			line.end_pt.x = r_start;
  			line.end_pt.y = c_start;
  		}
    }
  }

}

#endif
