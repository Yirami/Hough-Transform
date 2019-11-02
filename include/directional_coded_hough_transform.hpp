/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    directional_coded_hough_transform.hpp
 * @brief   Template class for Directional Coded Hough Transform(DCHT) .
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-07-30
 * @version 1.0
 * @note
 * @history
**/

#ifndef _DIRECTIONAL_CODED_HOUGH_TRANSFORM_HPP_
#define _DIRECTIONAL_CODED_HOUGH_TRANSFORM_HPP_

#include <cmath>

#include "standard_hough_transform.hpp"

namespace YHoughTransform {

  template <typename T, size_t PI_DIV=180>  // [-pi/2, pi/2)
  class DCHT: public SHT<T, PI_DIV> {
  public:
    DCHT();
    virtual void Vote();
    using SHT<T, PI_DIV>::PI;
  protected:
    using SHT<T, PI_DIV>::img_;
    using SHT<T, PI_DIV>::img_size_wh_;
    using SHT<T, PI_DIV>::theta_filter_;
    using SHT<T, PI_DIV>::theta_res_;
    using SHT<T, PI_DIV>::rho_res_;
    using SHT<T, PI_DIV>::rho_div_;
    using SHT<T, PI_DIV>::vote_map_;
    using SHT<T, PI_DIV>::tri_map_init_;
    using SHT<T, PI_DIV>::tri_map_;
  private:
    bool skip_unmatched_pixel_ = true;
  };

  template <typename T, size_t PI_DIV>
  DCHT<T, PI_DIV>::DCHT() {
    for (size_t i=0; i<PI_DIV; i++)
      theta_filter_.push_back(i);
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
  void DCHT<T, PI_DIV>::Vote() {
    const TriMap<T> *map_h = tri_map_->data();
    const T rho_shift = (T)(rho_div_-1)/2;
    const int theta_limit_node[3] = {(int)(ceil((T)PI_DIV/4.0)),
                                     (int)(ceil((T)PI_DIV/2.0)),
                                     (int)(ceil((T)PI_DIV/4.0*3))};
    // Attention:
    //   1 pixel is clipped around image here and vote may lose accuracy.
    for (int c=1; c<(int)img_size_wh_[0]-1; c++)
      for (int r=1; r<(int)img_size_wh_[1]-1; r++) {
        if (img_[r*(int)img_size_wh_[0]+c]) {
          // index of limit start&end angle := [start_idx, end_idx]
          int theta_limit[2] = {0, (int)PI_DIV-1};
          // index of reference point
          int top_left_idx = (r-1)*(int)img_size_wh_[0]+c-1;
          // convolution result
          unsigned char b4=0, b3=0, b2=0, b1=0;
          if (img_[top_left_idx]) b4++; // 11
          if (img_[top_left_idx+2*img_size_wh_[0]+2]) b4++; // 33
          if (img_[top_left_idx+img_size_wh_[0]]) b3++; // 21
          if (img_[top_left_idx+img_size_wh_[0]+2]) b3++; // 23
          if (img_[top_left_idx+2]) b2++; // 13
          if (img_[top_left_idx+2*img_size_wh_[0]]) b2++; // 31
          if (img_[top_left_idx+1]) b1++; // 12
          if (img_[top_left_idx+2*img_size_wh_[0]+1]) b1++; // 32
          const unsigned char sum = (b4<<6)|(b3<<4)|(b2<<2)|b1;
          // for each point not zero judgment angle range based on convolution
  				switch (sum) {
  					// Note: When we need to reduce the code space, we can compress
  					// 	... the range of the "case" to reduce the size of the table !
  					//  ... DETAILS may get here :
  					//  ... 	http://www.cnblogs.com/idorax/p/6275259.html
  					//
  					//  The configuration below can be adjusted as appropriate
  					//
            // -90~-45
  					case 0x42:case 0x81:case 0x41:case 0x82:
              theta_limit[0] = 0;
              theta_limit[1] = theta_limit_node[0];
  						break;
  					// -45~0
  					case 0x90:case 0x60:case 0x50:case 0xA0:
              theta_limit[0] = theta_limit_node[0];
              theta_limit[1] = theta_limit_node[1];
  						break;
  					// 0~45
  					case 0x24:case 0x18:case 0x14:case 0x28:
              theta_limit[0] = theta_limit_node[1];
              theta_limit[1] = theta_limit_node[2];
  						break;
  					// 45~90
  					case 0x09:case 0x06:case 0x05:case 0x0A:
              theta_limit[0] = theta_limit_node[2];
              theta_limit[1] = (int)PI_DIV-1;
  						break;
  					// -90~0
  					case 0x51:case 0x80:case 0x91:case 0x61:case 0x52:case 0x95:
              theta_limit[0] = 0;
              theta_limit[1] = theta_limit_node[1];
  						break;
  					// -45~45
  					case 0x54:case 0x20:case 0x94:case 0x64:case 0x58:case 0x65:
              theta_limit[0] = theta_limit_node[0];
              theta_limit[1] = theta_limit_node[2];
  						break;
  					// 0~90
  					case 0x15:case 0x08:case 0x25:case 0x19:case 0x16:case 0x59:
              theta_limit[0] = theta_limit_node[1];
              theta_limit[1] = (int)PI_DIV-1;
  						break;
  					// 45~-45
  					case 0x45:case 0x02:case 0x85:case 0x49:case 0x46:case 0x56:
              theta_limit[0] = theta_limit_node[2];
              theta_limit[1] = theta_limit_node[0];
  						break;
  					// else
  					default:
  						if (skip_unmatched_pixel_) continue;
  				}
          for (auto th:theta_filter_) {
            if (theta_limit[0]<theta_limit[1]) {
              if (th<theta_limit[0] || th>theta_limit[1]) continue;
            }
            else {
              if (th<theta_limit[0] && th>theta_limit[1]) continue;
            }
            const T rho = floor((r*map_h[th].cos+c*map_h[th].sin)/rho_res_+0.5);
            vote_map_[PI_DIV*(size_t)(rho+rho_shift)+th]++;
          }
        }
      }
  }

}

#endif
