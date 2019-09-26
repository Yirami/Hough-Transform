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

#include "./standard_hough_transform.hpp"

namespace YHoughTransform {

  template <typename T, size_t PI_DIV=180>  // [-pi/2, pi/2)
  class DCHT: public SHT<T, PI_DIV> {
  public:
    DCHT();
    void Vote();
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
    const double rho_shift = (double)(rho_div_-1)/2;
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
          int ref_pt_idx = (r-1)*(int)img_size_wh_[0]+c-1;
          // convolution result
          const int mat11 = img_[ref_pt_idx]?1:0;
          const int mat12 = img_[ref_pt_idx+1]?1:0;
          const int mat13 = img_[ref_pt_idx+2]?1:0;
          const int mat21 = img_[ref_pt_idx+img_size_wh_[0]]?1:0;
          const int mat23 = img_[ref_pt_idx+img_size_wh_[0]+2]?1:0;
          const int mat31 = img_[ref_pt_idx+2*img_size_wh_[0]]?1:0;
          const int mat32 = img_[ref_pt_idx+2*img_size_wh_[0]+1]?1:0;
          const int mat33 = img_[ref_pt_idx+2*img_size_wh_[0]+2]?1:0;
  				int sum = 1000*(mat11+mat33)+ \
                    100*(mat21+mat23)+ \
                    10*(mat13+mat31)+ \
                    mat12+mat32;
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
  					case 1002:case 2001:case 1001:case 2002:
              theta_limit[0] = 0;
              theta_limit[1] = theta_limit_node[0];
  						break;
  					// -45~0
  					case 2100:case 1200:case 1100:case 2200:
              theta_limit[0] = theta_limit_node[0];
              theta_limit[1] = theta_limit_node[1];
  						break;
  					// 0~45
  					case 210:case 120:case 110:case 220:
              theta_limit[0] = theta_limit_node[1];
              theta_limit[1] = theta_limit_node[2];
  						break;
  					// 45~90
  					case 21:case 12:case 11:case 22:
              theta_limit[0] = theta_limit_node[2];
              theta_limit[1] = (int)PI_DIV-1;
  						break;
  					// -90~0
  					case 1101:case 2000:case 2101:case 1201:case 1102:case 2111:
              theta_limit[0] = 0;
              theta_limit[1] = theta_limit_node[1];
  						break;
  					// -45~45
  					case 1110:case 200:case 2110:case 1210:case 1120:case 1211:
              theta_limit[0] = theta_limit_node[0];
              theta_limit[1] = theta_limit_node[2];
  						break;
  					// 0~90
  					case 111:case 20:case 211:case 121:case 112:case 1121:
              theta_limit[0] = theta_limit_node[1];
              theta_limit[1] = (int)PI_DIV-1;
  						break;
  					// 45~-45
  					case 1011:case 2:case 2011:case 1021:case 1012:case 1112:
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
            const double rho_c = floor((r*tri_map_->at(th).cos+
                                        c*tri_map_->at(th).sin)/rho_res_+0.5);
            const size_t rho_c_shift = (size_t)(rho_c + rho_shift);
            vote_map_[PI_DIV*rho_c_shift+th]++;
          }
        }
      }
  }

}

#endif
