/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    singleton.hpp
 * @brief   Template class for Singleton Pattern .
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-06-25
 * @version 1.0
 * @note
 * @history
**/

#ifndef _SINGLETON_HPP_
#define _SINGLETON_HPP_

namespace YPattern {
  template <typename T>
  class Singleton {
  public:
    static T * Get() {if (instance_==nullptr) instance_=new T; return instance_;};
  private:
    class Garbo {
    public:
      ~Garbo() {delete Singleton<T>::instance_;};
    };
  private:
    static T *instance_;
    static Garbo garbo_;
  };

  template <typename T>
  T * Singleton<T>::instance_ = nullptr;
}
#endif
