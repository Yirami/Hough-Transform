/**
 * Copyright (C) 2019 Yirami .
 * All rights reserved.
 * @file    timer.hpp
 * @brief   Template class for time meter .
 * @author  [Tang zhiyong](https://yirami.xyz)
 * @date    2019-08-16
 * @version 1.0
 * @note
 * @history
**/

#ifndef _TIMER_HPP_
#define _TIMER_HPP_

namespace YUtils {

#ifdef __linux__
  #include <sys/time.h>

  template <typename T>
  class Timer {
  public:
    Timer() {};
    void Start() {gettimeofday(&start_time_, NULL);};
    void Stop();
    void Restart() {elapsed_ = 0; Start();};
    T Elapsed_s() {return static_cast<T>(elapsed_)/(T)1e6;};
    T Elapsed_ms() {return static_cast<T>(elapsed_)/(T)1e3;};
    T Elapsed_us() {return static_cast<T>(elapsed_);};
  private:
    struct timeval start_time_;
    long long elapsed_ = 0;
  };

  template <typename T>
  void Timer<T>::Stop() {
    struct timeval stop_time;
    gettimeofday(&stop_time, NULL);
    elapsed_ += 1000000*(stop_time.tv_sec-start_time_.tv_sec)+stop_time.tv_usec-start_time_.tv_usec;
  }

#else
  #include <windows.h>

  template <typename T>
  class Timer {
  public:
    Timer() {QueryPerformanceFrequency(&freq_);};
    void Start() {QueryPerformanceCounter(&start_time_);};
    void Stop();
    void Restart() {elapsed_ = 0; Start();};
    T Elapsed_s() {return static_cast<T>(elapsed_)/(T)1e6;};
    T Elapsed_ms() {return static_cast<T>(elapsed_)/(T)1e3;};
    T Elapsed_us() {return static_cast<T>(elapsed_);};
  private:
    LARGE_INTEGER freq_ = {}, start_time_ = {};
    long long elapsed_ = 0;
  };

  template <typename T>
  void Timer<T>::Stop() {
    LARGE_INTEGER stop_time;
    QueryPerformanceCounter(&stop_time);
    elapsed_ += (stop_time.QuadPart-start_time_.QuadPart)*1000000/freq_.QuadPart;
  }

#endif
}
#endif
