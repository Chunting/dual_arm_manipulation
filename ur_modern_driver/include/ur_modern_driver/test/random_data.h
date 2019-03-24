#pragma once
#include <inttypes.h>
#include <algorithm>
#include <bitset>
#include <climits>
#include <cstddef>
#include <functional>
#include <random>
#include "ur_modern_driver/bin_parser.h"

class RandomDataTest
{
private:
  using random_bytes_engine = std::independent_bits_engine<std::default_random_engine, CHAR_BIT, uint8_t>;
  uint8_t* buf_;
  BinParser bp_;
  size_t n_;

public:
  RandomDataTest(size_t n) : buf_(new uint8_t[n]), bp_(buf_, n), n_(n)
  {
    random_bytes_engine rbe;
    std::generate(buf_, buf_ + n, std::ref(rbe));
  }

  ~RandomDataTest()
  {
    delete buf_;
  }

  BinParser getParser(bool skip = false)
  {
    return BinParser(buf_, n_ - (skip ? sizeof(int32_t) : 0));
  }

  template <typename T>
  T getNext()
  {
    T actual;
    bp_.parse(actual);
    return actual;
  }

  template <typename T, size_t N>
  std::bitset<N> getNext()
  {
    T actual;
    bp_.parse(actual);
    return std::bitset<N>(actual);
  }

  template <typename T>
  void set(T data, size_t pos)
  {
    std::memcpy(&data, buf_ + pos, sizeof(T));
  }

  void skip(size_t n)
  {
    bp_.consume(n);
  }
};