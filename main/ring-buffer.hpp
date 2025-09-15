#pragma once

#include <array>

template<typename T, int SIZE>
class RingBuffer
{
public:
  const int size = SIZE;
  std::array<T, SIZE> buf = { 0 };

private:
  int idx = 0;

public:
  RingBuffer() {};
  ~RingBuffer() {};

  void push(T item)
  {
    buf[idx] = item;
    // Increment index wrapping back to the beginning.
    idx = (idx >= SIZE - 1) ? 0 : (idx + 1);
  };

  T next()
  {
    T item = buf[idx];
    // Increment index wrapping back to the beginning.
    idx = (idx >= SIZE - 1) ? 0 : (idx + 1);
    return item;
  }
};