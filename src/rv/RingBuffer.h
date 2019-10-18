/** \brief ring buffer with predefined capacity
 *
 *  a ring buffer offers push_back and push_front mechanism
 *  to add elements with a predefined capacity, i.e.,the
 *  buffer size will never be larger than capacity.
 *  Thus elements at the end or the beginning are dropped,
 *  if the capacity is reached. The order of elements is
 *  preserved.
 *
 *
 *  \author behley
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <stdint.h>
#include <vector>
#include <cassert>
#include <iostream>

namespace rv
{

template<typename T>
class RingBuffer
{
  public:
    /** \brief initialize a buffer of capacity c **/
    RingBuffer(uint32_t c) :
      mStart(0), mCapacity(c), mSize(0), mData(c)
    {
    }

    /** \brief access the i-th element
     *  \param index of element in the buffer.
     **/
    T& operator[](uint32_t i);
    const T& operator[](uint32_t i) const;

    /** \brief insert an element at the end, but this might drop an element at the front. **/
    void push_back(const T& value);

    /** \brief insert an element at the front, but this might drop an element at the end. **/
    void push_front(const T& value);

    /** \brief number of items in buffer **/
    uint32_t size() const
    {
      return mSize;
    }

    /** \brief capacity. **/
    uint32_t capacity() const
    {
      return mCapacity;
    }

    /** \brief remove all elements from the buffer. **/
    void clear()
    {
      mStart = 0;
      mSize = 0;
    }

    friend std::ostream& operator<<(std::ostream& out, const RingBuffer& buffer)
    {
      out << "[";
      if (buffer.mSize > 0) out << buffer.mData[buffer.mStart];
      for (uint32_t i = 1; i < buffer.mSize; ++i)
      {
        out << ", " << buffer.mData[(i + buffer.mStart) % buffer.mCapacity];
      }
      out << "]";

      return out;
    }

  private:
    uint32_t mStart;
    uint32_t mCapacity;
    uint32_t mSize;
    std::vector<T> mData;
};

template<typename T>
T& RingBuffer<T>::operator[](uint32_t i)
{
  assert(i < mSize);
  return mData[(i + mStart) % mCapacity];
}

template<typename T>
const T& RingBuffer<T>::operator[](uint32_t i) const
{
  assert(i < mSize);
  return mData[(i + mStart) % mCapacity];
}

template<typename T>
void RingBuffer<T>::push_back(const T& value)
{
  mData[(mStart + mSize) % mCapacity] = value;
  if (mSize == mCapacity) mStart = (mStart + 1) % mCapacity;
  if (mSize < mCapacity) ++mSize;
}

template<typename T>
void RingBuffer<T>::push_front(const T& value)
{
  mStart = (mStart - 1 + mCapacity) % mCapacity;
  mData[mStart] = value;
  if (mSize < mCapacity) ++mSize;
}

}
#endif /* RINGBUFFER_H_ */
