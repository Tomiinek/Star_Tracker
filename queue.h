/*
 * queue.h 
 * By Steven de Salas
 */

#ifndef QUEUE_H
#define QUEUE_H

#include <Arduino.h>

template<class T>
class queue {
  private:
    int _front, _back, _count;
    T *_data;
    int _maxitems;
  public:
    queue(int maxitems = 256) { 
      _front = 0;
      _back = 0;
      _count = 0;
      _maxitems = maxitems;
      _data = new T[maxitems + 1];   
    }
    ~queue() {
      delete[] _data;  
    }
    inline int count();
    inline int front();
    inline int back();
    void push(const T &item);
    T peek();
    T pop();
    void clear();
};

template<class T>
inline int queue<T>::count() 
{
  return _count;
}

template<class T>
inline int queue<T>::front() 
{
  return _front;
}

template<class T>
inline int queue<T>::back() 
{
  return _back;
}

template<class T>
void queue<T>::push(const T &item)
{
  if(_count < _maxitems) { // Drops out when full
    _data[_back++]=item;
    ++_count;
    // Check wrap around
    if (_back > _maxitems)
      _back -= (_maxitems + 1);
  }
}

template<class T>
T queue<T>::pop() {
  if(_count <= 0) return T(); // Returns empty
  else {
    T result = _data[_front];
    _front++;
    --_count;
    // Check wrap around
    if (_front > _maxitems) 
      _front -= (_maxitems + 1);
    return result; 
  }
}

template<class T>
T queue<T>::peek() {
  if(_count <= 0) return T(); // Returns empty
  else return _data[_front];
}

template<class T>
void queue<T>::clear() 
{
  _front = _back;
  _count = 0;
}

#endif