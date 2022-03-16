//
// Created by paolo on 20/03/18.
//

#ifndef MINHEAP_H
#define MINHEAP_H

#include <vector>
#include <memory>
#include <cstddef>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <cassert>
#include <cstring>

#undef max
#include <limits>

typedef size_t uidx_type;

template <typename ItemType>
class MinHeap {
public:
  typedef double ValueType;

  inline MinHeap():
    heap() 
  {}

  inline MinHeap(int maxsize):
    heap() 
  {
    heap.reserve(maxsize);
  }

  inline MinHeap(const MinHeap& other) = default;
  virtual ~MinHeap() = default;

  inline void insert(const ItemType& item, const ValueType& value);
  inline ValueType extractMin(ItemType& item);

  inline uidx_type size() const { return heap.size(); }
  inline bool empty() const { return size()==0; }
    
  inline void print() const;
  inline uidx_type capacity() const { return heap.capacity(); }
    
private:
  struct Entry {
    ItemType  item;
    ValueType value;
    Entry() = default;
    Entry(const ItemType& item, const ValueType& value):
      item(item), value(value) {}
    Entry(const Entry& other) = default;
    virtual ~Entry() = default;
  };

  std::vector<Entry> heap;

  inline uidx_type parent(uidx_type pos) {
    return (pos-1)/2;
  }
  inline uidx_type leftChild(uidx_type pos) {
    return 2*pos+1;
  }
  inline uidx_type rightChild(uidx_type pos) {
    return 2*pos+2;
  }
  inline ValueType minChild(uidx_type pos, uidx_type& minPos) {
    uidx_type left = leftChild(pos);
    uidx_type right = rightChild(pos);
    if (left>=size()) {
      minPos = pos;
      return std::numeric_limits<ValueType>::max();
    }
    if (right>=size()) {
      minPos = left;
      return heap[minPos].value;
    }
    minPos = heap[left].value <= heap[right].value ? left : right;
    return heap[minPos].value;
  }

};

template <typename ItemType>
void MinHeap<ItemType>::insert(const ItemType& item, const ValueType& value) {
  uidx_type sz = heap.size();
  heap.resize(sz+1);
  uidx_type currPos = sz;
  uidx_type parentPos = parent(currPos);

  while (currPos!=0 && heap[parentPos].value>value) {
    heap[currPos] = heap[parentPos];
    currPos = parentPos;
    parentPos = parent(currPos);
  }

  heap[currPos].item = item;
  heap[currPos].value = value;
}

template <typename ItemType>
typename MinHeap<ItemType>::ValueType MinHeap<ItemType>::extractMin(ItemType& item) {
  if (empty()) {
    return 0;
  }

  item = heap[0].item;
  ValueType result = heap[0].value;    

  uidx_type currPos = 0;
  ItemType newItem = heap.back().item;
  ValueType newValue = heap.back().value;
  heap.pop_back();

  if (heap.empty())
    return result;

  uidx_type childMin;
  ValueType valChildMin;
  valChildMin = minChild(currPos, childMin);

  while (childMin!=currPos && valChildMin<newValue) {
    heap[currPos] = heap[childMin];
    currPos = childMin;
    valChildMin = minChild(currPos, childMin);
  }
  heap[currPos].item = newItem;
  heap[currPos].value = newValue;

  return result;
}


template <typename ItemType>
void MinHeap<ItemType>::print() const {
  std::cout << "######################################" << std::endl;
  for (int i=0; i<size(); ++i) {
    std::cout << "(" << heap[i].value << ")" << " " 
              << heap[i].item << std::endl;
  }
  std::cout << "######################################" << std::endl;
}

#endif


