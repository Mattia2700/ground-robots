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
    inline void decreaseValue(const ItemType& item, const ValueType& newValue);
    inline bool contains(const ItemType& item) const;
    inline ValueType getValue(const ItemType& item) const;

    inline size_t size() const { return heap.size(); }
    inline bool empty() const { return size()==0; }
    
    inline void print() const;
    
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
    std::unordered_map<ItemType, size_t> currentItemPosition;

    inline size_t parent(size_t pos) {
        return (pos-1)/2;
    }
    inline size_t leftChild(size_t pos) {
        return 2*pos+1;
    }
    inline size_t rightChild(size_t pos) {
        return 2*pos+2;
    }
    inline ValueType minChild(size_t pos, size_t& minPos) {
        size_t left = leftChild(pos);
        size_t right = rightChild(pos);
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
    size_t sz = heap.size();
    heap.resize(sz+1);
    size_t currPos = sz;
    size_t parentPos = parent(currPos);

    while (currPos!=0 && heap[parentPos].value>value) {
        heap[currPos] = heap[parentPos];
        currentItemPosition[heap[currPos].item] = currPos;

        currPos = parentPos;
        parentPos = parent(currPos);
    }

    heap[currPos].item = item;
    heap[currPos].value = value;
    currentItemPosition[item] = currPos;
}

template <typename ItemType>
typename MinHeap<ItemType>::ValueType MinHeap<ItemType>::extractMin(ItemType& item) {
    if (empty()) {
        return 0;
    }

    item = heap[0].item;
    ValueType result = heap[0].value;
    currentItemPosition.erase(item);


    size_t currPos = 0;
    ItemType newItem = heap.back().item;
    ValueType newValue = heap.back().value;
    heap.pop_back();

    if (heap.empty())
        return result;

    size_t childMin;
    ValueType valChildMin;
    valChildMin = minChild(currPos, childMin);

    while (childMin!=currPos && valChildMin<newValue) {
        heap[currPos] = heap[childMin];
        currentItemPosition[heap[currPos].item] = currPos;
        currPos = childMin;
        valChildMin = minChild(currPos, childMin);
    }
    heap[currPos].item = newItem;
    heap[currPos].value = newValue;
    currentItemPosition[newItem] = currPos;

    return result;
}

template <typename ItemType>
void MinHeap<ItemType>::decreaseValue(const ItemType &item, const ValueType &newValue) {
    size_t currPos = currentItemPosition[item];

    size_t parentPos = parent(currPos);
    while (currPos!=0 && heap[parentPos].value>newValue) {
        heap[currPos] = heap[parentPos];
        currentItemPosition[heap[currPos].item] = currPos;
        currPos = parentPos;
        parentPos = parent(currPos);
    }

    heap[currPos].item = item;
    heap[currPos].value = newValue;
    currentItemPosition[item] = currPos;
}

template <typename ItemType>
bool MinHeap<ItemType>::contains(const ItemType& item) const {
    return currentItemPosition.find(item) != currentItemPosition.end();
}

template <typename ItemType>
typename MinHeap<ItemType>::ValueType MinHeap<ItemType>::getValue(const ItemType &item) const {
    size_t pos = currentItemPosition.at(item);
    return heap[pos].value;
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


