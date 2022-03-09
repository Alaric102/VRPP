#pragma once

#include <stdint.h>
#include <iostream>
#include <string>
class CircleBuffer
{
private:
    const size_t size_;
    uint8_t* buffer_;
    size_t begin_;
    size_t end_;

    void AddItem(uint8_t data);
    uint8_t GetItem();
public:
    CircleBuffer(const size_t size);
    ~CircleBuffer();
    void Reset();
    bool IsEmpty() const;
    size_t GetCapacity() const;
    size_t GetSize() const;
    void InsertData(uint8_t* pSrc, unsigned long len);
    uint8_t GetData(uint8_t* pDst, unsigned long len);

    bool isFull;
};

CircleBuffer::CircleBuffer(const size_t size): size_(size) {
    buffer_ = new uint8_t[size_];
    Reset();
}

CircleBuffer::~CircleBuffer() {
    delete[] buffer_;
}

void CircleBuffer::Reset() {
    begin_ = 0;
    end_ = 0;
    isFull = false;
}

bool CircleBuffer::IsEmpty() const {
    return (!isFull && (begin_ == end_));
}

// return maximum number of elements in buffer;
size_t CircleBuffer::GetCapacity() const {
    return size_;
}

// return number of elements in buffer;
size_t CircleBuffer::GetSize() const {
    if (isFull){
        return size_;
    }
    if ( begin_ >= end_ ){
        return begin_ - end_;
    } else {
        return size_ + begin_ - end_;
    }
}

void CircleBuffer::AddItem(uint8_t data){
    buffer_[begin_] = data;
    if (isFull){
        end_ = (end_ + 1) % size_;
    }
    begin_ = (begin_ + 1) % size_;
    isFull = begin_ == end_;
}

void CircleBuffer::InsertData(uint8_t* pData, unsigned long len){
    for (unsigned long i = 0; i < len; ++i){
        AddItem(pData[i]);
    }
}

uint8_t CircleBuffer::GetItem(){
    uint8_t res = buffer_[end_];
    isFull = false;
    end_ = (end_ + 1) % size_;
    return res;
}

uint8_t CircleBuffer::GetData(uint8_t* pDst, unsigned long len){
    for (unsigned long i = 0; i < len; ++i){
        pDst[i] = GetItem();
    }
}