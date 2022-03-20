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
    bool isFull_;

public:
    CircleBuffer(const size_t size);
    ~CircleBuffer();
    void Reset();
    bool IsEmpty() const;
    size_t GetCapacity() const;
    size_t GetSize() const;
    void PrintBuffer() const;

    void AddItem(uint8_t data);
    void AddData(uint8_t* pSrc, unsigned long len);
    bool IsFull() const;

    uint8_t GetItem();
    void GetData(uint8_t* pDst, unsigned long len);

    uint8_t PredictNext() const{
        return buffer_[end_];
    }

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
    isFull_ = false;
}

bool CircleBuffer::IsEmpty() const {
    return (!isFull_ && (begin_ == end_));
}

// return maximum number of elements in buffer;
size_t CircleBuffer::GetCapacity() const {
    return size_;
}

// return number of elements in buffer;
size_t CircleBuffer::GetSize() const {
    if (isFull_){
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
    if (isFull_){
        end_ = (end_ + 1) % size_;
    }
    begin_ = (begin_ + 1) % size_;
    isFull_ = begin_ == end_;
}

void CircleBuffer::AddData(uint8_t* pData, unsigned long len){
    for (unsigned long i = 0; i < len; ++i){
        AddItem(pData[i]);
    }
}

uint8_t CircleBuffer::GetItem(){
    uint8_t res = buffer_[end_];
    isFull_ = false;
    end_ = (end_ + 1) % size_;
    return res;
}

void CircleBuffer::GetData(uint8_t* pDst, unsigned long len){
    for (unsigned long i = 0; i < len; ++i){
        pDst[i] = GetItem();
    }
}

bool CircleBuffer::IsFull() const{
    return isFull_;
}

void CircleBuffer::PrintBuffer() const{
    std::cout << "Print: ";
    for (unsigned long i = 0; i < GetSize(); ++i){
        unsigned long id = (end_ + i) % size_;
        std::cout << +buffer_[id] << " ";
    }
    std::cout << std::endl;
}
