#pragma once

#include <stdint.h>
#include <iostream>
class CircleBuffer
{
private:
    const size_t size_;
public:
    CircleBuffer(const size_t size);
    ~CircleBuffer();
    void Reset();
    bool IsEmpty();
    size_t GetCapacity();
    size_t GetSize();

    uint8_t* buffer;
    size_t begin;
    size_t end;
    
    bool isFull;
};

CircleBuffer::CircleBuffer(const size_t size): size_(size) {
    buffer = new uint8_t[size_];
    Reset();
}

CircleBuffer::~CircleBuffer() {
    delete[] buffer;
}

void CircleBuffer::Reset() {
    begin = 0;
    end = 0;
    isFull = false;
}

bool CircleBuffer::IsEmpty() {
    return (!isFull && (begin == end));
}

// return maximum number of elements in buffer;
size_t CircleBuffer::GetCapacity(){
    return size_;
}

// return number of elements in buffer;
size_t CircleBuffer::GetSize(){
    if (isFull){
        return size_;
    }
    if ( begin >= end ){
        return begin - end;
    } else {
        return size_ + begin - end;
    }

}