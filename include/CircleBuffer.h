#pragma once

#include <stdint.h>
#include <iostream>
#include <string>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class CircleBuffer {
private:
    const size_t size_;
    uint8_t* buffer_;
    size_t begin_;
    size_t end_;
    bool isFull_;
    
    void GetFloat(float *fDst);

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

    // Special functions
    int GetCommand();
    void GetVector3(geometry_msgs::Vector3 &v);
    void GetQuaternion(geometry_msgs::Quaternion &q);

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
    std::cout << "buffer: ";
    for (unsigned long i = 0; i < GetSize(); ++i){
        unsigned long id = (end_ + i) % size_;
        std::cout << +buffer_[id] << " ";
    }
    std::cout << std::endl;
}

int CircleBuffer::GetCommand(){  
    if (IsEmpty()){
        return -1;
    }  

    uint8_t fhead = GetItem();
    uint8_t shead = PredictNext();
    if ((fhead == 255) && (shead == 255)){
        GetItem();
    } else {
        return -1;
    }
    
    // Get message length
    uint16_t msgLen = 0;
    GetData((uint8_t*)(&msgLen), 2);

    // Get message command code
    uint16_t cmdCode = 0;
    GetData((uint8_t*)(&cmdCode), 2);

    return cmdCode;
}

void CircleBuffer::GetFloat(float *fDst){
    GetData((uint8_t*)fDst, 4);
}

void CircleBuffer::GetVector3(geometry_msgs::Vector3 &v){
    float f;
    GetFloat(&f); v.x = f;
    GetFloat(&f); v.y = f;
    GetFloat(&f); v.z = f;
}

void CircleBuffer::GetQuaternion(geometry_msgs::Quaternion &q){
    float roll; GetFloat(&roll);
    float pitch; GetFloat(&pitch);
    float yaw; GetFloat(&yaw);
    
    const float pi = 3.1415926f;
    double cy = cos(yaw*pi/180.0f * 0.5);
    double sy = sin(yaw*pi/180.0f * 0.5);
    double cp = cos(pitch*pi/180.0f * 0.5);
    double sp = sin(pitch*pi/180.0f * 0.5);
    double cr = cos(roll*pi/180.0f * 0.5);
    double sr = sin(roll*pi/180.0f * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}