#ifndef __TIMER_H
#define __TIMER_H
#pragma once

#include <chrono>

std::chrono::high_resolution_clock::time_point start;
std::chrono::high_resolution_clock::time_point stop;

void tick(){
    start = std::chrono::high_resolution_clock::now();
}

unsigned long tock(){
    stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    return duration.count();
}

#endif