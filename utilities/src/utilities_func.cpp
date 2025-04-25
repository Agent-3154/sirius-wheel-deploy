//
// Created by lingwei on 4/3/24.
//
#include "../inc/utilities_fun.h"
#include <assert.h>

std::string getLcmUrl(int64_t ttl) {
    assert(ttl >= 0 && ttl <= 255);
    return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);

}