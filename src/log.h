// vim: et ts=4 sts=4 sw=4
#pragma once


#define LOG(fmt, ...)\
    fprintf(stderr, "%s:%d: %s :: " fmt "\n",\
    __FILE__, __LINE__, __PRETTY_FUNCTION__, ##__VA_ARGS__)
