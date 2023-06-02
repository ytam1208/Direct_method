#ifndef __MYEXCEPTION__
#define __MYEXCEPTION__

#include <iostream>
#include <vector>
#include <exception>
#include <string>

namespace MY
{
    class Exception : public std::exception{
    protected:
        std::string name_;
    public:
        Exception(std::string name = "") : std::exception(), name_(name) {}
        virtual ~Exception() throw() {}
        virtual const char* what() const noexcept { return name_.c_str(); }
    };
}

#endif