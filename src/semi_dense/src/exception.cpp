#include "semi_dense/exception.hpp"

Exception::Exception(std::string name = ""):std::exception(), name_(name){}

virtual const char* Exception::what() const noexcept
{
    return name_.c_str();
}