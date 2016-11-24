#ifndef RAY_TRACE_CLOUD_EXCEPTIONS_H
#define RAY_TRACE_CLOUD_EXCEPTIONS_H

#include <exception>

class MeshLoadException : std::exception
{
    virtual const char* what() const throw()
    {
        return "There is no defined mesh to generate point clouds from";
    }
};

#endif // RAY_TRACE_CLOUD_EXCEPTIONS_H
