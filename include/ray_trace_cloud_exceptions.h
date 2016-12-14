#ifndef RAY_TRACE_CLOUD_EXCEPTIONS_H
#define RAY_TRACE_CLOUD_EXCEPTIONS_H

#include <exception>

namespace ih
{

/*!
 * Exception thrown when a mesh is missing or not defined
 */
class MeshLoadException : std::exception
{
	/*!
	 * Function explaining what happened
	 */
    virtual const char* what() const throw()
    {
        return "There is no defined mesh to generate point clouds from";
    }
};

} // End of namespace

#endif // RAY_TRACE_CLOUD_EXCEPTIONS_H
