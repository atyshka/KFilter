#include <stdexcept>

class KFilterException: public std::runtime_error
{
    public:
        KFilterException(const char* msg) : runtime_error(msg)
        {}
};

class InvalidDimensionException: public KFilterException 
{
    public:
        InvalidDimensionException() : KFilterException("Error: attempting to set matrix with invalid dimensions for this filter")
        {}
};