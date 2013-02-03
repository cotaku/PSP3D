#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <malloc.h>

template<class T>
class vector
{
public:
    vector()
    {
        _capacity = 16;
        _pdata = (T*)malloc(_capacity*sizeof(T));
        _last = 0;
    }

    vector(int n)
    {
        _capacity = n - 1;
        _capacity |= _capacity >> 1;
        _capacity |= _capacity >> 2;
        _capacity |= _capacity >> 4;
        _capacity |= _capacity >> 8;
        _capacity |= _capacity >> 16;
        _capacity++;
        _pdata = (T*)malloc(_capacity*sizeof(T));
        _last = 0;
    }

    ~vector()
    {
        if (_pdata != NULL)
            free(_pdata);
    }

    int size() { return _last; }

    bool push_back(const T& x)
    {
        if (_capacity > _last + 1)
        {
            _pdata[_last] = x;
            _last++;
        }
        else
            return false;

        if (_capacity == _last + 1)    // reach the end, reallocate
            reallocate();

        return true;
    }

    void pop_back()
    {
        if (_last > 0)
            _last--;
    }

    bool insert(int index, const T& x)
    {
        if (_capacity > _last + 1)
        {
            memcpy(_pdata+index+1, _pdata+index, (_last-index)*sizeof(T));
            _pdata[index] = x;
            _last++;
        }
        else
            return false;

        if (_capacity == _last + 1)    // reach the end, reallocate
            reallocate();

        return true;
    }

    void erase(int index)
    {
        memcpy(_pdata+index, _pdata+index+1, (_last-index-1)*sizeof(T));
        _last--;
    }

    void clear()
    {
        _last = 0;
    }

    T& operator[](int index)
    {
        return _pdata[index];
    }

private:
    void reallocate()
    {
        _capacity = _capacity << 1;
        T* old = _pdata;
        _pdata = (T*)malloc(_capacity*sizeof(T));
        if (_pdata != NULL)
        {
            memcpy(_pdata, old, (_last+1)*sizeof(T));
            if (old != NULL)
                free(old);
        }
    }

    T* _pdata;
    int _capacity;
    int _last;
};

#endif
