#ifndef __ITIA_BUTILS__
#define __ITIA_BUTILS__

#include <mutex>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/thread/condition.hpp>

namespace itia
{
namespace butils
{
// Thread safe circular buffer 
template <typename T>
class circ_buffer : private boost::noncopyable
{
public:
    typedef boost::mutex::scoped_lock lock;
    circ_buffer() {}
    ~circ_buffer() {}
    circ_buffer(int n) {cb.set_capacity(n);}
    void push_back (T imdata) {
      lock lk(monitor);
      cb.push_back(imdata);
      buffer_not_empty.notify_one();
    }
    const T& front() {
      lock lk(monitor);
      while (cb.empty())
          buffer_not_empty.wait(lk);
      return cb.front();
    }
    void pop_front() {
      lock lk(monitor);
      if(cb.empty())
          return;
      return cb.pop_front();
    }
    void clear() {
        lock lk(monitor);
        cb.clear();
    }
    int size() {
        lock lk(monitor);
        return cb.size();
    }
    void set_capacity(int capacity) {
        lock lk(monitor);
        cb.set_capacity(capacity);
    }
    bool empty() {
        lock lk(monitor);
        return cb.empty();
    }
    bool full() {
        lock lk(monitor);
        return cb.full();
    }
    boost::circular_buffer<T>& get() {
      return cb;
    }
private:
    boost::condition buffer_not_empty;
    boost::mutex monitor;
    boost::circular_buffer<T> cb;
};

};
};

#endif
