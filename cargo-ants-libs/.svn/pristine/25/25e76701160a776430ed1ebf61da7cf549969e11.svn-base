#include "Pthread.hpp"
#include <iostream>
#include <unistd.h>		// for usleep on OpenBSD

using namespace sfl;
using namespace boost;
using namespace std;

class Thread: public SimpleThread {
  shared_ptr<Mutex> m_mutex;
  const size_t m_count;
  
public:
  Thread(const std::string & name, shared_ptr<Mutex> mutex, size_t count)
    : SimpleThread(name), m_mutex(mutex), m_count(count) {}
  
  void Step() {
    m_mutex->Lock();
    for(size_t ii(0); ii < m_count; ++ii)
      cout << name;
    cout << "\n";
    m_mutex->Unlock();
  }
};

int main(int argc, char ** argv)
{
  shared_ptr<Mutex> mutex(Mutex::Create("foo"));
  if( ! mutex){
    cout << "ERROR creating mutex\n";
    return 1;
  }
  Thread aa("a", mutex, 10);
  Thread bb("b", mutex, 20);
  mutex->Lock();
  cout << "main: starting threads\n";
  aa.Start(100000);
  bb.Start(1000000);
  mutex->Unlock();
  while(true){
    usleep(2000000);
    mutex->Lock();
    cout << "main is also here\n";
    mutex->Unlock();
  }
}
