// g++ -o tpthread -Wall -I/Users/rolo/local-sfl/include tpthread.cpp -L/Users/rolo/local-sfl/lib -lsunflower

#include <sfl/util/Pthread.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <unistd.h>		// for usleep on OpenBSD

using namespace sfl;
using namespace boost;
using namespace std;

static const unsigned int usecsleep(50000);

class Thread: public SimpleThread {
public:
  Thread(const string & _msg): SimpleThread("Thread " + _msg), msg(_msg) {}
  ~Thread() { cerr << " DTOR " << msg << "\n"; }
  void Step() { cerr << msg; }
  const string msg;
};

int main(int argc, char ** argv)
{
  vector<shared_ptr<Thread> > thread;
  for(char cc('a'); cc < 'd'; ++cc){
    cerr << " STARTING " << cc << "\n";
    const char msg[] = { cc, '\0' };
    thread.push_back(shared_ptr<Thread>(new Thread(msg)));
    thread.back()->Start(usecsleep);
    usleep(5*usecsleep);
  }
  for(size_t ii(0); ii < thread.size(); ++ii){
    cerr << " STOPPING " << thread[ii]->msg << "\n";
    thread[ii]->Stop();
    usleep(5*usecsleep);
  }
  cerr << " CLEARING THREADS\n";
  thread.clear();;
  usleep(5*usecsleep);
  cerr << " BYE BYE\n";
}
