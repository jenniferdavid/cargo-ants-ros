#include "ringbuf.hpp"
#include <iostream>

using namespace sfl;
using namespace std;

void dump(const ringbuf<int> & buf, ostream & os)
{
  os << "size " << buf.size() << ":";
  if(0 == buf.size())
    os << " no data";
  else
    for(size_t ii(0); ii < buf.size(); ++ii)
      os << " " << buf[ii];
  os << "\n";
}

int main(int argc, char ** argv)
{
  static const size_t length(8);
  ringbuf<int> buf(length);
  for(int ii(0); ii < static_cast<int>(2 * length); ++ii){
    dump(buf, cout);
    buf.push_back(ii);
  }
  dump(buf, cout);
}
