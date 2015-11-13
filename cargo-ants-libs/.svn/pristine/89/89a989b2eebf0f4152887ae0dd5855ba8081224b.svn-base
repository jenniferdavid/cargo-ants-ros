#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
using namespace boost;
using namespace std;

int main(int argc, char ** argv)
{
  vector<shared_ptr<string> > line;
  for(int ii(0); ii < argc; ++ii){
    ostringstream os;
    os << "  " << ii << " is " << argv[ii] << "\n";
    line.push_back(shared_ptr<string>(new string(os.str())));
  }
  cout << "Hello, my name is " << argv[0] << " and I have " << argc
       << " arguments:\n";
  for(size_t ii(0); ii < line.size(); ++ii)
    cout << *line[ii];
}
