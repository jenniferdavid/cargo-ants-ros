// g++ -Wall -o xfig2world xfig2world.cpp

#include <iostream>
#include <sstream>
//#include <fstream>

using namespace std;


int main(int argc, char ** argv)
{
  static double const scale(1e-2);
  istream & is(cin);
  ostream & os(cout);
  string line;
  
  os << "name world\n";
  while (getline(is, line))
    if ((line.size() > 1) && ('2' == line[0]) && (' ' == line[1])) {
      while (getline(is, line)) {
	if ('\t' != line[0])
	  break;
	istringstream iss(line);
	double x0, y0;
	iss >> x0 >> y0;
	if ( ! iss)
	  break;
	x0 *= scale;
	y0 *= scale;
	while (true) {
	  double x1, y1;
	  iss >> x1 >> y1;
	  if ( ! iss)
	    break;
	  x1 *= scale;
	  y1 *= scale;
	  os << "line " << x0 << " " << y0 << " " << x1 << " " << y1 << "\n";
	  x0 = x1;
	  y0 = y1;
	}
      }
    }
}
