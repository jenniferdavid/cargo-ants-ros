#include <npm/Plugin.hpp>
#include <iostream>

using namespace std;


int main (int argc, char ** argv)
{
  if (argc < 2) {
    cout << "Please specify some plugin file names on the command line\n";
    return 42;
  }
  for (int ii (1); ii < argc; ++ii) {
    npm::Plugin plugin(argv[ii]);
    if (plugin.load (argv[ii], cout)) {
      cout << "loaded " << argv[ii] << "\n";
    }
  }
  return 0;
}
