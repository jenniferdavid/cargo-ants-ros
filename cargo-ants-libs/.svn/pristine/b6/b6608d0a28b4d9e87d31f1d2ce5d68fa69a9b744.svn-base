#include <sfl/util/GoalManager.hpp>
#include <iostream>
#include <sstream>

int main(int argc, char ** argv)
{
  {
    sfl::GoalManager mgr;
    std::istringstream cfg("blah");
    if(mgr.ParseConfig(cfg, 0)){
      std::cout << "ParseConfig() should have failed in test 1\n";
      exit(EXIT_FAILURE);
    }
    std::cout << "passed test 1\n";
  }
  {
    sfl::GoalManager mgr;
    std::istringstream cfg("repeat none\n"
			   "Goal 1  1  1  0.1 3.8\n"
			   "Goal 10 10 10 0.1 3.8\n");
    if( ! mgr.ParseConfig(cfg, &std::cout)){
      std::cout << "ParseConfig() should have passed in test 2\n";
      exit(EXIT_FAILURE);
    }
    std::cout << "passed config test 2\n";
    if( ! mgr.GetCurrentGoal()){
      std::cout << "GetCurrentGoal() A failed in test 2\n";
      exit(EXIT_FAILURE);
    }
    std::cout << "passed current goal A test 2\n";
    if( ! mgr.GoalReached(1, 1.05, 0, true)){
      std::cout << "GoalReached() A failed in test 2\n";
      exit(EXIT_FAILURE);
    }
    std::cout << "passed goal A test 2\n";
    if(mgr.GoalReached(1, 3, 0, false)){
      std::cout << "GoalReached() B failed in test 2\n";
      exit(EXIT_FAILURE);
    }
    std::cout << "passed goal B test 2\n";
    mgr.NextGoal();
    if(mgr.GoalReached(1, 1.05, 0, true)){
      std::cout << "GoalReached() C failed in test 2\n";
      exit(EXIT_FAILURE);
    }
    std::cout << "passed goal C test 2\n";
    mgr.NextGoal();
    if(mgr.GetCurrentGoal()){
      std::cout << "GetCurrentGoal() B failed in test 2\n";
      exit(EXIT_FAILURE);
    }
    std::cout << "passed current goal B test 2\n";
  }
  std::cout << "PASSED ALL\n";
}
