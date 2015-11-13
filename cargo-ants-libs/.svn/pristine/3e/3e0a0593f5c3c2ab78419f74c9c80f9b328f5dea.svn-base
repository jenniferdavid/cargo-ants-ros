#include <npm/Plugin.hpp>
#include <npm/Factory.hpp>
#include <npm/ext/Zombie.hpp>
#include <iostream>


int npm_plugin_init ()
{
  npm::Factory::Instance().declare<npm::LidarZombie>("testZombie");
  return 0;
}


void npm_plugin_fini (void)
{
  std::cout << "Byebye from the test plugin!\n";
}
