#include <iostream>
#include "hash_map.h"
#include "network_app.h"

int main(int argc, char** argv)
{
  using ohantsev::NetworkApp;
  using ohantsev::HashMap;
  using ohantsev::Graph;

  if (argc < 2)
  {
    std::cout << "Empty filename\n";
    return 1;
  }
  HashMap< std::string, Graph< std::string > > networks;
  NetworkApp app(networks, std::cin, std::cout);
  try
  {
    app.input(std::string(argv[1]));
  }
  catch (const std::exception& e)
  {
    std::cout << e.what() << "\n";
    return 1;
  }
  while (!std::cin.eof())
  {
    app();
  }
}
