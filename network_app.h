#ifndef NETWORK_H
#define NETWORK_H
#include <string>
#include <iosfwd>
#include "graph.h"
#include "hash_map.h"
#include "command_handler.h"

namespace ohantsev
{
  class NetworkApp: public CommandHandler
  {
  public:
    using graph_type = Graph< std::string >;
    using map_type = HashMap< std::string, graph_type >;
    using outIter = std::ostream_iterator< std::string >;

    NetworkApp(map_type& networks, std::istream& in, std::ostream& out);
    void operator()() override;
    void input(const std::string& filename);

  private:
    map_type& networks_;

    static void create(map_type& networks, std::istream& in);
    static void deleteNetwork(map_type& networks, std::istream& in);
    static void showAll(const map_type& networks, std::ostream& out);
    static void showNetwork(const map_type& networks, std::istream& in, std::ostream& out);
    static void addDevice(map_type& networks, std::istream& in);
    static void connect(map_type& networks, std::istream& in);
    static void disconnect(map_type& networks, std::istream& in);
    static void deleteDevice(map_type& networks, std::istream& in);
    static void forceDeleteDevice(map_type& networks, std::istream& in);
    static void copy(map_type& networks, std::istream& in);
    static void removeLoops(map_type& networks, std::istream& in);
    static void removeLoopsNew(map_type& networks, std::istream& in);
    static void distance(const map_type& networks, std::istream& in, std::ostream& out);
    template < bool AllowCycles >
    static void topPaths(const map_type& networks, std::istream& in, std::ostream& out);
    static void topPathsWithCycles(const map_type& networks, std::istream& in, std::ostream& out);
    static void topPathsNoCycles(const map_type& networks, std::istream& in, std::ostream& out);
    static void copyConnections(const graph_type& src, graph_type& dest);
    static void merge(map_type& networks, std::istream& in);
    static void save(const map_type& networks, std::istream& in);
    static std::size_t countConnections(const graph_type::GraphMap& map);
    static void saveGraph(const graph_type::GraphMap& map, std::ofstream& fout);
    static void printWay(const graph_type::Way& way, std::ostream& out);
    static void printWays(const std::vector< graph_type::Way >& ways, std::ostream& out);
  };

  std::istream& operator>>(std::istream& in, Graph< std::string >& graph);
  void skipGraph(std::istream& in);
  void readConnection(std::istream& in, Graph< std::string >& graph);
}
#endif
