#ifndef GRAPH_H
#define GRAPH_H
#include <limits>
#include <vector>
#include <algorithm>
#include <queue>
#include <stdexcept>
#include <unordered_set>

#include "hash_map.h"
#include "unique_ptr.h"

namespace ohantsev
{
  template< class Key,
    class Hash = std::hash< Key >,
    class KeyEqual = std::equal_to< Key > >
  class Graph
  {
  public:
    using this_t = Graph;
    using ConstKeyRef = std::reference_wrapper< const Key >;
    using ItemVec = std::vector< ConstKeyRef >;
    struct Connection;
    using ConnectionVec = std::vector< Connection >;
    struct Way;
    using WayVec = std::vector< Way >;

    explicit Graph(std::size_t capacity = 1);
    ~Graph() noexcept = default;
    Graph(this_t&& rhs) noexcept = default;
    this_t& operator=(this_t&& rhs) noexcept = default;
    Graph(const this_t& rhs);
    this_t& operator=(const this_t& rhs);

    void clear() noexcept;
    std::size_t size() const noexcept;
    bool insert(const Key& key);
    bool link(const Key& first, const Key& second, std::size_t weight);
    ItemVec nodes() const;
    bool contains(const Key& key) const;
    bool contains(const ConstKeyRef& key) const;
    ConnectionVec connections(const Key& key) const;
    bool remove(const Key& key);
    bool removeForce(const Key& key);
    bool removeLink(const Key& first, const Key& second);
    void removeCycles();
    auto path(const Key& start, const Key& end) const -> Way;
    WayVec path(const Key& start, const Key& end, std::size_t count) const;

  private:
    struct ConnectionPrivate;
    struct Node;
    struct RefKeyHash;
    struct RefKeyEqual;
    struct DSU;
    struct edgeLess;
    struct stepGreater;
    struct DijkstraContainers;
    struct WayGreater;
    struct GraphModifierRAII;
    struct WayRefHash;
    struct WayRefEqual;
    struct YensContainers;

    using Edge = std::pair< std::size_t, std::pair< Node*, Node* > >;
    using NodeMap = HashMap< Key, UniquePtr< Node >, Hash, KeyEqual >;
    using ConnectionMap = HashMap< ConstKeyRef, ConnectionPrivate, RefKeyHash, RefKeyEqual >;
    using Step = std::pair< std::size_t, Node* >;
    using StepQueue = std::priority_queue< Step, std::vector< Step >, stepGreater >;
    using WaysQueue = std::priority_queue< Way, WayVec, WayGreater >;
    using ConstWayRef = std::reference_wrapper< const Way >;
    using WaySet = std::unordered_set< std::reference_wrapper< const Way >, WayRefHash, WayRefEqual >;

    NodeMap nodes_;

    std::vector< Edge > collectEdges() const;
    DSU makeUnrelatedDSU() const;
    Way constructPath(const Key& start, const Key& end) const;
    void pushNeighbors(DijkstraContainers& cont, Node* node) const;
    Way releasePath(const DijkstraContainers& cont, Node* start, Node* end) const;
    void addAnotherPaths(YensContainers& cont, std::size_t count) const;
    void pushCandidates(YensContainers& cont) const;
    auto extractRootPath(const Way& path, std::size_t spurIndex) const -> Way;
    auto combinePaths(const Way& rootPath, const Way& spurPath) const -> Way;
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::ConnectionPrivate
  {
    Node* target_;
    std::size_t weight_;

    ConnectionPrivate(Node* target, size_t weight) :
      target_(target),
      weight_(weight)
    {}
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::Connection
  {
    const Key& target_;
    std::size_t weight_;

    Connection(const Key& target, std::size_t weight) :
      target_(target),
      weight_(weight)
    {}
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::Node
  {
    const Key& data_;
    ConnectionMap connections_;

    explicit Node(const Key& key):
      data_(key)
    {}
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::RefKeyHash
  {
    std::size_t operator()(const ConstKeyRef& ref) const
    {
      return Hash{}(ref.get());
    }
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::RefKeyEqual
  {
    bool operator()(const ConstKeyRef& lhs, const ConstKeyRef& rhs) const
    {
      return KeyEqual{}(lhs.get(), rhs.get());
    }
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::edgeLess
  {
    bool operator()(const Edge& lhs, const Edge& rhs) const
    {
      return lhs.first < rhs.first;
    }
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::stepGreater
  {
    bool operator()(const Step& lhs, const Step& rhs) const
    {
      return lhs.first > rhs.first;
    }
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::WayGreater
  {
    bool operator()(const Way& lhs, const Way& rhs) const
    {
      return lhs.length_ > rhs.length_;
    }
  };

  template< class Key, class Hash, class KeyEqual >
  Graph< Key, Hash, KeyEqual >::Graph(size_t capacity)
  {
    nodes_.reserve(capacity);
  }

  template< class Key, class Hash, class KeyEqual >
  Graph< Key, Hash, KeyEqual >::Graph(const this_t& rhs)
  {
    nodes_.reserve(rhs.nodes_.size());
    for (const auto& rhsNodePair: rhs.nodes_)
    {
      nodes_.emplace(rhsNodePair.first, makeUnique< Node >(rhsNodePair.first));
    }
    for (auto& newNodePair: nodes_)
    {
      const Key& key = newNodePair.first;
      Node* node = newNodePair.second.get();
      for (const auto& rhsCntPair: rhs.nodes_.at(key)->connections_)
      {
        const Key& targetKey = rhsCntPair.first.get();
        const std::size_t weight = rhsCntPair.second.weight_;
        Node* target = nodes_[targetKey].get();
        node->connections_.emplace(std::cref(target->data_), ConnectionPrivate{ target, weight });
      }
    }
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::operator=(const this_t& rhs) -> this_t&
  {
    if (this != &rhs)
    {
      Graph temp(rhs);
      std::swap(nodes_, temp.nodes_);
    }
    return *this;
  }

  template< class Key, class Hash, class KeyEqual >
  void Graph< Key, Hash, KeyEqual >::clear() noexcept
  {
    nodes_.clear();
  }

  template< class Key, class Hash, class KeyEqual >
  std::size_t Graph< Key, Hash, KeyEqual >::size() const noexcept
  {
    return nodes_.size();
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::insert(const Key& key)
  {
    auto result = nodes_.emplace(key, nullptr);
    if (result.second)
    {
      result.first->second = makeUnique< Node >(result.first->first);
    }
    return result.second;
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::link(const Key& first, const Key& second, std::size_t weight)
  {
    if (KeyEqual{}(first, second))
    {
      return false;
    }
    auto firstIter = nodes_.find(first);
    auto secondIter = nodes_.find(second);
    if (firstIter == nodes_.end() || secondIter == nodes_.end())
    {
      return false;
    }
    Node* firstNode = firstIter->second.get();
    Node* secondNode = secondIter->second.get();
    auto& firstCnts = firstNode->connections_;
    auto& secondCnts = secondNode->connections_;
    if (firstCnts.find(std::cref(secondIter->first)) != firstCnts.end())
    {
      return false;
    }
    firstCnts.emplace(std::cref(secondIter->first), ConnectionPrivate{ secondNode, weight });
    secondCnts.emplace(std::cref(firstIter->first), ConnectionPrivate{ firstNode, weight });
    return true;
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::nodes() const -> ItemVec
  {
    ItemVec result;
    result.reserve(nodes_.size());
    for (const auto& pair: nodes_)
    {
      result.emplace_back(pair.first);
    }
    return result;
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::contains(const Key& key) const
  {
    return nodes_.find(key) != nodes_.end();
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::contains(const ConstKeyRef& key) const
  {
    return nodes_.find(key.get()) != nodes_.end();
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::connections(const Key& key) const -> ConnectionVec
  {
    auto iter = nodes_.find(key);
    if (iter == nodes_.end())
    {
      throw std::invalid_argument("Node not found");
    }
    ConnectionVec result;
    result.reserve(iter->second->connections_.size());
    for (const auto& pair: iter->second->connections_)
    {
      result.emplace_back(pair.first.get(), pair.second.weight_);
    }
    return result;
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::remove(const Key& key)
  {
    auto iter = nodes_.find(key);
    if (iter == nodes_.end() || !iter->second->connections_.empty())
    {
      return false;
    }
    nodes_.erase(iter);
    return true;
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::removeForce(const Key& key)
  {
    auto iter = nodes_.find(key);
    if (iter == nodes_.end())
    {
      return false;
    }
    for (auto& pair: iter->second->connections_)
    {
      nodes_[pair.first.get()]->connections_.erase(std::cref(iter->first));
    }
    nodes_.erase(iter);
    return true;
  }


  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::removeLink(const Key& first, const Key& second)
  {
    if (KeyEqual{}(first, second))
    {
      return false;
    }
    auto firstIter = nodes_.find(first);
    auto secondIter = nodes_.find(second);
    if (firstIter == nodes_.end() || secondIter == nodes_.end())
    {
      return false;
    }
    return firstIter->second->connections_.erase(std::cref(secondIter->first)) &&
           secondIter->second->connections_.erase(std::cref(firstIter->first));
  }

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::DSU
  {
    HashMap< Node*, Node* > parent;
    HashMap< Node*, std::size_t > rank;

    Node* findRoot(Node* node);
    void unionSets(Node* first, Node* second);
  };

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::DSU::findRoot(Node* node) -> Node*
  {
    if (parent[node] != node)
    {
      parent[node] = findRoot(parent[node]);
    }
    return parent[node];
  }

  template< class Key, class Hash, class KeyEqual >
  void Graph< Key, Hash, KeyEqual >::DSU::unionSets(Node* first, Node* second)
  {
    first = findRoot(first);
    second = findRoot(second);
    if (first == second)
    {
      return;
    }
    if (rank[first] < rank[second])
    {
      parent[first] = second;
    }
    else
    {
      parent[second] = first;
      if (rank[first] == rank[second])
      {
        ++rank[first];
      }
    }
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::collectEdges() const -> std::vector< Edge >
  {
    std::vector< Edge > edges;
    edges.reserve(nodes_.size() * 2);
    for (const auto& pair: nodes_)
    {
      Node* from = pair.second.get();
      for (const auto& connection: from->connections_)
      {
        Node* to = connection.second.target_;
        if (from < to)
        {
          edges.emplace_back(connection.second.weight_, std::make_pair(from, to));
        }
      }
    }
    std::sort(edges.begin(), edges.end(), edgeLess{});
    return edges;
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::makeUnrelatedDSU() const -> DSU
  {
    DSU dsu;
    for (const auto& pair: nodes_)
    {
      Node* node = pair.second.get();
      dsu.parent.emplace(node, node);
      dsu.rank.emplace(node, 0);
    }
    return dsu;
  }

  template< class Key, class Hash, class KeyEqual >
  void Graph< Key, Hash, KeyEqual >::removeCycles()
  {
    if (nodes_.empty())
    {
      return;
    }
    auto edges = collectEdges();
    auto dsu = makeUnrelatedDSU();
    for (const auto& edge: edges)
    {
      Node* from = edge.second.first;
      Node* to = edge.second.second;
      if (dsu.findRoot(from) != dsu.findRoot(to))
      {
        dsu.unionSets(from, to);
      }
      else
      {
        removeLink(from->data_, to->data_);
      }
    }
  }

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::Way
  {
    std::vector< ConstKeyRef > steps_;
    std::size_t length_{ 0 };
    bool operator==(const Way& rhs) const;
    bool operator!=(const Way& rhs) const;
    bool hasRootPath(const Way& root) const;
  };

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::Way::operator==(const Way& rhs) const
  {
    return steps_.size() == steps_.size() &&
           std::equal(rhs.steps_.begin(), rhs.steps_.end(), steps_.begin(), RefKeyEqual{});
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::Way::operator!=(const Way& rhs) const
  {
    return !(*this == rhs);;
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::Way::hasRootPath(const Way& root) const
  {
    return steps_.size() > steps_.size() &&
           std::equal(root.steps_.begin(), root.steps_.end(), steps_.begin(), RefKeyEqual{});
  }

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::DijkstraContainers
  {
    using DistanceMap = HashMap< Node*, std::size_t >;
    using PreviousMap = HashMap< Node*, std::pair< Node*, std::size_t > >;

    DistanceMap distances;
    PreviousMap previous;
    StepQueue queue;

    DijkstraContainers(const Graph& graph, const Key& start)
    {
      for (const auto& pair: graph.nodes_)
      {
        distances.emplace(pair.second.get(), std::numeric_limits< std::size_t >::max());
      }
      Node* startNode = graph.nodes_[start].get();
      distances[startNode] = 0;
      queue.emplace(0, startNode);
    }
  };

  template< class Key, class Hash, class KeyEqual >
  void Graph< Key, Hash, KeyEqual >::pushNeighbors(DijkstraContainers& cont, Node* node) const
  {
    for (const auto& cnt: node->connections_)
    {
      auto neighbor = cnt.second.target_;
      auto newDistance = cont.distances[node] + cnt.second.weight_;
      if (newDistance < cont.distances[neighbor])
      {
        cont.distances[neighbor] = newDistance;
        cont.previous[neighbor] = { node, cnt.second.weight_ };
        cont.queue.emplace(newDistance, neighbor);
      }
    }
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::constructPath(const Key& start, const Key& end) const -> Way
  {
    DijkstraContainers cont(*this, start);
    Node* startNode = nodes_[start].get();
    Node* endNode = nodes_[end].get();
    while (!cont.queue.empty())
    {
      auto current = cont.queue.top();
      cont.queue.pop();
      auto currentDistance = current.first;
      auto currentNode = current.second;
      if (currentDistance > cont.distances[currentNode])
      {
        continue;
      }
      if (currentNode == endNode)
      {
        break;
      }
      pushNeighbors(cont, currentNode);
    }
    return releasePath(cont, startNode, endNode);
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::releasePath(const DijkstraContainers& cont, Node* start, Node* end) const -> Way
  {
    if (cont.distances[end] == std::numeric_limits< std::size_t >::max())
    {
      throw std::runtime_error("Nodes don't connected");
    }
    Way path;
    Node* current = end;
    while (current != start)
    {
      auto prevStep = cont.previous[current];
      auto prev = prevStep.first;
      auto weight = prevStep.second;
      path.steps_.emplace_back(std::cref(current->data_));
      path.length_ += weight;
      current = prev;
    }
    path.steps_.emplace_back(std::cref(current->data_));
    std::reverse(path.steps_.begin(), path.steps_.end());
    return path;
  }


  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::path(const Key& start, const Key& end) const -> Way
  {
    if (!contains(start) || !contains(end))
    {
      throw std::invalid_argument("Key not found");
    }
    return constructPath(start, end);
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::path(const Key& start, const Key& end,
                                          std::size_t count) const -> std::vector< Way >
  {
    try
    {
      YensContainers cont;
      if (count > 0)
      {
        cont.result.reserve(count);
        cont.result.push_back(path(start, end));
        cont.set.insert(std::cref(cont.result[0]));
      }
      return cont.result;
    }
    catch (std::runtime_error& e)
    {
      return {};
    }
  }

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::YensContainers
  {
    WayVec result;
    WaysQueue candidates;
    WaySet set;
    YensContainers() = default;
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::WayRefHash
  {
    size_t operator()(const ConstWayRef& wayRef) const
    {
      const Way& way = wayRef.get();
      size_t seed = way.length_;
      for (const auto& step: way.steps_)
      {
        Hash hasher;
        seed ^= hasher(step.get()) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      }
      return seed;
    }
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::WayRefEqual
  {
    bool operator()(const ConstWayRef& lhsRef, const ConstWayRef& rhsRef) const
    {
      const Way& lhs = lhsRef.get();
      const Way& rhs = rhsRef.get();
      if (lhs.length_ != rhs.length_ || lhs.steps_.size() != rhs.steps_.size())
      {
        return false;
      }
      for (std::size_t i = 0; i < lhs.steps_.size(); ++i)
      {
        if (lhs.steps_[i].get() != rhs.steps_[i].get())
        {
          return false;
        }
      }
      return true;
    }
  };

  template< class Key, class Hash, class KeyEqual >
  void Graph< Key, Hash, KeyEqual >::addAnotherPaths(YensContainers& cont, std::size_t count) const
  {
    for (std::size_t i = 1; i < count; ++i)
    {
      pushCandidates(cont);
      if (cont.candidates.empty())
      {
        break;
      }
      cont.result.push_back(cont.candidates.top());
      cont.set.erase(std::cref(cont.result.back()));
      cont.set.insert(std::cref(cont.result.back()));
      cont.candidates.pop();
    }
  }

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::GraphModifierRAII
  {
  public:
    GraphModifierRAII(Graph& graph, const Way& rootPath, const WayVec& result);
    ~GraphModifierRAII();
    GraphModifierRAII(const GraphModifierRAII&) = delete;
    GraphModifierRAII& operator=(const GraphModifierRAII&) = delete;
    GraphModifierRAII(GraphModifierRAII&&) = default;
    GraphModifierRAII& operator=(GraphModifierRAII&&) = default;

  private:
    Graph& graph_;
    const Key& spurKey_;
    Node* spurNode_;
    HashMap< Key, ConnectionPrivate > removedEdges_;

    void removeSpurCnt(const Way& rootPath, const Way& way);
  };

  template <class Key, class Hash, class KeyEqual >
  void Graph<Key, Hash, KeyEqual>::GraphModifierRAII::removeSpurCnt(const Way& rootPath, const Way& way)
  {
    if (way.hasRootPath(rootPath))
    {
      const Key& nextInPath = way.steps_[rootPath.steps_.size()].get();
      auto it = spurNode_->connections_.find(std::cref(nextInPath));
      if (it != spurNode_->connections_.end())
      {
        removedEdges_.emplace(nextInPath, it->second);
        spurNode_->connections_.erase(it);
      }
    }
  }

  template< class Key, class Hash, class KeyEqual >
  Graph< Key, Hash, KeyEqual >::GraphModifierRAII::GraphModifierRAII(Graph& graph, const Way& rootPath,
                                                                     const WayVec& result):
    graph_(graph),
    spurKey_(rootPath.steps_.back().get())
  {
    spurNode_ = graph.nodes_[spurKey_];
    removeCnts(rootPath, result);
    removeNodes(rootPath);
    for (const auto& way: result)
    {
      removeSpurCnt(rootPath, way);
    }
  }

  template< class Key, class Hash, class KeyEqual >
  Graph< Key, Hash, KeyEqual >::GraphModifierRAII::~GraphModifierRAII()
  {
    for (auto& pair: removedEdges_)
    {
      const Key& target = pair.first;
      Node* targetNode = graph_.nodes_[target].get();
      spurNode_->connections_.emplace(std::cref(target), pair.second);
      targetNode->connections_.emplace(std::cref(spurKey_), ConnectionPrivate{ spurNode_, pair.second.weight_ });
    }
  }

  template< class Key, class Hash, class KeyEqual >
  void Graph< Key, Hash, KeyEqual >::pushCandidates(YensContainers& cont) const
  {
    const Way& previousPath = cont.result.back();
    for (std::size_t i = 0; i < previousPath.steps_.size() - 1; ++i)
    {
      const Key& spurKey = previousPath.steps_[i].get();
      Way rootPath = extractRootPath(previousPath, i);
      GraphModifierRAII mod(*this, rootPath, cont.result);
      Way spurPath;
      try
      {
        spurPath = constructPath(spurKey, cont.result[0].steps_.back().get());
      }
      catch (const std::runtime_error&)
      {
        continue;
      }
      Way totalPath = combinePaths(rootPath, spurPath);
      if (!cont.set.contains(std::cref(totalPath)))
      {
        cont.candidates.push(totalPath);
        cont.set.insert(cont.candidates.top());
      }
    }
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::extractRootPath(const Way& path, std::size_t spurIndex) const -> Way
  {
    Way rootPath;
    rootPath.steps_.assign(path.steps_.begin(), path.steps_.begin() + spurIndex + 1);
    rootPath.length_ = 0;
    for (std::size_t i = 0; i < spurIndex; ++i)
    {
      const Key& from = path.steps_[i].get();
      const Key& to = path.steps_[i + 1].get();
      rootPath.length_ += nodes_[from]->connections_[std::cref(to)].weight_;
    }
    return rootPath;
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::combinePaths(const Way& rootPath, const Way& spurPath) const -> Way
  {
    Way totalPath = rootPath;
    totalPath.steps_.insert(totalPath.steps_.end(), spurPath.steps_.begin() + 1, spurPath.steps_.end());
    totalPath.length_ += spurPath.length_;
    return totalPath;
  }
}
#endif
