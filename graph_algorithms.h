#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <algorithm>
#include <stdexcept>
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
    using NodesOutVec = std::vector< ConstKeyRef >;
    using CntsOutVec = std::vector< std::pair< ConstKeyRef, std::size_t > >;
    using Way = std::vector< std::pair< ConstKeyRef, std::size_t > >;

    explicit Graph(std::size_t capacity = 20);
    ~Graph() noexcept = default;
    Graph(this_t&& rhs) noexcept = default;
    this_t& operator=(this_t&& rhs) noexcept = default;
    Graph(const this_t& rhs);
    this_t& operator=(const this_t& rhs);

    void clear() noexcept;
    std::size_t size() const noexcept;
    bool insert(const Key& key);
    bool link(const Key& first, const Key& second, std::size_t weight);
    NodesOutVec nodes() const;
    CntsOutVec connections(const Key& key) const;
    bool remove(const Key& key);
    bool removeForce(const Key& key);
    bool removeLink(const Key& first, const Key& second);
    void removeCycles();
    std::vector< Way > ways(std::size_t top) const;

  private:
    struct Connection;
    struct Node;

    struct RefKeyHash
    {
      std::size_t operator()(const ConstKeyRef& ref) const;
    };

    struct RefKeyEqual
    {
      bool operator()(const ConstKeyRef& a, const ConstKeyRef& b) const;
    };

    using NodeMap = HashMap< Key, UniquePtr< Node >, Hash, KeyEqual >;
    using ConnectionMap = HashMap< ConstKeyRef, Connection, RefKeyHash, RefKeyEqual >;

    NodeMap nodes_;

    struct DSU;

    using Edge = std::pair< std::size_t, std::pair< Node*, Node* > >;

    struct edgeLess
    {
      bool operator()(const Edge& lhs, const Edge& rhs) const;
    };

    std::vector< Edge > collectEdges() const;
    DSU makeUnrelatedDSU() const;
  };

  template< class Key, class Hash, class KeyEqual >
  struct Graph< Key, Hash, KeyEqual >::Connection
  {
    Node* target_;
    std::size_t weight_;

    Connection(Node* target, size_t weight) :
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
  std::size_t Graph< Key, Hash, KeyEqual >::RefKeyHash::operator()(const ConstKeyRef& ref) const
  {
    return Hash{}(ref.get());
  }

  template< class Key, class Hash, class KeyEqual >
  bool Graph< Key, Hash, KeyEqual >::RefKeyEqual::operator()(const ConstKeyRef& a, const ConstKeyRef& b) const
  {
    return KeyEqual{}(a.get(), b.get());
  }

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
        node->connections_.emplace(std::cref(target->data_), Connection{ target, weight });
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
    firstCnts.emplace(std::cref(secondIter->first), Connection{ secondNode, weight });
    secondCnts.emplace(std::cref(firstIter->first), Connection{ firstNode, weight });
    return true;
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::nodes() const -> NodesOutVec
  {
    NodesOutVec result;
    result.reserve(nodes_.size());
    for (const auto& pair: nodes_)
    {
      result.emplace_back(pair.first);
    }
    return result;
  }

  template< class Key, class Hash, class KeyEqual >
  auto Graph< Key, Hash, KeyEqual >::connections(const Key& key) const -> CntsOutVec
  {
    auto iter = nodes_.find(key);
    if (iter == nodes_.end())
    {
      throw std::invalid_argument("Node not found");
    }
    CntsOutVec result;
    result.reserve(iter->second->connections_.size());
    for (const auto& pair: iter->second->connections_)
    {
      result.emplace_back(pair.first, pair.second.weight_);
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
  bool Graph< Key, Hash, KeyEqual >::edgeLess::operator()(const Edge& lhs, const Edge& rhs) const
  {
    return lhs.first < rhs.first;
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
  auto Graph< Key, Hash, KeyEqual >::ways(std::size_t top) const -> std::vector< Way >
  {
    std::vector< Way > result;
    return result;
  }
}
#endif
