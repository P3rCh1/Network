#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <memory>
#include <functional>
#include <unordered_map>

template<
  class Key,
  class Hash = std::hash< Key >,
  class KeyEqual = std::equal_to< Key > >
class Graph
{
public:
  using this_t = Graph< Key, Hash, KeyEqual >;
  using ConstKeyRef = std::reference_wrapper< const Key >;
  using CntOutVec = std::vector< std::pair< Key, std::size_t > >;

  explicit Graph(std::size_t capacity = 100);
  ~Graph() noexcept = default;
  Graph(this_t&& rhs) noexcept = default;
  this_t& operator=(this_t&& rhs) noexcept = default;
  Graph(const this_t& rhs);
  this_t& operator=(const this_t& rhs);
  void clear() noexcept;
  std::size_t size() const noexcept;

  bool insert(const Key& key);
  bool link(const Key& first, const Key& second, std::size_t weight);
  CntOutVec connections(const Key& key) const;
  bool remove(const Key& key);
  bool removeForce(const Key& key);
  bool removeLink(const Key& first, const Key& second);

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

  using NodeMap = std::unordered_map< Key, std::unique_ptr< Node >, Hash, KeyEqual >;
  using ConnectionMap = std::unordered_map< ConstKeyRef, Connection, RefKeyHash, RefKeyEqual >;

  NodeMap nodes_;
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
    result.first->second = std::make_unique< Node >(result.first->first);
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
  auto& firstСnts = firstNode->connections_;
  auto& secondCnts = secondNode->connections_;
  if (firstСnts.find(std::cref(secondIter->first)) != firstСnts.end())
  {
    return false;
  }
  firstСnts.emplace(std::cref(secondIter->first), Connection{ secondNode, weight });
  secondCnts.emplace(std::cref(firstIter->first), Connection{ firstNode, weight });
  return true;
}

template< class Key, class Hash, class KeyEqual >
auto Graph< Key, Hash, KeyEqual >::connections(const Key& key) const -> CntOutVec
{
  auto it = nodes_.find(key);
  if (it == nodes_.end())
  {
    throw std::invalid_argument("Node not found");
  }
  CntOutVec result;
  result.reserve(it->second->connections_.size());
  for (const auto& pair: it->second->connections_)
  {
    result.emplace_back(pair.first.get(), pair.second.weight_);
  }
  return result;
}

template< class Key, class Hash, class KeyEqual >
bool Graph< Key, Hash, KeyEqual >::remove(const Key& key)
{
  auto it = nodes_.find(key);
  if (it == nodes_.end() || !it->second->connections_.empty())
  {
    return false;
  }
  nodes_.erase(it);
  return true;
}

template< class Key, class Hash, class KeyEqual >
bool Graph< Key, Hash, KeyEqual >::removeForce(const Key& key)
{
  auto it = nodes_.find(key);
  if (it == nodes_.end())
  {
    return false;
  }
  for (auto& pair: it->second->connections_)
  {
    nodes_[pair.first.get()]->connections_.erase(std::cref(it->first));
  }
  nodes_.erase(it);
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
Graph< Key, Hash, KeyEqual >::Graph(const this_t& rhs)
{
  nodes_.reserve(rhs.nodes_.size());
  for (const auto& rhsNodePair: rhs.nodes_)
  {
    nodes_.emplace(rhsNodePair.first, std::make_unique< Node >(rhsNodePair.first));
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
  if (this != & rhs)
  {
    Graph temp(rhs);
    std::swap(nodes_, temp.nodes_);
  }
  return * this;
}
#endif
