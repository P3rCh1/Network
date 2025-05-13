#ifndef HASH_MAP_H
#define HASH_MAP_H
#include <cassert>
#include <iterator>
#include <stdexcept>
#include "fwd_list.h"
#include "unique_ptr.h"

namespace ohantsev
{
  template< class Key, class Value,
    class Hash = std::hash< Key >,
    class KeyEqual = std::equal_to< Key > >
  class HashMap
  {
  public:
    using key_type = Key;
    using mapped_type = Value;
    using value_type = std::pair< const key_type, Value >;
    using size_type = std::size_t;
    using this_t = HashMap;

    template< bool IsConst >
    class HashMapIterator;

    using iterator = HashMapIterator< false >;
    using const_iterator = HashMapIterator< true >;

    static constexpr double MAX_LOAD_FACTOR{ 0.7 };
    static constexpr double EXPANSION_COEFFICIENT{ 2.0 };

    explicit HashMap(size_type = 10);
    ~HashMap();
    HashMap(const this_t& rhs);
    this_t& operator=(const this_t& rhs);
    HashMap(this_t&& rhs) noexcept;
    this_t& operator=(this_t&& rhs) noexcept;
    size_type size() const noexcept;
    bool empty() const noexcept;
    double loadFactor() const noexcept;
    void rehash();
    void reserve(std::size_t capacity);
    template< class Pair >
    std::pair< iterator, bool > insert(Pair&& pair);
    template< class K, class V >
    std::pair< iterator, bool > emplace(K&& key, V&& value);
    bool erase(const Key& key);
    bool erase(const iterator& iter);
    bool erase(const const_iterator& iter);
    void clear() noexcept;
    const mapped_type& operator[](const Key& key) const;
    const mapped_type& at(const Key& key) const;
    mapped_type& operator[](const Key& key);
    mapped_type& at(const Key& key);
    iterator find(const Key& key);
    const_iterator find(const Key& key) const;
    const_iterator cbegin() const noexcept;
    const_iterator cend() const noexcept;
    iterator begin() noexcept;
    iterator end() noexcept;
    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

  private:
    using node_t = FwdListNode< value_type >;

    size_type size_{ 0 };
    size_type bucketCount_{ 0 };
    UniquePtr< node_t >* map_{ nullptr };

    void swap(this_t& rhs) noexcept;
    size_type hash(const Key& key) const;
    size_type hash(const value_type& value) const;
    void resize(size_type newSize_);
    void removeContainer() noexcept;
  };

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  class HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator
  {
  public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = std::pair< const Key, Value >;
    using const_value_type = std::pair< const Key, const Value >;
    using difference_type = std::ptrdiff_t;
    using pointer = std::conditional_t< IsConst, const_value_type*, value_type* >;
    using reference = std::conditional_t< IsConst, const_value_type&, value_type& >;

    HashMapIterator() = delete;
    HashMapIterator(const HashMapIterator& rhs) = default;
    HashMapIterator& operator=(const HashMapIterator& rhs) = default;
    HashMapIterator(HashMapIterator&& rhs) = default;
    HashMapIterator& operator=(HashMapIterator&& rhs) = default;
    reference operator*() const;
    pointer operator->() const;
    HashMapIterator& operator++();
    HashMapIterator operator++(int);
    bool operator==(const HashMapIterator& rhs) const noexcept;
    bool operator!=(const HashMapIterator& rhs) const noexcept;

  private:
    friend class HashMap;

    using node_type = FwdListNode< value_type >;

    node_type* current_{ nullptr };
    size_type bucket_{ 0 };
    const HashMap* owner_{ nullptr };

    HashMapIterator(node_type* node, size_type bucket, const HashMap* owner) noexcept;
  };

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  auto HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator< IsConst >::operator*() const -> reference
  {
    assert(current_ != nullptr);
    return reinterpret_cast< reference >(current_->data_);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  auto HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator< IsConst >::operator->() const -> pointer
  {
    assert(current_ != nullptr);
    return reinterpret_cast< pointer >(&current_->data_);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  auto HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator< IsConst >::operator++() -> HashMapIterator&
  {
    if (!current_)
    {
      return *this;
    }
    if (current_->next_.get())
    {
      current_ = current_->next_.get();
      return *this;
    }
    current_ = nullptr;
    ++bucket_;
    while (bucket_ < owner_->bucketCount_)
    {
      if (owner_->map_[bucket_])
      {
        current_ = owner_->map_[bucket_].get();
        break;
      }
      ++bucket_;
    }
    return *this;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  auto HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator< IsConst >::operator++(int) -> HashMapIterator
  {
    HashMapIterator tmp = *this;
    ++(*this);
    return tmp;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  bool HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator< IsConst >::
  operator==(const HashMapIterator& rhs) const noexcept
  {
    assert(owner_ != nullptr);
    assert(rhs.owner_ != nullptr);
    return current_ == rhs.current_;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  bool HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator< IsConst >::
  operator!=(const HashMapIterator& rhs) const noexcept
  {
    return !(*this == rhs);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< bool IsConst >
  HashMap< Key, Value, Hash, KeyEqual >::HashMapIterator< IsConst >::
  HashMapIterator(node_type* node, size_type bucket, const HashMap* owner) noexcept:
    current_(node),
    bucket_(bucket),
    owner_(owner)
  {}

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::size() const noexcept -> size_type
  {
    return size_;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  bool HashMap< Key, Value, Hash, KeyEqual >::empty() const noexcept
  {
    return size_ == 0;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  HashMap< Key, Value, Hash, KeyEqual >::HashMap(const size_type capacity)
  {
    if (capacity == 0)
    {
      throw std::invalid_argument("Invalid capacity");
    }
    const size_type tmp = static_cast< size_type >(capacity / MAX_LOAD_FACTOR) + 1;
    map_ = new UniquePtr< node_t >[tmp]();
    bucketCount_ = tmp;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  void HashMap< Key, Value, Hash, KeyEqual >::clear() noexcept
  {
    for (size_type i = 0; i < bucketCount_; i++)
    {
      map_[i].reset();
    }
    size_ = 0;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  void HashMap< Key, Value, Hash, KeyEqual >::removeContainer() noexcept
  {
    delete[] map_;
    map_ = nullptr;
    bucketCount_ = 0;
    size_ = 0;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  HashMap< Key, Value, Hash, KeyEqual >::~HashMap()
  {
    delete[] map_;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  HashMap< Key, Value, Hash, KeyEqual >::HashMap(this_t&& rhs) noexcept
  {
    swap(rhs);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::operator=(this_t&& rhs) noexcept -> this_t&
  {
    if (this != &rhs)
    {
      removeContainer();
      swap(rhs);
    }
    return (*this);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  void HashMap< Key, Value, Hash, KeyEqual >::resize(size_type newSize_)
  {
    this_t tmp(newSize_);
    for (auto& pair: *this)
    {
      auto bucket = tmp.hash(pair.first);
      tmp.map_[bucket] = makeUnique< node_t >(std::move(pair), std::move(tmp.map_[bucket]));
      ++tmp.size_;
    }
    removeContainer();
    swap(tmp);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  HashMap< Key, Value, Hash, KeyEqual >::HashMap(const this_t& rhs)
  {
    static_assert(std::is_copy_constructible< Key >::value && std::is_copy_constructible< Value >::value);
    this_t tmp(rhs.size());
    for (const auto& pair: rhs)
    {
      auto bucket = tmp.hash(pair.first);
      tmp.map_[bucket] = makeUnique< node_t >(pair, std::move(tmp.map_[bucket]));
      ++tmp.size_;
    }
    removeContainer();
    swap(tmp);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  void HashMap< Key, Value, Hash, KeyEqual >::swap(this_t& rhs) noexcept
  {
    std::swap(size_, rhs.size_);
    std::swap(bucketCount_, rhs.bucketCount_);
    std::swap(map_, rhs.map_);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::operator=(const this_t& rhs) -> this_t&
  {
    if (this != &rhs)
    {
      auto tmp(rhs);
      swap(tmp);
    }
    return *this;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  double HashMap< Key, Value, Hash, KeyEqual >::loadFactor() const noexcept
  {
    assert(bucketCount_ != 0);
    return static_cast< double >(size_) / bucketCount_;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  void HashMap< Key, Value, Hash, KeyEqual >::rehash()
  {
    resize(static_cast< size_type >(size_ * EXPANSION_COEFFICIENT));
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  void HashMap< Key, Value, Hash, KeyEqual >::reserve(const size_type capacity)
  {
    if (capacity > size_)
    {
      resize(capacity);
    }
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< class Pair >
  auto HashMap< Key, Value, Hash, KeyEqual >::
  insert(Pair&& pair) -> std::pair< iterator, bool >
  {
    auto iter = find(pair.first);
    if (iter != end())
    {
      return std::make_pair(iter, false);
    }
    if (loadFactor() >= MAX_LOAD_FACTOR)
    {
      rehash();
    }
    auto bucket = hash(pair.first);
    map_[bucket] = makeUnique< node_t >(std::forward< Pair >(pair), std::move(map_[bucket]));
    ++size_;
    return std::make_pair(iterator{ map_[bucket].get(), bucket, this }, true);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  template< class K, class V >
  auto HashMap< Key, Value, Hash, KeyEqual >::
  emplace(K&& key, V&& value) -> std::pair< iterator, bool >
  {
    return insert(value_type(std::forward< K >(key), std::forward< V >(value)));
  }


  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::hash(const Key& key) const -> size_type
  {
    assert(bucketCount_ != 0);
    return Hash{}(key) % bucketCount_;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::hash(const value_type& value) const -> size_type
  {
    return hash(value.first);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  bool HashMap< Key, Value, Hash, KeyEqual >::erase(const Key& key)
  {
    {
      size_type bucket = hash(key);
      if (!map_[bucket])
      {
        return false;
      }
      node_t* cur = map_[bucket].get();
      if (KeyEqual{}(cur->data_.first, key))
      {
        map_[bucket] = std::move(cur->next_);
        --size_;
        return true;
      }
      while (cur->next_)
      {
        if (KeyEqual{}(cur->next_->data_.first, key))
        {
          cur->next_ = std::move(cur->next_->next_);
          --size_;
          return true;
        }
        cur = cur->next_.get();
      }
      return false;
    }
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  bool HashMap< Key, Value, Hash, KeyEqual >::erase(const iterator& iter)
  {
    if (iter == end())
    {
      return false;
    }
    return erase(iter->first);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  bool HashMap< Key, Value, Hash, KeyEqual >::erase(const const_iterator& iter)
  {
    if (iter == cend())
    {
      return false;
    }
    return erase(iter->first);
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::cbegin() const noexcept -> const_iterator
  {
    for (size_type bucket = 0; bucket < bucketCount_; ++bucket)
    {
      if (map_[bucket])
      {
        return const_iterator{ map_[bucket].get(), bucket, this };
      }
    }
    return end();
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::cend() const noexcept -> const_iterator
  {
    return const_iterator{ nullptr, bucketCount_, this };
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::begin() noexcept -> iterator
  {
    for (size_type bucket = 0; bucket < bucketCount_; ++bucket)
    {
      if (map_[bucket])
      {
        return iterator{ map_[bucket].get(), bucket, this };
      }
    }
    return end();
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::end() noexcept -> iterator
  {
    return iterator{ nullptr, bucketCount_, this };
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::begin() const noexcept -> const_iterator
  {
    return cbegin();
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::end() const noexcept -> const_iterator
  {
    return cend();
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::find(const Key& key) -> iterator
  {
    auto bucket = hash(key);
    for (auto node = map_[bucket].get(); node != nullptr; node = node->next_.get())
    {
      if (KeyEqual{}(node->data_.first, key))
      {
        return iterator{ node, bucket, this };
      }
    }
    return end();
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::find(const Key& key) const -> const_iterator
  {
    auto bucket = hash(key);
    for (auto node = map_[bucket].get(); node != nullptr; node = node->next_.get())
    {
      if (KeyEqual{}(node->data_.first, key))
      {
        return const_iterator{ node, bucket, this };
      }
    }
    return end();
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::operator[](const Key& key) -> mapped_type&
  {
    return emplace(key, mapped_type{}).first->second;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::operator[](const Key& key) const -> const mapped_type&
  {
    auto iter = find(key);
    if (iter == end())
    {
      static mapped_type defaultVal{};
      return defaultVal;
    }
    return iter->second;
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::at(const Key& key) -> mapped_type&
  {
    auto iter = find(key);
    if (iter != end())
    {
      return iter->second;
    }
    throw std::out_of_range("Key not found");
  }

  template< class Key, class Value, class Hash, class KeyEqual >
  auto HashMap< Key, Value, Hash, KeyEqual >::at(const Key& key) const -> const mapped_type&
  {
    auto iter = find(key);
    if (iter != end())
    {
      return iter->second;
    }
    throw std::out_of_range("Key not found");
  }
}
#endif
