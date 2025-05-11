#ifndef FWD_LIST_H
#define FWD_LIST_H
#include "unique_ptr.h"

namespace ohantsev
{
  template< class T >
  struct FwdListNode
  {
    T data_;
    UniquePtr< FwdListNode > next_;
    FwdListNode();
    explicit FwdListNode(T data, UniquePtr< FwdListNode >&& next = nullptr);
    FwdListNode(const FwdListNode&) = delete;
    FwdListNode& operator=(const FwdListNode&) = delete;
    ~FwdListNode() = default;
    FwdListNode(FwdListNode&&) = default;
    FwdListNode& operator=(FwdListNode&&) = default;
  };

  template< class T >
  FwdListNode<T>::FwdListNode():
    data_(),
    next_(nullptr)
  {}

  template< class T >
  FwdListNode<T>::FwdListNode(T data, UniquePtr<FwdListNode>&& next):
    data_(std::move(data)),
    next_(std::move(next))
  {}
}
#endif
