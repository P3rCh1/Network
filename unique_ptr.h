#ifndef UNIQUE_PTR_H
#define UNIQUE_PTR_H
#include <cassert>
#include <utility>

namespace ohantsev
{
  template< class T >
  class UniquePtr
  {
  public:
    UniquePtr() = default;
    ~UniquePtr() noexcept;
    explicit UniquePtr(T* pointer) noexcept;
    UniquePtr(const UniquePtr&) = delete;
    UniquePtr& operator=(const UniquePtr&) = delete;
    UniquePtr(UniquePtr&& rhs) noexcept;
    UniquePtr& operator=(UniquePtr&& rhs) noexcept;
    explicit UniquePtr(std::nullptr_t) noexcept;
    UniquePtr& operator=(std::nullptr_t) noexcept;
    T& operator*() const;
    T* operator->() const;
    T* get() const noexcept;
    T* release() noexcept;
    void reset(T* pointer = nullptr) noexcept;
    explicit operator bool() const noexcept;

  private:
    T* pointer_{ nullptr };
  };

  template< class T >
  UniquePtr<T>::~UniquePtr() noexcept
  {
    delete pointer_;
  }

  template< class T >
  UniquePtr<T>::UniquePtr(T* pointer) noexcept:
    pointer_(pointer)
  {}

  template< class T >
  UniquePtr<T>::UniquePtr(UniquePtr&& rhs) noexcept:
    pointer_(rhs.pointer_)
  {
    rhs.pointer_ = nullptr;
  }

  template< class T >
  UniquePtr<T>& UniquePtr<T>::operator=(UniquePtr&& rhs) noexcept
  {
    if (this != &rhs)
    {
      delete pointer_;
      pointer_ = rhs.pointer_;
      rhs.pointer_ = nullptr;
    }
    return *this;
  }

  template< class T >
  UniquePtr<T>::UniquePtr(std::nullptr_t) noexcept:
    pointer_(nullptr)
  {}

  template< class T >
  UniquePtr<T>& UniquePtr<T>::operator=(std::nullptr_t) noexcept
  {
    reset();
    return *this;
  }

  template< class T >
  T& UniquePtr<T>::operator*() const
  {
    assert(pointer_ != nullptr);
    return *pointer_;
  }

  template< class T >
  T* UniquePtr<T>::operator->() const
  {
    assert(pointer_ != nullptr);
    return pointer_;
  }

  template< class T >
  T* UniquePtr<T>::get() const noexcept
  {
    return pointer_;
  }

  template< class T >
  T* UniquePtr<T>::release() noexcept
  {
    T* pointer = pointer_;
    pointer_ = nullptr;
    return pointer;
  }

  template< class T >
  void UniquePtr<T>::reset(T* pointer) noexcept
  {
    if (pointer != pointer_)
    {
      delete pointer_;
      pointer_ = pointer;
    }
  }

  template< class T >
  UniquePtr<T>::operator bool() const noexcept
  {
    return (pointer_ != nullptr);
  }

  template< class T, class... Args >
  UniquePtr< T > makeUnique(Args&&... args)
  {
    return UniquePtr< T >(new T(std::forward< Args >(args)...));
  }
}
#endif
