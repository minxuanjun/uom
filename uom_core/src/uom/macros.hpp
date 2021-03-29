#pragma once

#include <memory>

#define POINTER_TYPEDEFS(TypeName)                 \
  typedef std::shared_ptr<TypeName> Ptr;                  \
  typedef std::shared_ptr<const TypeName> ConstPtr;       \
  typedef std::unique_ptr<TypeName> UniquePtr;            \
  typedef std::unique_ptr<const TypeName> ConstUniquePtr; \
  typedef std::weak_ptr<TypeName> WeakPtr;                \
  typedef std::weak_ptr<const TypeName> WeakConstPtr;     \


#define DELETE_COPY_CONSTRUCTORS(TypeName)                \
  TypeName(const TypeName&) = delete;                     \
  void operator=(const TypeName&) = delete;
