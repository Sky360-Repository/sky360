// Copyright © 2021 Dominic van Berkel <dominic@baudvine.net>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file ringbuf.h
 * @author Dominic van Berkel
 * @copyright MIT License
 *
 * A ring buffer for C++11, with an STL-like interface.
 *
 * The comments frequently refer to "physical" and "logical" indices. This is
 * meant to make explicit the distinction between:
 *
 * - The backing array of RingBuf, which is always of size Capacity + 1 and is
 *   allocated once during RingBuf construction. Physical indices are relative
 *   to the start of this array.
 * - The conceptual ring buffer, which moves around in the backing array and has
 *   a variable length. Logical indices are relative to its start
 *   ("ring_offset"), and ring_offset + index may exceed Capacity (before
 *   wrapping).
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <type_traits>

/** The baudvine "project". */
namespace baudvine {
namespace detail {
namespace ringbuf {

/** @private */
template <typename Allocator>
void MoveAllocator(Allocator& lhs,
                   Allocator& rhs,
                   std::true_type /*propagate*/) {
  // Swap instead of move-assign because data_ & co are also swapped, and the
  // moved-from ringbuf will need to be able to clean that up.
  std::swap(lhs, rhs);
}

/** @private */
template <typename Allocator>
void MoveAllocator(Allocator& /*lhs*/,
                   Allocator& /*rhs*/,
                   std::false_type /*propagate*/) noexcept {}

/** @private */
template <typename Allocator>
void MoveAllocator(Allocator& lhs, Allocator& rhs) {
  using AllocTraits = std::allocator_traits<Allocator>;
  using Propagate =
      typename AllocTraits::propagate_on_container_move_assignment;
  MoveAllocator(lhs, rhs, Propagate{});
}

/** @private */
template <typename Allocator>
void SwapAllocator(Allocator& lhs,
                   Allocator& rhs,
                   std::true_type /*propagate*/) {
  std::swap(lhs, rhs);
}

/** @private */
template <typename Allocator>
void SwapAllocator(Allocator& /*lhs*/,
                   Allocator& /*rhs*/,
                   std::false_type /*propagate*/) {}

/** @private */
template <typename Allocator>
void SwapAllocator(Allocator& lhs, Allocator& rhs) {
  using AllocTraits = std::allocator_traits<Allocator>;
  using Propagate = typename AllocTraits::propagate_on_container_swap;
  SwapAllocator(lhs, rhs, Propagate{});
}

/** @private */
template <typename Allocator>
void CopyAllocator(Allocator& lhs,
                   const Allocator& rhs,
                   std::true_type /*propagate*/) {
  lhs = rhs;
}

/** @private */
template <typename Allocator>
void CopyAllocator(Allocator& /*lhs*/,
                   const Allocator& /*rhs*/,
                   std::false_type /*propagate*/) {}

/** @private */
template <typename Allocator>
void CopyAllocator(Allocator& lhs, const Allocator& rhs) {
  using AllocTraits = std::allocator_traits<Allocator>;
  using Propagate =
      typename AllocTraits::propagate_on_container_copy_assignment;
  CopyAllocator(lhs, rhs, Propagate{});
}

/**
 * Wrap a physical position into an array of size Capacity.
 *
 * Precondition: ring_index < 2 * Capacity + 1.
 *
 * @tparam Capacity The backing array size.
 * @param ring_index The physical index into the backing array.
 * @returns The ring_index wrapped to [0..Capacity].
 * @private
 */
template <std::size_t Capacity>
constexpr std::size_t RingWrap(const std::size_t ring_index) {
  // This is a bit faster than `return ring_index % Capacity` (~40% reduction in
  // Speed.PushBackOverFull test)
  return (ring_index <= Capacity) ? ring_index : ring_index - Capacity - 1;
}

/**
 * An iterator into RingBuf.
 *
 * @tparam Ptr The pointer type, which determines constness.
 * @tparam AllocTraits The allocator traits for the container, used for
 *                     size/difference_type and const_pointer (for auto
 *                     conversion to const iterator).
 * @tparam Capacity The size of the backing array, and maximum size of the ring
 *                  buffer.
 */
template <typename Ptr, typename AllocTraits, std::size_t Capacity>
class Iterator {
 public:
  using difference_type = typename AllocTraits::difference_type;
  using size_type = typename AllocTraits::difference_type;
  using value_type = typename AllocTraits::value_type;
  using pointer = Ptr;
  using reference = decltype(*pointer{});
  using iterator_category = std::random_access_iterator_tag;

  constexpr Iterator() noexcept = default;
  /**
   * Construct a new iterator object.
   *
   * @param data Pointer to the start of the RingBuf's backing array.
   * @param ring_offset Physical index of the start of the ring buffer.
   * @param ring_index Logical index in the ring buffer: when the iterator is at
   *                   ring_offset, ring_index is 0.
   */
  Iterator(pointer data,
           const size_type ring_offset,
           const size_type ring_index)
      : data_(data), ring_offset_(ring_offset), ring_index_(ring_index) {}

  /**
   * Convert an iterator into a const iterator.
   *
   * @returns A const iterator pointing to the same place.
   */
  operator Iterator<typename AllocTraits::const_pointer,
                    AllocTraits,
                    Capacity>() const {
    return Iterator<typename AllocTraits::const_pointer, AllocTraits, Capacity>(
        data_, ring_offset_, ring_index_);
  }

  reference operator*() const {
    return data_[RingWrap<Capacity>(ring_offset_ + ring_index_)];
  }

  pointer operator->() const noexcept { return &**this; }

  Iterator operator++(int) noexcept {
    Iterator copy(*this);
    operator++();
    return copy;
  }

  Iterator& operator++() noexcept {
    ++ring_index_;
    return *this;
  }

  Iterator operator--(int) noexcept {
    Iterator copy(*this);
    operator--();
    return copy;
  }

  Iterator& operator--() noexcept {
    --ring_index_;
    return *this;
  }

  Iterator& operator+=(difference_type n) noexcept {
    ring_index_ += n;
    return *this;
  }

  Iterator operator+(difference_type n) const noexcept {
    return Iterator(data_, ring_offset_, ring_index_ + n);
  }

  Iterator& operator-=(difference_type n) noexcept {
    ring_index_ -= n;
    return *this;
  }

  Iterator operator-(difference_type n) const noexcept {
    return Iterator(data_, ring_offset_, ring_index_ - n);
  }

  reference operator[](difference_type n) const { return *(*this + n); }

  friend difference_type operator-(const Iterator& lhs,
                                   const Iterator& rhs) noexcept {
    return lhs.ring_index_ > rhs.ring_index_
               ? lhs.ring_index_ - rhs.ring_index_
               : -(rhs.ring_index_ - lhs.ring_index_);
  }

  friend Iterator operator+(difference_type lhs, const Iterator& rhs) noexcept {
    return rhs + lhs;
  }

  friend bool operator<(const Iterator& lhs, const Iterator& rhs) noexcept {
    // Comparison via std::tie uses std::tuple::operator<, which compares its
    // elements lexicographically.
    return std::tie(lhs.data_, lhs.ring_offset_, lhs.ring_index_) <
           std::tie(rhs.data_, rhs.ring_offset_, rhs.ring_index_);
  }

  friend bool operator>(const Iterator& lhs, const Iterator& rhs) noexcept {
    return rhs < lhs;
  }

  friend bool operator<=(const Iterator& lhs, const Iterator& rhs) noexcept {
    return !(rhs < lhs);
  }

  friend bool operator>=(const Iterator& lhs, const Iterator& rhs) noexcept {
    return !(lhs < rhs);
  }

  friend bool operator==(const Iterator& lhs, const Iterator& rhs) noexcept {
    return &*lhs == &*rhs;
  }

  friend bool operator!=(const Iterator& lhs, const Iterator& rhs) noexcept {
    return !(lhs == rhs);
  }

  template <typename P, typename A, std::size_t C, typename OutputIt>
  // https://github.com/llvm/llvm-project/issues/47430
  // NOLINTNEXTLINE(readability-redundant-declaration)
  friend OutputIt copy(const Iterator<P, A, C>& begin,
                       const Iterator<P, A, C>& end,
                       OutputIt out);

 private:
  pointer data_{};

  // Keeping both ring_offset_ and ring_index_ around is a little redundant,
  // algorithmically, but it makes it much easier to express iterator-mutating
  // operations.

  // Physical index of begin().
  size_type ring_offset_{};
  // Logical index of this iterator.
  size_type ring_index_{};
};

/**
 * @see baudvine::copy
 * @private
 */
template <typename Ptr,
          typename AllocTraits,
          std::size_t Capacity,
          typename OutputIt>
OutputIt copy(const Iterator<Ptr, AllocTraits, Capacity>& begin,
              const Iterator<Ptr, AllocTraits, Capacity>& end,
              OutputIt out) {
  assert(begin <= end);

  if (begin == end) {
    // Empty range, pass
  } else if (&*end > &*begin) {
    // Fully contiguous range.
    out = std::copy(&*begin, &*end, out);
  } else {
    // Copy in two sections.
    out = std::copy(&*begin, &begin.data_[Capacity + 1], out);
    out = std::copy(end.data_, &*end, out);
  }

  return out;
}
}  // namespace ringbuf
}  // namespace detail

/**
 * An STL-like ring buffer with dynamic allocation and compile-time capacity
 * limits.
 *
 * @tparam Elem The type of elements contained by the ring buffer.
 * @tparam Capacity The maximum size of the ring buffer, and the fixed size of
 *         the backing array.
 * @tparam Allocator The allocator type to use for storage and element
           construction.
 */
template <typename Elem,
          std::size_t Capacity,
          typename Allocator = std::allocator<Elem>>
class RingBuf {
 public:
  using allocator_type = Allocator;
  using alloc_traits = std::allocator_traits<allocator_type>;
  using value_type = Elem;
  using pointer = typename alloc_traits::pointer;
  using const_pointer = typename alloc_traits::const_pointer;
  using reference = decltype(*pointer{});
  using const_reference = decltype(*const_pointer{});
  using iterator = detail::ringbuf::Iterator<pointer, alloc_traits, Capacity>;
  using const_iterator =
      detail::ringbuf::Iterator<const_pointer, alloc_traits, Capacity>;
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;
  using difference_type = typename alloc_traits::difference_type;
  using size_type = typename alloc_traits::size_type;
  using unsigned_difference =
      typename std::make_unsigned<difference_type>::type;

  using self = RingBuf<Elem, Capacity>;

 private:
  // The allocator is used to allocate memory, and to construct and destroy
  // elements.
  allocator_type alloc_{};

  // The start of the dynamically allocated backing array.
  pointer data_{nullptr};
  // The next position to write to for push_back().
  size_type next_{0U};

  // Start of the ring buffer in data_.
  size_type ring_offset_{0U};
  // The number of elements in the ring buffer (distance between begin() and
  // end()).
  size_type size_{0U};

  constexpr static size_type Decrement(const size_type index) {
    return index > 0 ? index - 1 : Capacity;
  }

  constexpr static size_type Increment(const size_type index) {
    return index < (Capacity) ? index + 1 : 0;
  }

  // Swap everything but the allocator - caller has to figure that out
  // separately.
  void Swap(RingBuf& other) noexcept {
    std::swap(data_, other.data_);
    std::swap(next_, other.next_);
    std::swap(ring_offset_, other.ring_offset_);
    std::swap(size_, other.size_);
  }

  // Move things after pop_front.
  void ShrinkFront() noexcept {
    ring_offset_ = Increment(ring_offset_);
    // Precondition: size != 0 (when it is, pop_front returns early.)
    size_--;
  }

  // Move things around before pop_back destroys the last entry.
  void ShrinkBack() noexcept {
    next_ = Decrement(next_);
    // Precondition: size != 0 (when it is, pop_back returns early.)
    size_--;
  }

  // Move things around before emplace_front constructs its new entry.
  void GrowFront() noexcept {
    // Move ring_offset_ down, and possibly around
    ring_offset_ = Decrement(ring_offset_);
    // Precondition: size != Capacity (when it is, emplace_front pop_backs
    // first.)
    size_++;
  }

  // Move things around after emplace_back.
  void GrowBack() noexcept {
    next_ = Increment(next_);
    // Precondition: size != Capacity (when it is, emplace_back pop_fronts
    // first)
    size_++;
  }

  iterator UnConstIterator(const_iterator it) const {
    return iterator(data_, ring_offset_, it - begin());
  }

  reference UncheckedAt(size_type index) { return (*this)[index]; }

 public:
  /**
   * Construct a new ring buffer object with a default-constructed allocator,
   * and allocate the required memory.
   *
   * Allocates Capacity + 1 to allow for strong exception guarantees in
   * emplace_front/back.
   */
  RingBuf() : RingBuf(allocator_type{}){};
  /**
   * Construct a new ring buffer object with the provided allocator, and
   * allocate the required memory.
   *
   * Allocates Capacity + 1 to allow for strong exception guarantees in
   * emplace_front/back.
   *
   * @param allocator The allocator to use for storage and element construction.
   */
  explicit RingBuf(const allocator_type& allocator)
      : alloc_(allocator),
        data_(alloc_traits::allocate(alloc_, Capacity + 1)) {}

  /**
   * Destroy the ring buffer object.
   *
   * Destroys the active elements via clear() and deallocates the backing array.
   */
  ~RingBuf() {
    clear();
    alloc_traits::deallocate(alloc_, data_, Capacity + 1);
  }
  /**
   * Construct a new RingBuf object out of another, using elementwise
   * copy assignment.
   *
   * @param other The RingBuf to copy values from.
   * @todo maybe allow other (smaller) sizes as input?
   */
  RingBuf(const RingBuf& other)
      : RingBuf(
            other,
            alloc_traits::select_on_container_copy_construction(other.alloc_)) {
  }
  /**
   * Allocator-extended copy constructor.
   *
   * @param other The RingBuf to copy values from.
   * @param allocator The allocator to use for storage and element construction.
   * @todo maybe allow other (smaller) sizes as input?
   * @todo Use memcpy/std::copy if Elem is POD
   */
  RingBuf(const RingBuf& other, const allocator_type& allocator)
      : RingBuf(allocator) {
    clear();

    for (const auto& value : other) {
      push_back(value);
    }
  }
  /**
   * Construct a new RingBuf object out of another, using bulk move assignment.
   *
   * @param other The RingBuf to move the data out of.
   */
  RingBuf(RingBuf&& other) noexcept : RingBuf(std::move(other.alloc_)) {
    Swap(other);
  }
  /**
   * Allocator-extended move constructor.
   *
   * May move elementwise if the provided allocator and other's allocator are
   * not the same.
   *
   * @param other The RingBuf to move the data out of.
   * @param allocator The allocator to use for storage and element construction.
   */
  RingBuf(RingBuf&& other, const allocator_type& allocator)
      : RingBuf(allocator) {
    if (other.alloc_ == allocator) {
      Swap(other);
    } else {
      for (auto& element : other) {
        emplace_back(std::move(element));
      }
    }
  }

  /**
   * Copy a RingBuf into this one.
   *
   * First clear()s this RingBuf, and then copies @c other element by element.
   *
   * @param other The RingBuf to copy from.
   * @returns This RingBuf.
   */
  RingBuf& operator=(const RingBuf& other) {
    clear();

    detail::ringbuf::CopyAllocator(alloc_, other.alloc_);

    for (const auto& value : other) {
      push_back(value);
    }
    return *this;
  }
  /**
   * Move a RingBuf into this one.
   *
   * If the allocator is the same or can be moved as well, no elementwise moves
   * are performed.
   *
   * @param other The RingBuf to copy from.
   * @returns This RingBuf.
   */
  RingBuf& operator=(RingBuf&& other) noexcept(
      alloc_traits::propagate_on_container_move_assignment::value ||
      std::is_nothrow_move_constructible<value_type>::value) {
    if (alloc_traits::propagate_on_container_move_assignment::value ||
        alloc_ == other.alloc_) {
      // We're either getting the other's allocator or they're already the same,
      // so swap data in one go.
      detail::ringbuf::MoveAllocator(alloc_, other.alloc_);
      Swap(other);
    } else {
      // Different allocators and can't swap them, so move elementwise.
      clear();
      for (auto& element : other) {
        emplace_back(std::move(element));
      }
    }

    return *this;
  }

  /**
   * Get a copy of the allocator used by this RingBuf.
   */
  allocator_type get_allocator() const { return alloc_; }

  /**
   * Returns the first element in the ring buffer.
   * @throws std::out_of_range The buffer is empty.
   */
  reference front() { return at(0); }
  /**
   * Returns the first element in the ring buffer.
   * @throws std::out_of_range The buffer is empty.
   */
  const_reference front() const { return at(0); }
  /**
   * Returns he last element in the ring buffer.
   * @throws std::out_of_range The buffer is empty.
   */
  reference back() { return at(size() - 1); }
  /**
   * Returns he last element in the ring buffer.
   * @throws std::out_of_range The buffer is empty.
   */
  const_reference back() const { return at(size() - 1); }

  /**
   * Retrieve an element from the ring buffer without range checking.
   *
   * The behaviour is undefined when @c index is outside [0, size()).
   *
   * @param index The logical index into the ring buffer.
   * @returns A const reference to the element.
   */
  const_reference operator[](const size_type index) const {
    return data_[detail::ringbuf::RingWrap<Capacity>(ring_offset_ + index)];
  }
  /**
   * Retrieve an element from the ring buffer without range checking.
   *
   * The behaviour is undefined when @c index is outside [0, size()).
   *
   * @param index The logical index into the ring buffer.
   * @returns A reference to the element.
   */
  reference operator[](const size_type index) {
    return data_[detail::ringbuf::RingWrap<Capacity>(ring_offset_ + index)];
  }
  /**
   * Retrieve an element from the ring buffer with range checking.
   *
   * @param index The logical index into the ring buffer. Must be in range
   *              [0, size()).
   * @returns A const reference to the element.
   * @throw std::out_of_range The index is out of range.
   */
  const_reference at(const size_type index) const {
    if (index >= size()) {
      throw std::out_of_range("RingBuf::at: index >= Size");
    }
    return (*this)[index];
  }
  /**
   * Retrieve an element from the ring buffer with range checking.
   *
   * @param index The logical index into the ring buffer. Must be in range
   *              [0, size()).
   * @returns A reference to the element.
   * @throws std::out_of_range The index is out of range.
   */
  reference at(const size_type index) {
    if (index >= size()) {
      throw std::out_of_range("RingBuf::at: index >= Size");
    }
    return (*this)[index];
  }

  /**
   * Get an iterator pointing to the first element.
   */
  iterator begin() noexcept { return iterator(&data_[0], ring_offset_, 0); }
  /**
   * Get an iterator pointing to one past the last element.
   */
  iterator end() noexcept { return iterator(&data_[0], ring_offset_, size()); }
  /**
   * Get a const iterator pointing to the first element.
   */
  const_iterator begin() const noexcept {
    return const_iterator(&data_[0], ring_offset_, 0);
  }
  /**
   * Get a const iterator pointing to one past the last element.
   */
  const_iterator end() const noexcept {
    return const_iterator(&data_[0], ring_offset_, size());
  }
  /**
   * Get a const iterator pointing to the first element.
   */
  const_iterator cbegin() const noexcept {
    return const_cast<self const&>(*this).begin();
  }
  /**
   * Get a const iterator pointing to one past the last element.
   */
  const_iterator cend() const noexcept {
    return const_cast<self const&>(*this).end();
  }
  /**
   * Get a reverse iterator pointing to the last element.
   */
  reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }
  /**
   * Get a reverse iterator pointing to one before the first element.
   */
  reverse_iterator rend() noexcept { return reverse_iterator(begin()); }
  /**
   * Get a const reverse iterator pointing to the last element.
   */
  const_reverse_iterator rbegin() const noexcept {
    return const_reverse_iterator(end());
  }
  /**
   * Get a const reverse iterator pointing to one before the first element.
   */
  const_reverse_iterator rend() const noexcept {
    return const_reverse_iterator(begin());
  }
  /**
   * Get a const reverse iterator pointing to the last element.
   */
  const_reverse_iterator crbegin() const noexcept {
    return const_cast<self const&>(*this).rbegin();
  }
  /**
   * Get a const reverse iterator pointing to one before the first element.
   */
  const_reverse_iterator crend() const noexcept {
    return const_cast<self const&>(*this).rend();
  }

  /**
   * Get whether the ring buffer is empty (size() == 0)
   */
  bool empty() const noexcept { return size() == 0; }
  /**
   * Get the number of elements in the ring buffer.
   */
  size_type size() const noexcept { return size_; }
  /**
   * Get the maximum number of elements in this ring buffer (Capacity).
   */
  constexpr size_type max_size() const noexcept { return Capacity; }

  /**
   * Push a new element at the front of the ring buffer, popping the back if
   * necessary.
   *
   * @param value The value to copy into the ring buffer.
   */
  void push_front(const_reference value) { emplace_front(value); }
  /**
   * Push a new element at the front of the ring buffer, popping the back if
   * necessary.
   *
   * @param value The value to move into the ring buffer.
   */
  void push_front(value_type&& value) { emplace_front(std::move(value)); }
  /**
   * Construct a new element in-place before the front of the ring buffer,
   * popping the back if necessary.
   *
   * @tparam Args Arguments to the element constructor.
   * @param args Arguments to the element constructor.
   */
  template <typename... Args>
  reference emplace_front(Args&&... args) {
    if (max_size() == 0) {
      // A buffer of size zero is conceptually sound, so let's support it.
      return UncheckedAt(0);
    }

    alloc_traits::construct(alloc_, &data_[Decrement(ring_offset_)],
                            std::forward<Args>(args)...);

    // If required, make room for next time.
    if (size() == max_size()) {
      pop_back();
    }
    GrowFront();
    return UncheckedAt(0);
  }

  /**
   * Push a new element into the ring buffer, popping the front if necessary.
   *
   * @param value The value to copy into the ring buffer.
   */
  void push_back(const_reference value) { emplace_back(value); }
  /**
   * Push a new element into the ring buffer, popping the front if necessary.
   *
   * @param value The value to move into the ring buffer.
   */
  void push_back(value_type&& value) { emplace_back(std::move(value)); }
  /**
   * Construct a new element in-place at the end of the ring buffer, popping the
   * front if necessary.
   *
   * @tparam Args Arguments to the element constructor.
   * @param args Arguments to the element constructor.
   */
  template <typename... Args>
  reference emplace_back(Args&&... args) {
    if (max_size() == 0) {
      // A buffer of size zero is conceptually sound, so let's support it.
      return UncheckedAt(0);
    }

    alloc_traits::construct(alloc_, &data_[next_], std::forward<Args>(args)...);

    // If required, make room for next time.
    if (size() == max_size()) {
      pop_front();
    }
    GrowBack();
    return UncheckedAt(size() - 1);
  }

  /**
   * Pop an element off the front, destroying the first element in the ring
   * buffer.
   */
  void pop_front() noexcept {
    if (size() == 0) {
      return;
    }

    alloc_traits::destroy(alloc_, &data_[ring_offset_]);
    ShrinkFront();
  }
  /**
   * Pop an element off the back, destroying the last element in the ring
   * buffer.
   */
  void pop_back() noexcept {
    if (size() == 0) {
      return;
    }

    ShrinkBack();
    alloc_traits::destroy(alloc_, &data_[next_]);
  }
  /**
   * Remove all elements from the ring buffer, destroying each one starting at
   * the front.
   *
   * After clear(), size() == 0.
   */
  void clear() noexcept(noexcept(pop_front())) {
    // It might be fractionally more efficient to iterate through begin..end and
    // allocator::destroy each one, but this is a lot nicer to read.
    while (!empty()) {
      pop_front();
    }
  }

  /**
   * Erase elements in the range [first, last).
   * @param from The first element to erase.
   * @param to One past the last element to erase.
   * @returns Iterator pointing to the element after @c to.
   */
  iterator erase(const_iterator from, const_iterator to) noexcept(
      noexcept(pop_front()) && std::is_nothrow_move_assignable<Elem>::value) {
    if (from == to) {
      return UnConstIterator(to);
    }

    const iterator first = UnConstIterator(from);
    const iterator last = UnConstIterator(to);

    const auto leading = first - begin();
    const auto trailing = end() - last;
    if (leading > trailing) {
      // Move from back towards first
      for (auto i = 0; i < trailing; i++) {
        first[i] = std::move(last[i]);
      }
      const auto to_pop = last - first;
      for (auto i = 0; i < to_pop; i++) {
        pop_back();
      }
    } else {
      // Move from front towards last
      for (auto i = -1; i >= -leading; i--) {
        last[i] = std::move(first[i]);
      }
      const auto to_pop = last - first;
      for (auto i = 0; i < to_pop; i++) {
        pop_front();
      }
    }

    return end() - trailing;
  }

  /**
   * Erase an element.
   *
   * @param pos An iterator pointing to the element to erase.
   * @returns Iterator pointing to the element after @c pos.
   */
  iterator erase(const_iterator pos) noexcept(noexcept(erase(pos, pos + 1))) {
    return erase(pos, pos + 1);
  }

  /**
   * Swap this ring buffer with another using std::swap.
   *
   * @param other The RingBuf to swap with.
   */
  void swap(RingBuf& other) noexcept {
    detail::ringbuf::SwapAllocator(alloc_, other.alloc_);
    Swap(other);
  }

  /**
   * Elementwise lexicographical comparison of two ring buffers.
   *
   * @returns True if the left-hand side compares as less than the right.
   */
  friend bool operator<(const RingBuf& lhs, const RingBuf& rhs) {
    return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(),
                                        rhs.end());
  }
  /**
   * Elementwise lexicographical comparison of two ring buffers.
   *
   * @returns True if the left-hand side compares as more than the right.
   */
  friend bool operator>(const RingBuf& lhs, const RingBuf& rhs) {
    return rhs < lhs;
  }
  /**
   * Elementwise comparison of two ring buffers.
   *
   * @returns True if @c lhs is equal to @c rhs.
   */
  friend bool operator==(const RingBuf& lhs, const RingBuf& rhs) {
    if (lhs.size() != rhs.size()) {
      return false;
    }

    return std::equal(lhs.begin(), lhs.end(), rhs.begin());
  }
  /**
   * Elementwise comparison of two ring buffers.
   *
   * @returns True if @c lhs is greater than or equal to @c rhs.
   */
  friend bool operator>=(const RingBuf& lhs, const RingBuf& rhs) {
    return !(lhs < rhs);
  }
  /**
   * Elementwise comparison of two ring buffers.
   *
   * @returns True if @c lhs is less than or equal to @c rhs.
   */
  friend bool operator<=(const RingBuf& lhs, const RingBuf& rhs) {
    return !(lhs > rhs);
  }
  /**
   * Elementwise comparison of two ring buffers.
   *
   * @returns True if @c lhs is not equal to @c rhs.
   */
  friend bool operator!=(const RingBuf& lhs, const RingBuf& rhs) {
    return !(lhs == rhs);
  }
};

/**
 * Copy the elements in the range [@c begin, @c end) to a destination range
 * starting at @c out.
 *
 * Can be used like std::copy:
 * @code
 * std::vector<int> vec;
 * baudvine::copy(ring.begin(), ring.end(), std::back_inserter(vec));
 * @endcode
 *
 * @tparam Ptr The pointer type of the input iterator.
 * @tparam AllocTraits The allocator traits of the input iterator.
 * @tparam Capacity The capacity of the input iterator.
 * @param begin Start of the source range.
 * @param end End of the source range, one past the last element to copy.
 * @param out Start of the destination range.
 * @returns One past the last copied element in the destination range.
 */
template <typename Ptr,
          typename AllocTraits,
          std::size_t Capacity,
          typename OutputIt>
OutputIt copy(
    const detail::ringbuf::Iterator<Ptr, AllocTraits, Capacity>& begin,
    const detail::ringbuf::Iterator<Ptr, AllocTraits, Capacity>& end,
    OutputIt out) {
  return detail::ringbuf::copy(begin, end, out);
}

}  // namespace baudvine