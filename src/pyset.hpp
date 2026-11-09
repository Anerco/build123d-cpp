// CPython 3.12's set and frozenset (Objects/setobject.c) for non-negative integer keys, whose
// hash is the integer itself. Code ported from Python whose results depend on the iteration
// order of sets (or on set.pop()) uses it: the hash table, its probing, resizing, dummy entries
// and the "finger" of pop() follow CPython exactly, so iteration orders are Python's.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace build123d::pyset {

class Set {
public:
    /// set() / frozenset()
    Set() : table_(kMinSize, kEmpty) {}

    /// set(range(n))
    static Set from_range(long long n) {
        Set s;
        for (long long i = 0; i < n; ++i) s.add(i);
        return s;
    }

    /// set(iterable) / frozenset(iterable) for a list or another iterable of ints.
    template <class Container>
    static Set from_iterable(const Container& values) {
        Set s;
        for (const auto& v : values) s.add(static_cast<long long>(v));
        return s;
    }

    /// set(other) / frozenset(other) / other.copy() where other is a set (frozenset(f) of a
    /// frozenset f is f itself: copy the object instead).
    Set copy() const {
        Set s;
        s.merge(*this);
        return s;
    }

    std::size_t size() const { return used_; }
    /// The size of the hash table (sys.getsizeof reflects it in Python).
    std::size_t table_size() const { return mask_ + 1; }
    bool empty() const { return used_ == 0; }

    bool contains(long long key) const {
        const std::size_t index = lookkey(key);
        return table_[index] >= 0;
    }

    /// set.add
    void add(long long key) { add_entry(key); }

    /// set.discard; true if the key was there.
    bool discard(long long key) {
        const std::size_t index = lookkey(key);
        if (table_[index] < 0) return false;
        table_[index] = kDummy;
        --used_;
        return true;
    }

    /// set.remove (KeyError -> std::out_of_range)
    void remove(long long key) {
        if (!discard(key)) throw std::out_of_range("KeyError: " + std::to_string(key));
    }

    /// set.pop
    long long pop() {
        if (used_ == 0) throw std::out_of_range("pop from an empty set");
        std::size_t i = finger_ & mask_;
        while (table_[i] < 0) {
            ++i;
            if (i > mask_) i = 0;
        }
        const long long key = table_[i];
        table_[i] = kDummy;
        --used_;
        finger_ = i + 1;
        return key;
    }

    /// set.update(other) for a set or frozenset (set_merge).
    void update(const Set& other) { merge(other); }

    /// set.clear() (set_clear_internal: pop()'s finger stays)
    void clear() {
        table_.assign(kMinSize, kEmpty);
        mask_ = kMinSize - 1;
        fill_ = 0;
        used_ = 0;
    }

    /// set.difference_update(other) for a set or frozenset.
    void difference_update(const Set& other) {
        if (&other == this) {
            clear();
            return;
        }
        if ((other.used_ >> 3) > used_) {
            const Set common = intersection(other);
            for (long long key : common.items()) discard(key);
        } else {
            for (long long key : other.items()) discard(key);
        }
        // If more than 1/4th are dummies, then resize them away.
        if ((fill_ - used_) <= mask_ / 4) return;
        table_resize(used_ > 50000 ? used_ * 2 : used_ * 4);
    }

    /// self - other (set_difference) for a set or frozenset `other`.
    Set difference(const Set& other) const {
        if ((used_ >> 2) > other.used_) {
            Set result = copy();
            result.difference_update(other);
            return result;
        }
        Set result;
        for (std::size_t i = 0; i <= mask_; ++i) {
            const long long key = table_[i];
            if (key >= 0 && !other.contains(key)) result.add_entry(key);
        }
        return result;
    }

    /// self & other (set_intersection)
    Set intersection(const Set& other) const {
        if (&other == this) return copy();
        Set result;
        const Set* so = this;
        const Set* o = &other;
        if (o->used_ > so->used_) std::swap(so, o);
        for (std::size_t i = 0; i <= o->mask_; ++i) {
            const long long key = o->table_[i];
            if (key >= 0 && so->contains(key)) result.add_entry(key);
        }
        return result;
    }

    /// self | other (set_or)
    Set operator|(const Set& other) const {
        Set result = copy();
        if (&other != this) result.merge(other);
        return result;
    }

    /// self.union(*others) (set_union): a copy of self updated with each other set.
    Set union_with(const std::vector<const Set*>& others) const {
        Set result = copy();
        for (const Set* other : others) {
            if (other == this) continue;
            result.merge(*other);
        }
        return result;
    }

    /// The keys in iteration order.
    std::vector<long long> items() const {
        std::vector<long long> out;
        out.reserve(used_);
        for (std::size_t i = 0; i <= mask_; ++i) {
            if (table_[i] >= 0) out.push_back(table_[i]);
        }
        return out;
    }

    /// The keys in ascending order (sorted(s)).
    std::vector<long long> sorted() const;

private:
    static constexpr std::size_t kMinSize = 8;
    static constexpr int kLinearProbes = 9;
    static constexpr int kPerturbShift = 5;
    static constexpr long long kEmpty = -1;  // key NULL
    static constexpr long long kDummy = -2;  // key dummy, hash -1

    std::vector<long long> table_;
    std::size_t mask_ = kMinSize - 1;
    std::size_t fill_ = 0;
    std::size_t used_ = 0;
    std::size_t finger_ = 0;

    /// set_lookkey: the index of the key's entry, or of the unused entry ending the search.
    std::size_t lookkey(long long key) const {
        const auto hash = static_cast<std::size_t>(key);
        std::size_t perturb = hash;
        std::size_t i = hash & mask_;
        while (true) {
            std::size_t index = i;
            int probes = (i + kLinearProbes <= mask_) ? kLinearProbes : 0;
            do {
                const long long k = table_[index];
                if (k == kEmpty) return index;
                if (k == key) return index;
                ++index;
            } while (probes--);
            perturb >>= kPerturbShift;
            i = (i * 5 + 1 + perturb) & mask_;
        }
    }

    /// set_add_entry
    void add_entry(long long key) {
        const auto hash = static_cast<std::size_t>(key);
        const std::size_t mask = mask_;
        std::size_t i = hash & mask;
        std::size_t perturb = hash;
        std::size_t freeslot = static_cast<std::size_t>(-1);
        std::size_t found = 0;
        while (true) {
            std::size_t index = i;
            int probes = (i + kLinearProbes <= mask) ? kLinearProbes : 0;
            bool done = false;
            do {
                const long long k = table_[index];
                if (k == kEmpty) {
                    found = index;
                    done = true;
                    break;
                }
                if (k == key) return;  // found_active
                if (k == kDummy) freeslot = index;
                ++index;
            } while (probes--);
            if (done) break;
            perturb >>= kPerturbShift;
            i = (i * 5 + 1 + perturb) & mask;
        }
        if (freeslot != static_cast<std::size_t>(-1)) {
            ++used_;
            table_[freeslot] = key;
            return;
        }
        ++fill_;
        ++used_;
        table_[found] = key;
        if (fill_ * 5 < mask * 3) return;
        table_resize(used_ > 50000 ? used_ * 2 : used_ * 4);
    }

    /// set_insert_clean
    static void insert_clean(std::vector<long long>& table, std::size_t mask, long long key) {
        const auto hash = static_cast<std::size_t>(key);
        std::size_t perturb = hash;
        std::size_t i = hash & mask;
        while (true) {
            if (table[i] == kEmpty) {
                table[i] = key;
                return;
            }
            if (i + kLinearProbes <= mask) {
                for (int j = 1; j <= kLinearProbes; ++j) {
                    if (table[i + static_cast<std::size_t>(j)] == kEmpty) {
                        table[i + static_cast<std::size_t>(j)] = key;
                        return;
                    }
                }
            }
            perturb >>= kPerturbShift;
            i = (i * 5 + 1 + perturb) & mask;
        }
    }

    /// set_table_resize
    void table_resize(std::size_t minused) {
        std::size_t newsize = kMinSize;
        while (newsize <= minused) newsize <<= 1;
        if (newsize == kMinSize && mask_ == kMinSize - 1 && fill_ == used_) {
            return;  // the small table without dummies: nothing to do
        }
        std::vector<long long> old = std::move(table_);
        table_.assign(newsize, kEmpty);
        mask_ = newsize - 1;
        fill_ = used_;
        for (const long long key : old) {
            if (key >= 0) insert_clean(table_, mask_, key);
        }
    }

    /// set_merge
    void merge(const Set& other) {
        if (&other == this || other.used_ == 0) return;
        if ((fill_ + other.used_) * 5 >= mask_ * 3) table_resize((used_ + other.used_) * 2);
        if (fill_ == 0 && mask_ == other.mask_ && other.fill_ == other.used_) {
            // copy the table as it is
            for (std::size_t i = 0; i <= other.mask_; ++i) {
                if (other.table_[i] >= 0) table_[i] = other.table_[i];
            }
            fill_ = other.fill_;
            used_ = other.used_;
            return;
        }
        if (fill_ == 0) {
            fill_ = other.used_;
            used_ = other.used_;
            for (std::size_t i = 0; i <= other.mask_; ++i) {
                if (other.table_[i] >= 0) insert_clean(table_, mask_, other.table_[i]);
            }
            return;
        }
        for (std::size_t i = 0; i <= other.mask_; ++i) {
            if (other.table_[i] >= 0) add_entry(other.table_[i]);
        }
    }
};

inline std::vector<long long> Set::sorted() const {
    std::vector<long long> out = items();
    std::sort(out.begin(), out.end());
    return out;
}

}  // namespace build123d::pyset
