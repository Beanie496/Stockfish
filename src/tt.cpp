/*
  Stockfish, a UCI chess playing engine derived from Glaurung 2.1
  Copyright (C) 2004-2024 The Stockfish developers (see AUTHORS file)

  Stockfish is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stockfish is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "tt.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include "misc.h"

namespace Stockfish {

// Populates the TTEntry with a new node's data, possibly
// overwriting an old position. The update is not atomic and can be racy.
void TTEntry::save(
  Key k, Value v, bool pv, Bound b, Depth d, Move m, Value ev, uint8_t generation8) {

    // Preserve any existing move for the same position
    if (m || uint16_t(k) != key16)
        move16 = m;

    // Overwrite less valuable entries (cheapest checks first)
    if (b == BOUND_EXACT || uint16_t(k) != key16 || d - DEPTH_OFFSET + 2 * pv > depth8 - 4)
    {
        assert(d > DEPTH_OFFSET);
        assert(d < 256 + DEPTH_OFFSET);

        key16     = uint16_t(k);
        depth8    = uint8_t(d - DEPTH_OFFSET);
        genBound8 = uint8_t(generation8 | uint8_t(pv) << 2 | b);
        value16   = int16_t(v);
        eval16    = int16_t(ev);
    }
}


// Sets the size of the transposition table, measured in megabytes. The size of
// the table will be a multiple of the size of a TTEntry.
void TranspositionTable::resize(size_t mbSize, int threadCount) {
    aligned_large_pages_free(table);

    entryCount = mbSize * 1024 * 1024 / sizeof(TTEntry);

    table = static_cast<TTEntry*>(aligned_large_pages_alloc(entryCount * sizeof(TTEntry)));
    if (!table)
    {
        std::cerr << "Failed to allocate " << mbSize << "MB for transposition table." << std::endl;
        exit(EXIT_FAILURE);
    }

    clear(threadCount);
}


// Initializes the entire transposition table to zero,
// in a multi-threaded way.
void TranspositionTable::clear(size_t threadCount) {
    std::vector<std::thread> threads;

    for (size_t idx = 0; idx < size_t(threadCount); ++idx)
    {
        threads.emplace_back([this, idx, threadCount]() {
            // Thread binding gives faster search on systems with a first-touch policy
            if (threadCount > 8)
                WinProcGroup::bind_this_thread(idx);

            // Each thread will zero its part of the hash table
            const size_t stride = size_t(entryCount / threadCount), start = size_t(stride * idx),
                         len = idx != size_t(threadCount) - 1 ? stride : entryCount - start;

            std::memset(&table[start], 0, len * sizeof(TTEntry));
        });
    }

    for (std::thread& th : threads)
        th.join();
}


// Looks up the current position in the transposition table. It returns whether
// or not the position is found and a pointer to the TTEntry to be replaced.
TTEntry* TranspositionTable::probe(const Key key, bool& found) const {

    TTEntry* const tte   = entry(key);
    const uint16_t key16 = uint16_t(key);  // Use the low 16 bits as key inside the cluster
    found                = false;          // Assume we have not found an entry

    if (tte->key16 == key16 || !tte->depth8)
    {
        constexpr uint8_t lowerBits = GENERATION_DELTA - 1;

        // Refresh with new generation, keeping the lower bits the same.
        tte->genBound8 = uint8_t(generation8 | (tte->genBound8 & lowerBits));
        found          = bool(tte->depth8);
    }

    return tte;
}


// Prefetches the cache line/s containing the TTEntry associated with the given
// key.
void TranspositionTable::prefetch_entry(Key key) {
    TTEntry* const tte = entry(key);
    prefetch(tte);
    // this prefetches the cache line containing the last byte of the entry,
    // guaranteeing the whole entry is in the cache
    prefetch((char*) tte + sizeof(TTEntry) - 1);
}


// Returns an approximation of the hashtable
// occupation during a search. The hash is x permill full, as per UCI protocol.
// Only counts entries which match the current generation.
int TranspositionTable::hashfull() const {

    int cnt = 0;
    for (int i = 0; i < 1000; ++i)
        cnt += table[i].depth8 && (table[i].genBound8 & GENERATION_MASK) == generation8;

    return cnt;
}

}  // namespace Stockfish
