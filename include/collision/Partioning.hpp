// Partitioning.hpp
// This is a basic uniform-grid spatial hashing, used to find candidates close to each other in world-space.

#pragma once
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <cstdint>
#include <cmath>
#include "collision/AABB.hpp" 

namespace partioning {

struct GridConfig {
    float cellSize = 2.0f; // Size of each grid cell 
};

// Packs 2D cell coords into one 64 bit key
inline uint64_t cellKey(int cx, int cy) {
    return (uint64_t(uint32_t(cx)) << 32) | uint32_t(cy);
}

// Packs a pair of body indices into a unique 64 bit key
inline uint64_t pairKey(int a, int b) {
    if (a > b) std::swap(a, b);
    return (uint64_t(uint32_t(a)) << 32) | uint32_t(b);
}

inline int cellCoord(float x, float cellSize) {
    return static_cast<int>(std::floor(x / cellSize));
}

inline std::vector<std::pair<int,int>> buildPairsFromAABBs(const std::vector<AABB>& aabbs, const GridConfig& cfg) {

    // Build candidate pairs from AABBs using a spatial hash grid.
    // Returns pairs of indices (i,j) into bodies/AABB arrays.

    std::unordered_map<uint64_t, std::vector<int>> buckets;
    buckets.reserve(aabbs.size() * 2);

    // Insert indices into buckets
    for (int i = 0; i < (int)aabbs.size(); ++i) {
        const AABB& b = aabbs[i];

        // Compute grid-cell range overlapped by this AABB
        int x0 = cellCoord(b.min.x, cfg.cellSize);
        int x1 = cellCoord(b.max.x, cfg.cellSize);
        int y0 = cellCoord(b.min.y, cfg.cellSize);
        int y1 = cellCoord(b.max.y, cfg.cellSize);

        for (int cy = y0; cy <= y1; ++cy) {
            for (int cx = x0; cx <= x1; ++cx) {
                buckets[cellKey(cx, cy)].push_back(i);
            }
        }
    }

    // Generate unique pairs within each bucket
    std::unordered_set<uint64_t> seen;
    seen.reserve(aabbs.size() * 8);
    std::vector<std::pair<int,int>> pairs;
    pairs.reserve(aabbs.size() * 4);

    for (auto& [key, ids] : buckets) {  // Iterate over each occupied grid cell
       
        if (ids.size() < 2) continue;

        for (size_t a = 0; a < ids.size(); ++a) { // Generate all unique pairs within this cell
            for (size_t b = a + 1; b < ids.size(); ++b) {
                
                int i = ids[a];
                int j = ids[b];

                // Create unique order-independent pair key
                uint64_t pk = pairKey(i, j);
                if (seen.insert(pk).second) {
                    // Only emit pair once across all cells
                    pairs.push_back({i, j});
                }

            }
        }

    }

    return pairs; // Returns canditate pairs for narrow testing 
}

} // namespace broadphase
