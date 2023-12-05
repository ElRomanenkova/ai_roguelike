#pragma once
#include <flecs.h>
#include <vector>

#include <raylib.h>
#include "math.h"
#include "ecsTypes.h"

struct PortalConnection
{
  size_t connIdx;
  float score;
};

struct PathPortal
{
  size_t startX, startY;
  size_t endX, endY;
  std::vector<PortalConnection> conns;
};

inline bool operator==(const PathPortal& lhs, const PathPortal& rhs)
{
  return lhs.startX == rhs.startX && lhs.startY == rhs.startY && lhs.endX == rhs.endX && lhs.endY == rhs.endY;
}

struct DungeonPortals
{
  size_t tileSplit;
  std::vector<PathPortal> portals;
  std::vector<std::vector<size_t>> tilePortalsIndices;
};

void prebuild_map(flecs::world &ecs);
std::vector<IVec2> find_hierarchical_path(const DungeonPortals &dp, const DungeonData &dd, IVec2 from, IVec2 to);
void draw_path(const std::vector<IVec2>& path, float tile_size);
