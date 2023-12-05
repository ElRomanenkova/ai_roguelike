#include "pathfinder.h"
#include "dungeonUtils.h"
#include "math.h"
#include <algorithm>

float heuristic(IVec2 lhs, IVec2 rhs)
{
  return sqrtf(sqr(float(lhs.x - rhs.x)) + sqr(float(lhs.y - rhs.y)));
};

float portal_heuristic(const PathPortal& lhs, const PathPortal& rhs)
{
  IVec2 lhs_coord{static_cast<int>((lhs.startX + lhs.endX) / 2), static_cast<int>((lhs.startY + lhs.endY) / 2)};
  IVec2 rhs_coord{static_cast<int>((rhs.startX + rhs.endX) / 2), static_cast<int>((rhs.startY + rhs.endY) / 2)};

  return sqrtf(sqr(float(lhs_coord.x - rhs_coord.x)) + sqr(float(lhs_coord.y - rhs_coord.y)));
}

template<typename T>
static size_t coord_to_idx(T x, T y, size_t w)
{
  return size_t(y) * w + size_t(x);
}

static size_t portal_to_idx(const PathPortal& p, size_t w)
{
  return coord_to_idx(static_cast<int>((p.startX + p.endX) / 2), static_cast<int>((p.startY + p.endY) / 2), w);
}

static std::vector<IVec2> reconstruct_path(std::vector<IVec2> prev, IVec2 to, size_t width)
{
  IVec2 curPos = to;
  std::vector<IVec2> res = {curPos};
  while (prev[coord_to_idx(curPos.x, curPos.y, width)] != IVec2{-1, -1})
  {
    curPos = prev[coord_to_idx(curPos.x, curPos.y, width)];
    res.insert(res.begin(), curPos);
  }
  return res;
}

static std::vector<IVec2> find_path_a_star(const DungeonData &dd, IVec2 from, IVec2 to,
                                           IVec2 lim_min, IVec2 lim_max)
{
  if (from.x < 0 || from.y < 0 || from.x >= int(dd.width) || from.y >= int(dd.height))
    return std::vector<IVec2>();
  size_t inpSize = dd.width * dd.height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<float> f(inpSize, std::numeric_limits<float>::max());
  std::vector<IVec2> prev(inpSize, {-1,-1});

  auto getG = [&](IVec2 p) -> float { return g[coord_to_idx(p.x, p.y, dd.width)]; };
  auto getF = [&](IVec2 p) -> float { return f[coord_to_idx(p.x, p.y, dd.width)]; };

  g[coord_to_idx(from.x, from.y, dd.width)] = 0;
  f[coord_to_idx(from.x, from.y, dd.width)] = heuristic(from, to);

  std::vector<IVec2> openList = {from};
  std::vector<IVec2> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
      return reconstruct_path(prev, to, dd.width);
    IVec2 curPos = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPos) != closedList.end())
      continue;
    size_t idx = coord_to_idx(curPos.x, curPos.y, dd.width);
    closedList.emplace_back(curPos);
    auto checkNeighbour = [&](IVec2 p)
    {
      // out of bounds
      if (p.x < lim_min.x || p.y < lim_min.y || p.x >= lim_max.x || p.y >= lim_max.y)
        return;
      size_t idx = coord_to_idx(p.x, p.y, dd.width);
      // not empty
      if (dd.tiles[idx] == dungeon::wall)
        return;
      float edgeWeight = 1.f;
      float gScore = getG(curPos) + 1.f * edgeWeight; // we're exactly 1 unit away
      if (gScore < getG(p))
      {
        prev[idx] = curPos;
        g[idx] = gScore;
        f[idx] = gScore + heuristic(p, to);
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };
    checkNeighbour({curPos.x + 1, curPos.y + 0});
    checkNeighbour({curPos.x - 1, curPos.y + 0});
    checkNeighbour({curPos.x + 0, curPos.y + 1});
    checkNeighbour({curPos.x + 0, curPos.y - 1});
  }
  // empty path
  return std::vector<IVec2>();
}

static std::vector<IVec2> find_path_a_star_portal(const DungeonData &dd, const DungeonPortals &dp,
                                                  PathPortal from, PathPortal to)
{
  size_t inpSize = dd.width * dd.height;

  std::vector<float> g(inpSize, std::numeric_limits<float>::max());
  std::vector<IVec2> prev(inpSize, {-1, -1});

  auto getG = [&](const PathPortal& p) -> float { return g[portal_to_idx(p, dd.width)]; };
  auto getF = [&](const PathPortal& p) -> float { return getG(p) + portal_heuristic(p, to); };

  g[coord_to_idx(static_cast<int>((from.startX + from.endX) / 2), static_cast<int>((from.startY + from.endY) / 2), dd.width)] = 0;

  std::vector<PathPortal> openList = {from};
  std::vector<PathPortal> closedList;

  while (!openList.empty())
  {
    size_t bestIdx = 0;
    float bestScore = getF(openList[0]);
    for (size_t i = 1; i < openList.size(); ++i)
    {
      float score = getF(openList[i]);
      if (score < bestScore)
      {
        bestIdx = i;
        bestScore = score;
      }
    }
    if (openList[bestIdx] == to)
    {
      IVec2 to_coord = {static_cast<int>((to.startX + to.endX) / 2), static_cast<int>((to.startY + to.endY) / 2)};
      return reconstruct_path(prev, to_coord, dd.width);
    }
    const auto curPortal = openList[bestIdx];
    openList.erase(openList.begin() + bestIdx);
    if (std::find(closedList.begin(), closedList.end(), curPortal) != closedList.end())
      continue;
    closedList.emplace_back(curPortal);

    auto checkNeighbour = [&](const PathPortal& p, float edgeWeight) {
      size_t idx = portal_to_idx(p, dd.width);
      float gScore = getG(curPortal) + 1.f * edgeWeight;
      if (gScore < getG(p)) {
        prev[idx] = {static_cast<int>((curPortal.startX + curPortal.endX) / 2), static_cast<int>((curPortal.startY + curPortal.endY) / 2)};
        g[idx] = gScore;
      }
      bool found = std::find(openList.begin(), openList.end(), p) != openList.end();
      if (!found)
        openList.emplace_back(p);
    };

    for (auto c : curPortal.conns)
      checkNeighbour(dp.portals[c.connIdx], c.score);
  }
  return {};
}

static std::vector<IVec2> get_shortest_path_portal(const DungeonData &dd, const DungeonPortals &dp,
                                            IVec2 from, IVec2 tile, PathPortal &p)
{
  auto shortestPath = std::vector<IVec2>();
  const auto width = dd.width / dp.tileSplit;

  for (const auto portal_ind : dp.tilePortalsIndices[tile.y * width + tile.x])
  {
    IVec2 limMin{static_cast<int>(tile.x * dp.tileSplit), static_cast<int>(tile.y * dp.tileSplit)};
    IVec2 limMax{static_cast<int>((tile.x + 1) * dp.tileSplit), static_cast<int>((tile.y + 1) * dp.tileSplit)};

    bool noPath = false;
    const auto portal = dp.portals[portal_ind];
    auto path = std::vector<IVec2>();
    for (size_t toY = std::max(portal.startY, size_t(limMin.y));
         toY <= std::min(portal.endY, size_t(limMax.y - 1)) && !noPath; ++toY)
    {
      for (size_t toX = std::max(portal.startX, size_t(limMin.x));
           toX <= std::min(portal.endX, size_t(limMax.x - 1)) && !noPath; ++toX)
      {
        IVec2 to{int(toX), int(toY)};
        std::vector<IVec2> curPath = find_path_a_star(dd, from, to, limMin, limMax);
        if (curPath.empty() && from != to)
        {
          noPath = true;
          break;
        }
        if (path.empty() || curPath.size() < path.size())
          path = std::move(curPath);
      }
    }

    if (shortestPath.empty() || path.size() < shortestPath.size())
    {
      shortestPath = std::move(path);
      p = dp.portals[portal_ind];
    }
  }
  return shortestPath;
}

std::vector<IVec2> find_hierarchical_path(const DungeonPortals &dp, const DungeonData &dd,
                                          IVec2 from, IVec2 to)
{
  const IVec2 fromTile{static_cast<int>(from.x / dp.tileSplit), static_cast<int>(from.y / dp.tileSplit)};
  const IVec2 toTile{static_cast<int>(to.x / dp.tileSplit), static_cast<int>(to.y / dp.tileSplit)};

  if (fromTile == toTile)
  {
    IVec2 limMin{static_cast<int>(fromTile.x * dp.tileSplit), static_cast<int>(fromTile.y * dp.tileSplit)};
    IVec2 limMax{static_cast<int>((fromTile.x + 1) * dp.tileSplit), static_cast<int>((fromTile.y + 1) * dp.tileSplit)};
    return find_path_a_star(dd, from, to, limMin, limMax);
  }

  auto fromPortal = PathPortal();
  auto toPortal = PathPortal();
  const auto fromStartToClosest = get_shortest_path_portal(dd, dp, from, fromTile, fromPortal);
  const auto fromTargetToClosest = get_shortest_path_portal(dd, dp, to, toTile, toPortal);
  const auto fromStartToTarget = find_path_a_star_portal(dd, dp, fromPortal, toPortal);

  auto resPath = std::vector<IVec2>();
  resPath.reserve(fromStartToClosest.size() + fromStartToTarget.size() + fromTargetToClosest.size());
  resPath.insert(resPath.end(), fromStartToClosest.begin(), fromStartToClosest.end());
  resPath.insert(resPath.end(), fromStartToTarget.begin(), fromStartToTarget.end());
  resPath.insert(resPath.end(), fromTargetToClosest.begin(), fromTargetToClosest.end());
  return resPath;
}

void draw_path(const std::vector<IVec2>& path, float tile_size)
{
  if (path.empty())
    return;

  auto tileColor = BLUE;
  auto pathColor = DARKBLUE;

  DrawRectangleRec({path[0].x * tile_size, path[0].y * tile_size, tile_size, tile_size}, pathColor);

  for (size_t i = 1; i < path.size(); i++)
  {
    DrawRectangleRec({path[i].x * tile_size, path[i].y * tile_size, tile_size, tile_size}, tileColor);
    DrawLineEx(Vector2{(path[i - 1].x ) * tile_size, (path[i - 1].y) * tile_size},
               Vector2{(path[i].x) * tile_size, (path[i].y) * tile_size}, 10.f, pathColor);
  }

  DrawRectangleRec({path[path.size() - 1].x * tile_size, path[path.size() - 1].y * tile_size, tile_size, tile_size}, pathColor);
}

void prebuild_map(flecs::world &ecs)
{
  auto mapQuery = ecs.query<const DungeonData>();

  constexpr size_t splitTiles = 10;
  ecs.defer([&]()
  {
    mapQuery.each([&](flecs::entity e, const DungeonData &dd)
    {
      // go through each super tile
      const size_t width = dd.width / splitTiles;
      const size_t height = dd.height / splitTiles;

      auto check_border = [&](size_t xx, size_t yy,
                              size_t dir_x, size_t dir_y,
                              int offs_x, int offs_y,
                              std::vector<PathPortal> &portals)
      {
        int spanFrom = -1;
        int spanTo = -1;
        for (size_t i = 0; i < splitTiles; ++i)
        {
          size_t x = xx * splitTiles + i * dir_x;
          size_t y = yy * splitTiles + i * dir_y;
          size_t nx = x + offs_x;
          size_t ny = y + offs_y;
          if (dd.tiles[y * dd.width + x] != dungeon::wall &&
              dd.tiles[ny * dd.width + nx] != dungeon::wall)
          {
            if (spanFrom < 0)
              spanFrom = i;
            spanTo = i;
          }
          else if (spanFrom >= 0)
          {
            // write span
            portals.push_back({xx * splitTiles + spanFrom * dir_x + offs_x,
                               yy * splitTiles + spanFrom * dir_y + offs_y,
                               xx * splitTiles + spanTo * dir_x,
                               yy * splitTiles + spanTo * dir_y});
            spanFrom = -1;
          }
        }
        if (spanFrom >= 0)
        {
          portals.push_back({xx * splitTiles + spanFrom * dir_x + offs_x,
                             yy * splitTiles + spanFrom * dir_y + offs_y,
                             xx * splitTiles + spanTo * dir_x,
                             yy * splitTiles + spanTo * dir_y});
        }
      };

      std::vector<PathPortal> portals;
      std::vector<std::vector<size_t>> tilePortalsIndices;

      auto push_portals = [&](size_t x, size_t y,
                              int offs_x, int offs_y,
                              const std::vector<PathPortal> &new_portals)
      {
        for (const PathPortal &portal : new_portals)
        {
          size_t idx = portals.size();
          portals.push_back(portal);
          tilePortalsIndices[y * width + x].push_back(idx);
          tilePortalsIndices[(y + offs_y) * width + x + offs_x].push_back(idx);
        }
      };
      for (size_t y = 0; y < height; ++y)
        for (size_t x = 0; x < width; ++x)
        {
          tilePortalsIndices.push_back(std::vector<size_t>{});
          // check top
          if (y > 0)
          {
            std::vector<PathPortal> topPortals;
            check_border(x, y, 1, 0, 0, -1, topPortals);
            push_portals(x, y, 0, -1, topPortals);
          }
          // left
          if (x > 0)
          {
            std::vector<PathPortal> leftPortals;
            check_border(x, y, 0, 1, -1, 0, leftPortals);
            push_portals(x, y, -1, 0, leftPortals);
          }
        }
      for (size_t tidx = 0; tidx < tilePortalsIndices.size(); ++tidx)
      {
        const std::vector<size_t> &indices = tilePortalsIndices[tidx];
        size_t x = tidx % width;
        size_t y = tidx / width;
        IVec2 limMin{int((x + 0) * splitTiles), int((y + 0) * splitTiles)};
        IVec2 limMax{int((x + 1) * splitTiles), int((y + 1) * splitTiles)};
        for (size_t i = 0; i < indices.size(); ++i)
        {
          PathPortal &firstPortal = portals[indices[i]];
          for (size_t j = i + 1; j < indices.size(); ++j)
          {
            PathPortal &secondPortal = portals[indices[j]];
            // check path from i to j
            // check each position (to find closest dist) (could be made more optimal)
            bool noPath = false;
            size_t minDist = 0xffffffff;
            for (size_t fromY = std::max(firstPortal.startY, size_t(limMin.y));
                        fromY <= std::min(firstPortal.endY, size_t(limMax.y - 1)) && !noPath; ++fromY)
            {
              for (size_t fromX = std::max(firstPortal.startX, size_t(limMin.x));
                          fromX <= std::min(firstPortal.endX, size_t(limMax.x - 1)) && !noPath; ++fromX)
              {
                for (size_t toY = std::max(secondPortal.startY, size_t(limMin.y));
                            toY <= std::min(secondPortal.endY, size_t(limMax.y - 1)) && !noPath; ++toY)
                {
                  for (size_t toX = std::max(secondPortal.startX, size_t(limMin.x));
                              toX <= std::min(secondPortal.endX, size_t(limMax.x - 1)) && !noPath; ++toX)
                  {
                    IVec2 from{int(fromX), int(fromY)};
                    IVec2 to{int(toX), int(toY)};
                    std::vector<IVec2> path = find_path_a_star(dd, from, to, limMin, limMax);
                    if (path.empty() && from != to)
                    {
                      noPath = true; // if we found that there's no path at all - we can break out
                      break;
                    }
                    minDist = std::min(minDist, path.size());
                  }
                }
              }
            }
            // write pathable data and length
            if (noPath)
              continue;
            firstPortal.conns.push_back({indices[j], float(minDist)});
            secondPortal.conns.push_back({indices[i], float(minDist)});
          }
        }
      }
      e.set(DungeonPortals{splitTiles, portals, tilePortalsIndices});
    });
  });
}

