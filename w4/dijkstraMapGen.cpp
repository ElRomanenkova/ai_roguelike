#include "dijkstraMapGen.h"
#include "ecsTypes.h"
#include "dungeonUtils.h"
#include "math.h"

template<typename Callable>
static void query_dungeon_data(flecs::world &ecs, Callable c)
{
  static auto dungeonDataQuery = ecs.query<const DungeonData>();

  dungeonDataQuery.each(c);
}

template<typename Callable>
static void query_characters_positions(flecs::world &ecs, Callable c)
{
  static auto characterPositionQuery = ecs.query<const Position, const Team>();

  characterPositionQuery.each(c);
}

constexpr float invalid_tile_value = 1e5f;

static void init_tiles(std::vector<float> &map, const DungeonData &dd)
{
  map.resize(dd.width * dd.height);
  for (float &v : map)
    v = invalid_tile_value;
}

// scan version, could be implemented as Dijkstra version as well
static void process_dmap(std::vector<float> &map, const DungeonData &dd)
{
  bool done = false;
  auto getMapAt = [&](size_t x, size_t y, float def)
  {
    if (x < dd.width && y < dd.width && dd.tiles[y * dd.width + x] == dungeon::floor)
      return map[y * dd.width + x];
    return def;
  };
  auto getMinNei = [&](size_t x, size_t y)
  {
    float val = map[y * dd.width + x];
    val = std::min(val, getMapAt(x - 1, y + 0, val));
    val = std::min(val, getMapAt(x + 1, y + 0, val));
    val = std::min(val, getMapAt(x + 0, y - 1, val));
    val = std::min(val, getMapAt(x + 0, y + 1, val));
    return val;
  };
  while (!done)
  {
    done = true;
    for (size_t y = 0; y < dd.height; ++y)
      for (size_t x = 0; x < dd.width; ++x)
      {
        const size_t i = y * dd.width + x;
        if (dd.tiles[i] != dungeon::floor)
          continue;
        const float myVal = getMapAt(x, y, invalid_tile_value);
        const float minVal = getMinNei(x, y);
        if (minVal < myVal - 1.f)
        {
          map[i] = minVal + 1.f;
          done = false;
        }
      }
  }
}

void dmaps::gen_player_approach_map(flecs::world &ecs, std::vector<float> &map)
{
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    query_characters_positions(ecs, [&](const Position &pos, const Team &t)
    {
      if (t.team == 0) // player team hardcode
        map[pos.y * dd.width + pos.x] = 0.f;
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_range_approach_map(flecs::world &ecs, std::vector<float> &map, float range)
{
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    query_characters_positions(ecs, [&](const Position &pos, const Team &t)
    {
      if (t.team != 0)
        return;

      for (int dy = - range; dy <= range; dy++)
        for (int dx = - range; dx <= range; dx++)
        {
          int x = pos.x + dx;
          int y = pos.y + dy;
          if (x >= 0 && x < int(dd.width) && y >= 0 && y < int(dd.height) && dd.tiles[y * dd.width + x] == dungeon::floor)
          {
            bool isVisible = true;
            Position curPos{pos};
            Position destPos{x, y};

            while (destPos != curPos)
            {
              Position dpos = destPos - curPos;
              if (abs(dpos.x) > abs(dpos.y))
                curPos.x += dpos.x > 0 ? 1 : -1;
              else
                curPos.y += dpos.y > 0 ? 1 : -1;

              if (dd.tiles[curPos.y * dd.width + curPos.x] == dungeon::wall)
              {
                isVisible = false;
                break;
              }
            }

            if (isVisible && (static_cast<float>(abs(x - pos.x) + abs(y - pos.y))) <= range)
              map[y * dd.width + x] = 0;
          }
        }
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_player_flee_map(flecs::world &ecs, std::vector<float> &map)
{
  gen_player_approach_map(ecs, map);
  for (float &v : map)
    if (v < invalid_tile_value)
      v *= -1.2f;
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    process_dmap(map, dd);
  });
}

void dmaps::gen_hive_pack_map(flecs::world &ecs, std::vector<float> &map)
{
  static auto hiveQuery = ecs.query<const Position, const Hive>();
  query_dungeon_data(ecs, [&](const DungeonData &dd)
  {
    init_tiles(map, dd);
    hiveQuery.each([&](const Position &pos, const Hive &)
    {
      map[pos.y * dd.width + pos.x] = 0.f;
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_exploration_map(flecs::world& ecs, std::vector<float>& map)
{
  auto static query = ecs.query<const Position, ExplorationData>();
  query_dungeon_data(ecs, [&](const DungeonData& dd)
  {
    init_tiles(map, dd);
    query.each([&](const Position& pos, ExplorationData& ed) {
      float minDist = 1e5f;
      Position minPos{pos};

      for(size_t y = 0; y < dd.height; ++y)
        for (size_t x = 0; x < dd.width; ++x)
        {
          if (dd.tiles[y * dd.width + x] != dungeon::floor)
            continue;
          float range = dist(Position{ static_cast<int>(x), static_cast<int>(y) }, pos);
//          if (!ed.isExplored[y * dd.width + x] && range <= ed.range) {
//            ed.isExplored[y * dd.width + x] = true;
//          }
          if (!ed.isExplored[y * dd.width + x] && range < minDist)
          {
            minPos = Position{ static_cast<int>(x), static_cast<int>(y) };
            minDist = range;
          }
        }
      map[minPos.y * dd.width + minPos.x] = 0;
    });
    process_dmap(map, dd);
  });
}

void dmaps::gen_ally_map(flecs::world& ecs, std::vector<float>& map, const flecs::entity& e, float crit_hp)
{
  static auto allyQuery = ecs.query<const Position, const Team>();
  query_dungeon_data(ecs, [&](const DungeonData& dd)
  {
    init_tiles(map, dd);
    e.get([&](const Team& eteam, const Hitpoints& hp)
    {
      allyQuery.each([&](flecs::entity ae, const Position& pos, const Team& team)
      {
       if (e != ae && team.team == eteam.team && hp.hitpoints < crit_hp)
         map[pos.y * dd.width + pos.x] = 0.0f;
      });
    });
    process_dmap(map, dd);
  });
}