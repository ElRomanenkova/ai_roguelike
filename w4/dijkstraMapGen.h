#pragma once
#include <vector>
#include <flecs.h>

namespace dmaps
{
  void gen_player_approach_map(flecs::world &ecs, std::vector<float> &map);
  void gen_range_approach_map(flecs::world &ecs, std::vector<float> &map, float range);
  void gen_player_flee_map(flecs::world &ecs, std::vector<float> &map);
  void gen_hive_pack_map(flecs::world &ecs, std::vector<float> &map);
  void gen_exploration_map(flecs::world& ecs, std::vector<float>& map);
  void gen_ally_map(flecs::world& ecs, std::vector<float>& map, const flecs::entity& e, float crit_hp);
};

