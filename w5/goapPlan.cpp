#include "goapPlanner.h"
#include <algorithm>
#include <cfloat>

struct PlanNode
{
  goap::WorldState worldState;
  goap::WorldState prevState;

  float g = 0;
  float h = 0;

  size_t actionId;
};

static float heuristic(const goap::WorldState &from, const goap::WorldState &to)
{
  float cost = 0;
  for (size_t i = 0; i < to.size(); ++i)
    if (to[i] >= 0) // we care about it
      cost += float(abs(to[i] - from[i]));
  return cost;
}

static void reconstruct_plan(PlanNode &goal_node, const std::vector<PlanNode> &closed, std::vector<goap::PlanStep> &plan)
{
  PlanNode &curNode = goal_node;
  while (curNode.actionId != size_t(-1))
  {
    plan.push_back({curNode.actionId, curNode.worldState});
    auto itf = std::find_if(closed.begin(), closed.end(), [&](const PlanNode &n) { return n.worldState == curNode.prevState; });
    curNode = *itf;
  }
  std::reverse(plan.begin(), plan.end());
}

float goap::make_plan(const Planner &planner, const WorldState &from, const WorldState &to, std::vector<PlanStep> &plan)
{
  std::vector<PlanNode> openList = {PlanNode{from, from, 0, heuristic(from, to), size_t(-1)}};
  std::vector<PlanNode> closedList = {};
  while (!openList.empty())
  {
    auto minIt = openList.begin();
    float minF = minIt->g + minIt->h;
    for (auto it = openList.begin(); it != openList.end(); ++it)
      if (it->g + it->h < minF)
      {
        minF = it->g + it->h;
        minIt = it;
      }
    PlanNode cur = *minIt;
    openList.erase(minIt);
    if (heuristic(cur.worldState, to) == 0) // we've reached our goal
    {
      reconstruct_plan(cur, closedList, plan);
      return minF;
    }
    closedList.push_back(cur);
    std::vector<size_t> transitions = find_valid_state_transitions(planner, cur.worldState);
    //const bool firstIter = openList.empty();
    //printf("------------\n");
    for (size_t actId : transitions)
    {
      //printf("valid action: %s\n", planner.actions[actId].name.c_str());
      WorldState st = apply_action(planner, actId, cur.worldState);
      const float score = cur.g + get_action_cost(planner, actId);
      auto openIt = std::find_if(openList.begin(), openList.end(), [&](const PlanNode &n) { return st == n.worldState; });
      auto closeIt = std::find_if(closedList.begin(), closedList.end(), [&](const PlanNode &n) { return st == n.worldState; });
      if (openIt != openList.end() && score < openIt->g)
      {
        openIt->g = score;
        openIt->prevState = cur.worldState;
      }
      if (closeIt != closedList.end() && score < closeIt->g)
      {
        closeIt->g = score;
        closeIt->prevState = cur.worldState;
      }
      if (closeIt == closedList.end() && openIt == openList.end())
        openList.push_back({st, cur.worldState, score, heuristic(st, to), actId});
    }
  }
  return 0.f;
}

static float ida_star_search(const goap::Planner &planner, std::vector<goap::PlanStep> &plan, const float g, const float bound, const goap::WorldState &to)
{
  const goap::PlanStep p = plan.back();
  const float f = g + heuristic(p.worldState, to);
  if (f > bound)
    return f;
  if (heuristic(p.worldState, to) == 0)
    return -f;
  float min = FLT_MAX;
  auto checkNeighbour = [&](size_t actId) -> float
  {
    goap::WorldState st = apply_action(planner, actId, p.worldState);
    if (std::find_if(plan.begin(), plan.end(), [&](const goap::PlanStep &step) { return st == step.worldState; }) != plan.end())
      return 0.f;
    plan.push_back({ actId, st });
    float gScore = g + get_action_cost(planner, actId);
    const float t = ida_star_search(planner, plan, gScore, bound, to);
    if (t < 0.f)
      return t;
    if (t < min)
      min = t;
    plan.pop_back();
    return t;
  };

  std::vector<size_t> transitions = find_valid_state_transitions(planner, p.worldState);
  for (size_t actId : transitions)
  {
    float score = checkNeighbour(actId);
    if (score < 0.f)
      return score;
  }
  return min;
}

float goap::make_plan_ida(const Planner &planner, const WorldState &from, const WorldState &to, std::vector<PlanStep> &plan)
{
  float bound = heuristic(from, to);
  plan = {{size_t(-1), from}};
  while (true)
  {
    const float t = ida_star_search(planner, plan, 0.f, bound, to);
    if (t < 0.f) {
      plan.erase(plan.begin());
      return t;
    }
    if (t == FLT_MAX) {
      plan.erase(plan.begin());
      return {};
    }
    bound = t;
  }
  plan.erase(plan.begin());
  return {};
}

void goap::print_plan(const Planner &planner, const WorldState &init, const std::vector<PlanStep> &plan)
{
  printf("%15s: ", "");
  std::vector<int> dlen;
  for (size_t i = 0; i < planner.wdesc.size(); ++i)
  {
    // print names by searching
    for (auto it : planner.wdesc)
    {
      if (it.second == i)
      {
        printf("|%s|", it.first.c_str());
        dlen.push_back(int(it.first.size()));
        break;
      }
    }
  }
  printf("\n");
  printf("%15s: ", "");
  for (size_t i = 0; i < init.size(); ++i)
    printf("|%*d|", dlen[i], init[i]);
  printf("\n");
  for (const PlanStep &step : plan)
  {
    printf("%15s: ", planner.actions[step.action].name.c_str());
    for (size_t i = 0; i < step.worldState.size(); ++i)
      printf("|%*d|", dlen[i], step.worldState[i]);
    printf("\n");
  }
}

