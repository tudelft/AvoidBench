#include "astar_path_finding.h"

namespace avoidmetrics {

AStarFindPath::AStarFindPath()
{

}

int AStarFindPath::GetDistance(AStarNode* startNode, AStarNode* endNode){
    int x = std::abs(startNode->node_idx(0) - endNode->node_idx(0));
    int y = std::abs(startNode->node_idx(1) - endNode->node_idx(1));
    int z = std::abs(startNode->node_idx(2) - endNode->node_idx(2));
    return x + y + z;
}

AStarNode* AStarFindPath::getItem (const Eigen::Vector3d pos)
{
  AStarNode* node = new AStarNode(env_ptr_->pcl2Node(pos), env_ptr_->bounding);
  node->isWall = env_ptr_->checkOccupied(pos);
  return node;
}

std::vector<Eigen::Vector3d> AStarFindPath::getBestPath()
{
  return best_path;
}

std::vector<AStarNode*> AStarFindPath::GetAroundNodes(AStarNode* node)
{
  std::vector<AStarNode*> retGroup;
  for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        for (int k = -1; k <=1; k++) {
          if (i == 0 && j == 0 && k == 0) {
              continue;
          }
          Eigen::Vector3i nd;
          nd(0) = node->node_idx.x() + i;
          nd(1) = node->node_idx.y() + j;
          nd(2) = node->node_idx.z() + k;

          if (env_ptr_->checkIdxBounding(nd)) {
              AStarNode* new_nd = new AStarNode(nd, env_ptr_->bounding);
              new_nd->isWall = env_ptr_->checkOccupied(nd);
              new_nd->costG = INFINITY;
              retGroup.push_back(new_nd);
          }
        }
      }
  }
  return retGroup;
}

bool AStarFindPath::toFindPath(const Eigen::Vector3d startPos, const Eigen::Vector3d endPos) {
  best_path.clear();
  bool end_is_wall;
  AStarNode* start = getItem (startPos);
  AStarNode* end = getItem (endPos);
  end_is_wall = end->isWall;

  std::vector<AStarNode*> end_around = GetAroundNodes(end);

  std::vector<AStarNode*> openList;
  std::vector<AStarNode*> closeList;
  std::map<uint, AStarNode*> openMap;
  std::map<uint, AStarNode*> closeMap;
  start->if_in_openList = true;
  start->costG = 0;
  start->costH = GetDistance(start, end);
  openList.push_back (start);
  openMap.insert(std::make_pair(start->id, start));
  //judge from first node
  while (openList.size() > 0) {
      AStarNode* curNode = openList.front();
      int iter=0;
      int idx=0;
      for (auto node: openList) {
        if (node->CostF() < curNode->CostF()) {
            curNode = node;
            idx = iter;
        }
        iter++;
      }
      int size1 = openMap.size();
      openList.erase(openList.begin() + idx);
      
      for(auto iter_map = openMap.begin(); iter_map != openMap.end();) {
        if (iter_map->first == curNode->id)
          iter_map = openMap.erase(iter_map);
        else
          ++iter_map;
      }

      closeList.push_back(curNode);
      closeMap.insert(std::make_pair(curNode->id, curNode));
      
      if(!end_is_wall)
      {
        if (*curNode == *end) {
        std::cout<<"find astar path"<<std::endl;
        break;
      }
      } else
      {
        if (curNode->costH<=2)
        {
          std::cout<<"find astar path"<<std::endl;
          break;
        }
      }

      std::vector<AStarNode*> nodeItemGroup = GetAroundNodes(curNode);
      if(nodeItemGroup.empty()) continue;
      for(auto nodeCell: nodeItemGroup)
      {
        if (nodeCell->isWall) {
          continue;
        }

        auto it_close = closeMap.find(nodeCell->id);
        if(it_close == closeMap.end() && (it_close->first!=nodeCell->id)) {
        }
        else {
          continue;
        } 
        
        //calculate G H F
        int newCostg = curNode->costG + GetDistance (curNode, nodeCell);

        auto it_open = openMap.find(nodeCell->id);
        if(it_open == openMap.end() && (it_open->first != nodeCell->id)) {
          nodeCell->costG = newCostg;
          nodeCell->costH = GetDistance(nodeCell, end);
          nodeCell->parentNode = curNode;
          openList.push_back(nodeCell);
          openMap.insert(std::make_pair(nodeCell->id, nodeCell));
        }
        else {
          if(newCostg < nodeCell->costG) 
          {
            nodeCell->parentNode = curNode;
            nodeCell->costG = newCostg;
          }
        }
      }
    if(closeList.size() >= (env_ptr_->bounding.zWidth * env_ptr_->bounding.zHeight * env_ptr_->bounding.zLength))
      return false;
  }

  std::vector<Eigen::Vector3d> r_best_path;
  AStarNode* path_node = closeList.back();
  while(!(*(path_node) == *start))
  {
    Eigen::Vector3i ndi = path_node->node_idx;
    r_best_path.push_back(env_ptr_->Node2pcl(ndi));
    path_node = path_node->parentNode;
  }
  r_best_path.push_back(env_ptr_->Node2pcl(path_node->node_idx));

  std::vector<Eigen::Vector3d>::reverse_iterator riter;
  for (riter=r_best_path.rbegin(); riter!=r_best_path.rend(); riter++)
  {
    best_path.push_back(*riter);
  }
  return true;

}


}

