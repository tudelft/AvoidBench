#include "environment.h"

namespace avoidmetrics
{

class AStarNode {

public:
    bool isWall;
    uint id;

    Eigen::Vector3i node_idx;
    AStarNode *parentNode;

    int costG;
    int costH;

    bool if_in_openList;
    bool if_in_closeList;

    int CostF ()
    {
      return 2 * costG + 3 * costH;
    }

    AStarNode(Eigen::Vector3i node, Bounding bounding)
    {
      this->id = node.z() * (bounding.zWidth * bounding.zLength) + node.y() * bounding.zWidth + node.x();
      this->node_idx = node;
      this->costG = 0;
      this->costH = 0;
      this->isWall = false;
      this->if_in_closeList = false;
      this->if_in_openList = false;
    }

    bool operator  == (const AStarNode& a)const {
      return (this->node_idx(0) == a.node_idx(0)) && 
        (this->node_idx(1) == a.node_idx(1)) && (this->node_idx(2) == a.node_idx(2));
    }

};

class AStarFindPath {
public:
  AStarFindPath();
  ~AStarFindPath() {};
  void setMap(std::shared_ptr<Environment> env_ptr) { env_ptr_ = env_ptr; }
  bool toFindPath(const Eigen::Vector3d startPos, const Eigen::Vector3d endPos);
  std::vector<Eigen::Vector3d> getBestPath();

private:

    std::vector<Eigen::Vector3d> bestPath;
    std::vector<AStarNode> bestNodePath;
    std::shared_ptr<Environment> env_ptr_;
    std::vector<Eigen::Vector3d> best_path;

    int GetDistance(AStarNode* startNode, AStarNode* endNode);
    AStarNode* getItem (const Eigen::Vector3d pos);
    std::vector<AStarNode*> GetAroundNodes(AStarNode* node);


};

}
