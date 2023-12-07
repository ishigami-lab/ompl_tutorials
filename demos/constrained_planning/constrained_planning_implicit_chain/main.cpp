#include <iostream>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// 制約条件を規定するクラス。
class ChainConstraint : public ob::Constraint
{
private:
  class Wall
  {
    public:
    Wall(double offset, double thickness, double width, double joint_radius, unsigned int type)
    : offset_(offset), thickness_(thickness + joint_radius), width_(width + joint_radius), type_(type)
    {
    }

    bool within(double x) const
    {
      return !(x < (offset_ - thickness_) || x > (offset_ + thickness_));
    }

    bool checkJoint(const Eigen::Ref<const Eigen::VectorXd> &v) const
    {
      double x = v[0], y = v[1], z = v[2];

      if (!within(x))
          return true;

      if (z <= width_)
      {
        switch (type_)
        {
          case 0:
            if (y < 0)
              return true;
            break;

          case 1:
            if (y > 0)
              return true;
            break;
        }
      }

      return false;
    }

  private:
    const double offset_;
    const double thickness_;
    const double width_;
    const unsigned int type_;
  };

  const double WALL_WIDTH = 0.5;
  const double JOINT_RADIUS = 0.2;
  const double LINK_LENGTH = 1.0;

public:
  // ChainConstraintは各リンクの長さが固定、すなわちジョイント同士の距離が一定であるというR^3制約を定義している。
  // なお、追加で制約（Extra）を与えることもできる。追加制約の内容は以下の通り。
  // 1 - End-effector is constrained to be on the surface of a sphere of　radius links - 2
  // 2 - The (links - 5)th and (links - 4)th ball have the same z-value
  // 3 - The (links - 4)th and (links - 3)th ball have the same x-value
  // 4 - The (links - 3)th and (links - 2)th ball have the same z-value
  ChainConstraint(unsigned int links, unsigned int obstacles = 0, unsigned int extra = 1)
  : ob::Constraint(3 * links, links + extra)
  , links_(links)
  , length_(LINK_LENGTH)
  , width_(WALL_WIDTH)
  , radius_(links - 2)
  , jointRadius_(JOINT_RADIUS)
  , obstacles_(obstacles)
  , extra_(extra)
  {
    double step = 2 * radius_ / (double)(obstacles_ + 1);
    double current = -radius_ + step;

    for (unsigned int i = 0; i < obstacles_; i++, current += step)
      walls_.emplace_back(current, radius_ / 8, WALL_WIDTH, JOINT_RADIUS, i % 2);
  }

  // 制約関数f(x)=0の定義。
  void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    Eigen::VectorXd joint1 = Eigen::VectorXd::Zero(3);
    for (unsigned int i = 0; i < links_; i++)
    {
      auto &&joint2 = x.segment(3 * i, 3);
      out[i] = (joint1 - joint2).norm() - length_;
      joint1 = joint2;
    }

    if (extra_ >= 1)
      out[links_] = x.tail(3).norm() - radius_;

    const unsigned int o = links_ - 5;

    if (extra_ >= 2)
      out[links_ + 1] = x[(o + 0) * 3 + 2] - x[(o + 1) * 3 + 2];
    if (extra_ >= 3)
      out[links_ + 2] = x[(o + 1) * 3 + 0] - x[(o + 2) * 3 + 0];
    if (extra_ >= 4)
      out[links_ + 3] = x[(o + 2) * 3 + 2] - x[(o + 3) * 3 + 2];
  }

  // 制約関数f(x)のヤコビアンの定義（オプション）。ここでヤコビアンを定義しなかった場合は、数値解が自動的に計算される。
  // ただし数値解の導出は計算コストが高いので、できる限り解析解をここで定義しておくことが望ましい。
  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
  {
    out.setZero();

    Eigen::VectorXd plus(3 * (links_ + 1));
    plus.head(3 * links_) = x.segment(0, 3 * links_);
    plus.tail(3) = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd minus(3 * (links_ + 1));
    minus.head(3) = Eigen::VectorXd::Zero(3);
    minus.tail(3 * links_) = x.segment(0, 3 * links_);

    auto &&diagonal = plus - minus;

    for (unsigned int i = 0; i < links_; i++)
      out.row(i).segment(3 * i + 0, 3) = diagonal.segment(3 * i, 3).normalized();

    out.block(1, 0, links_ - 1, 3 * links_ - 3) -= out.block(1, 3, links_ - 1, 3 * links_ - 3);

    if (extra_ >= 1)
      out.row(links_).tail(3) = -diagonal.tail(3).normalized().transpose();

    const unsigned int o = links_ - 5;

    if (extra_ >= 2)
    {
      out(links_ + 1, (o + 0) * 3 + 2) = 1;
      out(links_ + 1, (o + 1) * 3 + 2) = -1;
    }
    if (extra_ >= 3)
    {
      out(links_ + 2, (o + 1) * 3 + 0) = 1;
      out(links_ + 2, (o + 2) * 3 + 0) = -1;
    }
    if (extra_ >= 4)
    {
      out(links_ + 3, (o + 2) * 3 + 2) = 1;
      out(links_ + 3, (o + 3) * 3 + 2) = -1;
    }
  }

  bool isValid(const ob::State *state)
  {
    auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

    for (unsigned int i = 0; i < links_; i++)
    {
      auto &&joint = x.segment(3 * i, 3);
      if (joint[2] < 0)
        return false;

      if (joint.norm() >= (radius_ - jointRadius_))
        for (auto wall : walls_)
          if (!wall.checkJoint(joint))
            return false;
    }

    for (unsigned int i = 0; i < links_ - 1; i++)
    {
      auto &&joint1 = x.segment(3 * i, 3);
      if (joint1.cwiseAbs().maxCoeff() < jointRadius_)
        return false;

      for (unsigned int j = i + 1; j < links_; j++)
      {
        auto &&joint2 = x.segment(3 * j, 3);
        if ((joint1 - joint2).cwiseAbs().maxCoeff() < jointRadius_)
          return false;
      }
    }

    return true;
  }

  ob::StateSpacePtr createSpace() const
  {
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3 * links_);
    ob::RealVectorBounds bounds(3 * links_);

    for (int i = 0; i < (int)links_; ++i)
    {
      bounds.setLow(3 * i + 0, -i - 1);
      bounds.setHigh(3 * i + 0, i + 1);

      bounds.setLow(3 * i + 1, -i - 1);
      bounds.setHigh(3 * i + 1, i + 1);

      bounds.setLow(3 * i + 2, -i - 1);
      bounds.setHigh(3 * i + 2, i + 1);
    }

    rvss->setBounds(bounds);
    return rvss;
  }

  void setStartAndGoalStates(Eigen::VectorXd &start, Eigen::VectorXd &goal) const
  {
    start = Eigen::VectorXd(3 * links_);
    goal = Eigen::VectorXd(3 * links_);

    int i = 0;
    for (; i < (int)links_ - 3; ++i)
    {
      start[3 * i] = i + 1;
      start[3 * i + 1] = 0;
      start[3 * i + 2] = 0;

      goal[3 * i] = -(i + 1);
      goal[3 * i + 1] = 0;
      goal[3 * i + 2] = 0;
    }

    start[3 * i] = i;
    start[3 * i + 1] = -1;
    start[3 * i + 2] = 0;

    goal[3 * i] = -i;
    goal[3 * i + 1] = 1;
    goal[3 * i + 2] = 0;

    i++;

    start[3 * i] = i;
    start[3 * i + 1] = -1;
    start[3 * i + 2] = 0;

    goal[3 * i] = -i;
    goal[3 * i + 1] = 1;
    goal[3 * i + 2] = 0;

    i++;

    start[3 * i] = i - 1;
    start[3 * i + 1] = 0;
    start[3 * i + 2] = 0;

    goal[3 * i] = -(i - 1);
    goal[3 * i + 1] = 0;
    goal[3 * i + 2] = 0;
  }

  ob::ProjectionEvaluatorPtr getProjection(ob::StateSpacePtr space) const
  {
    // 制約付き状態空間の射影を行うクラス
    class ChainProjection : public ob::ProjectionEvaluator
    {
    public:
      ChainProjection(const ob::StateSpacePtr &space, unsigned int links, double radius)
        : ob::ProjectionEvaluator(space), links_(links), radius_(radius)
      {
      }

      unsigned int getDimension() const override
      {
        return 2;
      }

      void defaultCellSizes() override
      {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
      }

      void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
      {
        auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();
        const unsigned int s = 3 * (links_ - 1);

        projection(0) = atan2(x[s + 1], x[s]);
        projection(1) = acos(x[s + 2] / radius_);
      }

    private:
      const unsigned int links_;  // Number of chain links.
      double radius_;             // Radius of sphere end-effector lies on (for extra = 1)
    };

    return std::make_shared<ChainProjection>(space, links_, radius_);
  }

  void dump(std::ofstream &file) const
  {
      file << links_ << std::endl;
      file << obstacles_ << std::endl;
      file << extra_ << std::endl;
      file << jointRadius_ << std::endl;
      file << length_ << std::endl;
      file << radius_ << std::endl;
      file << width_ << std::endl;
  }

private:
  const unsigned int links_;      // Number of chain links.
  const double length_;           // Length of one link.
  const double width_;            // Width of obstacle wall.
  const double radius_;           // Radius of the sphere that the end effector is constrained to.
  const double jointRadius_;      // Size of joints
  const unsigned int obstacles_;  // Number of obstacles on sphere surface
  const unsigned int extra_;      // Number of extra constraints
  std::vector<Wall> walls_;       // Obstacles
};

bool chainPlanning(unsigned int links, unsigned int obstacles, unsigned int extra)
{
  // 制約クラスのshared_ptrを定義
  auto constraint = std::make_shared<ChainConstraint>(links, obstacles, extra);

  // Planningの境界条件を与えた状態空間を定義
  auto rvss = constraint->createSpace();

  // 制約つき問題クラスのインスタンスを生成。
  OMPL_INFORM("Using Projection-Based State Space!");
  auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);
  auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
  css->setup();

  // Simple Setupクラスポインタを定義
  auto ss = std::make_shared<og::SimpleSetup>(csi);

  // Projection（射影）関数の定義
  css->registerProjection("chain", constraint->getProjection(css));

  // Start位置ベクトルとGoal位置ベクトルを設定
  Eigen::VectorXd sv, gv;
  constraint->setStartAndGoalStates(sv, gv);

  ob::ScopedState<> start(css);
  ob::ScopedState<> goal(css);
  start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
  ss->setStartAndGoalStates(start, goal);

  // validity checkerの設定。障害物判定関数を入れる
  ss->setStateValidityChecker(std::bind(&ChainConstraint::isValid, constraint, std::placeholders::_1));

  // Plannerの設定。
  auto pp = std::make_shared<og::RRTConnect>(csi);
  ss->setPlanner(pp);

  ss->setup();

  // 問題を解く。（Timeout時間: 5.0[sec]）
  ob::PlannerStatus stat = ss->solve(5.);
  if (stat)
  { 
    // 解（Path）の取得と、解の妥当性チェック。
    auto path = ss->getSolutionPath();
    if (!path.check())
      OMPL_WARN("Path fails check!");

    if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
      OMPL_WARN("Solution is approximate.");

    // 解（Path）の単純化処理と、処理後の解の妥当性チェック。
    OMPL_INFORM("Simplifying solution...");
    ss->simplifySolution(5.);

    auto simplePath = ss->getSolutionPath();
    OMPL_INFORM("Simplified Path Length: %.3f -> %.3f", path.length(), simplePath.length());

    if (!simplePath.check())
      OMPL_WARN("Simplified path fails check!");

    // 解（Path）の線形補間処理と、処理後の解の妥当性チェック。
    OMPL_INFORM("Interpolating path...");
    path.interpolate();

    if (!path.check())
      OMPL_WARN("Interpolated simplified path fails check!");

    OMPL_INFORM("Interpolating simplified path...");
    simplePath.interpolate();

    if (!simplePath.check())
      OMPL_WARN("Interpolated simplified path fails check!");

    // 最終結果（後処理なし）を出力
    OMPL_INFORM("Dumping path to `path.txt`.");
    std::ofstream pathfile("../output/path.txt");
    path.printAsMatrix(pathfile);
    pathfile.close();

    // 最終結果（後処理あり）を出力
    OMPL_INFORM("Dumping simplified path to `simplepath.txt`.");
    std::ofstream simplepathfile("../output/simplepath.txt");
    simplePath.printAsMatrix(simplepathfile);
    simplepathfile.close();
  }
  else
    OMPL_WARN("No solution found!");

  // 問題設定の情報を出力。
  // 出力順：リンク数(links)、障害物の数(obstacles)、追加制約の数(extra)、
  // ジョイント半径(jointRadius)、長さ(length)、半径(radius)、幅(width)
  OMPL_INFORM("Dumping problem information to `chain_info.txt`.");
  std::ofstream infofile("../output/chain_info.txt");
  dynamic_cast<ChainConstraint *>(constraint.get())->dump(infofile);
  infofile.close();

  return stat;
}

int main(int argc, char **argv)
{
  unsigned int links = 5;
  unsigned int obstacles = 0;
  unsigned int extra = 1;

  chainPlanning(links, obstacles, extra);

  return 0;
}