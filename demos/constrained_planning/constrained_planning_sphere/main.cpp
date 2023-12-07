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
class SphereConstraint : public ob::Constraint
{
public:
  SphereConstraint() : ob::Constraint(3, 1)
  {
  }

  // 制約関数f(x)=0の定義。おそらく等式制約しか与えることはできない。
  // 今回は半径1の球表面上としている。
  void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    out[0] = x.norm() - 1;
  }

  // 制約関数f(x)のヤコビアンの定義（オプション）。ここでヤコビアンを定義しなかった場合は、数値解が自動的に計算される。
  // ただし数値解の導出は計算コストが高いので、できる限り解析解をここで定義しておくことが望ましい。
  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
  {
    out = x.transpose().normalized();
  }
};

// 制約付き状態空間の射影を行うクラス
class SphereProjection : public ob::ProjectionEvaluator
{
public:
  SphereProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
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
    projection(0) = atan2(x[1], x[0]);
    projection(1) = acos(x[2]);
  }
};

// 障害物（通過禁止領域）を状態空間上に定義。
bool obstacles(const ob::State *state)
{
  auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

  if (-0.80 < x[2] && x[2] < -0.6)
  {
    if (-0.05 < x[1] && x[1] < 0.05)
      return x[0] > 0;
    return false;
  }
  else if (-0.1 < x[2] && x[2] < 0.1)
  {
    if (-0.05 < x[0] && x[0] < 0.05)
      return x[1] < 0;
    return false;
  }
  else if (0.6 < x[2] && x[2] < 0.80)
  {
    if (-0.05 < x[1] && x[1] < 0.05)
      return x[0] < 0;
    return false;
  }

  return true;
}

// Planningを実行する関数
bool executePlanning(void)
{
  // 状態空間を定義する。
  auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);

  // Planningの境界条件を定義。
  ob::RealVectorBounds bounds(3);
  bounds.setLow(-2);
  bounds.setHigh(2);
  rvss->setBounds(bounds);

  // 円表面制約クラスのshared_ptrを定義。
  auto constraint = std::make_shared<SphereConstraint>();

  // 制約つき問題クラスのインスタンスを生成。
  OMPL_INFORM("Using Projection-Based State Space!");
  auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);
  auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
  css->setup();

  // Simple Setupクラスポインタを定義
  auto ss = std::make_shared<og::SimpleSetup>(csi);

  // Start位置ベクトルとGoal位置ベクトル
  Eigen::VectorXd sv(3), gv(3);
  sv << 0, 0, -1;
  gv << 0, 0, 1;

  ob::ScopedState<> start(css);
  ob::ScopedState<> goal(css);
  start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
  ss->setStartAndGoalStates(start, goal);

  // validity checkerの設定。障害物判定関数を入れる。
  ss->setStateValidityChecker(obstacles);

  // Projection（射影）関数の定義。自分で定義しない場合はデフォルト実装のNewton法が使われる。
  css->registerProjection("sphere", std::make_shared<SphereProjection>(css));

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

  return stat;
}

int main(int argc, char **argv)
{
  executePlanning();
  return 0;
}