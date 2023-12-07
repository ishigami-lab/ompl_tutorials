#include <kinematic_chain.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
  
class ConstrainedKinematicChainValidityChecker : public KinematicChainValidityChecker
{
public:
  ConstrainedKinematicChainValidityChecker(const ob::ConstrainedSpaceInformationPtr &si)
  : KinematicChainValidityChecker(si)
  {
  }

  bool isValid(const ob::State *state) const override
  {
    auto &&space = si_->getStateSpace()->as<ob::ConstrainedStateSpace>()->getSpace()->as<KinematicChainSpace>();
    auto &&s = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
    return isValidImpl(space, s);
  }
};
  
class KinematicChainConstraint : public ob::Constraint
{
public:
    KinematicChainConstraint(unsigned int links, double linkLength) : ob::Constraint(links, 1), linkLength_(linkLength)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
      Eigen::Vector2d e = Eigen::Vector2d::Zero(), eN = Eigen::Vector2d::Zero();
      double theta = 0.;

      for (unsigned int i = 0; i < x.size(); ++i)
      {
        theta += x[i];
        eN[0] = e[0] + cos(theta) * linkLength_;
        eN[1] = e[1] + sin(theta) * linkLength_;
        e = eN;
      }

      out[0] = e[1] - linkLength_;
    }

private:
    double linkLength_;
};

ob::PlannerStatus chainPlanning(unsigned int links)
{
  Environment env = createEmptyEnvironment(links);

  // Planningの境界条件を与えた状態空間を定義
  auto rvss = std::make_shared<KinematicChainSpace>(links, 1. / (double)links, &env);

  // 制約クラスのshared_ptrを定義
  auto constraint = std::make_shared<KinematicChainConstraint>(links, 1. / (double)links);

  // 制約つき問題クラスのインスタンスを生成。
  OMPL_INFORM("Using Projection-Based State Space!");
  auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);
  auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
  css->setup();

  // Simple Setupクラスポインタを定義
  auto ss = std::make_shared<og::SimpleSetup>(csi);
  
  // Start位置ベクトルとGoal位置ベクトルを設定
  Eigen::VectorXd sv, gv;
  sv = Eigen::VectorXd::Constant(links, 0);
  gv = Eigen::VectorXd::Constant(links, 0);

  sv[links - 1] = boost::math::constants::pi<double>() / 2;
  gv[0] = boost::math::constants::pi<double>();
  gv[links - 1] = -boost::math::constants::pi<double>() / 2;

  ob::ScopedState<> start(css);
  ob::ScopedState<> goal(css);
  start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
  ss->setStartAndGoalStates(start, goal);

  // validity checkerの設定。障害物判定関数を入れる
  ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(csi));

  // Plannerの設定
  auto pp = std::make_shared<og::RRT>(csi);
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
  unsigned int links = 5;

  ob::PlannerStatus stat;
  stat = chainPlanning(links);
  std::cout << "Planner finished with status '" << stat << "'." << std::endl;

  return 0;
}