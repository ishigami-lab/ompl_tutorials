#include <iostream>
#include <fstream>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/prm/PRM.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class ParallelBase
{
public:
  virtual void getStart(Eigen::VectorXd &x) = 0;
  virtual void getGoal(Eigen::VectorXd &x) = 0;
};

// Chain(リンク結合部)に関する制約条件を規定するクラス
class ParallelChain : public ob::Constraint, public ParallelBase
{
public:
  ParallelChain(const unsigned int n, Eigen::Vector3d offset, unsigned int links, unsigned int chainNum, double length = 1)
  : ob::Constraint(n, links)
  , offset_(std::move(offset))
  , links_(links)
  , chainNum_(chainNum)
  , length_(length)
  {
    if (links % 2 == 0) throw ompl::Exception("Number of links must be odd!");
  }

  void getStart(Eigen::VectorXd &x) override
  {
    const double angle = boost::math::constants::pi<double>() / 16;
    const unsigned int offset = 3 * links_ * chainNum_;
    const Eigen::VectorXd axis =
        Eigen::AngleAxisd(boost::math::constants::pi<double>() / 2, Eigen::Vector3d::UnitZ()) * offset_;

    const Eigen::VectorXd step = Eigen::Vector3d::UnitZ() * length_;
    Eigen::VectorXd joint = offset_ + Eigen::AngleAxisd(angle, axis) * step;

    unsigned int i = 0;
    for (; i < links_; ++i)
    {
      x.segment(3 * i + offset, 3) = joint;
      if (i < links_ - 2)
        joint += step;
      else
        joint += Eigen::AngleAxisd(-angle, axis) * step;
    }
  }

  void getGoal(Eigen::VectorXd &x) override
  {
    unsigned int offset = 3 * links_ * chainNum_;

    if (links_ == 7)
    {
      Eigen::VectorXd nstep = offset_ * length_;
      Eigen::VectorXd estep = Eigen::AngleAxisd(boost::math::constants::pi<double>() / 2, Eigen::Vector3d::UnitZ()) * offset_ * length_;
      Eigen::VectorXd sstep = Eigen::AngleAxisd(boost::math::constants::pi<double>(), Eigen::Vector3d::UnitZ()) * offset_ * length_;
      Eigen::VectorXd wstep = Eigen::AngleAxisd(3 * boost::math::constants::pi<double>() / 2, Eigen::Vector3d::UnitZ()) * offset_ * length_;

      Eigen::VectorXd joint = offset_ + nstep;
      x.segment(3 * 0 + offset, 3) = joint;
      x.segment(3 * 1 + offset, 3) = x.segment(3 * 0 + offset, 3) + estep;
      x.segment(3 * 2 + offset, 3) = x.segment(3 * 1 + offset, 3) + estep;
      x.segment(3 * 3 + offset, 3) = x.segment(3 * 2 + offset, 3) + Eigen::Vector3d::UnitZ() * length_;
      x.segment(3 * 4 + offset, 3) = x.segment(3 * 3 + offset, 3) + sstep;
      x.segment(3 * 5 + offset, 3) = x.segment(3 * 4 + offset, 3) + sstep;
      x.segment(3 * 6 + offset, 3) = x.segment(3 * 5 + offset, 3) + wstep;
    }
    else
    {
      Eigen::VectorXd step = offset_ * length_;
      Eigen::VectorXd joint = offset_ + step;

      unsigned int i = 0;
      for (; i < links_ / 2; ++i, joint += step)
        x.segment(3 * i + offset, 3) = joint;

      joint += Eigen::Vector3d::UnitZ() * length_ - step;
      for (; i < links_; ++i, joint -= step)
        x.segment(3 * i + offset, 3) = joint;
    }
  }

  Eigen::Ref<const Eigen::VectorXd> getLink(const Eigen::VectorXd &x, const unsigned int idx) const
  {
    const unsigned int offset = 3 * links_ * chainNum_;
    return x.segment(offset + 3 * idx, 3);
  }

  // 制約関数f(x)=0の定義
  void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    unsigned int idx = 0;

    Eigen::VectorXd j1 = offset_;
    for (unsigned int i = 0; i < links_; ++i)
    {
      const Eigen::VectorXd j2 = getLink(x, i);
      out[idx++] = (j1 - j2).norm() - length_;
      j1 = j2;
    }
  }

  // 制約関数f(x)のヤコビアンの定義（オプション）
  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
  {
    const unsigned int offset = 3 * links_ * chainNum_;
    out.setZero();

    Eigen::VectorXd plus(3 * (links_ + 1));
    plus.head(3 * links_) = x.segment(offset, 3 * links_);
    plus.tail(3) = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd minus(3 * (links_ + 1));
    minus.head(3) = offset_;
    minus.tail(3 * links_) = x.segment(offset, 3 * links_);

    const Eigen::VectorXd diagonal = plus - minus;

    for (unsigned int i = 0; i < links_; i++)
      out.row(i).segment(3 * i + offset, 3) = diagonal.segment(3 * i, 3).normalized();

    out.block(1, offset, links_ - 1, 3 * links_ - 3) -= out.block(1, offset + 3, links_ - 1, 3 * links_ - 3);
  }

private:
  const Eigen::Vector3d offset_;
  const unsigned int links_;
  const unsigned int chainNum_;
  const double length_;
};

// Platform(台座部位)に関する制約条件を規定するクラス
class ParallelPlatform : public ob::Constraint, public ParallelBase
{
public:
  ParallelPlatform(unsigned int links, unsigned int chains, double radius = 1)
  : ob::Constraint(3 * links * chains, chains), links_(links), chains_(chains), radius_(radius)
  {
    if (chains == 2)
      setManifoldDimension(k_ + 1);

    if (chains >= 4)
      setManifoldDimension(k_ - (chains - 3));
  }

  Eigen::Ref<const Eigen::VectorXd> getTip(const Eigen::VectorXd &x, unsigned int id) const
  {
    return x.segment(3 * links_ * ((id % chains_) + 1) - 3, 3);
  }

  // 制約関数f(x)=0の定義
  void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
  {
    if (chains_ == 2)
    {
      out[0] = (getTip(x, 0) - getTip(x, 1)).norm() - radius_ * 2;
      return;
    }

    unsigned int idx = 0;

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (unsigned int i = 0; i < chains_; ++i)
      centroid += getTip(x, i);
    centroid /= chains_;

    for (unsigned int i = 0; i < chains_; ++i)
      out[idx++] = (centroid - getTip(x, i)).norm() - radius_;

    for (unsigned int i = 0; i < chains_ - 3; ++i)
    {
      const Eigen::Vector3d ab = getTip(x, i + 1) - getTip(x, i);
      const Eigen::Vector3d ac = getTip(x, i + 2) - getTip(x, i);
      const Eigen::Vector3d ad = getTip(x, i + 3) - getTip(x, i);

      out[idx++] = ad.dot(ab.cross(ac));
    }
  }

  void getStart(Eigen::VectorXd &) override
  {
  }

  void getGoal(Eigen::VectorXd &) override
  {
  }

private:
  const unsigned int links_;
  const unsigned int chains_;
  const double radius_;
};

// 制約条件（全体）を規定するクラス
// Chainの制約条件とPlatformの制約条件を統合し、一つの制約クラスにしている
class ParallelConstraint : public ob::ConstraintIntersection, public ParallelBase
{
public:
  ParallelConstraint(unsigned int links, unsigned int chains, double radius = 1, double length = 1, double jointRadius = 0.2)
  : ConstraintIntersection(3 * links * chains, {})
  , links_(links)
  , chains_(chains)
  , radius_(radius)
  , length_(length)
  , jointRadius_(jointRadius)
  {
    // Chain(リンク結合部)に関する制約条件を追加
    Eigen::Vector3d offset = Eigen::Vector3d::UnitX();
    for (unsigned int i = 0; i < chains_; ++i)
    {
      // addConstraint(std::make_shared<ParallelChain>(chains * links * 3, offset, links, i, length));
      addConstraint(new ParallelChain(chains * links * 3, offset, links, i, length));
      offset = Eigen::AngleAxisd(2 * boost::math::constants::pi<double>() / (double)chains, Eigen::Vector3d::UnitZ()) * offset;
    }

    // Plaform(台座)に関する制約条件を追加
    // addConstraint(std::make_shared<ParallelPlatform>(links, chains, radius));
    addConstraint(new ParallelPlatform(links, chains, radius));
  }

  void getStart(Eigen::VectorXd &x) override
  {
    x = Eigen::VectorXd(3 * links_ * chains_);
    for (auto &constraint : constraints_)
    {
      // std::dynamic_pointer_cast<ParallelBase>(constraint)->getStart(x);
      dynamic_cast<ParallelBase *>(constraint)->getStart(x);
    }
  }

  void getGoal(Eigen::VectorXd &x) override
  {
    x = Eigen::VectorXd(3 * links_ * chains_);
    for (auto &constraint : constraints_)
    {
      // std::dynamic_pointer_cast<ParallelBase>(constraint)->getGoal(x);
      dynamic_cast<ParallelBase *>(constraint)->getGoal(x);
    }
  }

  ob::StateSpacePtr createSpace() const
  {
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3 * links_ * chains_);
    ob::RealVectorBounds bounds(3 * links_ * chains_);

    for (unsigned int c = 0; c < chains_; ++c)
    {
      const unsigned int o = 3 * c * links_;
      for (int i = 0; i < (int)links_; ++i)
      {
        bounds.setLow(o + 3 * i + 0, -i - 2);
        bounds.setHigh(o + 3 * i + 0, i + 2);

        bounds.setLow(o + 3 * i + 1, -i - 2);
        bounds.setHigh(o + 3 * i + 1, i + 2);

        bounds.setLow(o + 3 * i + 2, -i - 2);
        bounds.setHigh(o + 3 * i + 2, i + 2);
      }
    }

    rvss->setBounds(bounds);
    return rvss;
  }

  bool isValid(const ob::State *state)
  {
    auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

    for (unsigned int i = 0; i < links_ * chains_; ++i)
    {
      if (x.segment(3 * i, 3)[2] < 0) return false;
    }

    for (unsigned int i = 0; i < links_ * chains_ - 1; ++i)
    {
      if (x.segment(3 * i, 3).cwiseAbs().maxCoeff() < jointRadius_) return false;

      for (unsigned int j = i + 1; j < links_ * chains_; ++j)
        if ((x.segment(3 * i, 3) - x.segment(3 * j, 3)).cwiseAbs().maxCoeff() < jointRadius_) return false;
    }

    return true;
  }

  ob::ProjectionEvaluatorPtr getProjection(ob::StateSpacePtr space) const
  {
    // 制約付き状態空間の射影を行うクラス
    class ParallelProjection : public ob::ProjectionEvaluator
    {
    public:
      ParallelProjection(const ob::StateSpacePtr &space, unsigned int links, unsigned int chains)
      : ob::ProjectionEvaluator(space), chains_(chains), links_(links)
      {
      }

      unsigned int getDimension() const override
      {
        return 1;
      }

      void defaultCellSizes() override
      {
        cellSizes_.resize(1);
        cellSizes_[0] = 0.1;
      }

      void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
      {
        auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

        for (unsigned int i = 0; i < chains_; ++i)
          projection(0) = x[3 * (i + 1) * links_ - 1];

        projection(0) /= chains_;
      }

    private:
      const unsigned int chains_;
      const unsigned int links_;
    };

    return std::make_shared<ParallelProjection>(space, links_, chains_);
  }

  void dump(std::ofstream &file) const
  {
    file << links_ << std::endl;
    file << chains_ << std::endl;
    file << jointRadius_ << std::endl;
    file << length_ << std::endl;
    file << radius_ << std::endl;
  }

private:
  const unsigned int links_;
  const unsigned int chains_;
  const double radius_;
  const double length_;
  const double jointRadius_;
};

ob::PlannerStatus parallelPlanning(unsigned int links, unsigned int chains)
{
    // 制約クラスのshared_ptrを定義
  auto constraint = std::make_shared<ParallelConstraint>(links, chains);

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
  css->registerProjection("parallel", constraint->getProjection(css));

  // Start位置ベクトルとGoal位置ベクトルを設定
  Eigen::VectorXd sv, gv;
  constraint->getStart(sv);
  constraint->getGoal(gv);

  ob::ScopedState<> start(css);
  ob::ScopedState<> goal(css);
  start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
  goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
  ss->setStartAndGoalStates(start, goal);

  // validity checkerの設定。障害物判定関数を入れる
  ss->setStateValidityChecker(std::bind(&ParallelConstraint::isValid, constraint, std::placeholders::_1));

  // Plannerの設定
  // auto pp = std::make_shared<og::RRTConnect>(csi);
  auto pp = std::make_shared<og::PRM>(csi);
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
  // 出力順：リンク数(links)、チェイン数(chains)、ジョイント半径(jointRadius)、
  // 長さ(length)、半径(radius)
  OMPL_INFORM("Dumping problem information to `parallel_info.txt`.");
  std::ofstream infofile("../output/parallel_info.txt");
  dynamic_cast<ParallelConstraint *>(constraint.get())->dump(infofile);
  infofile.close();

  return stat;
}

int main(int argc, char **argv)
{
  unsigned int links = 3;
  unsigned int chains = 4;

  ob::PlannerStatus stat;
  stat = parallelPlanning(links, chains);
  std::cout << "Planner finished with status '" << stat << "'." << std::endl;

  return 0;
}