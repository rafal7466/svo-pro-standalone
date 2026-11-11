// Stubs for OpenGV's machine-generated Groebner-basis solvers
// (Kneip 5-point, 6-point non-central relative pose) and the generalized
// eigensolver (GE) module.
//
// These sources (~1.5 MB of generated code) take 10-20 minutes to compile with
// MSVC and are NOT used by the SVO mono / stereo pipelines (SVO uses Nister's
// 5-point algorithm and its 2-point translation solver). They are only needed
// for the camera-array initializer (FrameHandlerArray, SIXPT), which is not
// exposed by svo_euroc. Calling them throws a descriptive error.

#include <stdexcept>
#include <opengv/relative_pose/modules/fivept_kneip/modules.hpp>
#include <opengv/relative_pose/modules/sixpt/modules.hpp>
#include <opengv/relative_pose/modules/ge/modules.hpp>

namespace opengv {
namespace relative_pose {
namespace modules {

namespace fivept_kneip {
Eigen::Matrix<double, 1, 197> initEpncpRowR(
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >&,
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >&)
{
  throw std::runtime_error(
      "opengv fivept_kneip is not compiled in this build (use NISTER/STEWENIUS)");
}
void initMatrix(Eigen::Matrix<double, 66, 197>&)
{
  throw std::runtime_error("opengv fivept_kneip is not compiled in this build");
}
void computeBasis(Eigen::Matrix<double, 66, 197>&)
{
  throw std::runtime_error("opengv fivept_kneip is not compiled in this build");
}
} // namespace fivept_kneip

namespace sixpt {
void setupAction(
    const std::vector<Eigen::Matrix<double, 6, 1>,
                      Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1> > >&,
    const std::vector<Eigen::Matrix<double, 6, 1>,
                      Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1> > >&,
    Eigen::Matrix<double, 64, 64>&)
{
  throw std::runtime_error(
      "opengv sixpt (non-central 6pt) is not compiled in this build");
}
} // namespace sixpt

} // namespace modules
} // namespace relative_pose
} // namespace opengv

namespace opengv {
namespace relative_pose {
namespace modules {
namespace ge {
void getEV(
    const Eigen::Matrix3d & xxF,
    const Eigen::Matrix3d & yyF,
    const Eigen::Matrix3d & zzF,
    const Eigen::Matrix3d & xyF,
    const Eigen::Matrix3d & yzF,
    const Eigen::Matrix3d & zxF,
    const Eigen::Matrix<double,3,9> & x1P,
    const Eigen::Matrix<double,3,9> & y1P,
    const Eigen::Matrix<double,3,9> & z1P,
    const Eigen::Matrix<double,3,9> & x2P,
    const Eigen::Matrix<double,3,9> & y2P,
    const Eigen::Matrix<double,3,9> & z2P,
    const Eigen::Matrix<double,9,9> & m11P,
    const Eigen::Matrix<double,9,9> & m12P,
    const Eigen::Matrix<double,9,9> & m22P,
    const cayley_t & cayley,
    Eigen::Vector4d & roots )
{
  throw std::runtime_error("opengv ge::getEV (generalized eigensolver) is not compiled in this build");
}
double getCost(
    const Eigen::Matrix3d & xxF,
    const Eigen::Matrix3d & yyF,
    const Eigen::Matrix3d & zzF,
    const Eigen::Matrix3d & xyF,
    const Eigen::Matrix3d & yzF,
    const Eigen::Matrix3d & zxF,
    const Eigen::Matrix<double,3,9> & x1P,
    const Eigen::Matrix<double,3,9> & y1P,
    const Eigen::Matrix<double,3,9> & z1P,
    const Eigen::Matrix<double,3,9> & x2P,
    const Eigen::Matrix<double,3,9> & y2P,
    const Eigen::Matrix<double,3,9> & z2P,
    const Eigen::Matrix<double,9,9> & m11P,
    const Eigen::Matrix<double,9,9> & m12P,
    const Eigen::Matrix<double,9,9> & m22P,
    const cayley_t & cayley,
    int step )
{
  throw std::runtime_error("opengv ge::getCost (generalized eigensolver) is not compiled in this build");
}
double getCostWithJacobian(
    const Eigen::Matrix3d & xxF,
    const Eigen::Matrix3d & yyF,
    const Eigen::Matrix3d & zzF,
    const Eigen::Matrix3d & xyF,
    const Eigen::Matrix3d & yzF,
    const Eigen::Matrix3d & zxF,
    const Eigen::Matrix<double,3,9> & x1P,
    const Eigen::Matrix<double,3,9> & y1P,
    const Eigen::Matrix<double,3,9> & z1P,
    const Eigen::Matrix<double,3,9> & x2P,
    const Eigen::Matrix<double,3,9> & y2P,
    const Eigen::Matrix<double,3,9> & z2P,
    const Eigen::Matrix<double,9,9> & m11P,
    const Eigen::Matrix<double,9,9> & m12P,
    const Eigen::Matrix<double,9,9> & m22P,
    const cayley_t & cayley,
    Eigen::Matrix<double,1,3> & jacobian,
    int step )
{
  throw std::runtime_error("opengv ge::getCostWithJacobian (generalized eigensolver) is not compiled in this build");
}
void getQuickJacobian(
    const Eigen::Matrix3d & xxF,
    const Eigen::Matrix3d & yyF,
    const Eigen::Matrix3d & zzF,
    const Eigen::Matrix3d & xyF,
    const Eigen::Matrix3d & yzF,
    const Eigen::Matrix3d & zxF,
    const Eigen::Matrix<double,3,9> & x1P,
    const Eigen::Matrix<double,3,9> & y1P,
    const Eigen::Matrix<double,3,9> & z1P,
    const Eigen::Matrix<double,3,9> & x2P,
    const Eigen::Matrix<double,3,9> & y2P,
    const Eigen::Matrix<double,3,9> & z2P,
    const Eigen::Matrix<double,9,9> & m11P,
    const Eigen::Matrix<double,9,9> & m12P,
    const Eigen::Matrix<double,9,9> & m22P,
    const cayley_t & cayley,
    double currentValue,
    Eigen::Matrix<double,1,3> & jacobian,
    int step )
{
  throw std::runtime_error("opengv ge::getQuickJacobian (generalized eigensolver) is not compiled in this build");
}
Eigen::Matrix4d composeG(
    const Eigen::Matrix3d &, const Eigen::Matrix3d &, const Eigen::Matrix3d &,
    const Eigen::Matrix3d &, const Eigen::Matrix3d &, const Eigen::Matrix3d &,
    const Eigen::Matrix<double,3,9> &, const Eigen::Matrix<double,3,9> &,
    const Eigen::Matrix<double,3,9> &, const Eigen::Matrix<double,3,9> &,
    const Eigen::Matrix<double,3,9> &, const Eigen::Matrix<double,3,9> &,
    const Eigen::Matrix<double,9,9> &, const Eigen::Matrix<double,9,9> &,
    const Eigen::Matrix<double,9,9> &, const cayley_t &)
{
  throw std::runtime_error("opengv ge::composeG (generalized eigensolver) is not compiled in this build");
}
} // namespace ge
} // namespace modules
} // namespace relative_pose
} // namespace opengv
