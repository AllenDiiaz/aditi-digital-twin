// Stub motion backend for CI and compile checks only.
//
// It talks to no hardware: every command succeeds and does nothing, every axis
// reports Standstill at position 0. It exists so the bridge node can be built
// and started without a vendor library. To drive real hardware, implement
// IMotionBackend for your controller instead (see motion_backend.hpp).

#include "gantry_bridge/motion_backend.hpp"

namespace gantry_bridge {
namespace {

class StubBackend final : public IMotionBackend {
 public:
  bool open() override { return true; }
  void close() override {}
  bool set_profile(int, const MotionProfile&) override { return true; }
  bool reset_error(int) override { return true; }
  bool enable(int) override { return true; }
  bool move_absolute(int, double) override { return true; }
  bool jog(int, Direction) override { return true; }
  bool change_velocity(int, double, double, double) override { return true; }
  bool reset_pvt(int) override { return true; }
  bool load_pvt(int, const std::vector<double>&, const std::vector<double>&,
                const std::vector<double>&) override { return true; }
  bool start_pvt(int) override { return true; }
  bool stop(int, StopMode, double) override { return true; }
  std::optional<AxisState> state(int) override { return AxisState::Standstill; }
  std::optional<double> actual_position(int) override { return 0.0; }
  std::optional<double> command_position(int) override { return 0.0; }
  std::optional<double> actual_velocity(int) override { return 0.0; }
};

}  // namespace

std::unique_ptr<IMotionBackend> create_motion_backend() {
  return std::make_unique<StubBackend>();
}

}  // namespace gantry_bridge