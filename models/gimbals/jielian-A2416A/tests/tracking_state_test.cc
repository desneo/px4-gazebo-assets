#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "tracking.hh"

using jielian_a2416a::Box;
using jielian_a2416a::Detection;
using jielian_a2416a::LockMode;
using jielian_a2416a::Optics;
using jielian_a2416a::ProjectBounds;
using jielian_a2416a::TrackState;
using jielian_a2416a::TrackerConfig;
using jielian_a2416a::TruthTracker;
using jielian_a2416a::Vec3;

namespace
{
Detection D(std::uint64_t id, std::string name, double x, double y,
            double range, double w = 80, double h = 50)
{
  return {id, std::move(name), {x - w / 2, y - h / 2, w, h},
          x, y, range, 1.0, 4, false, 0, 0, 0};
}

TruthTracker Tracker()
{
  TruthTracker tracker;
  TrackerConfig config;
  assert(tracker.Configure(config));
  return tracker;
}
}

int main()
{
  {
    Optics optics;
    const Vec3 center{500, -10, 5};
    std::array<Vec3, 8> corners;
    std::size_t index = 0;
    for (const double x : {497.0, 503.0})
      for (const double y : {-16.0, -4.0})
        for (const double z : {4.0, 6.0})
          corners[index++] = {x, y, z};
    const auto detection = ProjectBounds(1, "aircraft", corners, center, optics);
    assert(detection);
    assert(detection->centerX > optics.cx);  // 目标右侧为正偏航。
    assert(detection->centerY < optics.cy);  // 目标上方为正俯仰。
    assert(detection->box.w > 0 && detection->box.h > 0);
    assert(!detection->geographicValid && detection->latitudeDeg == 0 &&
           detection->longitudeDeg == 0 && detection->altitudeM == 0);
    auto outside = center;
    outside.y = -1000;
    assert(!ProjectBounds(1, "outside", corners, outside, optics));
  }

  {
    auto tracker = Tracker();
    tracker.SetRecognition(true);
    tracker.Update(0.0, {D(1, "z", 1000, 540, 500), D(2, "b", 930, 540, 500),
                         D(3, "a", 990, 540, 500), D(4, "d", 800, 540, 500),
                         D(5, "e", 1200, 540, 500)});
    assert(tracker.State() == TrackState::DETECTING);
    assert(tracker.Slots().size() == 4);
    assert(tracker.Slots()[0].name == "a");
    assert(tracker.Slots()[1].name == "b");
    assert(tracker.Slots()[2].name == "z");
    assert(tracker.Slots()[3].name == "d");

    tracker.SetAutoLock(true, 0.0);
    assert(tracker.Locked() && tracker.Locked()->name == "a");
    tracker.Update(.02, {D(1, "z", 1000, 540, 500), D(2, "b", 930, 540, 500),
                         D(3, "a", 990, 540, 500)});
    assert(tracker.Slots()[0].name == "a");
    assert(tracker.State() == TrackState::TRACKING);
  }

  {
    auto tracker = Tracker();
    tracker.SetRecognition(true);
    tracker.Update(0.0, {D(10, "manual", 1200, 400, 1000),
                         D(11, "auto", 960, 540, 700)});
    // 1000m 目标不出现在自动检测槽位，但仍能人工点选。
    assert(tracker.Slots().size() == 1 && tracker.Slots()[0].name == "auto");
    assert(tracker.PointLock(1200, 400, .01));
    assert(tracker.Locked()->name == "manual");
    assert(tracker.Mode() == LockMode::MANUAL);
    assert(!tracker.AutoLockEnabled());
    const auto miss = tracker.MisalignmentDeg();
    assert(miss[0] > 0 && miss[1] > 0);

    tracker.Update(.02, {});
    assert(tracker.State() == TrackState::TEMP_LOST);
    assert(tracker.Locked());
    tracker.Update(2.51, {});
    assert(tracker.State() == TrackState::TEMP_LOST);
    tracker.Update(2.53, {});
    assert(tracker.State() == TrackState::LOST);
    assert(!tracker.Locked());
  }

  {
    auto tracker = Tracker();
    tracker.SetRecognition(true);
    tracker.Update(0.0, {D(1, "auto", 960, 540, 500)});
    tracker.SetAutoLock(true, 0.0);
    assert(tracker.Locked() && tracker.AutoLockEnabled());
    tracker.SetRecognition(false);
    assert(!tracker.Locked() && !tracker.AutoLockEnabled());
  }

  {
    auto tracker = Tracker();
    tracker.Update(0.0, {D(1, "box", 960, 540, 1100, 100, 80)});
    assert(tracker.BoxLock({930, 510, 60, 60}, .01));
    const auto base = tracker.Gate();
    assert(base && std::abs(base->w - 100) < 1e-9);
    tracker.ScaleGate(1.1);
    const auto larger = tracker.Gate();
    assert(larger && std::abs(larger->w - 110) < 1e-9);
    assert(tracker.TrimAim(1000, -1000));
    const auto trimmed = tracker.Gate();
    assert(trimmed && trimmed->x >= 0 && trimmed->y >= 0);
  }

  std::cout << "PASS: A2416A truth projection and tracking state machine\n";
  return 0;
}
