
#include <gtest/gtest.h>

#include "agilib/pilot/pilot.hpp"

using namespace agi;


TEST(Pilot, ParamsConstructor) { PilotParams params; }
TEST(Pilot, Constructor) { Pilot pilot(PilotParams{}, ChronoTime); }
