#include "agilib/utils/logger.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "agilib/math/types.hpp"
#include "agilib/types/quad_state.hpp"
#include "agilib/utils/throttler.hpp"
#include "agilib/utils/timer.hpp"

using namespace agi;

struct LoggerConstCaller {
  Logger logger{"LoggerConstCaller"};
  void printInfo(const std::string& info) const { logger.info(info.c_str()); }
};

TEST(Logger, SimpleLogging) {
  Logger logger("Test");
  Timer timer("Timer", "Printing");
  timer.tic();

  logger << "This is a text stream log." << std::endl;
  logger.info("This is an info log.");
  logger.warn("This could be a warning, but just for demo.");
  logger.error("This could be an error, but just for demo.");

  logger.info(
    "You can print strings like \"%s\", and formatted numbers like %1.3f.",
    "text", M_PI);
  logger
    << "You can use the stream operator \'<<\' just like with \'std::cout\'."
    << std::endl;
  logger << "This can be helpul for printing complex objects like Eigen vector "
            "and matrices:"
         << std::endl
         << "A vector:" << std::endl
         << Vector<4>(0, 1, 2, 3).transpose() << std::endl
         << "A Matrix:" << std::endl
         << Matrix<3, 3>::Identity() << std::endl;

  timer.toc();
  QuadState state;
  state.setZero();
  logger << "And also our own defined objects, like so:" << std::endl
         << "A timer:" << std::endl
         << timer << std::endl
         << "A QuadState" << std::endl
         << state << std::endl;
}

TEST(Logger, NoColorLogging) {
  Logger logger("Test", false);

  logger << "This is a text stream log." << std::endl;
  logger.info("This is an info log.");
  logger.warn("This could be a warning, but just for demo.");
  logger.error("This could be an error, but just for demo.");
}

TEST(Logger, FatalLogging) {
  Logger logger("Test");
  const std::string message("Fatal message");
  EXPECT_THROW(logger.fatal(message.c_str()), std::runtime_error);
}

TEST(Logger, LogFromConstFunction) {
  const LoggerConstCaller caller;
  caller.printInfo("This should log just fine from a const call!");
}


TEST(Logger, ThrottledLogging) {
  Logger logger("ThrottledLogger");
  Throttler throttler(logger, 1.0);

  for (int i = 0; i < 101; ++i) {
    throttler(&Logger::info, "This should only appear %d times.", 2);
    usleep(10000);
  }
}


TEST(Logger, DebugLogging) {
  Logger logger("DebugLogger");

  logger.debug("This only prints with %s", "debug logging enabled!");
  logger.debug() << "This vector only prints with debug logging enabled:"
                 << Vector<3>(1, 2, 3).transpose() << std::endl;
  logger.debug([&]() {
    logger.info("This lambda only counts to 10 with debug logging enabled");
    for (int i = 1; i <= 10; ++i) logger << i << std::endl;
  });
}