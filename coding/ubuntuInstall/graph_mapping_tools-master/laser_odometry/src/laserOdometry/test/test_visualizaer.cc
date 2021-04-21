
#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "lio/LineObject.h"
#include "utils/TicToc.h"
#include "visualizer/Visualizer.h"

using namespace lio;
using namespace std;

TEST(LineObjectTest, LineObjectCreate) {

}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_alsologtostderr = true;

  PlaneNormalVisualizer vis;

  vis.Spin();

  return RUN_ALL_TESTS();
}