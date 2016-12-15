#include "gtest/gtest.h"
#include "utilities/pointpicker.h"
#include "model/pointcloud.h"
#include "utilities/cv.h"

#include <QTime>
#include <QDebug>

namespace {

// The fixture for testing class Foo.
class PerfTest : public ::testing::Test {
 protected:

    QTime t;
    int ms;

    PerfTest() : ms(0) {}

    virtual ~PerfTest() {}

    virtual void SetUp() {
        //t.restart();
        //t.start();
    }

    virtual void TearDown() {
        //int ms = t.elapsed();
    }
};

TEST_F(PerfTest, NearestNeighbours) {
    //EXPECT_EQ(0, 0);

    PointCloud cloud;
    bool succ = cloud.load_ptx("../../../misc/testdata/plane4x5.ptx");

    EXPECT_EQ(succ, true);

    // Test
    int idx = 4;

    std::vector<int> nn;
    grid_nn_op(idx, cloud, nn, 1.0, 50);
    for(int n : nn) {
        qDebug() << "Idx: " << n << "XYX: " << cloud.points[n].x << cloud.points[n].y << cloud.points[n].z;
    }


}

TEST(PTXTest, DISABLED_Read) {
    PointCloud pc;
    bool succ = pc.load_ptx("/home/rickert/Masters/utilities/ptxmaker/out.ptx");
    EXPECT_EQ(succ, true);
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
