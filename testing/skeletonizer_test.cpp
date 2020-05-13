#include <unistd.h>
#include "auto_mapping_ros/skeletonizer.h"
#include "auto_mapping_ros/utils.h"

int main()
{
    std::string pkgdir = amr::get_package_directory();

    // Example 1
    amr::Skeletonizer processor;
    if(processor.read_map(pkgdir + "/maps/levine.jpg") < 0) return -1;
    const auto skeleton = processor.skeletonize();

    // Example 2
    amr::Skeletonizer processor2;
    if(processor2.read_map(pkgdir + "/maps/levine_4.jpg") < 0) return -1;
    const auto skeleton2 = processor2.skeletonize();

    return 0;
}
