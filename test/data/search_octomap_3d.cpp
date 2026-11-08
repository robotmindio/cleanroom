// Regenerate search_octomap_3d.bin with the native OctoMap writer:
// c++ search_octomap_3d.cpp -loctomap -loctomath -o /path/to/generator
// /path/to/generator search_octomap_3d.bin
#include <octomap/OcTree.h>
#include <fstream>
int main(int argc, char** argv)
{
  if (argc != 2) return 2;
  octomap::OcTree tree(0.05);
  for (int x = -10; x < 41; ++x)
    for (int y = -20; y < 21; ++y)
      for (int z = 0; z < 20; ++z)
        tree.updateNode(octomap::point3d((x + .5) * .05, (y + .5) * .05, (z + .5) * .05), false);
  // An overhang invisible to the lidar's 2D occupancy grid.
  for (int x = 12; x <= 16; ++x)
    for (int y = -5; y <= 4; ++y)
      for (int z = 3; z <= 6; ++z)
        tree.updateNode(octomap::point3d((x + .5) * .05, (y + .5) * .05, (z + .5) * .05), true);
  tree.toMaxLikelihood();
  tree.prune();
  std::ofstream output(argv[1], std::ios::binary);
  tree.writeBinaryData(output);
  return output.good() ? 0 : 1;
}
