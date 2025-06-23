#include <iostream>
#include <vector>
#include <string>
#include "graph.h"

using ohantsev::Graph;

void printTestResult(bool condition, const std::string& testName)
{
  std::cout << "[" << (condition? "PASS": "FAIL") << "] " << testName << std::endl;
}

void printSectionHeader(const std::string& sectionName)
{
  std::cout << "\n=== " << sectionName << " ===" << std::endl;
}

void testNodeInsertion()
{
  printSectionHeader("Node Insertion");
  Graph< int > g;
  bool allPassed = true;

  allPassed &= g.insert(1);
  printTestResult(allPassed, "Add new node");

  allPassed &= !g.insert(1);
  printTestResult(!g.insert(1), "Reject duplicate node");

  allPassed &= (g.size() == 1);
  printTestResult(g.size() == 1, "Correct size after insertion");

  printTestResult(allPassed, "TOTAL");
}

void testLinkOperations()
{
  printSectionHeader("Link Operations");
  Graph< std::string > g;
  g.insert("A");
  g.insert("B");
  g.insert("C");
  bool allPassed = true;

  allPassed &= g.link("A", "B", 5);
  printTestResult(allPassed, "Create valid link");

  allPassed &= !g.link("A", "D", 3);
  printTestResult(!g.link("A", "D", 3), "Reject invalid link");

  allPassed &= !g.link("A", "A", 0);
  printTestResult(!g.link("A", "A", 0), "Reject self-link");

  bool symmetric = !g.watch("B").empty() && g.watch("B").at("A") == 5;
  allPassed &= symmetric;
  printTestResult(symmetric, "Links are symmetric");

  allPassed &= !g.link("A", "B", 2);
  printTestResult(!g.link("A", "B", 2), "Reject duplicate link");

  printTestResult(allPassed, "TOTAL");
}

void testCycleRemoval()
{
  printSectionHeader("Cycle Removal");
  Graph< int > g;
  bool allPassed = true;

  g.insert(1);
  g.insert(2);
  g.insert(3);
  g.link(1, 2, 1);
  g.link(2, 3, 2);
  g.link(3, 1, 3);

  g.removeCycles();

  size_t edgeCount = 0;
  for (const auto& node: g.watch())
  {
    edgeCount += node.second.size();
  }
  edgeCount /= 2;

  allPassed &= (edgeCount == 2);
  printTestResult(edgeCount == 2, "Removes one edge from triangle");

  bool connected = !g.watch(1).empty() &&
                   !g.watch(2).empty() &&
                   !g.watch(3).empty();
  allPassed &= connected;
  printTestResult(connected, "Graph stays connected");

  Graph< std::string > g2;
  g2.insert("A");
  g2.insert("B");
  g2.link("A", "B", 1);

  size_t before = g2.watch("A").size();
  g2.removeCycles();
  size_t after = g2.watch("A").size();

  allPassed &= (before == after);
  printTestResult(before == after, "Acyclic graph unchanged");

  printTestResult(allPassed, "TOTAL");
}

void testCopyAndMoveSemantics()
{
  printSectionHeader("Copy/Move Semantics");
  bool allPassed = true; {
    Graph< int > g1;
    g1.insert(1);
    g1.insert(2);
    g1.link(1, 2, 10);

    Graph< int > g2(g1);

    allPassed &= (g1.size() == g2.size());
    printTestResult(g1.size() == g2.size(), "Copy constructor - size match");

    bool linksCopied = !g2.watch(1).empty() && g2.watch(1).at(2) == 10;
    allPassed &= linksCopied;
    printTestResult(linksCopied, "Copy constructor - links copied");
  } {
    Graph< int > g1;
    g1.insert(1);
    g1.insert(2);
    g1.link(1, 2, 10);

    size_t originalSize = g1.size();
    Graph< int > g2(std::move(g1));

    allPassed &= (g2.size() == originalSize);
    printTestResult(g2.size() == originalSize, "Move constructor - size preserved");

    allPassed &= (g1.size() == 0);
    printTestResult(g1.size() == 0, "Move constructor - original empty");
  } {
    Graph< int > g1, g2;
    g1.insert(1);
    g1.insert(2);
    g1.link(1, 2, 10);

    g2 = g1;

    allPassed &= (g1.size() == g2.size());
    printTestResult(g1.size() == g2.size(), "Copy assignment - size match");

    bool linksCopied = !g2.watch(1).empty() && g2.watch(1).at(2) == 10;
    allPassed &= linksCopied;
    printTestResult(linksCopied, "Copy assignment - links copied");
  } {
    Graph< int > g1, g2;
    g1.insert(1);
    g1.insert(2);
    g1.link(1, 2, 10);

    size_t originalSize = g1.size();
    g2 = std::move(g1);

    allPassed &= (g2.size() == originalSize);
    printTestResult(g2.size() == originalSize, "Move assignment - size preserved");

    allPassed &= (g1.size() == 0);
    printTestResult(g1.size() == 0, "Move assignment - original empty");
  }

  printTestResult(allPassed, "TOTAL");
}

void testRemovalOperations()
{
  printSectionHeader("Removal Operations");
  Graph< int > g;
  bool allPassed = true;

  g.insert(1);
  g.insert(2);
  g.insert(3);

  g.link(1, 2, 1);
  g.link(2, 3, 2);

  allPassed &= g.removeLink(1, 2);
  printTestResult(allPassed, "Remove existing link");

  allPassed &= !g.removeLink(1, 2);
  printTestResult(!g.removeLink(1, 2), "Can't remove non-existent link");

  bool symmetricRemoval = g.watch(2).size() == 1;
  allPassed &= symmetricRemoval;
  printTestResult(symmetricRemoval, "Link removal is symmetric");

  allPassed &= !g.remove(2);
  printTestResult(!g.remove(2), "Can't remove connected node");

  allPassed &= g.remove(1);
  printTestResult(allPassed, "Remove isolated node");

  allPassed &= g.removeForce(2);
  printTestResult(allPassed, "Force remove connected node");

  allPassed &= (g.size() == 1);
  printTestResult(g.size() == 1, "Correct size after removals");

  printTestResult(allPassed, "TOTAL");
}

void testDijkstraAlgorithm()
{
  printSectionHeader("Dijkstra Algorithm");
  bool allPassed = true; {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.link("A", "B", 1);
    g.link("B", "C", 2);

    auto way = g.path("A", "C");
    bool correct = (way.steps_.size() == 3) &&
                   (way.steps_[0] == "A") &&
                   (way.steps_[1] == "B") &&
                   (way.steps_[2] == "C") &&
                   (way.length_ == 3);
    allPassed &= correct;
    printTestResult(correct, "Single shortest path (A-B-C)");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.insert("D");
    g.link("A", "B", 1);
    g.link("B", "C", 1);
    g.link("A", "D", 1);
    g.link("D", "C", 1);

    auto way = g.path("A", "C");
    bool correct = (way.steps_.size() == 3) &&
                   (way.steps_[0] == "A") &&
                   ((way.steps_[1] == "B" && way.steps_[2] == "C") ||
                    (way.steps_[1] == "D" && way.steps_[2] == "C")) &&
                   (way.length_ == 2);
    allPassed &= correct;
    printTestResult(correct, "Multiple shortest paths (A-B-C or A-D-C)");
  } {
    Graph< std::string > g;
    g.insert("A");

    auto way = g.path("A", "A");
    bool correct = (way.steps_.size() == 1) &&
                   (way.steps_[0] == "A") &&
                   (way.length_ == 0);
    allPassed &= correct;
    printTestResult(correct, "Path for A-A (single node)");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");

    auto way = g.path("A", "B");
    allPassed &= way.steps_.empty() && way.length_ == 0;
    printTestResult(way.steps_.empty() && way.length_ == 0, "Disconnected graph should return empty paths");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.link("A", "B", 5);
    g.link("B", "C", 3);

    auto way = g.path("A", "C");
    allPassed &= (way.length_ == 8);
    printTestResult(way.length_ == 8, "Correct path length calculation");
  }

  printTestResult(allPassed, "TOTAL");
}

void testNoCyclesPaths()
{
  printSectionHeader("Top paths without cycles");
  bool allPassed = true;

  auto comparePaths = [](const auto& paths, const std::vector< std::vector< std::string > >& expected)
  {
    if (paths.size() != expected.size()) return false;

    std::vector< bool > matched(expected.size(), false);
    for (const auto& path: paths)
    {
      bool found = false;
      for (size_t i = 0; i < expected.size(); ++i)
      {
        if (!matched[i] && path.steps_ == expected[i])
        {
          matched[i] = true;
          found = true;
          break;
        }
      }
      if (!found) return false;
    }
    return true;
  }; {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.link("A", "B", 1);
    g.link("B", "C", 2);

    auto paths = g.nPaths< false >("A", "C", 2);
    std::vector< std::vector< std::string > > expected = { { "A", "B", "C" } };

    bool correct = comparePaths(paths, expected) && (paths[0].length_ == 3);
    allPassed &= correct;
    printTestResult(correct, "Single shortest path (A-B-C) with Yen's");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.insert("D");
    g.link("A", "B", 1);
    g.link("B", "C", 1);
    g.link("A", "D", 1);
    g.link("D", "C", 1);

    auto paths = g.nPaths< false >("A", "C", 2);
    std::vector< std::vector< std::string > > expected = {
    { "A", "B", "C" },
    { "A", "D", "C" }
    };

    bool correct = comparePaths(paths, expected) &&
                   (paths[0].length_ == 2) &&
                   (paths[1].length_ == 2);
    allPassed &= correct;
    printTestResult(correct, "Multiple shortest paths (A-B-C or A-D-C) with Yen's");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");

    auto paths = g.nPaths< false >("A", "B", 2);
    bool correct = paths.empty();
    allPassed &= correct;
    printTestResult(correct, "Disconnected graph should return empty paths");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.insert("D");
    g.insert("E");
    g.link("A", "B", 1);
    g.link("B", "C", 1);
    g.link("A", "D", 2);
    g.link("D", "C", 2);
    g.link("A", "E", 5);
    g.link("E", "C", 1);

    auto paths = g.nPaths< false >("A", "C", 3);
    std::vector< std::vector< std::string > > expected = {
    { "A", "B", "C" },
    { "A", "D", "C" },
    { "A", "E", "C" }
    };

    bool correct = comparePaths(paths, expected) &&
                   (paths[0].length_ == 2) &&
                   (paths[1].length_ == 4) &&
                   (paths[2].length_ == 6);
    allPassed &= correct;
    printTestResult(correct, "Multiple paths with different lengths");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.link("A", "B", 1);
    g.link("B", "C", 1);

    auto paths = g.nPaths< false >("A", "C", 5);
    std::vector< std::vector< std::string > > expected = { { "A", "B", "C" } };

    bool correct = comparePaths(paths, expected);
    allPassed &= correct;
    printTestResult(correct, "Request more paths than exist");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.insert("D");
    g.link("A", "B", 1);
    g.link("B", "C", 1);
    g.link("C", "D", 1);
    g.link("D", "B", 1);

    auto paths = g.nPaths< false >("A", "C", 3);
    std::vector< std::vector< std::string > > expected = { { "A", "B", "C" }, { "A", "B", "D", "C" } };

    bool correct = comparePaths(paths, expected);
    allPassed &= correct;
    printTestResult(correct, "Handles graph with cycles (ignores cyclic paths)");
  } {
    Graph< std::string > g;
    g.insert("1");
    g.insert("2");
    g.insert("3");
    g.insert("4");
    g.insert("5");
    g.link("1", "2", 1);
    g.link("2", "5", 1);
    g.link("1", "3", 1);
    g.link("3", "4", 1);
    g.link("4", "5", 2);

    auto paths = g.nPaths< false >("1", "5", 2);
    std::vector< std::vector< std::string > > expected = {
    { "1", "2", "5" },
    { "1", "3", "4", "5" }
    };

    bool correct = comparePaths(paths, expected) &&
                   (paths[0].length_ == 2) &&
                   (paths[1].length_ == 4);
    allPassed &= correct;
    printTestResult(correct, "Different path lengths are ordered correctly");
  }

  printTestResult(allPassed, "TOTAL");
}

template< class Key, class Hash = std::hash< Key >, class KeyEqual = std::equal_to< Key > >
bool containsPath(const std::vector< typename Graph< Key, Hash, KeyEqual >::Way >& paths,
                  const std::vector< Key >& expected)
{
  for (const auto& path: paths)
  {
    if (path.steps_ == expected)
    {
      return true;
    }
  }
  return false;
}

void testWithCyclesPaths()
{
  printSectionHeader("Top paths with cycles allowed");
  bool allPassed = true; {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.insert("D");
    g.link("A", "B", 1);
    g.link("B", "C", 1);
    g.link("C", "D", 1);
    g.link("D", "B", 1);

    auto paths = g.nPaths< true >("A", "C", 5);

    bool foundBasic = containsPath< std::string >(paths, { "A", "B", "C" });
    bool foundCyclePath = containsPath< std::string >(paths, { "A", "B", "D", "C" }) ||
                          containsPath< std::string >(paths, { "A", "B", "C", "D", "B", "C" });

    bool correct = foundBasic && foundCyclePath;
    allPassed &= correct;
    printTestResult(correct, "Graph with cycle B->C->D->B");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.link("A", "B", 1);
    g.link("B", "A", 1);
    g.link("B", "C", 1);
    g.link("C", "B", 1);

    auto paths = g.nPaths< true >("A", "C", 5);

    bool correct = containsPath< std::string >(paths, { "A", "B", "C" }) &&
                   (containsPath< std::string >(paths, { "A", "B", "A", "B", "C" }) ||
                    containsPath< std::string >(paths, { "A", "B", "C", "B", "C" }));
    allPassed &= correct;
    printTestResult(correct, "Fully connected graph with cycles");
  } {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    bool correct = g.nPaths< true >("A", "B", 2).empty();
    allPassed &= correct;
    printTestResult(correct, "Disconnected graph returns empty");
  }

  printTestResult(allPassed, "TOTAL");
}

int main()
{
  testNodeInsertion();
  testLinkOperations();
  testCycleRemoval();
  testCopyAndMoveSemantics();
  testRemovalOperations();
  testDijkstraAlgorithm();
  testNoCyclesPaths();
  testWithCyclesPaths();
  return 0;
}
