#include <iostream>
#include <vector>
#include <string>
#include "graph_algorithms.h"

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

  bool symmetric = !g.connections("B").empty() &&
                   g.connections("B")[0].first.get() == "A";
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
  for (const auto& node: g.nodes())
  {
    edgeCount += g.connections(node).size();
  }
  edgeCount /= 2;

  allPassed &= (edgeCount == 2);
  printTestResult(edgeCount == 2, "Removes one edge from triangle");

  bool connected = !g.connections(1).empty() &&
                   !g.connections(2).empty() &&
                   !g.connections(3).empty();
  allPassed &= connected;
  printTestResult(connected, "Graph stays connected");

  Graph< std::string > g2;
  g2.insert("A");
  g2.insert("B");
  g2.link("A", "B", 1);

  size_t before = g2.connections("A").size();
  g2.removeCycles();
  size_t after = g2.connections("A").size();

  allPassed &= (before == after);
  printTestResult(before == after, "Acyclic graph unchanged");

  printTestResult(allPassed, "TOTAL");
}

void testCopyAndMoveSemantics()
{
    printSectionHeader("Copy/Move Semantics");
    bool allPassed = true;

    // Test copy constructor
    {
        Graph<int> g1;
        g1.insert(1);
        g1.insert(2);
        g1.link(1, 2, 10);

        Graph<int> g2(g1); // Copy constructor

        allPassed &= (g1.size() == g2.size());
        printTestResult(g1.size() == g2.size(), "Copy constructor - size match");

        bool linksCopied = !g2.connections(1).empty() &&
                          g2.connections(1)[0].first.get() == 2;
        allPassed &= linksCopied;
        printTestResult(linksCopied, "Copy constructor - links copied");
    }

    // Test move constructor
    {
        Graph<int> g1;
        g1.insert(1);
        g1.insert(2);
        g1.link(1, 2, 10);

        size_t originalSize = g1.size();
        Graph<int> g2(std::move(g1)); // Move constructor

        allPassed &= (g2.size() == originalSize);
        printTestResult(g2.size() == originalSize, "Move constructor - size preserved");

        allPassed &= (g1.size() == 0);
        printTestResult(g1.size() == 0, "Move constructor - original empty");
    }

    // Test copy assignment
    {
        Graph<int> g1, g2;
        g1.insert(1);
        g1.insert(2);
        g1.link(1, 2, 10);

        g2 = g1; // Copy assignment

        allPassed &= (g1.size() == g2.size());
        printTestResult(g1.size() == g2.size(), "Copy assignment - size match");

        bool linksCopied = !g2.connections(1).empty() &&
                          g2.connections(1)[0].first.get() == 2;
        allPassed &= linksCopied;
        printTestResult(linksCopied, "Copy assignment - links copied");
    }

    // Test move assignment
    {
        Graph<int> g1, g2;
        g1.insert(1);
        g1.insert(2);
        g1.link(1, 2, 10);

        size_t originalSize = g1.size();
        g2 = std::move(g1); // Move assignment

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
    Graph<std::string> g;
    bool allPassed = true;

    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.link("A", "B", 1);
    g.link("B", "C", 2);

    // Test removeLink
    allPassed &= g.removeLink("A", "B");
    printTestResult(g.removeLink("A", "B"), "Remove existing link");

    allPassed &= !g.removeLink("A", "B");
    printTestResult(!g.removeLink("A", "B"), "Can't remove non-existent link");

    bool symmetricRemoval = g.connections("B").empty();
    allPassed &= symmetricRemoval;
    printTestResult(symmetricRemoval, "Link removal is symmetric");

    // Test remove
    allPassed &= !g.remove("B");
    printTestResult(!g.remove("B"), "Can't remove connected node");

    allPassed &= g.remove("A");
    printTestResult(g.remove("A"), "Remove isolated node");

    // Test removeForce
    allPassed &= g.removeForce("B");
    printTestResult(g.removeForce("B"), "Force remove connected node");

    allPassed &= (g.size() == 1);
    printTestResult(g.size() == 1, "Correct size after removals");

    printTestResult(allPassed, "TOTAL");
}

void testAdvancedMethods()
{
    printSectionHeader("Advanced Methods");
    Graph<int> g;
    bool allPassed = true;

    g.insert(1);
    g.insert(2);
    g.insert(3);
    g.insert(4);
    g.link(1, 2, 1);
    g.link(2, 3, 2);
    g.link(3, 4, 3);
    g.link(4, 1, 4);
    g.link(1, 3, 5);

    // Test ways()
    auto paths = g.ways(3);
    allPassed &= (paths.size() > 0);
    printTestResult(paths.size() > 0, "ways() returns some paths");

    // Test clear()
    g.clear();
    allPassed &= (g.size() == 0);
    printTestResult(g.size() == 0, "clear() empties graph");

    allPassed &= g.nodes().empty();
    printTestResult(g.nodes().empty(), "nodes() empty after clear");

    printTestResult(allPassed, "TOTAL");
}

int main()
{
    testNodeInsertion();
    testLinkOperations();
    testCycleRemoval();
    testCopyAndMoveSemantics();
    testRemovalOperations();
    testAdvancedMethods();
    return 0;
}
