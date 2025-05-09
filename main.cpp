#include <iostream>
#include <vector>
#include <string>
#include "Graph.h"

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

int main()
{
  testNodeInsertion();
  testLinkOperations();
  testCycleRemoval();
  return 0;
}
