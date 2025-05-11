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
    Graph< int > g1;
    g1.insert(1);
    g1.insert(2);
    g1.link(1, 2, 10);

    Graph< int > g2(g1); // Copy constructor

    allPassed &= (g1.size() == g2.size());
    printTestResult(g1.size() == g2.size(), "Copy constructor - size match");

    bool linksCopied = !g2.connections(1).empty() &&
                       g2.connections(1)[0].first.get() == 2;
    allPassed &= linksCopied;
    printTestResult(linksCopied, "Copy constructor - links copied");
  }

  // Test move constructor
  {
    Graph< int > g1;
    g1.insert(1);
    g1.insert(2);
    g1.link(1, 2, 10);

    size_t originalSize = g1.size();
    Graph< int > g2(std::move(g1)); // Move constructor

    allPassed &= (g2.size() == originalSize);
    printTestResult(g2.size() == originalSize, "Move constructor - size preserved");

    allPassed &= (g1.size() == 0);
    printTestResult(g1.size() == 0, "Move constructor - original empty");
  }

  // Test copy assignment
  {
    Graph< int > g1, g2;
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
    Graph< int > g1, g2;
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
  Graph< std::string > g;
  bool allPassed = true;

  g.insert("A");
  g.insert("B");
  g.insert("C");
  g.link("A", "B", 1);
  g.link("B", "C", 2);

  // Test removeLink
  allPassed &= g.removeLink("A", "B");
  printTestResult(allPassed, "Remove existing link");

  allPassed &= !g.removeLink("A", "B");
  printTestResult(!g.removeLink("A", "B"), "Can't remove non-existent link");

  bool symmetricRemoval = g.connections("B").size() == 1;
  allPassed &= symmetricRemoval;
  printTestResult(symmetricRemoval, "Link removal is symmetric");

  // Test remove
  allPassed &= !g.remove("B");
  printTestResult(!g.remove("B"), "Can't remove connected node");

  allPassed &= g.remove("A");
  printTestResult(!g.remove("A"), "Remove isolated node");

  // Test removeForce
  allPassed &= g.removeForce("B");
  printTestResult(!g.removeForce("B"), "Force remove connected node");

  allPassed &= (g.size() == 1);
  printTestResult(g.size() == 1, "Correct size after removals");

  printTestResult(allPassed, "TOTAL");
}

void testDijkstraAlgorithm()
{
  printSectionHeader("Dijkstra Algorithm");
  Graph< std::string > g;
  bool allPassed = true;

  // Простой граф (A-B-C)
  g.insert("A");
  g.insert("B");
  g.insert("C");
  g.link("A", "B", 1);
  g.link("B", "C", 2);

  // Тест 1: Простой путь
  auto path1 = g.path("A", "C");
  bool simplePathCorrect = path1.size() == 2 &&
                           path1[0].first.get() == "B" && path1[0].second == 1 &&
                           path1[1].first.get() == "C" && path1[1].second == 2;
  allPassed &= simplePathCorrect;
  printTestResult(simplePathCorrect, "Simple path A-B-C");

  // Тест 2: Путь в одну вершину
  auto path2 = g.path("A", "A");
  allPassed &= path2.empty();
  printTestResult(path2.empty(), "Empty path for A-A");

  // Граф с альтернативными путями
  g.insert("D");
  g.link("A", "D", 4);
  g.link("D", "C", 1);

  // Тест 3: Выбор оптимального пути (A-B-C вместо A-D-C)
  auto path3 = g.path("A", "C");
  bool optimalPathCorrect = path3.size() == 2 &&
                            path3[0].first.get() == "B" && path3[0].second == 1 &&
                            path3[1].first.get() == "C" && path3[1].second == 2;
  allPassed &= optimalPathCorrect;
  printTestResult(optimalPathCorrect, "Chooses optimal path A-B-C (3) vs A-D-C (5)");

  // Тест 4: Несвязный граф
  g.insert("E");
  auto path4 = g.path("A", "E");
  printTestResult(path4.empty(), "Disconnected graphn");

  // Граф с циклом
  g.link("C", "A", 3);

  // Тест 5: Корректная работа с циклами
  auto path5 = g.path("A", "C");
  bool cyclePathCorrect = path5.size() == 2 &&
                          path5[0].first.get() == "B" && path5[0].second == 1 &&
                          path5[1].first.get() == "C" && path5[1].second == 2;
  allPassed &= cyclePathCorrect;
  printTestResult(cyclePathCorrect, "Correct path in graph with cycle");

  // Больший граф для комплексного теста
  Graph< int > g2;
  for (int i = 1; i <= 5; ++i) g2.insert(i);
  g2.link(1, 2, 2);
  g2.link(1, 3, 4);
  g2.link(2, 3, 1);
  g2.link(2, 4, 7);
  g2.link(3, 4, 3);
  g2.link(3, 5, 5);
  g2.link(4, 5, 2);

  // Тест 6: Комплексный маршрут
  auto path6 = g2.path(1, 5);
  bool complexPathCorrect = path6.size() == 4 &&
                            path6[0].first.get() == 2 && path6[0].second == 2 &&
                            path6[1].first.get() == 3 && path6[1].second == 1 &&
                            path6[2].first.get() == 4 && path6[2].second == 3 &&
                            path6[3].first.get() == 5 && path6[3].second == 2;
  allPassed &= complexPathCorrect;
  printTestResult(complexPathCorrect, "Complex path 1-2-3-4-5 (total 8)");

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
  return 0;
}
