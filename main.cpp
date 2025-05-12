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
                   g.connections("B")[0].target_ == "A";
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
                       g2.connections(1)[0].target_ == 2;
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
                       g2.connections(1)[0].target_ == 2;
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

void testDijkstraAlgorithm()
{
  printSectionHeader("Dijkstra Algorithm");
  bool allPassed = true;

  // Тест 1: Простой линейный граф (A-B-C)
  {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.link("A", "B", 1);
    g.link("B", "C", 2);
    auto way = g.path("A", "C");
    bool correct = (way.steps_.size() == 3) && // A->B->C
                   (way.steps_[0].get() == "A") && // Старт
                   (way.steps_[1].get() == "B") && // Промежуточная
                   (way.steps_[2].get() == "C") && // Конец
                   (way.length_ == 3); // 1 + 2
    allPassed &= correct;
    printTestResult(correct, "Single shortest path (A-B-C)");
  }

  // Тест 2: Граф с двумя равнозначными путями
  {
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
                   (way.steps_[0].get() == "A") &&
                   ((way.steps_[1].get() == "B" && way.steps_[2].get() == "C") ||
                    (way.steps_[1].get() == "D" && way.steps_[2].get() == "C")) &&
                   (way.length_ == 2);
    allPassed &= correct;
    printTestResult(correct, "Multiple shortest paths (A-B-C or A-D-C)");
  }

  // Тест 3: Путь в одну вершину (A-A)
  {
    Graph< std::string > g;
    g.insert("A");

    auto way = g.path("A", "A");
    bool correct = (way.steps_.size() == 1) && // Только A
                   (way.steps_[0].get() == "A") &&
                   (way.length_ == 0);
    allPassed &= correct;
    printTestResult(correct, "Path for A-A (single node)");
  }

  // Тест 4: Несвязный граф
  {
    Graph< std::string > g;
    g.insert("A");
    g.insert("B");

    try
    {
      auto way = g.path("A", "B");
      allPassed = false;
      printTestResult(false, "Disconnected graph should throw");
    }
    catch (const std::runtime_error&)
    {
      printTestResult(true, "Disconnected graph throws exception");
    } catch (...)
    {
      allPassed = false;
      printTestResult(false, "Wrong exception type");
    }
  }

  // Тест 5: Проверка длины пути
  {
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

void testYensAlgorithm()
{
  printSectionHeader("Yen's K-Shortest Paths");
  bool allPassed = true;

  // Граф с несколькими кратчайшими путями
  {
    Graph<std::string> g;
    g.insert("A");
    g.insert("B");
    g.insert("C");
    g.insert("D");
    g.insert("E");

    g.link("A", "B", 1);
    g.link("B", "E", 1);
    g.link("A", "C", 1);
    g.link("C", "D", 1);
    g.link("D", "E", 1);
    g.link("B", "C", 1); // Кратчайшие пути: A-B-E и A-C-D-E и A-B-C-D-E

    auto paths = g.path("A", "E", 3);

    bool countCorrect = paths.size() == 3;
    printTestResult(countCorrect, "Finds 3 shortest paths");

    bool path1 = (paths[0].steps_.size() == 3 &&
                  paths[0].steps_[0].get() == "A" &&
                  paths[0].steps_[1].get() == "B" &&
                  paths[0].steps_[2].get() == "E" &&
                  paths[0].length_ == 2);

    bool path2 = (paths[1].steps_[0].get() == "A" &&
                  paths[1].steps_[1].get() == "C" &&
                  paths[1].steps_[2].get() == "D" &&
                  paths[1].steps_[3].get() == "E" &&
                  paths[1].length_ == 3);

    bool path3 = (paths[2].steps_.size() == 5 &&
                  paths[2].steps_[1].get() == "B" &&
                  paths[2].steps_[2].get() == "C" &&
                  paths[2].steps_[3].get() == "D" &&
                  paths[2].steps_[4].get() == "E");

    allPassed &= path1 && path2 && path3;
    printTestResult(path1, "First shortest path A-B-E");
    printTestResult(path2, "Second shortest path A-C-D-E");
    printTestResult(path3, "Third shortest path A-B-C-D-E");
  }

  // Проверка выброса исключения при отсутствии пути
  {
    Graph<std::string> g;
    g.insert("A");
    g.insert("B");

    try
    {
      auto paths = g.path("A", "B", 2);
      printTestResult(false, "No path should throw");
      allPassed = false;
    }
    catch (const std::runtime_error&)
    {
      printTestResult(true, "Throws exception if no path exists");
    }
    catch (...)
    {
      printTestResult(false, "Wrong exception type");
      allPassed = false;
    }
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
  testYensAlgorithm();
  return 0;
}
