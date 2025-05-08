#include <iostream>
#include <vector>
#include <string>
#include "Graph.h"

using namespace std;

void printTestResult(bool condition, const string& testName)
{
  cout << "[" << (condition? "PASS": "FAIL") << "] " << testName << endl;
}

void printSectionHeader(const string& sectionName)
{
  cout << "\n=== " << sectionName << " ===" << endl;
}

void testNodeInsertion()
{
  printSectionHeader("Node Insertion Tests");
  Graph< int > g;
  bool allPassed = true;

  allPassed &= g.insert(1);
  printTestResult(allPassed, "Add new node");

  allPassed &= !g.insert(1);
  printTestResult(!g.insert(1), "Reject duplicate node");

  allPassed &= (g.size() == 1);
  printTestResult(g.size() == 1, "Correct size after insertion");

  printTestResult(allPassed, "TOTAL: Node Insertion");
}

void testLinkOperations()
{
  printSectionHeader("Link Operations Tests");
  Graph< string > g;
  g.insert("A");
  g.insert("B");
  g.insert("C");
  bool allPassed = true;

  allPassed &= g.link("A", "B", 5);
  printTestResult(g.link("A", "B", 5), "Create valid link");

  allPassed &= !g.link("A", "D", 3);
  printTestResult(!g.link("A", "D", 3), "Reject link to non-existent node");

  allPassed &= !g.link("A", "A", 0);
  printTestResult(!g.link("A", "A", 0), "Reject self-linking");

  bool symmetric = (g.connections("B").size() == 1 &&
                    g.connections("B")[0].first.get() == "A");
  allPassed &= symmetric;
  printTestResult(symmetric, "Links are symmetric");

  allPassed &= !g.link("A", "B", 2);
  printTestResult(!g.link("A", "B", 2), "Reject duplicate link");

  printTestResult(allPassed, "TOTAL: Link Operations");
}

bool hasConnection(const std::vector< std::pair< std::reference_wrapper< const int >, size_t > >& connections,
                   int target)
{
  for (const auto& conn: connections)
  {
    if (conn.first.get() == target) return true;
  }
  return false;
}

void testCycleRemoval()
{
  printSectionHeader("Cycle Removal Tests");

  Graph< int > g;
  bool allPassed = true;

  // 1. Тест: треугольник → должно остаться 2 ребра
  g.insert(1);
  g.insert(2);
  g.insert(3);
  g.link(1, 2, 1);
  g.link(2, 3, 2);
  g.link(3, 1, 3);

  // Запоминаем связи до удаления циклов
  auto conn1_before = g.connections(1);
  auto conn2_before = g.connections(2);
  auto conn3_before = g.connections(3);

  g.removeCycles();

  // Проверяем количество связей после
  bool test1 = g.connections(1).size() + g.connections(2).size() + g.connections(3).size() == 4;
  allPassed &= test1;
  printTestResult(test1, "Triangle → removes one edge (3→2→1)");

  // 2. Проверяем, что удалено самое тяжелое ребро (3-1)
  bool test2 = !hasConnection(g.connections(3), 1);
  allPassed &= test2;
  printTestResult(test2, "Removes heaviest edge in cycle");

  // 3. Тест: несвязный граф не меняется
  Graph< std::string > g2;
  g2.insert("A");
  g2.insert("B");
  g2.link("A", "B", 1);

  size_t before = g2.connections("A").size() + g2.connections("B").size();
  g2.removeCycles();
  size_t after = g2.connections("A").size() + g2.connections("B").size();

  bool test3 = (before == after);
  allPassed &= test3;
  printTestResult(test3, "Acyclic graph remains unchanged");

  printTestResult(allPassed, "TOTAL: Cycle Removal");
}

int main()
{
  testNodeInsertion();
  testLinkOperations();
  testCycleRemoval();
  return 0;
}
