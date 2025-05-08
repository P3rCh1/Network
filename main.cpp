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

  bool linkCreated = g.link("A", "B", 5);
  allPassed &= linkCreated;
  printTestResult(linkCreated, "Create valid link");

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
  Graph<int> g;
  bool allPassed = true;

  // 1. Создаем треугольник
  g.insert(1); g.insert(2); g.insert(3);
  g.link(1, 2, 1);
  g.link(2, 3, 2);
  g.link(3, 1, 3);

  // Вывод начального состояния
  cout << "\nInitial graph state:\n";
  for(int i = 1; i <= 3; i++) {
    cout << "Node " << i << " connected to: ";
    for(const auto& conn : g.connections(i)) {
      cout << conn.first.get() << " (w:" << conn.second << ") ";
    }
    cout << "\n";
  }

  // 2. Удаляем циклы
  g.removeCycles();

  // Вывод конечного состояния
  cout << "\nGraph after removeCycles():\n";
  int total_edges = 0;
  for(int i = 1; i <= 3; i++) {
    auto conns = g.connections(i);
    total_edges += conns.size();
    cout << "Node " << i << " connected to: ";
    for(const auto& conn : conns) {
      cout << conn.first.get() << " ";
    }
    cout << "\n";
  }
  total_edges /= 2; // Учитываем двунаправленность

  // 3. Проверки
  bool edges_ok = (total_edges == 2);
  allPassed &= edges_ok;
  printTestResult(edges_ok, "Triangle → becomes tree (2 edges remain)");
  cout << "Actual edges count: " << total_edges << "\n";

  // Проверка связности
  bool connected = (!g.connections(1).empty() &&
                   !g.connections(2).empty() &&
                   !g.connections(3).empty());
  allPassed &= connected;
  printTestResult(connected, "Graph remains connected");

  // 4. Проверка для ациклического графа
  Graph<string> g2;
  g2.insert("A"); g2.insert("B");
  g2.link("A", "B", 1);

  size_t before = g2.connections("A").size();
  g2.removeCycles();
  size_t after = g2.connections("A").size();

  bool unchanged = (before == after);
  allPassed &= unchanged;
  printTestResult(unchanged, "Acyclic graph remains unchanged");

  printTestResult(allPassed, "TOTAL: Cycle Removal");

  // Дополнительная диагностика
  if(!edges_ok) {
    cout << "\nDIAGNOSTICS:\n";
    cout << "Expected edge count: 2\n";
    cout << "Actual edge count: " << total_edges << "\n";

    // Проверяем какие конкретно ребра остались
    vector<pair<int,int>> remaining_edges;
    for(int i = 1; i <= 3; i++) {
      for(const auto& conn : g.connections(i)) {
        int j = conn.first.get();
        if(i < j) remaining_edges.emplace_back(i, j);
      }
    }

    cout << "Remaining edges: ";
    for(const auto& e : remaining_edges) {
      cout << e.first << "-" << e.second << " ";
    }
    cout << "\n";
  }
}

int main()
{
  testNodeInsertion();
  testLinkOperations();
  testCycleRemoval();
  return 0;
}
