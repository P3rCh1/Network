#include <iostream>
#include <vector>
#include <string>
#include "Graph.h"

using namespace std;

void printTestResult(bool condition, const string& testName) {
  cout << "[" << (condition ? "PASS" : "FAIL") << "] " << testName << endl;
}

void printSectionHeader(const string& sectionName) {
  cout << "\n=== " << sectionName << " ===" << endl;
}

void testNodeInsertion() {
  printSectionHeader("Node Insertion Tests");

  Graph<int> g;
  bool allPassed = true;
  bool test1 = g.insert(1);
  allPassed &= test1;
  printTestResult(test1, "Add new node");
  bool test2 = !g.insert(1);
  allPassed &= test2;
  printTestResult(test2, "Reject duplicate node");
  bool test3 = (g.size() == 1);
  allPassed &= test3;
  printTestResult(test3, "Correct size after insertion");

  printTestResult(allPassed, "TOTAL: Node Insertion");
}
void testLinkOperations() {
  printSectionHeader("Link Operations Tests");

  Graph<string> g;
  g.insert("A");
  g.insert("B");
  g.insert("C");
  bool allPassed = true;
  bool test1 = g.link("A", "B", 5);
  allPassed &= test1;
  printTestResult(test1, "Create valid link");
  bool test2 = !g.link("A", "D", 3);
  allPassed &= test2;
  printTestResult(test2, "Reject link to non-existent node");
  bool test3 = !g.link("A", "A", 0);
  allPassed &= test3;
  printTestResult(test3, "Reject self-linking");
  bool test4 = (g.connections("B").size() == 1 &&
    g.connections("B")[0].first == "A");
  allPassed &= test4;
  printTestResult(test4, "Links are symmetric");
  bool test5 = !g.link("A", "B", 2);
  allPassed &= test5;
  printTestResult(test5, "Reject duplicate link");

  printTestResult(allPassed, "TOTAL: Link Operations");
}
void testExceptionHandling() {
  printSectionHeader("Exception Handling Tests");

  Graph<int> g;
  bool allPassed = true;
  try {
    auto conn = g.connections(1);
    allPassed = false;
    printTestResult(false, "Throw on connections() for non-existent node");
  }
  catch (const invalid_argument&) {
    printTestResult(true, "Throw on connections() for non-existent node");
  }
  catch (...) {
    allPassed = false;
    printTestResult(false, "Correct exception type for connections()");
  }
  bool test2 = !g.remove(1);
  allPassed &= test2;
  printTestResult(test2, "Remove non-existent node");
  g.insert(1);
  g.insert(2);
  bool test3 = !g.removeLink(1, 2);
  allPassed &= test3;
  printTestResult(test3, "Remove non-existent link");

  printTestResult(allPassed, "TOTAL: Exception Handling");
}
void testCopySemantics() {
  printSectionHeader("Copy Semantics Tests");

  Graph<int> g1;
  g1.insert(1);
  g1.insert(2);
  g1.link(1, 2, 10);
  bool allPassed = true;
  Graph<int> g2(g1);
  bool test1 = (g2.size() == 2);
  allPassed &= test1;
  printTestResult(test1, "Copy constructor - size");

  bool test2 = (g2.connections(1).size() == 1 &&
    g2.connections(1)[0].second == 10);
  allPassed &= test2;
  printTestResult(test2, "Copy constructor - links");
  Graph<int> g3;
  g3 = g1;
  bool test3 = (g3.size() == 2);
  allPassed &= test3;
  printTestResult(test3, "Assignment operator - size");

  bool test4 = (g3.connections(2).size() == 1);
  allPassed &= test4;
  printTestResult(test4, "Assignment operator - links");
  g3.link(1, 2, 5);
  bool test5 = (g1.connections(1)[0].second == 10);
  allPassed &= test5;
  printTestResult(test5, "Copy independence");

  printTestResult(allPassed, "TOTAL: Copy Semantics");
}
void testRemoveOperations() {
  printSectionHeader("Remove Operations Tests");

  Graph<string> g;
  g.insert("X");
  g.insert("Y");
  g.insert("Z");
  g.link("X", "Y", 3);
  g.link("Y", "Z", 4);
  bool allPassed = true;

  bool test1 = g.removeLink("X", "Y");
  allPassed &= test1;
  printTestResult(test1, "Remove existing link");

  bool test2 = (g.connections("X").empty() &&
    g.connections("Y").size() == 1);
  allPassed &= test2;
  printTestResult(test2, "Integrity after link removal");

  bool test3 = g.remove("X");
  allPassed &= test3;
  printTestResult(test3, "Remove node without links");

  bool test4 = g.removeForce("Y");
  allPassed &= test4;
  printTestResult(test4, "Force remove node with links");

  bool test5 = (g.connections("Z").empty());
  allPassed &= test5;
  printTestResult(test5, "Links cleanup after force remove");

  printTestResult(allPassed, "TOTAL: Remove Operations");
}

void testStressOperations() {
  printSectionHeader("Stress Tests");

  Graph<int> g;
  const int N = 1000;
  bool allPassed = true;

  for (int i = 0; i < N; ++i) {
    if (!g.insert(i)) {
      allPassed = false;
      break;
    }
  }
  printTestResult(allPassed, "Massive node insertion");

  for (int i = 0; i < N - 1; ++i) {
    if (!g.link(i, i + 1, i)) {
      allPassed = false;
      break;
    }
  }
  printTestResult(allPassed, "Massive link creation");

  bool test3 = (g.size() == N);
  allPassed &= test3;
  printTestResult(test3, "Size after massive operations");

  bool test4 = (g.connections(0).size() == 1 &&
    g.connections(N / 2).size() == 2 &&
    g.connections(N - 1).size() == 1);
  allPassed &= test4;
  printTestResult(test4, "Links verification");
  g.clear();
  bool test5 = (g.size() == 0);
  allPassed &= test5;
  printTestResult(test5, "Clear massive graph");

  printTestResult(allPassed, "TOTAL: Stress Tests");
}

int main() {
  testNodeInsertion();
  testLinkOperations();
  testExceptionHandling();
  testCopySemantics();
  testRemoveOperations();
  testStressOperations();
  return 0;
}