#ifndef BENCHMARK_H
#define BENCHMARK_H
#include <chrono>
#include <iostream>
#include "../graph.h"

ohantsev::Graph< int > createGraph(int n)
{
  ohantsev::Graph< int > g;
  for (int i = 0; i < n; ++i)
  {
    g.insert(i);
  }
  for (int i = 1; i < n; ++i)
  {
    g.link(i - 1, i, 1);
  }
  for (int i = 0; i < n / 2; ++i)
  {
    g.link(rand() % n, rand() % n, rand() % 100 + 1);
  }
  return g;
}

void run_benchmark()
{
  setlocale(LC_ALL, "ru");
  for (int n = 5; n <= 25; n+=1)
  {
    std::cout << "=== Размер графа: " << n << " вершин ===\n";
    auto g = createGraph(n);
    auto start = std::chrono::high_resolution_clock::now();
    auto ways = g.nPaths< true >(0, n - 1, 5);
    auto end = std::chrono::high_resolution_clock::now();
    auto nPaths_time = std::chrono::duration_cast< std::chrono::microseconds >(end - start);
    std::cout << "Пути: " << nPaths_time.count() << " мкс" << std::endl;
  }

  for (int n = 2000; n <= 40000; n+=2000)
  {
    std::cout << "=== Размер графа: " << n << " вершин ===\n";
    auto g = createGraph(n);
    auto start = std::chrono::high_resolution_clock::now();
    g.removeCycles();
    auto end = std::chrono::high_resolution_clock::now();
    auto kruskal_time = std::chrono::duration_cast< std::chrono::microseconds >(end - start);
    std::cout << "Крускал: " << kruskal_time.count() << " мкс\n";

    start = std::chrono::high_resolution_clock::now();
    auto way = g.path(0, n - 1);
    end = std::chrono::high_resolution_clock::now();
    auto dijkstra_time = std::chrono::duration_cast< std::chrono::microseconds >(end - start);
    std::cout << "Дейкстра: " << dijkstra_time.count() << " мкс\n";

    start = std::chrono::high_resolution_clock::now();
    auto waysNC = g.nPaths< false >(0, n - 1, 5);
    end = std::chrono::high_resolution_clock::now();
    auto nPathsNC_time = std::chrono::duration_cast< std::chrono::microseconds >(end - start);
    std::cout << "Пути без циклов: " << nPathsNC_time.count() << " мкс" << std::endl;
  }
}
#endif
