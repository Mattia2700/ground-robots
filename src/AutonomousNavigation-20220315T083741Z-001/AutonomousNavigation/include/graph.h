#pragma once

#include <vector>
#include <utility>
#include <algorithm>

// Creating shortcut for an integer pair 
typedef  std::pair<int, int> iPair; 
  
// Structure to represent a graph 
struct Graph 
{ 
  //int V, E; 
  std::vector< std::pair<int, iPair> > edges;
  int V = 0; 
  
  // Constructor 
  Graph();  

  // Utility function to add an edge 
  void addEdge(int u, int v, int w);
  
  // Function to find MST using Kruskal's 
  // MST algorithm 
  std::vector<iPair> kruskalMST(); 
}; 
  
// To represent Disjoint Sets 
struct DisjointSets 
{ 
  //int *parent, *rnk; 
  std::vector<int> parent, rnk;
  int n; 
  
  // Constructor. 
  DisjointSets(int n);
  
  // Find the parent of a node 'u' 
  // Path Compression 
  int find(int u);
  
  // Union by rank 
  void merge(int x, int y);
}; 
  

