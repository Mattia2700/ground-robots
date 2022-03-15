#include "graph.h"

// Constructor 
Graph::Graph() 
{} 
  
// Utility function to add an edge 
void Graph::addEdge(int u, int v, int w) 
{ 
  edges.push_back({w, {u, v}}); 
  V = std::max(V, std::max(u, v));
} 
  
// Function to find MST using Kruskal's 
// MST algorithm 
std::vector<iPair> Graph::kruskalMST() 
{
  int mst_wt = 0; // Initialize result 
  std::vector<iPair> mst;
  
    // Sort edges in increasing order on basis of cost 
    std::sort(edges.begin(), edges.end()); 
  
    // Create disjoint sets 
    DisjointSets ds(V); 
  
    // Iterate through all sorted edges 
    std::vector< std::pair<int, iPair> >::iterator it; 
    for (it=edges.begin(); it!=edges.end(); it++) 
    { 
        int u = it->second.first; 
        int v = it->second.second; 
  
        int set_u = ds.find(u); 
        int set_v = ds.find(v); 
  
        // Check if the selected edge is creating 
        // a cycle or not (Cycle is created if u 
        // and v belong to same set) 
        if (set_u != set_v) 
        { 
            // Current edge will be in the MST 
            // so print it 
            //cout << u << " - " << v << endl; 
            mst.push_back(it->second);
  
            // Update MST weight 
            mst_wt += it->first; 
  
            // Merge two sets 
            ds.merge(set_u, set_v); 
        } 
    } 
  
    //return mst_wt;
    return mst; 
}
 
  
// Constructor. 
DisjointSets::DisjointSets(int n) 
{ 
        // Allocate memory 
        this->n = n; 
        //parent = new int[n+1]; 
        //rnk = new int[n+1]; 
        parent.resize(n+1);
        rnk.resize(n+1);
  
        // Initially, all vertices are in 
        // different sets and have rank 0. 
        for (int i = 0; i <= n; i++) 
        { 
            rnk[i] = 0; 
  
            //every element is parent of itself 
            parent[i] = i; 
        } 
} 

// Find the parent of a node 'u' 
// Path Compression 
int DisjointSets::find(int u) 
{ 
        /* Make the parent of the nodes in the path 
           from u--> parent[u] point to parent[u] */
        if (u != parent[u]) 
            parent[u] = find(parent[u]); 
        return parent[u]; 
} 
  
// Union by rank 
void DisjointSets::merge(int x, int y) 
{ 
        x = find(x), y = find(y); 
  
        /* Make tree with smaller height 
           a subtree of the other tree  */
        if (rnk[x] > rnk[y]) 
            parent[y] = x; 
        else // If rnk[x] <= rnk[y] 
            parent[x] = y; 
  
        if (rnk[x] == rnk[y]) 
            rnk[y]++; 
} 
  

