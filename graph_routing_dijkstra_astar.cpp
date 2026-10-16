#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>

struct Edge {
    int to;
    double weight;
};

struct Point {
    double x, y;
};

class Graph {
public:
    Graph(int n) : adj(n), coords(n) {}

    void addEdge(int u, int v, double w, bool bidirectional = true) {
        adj[u].push_back({v, w});
        if (bidirectional) adj[v].push_back({u, w});
    }

    void setCoord(int node, double x, double y) {
        coords[node] = {x, y};
    }

    struct Result {
        double dist;
        std::vector<int> path;
        int expanded;
    };

    Result dijkstra(int src, int dst) {
        int n = (int)adj.size();
        std::vector<double> dist(n, std::numeric_limits<double>::infinity());
        std::vector<int> prev(n, -1);
        std::vector<bool> visited(n, false);
        dist[src] = 0;

        using P = std::pair<double,int>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
        pq.push({0, src});

        int expanded = 0;
        while (!pq.empty()) {
            auto [d, u] = pq.top(); pq.pop();
            if (visited[u]) continue;
            visited[u] = true;
            expanded++;
            if (u == dst) break;

            for (auto& e : adj[u]) {
                if (dist[u] + e.weight < dist[e.to]) {
                    dist[e.to] = dist[u] + e.weight;
                    prev[e.to] = u;
                    pq.push({dist[e.to], e.to});
                }
            }
        }

        return {dist[dst], reconstruct(prev, src, dst), expanded};
    }

    Result astar(int src, int dst) {
        int n = (int)adj.size();
        std::vector<double> gScore(n, std::numeric_limits<double>::infinity());
        std::vector<int> prev(n, -1);
        std::vector<bool> visited(n, false);
        gScore[src] = 0;

        auto heuristic = [&](int a, int b) {
            double dx = coords[a].x - coords[b].x;
            double dy = coords[a].y - coords[b].y;
            return std::sqrt(dx*dx + dy*dy);
        };

        using P = std::pair<double,int>; // fScore, node
        std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
        pq.push({heuristic(src, dst), src});

        int expanded = 0;
        while (!pq.empty()) {
            auto [f, u] = pq.top(); pq.pop();
            if (visited[u]) continue;
            visited[u] = true;
            expanded++;
            if (u == dst) break;

            for (auto& e : adj[u]) {
                double tentative = gScore[u] + e.weight;
                if (tentative < gScore[e.to]) {
                    gScore[e.to] = tentative;
                    prev[e.to] = u;
                    pq.push({tentative + heuristic(e.to, dst), e.to});
                }
            }
        }

        return {gScore[dst], reconstruct(prev, src, dst), expanded};
    }

private:
    std::vector<int> reconstruct(std::vector<int>& prev, int src, int dst) {
        std::vector<int> path;
        for (int at = dst; at != -1; at = prev[at]) path.push_back(at);
        std::reverse(path.begin(), path.end());
        if (path.empty() || path[0] != src) return {};
        return path;
    }

    std::vector<std::vector<Edge>> adj;
    std::vector<Point> coords;
};

int main() {
    // 10x10 grid graph, coordinates used for the A* heuristic
    int rows = 25, cols = 25;
    Graph g(rows * cols);
    auto id = [&](int r, int c) { return r * cols + c; };

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            g.setCoord(id(r, c), c, r);
            if (c + 1 < cols) g.addEdge(id(r, c), id(r, c + 1), 1.0);
            if (r + 1 < rows) g.addEdge(id(r, c), id(r + 1, c), 1.0);
        }
    }

    int src = id(12, 12);
    int dst = id(12, 20);

    auto dres = g.dijkstra(src, dst);
    auto ares = g.astar(src, dst);

    std::cout << "Dijkstra: dist=" << dres.dist << "  nodes expanded=" << dres.expanded << "\n";
    std::cout << "A*:       dist=" << ares.dist << "  nodes expanded=" << ares.expanded << "\n";

    double reduction = 100.0 * (dres.expanded - ares.expanded) / dres.expanded;
    std::cout << "A* expanded " << reduction << "% fewer nodes than Dijkstra\n";

    std::cout << "\nPath (A*): ";
    for (int n : ares.path) std::cout << n << " ";
    std::cout << "\n";

    return 0;
}
