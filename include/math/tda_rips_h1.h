#pragma once
/**
 * @file tda_rips_h1.h
 * @brief Rips 一骨架（1-skeleton）近似的一维持续同调 β1 估计（稀疏邻接 + 降采样）。
 *
 * 是否增量：◑（追加点可局部更新邻桶；删除建议分段重建）
 * 优化策略：
 *  - **降采样**：均匀网格/体素下采样至 N_max；
 *  - **稀疏邻接**：半径 ε + 网格桶，只连接同桶或相邻桶点对，边数 ~O(n)；
 *  - **β1 近似**：β1 ≈ E - V + C - T（将 2-单形用三角形计数近似扣除）。
 *
 * 说明：严格的 Rips 复形 k-单形计数复杂度高；本实现以图与三角形计数近似 2-单形，
 *      在稀疏图 / 小 ε 情形下效果良好。高阶（k≥2）同调未在此处实现。
 */
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>

namespace factorlib { namespace math {

struct GridDownSampler {
    // 将点投影到等间隔网格，保留每个网格一个代表点
    static std::vector<std::vector<double>>
    downsample(const std::vector<std::vector<double>>& pts, double cell, std::size_t Nmax=0){
        struct KeyHash {
            std::size_t operator()(const std::vector<long long>& v) const noexcept {
                std::size_t h=1469598103934665603ull;
                for (auto x: v) { h ^= std::hash<long long>{}(x); h *= 1099511628211ull; }
                return h;
            }
        };
        std::unordered_map<std::vector<long long>, std::size_t, KeyHash> first;
        std::vector<std::vector<double>> out;
        out.reserve(pts.size());
        for (std::size_t i=0;i<pts.size();++i){
            std::vector<long long> key(pts[i].size());
            for (std::size_t d=0; d<pts[i].size(); ++d) key[d] = (long long)std::floor(pts[i][d]/cell);
            if (!first.count(key)) { first[key]=i; out.push_back(pts[i]); }
            if (Nmax>0 && out.size()>=Nmax) break;
        }
        return out;
    }
};

struct SparseGraph {
    std::size_t V{0};
    std::vector<std::vector<int>> adj; // 无向图
    std::vector<std::pair<int,int>> edges;
};

inline double dist2(const std::vector<double>& a, const std::vector<double>& b){
    double s=0.0; for (std::size_t i=0;i<a.size();++i){ double d=a[i]-b[i]; s+=d*d; } return s;
}

// 网格桶稀疏邻接：仅连接同桶与 1-邻桶内半径<=ε 的点
inline SparseGraph build_sparse_graph(const std::vector<std::vector<double>>& pts, double eps){
    const double eps2 = eps*eps;
    const int D = pts.empty()?0:(int)pts[0].size();
    double cell = eps; // 以 ε 为网格步长
    // 建桶
    struct KeyHash { size_t operator()(const std::vector<long long>& v) const noexcept{
        size_t h=1469598103934665603ull; for (auto x: v){ h^=std::hash<long long>{}(x); h*=1099511628211ull;} return h; }};
    std::unordered_map<std::vector<long long>, std::vector<int>, KeyHash> buckets;
    for (int i=0;i<(int)pts.size();++i){
        std::vector<long long> key(D);
        for (int d=0; d<D; ++d) key[d]=(long long)std::floor(pts[i][d]/cell);
        buckets[key].push_back(i);
    }
    // 连接
    SparseGraph G; G.V = pts.size(); G.adj.assign(G.V, {});
    auto add_edge=[&](int u,int v){
        if (u==v) return;
        if (u>v) std::swap(u,v);
        G.edges.emplace_back(u,v);
        G.adj[u].push_back(v);
        G.adj[v].push_back(u);
    };
    // 枚举桶及邻桶（3^D 个邻域）
    std::vector<long long> dkey(D,0);
    for (auto& kv : buckets){
        const auto& key = kv.first;
        // 生成邻域偏移
        std::vector<std::vector<long long>> neigh;
        neigh.push_back(std::vector<long long>(D,0));
        // D 不大（常见 2/3 维），直接递归生成 offset ∈ {-1,0,1}^D
        std::function<void(int,std::vector<long long>&)> dfs = [&](int dim, std::vector<long long>& cur){
            if (dim==D){ neigh.push_back(cur); return; }
            for (int t=-1;t<=1;++t){ cur[dim]=key[dim]+t; dfs(dim+1,cur); }
        };
        std::vector<long long> cur(D,0); dfs(0,cur);
        // 在当前桶与邻桶内做半径连接
        for (auto& nk : neigh){
            auto it = buckets.find(nk);
            if (it==buckets.end()) continue;
            const auto& cand = it->second;
            for (int u : kv.second){
                for (int v : cand){
                    if (u>=v) continue;
                    if (dist2(pts[u],pts[v]) <= eps2) add_edge(u,v);
                }
            }
        }
    }
    return G;
}

// β1 近似：β1 ≈ E - V + C - T（C 连通分量数，T 三角形数）
inline int beta1_approx(const SparseGraph& G){
    const int V = (int)G.V;
    const int E = (int)G.edges.size();
    // C via 并查集
    std::vector<int> p(V), r(V,0); for (int i=0;i<V;++i) p[i]=i;
    std::function<int(int)> find = [&](int x){ return p[x]==x?x:p[x]=find(p[x]); };
    auto unite=[&](int a,int b){ a=find(a); b=find(b); if (a==b) return; if(r[a]<r[b]) std::swap(a,b); p[b]=a; if(r[a]==r[b]) r[a]++; };
    for (auto& e : G.edges) unite(e.first, e.second);
    int C=0; for (int i=0;i<V;++i) if (find(i)==i) C++;
    // T via 排序邻接交集计数（三角形）
    std::vector<std::vector<int>> adj = G.adj;
    for (auto& v : adj) std::sort(v.begin(), v.end());
    long long T=0;
    for (int u=0; u<V; ++u){
        for (int v : adj[u]) if (v>u){
            // 交集 adj[u]∩adj[v] 中 >v 的元素数量
            const auto& Au = adj[u];
            const auto& Av = adj[v];
            std::size_t i=0,j=0;
            while (i<Au.size() && j<Av.size()){
                if (Au[i]==u || Au[i]==v){ ++i; continue; }
                if (Av[j]==u || Av[j]==v){ ++j; continue; }
                if (Au[i]==Av[j]){ if (Au[i]>v) T++; ++i; ++j; }
                else if (Au[i]<Av[j]) ++i;
                else ++j;
            }
        }
    }
    int beta1 = E - V + C - (int)T;
    return beta1<0?0:beta1;
}

}} // namespace factorlib::math
