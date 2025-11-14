#include "test_config.h"
#include <fstream>
#include <sstream>
#include <cctype>
#include <cstdlib>

using namespace std;

namespace testcfg {

static string trim(const string& s) {
    size_t l=0, r=s.size();
    while (l<r && isspace((unsigned char)s[l])) ++l;
    while (r>l && isspace((unsigned char)s[r-1])) --r;
    return s.substr(l, r-l);
}

static void parse_ini(istream& in, unordered_map<string,string>& out) {
    string line, sect;
    while (getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0]=='#' || line[0]==';') continue;
        if (line.front()=='[' && line.back()==']') { sect = trim(line.substr(1, line.size()-2)); continue; }
        auto pos = line.find('=');
        if (pos==string::npos) continue;
        string k = trim(line.substr(0,pos));
        string v = trim(line.substr(pos+1));
        string full = sect.empty()? k : (sect + "." + k);
        out[full] = v;
    }
}

static Cfg g;

const Cfg& global() {
    static bool inited=false;
    if (!inited) {
        const char* p = ::getenv("TEST_CFG");
        string path = p? p : "tests/utils/test_config.ini";
        ifstream fin(path);
        if (fin.good()) parse_ini(fin, g.kv);
        inited = true;
    }
    return g;
}

string Cfg::get(const string& k, const string& defv) const {
    auto it = kv.find(k); return it==kv.end()? defv : it->second;
}
int Cfg::geti(const string& k, int defv) const {
    auto it = kv.find(k); return it==kv.end()? defv : stoi(it->second);
}
int64_t Cfg::geti64(const string& k, int64_t defv) const {
    auto it = kv.find(k); return it==kv.end()? defv : stoll(it->second);
}
double Cfg::getd(const string& k, double defv) const {
    auto it = kv.find(k); return it==kv.end()? defv : stod(it->second);
}
bool Cfg::getb(const string& k, bool defv) const {
    auto it = kv.find(k); if (it==kv.end()) return defv;
    string v = it->second; for (auto& c: v) c=tolower(c);
    return (v=="1"||v=="true"||v=="yes"||v=="on");
}

vector<vector<string>> read_csv(const string& path, char sep) {
    vector<vector<string>> rows;
    ifstream fin(path);
    if (!fin.good()) return rows;
    string line;
    while (getline(fin, line)) {
        vector<string> cols;
        string cur; bool inq=false;
        for (size_t i=0;i<line.size();++i) {
            char c=line[i];
            if (c=='"') { inq=!inq; continue; }
            if (!inq && c==sep) { cols.push_back(cur); cur.clear(); }
            else cur.push_back(c);
        }
        cols.push_back(cur);
        rows.push_back(move(cols));
    }
    return rows;
}

    std::vector<factorlib::Bar> read_bars_from_cfg() {
    std::vector<factorlib::Bar> out;

    const auto& cfg = global();
    std::string path = cfg.get("global.bars_csv", "");
    if (path.empty()) {
        path = cfg.get("bars_csv", "");
    }
    if (path.empty()) {
        return out;
    }

    auto rows = read_csv(path);
    if (rows.size() < 2) return out;  // header + 至少一行数据

    for (size_t i = 1; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (r.size() < 39) continue;   // 你提供的 bars 字段一共是 40 列

        factorlib::Bar b{};
        b.instrument_id = r[0];                     // instrument_id
        b.data_time_ms  = std::stoll(r[1]);         // data_time_ms
        b.open          = std::stod(r[2]);          // open
        b.high          = std::stod(r[3]);          // high
        b.low           = std::stod(r[4]);          // low
        b.close         = std::stod(r[5]);          // close
        b.volume        = static_cast<uint64_t>(std::stoull(r[6]));  // volume
        b.turnover      = std::stod(r[7]);          // turnover
        b.interval_ms   = std::stoi(r[8]);          // interval_ms

        // 其余 9~39 列你现在的 Bar 结构没字段就先不塞，
        // 但 CSV 里的值会完整保留，后面其他因子自己通过 read_csv(path) 用原始行也行。

        out.push_back(b);
    }

    return out;
}

std::vector<factorlib::Transaction> read_transactions_from_cfg() {
    std::vector<factorlib::Transaction> out;

    const auto& cfg = global();
    std::string path = cfg.get("global.transactions_csv", "");
    if (path.empty()) {
        path = cfg.get("transactions_csv", "");
    }
    if (path.empty()) {
        return out;
    }

    auto rows = read_csv(path);
    if (rows.size() < 2) return out;

    for (size_t i = 1; i < rows.size(); ++i) {
        const auto& r = rows[i];
        if (r.size() < 17) continue;  // 你给的 transactions 一共 17 列

        factorlib::Transaction t{};
        t.instrument_id = r[0];                              // instrument_id
        t.data_time_ms  = std::stoll(r[1]);                   // data_time_ms
        t.main_seq      = static_cast<uint64_t>(std::stoull(r[2])); // main_seq
        t.price         = std::stod(r[3]);                    // price
        t.side          = std::stoi(r[4]);                    // side
        t.volume        = static_cast<uint64_t>(std::stoull(r[5])); // volume
        t.bid_no        = static_cast<uint64_t>(std::stoull(r[6])); // bid_no
        t.ask_no        = static_cast<uint64_t>(std::stoull(r[7])); // ask_no

        // 剩下 9~16 列（channel_id,...,amount）目前没直接映射到 Transaction 结构里，
        // CSV 里依然完整保留，之后你别的测试/因子可以直接用 read_csv 查看原始列。

        out.push_back(t);
    }

    return out;
}

std::vector<factorlib::QuoteDepth> read_quotes_from_cfg() {
    std::vector<factorlib::QuoteDepth> out;

    const auto& cfg = global();
    std::string path = cfg.get("global.quotes_csv", "");
    if (path.empty()) {
        path = cfg.get("quotes_csv", "");
    }
    if (path.empty()) {
        return out;
    }

    auto rows = read_csv(path);
    if (rows.size() < 2) return out;

    for (size_t i = 1; i < rows.size(); ++i) {
        const auto& r = rows[i];
        // 你给的 snapshot_quotes 一共 96 列；我们至少需要到 bid_price_1
        if (r.size() < 90) continue;

        factorlib::QuoteDepth q{};
        q.instrument_id = r[0];               // instrument_id
        q.trading_day   = std::stoi(r[1]);    // trading_day
        q.data_time_ms  = std::stoll(r[2]);   // data_time_ms

        // ask_price_1 在第 6 列（索引 5）
        q.ask_price     = std::stod(r[5]);
        // bid_price_1 在第 38 列（索引 35）
        q.bid_price     = std::stod(r[35]);

        // 其余深度档、成交统计等你后面如果需要，可以在 QuoteDepth 结构里扩字段，
        // 再从 r[...] 中填充。

        q.volume   = 0;
        q.turnover = 0.0;
        out.push_back(q);
    }

    return out;
}

} // namespace testcfg
