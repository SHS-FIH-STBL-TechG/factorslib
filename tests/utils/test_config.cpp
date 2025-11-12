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

} // namespace testcfg
