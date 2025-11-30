#include "config/runtime_config.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cctype>

namespace factorlib::config {

static std::string trim(const std::string& s) {
    size_t l=0, r=s.size();
    while (l<r && std::isspace((unsigned char)s[l])) ++l;
    while (r>l && std::isspace((unsigned char)s[r-1])) --r;
    return s.substr(l, r-l);
}

static void parse_ini(std::istream& in, std::unordered_map<std::string,std::string>& out) {
    std::string line, sect;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0]=='#' || line[0]==';') continue;
        if (line.front()=='[' && line.back()==']') { sect = trim(line.substr(1, line.size()-2)); continue; }
        auto pos = line.find('=');
        if (pos==std::string::npos) continue;
        std::string k = trim(line.substr(0,pos));
        std::string v = trim(line.substr(pos+1));
        std::string full = sect.empty()? k : (sect + "." + k);
        out[full] = v;
    }
}

RuntimeConfig::RuntimeConfig() {
    const char* p = std::getenv("FACTOR_CFG");
    std::string path = p? p : "config/factor_config.ini";
    std::ifstream fin(path);
    if (fin.good()) parse_ini(fin, kv_);
}

const RuntimeConfig& RuntimeConfig::instance() {
    static RuntimeConfig inst;
    return inst;
}

std::string RuntimeConfig::get(const std::string& key, const std::string& def) const {
    auto it = kv_.find(key);
    return (it==kv_.end()) ? def : it->second;
}

int RuntimeConfig::geti(const std::string& key, int def) const {
    auto it = kv_.find(key); return it==kv_.end()? def : std::stoi(it->second);
}
int64_t RuntimeConfig::geti64(const std::string& key, int64_t def) const {
    auto it = kv_.find(key); return it==kv_.end()? def : std::stoll(it->second);
}
double RuntimeConfig::getd(const std::string& key, double def) const {
    auto it = kv_.find(key); return it==kv_.end()? def : std::stod(it->second);
}
bool RuntimeConfig::getb(const std::string& key, bool def) const {
    auto it=kv_.find(key); if (it==kv_.end()) return def;
    std::string v = it->second; for (auto& c: v) c = (char)std::tolower((unsigned char)c);
    return (v=="1"||v=="true"||v=="yes"||v=="on");
}

} // namespace factorlib::config
