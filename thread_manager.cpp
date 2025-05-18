// ───────────────────────  thread_manager.cpp  (C++17)  ──────────────────────
/*
 *  Thread-Manager  +  One-shot Command Executor
 *  ————————————————————————————————————————————————————————————————
 *  Feature-parity with the original Python script, now with
 *  bullet-proof shutdown semantics.
 *
 *  Build (Debian/Ubuntu):
 *      sudo apt install build-essential libboost-all-dev
 *      g++ -std=c++17 -O2 -pthread thread_manager.cpp \
 *          -lboost_system -lboost_thread -lboost_filesystem \
 *          -lboost_program_options -o thread_manager
 *
 *  Author: ChatGPT — 18 May 2025   |   Licence: MIT
 */
#include <boost/asio.hpp>
#include <boost/process.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/beast/core/detail/base64.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>
#include <signal.h>


namespace fs   = std::filesystem;
namespace bp   = boost::process;
namespace asio = boost::asio;
using asio::ip::tcp;
using namespace std::chrono_literals;

// ───────────────────────────  constants  ────────────────────────────
static constexpr std::size_t MAX_CONNECTIONS     = 10;
static constexpr int         IDLE_TIMEOUT_SEC    = 60;
static constexpr int         RESTART_DELAY_SEC   = 5;
static constexpr int         MISSING_RECHECK_SEC = 30;

// ────────────────────────  misc helpers / debug  ────────────────────
std::atomic_bool DEBUG_ENABLED{false};
#define DLOG(msg)                                  \
    do {                                           \
        if (DEBUG_ENABLED) {                       \
            std::cerr << "[SHUT] " << msg << '\n'; \
        }                                          \
    } while (0)

// ──────────────────────────  Broadcaster  ───────────────────────────
class Broadcaster {
public:
    using Subset = std::set<std::pair<std::string, std::string>>; // {name, stream}

    struct Listener {
        std::weak_ptr<tcp::socket> sock;
        std::variant<Subset, std::monostate> subs; // monostate == ALL
    };

    void addListener(const std::shared_ptr<tcp::socket>& s, Subset subs) {
        std::lock_guard<std::mutex> lk(mu_);
        Listener l;
        l.sock  = s;
        l.subs  = std::variant<Subset, std::monostate>{std::move(subs)};
        map_[s.get()] = std::move(l);
    }
    void addListenerAll(const std::shared_ptr<tcp::socket>& s) {
        std::lock_guard<std::mutex> lk(mu_);
        Listener l;
        l.sock = s;
        l.subs = std::monostate{};
        map_[s.get()] = std::move(l);
    }
    void remove(const tcp::socket* s) {
        std::lock_guard<std::mutex> lk(mu_);
        map_.erase(s);
    }

    void publish(const std::string& name,
                 const std::string& stream,
                 const std::string& line)
    {
        const std::string payload = name + ":" + line;
        std::vector<std::shared_ptr<tcp::socket>> todo;

        {
            std::lock_guard<std::mutex> lk(mu_);
            for (auto it = map_.begin(); it != map_.end();) {
                auto& lst = it->second;
                if (auto sp = lst.sock.lock()) {
                    bool sendIt = std::holds_alternative<std::monostate>(lst.subs);
                    if (!sendIt) {
                        const auto& ss = std::get<Subset>(lst.subs);
                        sendIt = ss.count({name, stream}) > 0;
                    }
                    if (sendIt) todo.push_back(sp);
                    ++it;
                } else {
                    it = map_.erase(it);           // dead socket
                }
            }
        }

        for (auto& s : todo) {
            asio::post(s->get_executor(),
                       [s, payload]{ boost::system::error_code ec;
                                      asio::write(*s, asio::buffer(payload), ec); });
        }
        sent_.fetch_add(payload.size() * todo.size(), std::memory_order_relaxed);
        if (todo.empty())
            dropped_.fetch_add(payload.size(), std::memory_order_relaxed);
    }

    std::uint64_t sentBytes()    const { return sent_.load();    }
    std::uint64_t droppedBytes() const { return dropped_.load(); }

private:
    std::mutex mu_;
    std::unordered_map<const tcp::socket*, Listener> map_;
    std::atomic_uint64_t sent_{0}, dropped_{0};
};
static Broadcaster BROADCAST;

// ─────────────────────────  ManagedProcess  ─────────────────────────
class ManagedProcess {
    using clk     = std::chrono::steady_clock;
    using tpnt    = clk::time_point;

public:
    ManagedProcess(std::string n, std::string cmd, bool dbg)
        : name_(std::move(n)), cmd_(std::move(cmd)), debug_(dbg),
          lastMissing_(clk::now()) {}

    void start() {
        std::lock_guard<std::mutex> lk(mu_);
        if (missing_ && secsSince(lastMissing_) < MISSING_RECHECK_SEC) return;

        try {
            child_ = std::make_unique<bp::child>(
                         bp::search_path("sh"), "-c", cmd_,
                         bp::std_out > out_, bp::std_err > err_);
        }
        catch (const boost::process::process_error& e) {
            missing_ = true; lastMissing_ = clk::now();
            std::cerr << "[MISSING] " << name_ << ": " << e.what() << '\n';
            return;
        }
        alive_ = true; missing_ = false;
        if (debug_) std::cerr << "[START] " << name_
                              << " pid=" << child_->id()
                              << "\n        cmd: " << cmd_ << "\n";

        thOut_ = std::thread(&ManagedProcess::pump, this, std::ref(out_), "stdout");
        thErr_ = std::thread(&ManagedProcess::pump, this, std::ref(err_), "stderr");
    }

    void stop(bool forceKill = false) {
        std::lock_guard<std::mutex> lk(mu_);
        if (!child_) return;

        if (child_->running()) {
            child_->terminate();                    // SIGTERM
            if (!child_->wait_for(3s) && forceKill) {
            #ifdef BOOST_PROCESS_POSIX_API        // Linux, *BSD, macOS
                            ::kill(child_->id(), SIGKILL);     // hard-kill
                            child_->wait();
            #else                                 // Windows: fall back to TerminateProcess
                            child_->terminate();               // best we can do
            #endif
             }
        }

        // Close pipes so pump threads unblock even if child misbehaves
        try { out_.pipe().close(); } catch (...) {}
        try { err_.pipe().close(); } catch (...) {}

        if (thOut_.joinable()) thOut_.join();
        if (thErr_.joinable()) thErr_.join();

        alive_ = false;
        DLOG("    • " << name_ << " stopped");
    }

    bool          isRunning() const { return alive_ && child_ && child_->running(); }
    std::string   status()    const { return missing_ ? "missing"
                                          : (isRunning() ? "running" : "error"); }
    const std::string& cmd()  const { return cmd_;  }
    const std::string& name() const { return name_; }

    std::atomic_uint32_t restarts{0};

private:
    static long secsSince(tpnt t) {
        return std::chrono::duration_cast<std::chrono::seconds>(clk::now()-t).count();
    }
    void pump(bp::ipstream& is, const std::string& which) {
        std::string line;
        while (std::getline(is, line)) BROADCAST.publish(name_, which, line + "\n");
    }

    std::string name_, cmd_;
    bool        debug_;

    mutable std::mutex mu_;
    std::unique_ptr<bp::child> child_;
    bp::ipstream out_, err_;
    std::thread  thOut_, thErr_;

    std::atomic_bool alive_{false}, missing_{false};
    tpnt            lastMissing_;
};

// ──────────────────────────  CommandSet  ────────────────────────────
class CommandSet {
public:
    explicit CommandSet(const std::vector<std::pair<std::string,std::string>>& v){
        for (auto& [n,c]: v) { order_.push_back(n); map_.emplace(n,c); }
    }
    const std::vector<std::string>& list() const { return order_; }
    std::string tokenToName(const std::string& tok) const {
        if (std::all_of(tok.begin(), tok.end(), ::isdigit)) {
            int i = std::stoi(tok); if (i>=1 && i<=int(order_.size())) return order_[i-1];
        }
        return tok;
    }
    const std::string* get(const std::string& n) const {
        auto it = map_.find(n); return it==map_.end()?nullptr:&it->second;
    }
private:
    std::vector<std::string> order_;
    std::unordered_map<std::string,std::string> map_;
};

// ─────────────────────────  ThreadManager  ─────────────────────────
class ThreadManager {
public:
    ThreadManager(std::vector<std::unique_ptr<ManagedProcess>> ps,
                  std::vector<std::string> order)
        : order_(std::move(order))
    {
        for (auto& p: ps) map_.emplace(p->name(), std::move(p));
    }

    void startAll() {
        for (auto& [_,p]: map_) p->start();
        mon_ = std::thread(&ThreadManager::loop, this);
    }
    void stopAll() {
        for (auto& [_,p]: map_) p->stop(true);
        quit_ = true;  if (mon_.joinable()) mon_.join();
    }

    const std::vector<std::string>& names() const { return order_; }
    std::string tok2name(const std::string& t) const {
        if (std::all_of(t.begin(), t.end(), ::isdigit)) {
            int i=std::stoi(t); if (i>=1 && i<=int(order_.size())) return order_[i-1];
        }
        return t;
    }
    std::unordered_map<std::string,std::string> status(std::optional<std::string> n={}) {
        std::unordered_map<std::string,std::string> r;
        if (n) { auto* p=find(*n); r[*n]=p?p->status():"unknown"; }
        else   for (auto& [k,p]: map_) r[k]=p->status();
        return r;
    }
    std::unordered_map<std::string,std::string> info(std::optional<std::string> n={}) {
        std::unordered_map<std::string,std::string> r;
        if (n) { auto* p=find(*n); r[*n]=p?p->cmd():"unknown"; }
        else   for (auto& [k,p]: map_) r[k]=p->cmd();
        return r;
    }
    int running()  const { int c=0; for (auto&[_,p]:map_) if(p->isRunning()) ++c; return c; }
    int restarts() const { int s=0; for (auto&[_,p]:map_) s += p->restarts.load(); return s; }

private:
    ManagedProcess* find(const std::string& n){
        auto it=map_.find(n); return it==map_.end()?nullptr:it->second.get();
    }
    void loop() {
        while (!quit_) {
            std::this_thread::sleep_for(1s);
            for (auto& [_,p]: map_) {
                if (quit_) break;
                if (!p->isRunning() && p->status()!="missing") {
                    std::this_thread::sleep_for( std::chrono::seconds(RESTART_DELAY_SEC) );
                    p->restarts++; p->start();
                } else if (p->status()=="missing") {
                    p->start();
                }
            }
        }
    }
    std::unordered_map<std::string,std::unique_ptr<ManagedProcess>> map_;
    std::vector<std::string> order_;
    std::thread mon_;
    std::atomic_bool quit_{false};
};

// ───────────────────────────  help text  ───────────────────────────
static constexpr const char* HELP_TEXT =
"list                         – show thread names\n"
"list status                  – status of all threads\n"
"list info                    – command line of all threads\n"
"list stream all              – stream all stdout+stderr\n"
"list commands                – list one-shot commands\n"
"status <thread|index>        – status of one thread\n"
"info <thread|index>          – command line of one thread\n"
"stream <thread|index> (stdout|stderr|all)\n"
"exec   <command|index>       – run one-shot command\n"
"help                         – this message\n";

// ───────────────────────────  TCP session  ─────────────────────────
class Session : public std::enable_shared_from_this<Session> {
public:
    Session(tcp::socket s, ThreadManager& tm, CommandSet& cs)
        : sock_(std::move(s)), idle_(sock_.get_executor()),
          TM_(tm), CS_(cs)
    { idle_.expires_after(std::chrono::seconds(IDLE_TIMEOUT_SEC)); }

    void start() { read(); tickIdle(); }

private:
    void send(const std::string& s) {
        auto self = shared_from_this();
        asio::async_write(sock_, asio::buffer(s),
                          [self](auto,auto){});
    }
    void read() {
        auto self = shared_from_this();
        asio::async_read_until(sock_, buf_, "\n",
            [this,self](boost::system::error_code ec,std::size_t n){
                if (ec){ BROADCAST.remove(&sock_); return; }
                std::string line(asio::buffers_begin(buf_.data()),
                                 asio::buffers_begin(buf_.data())+n);
                buf_.consume(n);
                boost::trim(line);
                handle(line);
                read();
            });
    }
    void tickIdle() {
        auto self=shared_from_this();
        idle_.async_wait([this,self](boost::system::error_code ec){
            if (!ec){ BROADCAST.remove(&sock_);
                      boost::system::error_code ignore;
                      sock_.shutdown(tcp::socket::shutdown_both,ignore);
                      sock_.close(ignore); }
        });
    }
    void resetIdle(){ idle_.expires_after(std::chrono::seconds(IDLE_TIMEOUT_SEC)); }

    // ───── command parser (mirrors Python) ─────
    void handle(const std::string& ln){
        resetIdle();
        std::vector<std::string> p; boost::split(p,ln,boost::is_any_of(" \t"),boost::token_compress_on);
        if(p.empty()) return;
        std::string cmd=boost::algorithm::to_lower_copy(p[0]);

        if(cmd=="help"){ send(HELP_TEXT); return; }

        if(cmd=="list"){
            if(p.size()==1){ for(auto& n:TM_.names()) send(n+"\n"); }
            else if(p[1]=="status"){ for(auto&[k,v]:TM_.status()) send(k+": "+v+"\n"); }
            else if(p[1]=="info"){ for(auto&[k,v]:TM_.info()) send(k+": "+v+"\n"); }
            else if(p[1]=="commands"){ for(auto& n:CS_.list()) send(n+"\n"); }
            else if(p.size()>2 && p[1]=="stream" && p[2]=="all"){
                BROADCAST.addListenerAll(sockPtr()); send("STREAM ALL\n");
            } else send("ERR Unknown list subcommand\n");
            return;
        }

        if((cmd=="status"||cmd=="info") && p.size()==2){
            std::string n=TM_.tok2name(p[1]);
            auto d=(cmd=="status")?TM_.status(n):TM_.info(n);
            send(d.begin()->first+": "+d.begin()->second+"\n"); return;
        }

        if(cmd=="stream"&&p.size()==3){
            std::string tok=p[1], spec=p[2];
            static const std::set<std::string> val={"stdout","stderr","all"};
            if(!val.count(spec)) std::swap(tok,spec);
            if(!val.count(spec)){ send("ERR stream <thread|index> <stdout|stderr|all>\n"); return; }
            std::string name=TM_.tok2name(tok);
            if(!TM_.status(name).size()){ send("ERR No such thread\n"); return; }
            Broadcaster::Subset subs = (spec=="all") ?
                  Broadcaster::Subset{{name,"stdout"},{name,"stderr"}}
                : Broadcaster::Subset{{name,spec}};
            BROADCAST.addListener(sockPtr(), std::move(subs));
            send("STREAM "+name+" "+spec+"\n"); return;
        }

        if(cmd=="exec" && p.size()==2){
            std::string name=CS_.tokenToName(p[1]);
            auto* cm=CS_.get(name);
            if(!cm){ send("ERR No such command\n"); return; }
            try{
                bp::ipstream is;
                bp::child c(bp::search_path("sh"),"-c",*cm, bp::std_out>is);
                std::string out,ln; while(std::getline(is,ln)) out+=ln+"\n";
                c.wait();
                if(!out.empty()) send(out);
                send("EXIT "+std::to_string(c.exit_code())+"\n");
            }catch(std::exception&e){ send(std::string("ERR exec failed: ")+e.what()+"\n"); }
            return;
        }

        send("ERR Unknown command\n");
    }

    std::shared_ptr<tcp::socket> sockPtr(){
        return std::shared_ptr<tcp::socket>(shared_from_this(), &sock_);
    }

    tcp::socket        sock_;
    asio::streambuf    buf_;
    asio::steady_timer idle_;
    ThreadManager&     TM_;
    CommandSet&        CS_;
};

// ───────────────────────────  TCP server  ──────────────────────────
class Server {
public:
    Server(asio::io_context& io, uint16_t port, ThreadManager& tm, CommandSet& cs)
        : acceptor_(io,{tcp::v4(),port}), TM_(tm), CS_(cs) { accept(); }
private:
    void accept(){
        acceptor_.async_accept([this](auto ec, tcp::socket s){
            if(!ec){
                if(active_<MAX_CONNECTIONS){
                    ++active_;
                    std::make_shared<Session>(std::move(s),TM_,CS_)->start();
                }else{
                    asio::write(s,asio::buffer("ERR Too many connections\n"));
                    s.close();
                }
            }
            accept();
        });
    }
    tcp::acceptor   acceptor_;
    std::atomic_int active_{0};
    ThreadManager&  TM_; CommandSet& CS_;
};

// ───────────────────────────  debug ticker  ────────────────────────
void ticker(ThreadManager& TM){
    using namespace std::chrono_literals;
    while(true){
        std::cerr<<"RUN:"<<TM.running()
                 <<" RST:"<<TM.restarts()
                 <<" TX:"<< BROADCAST.sentBytes()
                 <<" DROP:"<<BROADCAST.droppedBytes()<<"\n";
        std::this_thread::sleep_for(1s);
    }
}

// ─────────────────────────────  helpers  ───────────────────────────
void ensureKey(const boost::property_tree::ptree& cfg, const fs::path& key){
    if(fs::exists(key)) return;
    std::string b64 = cfg.get<std::string>("settings.private_key_b64");
    std::string raw(b64.size()*3/4, '\0');
    auto decoded = boost::beast::detail::base64::decode(raw.data(),
                                                        b64.data(), b64.size());
    raw.resize(decoded.first);
    fs::create_directories(key.parent_path());
    std::ofstream(key, std::ios::binary).write(raw.data(), raw.size());
    fs::permissions(key, fs::perms::owner_read);
}

// ───────────────────────────────  main  ────────────────────────────
int main(int argc,char*argv[]){
    namespace po = boost::program_options;
    try{
        po::options_description desc("opts");
        desc.add_options()
          ("config",  po::value<std::string>()->required(),"ini config")
          ("key-path",po::value<std::string>()->required(),"private key path")
          ("port",    po::value<int>()->default_value(9500),"tcp port")
          ("debug",   po::bool_switch(),"debug")
          ("help","help");
        po::variables_map vm;
        po::store(po::parse_command_line(argc,argv,desc), vm);
        if(vm.count("help")){ std::cout<<desc; return 0; }
        po::notify(vm);

        DEBUG_ENABLED = vm["debug"].as<bool>();
        if(geteuid()!=0){ std::cerr<<"Must be run as root (sudo).\n"; return 1; }

        boost::property_tree::ptree cfg;
        boost::property_tree::ini_parser::read_ini(vm["config"].as<std::string>(), cfg);

        fs::path key=vm["key-path"].as<std::string>(); ensureKey(cfg,key);

        // threads
        std::vector<std::unique_ptr<ManagedProcess>> procs;
        std::vector<std::string> order;
        for(auto& kv: cfg.get_child("threads")){
            std::string cmd=kv.second.get_value<std::string>();
            boost::replace_all(cmd,"{priv_key_path}",key.string());
            order.push_back(kv.first);
            procs.emplace_back(std::make_unique<ManagedProcess>(kv.first,cmd,DEBUG_ENABLED));
        }
        // commands
        std::vector<std::pair<std::string,std::string>> cmdv;
        if(cfg.count("commands")){
            for(auto& kv: cfg.get_child("commands")){
                std::string cmd=kv.second.get_value<std::string>();
                boost::replace_all(cmd,"{priv_key_path}",key.string());
                cmdv.emplace_back(kv.first,cmd);
            }
        }

        CommandSet     CS(cmdv);
        ThreadManager  TM(std::move(procs), order);  TM.startAll();

        if(DEBUG_ENABLED) std::thread(ticker,std::ref(TM)).detach();

        asio::io_context io;
        Server srv(io, vm["port"].as<int>(), TM, CS);

        asio::signal_set sig(io,SIGINT,SIGTERM);
        sig.async_wait([&](auto, int){ DLOG("signal, shutting down");
                                       TM.stopAll(); io.stop(); });

        io.run();
    }
    catch(std::exception& e){ std::cerr<<"Fatal: "<<e.what()<<"\n"; return 1; }
}
