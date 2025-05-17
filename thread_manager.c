// ─────────────────────────── thread_manager.cpp ────────────────────────────
/*
 *  Thread Manager + One-Shot Command Executor (C++17, Boost.Process + Asio)
 *
 *  Feature parity with the original Python script:
 *    • reads identical INI file   • manages N long-running processes
 *    • restart-on-crash           • streams stdout/stderr over TCP
 *    • one-shot "[commands]"      • same text-based protocol
 *    • graceful SIGINT/SIGTERM    • optional debug ticker
 *
 *  Author: ChatGPT  (2025-05-17)
 *  License: MIT
 */
#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/process.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <set>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <boost/beast/core/detail/base64.hpp> 

using namespace std::chrono_literals;
namespace fs   = std::filesystem;
namespace bp   = boost::process;
namespace asio = boost::asio;
using asio::ip::tcp;

// ────────────────────────── global constants ───────────────────────────────
constexpr std::size_t   MAX_CONNECTIONS   = 10;
constexpr unsigned int  IDLE_TIMEOUT_SEC  = 60;
constexpr unsigned int  RESTART_DELAY_SEC = 5;
constexpr unsigned int  MISSING_RECHECK_SEC = 30;

// ──────────────────────── debug helpers ─────────────────────────────────────
std::atomic_bool DEBUG_ENABLED{false};

#define DLOG(msg)                                                               \
    do {                                                                        \
        if (DEBUG_ENABLED) {                                                    \
            std::cerr << "[SHUT] " << msg << '\n' << std::flush;                \
        }                                                                       \
    } while (0)

// ───────────────────────── forward declarations ─────────────────────────────
class Broadcaster;
class ManagedProcess;
class ThreadManager;

// ───────────────────────────── Broadcaster ─────────────────────────────────-
class Broadcaster {
public:
    // Subscription key = pair(process-name, stream) or special token "ALL"
    using Subset = std::set<std::pair<std::string, std::string>>;

    struct Listener {
        std::weak_ptr<tcp::socket> sock;
        std::variant<Subset, std::monostate> subs;  // monostate == "ALL"
    };

    void addListener(const std::shared_ptr<tcp::socket>& s, Subset subs) {
        std::lock_guard lk(mu_);
        listeners_[s.get()] = Listener{s, std::move(subs)};
    }
    void addListenerAll(const std::shared_ptr<tcp::socket>& s) {
        std::lock_guard lk(mu_);
        listeners_[s.get()] = Listener{s, std::monostate{}};
    }
    void remove(const tcp::socket* s) {
        std::lock_guard lk(mu_);
        listeners_.erase(s);
    }

    void publish(const std::string& name,
                 const std::string& stream,
                 const std::string& line)
    {
        const std::string payload = name + ":" + line;
        std::vector<std::shared_ptr<tcp::socket>> toSend;

        {
            std::lock_guard lk(mu_);
            for (auto it = listeners_.begin(); it != listeners_.end();) {
                auto& item = it->second;
                if (auto sp = item.sock.lock()) {
                    bool deliver = std::holds_alternative<std::monostate>(item.subs);
                    if (!deliver) {
                        const auto& ss = std::get<Subset>(item.subs);
                        deliver = ss.count({name, stream});
                    }
                    if (deliver) toSend.push_back(sp);
                    ++it;
                } else {
                    it = listeners_.erase(it); // dead socket
                }
            }
        }
        for (auto& s : toSend) {
            asio::post(s->get_executor(), [s, payload]() {
                boost::system::error_code ec;
                asio::write(*s, asio::buffer(payload), ec);
            });
        }
        sentBytes_.fetch_add(payload.size() * toSend.size(), std::memory_order_relaxed);
        if (toSend.empty())
            droppedBytes_.fetch_add(payload.size(), std::memory_order_relaxed);
    }

    std::uint64_t sentBytes()    const { return sentBytes_.load();   }
    std::uint64_t droppedBytes() const { return droppedBytes_.load();}

private:
    std::mutex mu_;
    std::unordered_map<const tcp::socket*, Listener> listeners_;
    std::atomic_uint64_t sentBytes_{0}, droppedBytes_{0};
};
Broadcaster BROADCAST;

// ─────────────────────────── ManagedProcess ────────────────────────────────
class ManagedProcess {
public:
    ManagedProcess(std::string n, std::string cmd, bool dbg)
        : name_(std::move(n)), cmdline_(std::move(cmd)), debug_(dbg) {}

    void start() {
        std::scoped_lock lk(mu_);
        if (missingExec_ && elapsedSec(lastMissing_) < MISSING_RECHECK_SEC)
            return;
        try {
            namespace bp  = boost::process;
            using boost::asio::buffer;
            child_ = std::make_unique<bp::child>(
                bp::search_path("sh"), "-c", cmdline_,
                bp::std_out > out_, bp::std_err > err_);
        }
        catch (const boost::process::process_error& e) {
            missingExec_ = true;
            lastMissing_ = now();
            std::cerr << "[MISSING] " << name_ << ": " << e.what() << '\n';
            return;
        }
        catch (const std::exception& e) {
            std::cerr << "[ERROR] Could not start " << name_ << ": "
                      << e.what() << '\n';
            return;
        }
        alive_ = true;
        missingExec_ = false;
        if (debug_) {
            std::cerr << "[START] " << name_
                      << " pid="    << child_->id()
                      << "\n        cmd: " << cmdline_ << std::endl;
        }

        // Launch reader threads
        thOut_  = std::thread(&ManagedProcess::pump, this, std::ref(out_), "stdout");
        thErr_  = std::thread(&ManagedProcess::pump, this, std::ref(err_), "stderr");
    }

    void stop(bool kill = false) {
        std::scoped_lock lk(mu_);
        if (!child_) return;
        std::string status{"TERM OK"};

        if (child_->running()) {
            child_->terminate();
            if (kill) {
                std::this_thread::sleep_for(3s);
                if (child_->running()) {
                    child_->terminate();
                    status = "KILL";
                }
            }
        }
        DLOG("    • " << name_ << " pid " << child_->id() << " -> " << status);
        alive_ = false;
        if (thOut_.joinable())  thOut_.detach();
        if (thErr_.joinable())  thErr_.detach();
    }

    bool isRunning() const {
        return alive_ && child_ && child_->running();
    }
    std::string status() const {
        if (missingExec_)     return "missing";
        return isRunning() ? "running" : "error";
    }

    const std::string& cmd()  const { return cmdline_; }
    const std::string& name() const { return name_;    }

    std::atomic_uint32_t restarts{0};

private:
    static inline auto now() { return std::chrono::steady_clock::now(); }
    static inline long elapsedSec(auto since) {
        return std::chrono::duration_cast<std::chrono::seconds>(now() - since).count();
    }

    void pump(boost::process::ipstream& is, const std::string& which) {
        std::string line;
        while (std::getline(is, line) && !line.empty()) {
            BROADCAST.publish(name_, which, line + "\n");
        }
    }

    std::string name_;
    std::string cmdline_;
    bool debug_{false};

    mutable std::mutex mu_;
    std::unique_ptr<bp::child>  child_;
    bp::ipstream out_, err_;
    std::thread  thOut_, thErr_;
    std::atomic_bool alive_{false}, missingExec_{false};
    std::chrono::steady_clock::time_point lastMissing_{now()};
};

// ────────────────────────── CommandSet (one-shot) ───────────────────────────
class CommandSet {
public:
    explicit CommandSet(const std::vector<std::pair<std::string, std::string>>& v)
    {
        for (auto& [n, c] : v) {
            order_.push_back(n);
            map_.emplace(n, c);
        }
    }
    const std::vector<std::string>& listNames() const { return order_; }

    std::string tokenToName(const std::string& tok) const {
        if (std::all_of(tok.begin(), tok.end(), ::isdigit)) {
            int idx = std::stoi(tok);
            if (1 <= idx && idx <= (int)order_.size())
                return order_[idx - 1];
        }
        return tok;
    }
    const std::string* get(const std::string& name) const {
        auto it = map_.find(name);
        return it == map_.end() ? nullptr : &it->second;
    }

private:
    std::vector<std::string>            order_;
    std::unordered_map<std::string, std::string> map_;
};

// ───────────────────────── ThreadManager  ──────────────────────────────────
class ThreadManager {
public:
    ThreadManager(std::vector<std::unique_ptr<ManagedProcess>> procs,
                  std::vector<std::string> order)
        : order_(std::move(order))
    {
        for (auto& p : procs) {
            name2proc_.emplace(p->name(), std::move(p));
        }
    }

    void startAll() {
        for (auto& [_, p] : name2proc_) p->start();
        thMon_ = std::thread(&ThreadManager::monitorLoop, this);
    }

    void stopAll() {
        for (auto& [_, p] : name2proc_) p->stop(true);
        stop_.store(true);
        if (thMon_.joinable()) thMon_.join();
    }

    // helpers matching Python API
    const std::vector<std::string>& listNames() const { return order_; }
    std::string tokenToName(const std::string& tok) const {
        if (std::all_of(tok.begin(), tok.end(), ::isdigit)) {
            int idx = std::stoi(tok);
            if (1 <= idx && idx <= (int)order_.size())
                return order_[idx - 1];
        }
        return tok;
    }
    std::unordered_map<std::string, std::string> status(const std::optional<std::string>& n = {})
    {
        std::unordered_map<std::string, std::string> out;
        if (n) {
            const auto* p = find(*n);
            out[*n] = p ? p->status() : "unknown";
        } else {
            for (auto& [name, p] : name2proc_)
                out[name] = p->status();
        }
        return out;
    }
    std::unordered_map<std::string, std::string> info(const std::optional<std::string>& n = {})
    {
        std::unordered_map<std::string, std::string> out;
        if (n) {
            const auto* p = find(*n);
            out[*n] = p ? p->cmd() : "unknown";
        } else {
            for (auto& [name, p] : name2proc_)
                out[name] = p->cmd();
        }
        return out;
    }
    int runningCount() const {
        int c = 0;
        for (auto& [_, p] : name2proc_) if (p->isRunning()) ++c;
        return c;
    }
    int restartsTotal() const {
        int sum = 0;
        for (auto& [_, p] : name2proc_) sum += p->restarts.load();
        return sum;
    }

private:
    ManagedProcess* find(const std::string& name) {
        auto it = name2proc_.find(name);
        return it == name2proc_.end() ? nullptr : it->second.get();
    }

    void monitorLoop() {
        while (!stop_) {
            std::this_thread::sleep_for(1s);
            for (auto& [name, p] : name2proc_) {
                if (stop_) break;
                if (!p->isRunning() && !p->status().starts_with("missing")) {
                    std::this_thread::sleep_for(std::chrono::seconds(RESTART_DELAY_SEC));
                    p->restarts++;
                    p->start();
                } else if (p->status() == "missing") {
                    // re-check every MISSING_RECHECK_SEC done in start()
                    p->start();
                }
            }
        }
    }
    std::unordered_map<std::string, std::unique_ptr<ManagedProcess>> name2proc_;
    std::vector<std::string> order_;
    std::thread thMon_;
    std::atomic_bool stop_{false};
};

// ──────────────────────── help text (unchanged) ────────────────────────────
constexpr const char* HELP_TEXT =
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

// ───────────────────────── TCP Session class ───────────────────────────────
class Session : public std::enable_shared_from_this<Session> {
public:
    Session(tcp::socket sock,
            ThreadManager& tm,
            CommandSet& cs)
        : socket_(std::move(sock)),
          idleTimer_(socket_.get_executor()),
          TM_(tm), CS_(cs)
    {
        idleTimer_.expires_after(std::chrono::seconds(IDLE_TIMEOUT_SEC));
    }

    void start() {
        readLine();
        checkIdle();
    }

private:
    void readLine() {
        auto self = shared_from_this();
        asio::async_read_until(socket_, buf_, "\n",
            [this, self](boost::system::error_code ec, std::size_t n) {
                if (!ec) {
                    std::string line(boost::asio::buffers_begin(buf_.data()),
                                     boost::asio::buffers_begin(buf_.data()) + n);
                    buf_.consume(n);
                    boost::algorithm::trim(line);
                    handleCommand(line);
                    readLine();
                } else {
                    BROADCAST.remove(&socket_);
                }
            });
    }

    void send(const std::string& s) {
        auto self = shared_from_this();
        asio::async_write(socket_, asio::buffer(s),
            [self](boost::system::error_code, std::size_t) {});
    }

    void checkIdle() {
        auto self = shared_from_this();
        idleTimer_.async_wait([this, self](boost::system::error_code ec) {
            if (!ec) {
                BROADCAST.remove(&socket_);
                boost::system::error_code ignore;
                socket_.shutdown(tcp::socket::shutdown_both, ignore);
                socket_.close(ignore);
            }
        });
    }

    // ─────────── command parser (mini, but mirrors Python logic) ────────────
    void handleCommand(const std::string& line) {
        idleTimer_.expires_after(std::chrono::seconds(IDLE_TIMEOUT_SEC));
        std::vector<std::string> parts;
        boost::algorithm::split(parts, line, boost::is_any_of(" \t"),
                                boost::token_compress_on);
        if (parts.empty()) return;
        std::string cmd = boost::algorithm::to_lower_copy(parts[0]);

        if (cmd == "help") { send(HELP_TEXT); return; }

        if (cmd == "list") {
            if (parts.size() == 1) {
                for (auto& n : TM_.listNames()) send(n + "\n");
            } else if (parts[1] == "status") {
                for (auto& [k,v]: TM_.status()) send(k + ": " + v + "\n");
            } else if (parts[1] == "info") {
                for (auto& [k,v]: TM_.info()) send(k + ": " + v + "\n");
            } else if (parts[1] == "commands") {
                for (auto& n : CS_.listNames()) send(n + "\n");
            } else if (parts.size()>2 && parts[1]=="stream" && parts[2]=="all") {
                BROADCAST.addListenerAll(socketPtr());
                send("STREAM ALL\n");
            } else send("ERR Unknown list subcommand\n");
            return;
        }

        if ((cmd=="status"||cmd=="info") && parts.size()==2) {
            std::string name = TM_.tokenToName(parts[1]);
            auto dict = (cmd=="status") ? TM_.status(name) : TM_.info(name);
            auto& [k,v] = *dict.begin();
            send(k + ": " + v + "\n");
            return;
        }

        if (cmd=="stream" && parts.size()==3) {
            std::string token = parts[1], spec = parts[2];
            static const std::set<std::string> valid = {"stdout","stderr","all"};
            if (!valid.count(spec)) std::swap(token,spec);
            if (!valid.count(spec)) { send("ERR stream <thread|index> <stdout|stderr|all>\n"); return; }
            std::string name = TM_.tokenToName(token);
            if (!TM_.status(name).size()) { send("ERR No such thread\n"); return; }
            Broadcaster::Subset subs =
                (spec=="all") ? Broadcaster::Subset{{name,"stdout"},{name,"stderr"}}
                              : Broadcaster::Subset{{name,spec}};
            BROADCAST.addListener(socketPtr(), std::move(subs));
            send("STREAM " + name + " " + spec + "\n");
            return;
        }

        if (cmd=="exec" && parts.size()==2) {
            std::string name = CS_.tokenToName(parts[1]);
            auto* cmline = CS_.get(name);
            if (!cmline) { send("ERR No such command\n"); return; }
            try {
                bp::ipstream is;
                bp::child c(bp::search_path("sh"), "-c", *cmline,  // ← replace this line
                bp::std_out > is);
                std::string out;
                std::string line;
                while (std::getline(is, line))
                    out += line + "\n";
                c.wait();
                if (!out.empty()) send(out);
                send("EXIT " + std::to_string(c.exit_code()) + "\n");
            }
            catch (const std::exception& e) {
                send(std::string("ERR exec failed: ") + e.what() + "\n");
            }
            return;
        }

        send("ERR Unknown command\n");
    }

    std::shared_ptr<tcp::socket> socketPtr() {
        return std::shared_ptr<tcp::socket>(shared_from_this(),
                                            &socket_); // aliasing ptr
    }

    tcp::socket                socket_;
    asio::streambuf            buf_;
    asio::steady_timer         idleTimer_;

    ThreadManager& TM_;
    CommandSet&     CS_;
};

// ─────────────────────────── TCP server  ────────────────────────────────────
class Server {
public:
    Server(asio::io_context& io, uint16_t port,
           ThreadManager& tm, CommandSet& cs)
        : acceptor_(io, {tcp::v4(), port}), TM_(tm), CS_(cs)
    {
        accept();
    }
private:
    void accept() {
        acceptor_.async_accept( [this](boost::system::error_code ec, tcp::socket s) {
            if (!ec) {
                if ((int)sessions_ < MAX_CONNECTIONS) {
                    ++sessions_;
                    std::make_shared<Session>(std::move(s), TM_, CS_)->start();
                } else {
                    asio::write(s, asio::buffer("ERR Too many connections\n"));
                    s.close();
                }
            }
            accept();
        });
    }
    tcp::acceptor   acceptor_;
    std::atomic_int sessions_{0};
    ThreadManager& TM_;
    CommandSet&    CS_;
};

// ───────────────────────── debug ticker thread ─────────────────────────────
void debugTicker(ThreadManager& TM) {
    while (true) {
        std::cerr << "RUN:"  << TM.runningCount()
                  << " RST:" << TM.restartsTotal()
                  << " TX:"  << BROADCAST.sentBytes()
                  << " DROP:"<< BROADCAST.droppedBytes() << '\n';
        std::this_thread::sleep_for(1s);
    }
}

// ─────────────────────────── config helpers ────────────────────────────────
void ensureKey(const boost::property_tree::ptree& cfg, const fs::path& keyPath)
{
    if (fs::exists(keyPath)) return;
    auto b64 = cfg.get<std::string>("settings.private_key_b64");
    std::string raw;
    raw.resize(b64.size() * 3 / 4);
    std::size_t len = boost::beast::detail::base64::decode(raw.data(), b64.data(), b64.size());
    raw.resize(len);
    fs::create_directories(keyPath.parent_path());
    std::ofstream(keyPath, std::ios::binary).write(raw.data(), raw.size());
    fs::permissions(keyPath, fs::perms::owner_read);
}

// ─────────────────────────────── main ──────────────────────────────────────
int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    try {
        po::options_description desc("options");
        desc.add_options()
            ("config",  po::value<std::string>()->required(), "ini config")
            ("key-path",po::value<std::string>()->required(), "private key path")
            ("port",    po::value<int>()->default_value(9500), "tcp port")
            ("debug",   po::bool_switch(), "debug output")
            ("help",    "help");
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) { std::cout << desc; return 0; }
        po::notify(vm);

        if (geteuid() != 0) {
            std::cerr << "Must be run as root (sudo).\n";
            return 1;
        }

        DEBUG_ENABLED = vm["debug"].as<bool>();

        // parse INI
        boost::property_tree::ptree cfg;
        boost::property_tree::ini_parser::read_ini(vm["config"].as<std::string>(), cfg);

        fs::path keyPath = vm["key-path"].as<std::string>();
        ensureKey(cfg, keyPath);

        // Build ManagedProcess objects
        std::vector<std::unique_ptr<ManagedProcess>> procs;
        std::vector<std::string> order;
        for (auto& kv : cfg.get_child("threads")) {
            std::string cmd = kv.second.get_value<std::string>();
            boost::replace_all(cmd, "{priv_key_path}", keyPath.string());
            order.push_back(kv.first);
            procs.emplace_back(std::make_unique<ManagedProcess>(kv.first, cmd, DEBUG_ENABLED));
        }

        // one-shot commands
        std::vector<std::pair<std::string,std::string>> cmdVec;
        if (cfg.count("commands")) {
            for (auto& kv : cfg.get_child("commands")) {
                std::string cmd = kv.second.get_value<std::string>();
                boost::replace_all(cmd, "{priv_key_path}", keyPath.string());
                cmdVec.emplace_back(kv.first, cmd);
            }
        }
        CommandSet CS(cmdVec);
        ThreadManager TM(std::move(procs), order);
        TM.startAll();

        if (DEBUG_ENABLED) std::thread(debugTicker, std::ref(TM)).detach();

        asio::io_context io;
        Server srv(io, vm["port"].as<int>(), TM, CS);

        // signal handling
        asio::signal_set sigs(io, SIGINT, SIGTERM);
        sigs.async_wait([&](auto, int){
            DLOG("signal received, shutting down …");
            TM.stopAll();
            io.stop();
        });

        io.run();
    }
    catch (std::exception& e) {
        std::cerr << "Fatal: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
