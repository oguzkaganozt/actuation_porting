#include "node/node.hpp"
#include <signal.h>
#include <time.h>


// Signal handler for timer
static void timer_callback (int signum, siginfo_t *si, void *context) {
    Node* node = static_cast<Node*>(context);
    if (node) {
        node->timer_callback_();
    }
}

Node::Node(const std::string& node_name)
    : node_name_(node_name)
    , timer_active_(false)
    , timer_id_(nullptr)
    , dds_(DDS(node_name))
{
    // Initialize parameters mutex
    pthread_mutex_init(&param_mutex, nullptr);
}

Node::~Node() {
    stop();
    cancel_timer();
}

int Node::spin() {
    // Cancel the thread if it is running
    int result = pthread_cancel(thread_);
    if (result != 0) {
        printk("Failed to cancel thread for node %s\n", node_name_.c_str());
        return -1;
    }

    // Create the thread
    result = pthread_create(&thread_, nullptr, thread_entry, this);
    if (result != 0) {
        printk("Failed to create thread for node %s\n", node_name_.c_str());
        return -1;
    }

    return 0;
}

void Node::stop() {
    if (pthread_cancel(thread_) == 0) {
        pthread_join(thread_, nullptr);
    }
}

bool Node::has_parameter(const std::string& name) const {
    pthread_mutex_lock(&param_mutex);
    bool exists = parameters_map_.find(name) != parameters_map_.end();
    pthread_mutex_unlock(&param_mutex);
    return exists;
}

bool Node::create_timer(uint32_t period_ms, void (*callback)(void*)) {
    if (timer_active_) {
        printk("Timer already active for node %s\n", node_name_.c_str());
        return false;
    }

    // Set up signal handler for timer
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timer_signal_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGRTMIN, &sa, nullptr) == -1) {
        printk("Failed to set up signal handler for node %s\n", node_name_.c_str());
        return false;
    }

    timer_callback_ = callback;

    // Create timer
    struct sigevent sev;
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN+1;
    sev.sigev_value.sival_ptr = nullptr;

    timer_t* timer_id = new timer_t;
    if (timer_create(CLOCK_MONOTONIC, &sev, timer_id) == -1) {
        delete timer_id;
        return false;
    }

    // Set timer period
    struct itimerspec its;
    its.it_value.tv_sec = period_ms / 1000;
    its.it_value.tv_nsec = (period_ms % 1000) * 1000000;
    its.it_interval = its.it_value;

    if (timer_settime(*timer_id, 0, &its, nullptr) == -1) {
        timer_delete(*timer_id);
        delete timer_id;
        return false;
    }

    timer_id_ = timer_id;
    timer_active_ = true;
    return true;
}

void Node::cancel_timer() {
    if (timer_active_ && timer_id_) {
        timer_delete(*timer_id_);
        delete timer_id_;
        timer_id_ = nullptr;
        timer_active_ = false;
        timer_callback_ = nullptr;
    }
}

const char* Node::get_name() const {
    return node_name_.c_str();
}

void* Node::thread_entry(void* arg) {
    Node* node = static_cast<Node*>(arg);
    if (node) {
        node->run();
    }
    else {
        printk("Node thread entry: Node pointer is nullptr\n");
        exit(-1);
    }
    return nullptr;
}
