#include "common/node/node.hpp"
#include <cstring>

Node::Node(const std::string& node_name)
    : node_name_(node_name)
    , param_mutex_(PTHREAD_MUTEX_INITIALIZER)
    , dds_(node_name)
{
}

Node::~Node() {
    stop_timer();
    stop();
}

// THREAD
int Node::spin() {
    thread_active_ = true;
    return pthread_create(&thread_, nullptr, thread_entry_, this);
}

void Node::stop() {
    if (thread_active_) {
        pthread_cancel(thread_);
        pthread_join(thread_, nullptr);
        thread_active_ = false;
    }
}

void* Node::thread_entry_(void* arg) {
    Node* node = static_cast<Node*>(arg);
    node->run_();
    return nullptr;
}

void Node::run_() {
    while (thread_active_) {
        // timer overruns are checked in timer_handler_

        // check subscriptions
        // for (auto& subscription : subscriptions_) {
        //     subscription->process_message();
        // }
    }
}

// TIMER
bool Node::create_timer(uint32_t period_ms, void (*callback)(Node*)) {
    if (timer_active_) {
        return false;
    }

    struct sigevent sev;
    memset(&sev, 0, sizeof(struct sigevent));
    
    // TODO: Check if SIGEV_THREAD is supported in zephyr
    sev.sigev_notify = SIGEV_THREAD;
    sev.sigev_signo = SIGALRM;
    sev.sigev_value.sival_ptr = this;
    sev.sigev_notify_function = timer_handler_;

    if (int ret = timer_create(CLOCK_REALTIME, &sev, &timer_id_); ret < 0) {
        fprintf(stderr, "Node: %s timer creation failed: %s\n", node_name_.c_str(), strerror(errno));
        return false;
    }
    fprintf(stderr, "Node: %s timer has been created\n", node_name_.c_str());
    
    struct itimerspec its;
    its.it_value.tv_sec  = period_ms / 1000;
    its.it_value.tv_nsec = (period_ms % 1000) * 1000000;
    its.it_interval.tv_sec  = period_ms / 1000;
    its.it_interval.tv_nsec = (period_ms % 1000) * 1000000;
    
    if (int ret = timer_settime(timer_id_, 0, &its, nullptr); ret < 0) {
        fprintf(stderr, "Node: %s timer settime failed: %s\n", node_name_.c_str(), strerror(errno));
        timer_delete(timer_id_);
        return false;
    }
    timer_callback_ = callback;
    timer_active_ = true;

    fprintf(stderr, "Node: %s timer has been set\n", node_name_.c_str());
    return true;
}

void Node::stop_timer() {
    if (timer_active_) {
        timer_delete(timer_id_);
        timer_active_ = false;
    }
}

void Node::timer_handler_(union sigval val) {
    Node* node = static_cast<Node*>(val.sival_ptr);
    if (node->thread_active_ && node->timer_active_) {
        node->timer_callback_(node);
    }
}
