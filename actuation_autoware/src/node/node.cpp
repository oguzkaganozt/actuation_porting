#include "node/node.hpp"

Node::Node(const std::string& node_name)
    : node_name_(node_name)
    , running_(0)
    , timer_active_(false)
{
    // Initialize parameters mutex
    k_mutex_init(&param_mutex);
    
    // Initialize timer
    k_timer_init(&node_timer_, timer_expiry_handler, nullptr);

    // Initialize DDS
    dds_ = DDS(node_name);
}

Node::~Node() {
    stop();
    
    // Stop timer if active
    if (timer_active_) {
        k_timer_stop(&node_timer_);
        timer_active_ = false;
    }
}

bool Node::has_parameter(const std::string& name) const {
    k_mutex_lock(&param_mutex, K_FOREVER);
    bool exists = parameters_map_.find(name) != parameters_map_.end();
    k_mutex_unlock(&param_mutex);
    return exists;
}

int Node::spin(int priority, int thread_sleep_ms) {
    if (atomic_get(&running_)) {
        printk("Node %s already running\n", node_name_.c_str());
        return 0;
    }

    // Create thread
    thread_id_ = k_thread_create(
        &thread_,
        stack_,
        THREAD_STACK_SIZE,
        thread_entry,
        this, nullptr, nullptr,
        priority,
        0,
        K_NO_WAIT);
    
    if (!thread_id_) {
        printk("Failed to create thread for node %s\n", node_name_.c_str());
        k_panic();
    }
    
    atomic_set(&running_, 1);
    thread_sleep_ms_ = thread_sleep_ms;
    return 0;
}

void Node::stop() {
    if (atomic_get(&running_) && thread_id_) {
        k_thread_abort(thread_id_);
        thread_id_ = nullptr;
        atomic_set(&running_, 0);
    }
}

bool Node::create_timer(uint32_t period_ms, void (*callback)(void*)) {
    if (timer_active_) {
        printk("Timer already active for node %s\n", node_name_.c_str());
        return false;
    }

    k_timer_start(&node_timer_, K_MSEC(period_ms), K_MSEC(period_ms));
    timer_active_ = true;
    
    return true;
}

void Node::stop_timer() {
    if (timer_active_) {
        k_timer_stop(&node_timer_);
        timer_active_ = false;
    }
}

const char* Node::get_name() const {
    return node_name_.c_str();
}

void Node::thread_entry(void* p1, void* p2, void* p3) {
    Node* node = static_cast<Node*>(p1);
    if (node) {
        while (atomic_get(&node->running_)) {
            // Process DDS messages
            k_msleep(node->thread_sleep_ms_);
        }
    }
}

void Node::timer_expiry_handler(struct k_timer *timer_id) {
    k_work_submit(&timer_work);
}

void Node::timer_work_handler(struct k_work *work) {

}
