#ifndef MESSAGE_QUEUE_HPP
#define MESSAGE_QUEUE_HPP

#include <queue>
#include <pthread.h>

template<typename T>
class MessageQueue {
private:
    std::queue<T> queue;
    pthread_mutex_t mutex;

public:
    MessageQueue() {
        pthread_mutex_init(&mutex, NULL);
    }

    ~MessageQueue() {
        pthread_mutex_destroy(&mutex);
    }

    void push(const T& value) {
        pthread_mutex_lock(&mutex);
        queue.push(value);
        pthread_mutex_unlock(&mutex);
    }

    bool pop(T& value) {
        pthread_mutex_lock(&mutex);
        if (queue.empty()) {
            pthread_mutex_unlock(&mutex);
            return false;
        }
        value = queue.front();
        queue.pop();
        pthread_mutex_unlock(&mutex);
        return true;
    }
};

#endif // MESSAGE_QUEUE_HPP 