#ifndef COMMON__ISUBSCRIPTION_HANDLER_HPP_
#define COMMON__ISUBSCRIPTION_HANDLER_HPP_

class ISubscriptionHandler {
public:
    virtual ~ISubscriptionHandler() = default;
    virtual bool is_data_available() = 0;
    virtual void process_next_message() = 0;
};

#endif // COMMON__ISUBSCRIPTION_HANDLER_HPP_ 