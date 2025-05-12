#ifndef MOCK_NOTIFICATION_CENTER_H
#define MOCK_NOTIFICATION_CENTER_H

class NotificationCenter {
public:
    static NotificationCenter& instance() {
        static NotificationCenter center;
        return center;
    }
    
    void postNotification(const char* name) {
        // Mock implementation - does nothing in tests
    }
    
private:
    NotificationCenter() = default;
};

#endif 