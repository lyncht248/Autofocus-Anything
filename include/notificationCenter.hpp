// NotificationCenter.hpp
#include <functional>
#include <queue>
#include <map>
#include <string>
#include <vector>
#include <gtkmm.h>

class NotificationCenter {
public:
    static NotificationCenter& instance() {
        static NotificationCenter instance;
        return instance;
    }

    void registerListener(const std::string& notification, std::function<void()> listener) {
        listeners[notification].push_back(listener);
    }

    void postNotification(const std::string& notification) {
        notifications.push(notification);
        g_idle_add(&NotificationCenter::dispatchNotificationsStatic, this);
    }

private:
    NotificationCenter() = default;

    static gboolean dispatchNotificationsStatic(gpointer self) {
        return ((NotificationCenter*)self)->dispatchNotifications();
    }

    gboolean dispatchNotifications() {
        while (!notifications.empty()) {
            std::string notification = notifications.front();
            notifications.pop();
            for (auto& listener : listeners[notification]) {
                listener();
            }
        }
        return G_SOURCE_REMOVE;
    }

    std::queue<std::string> notifications;
    std::map<std::string, std::vector<std::function<void()>>> listeners;
};