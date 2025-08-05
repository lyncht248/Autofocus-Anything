/**
 * @file notificationCenter.hpp
 * @brief Notification center for managing event listeners and dispatching notifications.
 */
#include <functional>
#include <queue>
#include <map>
#include <string>
#include <vector>
#include <gtkmm.h>



/**
 * @class NotificationCenter
 * @brief Singleton class for managing notifications and listeners.
 *
 * This class allows registering listeners for specific notifications and dispatching
 * notifications to the registered listeners. It uses GTK's idle function to process
 * notifications asynchronously.
 */
class NotificationCenter {
public:
    /**
     * @brief Gets the singleton instance of NotificationCenter.
     * @return Reference to the singleton instance.
     */
    static NotificationCenter& instance() {
        static NotificationCenter instance;
        return instance;
    }

    /**
     * @brief Registers a listener for a specific notification.
     * @param notification The name of the notification.
     * @param listener The callback function to be invoked when the notification is posted.
     */
    void registerListener(const std::string& notification, std::function<void()> listener) {
        listeners[notification].push_back(listener);
    }

    /**
     * @brief Posts a notification to be dispatched to registered listeners.
     * @param notification The name of the notification to post.
     */
    void postNotification(const std::string& notification) {
        notifications.push(notification);
        g_idle_add(&NotificationCenter::dispatchNotificationsStatic, this);
    }

private:
    NotificationCenter() = default;

    /**
     * @brief Static method for dispatching notifications.
     * @param self Pointer to the NotificationCenter instance.
     * @return G_SOURCE_REMOVE to indicate the idle function should be removed.
     */
    static gboolean dispatchNotificationsStatic(gpointer self) {
        return ((NotificationCenter*)self)->dispatchNotifications();
    }

    /**
     * @brief Dispatches notifications to the registered listeners.
     * @return G_SOURCE_REMOVE to indicate the idle function should be removed.
     */
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

    std::queue<std::string> notifications; /**< Queue of notifications to be dispatched. */
    std::map<std::string, std::vector<std::function<void()>>> listeners; /**< Map of notification names to their listeners. */
};