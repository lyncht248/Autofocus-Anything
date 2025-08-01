/**
 * @file thread.hpp
 * @brief Header file for thread management and synchronization utilities.
 * 
 * This file defines interfaces for managing threads, including stopping threads
 * at predefined points, dispatching signals, and creating objects in dedicated threads.
 */

#ifndef HVIGTK_THREAD_H
#define HVIGTK_THREAD_H

#include "glibmm/main.h"
#include "glibmm/threads.h"
#include "sigc++/connection.h"
#include <gtkmm.h>
#include <initializer_list>
#include <queue>

/**
 * @brief Gets the default thread context for the current thread.
 * 
 * Provides access to the Glib main context associated with the current thread.
 * @return Reference to the Glib main context.
 */
Glib::RefPtr<Glib::MainContext> hvigtk_threadcontext();

/**
 * @namespace ThreadStopper
 * @brief Provides utilities for managing thread stopping points and synchronization.
 * 
 * The ThreadStopper namespace includes functions for making threads stoppable,
 * locking and unlocking mutexes, and sending stop requests to threads.
 */
namespace ThreadStopper
{
    /**
     * @brief Makes the current thread responsive to stop requests.
     * 
     * This function should be called at the start of the thread routine.
     */
    extern void makeStoppable();

    /**
     * @brief Locks a mutex and registers it for automatic unlocking on stop requests.
     * 
     * @param mutex Mutex to lock.
     */
    extern void lock(Glib::Threads::Mutex &mutex);

    /**
     * @brief Unlocks a mutex and unregisters it from automatic unlocking.
     * 
     * @param mutex Mutex to unlock.
     */
    extern void unlock(Glib::Threads::Mutex &mutex);

    /**
     * @brief Sets a stop point for the current thread.
     * 
     * Makes the thread responsive to stop requests while waiting on the optional condition.
     * @param condition Optional condition variable for synchronization.
     */
    extern void stopPoint(Glib::Threads::Cond *condition = nullptr);

    /**
     * @brief Sends stop requests to the specified threads.
     * 
     * This function also calls `join()` on each thread.
     * @param ts List of threads to stop.
     */
    extern void stop(std::initializer_list<Glib::Threads::Thread*> ts);

    /**
     * @brief Waits for a thread to finish and cleans up its data.
     * 
     * @param thread Thread to join.
     */
    extern void join(Glib::Threads::Thread *thread);
};

/**
 * @class VDispatcher
 * @brief A thread-safe signal dispatcher for asynchronous communication.
 * 
 * The VDispatcher class provides functionality to emit signals and connect
 * slots for handling dispatched values in a thread-safe manner.
 * @tparam T Type of the value to dispatch.
 */
template <typename T>
class VDispatcher
{
    friend class Connection;

    /**
     * @class Connection
     * @brief Represents a connection to a VDispatcher signal.
     * 
     * The Connection class manages the lifecycle of a signal connection,
     * including disconnection and cleanup.
     */
    class Connection
    {
        friend class VDispatcher;
    public:
        /**
         * @brief Constructs a Connection object.
         * 
         * @param dispatcher Reference to the VDispatcher.
         * @param connection Signal connection object.
         */
        Connection(VDispatcher<T> &dispatcher, const sigc::connection &connection);

        /**
         * @brief Copy constructor for Connection.
         * 
         * @param connection Connection object to copy.
         */
        Connection(const Connection &connection);

        /**
         * @brief Disconnects the signal connection.
         */
        void disconnect();

        VDispatcher<T> &dispatcher; ///< Reference to the VDispatcher.
        sigc::connection connection; ///< Signal connection object.
    };

public:
    /**
     * @brief Constructs a VDispatcher object.
     */
    VDispatcher();

    /**
     * @brief Emits a value to all connected slots.
     * 
     * @param v Value to emit.
     */
    void emit(const T &v);

    /**
     * @brief Connects a slot to the dispatcher.
     * 
     * @param slot Slot to connect.
     * @return Connection object representing the connection.
     */
    Connection connect(const sigc::slot<void(const T&)>& slot);

private:
    int nConnections; ///< Number of active connections.
    int n; ///< Counter for managing the queue.
    std::queue<T> queue; ///< Queue for storing dispatched values.
    Glib::Threads::Mutex mutex; ///< Mutex for thread-safe access.
    Glib::Dispatcher dispatch; ///< Dispatcher for emitting signals.
};

/**
 * @class ObjectThread
 * @brief Manages the creation and execution of objects in a dedicated thread.
 * 
 * The ObjectThread class provides functionality to create objects in a separate
 * thread and manage their lifecycle.
 */
class ObjectThread
{
public:
    /**
     * @brief Constructs an ObjectThread object.
     */
    ObjectThread();

    /**
     * @brief Destroys the ObjectThread object.
     */
    ~ObjectThread();

    /**
     * @brief Creates an object in the thread.
     * 
     * @tparam T Type of the object to create.
     * @tparam As Variadic template for constructor arguments.
     * @param as Constructor arguments for the object.
     * @return Pointer to the created object.
     */
    template <class T, typename... As> 
    T* createObject(As... as);

protected:
    /**
     * @brief Slot function for the thread.
     * 
     * Initializes the thread context and runs the main loop.
     */
    void slot();

    Glib::Threads::Mutex mutex; ///< Mutex for thread synchronization.
    Glib::Threads::Cond objCreated; ///< Condition variable for object creation.
    Glib::Threads::Cond started; ///< Condition variable for thread start.
    bool _started; ///< Indicates whether the thread has started.
    bool _objCreated; ///< Indicates whether the object has been created.

    Glib::RefPtr<Glib::MainLoop> loop; ///< Main loop for the thread.
    Glib::Threads::Thread *thread; ///< Pointer to the thread object.
};

#endif
