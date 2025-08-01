#ifndef HVIGTK_COND_H
#define HVIGTK_COND_H

#include <cstdint>
#include <gtkmm.h>
#include <initializer_list>
#include <unordered_map>


/**
 * @class Condition
 * @brief Represents a condition that can be toggled and emits signals based on its state.
 * 
 * Conditions are used to signal other objects when certain events occur, such as button presses
 * or interactions. They emit signals when toggled, set to true, set to false, or destroyed.
 */
class Condition
{
    friend class OpCondition;
public:
    /**
     * @brief Constructor for the Condition class.
     * 
     * Initializes the condition with a given boolean value.
     * @param val Initial value of the condition (default is false).
     */
    Condition(bool val = false);

    /**
     * @brief Destructor for the Condition class.
     * 
     * Emits the destroyed signal when the condition is destroyed.
     */
    ~Condition();

    /**
     * @brief Signal emitted when the condition is toggled.
     * 
     * @return SignalToggled object for connecting to the toggled signal.
     */
    using SignalToggled = sigc::signal<void(bool)>;
    SignalToggled signalToggled();

    /**
     * @brief Signal emitted when the condition is set to true.
     * 
     * @return SignalTrue object for connecting to the true signal.
     */
    using SignalTrue = sigc::signal<void()>;
    SignalTrue signalTrue();

    /**
     * @brief Signal emitted when the condition is set to false.
     * 
     * @return SignalFalse object for connecting to the false signal.
     */
    using SignalFalse = sigc::signal<void()>;
    SignalFalse signalFalse();

    /**
     * @brief Signal emitted when the condition is destroyed.
     * 
     * @return SignalDestroyed object for connecting to the destroyed signal.
     */
    using SignalDestroyed = sigc::signal<void()>;
    SignalDestroyed signalDestroyed();

    /**
     * @brief Connects the condition to a signal for toggling.
     * 
     * @param signal Signal to connect to.
     * @return Connection object representing the connection.
     */
    sigc::connection toggleOnSignal(Glib::SignalProxy<void> signal);

    /**
     * @brief Gets the current value of the condition.
     * 
     * @return Current boolean value of the condition.
     */
    bool getValue() const;

    /**
     * @brief Toggles the condition's value.
     * 
     * Flips the value of the condition and emits the appropriate signals.
     * @return New value of the condition after toggling.
     */
    virtual bool toggle();

    /**
     * @brief Sets the condition's value.
     * 
     * Updates the value of the condition and emits the appropriate signals if the value changes.
     * @param val New value for the condition (default is true).
     */
    virtual void setValue(bool val = true);

    /**
     * @brief Gets the unique identifier for the condition.
     * 
     * @return String representing the unique identifier of the condition.
     */
    const std::string &getID() const;

protected:
    /**
     * @brief Handles the toggle signal.
     * 
     * Toggles the condition when the signal is received.
     */
    void onSigToggle();

    SignalToggled sigToggled; ///< Signal emitted when the condition is toggled.
    SignalTrue sigTrue; ///< Signal emitted when the condition is set to true.
    SignalFalse sigFalse; ///< Signal emitted when the condition is set to false.
    SignalDestroyed sigDestroyed; ///< Signal emitted when the condition is destroyed.

    bool value; ///< Current value of the condition.
    std::string identity; ///< Unique identifier for the condition.
};

/**
 * @class OpCondition
 * @brief Represents a composite condition formed by combining two conditions with an operator.
 * 
 * OpCondition allows logical operations (AND, OR, NOT) to be performed on two conditions.
 * It updates its value based on the values of the combined conditions and emits signals accordingly.
 */
class OpCondition : public Condition
{
public:
    /**
     * @enum Operator
     * @brief Defines the logical operators for combining conditions.
     */
    enum Operator
    {
        OPCOND_AND, ///< Logical AND operator.
        OPCOND_OR, ///< Logical OR operator.
        OPCOND_NOT ///< Logical NOT operator.
    };

    /**
     * @brief Constructor for the OpCondition class.
     * 
     * Initializes the composite condition with two conditions and an operator.
     * @param op Logical operator for combining the conditions.
     * @param a First condition.
     * @param b Second condition.
     */
    OpCondition(Operator op, Condition &a, Condition &b);

    /**
     * @brief Destructor for the OpCondition class.
     */
    ~OpCondition();

    /**
     * @brief Toggles the composite condition's value.
     * 
     * This method is overridden to prevent direct toggling of composite conditions.
     * @return Current value of the composite condition.
     */
    virtual bool toggle();

    /**
     * @brief Sets the composite condition's value.
     * 
     * This method is overridden to prevent direct setting of composite conditions.
     * @param val New value for the composite condition.
     */
    virtual void setValue(bool val = true);

    /**
     * @brief Gets the global map of all composite conditions.
     * 
     * @return Reference to the map of composite conditions.
     */
    static std::unordered_map<std::string, OpCondition*>& getConds();

protected:
    /**
     * @brief Handles the toggle signal from the combined conditions.
     * 
     * Updates the composite condition's value based on the operator and the values of the combined conditions.
     * @param v Value of the toggled condition.
     */
    void onCondToggle(bool v);

    /**
     * @brief Handles the destroyed signal from the combined conditions.
     * 
     * Removes the composite condition from the global map and deletes it.
     */
    void onCondDestroy();

    Operator op; ///< Logical operator for combining the conditions.
    Condition &cond1; ///< First condition.
    Condition &cond2; ///< Second condition.

    static std::unordered_map<std::string, OpCondition*> conds; ///< Global map of composite conditions.
};

/**
 * @brief Combines two conditions with a logical AND operator.
 * 
 * Creates a composite condition that represents the logical AND of the two conditions.
 * @param a First condition.
 * @param b Second condition.
 * @return Reference to the composite condition.
 */
extern OpCondition& operator&&(Condition &a, Condition &b);

/**
 * @brief Combines two conditions with a logical OR operator.
 * 
 * Creates a composite condition that represents the logical OR of the two conditions.
 * @param a First condition.
 * @param b Second condition.
 * @return Reference to the composite condition.
 */
extern OpCondition& operator||(Condition &a, Condition &b);

/**
 * @brief Negates a condition with a logical NOT operator.
 * 
 * Creates a composite condition that represents the logical NOT of the condition.
 * @param a Condition to negate.
 * @return Reference to the composite condition.
 */
extern OpCondition& operator!(Condition &a);

#endif
