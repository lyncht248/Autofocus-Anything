#ifndef HVIGTK_COND_H
#define HVIGTK_COND_H

#include <cstdint>
#include <gtkmm.h>
#include <initializer_list>
#include <unordered_map>



class Condition
{
	friend class OpCondition;
public:
    Condition(bool val = false);
    ~Condition();
    
    using SignalToggled = sigc::signal<void(bool)>;
    SignalToggled signalToggled();
    
    using SignalTrue = sigc::signal<void()>;
    SignalTrue signalTrue();
    
    using SignalFalse = sigc::signal<void()>;
    SignalFalse signalFalse();

    using SignalDestroyed = sigc::signal<void()>;
    SignalDestroyed signalDestroyed();

	sigc::connection toggleOnSignal(Glib::SignalProxy<void> signal);
    
    bool getValue() const;

    virtual bool toggle();
    virtual void setValue(bool val=true);

	const std::string &getID() const;

protected:
	void onSigToggle();
    SignalToggled sigToggled;
    SignalTrue sigTrue;
    SignalFalse sigFalse;
	SignalDestroyed sigDestroyed;
    
    bool value; //the signal 
	std::string identity;
};

class OpCondition : public Condition
{
public:
	enum Operator
	{
		OPCOND_AND,
		OPCOND_OR,
		OPCOND_NOT
	};
	OpCondition(Operator op, Condition &a, Condition &b);
	~OpCondition();

    virtual bool toggle();
    virtual void setValue(bool val=true);

	static std::unordered_map<std::string, OpCondition*>& getConds();

protected:
	void onCondToggle(bool v);
	void onCondDestroy();
	
	Operator op;
	Condition &cond1, &cond2;

	static std::unordered_map<std::string, OpCondition*> conds;
};

extern OpCondition& operator&&(Condition &a, Condition &b);
extern OpCondition& operator||(Condition &a, Condition &b);
extern OpCondition& operator!(Condition &a);

#endif
