#include "cond.hpp"
#include <string>
#include <tuple>

Condition::Condition(bool val) : 
	value(val),
	identity(std::string("#") + std::to_string(std::uintptr_t(this) ) )
{
}

Condition::~Condition()
{
	sigDestroyed.emit();
}

Condition::SignalToggled Condition::signalToggled()
{
    return sigToggled;
}

Condition::SignalTrue Condition::signalTrue()
{
    return sigTrue;
}

Condition::SignalFalse Condition::signalFalse()
{
    return sigFalse;
}

Condition::SignalDestroyed Condition::signalDestroyed()
{
	return sigDestroyed;
}

sigc::connection Condition::toggleOnSignal(Glib::SignalProxy<void> signal)
{
	return signal.connect(sigc::mem_fun(*this, &Condition::onSigToggle) );
}

bool Condition::getValue() const
{
    return value;
}

bool Condition::toggle()
{
    value = !value;
    sigToggled.emit(value);
    if (value)
        sigTrue.emit();
    else
        sigFalse.emit();
    return value;
}

void Condition::setValue(bool val)
{
    if (val != value)
    {
        value = val;
        sigToggled.emit(value);
        if (value)
            sigTrue.emit();
        else
            sigFalse.emit();
    }
}

const std::string& Condition::getID() const
{
	return identity;
}

void Condition::onSigToggle()
{
	toggle();
}

Condition nullcond = Condition();
OpCondition nullop = OpCondition(OpCondition::OPCOND_OR, nullcond, nullcond);

std::unordered_map<std::string, OpCondition*> OpCondition::conds = std::unordered_map<std::string, OpCondition*>();

OpCondition::OpCondition(Operator op, Condition &a, Condition &b) : Condition(a.getValue() && b.getValue() ),
	op(op),
	cond1(a),
	cond2(b)
{
	identity = std::string("(") + std::to_string(op) + cond1.getID() + cond2.getID() + ")";
	cond1.signalToggled().connect(sigc::mem_fun(*this, &OpCondition::onCondToggle) );
	cond1.signalDestroyed().connect(sigc::mem_fun(*this, &OpCondition::onCondDestroy) );
	if (&cond1 != &cond2)
	{
		cond2.signalToggled().connect(sigc::mem_fun(*this, &OpCondition::onCondToggle) );
		cond2.signalDestroyed().connect(sigc::mem_fun(*this, &OpCondition::onCondDestroy) );
	}
}

OpCondition::~OpCondition()
{
}

bool OpCondition::toggle()
{
	return value;
}

void OpCondition::setValue(bool val)
{
}

void OpCondition::onCondToggle(bool v)
{
	switch(op)
	{
		case OPCOND_AND:
			Condition::setValue(cond1.getValue() && cond2.getValue() );
			break;
		case OPCOND_OR:
			Condition::setValue(cond1.getValue() || cond2.getValue() );
			break;
		case OPCOND_NOT:
			Condition::setValue(!cond1.getValue() );
			break;
	}
}

void OpCondition::onCondDestroy()
{
	if (OpCondition::getConds().count(identity) > 0)
	{
		OpCondition::getConds().erase(identity);
		delete this;
	}
}

std::unordered_map<std::string, OpCondition*>& OpCondition::getConds()
{
	return conds;
}

OpCondition& operator&&(Condition &a, Condition &b)
{
	OpCondition *opc = new OpCondition(OpCondition::OPCOND_AND, a, b);
	std::string id = opc->getID();
	if (OpCondition::getConds().count(id) == 0)
	{
		OpCondition::getConds().insert({id, opc});
	}
	else 
	{
		delete opc;
	}
	return *OpCondition::getConds()[id];
}

OpCondition& operator||(Condition &a, Condition &b)
{
	OpCondition *opc = new OpCondition(OpCondition::OPCOND_OR, a, b);
	std::string id = opc->getID();
	if (OpCondition::getConds().count(id) == 0)
	{
		OpCondition::getConds().insert({id, opc});
	}
	else 
	{
		delete opc;
	}
	return *OpCondition::getConds()[id];
}

OpCondition& operator!(Condition &a)
{
	OpCondition *opc = new OpCondition(OpCondition::OPCOND_NOT, a, a);
	std::string id = opc->getID();
	if (OpCondition::getConds().count(id) == 0)
	{
		OpCondition::getConds().insert({id, opc});
	}
	else 
	{
		delete opc;
	}
	return *OpCondition::getConds()[id];
}

