/*
	Satellite Attitude Dynamics Stepper
	@ref		:	boost lib
*/
#pragma once
template< class Obj, class Mem >
class ode_wrapper
{
	Obj m_obj;
	Mem m_mem;

public:

	ode_wrapper(Obj obj, Mem mem) : m_obj(obj), m_mem(mem) { }

	template< class State, class Deriv, class Time >
	void operator()(const State& x, Deriv& dxdt, Time t)
	{
		(m_obj.*m_mem)(x, dxdt, t);
	}
};

template< class Obj, class Mem >
ode_wrapper< Obj, Mem > make_ode_wrapper(Obj obj, Mem mem)
{
	return ode_wrapper< Obj, Mem >(obj, mem);
}


template< class Obj, class Mem >
class observer_wrapper
{
	Obj m_obj;
	Mem m_mem;

public:
	observer_wrapper(Obj obj, Mem mem) : m_obj(obj), m_mem(mem) { }
	template< class State, class Time >
	void operator()(const State& x, Time t)
	{
		(m_obj.*m_mem)(x, t);
	}
};

template< class Obj, class Mem >
observer_wrapper< Obj, Mem > make_observer_wrapper(Obj obj, Mem mem)
{
	return observer_wrapper< Obj, Mem >(obj, mem);
}