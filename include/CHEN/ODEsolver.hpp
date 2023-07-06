//
// Created by Administrator on 2023/7/6.
//

#ifndef TCCPPLIB_INCLUDE_CHEN_ODESOLVER_HPP_
#define TCCPPLIB_INCLUDE_CHEN_ODESOLVER_HPP_

#include "TcEigen/Dense"

namespace ODEsolver
{
	template <size_t N>
	using State = Eigen::Vector<double, N>;

	template <size_t N>
	using dState = Eigen::Vector<double, N>;

	template <size_t N, size_t L, typename ...Ts>
	using FuncHandle = dState<N> (*)(const State<N>& y, const State<L>& u, const double& t, Ts...args); //微分方程函数原型


	class Model
	{
	 public:
        // First Order Integrator
        static dState<1> FirstOrderIntegrator (const State<1>& y, const State<1>& u, const double& t)
        {
            return u;
        }

        // First Order Integrator with K
        static dState<1> FirstOrderIntegratorWithK (const State<1>& y, const State<1>& u, const double& t, double K)
        {
            return K*u;
        }

        // Second Order Integrator
        static dState<2> SecondOrderIntegrator (const State<2>& y, const State<1>& u, const double& t)
        {
            dState<2> dx;
            dx(0) = y(1);
            dx(1) = u(0);
            return dx;
        }


    };

	template <size_t N, size_t L, typename ...Ts>
    State<N> RK4(const State<N>& yk, const State<L>& u, const double& t, const double& h, const FuncHandle<N, L, Ts...>& func, Ts...args)
    {
        State<N> yk_;
        dState<N> k1;
        dState<N> k2;
        dState<N> k3;
        dState<N> k4;
        k1 = func(yk, u, t, args...);
        k2 = func(yk+0.5*k1*h, u, t+0.5*h, args...);
        k3 = func(yk+0.5*k2*h, u, t+0.5*h, args...);
        k4 = func(yk+k3*h, u, t+h, args...);

        yk_ = yk + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
        return yk_;
    }


    template <size_t N, size_t L, typename ...Ts>
    State<N> RK5(const State<N>& yk, const State<L>& u, const double& t, const double& h, const FuncHandle<N, L, Ts...>& func, Ts...args)
    {
        State<N> yk_;
        dState<N> k1;
        dState<N> k2;
        dState<N> k3;
        dState<N> k4;
        dState<N> k5;

        k1 = func(yk, u, t, args...);
        k2 = func(yk+k1*(h/2), u, t+0.5*h, args...);
        k3 = func(yk+(k1+k2)*(h/4), u, t+0.5*h, args...);
        k4 = func(yk+(k1+k2+k3)*(h/6), u, t+0.5*h, args...);
        k5 = func(yk+(k1+k2+k3+k4)*(h/8), u, t+0.5*h, args...);

        yk_ = yk + (h/5)*(k1 + k2 + k3 + k4 + k5);
        return yk_;
    }

}



#endif //TCCPPLIB_INCLUDE_CHEN_ODESOLVER_HPP_
