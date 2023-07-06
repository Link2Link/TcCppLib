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

	template <size_t N>
	using FuncHandle = dState<N> (*)(const State<N>& y, const double& t); //微分方程函数原型

	class Model
	{
	 public:
		// First Order Integrator
		static dState<1> FirstOrderIntegrator (const State<1>& y, const double& t)
		{
			return y;
		}
	};

	template <size_t N>
	State<N> RK4(const State<N>& yk, const double& t, const double& h, const FuncHandle<N>& func)
	{
		State<N> yk_;
		dState<N> k1;
		dState<N> k2;
		dState<N> k3;
		dState<N> k4;
		k1 = func(yk, t);
		k2 = func(yk+0.5*k1*h, t+0.5*h);
		k3 = func(yk+0.5*k2*h, t+0.5*h);
		k4 = func(yk+k3*h, t+h);

		yk_ = yk + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
		return yk_;
	}

}



#endif //TCCPPLIB_INCLUDE_CHEN_ODESOLVER_HPP_
