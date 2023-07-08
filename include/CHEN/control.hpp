#ifndef TCCPPLIB_INCLUDE_CHEN_CONTROL_HPP_
#define TCCPPLIB_INCLUDE_CHEN_CONTROL_HPP_

#include "TcEigen/Dense"
#include "ODEsolver.hpp"
#include <array>

namespace Control
{
	template<size_t N>
	using State = Eigen::Vector<double, N>;

	// continue LTI system
	// order N, input u_size, output y_size
	template<size_t N, size_t u_size, size_t y_size, size_t CatchLength = 10>
	class StateSpace
	{
	 public:
		State<N> x;
		State<u_size> u;
		State<y_size> y;
		double t;

		std::array<State<N>, CatchLength> x_catch;
		std::array<State<u_size>, CatchLength> u_catch;
		std::array<State<y_size>, CatchLength> y_catch;

		Eigen::Matrix<double, N, N> A;
		Eigen::Matrix<double, N, u_size> B;
		Eigen::Matrix<double, y_size, N> C;
		Eigen::Matrix<double, y_size, u_size> D;

		void cleanAll()
		{
			x.setZero();
			y.setZero();
			u.setZero();
			t = 0;
			A.setZero();
			B.setZero();
			C.setZero();
			D.setZero();
			cleanCatch();
		}



		StateSpace()
		{
			cleanAll();
		}
		StateSpace(const Eigen::Matrix<double, N, N>& A, const Eigen::Matrix<double, N, u_size> B, const Eigen::Matrix<double, y_size, N> C)
		{
			cleanAll();
			this->A = A;
			this->B = B;
			this->C = C;
		};
		StateSpace(const Eigen::Matrix<double, N, N>& A,
			const Eigen::Matrix<double, N, u_size> B,
			const Eigen::Matrix<double, y_size, N> C,
			const Eigen::Matrix<double, y_size, u_size> D)
		{
			cleanAll();
			this->A = A;
			this->B = B;
			this->C = C;
			this->D = D;
		};

		void setState(const State<N>& x)
		{
			this->x = x;
		}
		State<N> getState()
		{
			return this->x;
		}

		void setInput(const State<u_size>& u)
		{
			this->u = u;
		}

		State<u_size> getInput()
		{
			return this->u;
		}

		State<y_size> getOutput()
		{
			return this->y;
		}

		// solve the ODE by step dt
		void step(double dt)
		{
			State<N> x_;
			auto func = this->LTI;
			x_ = ODEsolver::RK4<N, u_size, Eigen::Matrix<double, N, N>, Eigen::Matrix<double, N, u_size>>(
				this->x, this->u, this->t, dt, func, this->A, this->B);

			this->x = x_;
			this->y = C * x + D * u;
			this->t += dt;

			rollingCatch();

		}

	 private:
		static State<N> LTI(const State<N>& x,
			const State<u_size>& u,
			const double& t,
			const Eigen::Matrix<double, N, N> A,
			const Eigen::Matrix<double, N, u_size> B)
		{
			State<N> dx;
			dx = A * x + B * u;
			return dx;
		}

		// TODO
		void rollingCatch()
		{

		}

		// TODO
		void cleanCatch()
		{

		}

	};

	template<size_t N, size_t u_size, size_t y_size>
	class DiscreteStateSpace
	{

	};

}

#endif //TCCPPLIB_INCLUDE_CHEN_CONTROL_HPP_
