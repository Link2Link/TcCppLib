#ifndef TCCPPLIB_INCLUDE_CHEN_FILTER_HPP_
#define TCCPPLIB_INCLUDE_CHEN_FILTER_HPP_

#include <array>


namespace CHEN
{
	template <typename DT>
	class FilterBase
	{
	 public:
		virtual void reset(DT) = 0;
		virtual void in(DT) = 0;
		virtual DT out() const = 0;
	};

	template <typename  DT, std::size_t N>
	class FIRBase : FilterBase<DT>
	{
	 protected:
		std::array<DT, N> _pool;
		virtual void rollingpool()
		{
			for (int k = 0; k < N - 1; ++k)
			{
				_pool[k] = _pool[k+1];
			}
		}

	 public:
		void reset(DT in)
		{
			for (int k = 0; k < N; ++k)
			{
				_pool[k] = in;
			}
		}
		void in(DT in)
		{
			rollingpool();
			_pool[N-1] = in;
		}
	};

	template <typename DT, std::size_t N>
	class MovingAverage : FIRBase<DT, N>
	{
	 public:
		void reset(DT in)
		{
			FIRBase<DT, N>::reset(in);
		}
		void in(DT in)
		{
			FIRBase<DT, N>::in(in);
		}
		DT out() const
		{
			DT sum = 0;
			for (int k = 0; k < N; ++k)
			{
				sum += _pool[k];
			}
			return sum / N;
		}
	};

	template <typename DT, std::size_t N>
	class FIR : FIRBase<DT, N>
	{
	 public:
		std::array<DT, N> param;
		FIR()= default;
		FIR(std::array<DT, N> param):param(param){};
		void setparam(std::array<DT, N> param)
		{
			this->param = param;
		}

		DT out() const
		{
			DT sum = 0;
			for (int k = 0; k < N; ++k)
			{
				sum += _pool[k] * param[N-1-k]; //conv
			}
			return sum / N;
		}
		void reset(DT in)
		{
			FIRBase<DT, N>::reset(in);
		}
		void in(DT in)
		{
			FIRBase<DT, N>::in(in);
		}
	};

}


#endif //TCCPPLIB_INCLUDE_CHEN_FILTER_HPP_
