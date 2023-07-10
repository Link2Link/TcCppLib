//
// Created by Administrator on 2023/7/3.
//

#ifndef TCCPPLIB_INCLUDE_PPX_PPX_H_
#define TCCPPLIB_INCLUDE_PPX_PPX_H_

//#define TC_ENVIRONMENT_RT

#ifdef TC_ENVIRONMENT_RT
	#define TWINCAT_INCOMPATIBLE
#endif

#include "exprtmpl.hpp"
#include "matrixs.hpp"
#include "linalg.hpp"
#include "optimization.hpp"
#ifndef TWINCAT_INCOMPATIBLE	// Twincat实时环境下裁剪这两个
	#include "statistics.hpp"
	#include "signals.hpp"
#endif
#include "liegroup.hpp"
#include "robotics.hpp"

#endif //TCCPPLIB_INCLUDE_PPX_PPX_H_
