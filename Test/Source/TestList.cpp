//
//  TestList.c
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include <stdlib.h>
#include "TestList.h"

#include "Test_qtmul.h"
#include "Test_qtmulQV3.h"
#include "Test_qtmulV3Q.h"
#include "Test_qtdot.h"
#include "Test_qtnorm.h"

#include "Test_v3dot.h"
#include "Test_v3sdiv.h"
#include "Test_v3norm.h"
#include "Test_v3cross.h"
#include "Test_v3triple.h"
#include "Test_v3interp.h"
#include "Test_v3lerp.h"
#include "Test_v3skew.h"
#include "Test_v3div.h"
#include "Test_v3rotate.h"

#include "Test_maxdot.h"
#include "Test_mindot.h"
#include "Test_dot3.h"
#include "Test_3x3transpose.h"
#include "Test_3x3transposeTimes.h"
#include "Test_3x3timesTranspose.h"
#include "Test_3x3mulM.h"
#include "Test_3x3mulM1M2.h"
#include "Test_3x3mulMV.h"
#include "Test_3x3mulVM.h"
#include "Test_3x3setRot.h"
#include "Test_3x3getRot.h"

#include "Test_btDbvt.h"
#include "Test_quat_aos_neon.h"

#include "LinearMath/btScalar.h"
#define ENTRY( _name, _func )       { _name, _func }

//
// Test functions have the form  int (*TestFunc)( void )
// They return a non-zero result in case of failure.
//
// Please see handy stuff in Utils.h, vector.h when writing your test code.
//
#if defined (BT_USE_NEON) || defined (BT_USE_SSE_IN_API)

TestDesc  gTestList[] = 
{
    ENTRY( "maxdot", Test_maxdot ),
    ENTRY( "mindot", Test_mindot ),

    ENTRY( "qtmul", Test_qtmul ),
    ENTRY( "qtmulQV3", Test_qtmulQV3 ),
    ENTRY( "qtmulV3Q", Test_qtmulV3Q ),
    ENTRY( "qtdot", Test_qtdot ),
    ENTRY( "qtnorm", Test_qtnorm ),

    ENTRY( "v3dot", Test_v3dot ),
    ENTRY( "v3sdiv", Test_v3sdiv ),
    ENTRY( "v3norm", Test_v3norm ),
    ENTRY( "v3cross", Test_v3cross ),
    ENTRY( "v3triple", Test_v3triple ),
    ENTRY( "v3interp", Test_v3interp ),
    ENTRY( "v3lerp", Test_v3lerp ),
    ENTRY( "v3skew", Test_v3skew ),
    ENTRY( "v3div", Test_v3div ),
    ENTRY( "v3rotate", Test_v3rotate ),

    ENTRY( "dot3", Test_dot3 ),
    ENTRY( "3x3transpose", Test_3x3transpose ),
    ENTRY( "3x3transposeTimes", Test_3x3transposeTimes ),
    ENTRY( "3x3timesTranspose", Test_3x3timesTranspose ),
    ENTRY( "3x3mulM", Test_3x3mulM ),
    ENTRY( "3x3mulM1M2", Test_3x3mulM1M2 ),
    ENTRY( "3x3mulMV", Test_3x3mulMV ),
    ENTRY( "3x3mulVM", Test_3x3mulMV ),
    ENTRY( "3x3setRot", Test_3x3setRot ),
    ENTRY( "3x3getRot", Test_3x3getRot ),
  
    ENTRY( "btDbvt", Test_btDbvt ),
    ENTRY("quat_aos_neon", Test_quat_aos_neon),
    
    { NULL, NULL }
};
#else
TestDesc  gTestList[]={{NULL,NULL}};

#endif

