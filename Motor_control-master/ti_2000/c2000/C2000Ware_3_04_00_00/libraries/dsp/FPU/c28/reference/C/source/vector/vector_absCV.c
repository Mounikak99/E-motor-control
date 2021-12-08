//#############################################################################
//! \file   reference/C/source/$FILENAME$
//! \brief  Absolute of a Complex Vector
//! \author Vishal Coelho 
//! \date   14-Sep-2015
//! 
//
//  Group:            C2000
//  Target Family:    $DEVICE$
//
//#############################################################################
// $TI Release: $
// $Release Date: $
// $Copyright: $
//#############################################################################


//*****************************************************************************
// the includes
//*****************************************************************************
#include "vector.h"
#include <math.h>

//*****************************************************************************
// defines
//*****************************************************************************
DSP_FILENUM(2)

//*****************************************************************************
// VECTOR_absCV
//*****************************************************************************
void VECTOR_absCV(fsize_t *y, const complex_t *x, const uint16_t n)
{
    // Locals
    uint16_t i;
        
    for(i = 0U; i < n; i++)
    {
        y[i] = sqrt((x[i].r * x[i].r) + (x[i].i * x[i].i));
#ifdef _DEBUG
        // Debugging message
        printf("i=%4d, x=%10.7f+j%10.7f, y=|x|=%10.7f\n", 
               i, x[i].r, x[i].i, y[i]);
#endif
    }
}

// End of File
