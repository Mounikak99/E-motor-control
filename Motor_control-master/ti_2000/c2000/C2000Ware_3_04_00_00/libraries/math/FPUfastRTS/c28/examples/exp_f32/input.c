//#############################################################################
//! \file input.c
//! \brief  Exp Input Vector (512) 
//! \author Vishal Coelho 
//! \date   23-Jan-2017
//! 
//
//  Group:          C2000
//  Target Family:  $DEVICE$
//
//#############################################################################
// $TI Release: C28x Floating Point Unit Library V2.04.00.00 $
// $Release Date: Feb 12, 2021 $
// $Copyright: Copyright (C) 2021 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

#include "fastrts.h"

const float32_t test_input[512] = {
    -3.14159265359F, -3.12932080729F, -3.11704896098F, -3.10477711468F, 
    -3.09250526838F, -3.08023342207F, -3.06796157577F, -3.05568972947F, 
    -3.04341788317F, -3.03114603686F, -3.01887419056F, -3.00660234426F, 
    -2.99433049795F, -2.98205865165F, -2.96978680535F, -2.95751495904F, 
    -2.94524311274F, -2.93297126644F, -2.92069942013F, -2.90842757383F, 
    -2.89615572753F, -2.88388388123F, -2.87161203492F, -2.85934018862F, 
    -2.84706834232F, -2.83479649601F, -2.82252464971F, -2.81025280341F, 
    -2.79798095710F, -2.78570911080F, -2.77343726450F, -2.76116541819F, 
    -2.74889357189F, -2.73662172559F, -2.72434987928F, -2.71207803298F, 
    -2.69980618668F, -2.68753434038F, -2.67526249407F, -2.66299064777F, 
    -2.65071880147F, -2.63844695516F, -2.62617510886F, -2.61390326256F, 
    -2.60163141625F, -2.58935956995F, -2.57708772365F, -2.56481587734F, 
    -2.55254403104F, -2.54027218474F, -2.52800033844F, -2.51572849213F, 
    -2.50345664583F, -2.49118479953F, -2.47891295322F, -2.46664110692F, 
    -2.45436926062F, -2.44209741431F, -2.42982556801F, -2.41755372171F, 
    -2.40528187540F, -2.39301002910F, -2.38073818280F, -2.36846633650F, 
    -2.35619449019F, -2.34392264389F, -2.33165079759F, -2.31937895128F, 
    -2.30710710498F, -2.29483525868F, -2.28256341237F, -2.27029156607F, 
    -2.25801971977F, -2.24574787346F, -2.23347602716F, -2.22120418086F, 
    -2.20893233456F, -2.19666048825F, -2.18438864195F, -2.17211679565F, 
    -2.15984494934F, -2.14757310304F, -2.13530125674F, -2.12302941043F, 
    -2.11075756413F, -2.09848571783F, -2.08621387152F, -2.07394202522F, 
    -2.06167017892F, -2.04939833262F, -2.03712648631F, -2.02485464001F, 
    -2.01258279371F, -2.00031094740F, -1.98803910110F, -1.97576725480F, 
    -1.96349540849F, -1.95122356219F, -1.93895171589F, -1.92667986958F, 
    -1.91440802328F, -1.90213617698F, -1.88986433068F, -1.87759248437F, 
    -1.86532063807F, -1.85304879177F, -1.84077694546F, -1.82850509916F, 
    -1.81623325286F, -1.80396140655F, -1.79168956025F, -1.77941771395F, 
    -1.76714586764F, -1.75487402134F, -1.74260217504F, -1.73033032874F, 
    -1.71805848243F, -1.70578663613F, -1.69351478983F, -1.68124294352F, 
    -1.66897109722F, -1.65669925092F, -1.64442740461F, -1.63215555831F, 
    -1.61988371201F, -1.60761186570F, -1.59534001940F, -1.58306817310F, 
    -1.57079632679F, -1.55852448049F, -1.54625263419F, -1.53398078789F, 
    -1.52170894158F, -1.50943709528F, -1.49716524898F, -1.48489340267F, 
    -1.47262155637F, -1.46034971007F, -1.44807786376F, -1.43580601746F, 
    -1.42353417116F, -1.41126232485F, -1.39899047855F, -1.38671863225F, 
    -1.37444678595F, -1.36217493964F, -1.34990309334F, -1.33763124704F, 
    -1.32535940073F, -1.31308755443F, -1.30081570813F, -1.28854386182F, 
    -1.27627201552F, -1.26400016922F, -1.25172832291F, -1.23945647661F, 
    -1.22718463031F, -1.21491278401F, -1.20264093770F, -1.19036909140F, 
    -1.17809724510F, -1.16582539879F, -1.15355355249F, -1.14128170619F, 
    -1.12900985988F, -1.11673801358F, -1.10446616728F, -1.09219432097F, 
    -1.07992247467F, -1.06765062837F, -1.05537878207F, -1.04310693576F, 
    -1.03083508946F, -1.01856324316F, -1.00629139685F, -0.99401955055F, 
    -0.98174770425F, -0.96947585794F, -0.95720401164F, -0.94493216534F, 
    -0.93266031903F, -0.92038847273F, -0.90811662643F, -0.89584478013F, 
    -0.88357293382F, -0.87130108752F, -0.85902924122F, -0.84675739491F, 
    -0.83448554861F, -0.82221370231F, -0.80994185600F, -0.79767000970F, 
    -0.78539816340F, -0.77312631709F, -0.76085447079F, -0.74858262449F, 
    -0.73631077819F, -0.72403893188F, -0.71176708558F, -0.69949523928F, 
    -0.68722339297F, -0.67495154667F, -0.66267970037F, -0.65040785406F, 
    -0.63813600776F, -0.62586416146F, -0.61359231515F, -0.60132046885F, 
    -0.58904862255F, -0.57677677625F, -0.56450492994F, -0.55223308364F, 
    -0.53996123734F, -0.52768939103F, -0.51541754473F, -0.50314569843F, 
    -0.49087385212F, -0.47860200582F, -0.46633015952F, -0.45405831321F, 
    -0.44178646691F, -0.42951462061F, -0.41724277430F, -0.40497092800F, 
    -0.39269908170F, -0.38042723540F, -0.36815538909F, -0.35588354279F, 
    -0.34361169649F, -0.33133985018F, -0.31906800388F, -0.30679615758F, 
    -0.29452431127F, -0.28225246497F, -0.26998061867F, -0.25770877236F, 
    -0.24543692606F, -0.23316507976F, -0.22089323346F, -0.20862138715F, 
    -0.19634954085F, -0.18407769455F, -0.17180584824F, -0.15953400194F, 
    -0.14726215564F, -0.13499030933F, -0.12271846303F, -0.11044661673F, 
    -0.09817477042F, -0.08590292412F, -0.07363107782F, -0.06135923152F, 
    -0.04908738521F, -0.03681553891F, -0.02454369261F, -0.01227184630F, 
    -0.00000000000F,  0.01227184630F,  0.02454369261F,  0.03681553891F, 
     0.04908738521F,  0.06135923152F,  0.07363107782F,  0.08590292412F, 
     0.09817477042F,  0.11044661673F,  0.12271846303F,  0.13499030933F, 
     0.14726215564F,  0.15953400194F,  0.17180584824F,  0.18407769455F, 
     0.19634954085F,  0.20862138715F,  0.22089323346F,  0.23316507976F, 
     0.24543692606F,  0.25770877236F,  0.26998061867F,  0.28225246497F, 
     0.29452431127F,  0.30679615758F,  0.31906800388F,  0.33133985018F, 
     0.34361169649F,  0.35588354279F,  0.36815538909F,  0.38042723540F, 
     0.39269908170F,  0.40497092800F,  0.41724277430F,  0.42951462061F, 
     0.44178646691F,  0.45405831321F,  0.46633015952F,  0.47860200582F, 
     0.49087385212F,  0.50314569843F,  0.51541754473F,  0.52768939103F, 
     0.53996123734F,  0.55223308364F,  0.56450492994F,  0.57677677624F, 
     0.58904862255F,  0.60132046885F,  0.61359231515F,  0.62586416146F, 
     0.63813600776F,  0.65040785406F,  0.66267970037F,  0.67495154667F, 
     0.68722339297F,  0.69949523928F,  0.71176708558F,  0.72403893188F, 
     0.73631077819F,  0.74858262449F,  0.76085447079F,  0.77312631709F, 
     0.78539816340F,  0.79767000970F,  0.80994185600F,  0.82221370231F, 
     0.83448554861F,  0.84675739491F,  0.85902924122F,  0.87130108752F, 
     0.88357293382F,  0.89584478013F,  0.90811662643F,  0.92038847273F, 
     0.93266031903F,  0.94493216534F,  0.95720401164F,  0.96947585794F, 
     0.98174770425F,  0.99401955055F,  1.00629139685F,  1.01856324316F, 
     1.03083508946F,  1.04310693576F,  1.05537878207F,  1.06765062837F, 
     1.07992247467F,  1.09219432097F,  1.10446616728F,  1.11673801358F, 
     1.12900985988F,  1.14128170619F,  1.15355355249F,  1.16582539879F, 
     1.17809724510F,  1.19036909140F,  1.20264093770F,  1.21491278401F, 
     1.22718463031F,  1.23945647661F,  1.25172832291F,  1.26400016922F, 
     1.27627201552F,  1.28854386182F,  1.30081570813F,  1.31308755443F, 
     1.32535940073F,  1.33763124704F,  1.34990309334F,  1.36217493964F, 
     1.37444678595F,  1.38671863225F,  1.39899047855F,  1.41126232485F, 
     1.42353417116F,  1.43580601746F,  1.44807786376F,  1.46034971007F, 
     1.47262155637F,  1.48489340267F,  1.49716524898F,  1.50943709528F, 
     1.52170894158F,  1.53398078789F,  1.54625263419F,  1.55852448049F, 
     1.57079632679F,  1.58306817310F,  1.59534001940F,  1.60761186570F, 
     1.61988371201F,  1.63215555831F,  1.64442740461F,  1.65669925092F, 
     1.66897109722F,  1.68124294352F,  1.69351478983F,  1.70578663613F, 
     1.71805848243F,  1.73033032873F,  1.74260217504F,  1.75487402134F, 
     1.76714586764F,  1.77941771395F,  1.79168956025F,  1.80396140655F, 
     1.81623325286F,  1.82850509916F,  1.84077694546F,  1.85304879177F, 
     1.86532063807F,  1.87759248437F,  1.88986433068F,  1.90213617698F, 
     1.91440802328F,  1.92667986958F,  1.93895171589F,  1.95122356219F, 
     1.96349540849F,  1.97576725480F,  1.98803910110F,  2.00031094740F, 
     2.01258279371F,  2.02485464001F,  2.03712648631F,  2.04939833262F, 
     2.06167017892F,  2.07394202522F,  2.08621387152F,  2.09848571783F, 
     2.11075756413F,  2.12302941043F,  2.13530125674F,  2.14757310304F, 
     2.15984494934F,  2.17211679565F,  2.18438864195F,  2.19666048825F, 
     2.20893233456F,  2.22120418086F,  2.23347602716F,  2.24574787346F, 
     2.25801971977F,  2.27029156607F,  2.28256341237F,  2.29483525868F, 
     2.30710710498F,  2.31937895128F,  2.33165079759F,  2.34392264389F, 
     2.35619449019F,  2.36846633650F,  2.38073818280F,  2.39301002910F, 
     2.40528187540F,  2.41755372171F,  2.42982556801F,  2.44209741431F, 
     2.45436926062F,  2.46664110692F,  2.47891295322F,  2.49118479953F, 
     2.50345664583F,  2.51572849213F,  2.52800033844F,  2.54027218474F, 
     2.55254403104F,  2.56481587734F,  2.57708772365F,  2.58935956995F, 
     2.60163141625F,  2.61390326256F,  2.62617510886F,  2.63844695516F, 
     2.65071880147F,  2.66299064777F,  2.67526249407F,  2.68753434038F, 
     2.69980618668F,  2.71207803298F,  2.72434987928F,  2.73662172559F, 
     2.74889357189F,  2.76116541819F,  2.77343726450F,  2.78570911080F, 
     2.79798095710F,  2.81025280341F,  2.82252464971F,  2.83479649601F, 
     2.84706834232F,  2.85934018862F,  2.87161203492F,  2.88388388122F, 
     2.89615572753F,  2.90842757383F,  2.92069942013F,  2.93297126644F, 
     2.94524311274F,  2.95751495904F,  2.96978680535F,  2.98205865165F, 
     2.99433049795F,  3.00660234426F,  3.01887419056F,  3.03114603686F, 
     3.04341788317F,  3.05568972947F,  3.06796157577F,  3.08023342207F, 
     3.09250526838F,  3.10477711468F,  3.11704896098F,  3.12932080729F, 
}; 


// End of File
