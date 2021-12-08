//#############################################################################
//! \file golden.c
//! \brief Inverse Square Root Ouput Vector (512) 
//! \author Vishal Coelho 
//! \date   05-Jan-2016
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

const float64_t test_golden[512] = {
    6.065306597126e-01L, 6.064384410883e-01L, 6.063462645147e-01L, 
    6.062541299600e-01L, 6.061620373921e-01L, 6.060699867792e-01L, 
    6.059779780895e-01L, 6.058860112911e-01L, 6.057940863523e-01L, 
    6.057022032413e-01L, 6.056103619264e-01L, 6.055185623760e-01L, 
    6.054268045584e-01L, 6.053350884419e-01L, 6.052434139950e-01L, 
    6.051517811862e-01L, 6.050601899840e-01L, 6.049686403568e-01L, 
    6.048771322733e-01L, 6.047856657019e-01L, 6.046942406114e-01L, 
    6.046028569704e-01L, 6.045115147476e-01L, 6.044202139117e-01L, 
    6.043289544315e-01L, 6.042377362757e-01L, 6.041465594133e-01L, 
    6.040554238129e-01L, 6.039643294435e-01L, 6.038732762741e-01L, 
    6.037822642736e-01L, 6.036912934110e-01L, 6.036003636552e-01L, 
    6.035094749754e-01L, 6.034186273407e-01L, 6.033278207201e-01L, 
    6.032370550827e-01L, 6.031463303979e-01L, 6.030556466347e-01L, 
    6.029650037625e-01L, 6.028744017505e-01L, 6.027838405680e-01L, 
    6.026933201844e-01L, 6.026028405691e-01L, 6.025124016914e-01L, 
    6.024220035207e-01L, 6.023316460267e-01L, 6.022413291787e-01L, 
    6.021510529463e-01L, 6.020608172991e-01L, 6.019706222066e-01L, 
    6.018804676386e-01L, 6.017903535646e-01L, 6.017002799544e-01L, 
    6.016102467776e-01L, 6.015202540041e-01L, 6.014303016037e-01L, 
    6.013403895461e-01L, 6.012505178012e-01L, 6.011606863389e-01L, 
    6.010708951291e-01L, 6.009811441418e-01L, 6.008914333469e-01L, 
    6.008017627144e-01L, 6.007121322145e-01L, 6.006225418171e-01L, 
    6.005329914924e-01L, 6.004434812105e-01L, 6.003540109415e-01L, 
    6.002645806557e-01L, 6.001751903234e-01L, 6.000858399146e-01L, 
    5.999965293999e-01L, 5.999072587494e-01L, 5.998180279335e-01L, 
    5.997288369226e-01L, 5.996396856871e-01L, 5.995505741975e-01L, 
    5.994615024243e-01L, 5.993724703379e-01L, 5.992834779089e-01L, 
    5.991945251078e-01L, 5.991056119053e-01L, 5.990167382719e-01L, 
    5.989279041785e-01L, 5.988391095955e-01L, 5.987503544938e-01L, 
    5.986616388441e-01L, 5.985729626171e-01L, 5.984843257838e-01L, 
    5.983957283149e-01L, 5.983071701813e-01L, 5.982186513539e-01L, 
    5.981301718037e-01L, 5.980417315015e-01L, 5.979533304185e-01L, 
    5.978649685255e-01L, 5.977766457938e-01L, 5.976883621943e-01L, 
    5.976001176981e-01L, 5.975119122765e-01L, 5.974237459005e-01L, 
    5.973356185414e-01L, 5.972475301705e-01L, 5.971594807589e-01L, 
    5.970714702779e-01L, 5.969834986990e-01L, 5.968955659933e-01L, 
    5.968076721324e-01L, 5.967198170876e-01L, 5.966320008304e-01L, 
    5.965442233321e-01L, 5.964564845644e-01L, 5.963687844988e-01L, 
    5.962811231067e-01L, 5.961935003599e-01L, 5.961059162298e-01L, 
    5.960183706882e-01L, 5.959308637067e-01L, 5.958433952570e-01L, 
    5.957559653109e-01L, 5.956685738401e-01L, 5.955812208163e-01L, 
    5.954939062115e-01L, 5.954066299975e-01L, 5.953193921461e-01L, 
    5.952321926292e-01L, 5.951450314188e-01L, 5.950579084868e-01L, 
    5.949708238053e-01L, 5.948837773462e-01L, 5.947967690816e-01L, 
    5.947097989836e-01L, 5.946228670242e-01L, 5.945359731757e-01L, 
    5.944491174101e-01L, 5.943622996997e-01L, 5.942755200167e-01L, 
    5.941887783333e-01L, 5.941020746217e-01L, 5.940154088544e-01L, 
    5.939287810037e-01L, 5.938421910418e-01L, 5.937556389412e-01L, 
    5.936691246743e-01L, 5.935826482136e-01L, 5.934962095315e-01L, 
    5.934098086005e-01L, 5.933234453932e-01L, 5.932371198820e-01L, 
    5.931508320397e-01L, 5.930645818388e-01L, 5.929783692519e-01L, 
    5.928921942517e-01L, 5.928060568110e-01L, 5.927199569023e-01L, 
    5.926338944986e-01L, 5.925478695725e-01L, 5.924618820970e-01L, 
    5.923759320447e-01L, 5.922900193886e-01L, 5.922041441015e-01L, 
    5.921183061564e-01L, 5.920325055263e-01L, 5.919467421840e-01L, 
    5.918610161027e-01L, 5.917753272553e-01L, 5.916896756148e-01L, 
    5.916040611544e-01L, 5.915184838472e-01L, 5.914329436662e-01L, 
    5.913474405848e-01L, 5.912619745760e-01L, 5.911765456131e-01L, 
    5.910911536693e-01L, 5.910057987179e-01L, 5.909204807321e-01L, 
    5.908351996854e-01L, 5.907499555511e-01L, 5.906647483025e-01L, 
    5.905795779131e-01L, 5.904944443562e-01L, 5.904093476054e-01L, 
    5.903242876342e-01L, 5.902392644160e-01L, 5.901542779244e-01L, 
    5.900693281329e-01L, 5.899844150152e-01L, 5.898995385448e-01L, 
    5.898146986955e-01L, 5.897298954409e-01L, 5.896451287546e-01L, 
    5.895603986105e-01L, 5.894757049823e-01L, 5.893910478436e-01L, 
    5.893064271685e-01L, 5.892218429306e-01L, 5.891372951039e-01L, 
    5.890527836622e-01L, 5.889683085794e-01L, 5.888838698295e-01L, 
    5.887994673864e-01L, 5.887151012242e-01L, 5.886307713168e-01L, 
    5.885464776383e-01L, 5.884622201627e-01L, 5.883779988642e-01L, 
    5.882938137169e-01L, 5.882096646948e-01L, 5.881255517722e-01L, 
    5.880414749233e-01L, 5.879574341223e-01L, 5.878734293434e-01L, 
    5.877894605610e-01L, 5.877055277493e-01L, 5.876216308826e-01L, 
    5.875377699353e-01L, 5.874539448817e-01L, 5.873701556964e-01L, 
    5.872864023536e-01L, 5.872026848279e-01L, 5.871190030937e-01L, 
    5.870353571256e-01L, 5.869517468980e-01L, 5.868681723855e-01L, 
    5.867846335628e-01L, 5.867011304043e-01L, 5.866176628848e-01L, 
    5.865342309789e-01L, 5.864508346613e-01L, 5.863674739066e-01L, 
    5.862841486896e-01L, 5.862008589852e-01L, 5.861176047679e-01L, 
    5.860343860128e-01L, 5.859512026945e-01L, 5.858680547879e-01L, 
    5.857849422680e-01L, 5.857018651097e-01L, 5.856188232878e-01L, 
    5.855358167773e-01L, 5.854528455533e-01L, 5.853699095906e-01L, 
    5.852870088644e-01L, 5.852041433497e-01L, 5.851213130216e-01L, 
    5.850385178551e-01L, 5.849557578255e-01L, 5.848730329079e-01L, 
    5.847903430774e-01L, 5.847076883093e-01L, 5.846250685787e-01L, 
    5.845424838610e-01L, 5.844599341314e-01L, 5.843774193652e-01L, 
    5.842949395377e-01L, 5.842124946244e-01L, 5.841300846005e-01L, 
    5.840477094414e-01L, 5.839653691227e-01L, 5.838830636197e-01L, 
    5.838007929079e-01L, 5.837185569628e-01L, 5.836363557599e-01L, 
    5.835541892748e-01L, 5.834720574831e-01L, 5.833899603602e-01L, 
    5.833078978819e-01L, 5.832258700238e-01L, 5.831438767616e-01L, 
    5.830619180709e-01L, 5.829799939274e-01L, 5.828981043069e-01L, 
    5.828162491852e-01L, 5.827344285380e-01L, 5.826526423411e-01L, 
    5.825708905704e-01L, 5.824891732018e-01L, 5.824074902110e-01L, 
    5.823258415741e-01L, 5.822442272669e-01L, 5.821626472653e-01L, 
    5.820811015455e-01L, 5.819995900832e-01L, 5.819181128547e-01L, 
    5.818366698359e-01L, 5.817552610028e-01L, 5.816738863316e-01L, 
    5.815925457985e-01L, 5.815112393794e-01L, 5.814299670507e-01L, 
    5.813487287884e-01L, 5.812675245688e-01L, 5.811863543681e-01L, 
    5.811052181625e-01L, 5.810241159284e-01L, 5.809430476421e-01L, 
    5.808620132798e-01L, 5.807810128180e-01L, 5.807000462329e-01L, 
    5.806191135010e-01L, 5.805382145986e-01L, 5.804573495024e-01L, 
    5.803765181886e-01L, 5.802957206338e-01L, 5.802149568144e-01L, 
    5.801342267071e-01L, 5.800535302884e-01L, 5.799728675348e-01L, 
    5.798922384229e-01L, 5.798116429294e-01L, 5.797310810309e-01L, 
    5.796505527041e-01L, 5.795700579257e-01L, 5.794895966723e-01L, 
    5.794091689208e-01L, 5.793287746478e-01L, 5.792484138302e-01L, 
    5.791680864447e-01L, 5.790877924683e-01L, 5.790075318776e-01L, 
    5.789273046497e-01L, 5.788471107613e-01L, 5.787669501895e-01L, 
    5.786868229110e-01L, 5.786067289030e-01L, 5.785266681424e-01L, 
    5.784466406062e-01L, 5.783666462714e-01L, 5.782866851150e-01L, 
    5.782067571142e-01L, 5.781268622461e-01L, 5.780470004876e-01L, 
    5.779671718161e-01L, 5.778873762086e-01L, 5.778076136423e-01L, 
    5.777278840945e-01L, 5.776481875423e-01L, 5.775685239630e-01L, 
    5.774888933339e-01L, 5.774092956323e-01L, 5.773297308354e-01L, 
    5.772501989207e-01L, 5.771706998654e-01L, 5.770912336470e-01L, 
    5.770118002428e-01L, 5.769323996303e-01L, 5.768530317869e-01L, 
    5.767736966901e-01L, 5.766943943174e-01L, 5.766151246462e-01L, 
    5.765358876542e-01L, 5.764566833188e-01L, 5.763775116177e-01L, 
    5.762983725284e-01L, 5.762192660286e-01L, 5.761401920958e-01L, 
    5.760611507078e-01L, 5.759821418423e-01L, 5.759031654768e-01L, 
    5.758242215893e-01L, 5.757453101573e-01L, 5.756664311588e-01L, 
    5.755875845714e-01L, 5.755087703729e-01L, 5.754299885413e-01L, 
    5.753512390543e-01L, 5.752725218899e-01L, 5.751938370259e-01L, 
    5.751151844402e-01L, 5.750365641109e-01L, 5.749579760157e-01L, 
    5.748794201328e-01L, 5.748008964401e-01L, 5.747224049156e-01L, 
    5.746439455374e-01L, 5.745655182836e-01L, 5.744871231322e-01L, 
    5.744087600613e-01L, 5.743304290491e-01L, 5.742521300737e-01L, 
    5.741738631133e-01L, 5.740956281460e-01L, 5.740174251501e-01L, 
    5.739392541038e-01L, 5.738611149854e-01L, 5.737830077731e-01L, 
    5.737049324452e-01L, 5.736268889801e-01L, 5.735488773560e-01L, 
    5.734708975513e-01L, 5.733929495445e-01L, 5.733150333138e-01L, 
    5.732371488378e-01L, 5.731592960948e-01L, 5.730814750632e-01L, 
    5.730036857217e-01L, 5.729259280487e-01L, 5.728482020226e-01L, 
    5.727705076221e-01L, 5.726928448257e-01L, 5.726152136119e-01L, 
    5.725376139594e-01L, 5.724600458468e-01L, 5.723825092528e-01L, 
    5.723050041559e-01L, 5.722275305350e-01L, 5.721500883686e-01L, 
    5.720726776355e-01L, 5.719952983144e-01L, 5.719179503842e-01L, 
    5.718406338235e-01L, 5.717633486112e-01L, 5.716860947261e-01L, 
    5.716088721471e-01L, 5.715316808530e-01L, 5.714545208226e-01L, 
    5.713773920350e-01L, 5.713002944690e-01L, 5.712232281035e-01L, 
    5.711461929176e-01L, 5.710691888901e-01L, 5.709922160002e-01L, 
    5.709152742268e-01L, 5.708383635489e-01L, 5.707614839457e-01L, 
    5.706846353961e-01L, 5.706078178794e-01L, 5.705310313746e-01L, 
    5.704542758609e-01L, 5.703775513173e-01L, 5.703008577232e-01L, 
    5.702241950577e-01L, 5.701475633000e-01L, 5.700709624294e-01L, 
    5.699943924250e-01L, 5.699178532663e-01L, 5.698413449324e-01L, 
    5.697648674027e-01L, 5.696884206566e-01L, 5.696120046733e-01L, 
    5.695356194323e-01L, 5.694592649130e-01L, 5.693829410947e-01L, 
    5.693066479569e-01L, 5.692303854791e-01L, 5.691541536406e-01L, 
    5.690779524211e-01L, 5.690017818000e-01L, 5.689256417568e-01L, 
    5.688495322712e-01L, 5.687734533225e-01L, 5.686974048906e-01L, 
    5.686213869548e-01L, 5.685453994950e-01L, 5.684694424906e-01L, 
    5.683935159214e-01L, 5.683176197671e-01L, 5.682417540073e-01L, 
    5.681659186217e-01L, 5.680901135902e-01L, 5.680143388925e-01L, 
    5.679385945083e-01L, 5.678628804174e-01L, 5.677871965996e-01L, 
    5.677115430348e-01L, 5.676359197029e-01L, 5.675603265837e-01L, 
    5.674847636570e-01L, 5.674092309028e-01L, 5.673337283010e-01L, 
    5.672582558316e-01L, 5.671828134746e-01L, 5.671074012098e-01L, 
    5.670320190173e-01L, 5.669566668771e-01L, 5.668813447693e-01L, 
    5.668060526740e-01L, 5.667307905711e-01L, 5.666555584408e-01L, 
    5.665803562631e-01L, 5.665051840183e-01L, 5.664300416865e-01L, 
    5.663549292478e-01L, 5.662798466825e-01L, 5.662047939706e-01L, 
    5.661297710925e-01L, 5.660547780285e-01L, 5.659798147586e-01L, 
    5.659048812633e-01L, 5.658299775228e-01L, 5.657551035174e-01L, 
    5.656802592275e-01L, 5.656054446334e-01L, 5.655306597155e-01L, 
    5.654559044541e-01L, 5.653811788297e-01L, 5.653064828228e-01L, 
    5.652318164136e-01L, 5.651571795827e-01L, 5.650825723107e-01L, 
    5.650079945778e-01L, 5.649334463648e-01L, 5.648589276521e-01L, 
    5.647844384202e-01L, 5.647099786497e-01L, 5.646355483213e-01L, 
    5.645611474154e-01L, 5.644867759128e-01L, 5.644124337941e-01L, 
    5.643381210399e-01L, 5.642638376309e-01L, }; 

// End of File
