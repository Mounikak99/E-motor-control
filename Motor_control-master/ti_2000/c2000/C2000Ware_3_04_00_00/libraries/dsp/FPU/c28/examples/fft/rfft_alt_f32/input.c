//#############################################################################
//! \file   input.c
//! \brief  Input Vector (1024) 
//! \author Vishal Coelho 
//! \date   22-Aug-2016
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

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_1")
#else
#pragma DATA_SECTION(test_input, "FFT_buffer_1")
#endif

float test_input[256] = {
       0.000000000000F,   28.181947493983F,   38.979458543369F,   28.366989161081F,
       7.326272789999F,   -8.933215669711F,  -13.582694700454F,  -11.190830433269F,
      -9.778131637521F,  -11.220194912225F,  -10.091846750546F,   -1.556763928905F,
      10.877804607379F,   16.890687568601F,    9.470600186751F,   -6.578676281180F,
     -17.191145663469F,  -11.385074517232F,    7.957471898890F,   25.477057400908F,
      26.231864181194F,    8.497010773015F,  -15.114117034451F,  -28.803194057778F,
     -26.330289062628F,  -13.895720526344F,   -1.820659738631F,    5.576678064525F,
      11.696466129355F,   20.231548950995F,   27.674677584129F,   24.848159467490F,
       6.765710093676F,  -19.326270643355F,  -37.061124051535F,  -33.825464341682F,
     -11.912506489176F,   12.729463038734F,   23.526974088120F,   16.454482671904F,
       1.682755942300F,   -7.014881177877F,   -4.541976182728F,    2.901152450406F,
       6.136812160258F,    2.871787971451F,   -1.051128232820F,    0.361570562929F,
       5.234287759679F,    4.978181079424F,   -5.981884268498F,  -22.031160736430F,
     -29.103652152645F,  -17.028591364932F,    9.875806390724F,   34.517775918634F,
      40.323847064870F,   24.411954570794F,   -1.022134150775F,  -19.762475540052F,
     -24.411954570794F,  -19.539237374044F,  -13.733166227808F,   -9.875806390724F,
      -3.756018325894F,    8.319042461819F,   22.031160736430F,   26.766493959325F,
      15.806428611402F,   -5.234287759680F,  -21.146180253756F,  -19.733481458007F,
      -2.871787971450F,   14.647797530568F,   17.883457240420F,    4.541976182727F,
     -13.769728512950F,  -22.467365633126F,  -16.454482671904F,   -2.742364397293F,
       8.055146652093F,   11.912506489177F,   13.040854650856F,   16.276514360708F,
      19.326270643355F,   14.018899597150F,   -4.063549776664F,  -27.674677584130F,
     -41.016158641822F,  -32.481075820181F,   -5.576678064525F,   22.605269429458F,
      34.680330217171F,   26.330289062628F,    8.018584366951F,   -5.670492656376F,
      -8.497010773015F,   -5.447254490368F,   -4.692447710082F,   -7.957471898890F,
      -9.399535173594F,   -3.593464027358F,    6.578676281181F,   11.314009504076F,
       3.893922122225F,  -10.877804607379F,  -19.227845761921F,  -10.692762940280F,
      11.220194912225F,   30.562741328347F,   31.975440124095F,   13.582694700453F,
     -11.851394021116F,  -28.110882480826F,  -28.366989161081F,  -18.194848852542F,
      -7.397337803156F,    0.000000000000F,    7.397337803156F,   18.194848852543F,
      28.366989161081F,   28.110882480826F,   11.851394021114F,  -13.582694700454F,
     -31.975440124096F,  -30.562741328347F,  -11.220194912224F,   10.692762940281F,
      19.227845761921F,   10.877804607379F,   -3.893922122226F,  -11.314009504075F,
      -6.578676281180F,    3.593464027358F,    9.399535173594F,    7.957471898889F,
       4.692447710081F,    5.447254490368F,    8.497010773015F,    5.670492656376F,
      -8.018584366952F,  -26.330289062629F,  -34.680330217170F,  -22.605269429457F,
       5.576678064526F,   32.481075820182F,   41.016158641822F,   27.674677584129F,
       4.063549776663F,  -14.018899597151F,  -19.326270643355F,  -16.276514360708F,
     -13.040854650856F,  -11.912506489177F,   -8.055146652093F,    2.742364397294F,
      16.454482671905F,   22.467365633127F,   13.769728512949F,   -4.541976182728F,
     -17.883457240421F,  -14.647797530568F,    2.871787971451F,   19.733481458007F,
      21.146180253755F,    5.234287759678F,  -15.806428611403F,  -26.766493959325F,
     -22.031160736429F,   -8.319042461818F,    3.756018325895F,    9.875806390724F,
      13.733166227808F,   19.539237374044F,   24.411954570794F,   19.762475540051F,
       1.022134150774F,  -24.411954570795F,  -40.323847064871F,  -34.517775918634F,
      -9.875806390723F,   17.028591364933F,   29.103652152645F,   22.031160736429F,
       5.981884268498F,   -4.978181079424F,   -5.234287759679F,   -0.361570562929F,
       1.051128232819F,   -2.871787971451F,   -6.136812160259F,   -2.901152450406F,
       4.541976182728F,    7.014881177877F,   -1.682755942300F,  -16.454482671905F,
     -23.526974088120F,  -12.729463038733F,   11.912506489178F,   33.825464341683F,
      37.061124051534F,   19.326270643354F,   -6.765710093677F,  -24.848159467491F,
     -27.674677584129F,  -20.231548950995F,  -11.696466129355F,   -5.576678064525F,
       1.820659738631F,   13.895720526345F,   26.330289062629F,   28.803194057778F,
      15.114117034450F,   -8.497010773016F,  -26.231864181195F,  -25.477057400908F,
      -7.957471898889F,   11.385074517233F,   17.191145663468F,    6.578676281180F,
      -9.470600186751F,  -16.890687568600F,  -10.877804607378F,    1.556763928906F,
      10.091846750546F,   11.220194912224F,    9.778131637520F,   11.190830433269F,
      13.582694700454F,    8.933215669711F,   -7.326272790000F,  -28.366989161082F,
     -38.979458543369F,  -28.181947493981F,    0.000000000002F,   28.181947493984F,
      38.979458543369F,   28.366989161080F,    7.326272789998F,   -8.933215669712F,
     -13.582694700454F,  -11.190830433269F,   -9.778131637521F,  -11.220194912226F,
     -10.091846750546F,   -1.556763928904F,   10.877804607380F,   16.890687568602F,
       9.470600186750F,   -6.578676281181F,  -17.191145663469F,  -11.385074517232F,
       7.957471898890F,   25.477057400908F,   26.231864181194F,    8.497010773013F,
};
// End of File
