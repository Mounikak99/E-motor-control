//#############################################################################
//! \file   input.c
//! \brief  Input Vector (64) 
//! \author Vishal Coelho 
//! \date   22-Jul-2016
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


#include "dsp.h"

float64_t test_input[256] = {
     1.125541798361L, -2.052159867208L, -0.656497261735L, -0.685257998008L, 
    -0.832920102136L,  2.082120331303L,  3.066081352433L,  1.906094688038L, 
    -2.904472363724L, -2.761641029122L,  2.420081969933L, -0.632982095493L, 
     2.596757723074L,  0.168865823368L,  1.860978162734L, -0.522764360647L, 
    -2.521365114703L,  0.985579762252L, -1.496207478720L,  0.804080330174L, 
    -1.034483484073L, -1.307002572444L,  1.129264023384L, -0.429448362856L, 
    -2.283603987309L, -3.044284173343L,  1.390013368633L,  3.041462080758L, 
    -2.470788093372L, -2.091242556589L,  0.966085914025L, -2.474215675752L, 
    -0.036606235707L, -0.801673246622L,  1.753333687350L, -1.896778017650L, 
     1.351117811506L, -0.064794481300L,  2.536651094294L, -1.008492626689L, 
     2.456238535457L,  2.837677900566L, -1.041984270427L,  2.641024096838L, 
     1.248756893589L, -2.810613315736L, -1.898716856941L,  1.494506490946L, 
    -2.949698228501L, -1.450665427766L,  1.533563806602L, -0.484838130014L, 
     0.000140966971L,  0.300781743155L, -0.126152907751L,  2.781798514554L, 
     2.542944819714L, -0.516829035186L,  0.690312511118L,  3.035108159920L, 
     0.739319730211L, -1.247495349066L,  2.258442613615L,  1.263540748369L, 
     1.919446663704L,  1.045137828288L,  0.482055499655L,  0.245838830279L, 
    -1.992256881409L,  1.244733693668L, -1.634055370062L,  1.046325738726L, 
     2.428526098953L, -2.022353433370L, -2.961427640131L, -2.337254458161L, 
    -0.063451447323L,  3.135814603466L, -2.086475279162L, -2.066407283710L, 
     3.007639224657L, -2.936755657030L,  1.336398779371L,  0.384529638356L, 
     0.002963301960L,  2.399337984943L, -0.181657100285L,  1.062959787788L, 
    -2.766996260783L, -1.945065147247L,  1.143363194469L, -0.823621631786L, 
    -2.874989953878L, -0.246766213959L, -2.692687560146L,  3.026220496919L, 
     0.136029972074L, -2.158871355790L, -2.533819976840L,  2.233815670058L, 
     1.998986319111L,  0.909582411063L,  1.995207223290L, -0.777404630466L, 
     1.397629178494L, -1.941983696889L, -2.199960307358L, -0.450799740346L, 
     1.002829380022L, -0.112958721978L,  0.116835469570L, -2.383767537046L, 
     2.971786773162L,  0.562392112719L,  0.936141157905L, -1.720413547503L, 
     1.887032658351L, -0.724959422492L, -0.290297557685L,  0.521418820776L, 
    -0.424796710068L, -1.559448124614L,  2.044006859493L, -1.316700139197L, 
    -2.617136339275L,  0.735703724424L, -2.304854535250L, -1.474783538796L, 
     2.038116192857L,  0.641956106401L,  3.032663581446L, -0.711437566935L, 
     1.446695828572L,  2.613750073042L, -0.980949713858L, -3.134360348348L, 
     0.528223199640L, -0.235938890921L, -2.464459960441L, -0.475329001506L, 
     2.552909402350L, -0.245569714718L,  2.385434703481L,  1.697463637385L, 
     1.996548477839L, -1.115442532694L, -1.503390320754L,  1.789069753217L, 
     0.592857807815L, -0.179968310961L, -3.000141861658L, -2.916888773369L, 
    -0.469609541079L, -2.036541109058L, -1.176721938641L,  1.393346817152L, 
    -2.126954080797L, -0.166592419435L, -2.018371575567L, -2.182017050901L, 
    -0.484523505220L, -0.998243534661L, -2.549532255985L,  0.674746330099L, 
     0.619042467942L, -1.936821681751L, -0.182688285245L,  1.498080017780L, 
     1.231185846289L, -1.615723625583L,  1.255932401753L,  2.622754492824L, 
     0.870414424966L, -1.451028845398L, -2.930453524552L,  1.668185803492L, 
    -2.709271182567L, -1.956194492990L, -1.133488293325L, -1.335188356740L, 
     0.193925994974L, -2.569109877268L,  0.970411001736L,  0.478837660851L, 
    -0.580445703816L,  1.152105236146L,  2.010501317557L,  0.292753173010L, 
     1.371989703642L, -0.466659449502L,  2.944610585928L,  0.907560762018L, 
     0.196876741350L,  0.927508924982L, -1.098642082882L,  1.124795439057L, 
    -2.477904795224L,  0.853173064412L,  0.697173814338L,  2.797111446626L, 
     1.751766149438L, -1.828815818846L, -0.480959494881L,  1.314955719532L, 
    -2.570933118780L, -1.657312163116L, -1.467302897944L, -2.391403903697L, 
    -2.176139023271L,  0.674210543518L, -1.375984265469L, -0.313294089806L, 
    -0.376456174306L, -0.259335371867L,  0.170542876227L,  1.017528885748L, 
    -0.267510599955L,  1.698253975358L,  2.358529313082L, -0.941107977227L, 
     0.113424742018L,  1.017936328032L,  2.787363156495L, -0.526791115635L, 
     0.865251781672L,  2.148404228286L,  2.875775837998L,  2.091778066126L, 
    -1.629185744926L, -1.530326379057L,  1.106609071903L,  0.712894834484L, 
    -1.325346384019L,  0.516786742085L,  1.079502540584L,  0.255972804444L, 
     1.226103919619L,  2.324408059035L, -2.714381489745L, -1.477936964789L, 
    -1.540698885246L, -1.143074295932L, -1.733907623695L, -2.392545600836L, 
     1.054524124436L,  2.763530065736L,  2.163879737800L,  0.914529402260L, 
    -0.977271492027L, -0.129036363258L,  1.762556960417L,  0.875354282648L, 
     1.101643859379L,  0.280959608656L, -3.099399089331L,  0.925585328557L, 
};  

// End of File
