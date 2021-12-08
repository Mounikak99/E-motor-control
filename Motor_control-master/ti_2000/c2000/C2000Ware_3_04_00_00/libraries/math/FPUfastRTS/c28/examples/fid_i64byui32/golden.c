//#############################################################################
//! \file   golden.c
//! \brief  FID Ouput Vector (256) 
//! \author Vishal Coelho 
//! \date   11-Apr-2016
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

#include "fpu64/fid.h"
#include <stdint.h>

const int64_t test_quotient[256] = {
              -310897929L,           -712767079L,          -1332201441L,         -42125352010L, 
             -1629602525L,          -6153283674L,          -1762739935L,          10608223567L, 
               207102316L,           6457274442L,          -1841759335L,           6108323576L, 
             -5829722504L,          -1984355585L,           4055344126L,            580746497L, 
              -407018741L,          -6941731309L,          -2552746946L,          -2476460366L, 
             -3945095415L,         -11469590395L,          -1932425616L,           3114913249L, 
             26096313914L,          -1337150325L,           3142029170L,          -3225268902L, 
               754669124L,           3636620637L,          -2070554998L,          -3207250972L, 
             -2836882250L,           1791225951L,          -1726804321L,           6361032353L, 
               197728633L,            359432002L,           -974335049L,           -649781968L, 
            -40943560466L,           2893616550L,          -4226486121L,           -673641033L, 
              1661176990L,            502676613L,         -28053642838L,           2212993241L, 
             -3080117629L,         -10211599796L,           6605100670L,           5066623783L, 
              1403951500L,           -206827495L,         -22356532799L,           1959375899L, 
              6544789040L,           -851735032L,           4784950129L,         -17864428136L, 
             -1076877207L,           4946153900L,          -4210701943L,           2047874538L, 
             -2329160798L,          -7247950208L,          -3276273085L,           -683794502L, 
              1730666864L,           3392328942L,            547556176L,           2648657871L, 
               314457692L,           1487433476L,         -10586290915L,           -633930430L, 
             -2894459853L,           -956757025L,           2928797586L,          -1972059177L, 
               622295413L,         -14864275251L,            329411862L,           5089081802L, 
             -3242247968L,           4242261371L,          -3132645058L,          -4954501615L, 
              2717094700L,          -1775365373L,          -1978892966L,           1132913258L, 
             -5557512934L,           3481230281L,            252697651L,           -605741849L, 
             10166763020L,           3565084091L,          -1180102724L,           1342511939L, 
               784419165L,           5234532964L,           3773695788L,         -53299460249L, 
             25459724876L,          -3650859234L,           1636259859L,           6361323077L, 
             -1515994928L,            910053787L,           1091243949L,           -677649101L, 
               -31993804L,           2607305597L,           2189445711L,           1163513763L, 
              -829556417L,          -3016912061L,            389280428L,            641619426L, 
             -2146549901L,           2584263067L,           1810471593L,            856299136L, 
            -18455850056L,          -1836635462L,          -4285201335L,          -1875784421L, 
             -1903935474L,           2085380235L,         -52109759381L,         -21984180101L, 
                40117267L,           5054865189L,          -4375136098L,           1105649978L, 
              5989299401L,           2233078370L,          -1294943578L,          -1524983263L, 
             -1460019052L,          -1533529602L,          -1786158674L,          -3373906177L, 
              3100503065L,         -11811363878L,          -8131606742L,           1235038931L, 
            -18837948314L,          -7445072676L,           2587626256L,           8117287922L, 
             -1815309151L,           2300236569L,          -1881969047L,           3092484525L, 
               295379851L,          -2920664852L,          -8864611855L,           -177352348L, 
             -3371994917L,         -12133767896L,            276318447L,           1315972664L, 
            -34476181280L,           8483305214L,          -1422482822L,            462959692L, 
               754439078L,          43414284514L,          -5909012573L,           3285060324L, 
             -5019278189L,           -755065204L,          -2300961945L,            528086037L, 
             -1520590812L,          -7682751461L,         -11306283745L,           1858435290L, 
              1353737001L,           3761544793L,           7900970406L,           -704214791L, 
             -1596837911L,           4053722816L,           -924137389L,            377748282L, 
              7682606716L,          -2172155133L,          -2306106884L,          -1253461123L, 
               349388607L,          12028932011L,          -2361096785L,           2063828368L, 
              -437357401L,           1990175209L,            230764296L,          -1254213583L, 
              -322395243L,           2487585841L,           7814215645L,           3955275271L, 
             -5734456154L,          -3672037130L,           2692947825L,           1229791736L, 
             -5122663869L,             23189543L,           5640658800L,          -1226890125L, 
             -1725110718L,         -18476498384L,          -5551421767L,          -4338753778L, 
              4121486268L,          -1117378511L,          15750274807L,           1559060883L, 
             -3353128462L,           8658199422L,           1301245748L,           -181598244L, 
             20609876436L,          -1771844832L,            651835737L,          -2481568285L, 
             -2325459487L,           1305003755L,           1783732977L,            742823851L, 
              1550818552L,          -5191226701L,            544123124L,          -2740801106L, 
             -1601048249L,          -2081091451L,           1122488240L,           6187717563L, 
             18480014574L,          25017946018L,            222366127L,          -2271520344L, 
             -2852314532L,          -1754367651L,           2229613237L,          -2849460134L, 
               265217066L,         -13364265627L,           -843173419L,         -10609419862L, 
             -2182140076L,           3326985708L,           4970164688L,           8043944216L, 
}; 

const int64_t test_remainder[256] = {
             -1938700111L,          -3142218530L,           -745181005L,           -102546964L, 
              -525293143L,           -553710304L,           -113572531L,            219104668L, 
               971782492L,            981392976L,          -3159780161L,            400391864L, 
              -185116296L,          -1572055335L,            169523494L,           1093665888L, 
              -651834156L,           -191515829L,          -1355615432L,           -328152628L, 
              -880051498L,           -189051102L,           -134755600L,            393442293L, 
               235063614L,          -1252258943L,            756621930L,           -619607402L, 
              1327246756L,            458361806L,          -1976515182L,          -1884517064L, 
              -282306306L,           3115426548L,          -1573570675L,             41372683L, 
               323896096L,            656674454L,           -244431876L,           -933863872L, 
               -37188650L,           2852609944L,          -1460367767L,          -1122650295L, 
              1867160760L,           3183653435L,            -32024320L,           1733531300L, 
             -1880179604L,           -286427348L,            884624996L,            203545940L, 
              1594120084L,           -467071949L,            -67456260L,           1219538349L, 
               897354960L,           -311394856L,             48629646L,           -119710096L, 
              -892967318L,            303158104L,          -1366452971L,            515749336L, 
              -710042858L,           -329593984L,          -2139515278L,          -1435123354L, 
              2794910560L,           1384342638L,           1113924960L,            340413173L, 
              1832666092L,            379018812L,           -355728802L,            -13176558L, 
              -488253091L,           -722201267L,           1661480230L,          -1883077890L, 
               538327606L,            -62459233L,           1040848066L,            764871980L, 
              -144289408L,           1660979899L,           -688764748L,           -788583106L, 
               151216268L,           -871873411L,           -991259182L,           1454701912L, 
              -623023500L,           1925965024L,            448155814L,          -1048802198L, 
               139750044L,            579606726L,           -723432388L,            915675087L, 
               316945927L,             40491076L,            198730460L,             -6467270L, 
               252442216L,           -198366298L,           2641746776L,            562506620L, 
             -2190727936L,           1901924404L,           2342378361L,          -1959938789L, 
              -467566624L,           1875840185L,           1505741080L,            846119325L, 
             -1462260272L,           -222104617L,           1070895304L,           1404075848L, 
              -802965182L,           2506669011L,           1239434245L,            541261440L, 
              -188140600L,          -1908846590L,           -262328218L,          -2557858712L, 
             -1042233056L,            676931461L,            -84820677L,           -140220334L, 
              1150192817L,             22680307L,           -593417744L,            184749046L, 
               327266896L,           1792353274L,          -1178811024L,           -473924464L, 
             -3207831116L,           -299099408L,           -760614314L,            -60615433L, 
              1643285715L,           -129162002L,           -204201264L,            567048424L, 
              -367830338L,           -339176876L,           1584996720L,            122359462L, 
             -1044053036L,            763421649L,           -717164566L,             96019715L, 
                95301830L,            -35502968L,           -596473472L,          -1515361132L, 
             -2344972574L,            -14791752L,            895978844L,           1520907336L, 
                -4350912L,            245643666L,          -3916868188L,           2259204552L, 
              2185996240L,             72358688L,           -381071172L,            919708268L, 
              -192923923L,           -292820380L,           -779518696L,           1291549695L, 
             -2336564712L,           -281156623L,            -47377126L,            280522868L, 
              2207811491L,           2221167127L,            119604114L,          -2744194256L, 
             -2125389163L,            405749056L,          -1457825517L,            204917936L, 
               141877640L,          -1821677776L,          -1904624504L,           -859845699L, 
              2875408151L,            218691867L,          -1587722242L,             21848592L, 
             -1734858063L,            990543463L,             26543400L,          -1735324304L, 
             -3246356194L,            911324443L,            353369088L,           1797166151L, 
              -630303870L,           -283571232L,           2138892868L,           1245127248L, 
              -589312221L,           2627239441L,            855811600L,          -1209797814L, 
             -2150854900L,           -248672288L,           -504584102L,           -173896714L, 
               374285244L,           -797961485L,            138203355L,            792270466L, 
             -1947380290L,            458742806L,           1959058284L,           -320259140L, 
               305101372L,           -933411648L,           1824083033L,          -1562893982L, 
             -3477364227L,            929669985L,           1459235836L,           1335618752L, 
               361854624L,            -74458607L,           1687318188L,          -1077250790L, 
             -2360343161L,           -364490214L,           1574778672L,           1067909063L, 
               141981822L,            160670516L,           2635225650L,           -863185912L, 
              -778718824L,          -2176192260L,            562342105L,          -1286534120L, 
              1942114228L,           -344891382L,           -461402584L,           -114156662L, 
              -733120328L,            704805228L,            307560176L,            917797792L, 
}; 


// End of File
