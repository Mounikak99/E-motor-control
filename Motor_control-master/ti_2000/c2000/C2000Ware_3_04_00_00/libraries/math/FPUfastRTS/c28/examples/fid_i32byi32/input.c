//#############################################################################
//! \file   input.c
//! \brief  FID Input Vectors (256) 
//! \author Vishal Coelho 
//! \date   30-Mar-2016
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

const int32_t test_dividend[256] = {
#if SIGNED_DIV_TRUNCATED == 1
      1351727940,   1742863098,  -1602079425,   1775435782, 
       568478633,  -1728550799,   -951342906,    201354591, 
      1964976895,   1996681054,  -1470540617,   2021180606, 
      1963517091,    -62811111,   1289694793,  -1538086464, 
      -336032733,   1785570484,   1255020923,   1973504943, 
       668901209,  -1994103156,   1499498950,   1863986805, 
       767661644,   1106985431,   1044245998,   -462881427, 
       667772453,  -1412242423,    884961209,  -2010762614, 
      -958108484,  -1949179035,  -1730305824,   1389240793, 
       836782563,   -785551752,   1933688975,  -1999538859, 
      -263090972,   -508702554,   1140385921,   1267873920, 
     -1344871923,    -43961586,   -233705489,    628409593, 
       899215101,   1093870969,   -961964970,    771817119, 
       666140854,  -1449071564,  -1636392498,     -7026344, 
      1974585265,   -685538084,    366222201,  -1186218688, 
      1079183802,  -1051858470,     25585342,    855028013, 
      1678916685,   1972641650,    202789157,  -1552096200, 
     -1506270777,  -1041494118,   1463369471,  -1055350006, 
      1349843049,  -1101551872,   1843673222,   -644314819, 
     -1303113477,  -1069086690,    498408088,   -114723521, 
      -637117566,   1420898137,    366206482,    213561271, 
      1791833141,   -919814411,   1104666572,   1089758161, 
      -513481178,    291291728,  -1821691955,  -1915769653, 
       132274482,   1199014123,   1864061694,  -1589540731, 
       295595372,   -131466196,  -2096364649,   -699552916, 
     -1450915938,   1263942477,   -810825221,    122548883, 
     -1436027772,    438009103,  -1018030581,    661764688, 
       812670102,   1065802975,   -212422217,  -1787473571, 
     -1164035056,   1775270449,  -1493025040,   1399373262, 
       164679505,   2130882381,  -1811722309,   -246194957, 
     -1689413488,   1983837150,  -2127579807,   1180731454, 
      1362806955,   1583531701,  -1784834453,   -430430245, 
     -1031348767,   1288784308,   -294575368,   1763717987, 
     -1366456609,  -1014458749,  -1522398487,  -1563073638, 
      1586097954,    342328595,    214147936,  -1524907530, 
      1516257104,    524222797,   -640154650,     56906340, 
      -421731284,  -1821209192,  -1117051615,  -1617832856, 
     -1357605712,  -1116895398,   -355335233,  -1934219494, 
      1729652521,   1910346433,    -39238425,    -46159567, 
      -696989828,   1718218186,   -561580799,  -1669871451, 
      1203673467,   -473568090,  -1109427480,   -412694193, 
     -1733214617,  -1580662672,   1898592830,   1959082932, 
       323018456,  -1890732466,  -1139111599,   -630679135, 
      1379517897,  -2081326387,  -1962697827,  -1421676998, 
       640446088,    995240067,    634564079,   -210781076, 
       201901654,   -874795479,   1050947603,  -1335928038, 
       802194377,  -1359309236,   -564854357,    539527609, 
      1203567669,  -1799051124,   1844198702,   1184176937, 
       -56729507,   -275485265,   -228562056,   -831722685, 
        36544396,     46263515,   1364200619,   1266291293, 
       619841649,   -521368732,   1338227878,    140984830, 
      -641122209,   1885497351,   1614662080,    215419852, 
       526026488,    373854159,  -1255237295,   -853640512, 
      -124883268,  -1157544538,   1478795004,  -1310977394, 
     -1177156987,  -1414298169,  -1169672935,   -276172049, 
      -811309501,   1818401716,   -299756972,  -1353703597, 
      1738950518,   2060503594,   -262551467,  -1670230217, 
     -1039104219,   -392045276,    407575534,  -1021292767, 
       441707705,    907164869,  -1195088678,  -1643178678, 
      -873270475,   -778341267,   -325701287,     33751075, 
     -1780196096,  -1020131034,   1292847960,  -2021983512, 
      1841914503,    989263522,    -48924085,    337262568, 
     -1128358433,   -176742938,   1988950130,    201029031, 
        90777702,  -1152793331,    -47683827,    532834021, 
#elif SIGNED_DIV_MODULO == 1
     -1855109397,  -1526965500,   1623936882,    -22543452, 
       443398452,     22101929,   -754897140,    604099085, 
     -2002375771,  -1888458034,   1694095428,   -486007894, 
     -1581977277,   1260262229,   1329385206,  -1120379832, 
      -110498682,  -1599941553,  -2094097684,   -391235496, 
     -1233322887,   1233897014,  -2112329953,   1485627267, 
      1380807886,    877396031,   1138717105,   1146315096, 
      1377087903,    446650404,  -1789093292,     57904359, 
       131096044,    174836679,   -534766491,   1261265889, 
      -655865218,     72325643,   -826726811,   2143698835, 
       192229994,     14156899,  -1857054898,   -270550604, 
     -1404261078,  -1955837747,  -1346950882,   -446258191, 
     -1480181995,    741027167,  -1371787492,   1593609452, 
      1150617345,   -139056370,   -341740050,   1774315615, 
      -842228309,   -129540147,   1477248658,  -1364829547, 
      -518647967,  -1852434557,   1603945366,  -1742479439, 
        90371133,     94403269,    160935071,     -3815184, 
       223951289,  -1319674445,   -377048400,  -1259861057, 
     -1376658655,    709067953,    447542528,  -1999691641, 
      2077521475,  -1105098119,  -1010934112,   1238855986, 
     -1992767124,  -1769857405,   1243080521,  -1877612297, 
      -996776601,  -1012887381,  -1266016349,   -756302398, 
     -2046002048,  -1232512134,   1999573130,  -1658203932, 
     -1120087640,  -1191006577,    652155344,   1901364743, 
      1625064540,  -1607075242,  -1849134766,  -1975822130, 
      -639089258,   1070113724,   1954461471,   -421778082, 
       860729895,     61803042,     43928706,  -1748218329, 
       437105490,  -2101782579,   1693883977,  -1905506038, 
     -1340051527,  -1136763426,  -1870107796,  -1490075228, 
      -500050930,  -1373205425,  -1990552937,  -1437495017, 
     -1807941195,   1366305066,  -1365468588,   1493726835, 
     -1136554496,  -1057692031,   1464057061,  -1093570135, 
     -1250932063,   1177959194,   -855250024,   1207573653, 
      1696476335,  -1343879239,   1443349789,   1582229584, 
      1005289841,   -363002068,   -125693686,    605600402, 
      1098055748,   1750839294,   -967195246,   -578461145, 
      1176973837,    588507798,  -1941777143,   -892600403, 
       210711177,   1734172110,    886116801,   2087331258, 
      -458681235,    268887622,   -903099579,  -1117031067, 
       925704243,     73309465,  -2037551032,   1742696872, 
     -1202684830,   -995715843,   1311887688,  -1873647533, 
     -1309448458,   -300069631,  -2035901182,    606020597, 
     -2038095555,    300911111,   -741826214,   2075349564, 
       812870611,  -1178288913,   -765493944,   -605363487, 
      1412388989,   -855723377,    414318143,     74518472, 
       161990898,   -587849249,     36546516,     69807809, 
       818349434,   2095101135,   1007316637,    893235111, 
       540684600,   -237151436,   -309986875,    -51560855, 
      -226009272,    512908143,   1561982246,    701041872, 
       704840191,    553366771,  -1779822804,    -23312416, 
      -429617359,  -1007826940,   1740182062,   1293809934, 
     -1582527524,  -1375432626,  -2134534936,   1919998013, 
     -1574802961,    334028551,    449044904,   1231641739, 
      1559675409,    160541582,    431564754,  -1570310727, 
     -1522590141,   -127724171,    969269296,   1045113167, 
      -658146572,  -1897308521,   -357930471,   -112374639, 
     -1903351357,    549755987,   -522324793,   -727859523, 
       536083581,   -721717946,   1717874910,   1243761246, 
     -2041704956,   1492445605,    333260246,  -1747875410, 
       964509312,   -186693844,    -20505909,  -1379929502, 
      1368630088,   1269614687,   -755902716,   -870772342, 
      1102042488,    590174665,   -437737671,   1824702559, 
      2073430876,    755733947,   -113455223,  -1671863267, 
      2024716628,   -151373534,    719483717,  -1973556181, 
#else //SIGNED_DIV_EUCLIDEAN == 1
       188488651,    949388006,     96616602,   2120445213, 
     -1208274664,  -1693083525,  -1676336626,  -1874360789, 
      -409825798,   -221736655,   -576315132,   1131743814, 
       549310767,   1168146860,   1859091927,   2030406506, 
     -1322728167,  -1551023489,    842957498,  -1744529701, 
       109111083,    130327425,   1551083678,    -65054438, 
      -457601445,    736291138,   1036194976,     86124691, 
      -654069097,  -1503250348,    369762613,  -1021578082, 
     -1956554776,   1094930045,  -1104728476,   -247380182, 
       806578043,   -604610233,   1015072889,   -452227951, 
       787765150,    876377039,   -247796363,  -2063398395, 
      -726459873,   -325088236,   -986681019,  -1301144030, 
      1381781967,   -300985255,   1665463566,   -467365476, 
      1155837492,   -443277059,   1325057951,   1095547797, 
      -526582126,  -1219689469,   1247289503,   1929745606, 
      -740600822,    735574869,   -263517794,   1432374150, 
      1154720221,  -1429135140,   1554694317,   2103984878, 
        61948273,   1650474426,    378069028,  -1482827372, 
     -1289079360,   -399625932,   1068182925,   1398371840, 
      1245381730,   -779432431,    146304312,  -1761148425, 
     -1667711130,  -1562111608,    767305806,    -20714546, 
     -1332683659,    -21449819,  -1513511162,  -1911371485, 
      1506299466,    260101189,   1845156032,    844679194, 
       355584487,   1354620708,   1627852324,   2099859401, 
     -2145240063,   1569546796,    483469304,   2104320109, 
       118884992,    -87946391,   1294278110,  -1168905691, 
        -8184957,   1721648328,    320667494,   1482529015, 
      1024952249,    369311506,  -1087766928,    714752210, 
     -1788927694,    540993157,    691251612,    986776704, 
      1678267560,   2071476568,   1155471122,    349810001, 
      1839590594,    343985501,  -2074542483,  -1628395743, 
      1557830674,    -67445971,   1481143843,  -1248095661, 
       224589601,    557844891,  -2010083282,    492690383, 
      -590938270,  -1934742841,    -44796556,  -1320657793, 
     -1618842978,  -1264892905,  -1518206899,  -1335424842, 
     -1964292938,    580670631,   -936874720,    165771469, 
       838218871,     -3796694,    153764363,   -235436513, 
     -1615198569,    -41415110,   1516115532,   1606005978, 
      -986578331,  -1252148930,    279085131,    602634700, 
      -356357940,  -1262825545,   1923858106,  -1794990498, 
     -1693465118,  -1537421675,  -1432541499,    519513419, 
       316581029,  -1923810812,   1851995844,    982094444, 
      1021522124,  -1875163391,   1548080430,   1865755778, 
      2080474908,   1541630478,   1226466519,     57455575, 
     -1384686889,   -435554795,  -1572253305,  -2014814046, 
      1886099265,   -853383955,   -878175494,   -717533206, 
      -141441060,    636507308,  -2039129434,   1469766208, 
       253542850,   1520847701,   -653353886,   -231813782, 
     -1914526836,  -1386812583,    699255301,   -726583933, 
      1711484929,  -1640010935,   2097739030,    171721807, 
       888703548,   2145300172,   -911180126,   -367122900, 
      -151011301,   1133687019,   1366675940,  -1717035411, 
     -1382477156,   -602863456,  -1903938863,     93998252, 
      -705023286,  -1392990911,  -1250064517,   1740121285, 
       753299370,   -135428051,   1770095498,  -1700757336, 
      1054612355,   1014760994,    265692798,  -1356376023, 
       417519570,   -859264085,  -1571430038,  -1234367015, 
      1696261579,  -1840596154,  -1106011808,  -1916610292, 
      -250301859,  -2090432736,   1705923862,  -1302843148, 
     -1746460332,   -827352867,   -188730884,  -1710816927, 
      2127682678,   -721155789,   -870388798,  -1881001452, 
      -866535543,  -1948406481,     23313694,   1122815633, 
       562941359,  -1761401948,  -1800182185,   1190739037, 
      1740040476,    145049428,  -1678669877,   1399338388,
#endif
}; 

const int32_t test_divisor[256] = {
#if SIGNED_DIV_TRUNCATED == 1
       769381289,   -448758732,   -569355260,   2095866744, 
     -1985396452,   1654283998,   1775053408,   1272100050, 
     -1723517640,  -1022755478,   -707136988,    771925673, 
     -1560992389,    950164871,  -1688944944,    660382783, 
       -25022752,   1198518024,    923577218,   1733966603, 
      1678999370,   -712264265,    853606849,  -1297896912, 
     -2016311282,   1048290965,        96360,    -86233748, 
      1738268776,    471873661,    505373295,   1543792947, 
      1312067087,    329516400,  -1361837624,  -1116983510, 
      1660056111,  -2024329101,    -43373207,  -1426242049, 
      2055917734,    913515799,      2025610,   -124174486, 
     -1891422562,    781563376,  -1965243300,  -1840627714, 
        92985365,  -1732031351,   1366437633,   1363854375, 
       955370774,  -1503816474,    685499341,     79864669, 
      2031410244,    639913588,   1289909998,   -198437330, 
      -290376281,   1397212111,  -1788983523,  -1575518526, 
     -1402785225,   -468418573,   1423265157,   1302940140, 
     -1887761911,   -432684581,    115430813,   -357343565, 
       673708101,    549641392,   -893421574,   -293555989, 
     -2080966950,   2079037864,  -1429500795,  -1691287920, 
      -547995994,  -1296571589,    -44291258,   -689370541, 
      1939738075,   1805312364,  -1921237666,   1021592741, 
      -991624513,   -331418510,    205603955,   1901540867, 
      -353286382,   2074694545,   -852744502,    863712579, 
       714419927,    168046887,    850856730,    715231941, 
     -1382410582,  -1597665988,   2143533973,  -1412524265, 
     -2007464190,    262851108,   1640104130,    726602400, 
     -1329578994,   -562999148,   -168680816,   2068619247, 
     -1475729493,   1526958823,    621758951,   -531406811, 
     -1327472621,   -308151049,    -77214660,  -1629460713, 
       384431719,  -1176014961,   -495557088,    356423799, 
     -1065984587,   -900050494,    502901519,  -1008110816, 
      1393185456,   2073023516,    988911032,   -670543162, 
       361075036,  -1684619252,   1745080218,   1630600330, 
      1364771210,  -1027665419,    405257010,  -2050792798, 
      -321008776,   -804366257,  -1453911953,  -1379688722, 
      -331203444,  -1742771719,    423155935,   -124879369, 
       841595892,    858511778,    594985076,  -2003156271, 
     -1851963703,   -774813238,    132561076,    663339263, 
      -396772528,   1374308886,    937844519,   2012833546, 
       134578103,   -750993578,  -1693809674,    476563810, 
      1197446510,   -328767210,  -1757400606,  -1002997310, 
     -1487533071,   -940575064,   -257332369,    116577188, 
      -182860957,   1612208739,     77533215,   1905344663, 
       591456072,   1965780502,  -1113654803,    756439535, 
      -905960767,    737910451,    838122063,  -1855456931, 
     -1053168258,  -1185239043,    720836073,   1479153048, 
      -668028857,   1204822734,    753045488,  -2118641593, 
       438818902,   -486314017,   1786668788,  -2142539895, 
      -161279633,   -324918400,   -167862930,   1160327198, 
      -762477783,   1222945958,   -123020089,  -1993883878, 
     -1392108785,    952443500,   -113876794,  -1491551087, 
      -682364617,    461233160,  -1323944047,   1024035479, 
     -1104452566,   1792823897,   -991872933,   1140313888, 
     -1337186628,   -912688397,  -1756154301,    327316797, 
       787539133,    200115903,   -318992196,    620377022, 
       634012893,    768871104,    583199480,   1912008256, 
     -1250114989,    898858068,  -1132881046,  -1634680669, 
       460866915,   -214156961,   -177272655,    695547413, 
      1160867446,   -643308734,    695825926,   -360096115, 
      1468574528,   1429866849,  -1046077973,    487310153, 
       353257471,    174974120,   1588884635,  -1010266389, 
      -781365896,  -1635461093,   1889053190,    625140542, 
       -88204778,    598361791,    192054232,    632697990, 
#elif SIGNED_DIV_MODULO == 1
       575997406,    903701839,    -21363243,   1081453520, 
      -620816591,    288068001,      5039787,   1185467199, 
       669781354,   1109549523,   1253639739,  -2077249780, 
       977385403,    548449201,   1425923143,  -1443458171, 
      2073175204,   1323906934,    559559719,    960478999, 
     -1105298219,    835425197,    461659069,  -2092753733, 
      2046654075,   1775051209,    -86069220,    470373961, 
      -852032304,   1127525390,   -720438725,  -1102693688, 
      -504198418,  -1894202502,    395197482,   -984431881, 
     -1534154323,    644816912,   1801382023,   1822066668, 
      -747029454,   1851804337,   -341922483,    382177675, 
     -1205613829,   2070984156,   2062361404,  -1105462419, 
      1123452463,  -1974553666,    330078956,    862564539, 
       129623082,    -43992997,  -1119089835,   -514753447, 
     -1224628667,    315147625,   2048422846,  -1554996783, 
     -1680630212,    427333631,   -144524169,   1749247766, 
       809571795,  -2077963674,  -1356524422,   1279929929, 
      1678555402,   -920992400,   1283121059,   1615709481, 
     -1334835122,   -427903884,    854201397,    647748173, 
     -1571437046,    923415374,   1723706697,    630290429, 
     -1514802026,   -770393592,    -72125007,   1240050025, 
      1972816482,    -97500660,     27772542,    348751178, 
      1104092549,   1214119944,  -1186308642,  -1471008650, 
       370433469,     81789218,   1493347450,    248922344, 
     -1045901256,    102154874,  -1053942486,  -1543803093, 
     -2018989287,    821959048,    311560132,    127062974, 
     -1312274258,   -201126487,  -1247971087,    -97478339, 
       964363353,  -1980539318,    551608040,   1240024486, 
      2008963193,  -2091547402,   1743603345,   1822429041, 
     -1076870066,   2003115443,     80449184,  -1113088555, 
     -1190023828,   -201192560,   2019927355,  -1590999816, 
     -1455320340,  -2110088284,    700300223,  -1517340478, 
     -1286178645,   -974364731,  -1216523424,    379188077, 
     -1537861747,   -826071680,    -18512665,  -2110556774, 
       185073165,   -467134583,    221458156,    648615905, 
     -1842274487,   1336091023,  -1838784148,  -1693565823, 
      2069578596,    377618351,   -748696548,   2121660768, 
      -337794841,  -1019696636,   1536084153,    420593729, 
     -2138118365,   1531047777,  -2108911889,   1314638568, 
     -2092591702,    200998933,   -441810215,   -908243131, 
        85768077,   -209344661,   1753837046,  -1945335843, 
      -621685938,  -1250583760,  -1340647970,  -1264369873, 
     -1653414713,   -651731473,    527283988,  -1249506183, 
      -254384938,   -811430859,   1712371746,   1573366810, 
     -1821537968,   -376281717,   1752699656,  -1757085385, 
      1840798137,   1628948799,   -859751827,  -2122487166, 
      2132003057,    353130717,   -965042330,    297624272, 
       149009005,  -1007930392,     77446284,   1572140220, 
      2024920488,   1582338543,  -1921085279,  -1145267208, 
      2074876581,   -240012363,   -919622332,    331009357, 
      1729648866,   -413963847,   -189393648,  -2109158967, 
      2071709253,   1714197849,   -202516881,  -2138459576, 
       349858659,   -277831631,    878521697,    648825115, 
     -1610964135,   -403442300,  -2097966663,  -1350697695, 
      1144546716,  -1545653197,   -227019312,   -608217814, 
      -796401950,   -225141979,   2077652032,    896784504, 
      2125808228,  -1573834535,  -2076333720,   1035389200, 
     -1332385680,   -622133732,   -935540530,   -385892045, 
      1360984286,  -1253041092,  -1577212162,   1448287389, 
       777882198,  -1097375465,    611475000,  -1880386496, 
       917359097,   2136509077,    912380631,   1975548303, 
      1224171070,  -1077693581,  -2117656848,    186341029, 
      2136764853,    522037800,  -1008137688,    459319423, 
      1864928376,    247771415,   -670323874,   -273908117,
#else //SIGNED_DIV_EUCLIDEAN == 1
      -695365003,   -884879000,   1057908116,  -2103088210, 
     -1939403910,    721194250,    444391606,    112109236, 
       986594567,    890146941,   1208505235,   -910631956, 
       826918584,    243395087,   -444439812,  -1882953747, 
      1203344744,   -697571993,    463280543,   1036178252, 
     -1697314202,  -1598207239,    212773139,    -63439208, 
      1677080271,   1284024620,   1006487290,  -1927014876, 
     -1834443672,  -1767261104,   1281407203,   1902705470, 
       789052375,  -1580191673,    956594613,  -1673519058, 
     -1642855691,    604378876,   -735237349,    660617606, 
      1070011486,    357279996,   1030930998,  -1138909729, 
      1009134957,   2021205274,   1575953602,  -1777109163, 
      -573650365,   -561786858,    794691238,    420656120, 
      1242808674,   -568426387,  -1262600730,  -1775253661, 
      1167947280,  -1264118305,   -479869691,    222387101, 
     -1164136918,    609630322,    -66656293,  -1495312084, 
      1210888575,  -1715382784,   -884478362,  -1127974292, 
       132595334,  -1754499589,   -406667175,  -1697172446, 
     -1665227703,   1221608488,   -895198668,    444672733, 
      1994680166,   -289974743,    836454306,   1108527946, 
      -289299007,    667858995,  -1676089295,   1862984362, 
     -1342345615,  -1004254240,   1279171227,    -53241380, 
      1155166947,   -446647629,   -975220454,  -1987562112, 
       744295988,   -302518344,   -207278408,    471832949, 
     -1892348431,   -791083849,   1171332632,    843673263, 
     -1609186029,  -1588487425,  -1750833374,  -2113895743, 
      -330242677,    668181698,    957444952,    134042895, 
     -1680114162,    565932264,  -1604170864,  -1570655348, 
     -1724025244,  -1537481261,  -1424849824,  -1304600945, 
      -783918397,   -788431445,  -1213056349,  -1069267130, 
      1687588880,    872837103,    239392641,  -1355347077, 
     -1236818114,  -1815281637,   1777259230,    887835099, 
       248201722,   -801316387,  -1433644781,    526121722, 
      2095663728,  -1415483683,  -1040274363,   -443243552, 
     -1829678533,    790686586,   -419238919,   2073761398, 
      -420116585,    518282066,  -1484470382,   -509618467, 
     -1455418509,   1108584450,   1593910131,   -640909001, 
       796869801,   -884124886,    131551858,   1427747572, 
       418717185,   -707332449,   -862321959,   -203613484, 
      -332234390,   -602986273,    250479056,   1041724413, 
      -324979630,   -303414578,  -1611159233,  -2042540348, 
      -901147425,   -783743129,    660094098,   1962524849, 
      1871449848,   -180876819,  -1114636798,   1133433039, 
      1113802629,   1033575568,   1046633456,  -1692558922, 
       779796110,   -157794614,  -1236249620,  -1724348892, 
      1389741783,  -1395822550,  -1444956235,    712909665, 
      1693889468,     71116963,    870599778,  -1487818004, 
      1947583284,    175595791,    771951214,  -1990446681, 
      1328020428,   1067809923,  -1631284336,    107568163, 
      -748038869,    199498825,   -434303862,   -364671128, 
     -1371220879,  -1050605950,  -2059283167,   1819672899, 
       660135996,   1858061143,  -1445203373,   1808598942, 
      1265545980,    332405543,   -257545156,  -1041041074, 
      1082101521,  -1165355701,  -1871802207,   1148171505, 
       735307786,    924330712,    610146612,   -347684932, 
      -469173285,   1357811402,   -784141355,   1350938037, 
      1241561292,   1512961888,     24209088,    582661228, 
      1936576767,   -240672122,  -1889704780,   1575178813, 
       563451323,   -622453926,   2134612797,  -1184674391, 
       654772372,    450931373,   -484277185,  -1536794449, 
     -2039529706,   -338820291,  -1356778926,    969697389, 
      -556788022,   1466989404,   1006008863,    305053800, 
     -1387896960,   1964449418,  -1007934180,   1823561059, 
     -1186397078,   -543039312,  -1771672508,    601795992, 
#endif
}; 

// End of File
