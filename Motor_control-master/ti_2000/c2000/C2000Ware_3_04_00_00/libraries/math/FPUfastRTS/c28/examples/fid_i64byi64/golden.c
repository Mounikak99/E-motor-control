//#############################################################################
//! \file   golden.c
//! \brief  FID Ouput Vector (256) 
//! \author Vishal Coelho 
//! \date   12-Apr-2016
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
#if SIGNED_DIV_TRUNCATED == 1
                       0L,                   -3L,                    3L,                   -1L, 
                       1L,                   -1L,                  -39L,                    0L, 
                     -45L,                    0L,                    0L,                    0L, 
                      -1L,                    0L,                   -7L,                    0L, 
                       0L,                    0L,                    1L,                   42L, 
                       1L,                    0L,                    0L,                    0L, 
                       0L,                    1L,                   -1L,                   -1L, 
                       0L,                  -13L,                    0L,                   -1L, 
                       2L,                   -1L,                   -2L,                    0L, 
                       0L,                    2L,                   -2L,                   -5L, 
                       2L,                    0L,                    2L,                   -1L, 
                     -24L,                    0L,                    0L,                   -1L, 
                      -4L,                   -1L,                    0L,                    0L, 
                       0L,                    2L,                    1L,                    1L, 
                       3L,                    0L,                    0L,                   -4L, 
                       0L,                    0L,                    0L,                    1L, 
                       0L,                    0L,                    0L,                    0L, 
                      -1L,                    0L,                    0L,                    0L, 
                       0L,                   -2L,                    0L,                   -1L, 
                       1L,                    0L,                   -1L,                    6L, 
                       1L,                   -1L,                    0L,                   -1L, 
                      -1L,                    1L,                    0L,                   -1L, 
                      -1L,                    0L,                    0L,                    0L, 
                       0L,                    0L,                    1L,                    0L, 
                       0L,                    0L,                    0L,                    0L, 
                      -1L,                    0L,                    0L,                   -1L, 
                       1L,                    1L,                    0L,                   -2L, 
                       0L,                   -1L,                    1L,                   -2L, 
                       0L,                    0L,                    0L,                    0L, 
                      -1L,                    0L,                    0L,                    2L, 
                       0L,                   -1L,                    0L,                   -1L, 
                       0L,                    0L,                  -61L,                    0L, 
                       1L,                    0L,                    0L,                    0L, 
                      -1L,                    1L,                    0L,                   -1L, 
                       0L,                    1L,                   -1L,                    0L, 
                       1L,                    1L,                   27L,                   -1L, 
                      -1L,                  -13L,                    0L,                    0L, 
                      -1L,                    1L,                   -1L,                    1L, 
                       1L,                  -26L,                    0L,                    2L, 
                       0L,                    0L,                  -16L,                   -7L, 
                      21L,                   -1L,                    3L,                    0L, 
                      -1L,                    1L,                    0L,                    0L, 
                       0L,                    0L,                    0L,                   -1L, 
                      -2L,                    0L,                    0L,                   -5L, 
                       0L,                   -2L,                    1L,                    0L, 
                      -1L,                    0L,                    1L,                   -1L, 
                       6L,                    0L,                   -5L,                    0L, 
                       0L,                    0L,                    0L,                    0L, 
                       1L,                    0L,                   -1L,                    0L, 
                       0L,                    9L,                   -2L,                    0L, 
                      -1L,                    0L,                    3L,                   -2L, 
                       0L,                    2L,                    0L,                    0L, 
                       1L,                    0L,                    0L,                    0L, 
                      -3L,                  -16L,                    0L,                   -1L, 
                       0L,                   -1L,                    0L,                    1L, 
                      -2L,                   16L,                    2L,                   -2L, 
                      -1L,                   -1L,                    0L,                    0L, 
                      -1L,                    0L,                    0L,                    0L, 
                       0L,                    2L,                    0L,                    3L, 
                       0L,                   -1L,                  -30L,                   -1L, 
                       0L,                   -4L,                   -1L,                    1L, 
                      -2L,                    0L,                   -1L,                   -4L, 
                       1L,                    0L,                    0L,                    0L, 
                      10L,                    0L,                   -2L,                    0L, 
#elif SIGNED_DIV_MODULO == 1
                      -2L,                   -2L,                   -2L,                    1L, 
                       1L,                    0L,                    0L,                   -1L, 
                      -1L,                    1L,                    1L,                   -1L, 
                      -1L,                    2L,                    0L,                   -2L, 
                      -1L,                    3L,                   -2L,                    4L, 
                      -2L,                    0L,                    0L,                   -1L, 
                      -2L,                   -1L,                   -2L,                   -2L, 
                      57L,                   -2L,                   -1L,                   -2L, 
                      -1L,                    1L,                    1L,                  -15L, 
                      -1L,                   -1L,                   -1L,                    0L, 
                      -1L,                   -9L,                    4L,                   -1L, 
                       1L,                    1L,                    0L,                    3L, 
                       5L,                    1L,                    0L,                   -1L, 
                       2L,                   -3L,                   -1L,                    0L, 
                      -1L,                    0L,                    0L,                   -1L, 
                       0L,                    3L,                    7L,                    0L, 
                      -1L,                   -1L,                    0L,                   -2L, 
                       1L,                   -1L,                    1L,                    0L, 
                       0L,                   -3L,                    0L,                   -2L, 
                       2L,                    1L,                    1L,                   -7L, 
                      -3L,                    4L,                   -1L,                   -1L, 
                       7L,                   -1L,                   -2L,                   -3L, 
                       0L,                   -1L,                   -2L,                    0L, 
                       0L,                    1L,                    0L,                   -2L, 
                      -6L,                    1L,                   -2L,                   -1L, 
                       0L,                    1L,                    1L,                   -4L, 
                       2L,                    1L,                  -13L,                    0L, 
                      -2L,                   -1L,                    3L,                   -3L, 
                       0L,                    0L,                    1L,                    0L, 
                      -1L,                   -1L,                    0L,                    0L, 
                       1L,                    3L,                   -2L,                    2L, 
                      -1L,                    1L,                  -47L,                   -2L, 
                       2L,                   -2L,                   -1L,                    7L, 
                      12L,                   -2L,                   -4L,                   -2L, 
                       1L,                   -1L,                    1L,                    0L, 
                       0L,                    0L,                   -1L,                    1L, 
                       1L,                    0L,                   -2L,                   -5L, 
                       0L,                   12L,                   -2L,                    0L, 
                       0L,                   -8L,                   -1L,                   -1L, 
                      -3L,                   -2L,                   -2L,                    1L, 
                       0L,                   -1L,                    0L,                    3L, 
                       0L,                   -1L,                   -7L,                    0L, 
                      -1L,                   -1L,                    0L,                   -1L, 
                      -1L,                  -12L,                    0L,                    0L, 
                      -2L,                    0L,                    0L,                    0L, 
                      -1L,                    1L,                   -4L,                    0L, 
                       1L,                    0L,                    0L,                    0L, 
                      -1L,                   -2L,                    0L,                    0L, 
                       0L,                    0L,                    0L,                    0L, 
                       0L,                    0L,                    0L,                    0L, 
                       1L,                    3L,                   -1L,                    1L, 
                      -1L,                    0L,                    0L,                    1L, 
                      -1L,                    0L,                    3L,                    1L, 
                      -1L,                    1L,                    0L,                    1L, 
                       0L,                  -27L,                   -1L,                   -1L, 
                      -2L,                   -1L,                    2L,                   -1L, 
                      -5L,                    3L,                    1L,                    1L, 
                      -1L,                    0L,                   -2L,                    0L, 
                       0L,                   28L,                   -1L,                   27L, 
                       0L,                   -2L,                   -1L,                   -2L, 
                       0L,                    0L,                    2L,                    0L, 
                      -2L,                    2L,                   -7L,                    1L, 
                      -1L,                   -1L,                   -1L,                   -2L, 
                       0L,                    0L,                   -1L,                    1L, 
#else //SIGNED_DIV_EUCLIDEAN == 1
                      -1L,                   -1L,                   -1L,                    5L, 
                       1L,                    0L,                   -4L,                   -2L, 
                       0L,                    0L,                    1L,                    1L, 
                       2L,                    0L,                    7L,                   -9L, 
                       0L,                    0L,                    1L,                   -1L, 
                       0L,                    0L,                    5L,                    0L, 
                      -2L,                    1L,                    0L,                    0L, 
                       3L,                   -1L,                   -2L,                    1L, 
                       3L,                    0L,                    0L,                   -1L, 
                      -1L,                    0L,                   -1L,                    0L, 
                       1L,                    3L,                    1L,                  -13L, 
                       0L,                    1L,                    2L,                    1L, 
                       1L,                    2L,                    1L,                    2L, 
                       0L,                    1L,                    0L,                   -1L, 
                       0L,                    1L,                    1L,                    0L, 
                       1L,                    0L,                    0L,                    0L, 
                       1L,                   -2L,                   -6L,                    2L, 
                       2L,                   -1L,                    0L,                   21L, 
                       1L,                    0L,                   -1L,                    0L, 
                     -22L,                   -1L,                   -1L,                    0L, 
                       3L,                   -8L,                    4L,                    2L, 
                      -2L,                    0L,                    1L,                    2L, 
                       0L,                    2L,                    7L,                    1L, 
                      -1L,                    2L,                   -5L,                    3L, 
                      -5L,                   -1L,                    7L,                    2L, 
                       0L,                   -1L,                    1L,                    1L, 
                      -1L,                    1L,                    2L,                    0L, 
                      -8L,                   -6L,                    1L,                   -2L, 
                       1L,                    3L,                    1L,                   -1L, 
                      -3L,                    2L,                    4L,                    2L, 
                      -2L,                    0L,                   -3L,                    3L, 
                       5L,                    0L,                   -7L,                    0L, 
                       0L,                   -1L,                    1L,                    1L, 
                       1L,                    1L,                    1L,                    0L, 
                       0L,                   -2L,                    1L,                   -1L, 
                       1L,                   -1L,                   -9L,                    0L, 
                      -1L,                    0L,                   26L,                   -1L, 
                       0L,                    0L,                   -1L,                   -1L, 
                       1L,                    3L,                    2L,                   -2L, 
                       0L,                    0L,                    3L,                    2L, 
                       2L,                   -2L,                    1L,                    0L, 
                       1L,                    0L,                   -1L,                   11L, 
                      -1L,                   -1L,                   -1L,                  -10L, 
                       0L,                    3L,                    0L,                   -1L, 
                       0L,                    0L,                   -1L,                    0L, 
                       1L,                   -1L,                    1L,                   -2L, 
                      -1L,                   48L,                    5L,                   -3L, 
                      -1L,                    0L,                    1L,                    2L, 
                       1L,                   -1L,                   -1L,                   11L, 
                       2L,                22618L,                   -1L,                    3L, 
                      -1L,                    1L,                    1L,                    0L, 
                       3L,                    4L,                   -7L,                    0L, 
                       1L,                   -1L,                    0L,                    0L, 
                      -1L,                    0L,                    1L,                    4L, 
                      -4L,                   -4L,                    0L,                    1L, 
                      -2L,                    0L,                    1L,                    1L, 
                       0L,                    4L,                    0L,                   -1L, 
                       1L,                    0L,                   -1L,                   -1L, 
                       0L,                  -44L,                    1L,                    1L, 
                       0L,                    0L,                   -7L,                   -2L, 
                       1L,                    1L,                    1L,                   -4L, 
                       2L,                    0L,                    0L,                    1L, 
                      -1L,                   -3L,                    0L,                   18L, 
                       0L,                    7L,                   -2L,                    0L, 
#endif
}; 

const int64_t test_remainder[256] = {
#if SIGNED_DIV_TRUNCATED == 1
    -2415501404604182528L, -1781909742660403200L,  -412221563648940032L,  1939443326651770880L, 
     1709818014730620928L,   739556758077548544L,  -127212087053733888L,   928594177727928320L, 
      -70287111009961984L,  3256837159013918720L,  5945027540759646208L,  7589029285193203712L, 
    -2251206040955652096L, -3599659658380410880L,   519743837902706688L,  -235594058129778688L, 
      907103284638085120L,  6976864797695277056L,  2066704023782666240L,   -60456671020548096L, 
    -1028456903801085952L,   639121001494302720L,  4571430889937526784L,  8670212744503072768L, 
     3599754677185445888L,  -936957238978031616L,    24433837484249088L,   963059717148153856L, 
     -785379136409192448L,    51759478445242368L,  1074339629511962624L,  4254952315058288640L, 
     -229220716444805120L,  1238406753753645056L,   770689974655711232L, -2102931868691736576L, 
     4197570834500429824L,   367635948649195520L,  1313442531363305472L,  -320700285861265408L, 
     -780940137979373568L,  1335718508901908480L,  -193917957563514880L, -2298308887319828480L, 
      -64244045861545984L,  5391453109591689216L, -5703190698725490688L,  3888775703851491328L, 
      552220924517996544L,  1860587481475534848L, -1364134769269243904L, -3685324504916393984L, 
     3325772237319546880L,  1590803892629663744L,  2180616733920055296L,  2771547307238354944L, 
     1577073039074336768L,  -273911556350351360L,  -211390281676652544L,  -249133741580290048L, 
     -793559802623277056L,  1762684232156250112L, -7192690521392058368L,   -65032337485103104L, 
    -4352229842685050880L,  1479066209427810304L, -1191824163236552704L, -1566125163552802816L, 
    -3810866110118283264L, -4432833426542804992L, -4719082204382838784L,  3857123498845155328L, 
     2930278807102883840L, -1019491371450036224L, -6839699755158951936L, -1982414785264418816L, 
     2065801165662672896L,  1035784576265709568L,  2061224262105856000L,    -2521450542196736L, 
    -1729088097873838080L, -1256532928416296960L,  2887909759531802624L,  -481721689984397312L, 
      870564699380084736L, -3222427572828540928L,   442671569906827264L,  -608071252056971264L, 
     -103680495648419840L,  -573046447058808832L,  3885739537589702656L,  1869511242005579776L, 
    -5443321021337495552L,  2191224657789782016L,   239650046061729792L, -3415808133974087680L, 
     5622358147454746624L, -5939437823658602496L, -4772067167786231808L,  1747459491757785088L, 
     1965297703640428544L,  5878393932101144576L,  1661758933458800640L, -3748388770134919168L, 
      816341783634868224L,   228226488227657728L, -1857062700831152128L,   288433788995440640L, 
     3217044391206694912L, -1718131755179737088L, -2227240817497710592L,   661671194484989952L, 
    -3197909060803584000L, -3225381952413136896L, -1841475357370710016L,  4860829941261303808L, 
     -217355331949582336L,  3473343868865404928L,  2643724986985111552L,  1518939824378689536L, 
    -2240413693164738560L, -1642452632916740096L,  6631295961145104384L,   474006656252600320L, 
     1383696092439093248L,  4448208982099404800L,    33985105122402304L,  1016196269592637440L, 
     1161619283378690048L,  3142328738793185280L,   647618716459995136L,  3429731486543808512L, 
     1230710765518010368L,   893246710805125120L,  5886946265544024064L, -2236822998731466752L, 
     4638902895224997888L, -2234670902794731520L,  -703012775209334784L,  5365670797129355264L, 
     -193442429438478336L, -3549193192957526016L,   -22886539047700480L, -2786760244101724160L, 
      313800388248487936L,  -267948176492181504L, -1477880273781118976L, -6053552917757816832L, 
     1698695364880105472L,  -757246467091341312L, -2482860747704821760L,   682779735255468032L, 
    -2741175637554063360L,  -153968638754942976L,   550056745949272064L,   334775610168774656L, 
     1885526714334519296L,  1866381200619474944L,    43446249207052288L,   887392751989839872L, 
      -64796879696861184L,   937011715514824704L,  -611464022731706368L,  5756117245354883072L, 
    -4114657457511024640L,   -31886712820373504L,  1746289755589593088L, -5593139864715182080L, 
     5461918816386422784L, -5331326391345465344L, -6006010497934940160L,    34677098166839296L, 
      359825378105624576L,  2980347219090862080L, -5259196708322107392L,    18741079435270144L, 
    -2209979891796467712L, -1120467990528958464L,  -228150787298676736L,  7505731834170624000L, 
     3218039462427389952L,  1521278428753682432L,    97359167133501440L, -3843705256722253824L, 
     -670938487032467456L, -5263945043566383104L,   179913333343608832L, -2073150579289755648L, 
    -3355915262350364672L, -4394410391903367168L, -3753470012718272512L, -1324881255853686784L, 
     -592476898331500544L,  1244755671323758592L,  1403929942883657728L,  3423336438228838400L, 
    -2698013573074737152L,    54390738553884672L, -1173024603794989056L,  4579804749485039616L, 
     -611259251832866816L,  6359637362929438720L,  -331077026736857088L,    81144280025016320L, 
    -2847157284449552384L,  1340803979116984320L,  4621424366449952768L, -1209639081273120768L, 
     -513131155417389056L, -2343882975160195072L, -1046847277740677120L, -2207938694288689152L, 
      -86269760811923456L,   -16606665588727808L,  1965072112416000000L, -1657850537783801856L, 
    -1722118747009200128L, -3621663648365776896L,  -426486110159869952L, -1609516706740406272L, 
    -1097058852603707392L,   149211021878034432L,  -307474588559919104L,   977604468219738112L, 
    -2914808331127633920L,  3710876506103736320L,    49111694996627456L,  7220206776482480128L, 
      295238003170162688L,   -52026352501698560L,  1977753750629146624L, -2780130713933862912L, 
      483573295055286272L,   593568942221441024L, -4174942229652340736L,  -951781083345627136L, 
    -2801103305830369280L,    48628989232015360L,  -159324805502896128L, -1427562915822737408L, 
    -3638452463668379648L,  -385099849659746304L,  2009905297318971392L,  1559931020297539584L, 
    -1961853049808957440L, -7774210923877494784L,  -898220210946074624L,   -28969794752137216L, 
     -994222455379322880L,  -576168361800726528L, -4733755158113992704L,  7746098958469871616L, 
      220838795155990528L,  -209449230873946112L, -2313640440808663040L,   554621829142339584L, 
#elif SIGNED_DIV_MODULO == 1
     3041506886073663488L,  1048967618426294272L,  2079350387542896640L,   967548459751237632L, 
    -2252393048353890304L, -5771025303075352576L, -1909685790839920640L,  4282527048122509312L, 
     3725394949594570752L, -3524635343291926528L,  -255948144066748416L, -3609015730939668480L, 
    -2518487712476299264L,   -29474128975441920L, -3567655531289116672L, -2363975170909941760L, 
    -2380684777082193920L,  -106452324464431104L,  2647822015431831552L,  -258509652265148416L, 
    -4501061163194238976L, -1464301806440376320L,  1773547909473769472L, -2472564079483654144L, 
    -5261924474635960320L,  1064554484047042560L,  3706441115363842048L,   458690208303233024L, 
     -112947685983367168L,  -250814096075005952L,  6236958900034377728L, -3898038505563678720L, 
     6215277018702315520L,   884881726582392832L,   432063688263368704L,  -373016992258947072L, 
     6790436157218777088L,  7691740477062742016L, -8404188107477360640L,   451760120414783488L, 
     2367989412993605632L,  -128102179240273920L,  1129316119133521920L,  1908313644032557056L, 
     1615277214738311168L,  1197459512372834304L, -3547483573547270144L,  -210549302066624512L, 
     -443902504150747136L,  1549878405198153728L,   631116460454578176L,  6257550276207503360L, 
     1280422816487319552L,  1195769726308544512L,  3388139858704207872L,  4271112854713587712L, 
     3874128868386332672L, -1751649592278706176L, -4580598290638673920L, -5581893943828140032L, 
     5687637318793148416L, -1739303671629375488L,  -545545522336763904L,  -158457150666471424L, 
    -1533669415531139072L,  5056897509875937280L, -4006150129302294528L,  7968937249641877504L, 
    -3251238593780936704L, -6370756536478681088L,  3032782605950826496L, -2815094336875315200L, 
    -2795993476305461248L,  2343356751287812096L, -2004822129934614528L, -5902569285646389248L, 
     -439792494959917056L, -2653579968789856256L,  2156268410299914240L,   282916672221052928L, 
      226325425250887680L,   411430715806601216L, -1748595060243367936L, -2595680106772877312L, 
      471988810191560704L, -6277042828056776704L, -3346075272607291392L,  1566221241908766720L, 
    -7238641439403270144L, -5066708971241261056L,   -89178954956234752L, -1901294649487226880L, 
    -2829993716494348288L, -3977404381377513472L, -2308307455500019712L,   643790306020870144L, 
      281941321376499712L,  2282193245855766528L, -1741663750480056320L,  3275090234907682816L, 
     2307693115815223296L,   888821379302537216L,  2390489649502918656L,  -639630055974922240L, 
     1293099597673760768L,  2073563756582076416L,   448963843181645824L,  1319594672698406912L, 
    -5851792007978088448L, -3050452733603416064L, -1417533195145220096L,  -316493183801753600L, 
    -5015179764220227584L, -1096133232601360384L, -2360172965592461312L, -6878500465386553344L, 
    -3373871048375830528L,   714496673487413248L,    -1319608568135680L,   424194131675019264L, 
    -3265068844799084544L,   920666197457614848L,   107828226335522816L,  1226305330542016512L, 
    -1426465468749473792L,  2128936736572989440L,    66083871489167360L,  2750560805602338816L, 
    -1358774481484877824L,  7554476898449473536L,  1868750458677760000L,   115065814619230208L, 
      342621122312972288L, -3875292214630184960L,   303375543212455936L,   476143558832353280L, 
     1869827039271538688L,   -91799505864284160L,   654588133232893952L, -3509413908075153408L, 
    -7037139423371571200L,  -268804383098028032L,  5078940840268529664L,  2197592342479003648L, 
     -518990949096996864L, -5467468895070218240L,    22622007351871488L,  -790093425392594944L, 
     4587360757377898496L,  -154389974950594560L,  -550213171083565056L, -5173141069455841280L, 
     -935245692834564096L,  -586233140742025216L, -5166499762118776832L,  2362392630462330880L, 
     -950357735529801728L,  5386200027780505600L,  4025496593333434368L,  -324665065476704256L, 
      487659632430313472L, -1392468849623685120L,  -756331076183635968L,  -105417641338703872L, 
    -5160003341508329472L,    -2789695659833344L,   155809711665332224L, -4610568205732659200L, 
    -1428071801567215616L,  2100119443480180736L,  3047374925478043648L,  5713094527926067200L, 
     5118533778678300672L,  -280733021638901760L,   926240335395264512L,  2670675848295815168L, 
     -677305923134306304L,  1752421186183962624L, -3117748659302696960L, -5323572141799061504L, 
    -6892017919841140736L,  -739620926605926400L,  -878538937503954944L, -6246877097166950400L, 
     1524258846418302976L,  7829408696059998208L,  -621512956030840832L,  2094906447315357696L, 
     1512167273341462528L, -5747023735093911552L,  2826039982055309312L,  5006629329744015360L, 
    -1564748012473749504L, -6473502822586851328L,  -104800097855332352L,  6432276146575695872L, 
     3279571576997666816L,  -997448118412795904L,   -53913173653215232L, -6343316172658276352L, 
     1691893409024012288L,  -571455357514305536L, -1233028926740463616L, -1423655529999366144L, 
    -3999978090788116480L,  5034453593622546432L,  1468019010856933376L,  1134617233495562240L, 
     1578517704353077248L, -4117211182080618496L,  1853791863502704640L,  1631829051106174976L, 
    -1729254025466329088L,  1461045317499303936L,  2674124602870165504L, -3037561052093501440L, 
      514742122783952896L,   -53200735678724096L,   905224601991274496L, -2242845266708537344L, 
     3419191153861967872L,   553618041768140800L, -1088656425342562304L,  2300169305437396992L, 
    -1636474923182843904L,  -766756462679994368L,  1117203973020813312L,  -331232914225100800L, 
    -4307209644534294528L, -3136929994462162944L, -2597866894340900864L,  6547760077856333824L, 
      887275105969649664L,  -214684322475390976L, -5933764779206510592L,   189453928718583808L, 
     1222550043397703680L,  4507928240495765504L,  5337776277610063872L,  7836258193212899328L, 
    -2327562604735418368L, -4454383812514916352L, -1087350720795697152L,  -507557482034231296L, 
     1917844657003935744L, -2454539337307475968L,  -646249285993480192L, -2414444567357571072L, 
      630015863917389824L,  -167161250077763584L,  4141898425820078080L, -1978642292222066688L, 
      934716804966576128L, -1305705956851380224L,  2244698976700291072L,  -994768564186834944L,  
#else //SIGNED_DIV_EUCLIDEAN == 1
     1114136782351659008L,  3441066705609316352L,  5659486163390443520L,   230811311416506368L, 
     1859171119856193536L,  3118552656351037440L,    71432587406999552L,   888330849630343168L, 
     5604382974975438848L,  5500619788251543552L,  3143861530619963392L,   520609119741579264L, 
      789569764423647232L,  4839352907600918528L,   324062263895756800L,   450671958377658368L, 
     1289285944571785216L,  2001584581323974656L,  3381652212240975872L,  4993413593107628032L, 
     1844452193699209216L,   472771994710771712L,    47103896092276736L,  2871241237297434624L, 
     4148310994276200448L,  1402417639120957440L,  7908954416111089664L,  1530705398432475136L, 
     1735378174808547328L,  2846883169146118144L,  8156729109739188224L,  2082458493748273152L, 
      401782285166985216L,   670747005178030080L,  1601078726014629888L,   479084572204550144L, 
     6016128601572206592L,   291839730576570368L,  2607736294064222208L,  2949830625446938624L, 
     3780950348513034240L,   130022518175256576L,  3443564413265772544L,   182198287000434688L, 
     4264808065985370112L,  4158784852176160768L,  1267359725388580864L,  1432789649366818816L, 
     4063887453767794688L,  5666512633876611072L,   934404191934394368L,  1319322445275144192L, 
     1171096434965354496L,  7049273302283890688L,  2813652600165050368L,  1378181788437399552L, 
     3943283502467817472L,   194860726382434304L,  5776700597363269632L,  3556436112365168640L, 
     1182594620135766016L,  7779893092311689216L,  2444027345677846528L,  7972414442593619968L, 
     3773459737569953792L,    29890650589282304L,   103083484866004992L,  3140607941274075136L, 
     4764188153866645504L,  2735342093015250944L,  3956263009775884288L,    93098573324230656L, 
      586085420738828288L,  5679602405275623424L,  2527934001619556352L,  2634434386613280768L, 
       55360015280902144L,  1922330699785674752L,  1682324242847094784L,  2287047128160833536L, 
     2039834095528548352L,   597129150762467328L,  1967170607212673024L,  4279517674505476096L, 
     4082780229138352128L,   329142574747686912L,   842849990537424896L,  1652603370061965312L, 
     1131215660452620288L,   583792981872080896L,  1158981621832316928L,  7177050607778211840L, 
      362377780319438848L,  4265129699022981120L,  1247833658335633408L,   714633060905553920L, 
      137851487517333504L,   868490177654489088L,   441225937747324928L,  1527247458349176832L, 
      137153302978756608L,  1263833170497103872L,   636763129963939840L,   922105750063183872L, 
     1975544945063421952L,  2200033088065857536L,  8877837292112158720L,  4095843233668153344L, 
      455461675942633472L,   192514555457527808L,  2906052504000047104L,  3356066128357812224L, 
     2381240533873948672L,   432655390185185280L,   655348531739021312L,   846446928314087424L, 
      332337705689987072L,  6373285767648374784L,  1437092921234929664L,   360226310208741376L, 
     3238992808173395968L,   121216512831545344L,   922435343359858688L,  1521855385662629888L, 
     1332453603581001728L,  5159737512264435712L,   701561037572485120L,  5631670929269751808L, 
     1150778197360050176L,  6077054483618314240L,  2623986418372225024L,  4730090880849973248L, 
     5478701944470667264L,  1875557093995169792L,  3133324138605762560L,  2961892676751652864L, 
       97519824326002688L,  4687684934497497088L,  1437025509177634816L,    42284028293279744L, 
     3252850573481582592L,  6993509175885750272L,   224738058883137536L,  6470419241124896768L, 
     2877478408126593024L,  7598575771030929408L,   128175956517654528L,   901450125939208192L, 
     1179544452544030720L,  3104030241789806592L,   663925020038047744L,   346554911507130368L, 
     2255483791305248768L,   717490293371260928L,   300114534311120896L,  1920843786415257600L, 
     4379616938211559424L,  1244046278279852032L,  4259171094789021696L,   182572613820184576L, 
     5545643768438130688L,     7127304124385280L,  3605708222412453888L,  3503851911867199488L, 
      904100357449142272L,  1807384500351528960L,  2286935583304335360L,   360484838095570944L, 
     1388733624065212416L,  1139425816569008128L,   440665510395123712L,   116547727729287168L, 
      743256672612136960L,  1243687095519195136L,  4596608970540578816L,  1523379462361118720L, 
     3772304624399341568L,  4358732331130685440L,    62744916140226560L,  7214450529803151360L, 
     8527859029603803136L,  3531066168336523264L,   427036744256167936L,  2773576062832443392L, 
     3217387203908356096L,    83498678400385024L,   606696619215974400L,   444162587208439808L, 
     8554349868461453312L,  1815494895621857280L,  6271439929754404864L,   736262457539637248L, 
     2352538987973699584L,   964926533142960128L,    18040317481635840L,   290006672236904448L, 
     7873086438984884224L,      292872019378176L,  1498366336457322496L,   303050156663056384L, 
     1781769289734189056L,  5195079586117371904L,  1500822670134687744L,  3926228294152232960L, 
      318068881081767936L,    96250914272182272L,  1136584434507976704L,  5910172278096365568L, 
     1902645331477499904L,  1269818611092086784L,  6644716665918961664L,   934002343965669376L, 
     3552501637800058880L,  6456811144455262208L,  3469712132308301824L,   909296874810968064L, 
     1227976363762345984L,   407373141266702336L,  1441306388126627840L,  2256874573730824192L, 
      640301990495469568L,  3765775772733980672L,  4008165324315729920L,  3285480587015768064L, 
     4714789374144813056L,   384853767732338688L,   933666127329589248L,   342820287119357952L, 
      375759880597460992L,  4339659983190228992L,  4346189234608977920L,  7693098496971139072L, 
     4582945456769808384L,    51150369054574592L,  8127235523306053632L,   347696897149523968L, 
     4206124219313866752L,  2238989382063820800L,    17392525436299264L,  2849914883090663424L, 
     1473872487649277952L,   336837761967044608L,  1498451945476339712L,  1832550822368925696L, 
     8665001127863261184L,  2516679149009727488L,  5584824474597457920L,   297352064712278016L, 
     2913509734389815296L,   995727466574585856L,  3941042146223450112L,   219859382000082944L, 
     1477223361924476928L,   463355531142594560L,  1352660071747940352L,  1190863272955351040L,  
#endif
}; 


// End of File
