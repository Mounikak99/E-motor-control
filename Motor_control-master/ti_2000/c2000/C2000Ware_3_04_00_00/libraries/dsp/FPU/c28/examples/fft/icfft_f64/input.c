//#############################################################################
//! \file   input.c
//! \brief  Input Vector (512) 
//! \author Vishal Coelho 
//! \date   09-Feb-2016
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
#include "dsp.h"

float64_t test_input[1024] = {
    22.135880731747L, 26.988086456423L, 22.891574705857L, 28.364806745128L, 
    23.717344013372L, 29.876648378907L, 24.623735841161L, 31.545043528558L, 
    25.623562779333L, 33.396256589062L, 26.732553870235L, 35.462838892228L, 
    27.970246344921L, 37.785646689135L, 29.361229620542L, 40.416696322093L, 
    30.936916897853L, 43.423295252658L, 32.738127907195L, 46.894173745218L, 
    34.818956545351L, 50.948858549617L, 37.252745209213L, 55.752505152178L, 
    40.141655212640L, 61.540345858483L, 43.632675985751L, 68.660023282818L, 
    47.945847911891L, 77.649485338571L, 53.427368306447L, 89.391791423685L, 
    60.658236402800L, 105.455591725968L, 70.702956547965L, 128.957029176205L, 
    85.779262871659L, 167.255545786392L, 111.619900517950L, 244.140837137412L, 
    173.869306570843L, 541.889342779153L, 353.448122432919L, 237.554621652121L, 
    551.149573026325L, 440.220518671918L, -207.663175747273L, -266.751864999423L, 
    43.022807105043L, 183.691599696686L, -147.589664618524L, -354.887819348139L, 
    -68.446404385731L, -181.930454488071L, -37.913132121593L, -127.511888126815L, 
    -14.297501441619L, -95.791395104025L, 19.895217374674L, -68.037703603146L, 
    446.716279548241L, 107.635616042576L, -72.601665890851L, -85.669575092903L, 
    -26.695877529772L, -61.421181437230L, 11.521013601727L, -40.386898963668L, 
    419.683004562194L, 144.847998858088L, -99.365016482877L, -83.230336831975L, 
    -58.229263747673L, -62.311529067098L, -44.011793868152L, -53.715689522806L, 
    -36.190452618825L, -48.245019496309L, -31.004044989274L, -44.176778737453L, 
    -27.207147661022L, -40.918369462289L, -24.254731528162L, -38.196786924201L, 
    -21.864769421669L, -35.862080965093L, -19.873957325391L, -33.821758133743L, 
    -18.179800919750L, -32.013975761875L, -16.713967713703L, -30.394909112103L, 
    -15.428719578783L, -28.932128304689L, -14.289442856255L, -27.600827067971L, 
    -13.270271025919L, -26.381529900550L, -12.351385794213L, -25.258622341590L, 
    -11.517281252759L, -24.219367177519L, -10.755606982368L, -23.253221878696L, 
    -10.056373213787L, -22.351350589893L, -9.411390221849L, -21.506266223549L, 
    -8.813863781234L, -20.711562175754L, -8.258097309303L, -19.961707370277L, 
    -7.739268612286L, -19.251887040502L, -7.253259855217L, -18.577877173236L, 
    -6.796526184854L, -17.935944130640L, -6.365992872188L, -17.322763365333L, 
    -5.958973796546L, -16.735352780630L, -5.573106100554L, -16.171017426215L, 
    -5.206297232658L, -15.627303024015L, -4.856681568107L, -15.101956394920L, 
    507.477415507078L, -14.592891273234L, -4.202492335038L, -14.098158297911L, 
    -3.895026890391L, -13.615918187735L, -3.598923557682L, -13.144417264022L, 
    -3.313012290584L, -12.681964589623L, -3.036200718455L, -12.226910059703L, 
    -2.767458888124L, -11.777622809656L, -2.505805148617L, -11.332469302565L, 
    -2.250292742851L, -10.889790421064L, -1.999996687925L, -10.447876812667L, 
    -1.754000519376L, -10.004941616131L, -1.511382442768L, -9.559089517440L, 
    -1.271200374073L, -9.108280829390L, -1.032475251071L, -8.650288932062L, 
    -0.794171849413L, -8.182648913101L, -0.555176120941L, -7.702594548297L, 
    -0.314267760193L, -7.206979776832L, -0.070086255096L, -6.692179418577L, 
     0.178911976785L, -6.153961847970L,  0.434508728604L, -5.587323358509L, 
     0.698796222709L, -4.986269509741L,  0.974273396409L, -4.343522005226L, 
     1.263976635856L, -3.650119203051L,  1.571660715572L, -2.894861805398L, 
     1.902054440910L, -2.063528369343L,  2.261230102890L, -1.137740304492L, 
     2.657151074590L, -0.093278400152L,  3.100507010403L,  1.102485981158L, 
     3.606030299872L,  2.494641021728L,  4.194652227762L,  4.147369060867L, 
     4.897198664959L,  6.155319523954L,  5.761081672330L,  8.664202437877L, 
     6.863263867990L, 11.910687629216L,  8.337626991781L, 16.306635574530L, 
    10.439660515597L, 22.638187623283L, 13.725305780155L, 32.616190878834L, 
    19.676909393019L, 50.802683958334L, 34.011832371091L, 94.793748233015L, 
    119.620191108339L, 358.145121105170L, -79.874808121145L, -255.958319495217L, 
    -29.907787952206L, -102.293636111248L, -18.321536284214L, -66.749955837062L, 
    -13.133448307701L, -50.895053759086L, -10.171826469519L, -41.889870676029L, 
    -8.244715973895L, -36.066047274062L, -6.882526024275L, -31.978427020783L, 
    -5.862792057018L, -28.942425025719L, -5.066499099682L, -26.591837092717L, 
    -4.424202941863L, -24.713018000806L, -3.892648682871L, -23.172921040553L, 
    -3.443483072210L, -21.884358399870L, -3.057341873609L, -20.787804155230L, 
    -2.720545700203L, -19.841225296762L, -2.423153687235L, -19.014091733979L, 
    -2.157765719008L, -18.283690101992L, -1.918757557026L, -17.632769889088L, 
    -1.701776694784L, -17.047991987737L, -1.503400818030L, -16.518877683949L, 
    -1.320900798662L, -16.037079358981L, -1.152072702461L, -15.595863585374L, 
    -0.995116446237L, -15.189737787336L, -0.848546656492L, -14.814175999960L, 
    -0.711126179194L, -14.465414334726L, -0.581815796040L, -14.140296317215L, 
    511.540264283538L, -13.836154460969L, -0.344135747050L, -13.550718542703L, 
    -0.234371938276L, -13.282043806942L, -0.129888123609L, -13.028454223805L, 
    -0.030201194404L, -12.788497239790L,  0.065110743507L, -12.560907393458L, 
     0.156417969587L, -12.344576832581L,  0.244046986149L, -12.138531251297L, 
     0.328286960557L, -11.941910118471L,  0.409395054081L, -11.753950329454L, 
     0.487600856121L, -11.573972608398L,  0.563110094672L, -11.401370135351L, 
     0.636107757589L, -11.235598984206L,  0.706760731336L, -11.076170043323L, 
     0.775220042341L, -10.922642156963L,  0.841622769349L, -10.774616277230L, 
     0.906093681990L, -10.631730456658L,  0.968746650437L, -10.493655543474L, 
     1.029685862808L, -10.360091466859L,  1.089006880372L, -10.230764019743L, 
     1.146797555379L, -10.105422062878L,  1.203138832071L, -9.983835087021L, 
     1.258105447968L, -9.865791080656L,  1.311766549729L, -9.751094659356L, 
     1.364186235590L, -9.639565419931L,  1.415424034460L, -9.531036488354L, 
     1.465535330231L, -9.425353235242L,  1.514571738539L, -9.322372136670L, 
     1.562581442135L, -9.221959761391L,  1.609609490146L, -9.123991868319L, 
     1.655698065710L, -9.028352600440L,  1.700886725890L, -8.934933763280L, 
     1.745212617179L, -8.843634177694L,  1.788710669494L, -8.754359098144L, 
     1.831413771159L, -8.667019688804L,  1.873352927036L, -8.581532550867L, 
     1.914557401706L, -8.497819295241L,  1.955054849352L, -8.415806155613L, 
     1.994871431768L, -8.335423637462L,  2.034031925785L, -8.256606199136L, 
     2.072559821214L, -8.179291961621L,  2.110477410282L, -8.103422443986L, 
     2.147805869438L, -8.028942321889L,  2.184565334286L, -7.955799206785L, 
     2.220774968321L, -7.883943443800L,  2.256453026079L, -7.813327926417L, 
     2.291616911223L, -7.743907926355L,  2.326283230060L, -7.675640937188L, 
     2.360467840891L, -7.608486530421L,  2.394185899595L, -7.542406222850L, 
     2.427451901781L, -7.477363354189L,  2.460279721807L, -7.413322974031L, 
     2.492682648947L, -7.350251737310L,  2.524673420957L, -7.288117807523L, 
     2.556264255249L, -7.226890767029L,  2.587466877891L, -7.166541533839L, 
     2.618292550595L, -7.107042284317L,  2.648752095880L, -7.048366381332L, 
     2.678855920532L, -6.990488307391L,  2.708614037523L, -6.933383602357L, 
     2.738036086490L, -6.877028805373L,  2.767131352898L, -6.821401400679L, 
     2.795908785982L, -6.766479766986L,  2.824377015564L, -6.712243130166L, 
     2.852544367832L, -6.658671518976L,  2.880418880144L, -6.605745723596L, 
     2.908008314951L, -6.553447256781L,  2.935320172882L, -6.501758317415L, 
     2.962361705061L, -6.450661756304L,  2.989139924709L, -6.400141044042L, 
     3.015661618080L, -6.350180240804L,  3.041933354776L, -6.300763967925L, 
     3.067961497485L, -6.251877381147L,  3.093752211180L, -6.203506145408L, 
     3.119311471818L, -6.155636411084L,  3.144645074564L, -6.108254791566L, 
     3.169758641577L, -6.061348342098L,  3.194657629392L, -6.014904539784L, 
     3.219347335909L, -5.968911264691L,  3.243832907025L, -5.923356781974L, 
     3.268119342929L, -5.878229724962L,  3.292211504078L, -5.833519079133L, 
     3.316114116880L, -5.789214166942L,  3.339831779089L, -5.745304633422L, 
     3.363368964949L, -5.701780432535L,  3.386730030076L, -5.658631814211L, 
     3.409919216125L, -5.615849312042L,  3.432940655219L, -5.573423731583L, 
     3.455798374190L, -5.531346139236L,  3.478496298607L, -5.489607851670L, 
     3.501038256638L, -5.448200425754L,  3.523427982723L, -5.407115648972L, 
     3.545669121096L, -5.366345530287L,  3.567765229142L, -5.325882291437L, 
     3.589719780615L, -5.285718358628L,  3.611536168712L, -5.245846354616L, 
     3.633217709021L, -5.206259091140L,  3.654767642341L, -5.166949561700L, 
     3.676189137387L, -5.127910934656L,  3.697485293381L, -5.089136546628L, 
     3.718659142540L, -5.050619896184L,  3.739713652464L, -5.012354637804L, 
     3.760651728424L, -4.974334576095L,  3.781476215566L, -4.936553660260L, 
     3.802189901027L, -4.899005978786L,  3.822795515965L, -4.861685754364L, 
     3.843295737523L, -4.824587339008L,  3.863693190704L, -4.787705209379L, 
     3.883990450190L, -4.751033962290L,  3.904190042082L, -4.714568310393L, 
     3.924294445592L, -4.678303078037L,  3.944306094657L, -4.642233197283L, 
     3.964227379508L, -4.606353704077L,  3.984060648179L, -4.570659734560L, 
     4.003808207966L, -4.535146521529L,  4.023472326836L, -4.499809391018L, 
     4.043055234785L, -4.464643759015L,  4.062559125156L, -4.429645128285L, 
     4.081986155910L, -4.394809085324L,  4.101338450859L, -4.360131297406L, 
     4.120618100856L, -4.325607509742L,  4.139827164953L, -4.291233542738L, 
     4.158967671515L, -4.257005289337L,  4.178041619308L, -4.222918712464L, 
     4.197050978553L, -4.188969842549L,  4.215997691940L, -4.155154775131L, 
     4.234883675627L, -4.121469668540L,  4.253710820200L, -4.087910741657L, 
     4.272480991609L, -4.054474271742L,  4.291196032076L, -4.021156592328L, 
     4.309857760985L, -3.987954091178L,  4.328467975740L, -3.954863208315L, 
     4.347028452609L, -3.921880434094L,  4.365540947535L, -3.889002307339L, 
     4.384007196942L, -3.856225413539L,  4.402428918507L, -3.823546383079L, 
     4.420807811922L, -3.790961889537L,  4.439145559636L, -3.758468648015L, 
     4.457443827577L, -3.726063413522L,  4.475704265868L, -3.693742979392L, 
     4.493928509512L, -3.661504175752L,  4.512118179076L, -3.629343868017L, 
     4.530274881356L, -3.597258955432L,  4.548400210027L, -3.565246369642L, 
     4.566495746283L, -3.533303073298L,  4.584563059469L, -3.501426058693L, 
     4.602603707688L, -3.469612346433L,  4.620619238417L, -3.437858984129L, 
     4.638611189096L, -3.406163045119L,  4.656581087719L, -3.374521627221L, 
     4.674530453408L, -3.342931851504L,  4.692460796983L, -3.311390861083L, 
     4.710373621522L, -3.279895819939L,  4.728270422921L, -3.248443911759L, 
     4.746152690432L, -3.217032338789L,  4.764021907210L, -3.185658320719L, 
     4.781879550845L, -3.154319093570L,  4.799727093892L, -3.123011908610L, 
     4.817566004397L, -3.091734031276L,  4.835397746412L, -3.060482740115L, 
     4.853223780517L, -3.029255325739L,  4.871045564324L, -2.998049089788L, 
     4.888864552995L, -2.966861343906L,  4.906682199740L, -2.935689408734L, 
     4.924499956322L, -2.904530612901L,  4.942319273562L, -2.873382292028L, 
     4.960141601834L, -2.842241787750L,  4.977968391564L, -2.811106446727L, 
     4.995801093732L, -2.779973619675L,  5.013641160360L, -2.748840660395L, 
     5.031490045018L, -2.717704924812L,  5.049349203315L, -2.686563770011L, 
     5.067220093398L, -2.655414553283L,  5.085104176453L, -2.624254631167L, 
     5.103002917202L, -2.593081358497L,  5.120917784406L, -2.561892087449L, 
     5.138850251368L, -2.530684166584L,  5.156801796444L, -2.499454939898L, 
     5.174773903546L, -2.468201745859L,  5.192768062661L, -2.436921916456L, 
     5.210785770363L, -2.405612776226L,  5.228828530336L, -2.374271641298L, 
     5.246897853898L, -2.342895818411L,  5.264995260535L, -2.311482603945L, 
     5.283122278432L, -2.280029282932L,  5.301280445015L, -2.248533128066L, 
     5.319471307502L, -2.216991398704L,  5.337696423453L, -2.185401339861L, 
     5.355957361335L, -2.153760181183L,  5.374255701087L, -2.122065135930L, 
     5.392593034699L, -2.090313399929L,  5.410970966799L, -2.058502150521L, 
     5.429391115241L, -2.026628545501L,  5.447855111716L, -1.994689722036L, 
     5.466364602359L, -1.962682795569L,  5.484921248378L, -1.930604858711L, 
     5.503526726684L, -1.898452980116L,  5.522182730541L, -1.866224203330L, 
     5.540890970221L, -1.833915545635L,  5.559653173678L, -1.801523996860L, 
     5.578471087230L, -1.769046518180L,  5.597346476257L, -1.736480040887L, 
     5.616281125915L, -1.703821465146L,  5.635276841861L, -1.671067658714L, 
     5.654335450996L, -1.638215455650L,  5.673458802226L, -1.605261654983L, 
     5.692648767238L, -1.572203019361L,  5.711907241294L, -1.539036273672L, 
     5.731236144045L, -1.505758103626L,  5.750637420361L, -1.472365154315L, 
     5.770113041185L, -1.438854028734L,  5.789665004408L, -1.405221286271L, 
     5.809295335760L, -1.371463441154L,  5.829006089735L, -1.337576960871L, 
     5.848799350524L, -1.303558264540L,  5.868677232992L, -1.269403721246L, 
     5.888641883660L, -1.235109648329L,  5.908695481732L, -1.200672309634L, 
     5.928840240137L, -1.166087913707L,  5.949078406607L, -1.131352611949L, 
     5.969412264782L, -1.096462496718L,  5.989844135348L, -1.061413599375L, 
     6.010376377207L, -1.026201888277L,  6.031011388680L, -0.990823266718L, 
     6.051751608744L, -0.955273570804L,  6.072599518314L, -0.919548567266L, 
     6.093557641551L, -0.883643951214L,  6.114628547219L, -0.847555343822L, 
     6.135814850078L, -0.811278289937L,  6.157119212325L, -0.774808255625L, 
     6.178544345076L, -0.738140625633L,  6.200093009895L, -0.701270700775L, 
     6.221768020375L, -0.664193695239L,  6.243572243764L, -0.626904733799L, 
     6.265508602650L, -0.589398848947L,  6.287580076696L, -0.551670977926L, 
     6.309789704435L, -0.513715959668L,  6.332140585124L, -0.475528531631L, 
     6.354635880660L, -0.437103326527L,  6.377278817559L, -0.398434868943L, 
     6.400072689008L, -0.359517571844L,  6.423020856980L, -0.320345732962L, 
     6.446126754428L, -0.280913531049L,  6.469393887554L, -0.241215022012L, 
     6.492825838157L, -0.201244134900L,  6.516426266065L, -0.160994667758L, 
     6.540198911656L, -0.120460283322L,  6.564147598465L, -0.079634504570L, 
     6.588276235891L, -0.038510710103L,  6.612588822001L,  0.002917870643L, 
     6.637089446436L,  0.044658162361L,  6.661782293429L,  0.086717249022L, 
     6.686671644932L,  0.129102379217L,  6.711761883862L,  0.171820971706L, 
     6.737057497477L,  0.214880621176L,  6.762563080868L,  0.258289104220L, 
     6.788283340600L,  0.302054385552L,  6.814223098486L,  0.346184624462L, 
     6.840387295513L,  0.390688181528L,  6.866780995922L,  0.435573625596L, 
     6.893409391449L,  0.480849741035L,  6.920277805742L,  0.526525535292L, 
     6.947391698946L,  0.572610246751L,  6.974756672487L,  0.619113352913L, 
     7.002378474044L,  0.666044578918L,  7.030263002730L,  0.713413906421L, 
     7.058416314488L,  0.761231582843L,  7.086844627717L,  0.809508131008L, 
     7.115554329127L,  0.858254359199L,  7.144551979858L,  0.907481371641L, 
     7.173844321847L,  0.957200579443L,  7.203438284484L,  1.007423712016L, 
     7.233340991548L,  1.058162828991L,  7.263559768454L,  1.109430332678L, 
     7.294102149825L,  1.161238981066L,  7.324975887390L,  1.213601901423L, 
     7.356188958250L,  1.266532604514L,  7.387749573516L,  1.320044999462L, 
     7.419666187339L,  1.374153409306L,  7.451947506361L,  1.428872587281L, 
     7.484602499605L,  1.484217733862L,  7.517640408824L,  1.540204514619L, 
     7.551070759348L,  1.596849078926L,  7.584903371440L,  1.654168079566L, 
     7.619148372207L,  1.712178693298L,  7.653816208076L,  1.770898642427L, 
     7.688917657894L,  1.830346217441L,  7.724463846664L,  1.890540300779L, 
     7.760466259966L,  1.951500391797L,  7.796936759109L,  2.013246632993L, 
     7.833887597037L,  2.075799837588L,  7.871331435057L,  2.139181518517L, 
     7.909281360422L,  2.203413918948L,  7.947750904836L,  2.268520044388L, 
     7.986754063916L,  2.334523696509L,  8.026305317699L,  2.401449508777L, 
     8.066419652226L,  2.469322984012L,  8.107112582306L,  2.538170533991L, 
     8.148400175504L,  2.608019521242L,  8.190299077456L,  2.678898303150L, 
     8.232826538576L,  2.750836278552L,  8.276000442262L,  2.823863936963L, 
     8.319839334699L,  2.898012910623L,  8.364362456348L,  2.973316029550L, 
     8.409589775262L,  3.049807379814L,  8.455542022327L,  3.127522365231L, 
     8.502240728581L,  3.206497772750L,  8.549708264737L,  3.286771841757L, 
     8.597967883085L,  3.368384337601L,  8.647043761916L,  3.451376629630L, 
     8.696961052675L,  3.535791774077L,  8.747745930023L,  3.621674602131L, 
     8.799425645019L,  3.709071813610L,  8.852028581670L,  3.798032076617L, 
     8.905584317079L,  3.888606133665L,  8.960123685476L,  3.980846914750L, 
     9.015678846426L,  4.074809657911L,  9.072283357527L,  4.170552037864L, 
     9.129972251968L,  4.268134303364L,  9.188782121302L,  4.367619423960L, 
     9.248751203881L,  4.469073246950L,  9.309919479383L,  4.572564665328L, 
     9.372328769949L,  4.678165797671L,  9.436022848459L,  4.785952180943L, 
     9.501047554566L,  4.896002977337L,  9.567450919117L,  5.008401196339L, 
     9.635283297701L,  5.123233933367L,  9.704597514119L,  5.240592626424L, 
     9.775449014623L,  5.360573332399L,  9.847896033912L,  5.483277024770L, 
     9.921999773931L,  5.608809914703L,  9.997824596641L,  5.737283797695L, 
    10.075438232062L,  5.868816428194L, 10.154912003024L,  6.003531924846L, 
    10.236321068206L,  6.141561209362L, 10.319744685239L,  6.283042482289L, 
    10.405266495837L,  6.428121739379L, 10.492974835133L,  6.576953332651L, 
    10.582963067676L,  6.729700580749L, 10.675329952801L,  6.886536433711L, 
    10.770180042431L,  7.047644197933L, 10.867624114721L,  7.213218327767L, 
    10.967779647397L,  7.383465291042L, 11.070771335082L,  7.558604516701L, 
    11.176731655493L,  7.738869433792L, 11.285801489986L,  7.924508612300L, 
    11.398130804647L,  8.115787017652L, 11.513879398978L,  8.312987392382L, 
    11.633217730136L,  8.516411780257L, 11.756327821824L,  8.726383210355L, 
    11.883404268173L,  8.943247561047L, 12.014655344448L,  9.167375626757L, 
    12.150304238129L,  9.399165413768L, 12.290590415908L,  9.639044695290L, 
    12.435771144521L,  9.887473860683L, 12.586123186045L, 10.144949099172L, 
    12.741944691580L, 10.412005964896L, 12.903557321014L, 10.689223377746L, 
    13.071308621152L, 10.977228123577L, 13.245574699821L, 11.276699928186L, 
    13.426763240037L, 11.588377192448L, 13.615316905975L, 11.913063491546L, 
    13.811717201721L, 12.251634960025L, 14.016488854923L, 12.605048707087L, 
    14.230204810938L, 12.974352434183L, 14.453491939466L, 13.360695460629L, 
    14.687037575688L, 13.765341404306L, 14.931597042487L, 14.189682815427L, 
    15.188002330658L, 14.635258124396L, 15.457172151567L, 15.103771343297L, 
    15.740123623527L, 15.597115058844L, 16.037985911866L, 16.117397378373L, 
    16.352016216628L, 16.666973647291L, 16.683618595774L, 17.248483956449L, 
    17.034366231643L, 17.864897714736L, 17.406027902777L, 18.519566894548L, 
    17.800599623171L, 19.216289991028L, 18.220342672308L, 19.959389305503L, 
    18.667829583588L, 20.753804918826L, 19.146000116465L, 21.605209731379L, 
    19.658229851989L, 22.520151313564L, 20.208414884758L, 23.506228179073L, 
    20.801077227064L, 24.572310676734L, 21.441497126988L, 25.728820313791L, 
}; 


// End of File
