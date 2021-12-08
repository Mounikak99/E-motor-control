//#############################################################################
//! \file i64byui64Input.c
//! \brief  FID Input Vectors (256) 
//! \author Vishal Coelho 
//! \date   13-Apr-2016
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


#include <stdint.h>

const int64_t test_dividend[256] = {
     3715027897909870592L,  3179147459177635840L, -6868200032993247232L, -5516147615172474880L, 
     7430838155027742720L,  7228104097946695680L, -2256373732449990656L, -4608127617192998912L, 
     1894507921818941440L,  2314154106080827392L, -8860286812006862848L,  5898025488568233984L, 
      331018914629574656L,  7552728717210583040L, -7035127622748848128L,  8229997182781483008L, 
     7877586460433293312L, -7474528258096775168L, -1563225254578122752L, -3267613508780335104L, 
     6387250794378297344L,  8263224903445788672L,  4228373328357074944L,  3391428820955594752L, 
     2608589232387465216L, -3396366546994319360L,  4282352117958000640L, -6315272173329549312L, 
     4797457482161868800L, -7527933575181651968L,  8626693504679964672L,  2010074340593735680L, 
     7285586587047682048L, -5642000671807553536L,  9003079950108223488L, -4771898544217470976L, 
    -5101982845088657408L, -1807818886594048000L,  5181807833060980736L,  6257357189375031296L, 
    -1669434497357570048L, -1800392397231677440L,  5012903314887432192L, -4627832705999427584L, 
     6746731816764291072L,  3360739748252489728L,  8806346132182142976L, -5801060031729620992L, 
    -5235503980520880128L, -3450982603689805824L,  4648801563420581888L, -2439828963570257920L, 
     1926873164249053184L, -2150246726129203200L,   931336065195659264L, -2113214328329488384L, 
     3051737719621785600L, -8951943627656753152L,  5776126938429868032L, -2769668406522288128L, 
     3975559738471817216L,  8465096557159950336L,   136821727653681152L,  3065262632465522688L, 
    -1301133080670851072L,  6447110187337607168L,  2845103638389835776L,    78541580163635200L, 
    -2425450599639797760L,  3417248806921869312L, -7393316394218938368L, -8344443029407311872L, 
     6813045100816517120L,  2801458431239557120L,  2037603391859310592L, -1439847599130273792L, 
       68388997031280640L, -6195506431319068672L, -7504649608722176000L,  3418640346331277312L, 
    -6591551114381670400L,   529636519786813440L, -9157466001708490752L, -1382439750940848128L, 
    -5085012773801201664L,  2950411581390278656L,    57219888658950144L,  1845730295456960512L, 
    -5530819781558343680L, -6688770608981671936L, -8784541913874139136L,  8108131663548399616L, 
     3606362886026311680L,  1000425647272491008L, -3773940249758556160L, -8436955101202415616L, 
     6525724026607992832L,  1783469970730244096L, -8881613274271500288L,  7990962808687257600L, 
    -9042809324193308672L,  2600170765292216320L,  5325305388151891968L,   488893029429800960L, 
    -3850787021394886656L,  2367243751038902272L, -1374860341358204928L,   855382223893604352L, 
    -5703787070865135616L,  4510023005193617408L, -5296867835332952064L, -7376438717256867840L, 
     7312012396891086848L,  5946067690471198720L,  7061113872751005696L, -7597765712710592512L, 
     3692810304594128896L, -6083289346563934208L,  6143582433330765824L,  4206910676058933248L, 
     5089327841937457152L,  4860629468120266752L, -8170072808652331008L,  2026332807783127040L, 
     7350507323461599232L,   297559346744995840L,   113844783270842368L, -4508078839744428032L, 
     8699542334939213824L,  1496472235704496128L,  8640204312870791168L, -8458968976752037888L, 
    -5743795364734152704L,   364508752509442048L,  2598296207799398400L, -2937988532332787712L, 
     2736202213423751168L, -2666698852343095296L,  6464292969235159040L,   789607000013522944L, 
     3192764077643909120L, -4490443329929357312L, -6985075900078538752L, -8474315322326128640L, 
     7156287691254614016L,  4670481617857445888L,  5572377416174272512L, -1080875409397252096L, 
    -7836682586788663296L, -6378336282422996992L, -4662522507707987968L, -7761910533370075136L, 
     6514644078319009792L, -6323970812995086336L,  2152046668962338816L,   524808844345346048L, 
    -5695988893593485312L,  4791555796866144256L,  2121346523946643456L, -6418293599712731136L, 
     -836791408337639424L,  3359107820112683008L, -4035057002747615232L,  2303318033910444032L, 
     8148482521276698624L,  1232304043337668608L,  3562354033334317056L,  8828361041505304576L, 
     7362416139789172736L,  2738970641885794304L,  8908316681156950016L, -6201519243398936576L, 
      180535650563008512L,  2349163890054946816L,  5534039941011095552L, -1516892623526694912L, 
    -8211855305282945024L,   353148040776728576L, -6741692160122978304L, -2453888559363479552L, 
     3627015189476728832L, -6832977920229677056L,  5272993123268640768L, -8295335276311865344L, 
     5822078717609490432L,  6768827286485800960L, -5591273547285346304L,  1981065964488517632L, 
     9162124269638336512L, -8441703662903070720L, -6662539626582831104L, -5110597136577484800L, 
     1531273042443376640L,  3156357572377090048L,  4117239215755333632L,    81876113122967552L, 
     2667957443759550464L, -8552476117274271744L, -5647437026679310336L,  4603305885852397568L, 
    -3538484408592582656L,  4594201305716811776L,  6493942371374776320L, -7390586991699843072L, 
     1803689409029869568L,  4069393354167492608L,  1267546475357947904L, -4886356357457524736L, 
     7574901375954827264L,  8312467333696034816L,   300964068028121088L,  3820101739557083136L, 
     5511798454624438272L, -5539753395969875968L, -7252512391650865152L,  2187319788226979840L, 
    -8813412688934371328L, -8530474630050859008L, -7095279610632245248L,  5878926431509864448L, 
    -2270808476885368832L, -4996184194003933184L, -2432389277745397760L, -6003669507183450112L, 
     5068789674213853184L, -3135381757377144832L, -3398256612034289664L,  4142897078164344832L, 
     1567142837341208576L,  5123685071215480832L, -2590358438914588672L, -5815649628533387264L, 
     1359324118669346816L,  -311849871993231360L, -1445727771717292032L, -3369755424854403072L, 
     8717405649719808000L, -4460005067297359872L,  1237774676149342208L, -2728883948274784256L, 
    -3467441697694220288L, -9050016863882788864L,  3101404817339265024L,  7114681934514348032L, 
    -1451431762914760704L,  6614634462733496320L,  8264002841199577088L, -2285426617785139200L, 
    -5473273510049679360L, -5918278212320933888L, -3573243470168012800L,  6462114623199051776L, 
      472283337821538304L,  6058690163427121152L, -7251849352503339008L,  2160889079053459456L, 
}; 

const uint64_t test_divisor[256] = {
    10456103722186731520UL,  4813422506039703552UL,    42577122450274304UL,  5298920628821293056UL, 
    15149759026474426368UL, 16511964786575509504UL, 18369413655081811968UL,   155661038525480960UL, 
    13518571665653067776UL,  1560913085952401408UL, 11683218945176422400UL, 14851223250419079168UL, 
    15310149850233126912UL, 10463388916176013312UL,  4796953872485044224UL, 14472061932935364608UL, 
     3121287492593698816UL,  5059173503526619136UL,    90447970184792064UL,  7333710457647222784UL, 
      489789707208067072UL, 15766414631885504512UL,  3779584552826574848UL, 15870666983176267776UL, 
     1035318139547432960UL,  4123064462304036864UL,   835278123536472064UL, 13853353040069750784UL, 
    12876867389592729600UL, 16948645048641452032UL, 16766662664924891136UL, 14859292426945005568UL, 
    18440793991085912064UL, 14771530551701616640UL,  5956736498763051008UL,   463835708552589312UL, 
    13518207025979531264UL,  8721375340252973056UL,  7241977781044342784UL, 15864458670872907776UL, 
     1059980335381280768UL,  7093605755169841152UL,  2616493933814773760UL,   842673053685075968UL, 
    12875730203749148672UL, 16851312887041937408UL,  7075168119453980672UL,  4484603028271613952UL, 
    16732999723995514880UL, 12949944773837271040UL, 16521602882245421056UL, 17487025635993468928UL, 
    17278916833104648192UL, 18387467878373224448UL,  7322296191160453120UL, 12173678786551169024UL, 
    15180753531656923136UL, 11480639788417380352UL, 12922060772836653056UL,  9029959856873304064UL, 
     9386013312891054080UL, 14383171614061449216UL,   746830751501770752UL, 16620652963654897664UL, 
    13503860067773638656UL, 11359192669160036352UL, 13930760583862677504UL,  5247493370024146944UL, 
     3804624632029933568UL, 12030712157937590272UL, 17120790965637789696UL, 14841761716921247744UL, 
     6464404370245128192UL,  4984096377084311552UL, 13735772386744571904UL, 13336850421284435968UL, 
    15803701147547205632UL, 16949224913233829888UL,  4962328539673065472UL, 12694159757758717952UL, 
    11400033450834593792UL,  1319928602898720768UL,  3475334024777555968UL,  6190164111026919424UL, 
     5737782654430078976UL,  7751395978063194112UL,  1618495171624208384UL, 10327750619121649664UL, 
    12315512121357619200UL,  2126941202761680896UL,  5822906413761310720UL, 18092499116633724928UL, 
    16341408644855842816UL,   419817078494103552UL, 15924806777730093056UL,  4519334624127731712UL, 
     7998540129170558976UL,  2497250145753911296UL,  3977117989674366976UL, 14291226023499952128UL, 
     1608302642216245248UL,  7385667276657022976UL, 10885625993549996032UL,  3625219983140628480UL, 
    12853921195295885312UL, 10938353196015706112UL,  8562793683803893760UL,  4448523605272049664UL, 
     7614270935421761536UL, 14213114312086575104UL,   651887490017105920UL, 11207324880020645888UL, 
     1466030061825458176UL,  7662343160441372672UL, 18178852638718177280UL, 15544120029666023424UL, 
      966657389883787264UL, 13420336710141339648UL,  2120601353934012416UL,  8953698376687771648UL, 
    16022738007095265280UL,   308132183048552448UL, 15106193269477175296UL, 14459917021063118848UL, 
     3573276973489520640UL, 16839005021038120960UL, 12611853379591299072UL, 16387967688555055104UL, 
    13150124994942720000UL,  8944679024764270592UL, 15370159990549059584UL,  1673975999605936128UL, 
      470239908243789824UL,  6193086112094883840UL,  8982137071670284288UL,  4157210398826444800UL, 
    10232571259617765376UL, 12564970077424562176UL, 15401983681805654016UL, 16525936508117741568UL, 
     8072256806594785280UL,  7372338831127158784UL,  8118780155652683776UL,  5711598177009586176UL, 
     8244834466840850432UL, 17789857790765309952UL,  8528819514946070528UL,  9149437715088054272UL, 
      397340889676083200UL,  3547034609342537728UL, 16567652847819837440UL,  7181174015326105600UL, 
     1882419287948331008UL,  6450800239153270784UL,  3293594048042014720UL,  4594919078701213696UL, 
    11824076102249504768UL, 17999487271447009280UL, 16983250449992886272UL,   992562639435194368UL, 
    16658971950604869632UL,  3504869912943644672UL, 18310518516092526592UL,  6022854480039864320UL, 
     4197436632181852160UL, 17408390302196746240UL, 14957484770168311808UL, 17052984830129690624UL, 
    13254022517723246592UL, 16219420760641918976UL, 15957573032470972416UL,  6593329132121606144UL, 
     2513857134933164032UL, 14864024990474612736UL, 12564535729084715008UL,  7957687732431667200UL, 
    17471195742916284416UL, 17546040049284249600UL,   591782752928002048UL,  5094377542140028928UL, 
     1249822493483612160UL, 15254522130833139712UL,  7310877164628719616UL, 10720818060711909376UL, 
     6337079720189939712UL, 17915657581757908992UL, 11130872469369917440UL, 12315805426442436608UL, 
    15925538300098465792UL,  7749238270960631808UL,  1354193969931198464UL, 16074438518633064448UL, 
    12814109049578086400UL, 15521545903145091072UL, 12474533841634207744UL, 12938577813874483200UL, 
    16517532244735125504UL, 10835874428817203200UL,  5998795627640475648UL, 18351755252409864192UL, 
     3828408241646303232UL,  5280428751805239296UL, 15368136307927146496UL, 14208563070305923072UL, 
     3700031466646538240UL, 14107448149051506688UL, 11361781194421635072UL, 17787594241268897792UL, 
     9291463745871368192UL,  8830490012588275712UL, 10804199967796535296UL, 18070277868699772928UL, 
     6439741545068304384UL,  4814651058583341056UL, 10882516754479388672UL, 13749923447664197632UL, 
     1512336100274374656UL,  6916714397348939776UL,  1605022505027817472UL,  5868857553599098880UL, 
     8360616812607739904UL,  2971418234242766848UL,  1104372417163155456UL,  8206314820293193728UL, 
     5262372181212975104UL,  3674189709333737472UL,  9404369562452199424UL,   359313668642830336UL, 
    15312690126501464064UL,  3696461199864469504UL,  7868702666532972544UL, 13523095821440387072UL, 
     9717685249130897408UL,  4967680716772513792UL, 14164059937089751040UL, 17605802645704200192UL, 
     2170948818562867200UL,   459106626469541888UL,  7785265982038728704UL,  1187674440801116160UL, 
     2316960047534886912UL,  1704806652964517888UL,  4292359476595701760UL,  3895824349424672768UL, 
    11249027090638567424UL,  7277739571450796032UL, 13405219253896304640UL, 17732821022593857536UL, 
    13839064768166539264UL,  1452492438732275712UL, 16010907801969991680UL,  6691941283231238144UL, 
    14702186500242417664UL, 14506283198066847744UL,  6112416226588964864UL, 17524249248464013312UL, 
}; 


// End of File
