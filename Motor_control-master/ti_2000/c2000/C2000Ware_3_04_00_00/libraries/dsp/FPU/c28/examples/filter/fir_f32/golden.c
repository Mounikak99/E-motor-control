//#############################################################################
//! \file   golden.c
//! \brief  Ouput Vector (1024) 
//! \author Vishal Coelho 
//! \date   16-Sep-2016
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

const float test_golden[512] = {
     0.00402598192F, -0.00144481604F, -0.01262123375F, -0.02972191886F,
    -0.04710516766F, -0.06620499481F, -0.07720189497F, -0.07382175907F,
    -0.04641268422F,  0.00229934146F,  0.07255872742F,  0.15249384875F,
     0.22838662769F,  0.27418459076F,  0.27228131149F,  0.20395402994F,
     0.06546027185F, -0.13823304942F, -0.37352707379F, -0.59530250533F,
    -0.74304025165F, -0.76046621636F, -0.58823919831F, -0.19230983252F,
     0.43221287360F,  1.24187385081F,  2.16417278150F,  3.09117467311F,
     3.90114531045F,  4.46222868199F,  4.66946501012F,  4.45011577448F,
     3.78857617343F,  2.72092554361F,  1.34915184636F, -0.18562350769F,
    -1.71195899483F, -3.06222926996F, -4.07898761824F, -4.65211901021F,
    -4.72514874820F, -4.31246605544F, -3.47687125583F, -2.33271152757F,
    -1.02010491927F,  0.30343508430F,  1.50216774225F,  2.46460655120F,
     3.12219228114F,  3.43939660015F,  3.42897025924F,  3.13109494957F,
     2.61264179854F,  1.94222636304F,  1.19556143318F,  0.43251205694F,
    -0.29633984915F, -0.96085836719F, -1.52493299311F, -1.98311986735F,
    -2.31509997074F, -2.51508869295F, -2.56338980253F, -2.45570058754F,
    -2.17883590524F, -1.73977668938F, -1.14145171092F, -0.41330312418F,
     0.41280913728F,  1.27497314724F,  2.10979597107F,  2.82950410310F,
     3.35585408674F,  3.60286108609F,  3.51513534133F,  3.05285243580F,
     2.22927098439F,  1.09003288338F, -0.26019936464F, -1.69146610646F,
    -3.03398232324F, -4.12351490968F, -4.79844056996F, -4.95211390576F,
    -4.52648676956F, -3.55074466263F, -2.11611968512F, -0.39394925917F,
     1.41160219453F,  3.06652645353F,  4.36802893524F,  5.14913675264F,
     5.32884297522F,  4.89591388258F,  3.93273590248F,  2.57319007441F,
     1.00586787089F, -0.58150003648F, -2.00408314290F, -3.13030603771F,
    -3.86808626616F, -4.19438478901F, -4.12275587208F, -3.71865405304F,
    -3.05636773414F, -2.22915810660F, -1.30972218364F, -0.37102461228F,
     0.54157190816F,  1.38113939192F,  2.12204616355F,  2.72853124536F,
     3.17876578417F,  3.43783082526F,  3.48742333516F,  3.30377941581F,
     2.88990395046F,  2.25181019462F,  1.42547560264F,  0.44839171133F,
    -0.61457291790F, -1.69950725911F, -2.72002374182F, -3.59949028624F,
    -4.24931586915F, -4.60115584136F, -4.58714099877F, -4.17630466015F,
    -3.35724850274F, -2.17241991095F, -0.69478581282F,  0.94669499134F,
     2.60209271673F,  4.08853773549F,  5.23785352170F,  5.89379458741F,
     5.96301565513F,  5.40675695999F,  4.27661022076F,  2.68514470937F,
     0.81962067919F, -1.10849159144F, -2.86814208735F, -4.26900864458F,
    -5.16179697322F, -5.48168854162F, -5.22659687596F, -4.47853055956F,
    -3.36075018570F, -2.04097714810F, -0.67787660585F,  0.57604584234F,
     1.61902107132F,  2.38196424145F,  2.85523263565F,  3.05422714872F,
     3.03419986342F,  2.84846614938F,  2.56033795561F,  2.20498339869F,
     1.81206190217F,  1.38071052689F,  0.91193320593F,  0.38915227402F,
    -0.18575250189F, -0.81090622917F, -1.45297198948F, -2.07475895718F,
    -2.61262223950F, -3.01001226092F, -3.19920270145F, -3.13847593971F,
    -2.79521624569F, -2.18064075143F, -1.32658134800F, -0.31068971673F,
     0.77773393590F,  1.82158253559F,  2.71532683695F,  3.35176354970F,
     3.66317608572F,  3.60588385197F,  3.19242383307F,  2.46329833400F,
     1.50828623343F,  0.43051138676F, -0.63952913133F, -1.58695762839F,
    -2.30170734935F, -2.71883679686F, -2.80070703061F, -2.56770954705F,
    -2.06872577814F, -1.39923515635F, -0.66059103130F,  0.03000472031F,
     0.58408706436F,  0.92967263980F,  1.04715879475F,  0.94518586265F,
     0.68459291806F,  0.34177232391F,  0.01901824806F, -0.20246054486F,
    -0.25348689983F, -0.11611273462F,  0.19755157517F,  0.62000550842F,
     1.06576485730F,  1.42293173742F,  1.60260380218F,  1.53294211138F,
     1.20104017013F,  0.63151094677F, -0.09150535281F, -0.86505815145F,
    -1.56001074248F, -2.07040731518F, -2.30801890906F, -2.24459256343F,
    -1.88990907380F, -1.31348881642F, -0.60497632145F,  0.11668342962F,
     0.75098878507F,  1.20848887699F,  1.45066904085F,  1.46673318502F,
     1.29628120260F,  0.99243757932F,  0.63518843194F,  0.28963808033F,
     0.02017344861F, -0.14483754709F, -0.18903268106F, -0.13266190363F,
     0.00038016477F,  0.16245522878F,  0.31504311591F,  0.40759630261F,
     0.41126709835F,  0.29829766237F,  0.07254152067F, -0.25662835288F,
    -0.64760641701F, -1.05507122493F, -1.40804697626F, -1.64595137624F,
    -1.70294187163F, -1.54610514924F, -1.15822775506F, -0.57119091716F,
     0.15999883108F,  0.93603342230F,  1.65267940363F,  2.19175529600F,
     2.46862436432F,  2.42356918468F,  2.06032566514F,  1.42204052579F,
     0.61206381212F, -0.24864176472F, -1.01830586606F, -1.58757854422F,
    -1.87392193159F, -1.86051322207F, -1.57136078737F, -1.09092684774F,
    -0.52097682337F,  0.01498648324F,  0.42235087503F,  0.62787529777F,
     0.61801202380F,  0.41332967119F,  0.08712448229F, -0.27686805625F,
    -0.57632416359F, -0.73665750978F, -0.70088126218F, -0.46658028642F,
    -0.05937099925F,  0.44810496975F,  0.97582622992F,  1.42448501874F,
     1.71851355390F,  1.79345665712F,  1.63181731029F,  1.23832785895F,
     0.66405147336F, -0.02687830245F, -0.73979798159F, -1.38860056788F,
    -1.88187719064F, -2.16189090900F, -2.18672206667F, -1.96167262429F,
    -1.51160122388F, -0.90316763752F, -0.21030851831F,  0.46959355564F,
     1.05636046081F,  1.47355530573F,  1.68483337647F,  1.67137977667F,
     1.45956693328F,  1.09114341901F,  0.64243346008F,  0.18512417165F,
    -0.20066058296F, -0.46577628193F, -0.57439996204F, -0.53566193896F,
    -0.37551736920F, -0.15669215600F,  0.05973070563F,  0.20145919007F,
     0.22610335190F,  0.10498165607F, -0.14504179584F, -0.48936069315F,
    -0.85434489149F, -1.16514464469F, -1.33468258553F, -1.30818499483F,
    -1.05024628997F, -0.57937120187F,  0.05783362710F,  0.76519779239F,
     1.43695205230F,  1.95031650485F,  2.21449372040F,  2.16325898879F,
     1.79479696284F,  1.14818831626F,  0.32358817931F, -0.55992227173F,
    -1.35806456070F, -1.95169250258F, -2.24233935498F, -2.19557223030F,
    -1.82014769879F, -1.19053043795F, -0.40744422034F,  0.39374873986F,
     1.09269254052F,  1.57952408572F,  1.79820778533F,  1.72671819684F,
     1.40237806924F,  0.88631711339F,  0.27647090629F, -0.33616335477F,
    -0.85667581167F, -1.22556525569F, -1.39874136909F, -1.37549853903F,
    -1.16783031468F, -0.82156695198F, -0.38341719870F,  0.07882940185F,
     0.51147955482F,  0.85601506698F,  1.08142561936F,  1.16009332876F,
     1.09678675565F,  0.90285497760F,  0.62130177665F,  0.29408439953F,
    -0.01910744893F, -0.27671621797F, -0.43715257152F, -0.49308514766F,
    -0.44834419025F, -0.34373273059F, -0.22348321506F, -0.15050483848F,
    -0.16462768298F, -0.29570774619F, -0.52780576603F, -0.82400329123F,
    -1.10459592377F, -1.28726556558F, -1.28054131800F, -1.03319386711F,
    -0.52460422125F,  0.20040556945F,  1.06055043327F,  1.92256418669F,
     2.64836384404F,  3.09427468014F,  3.16453106764F,  2.80531259633F,
     2.04280448714F,  0.95522461872F, -0.31310124054F, -1.60033792089F,
    -2.72636835436F, -3.54663171556F, -3.94865840130F, -3.89404694142F,
    -3.39473175473F, -2.53243620401F, -1.41734658769F, -0.19666816163F,
     0.99283732305F,  2.01719476338F,  2.78615371605F,  3.23343500693F,
     3.34558652751F,  3.13319700477F,  2.65243319033F,  1.96938850957F,
     1.17634915391F,  0.35374056404F, -0.41299897075F, -1.06792991584F,
    -1.56209813351F, -1.88253511831F, -2.02533352681F, -2.01930382796F,
    -1.89172817468F, -1.68634907685F, -1.42919544781F, -1.15090001809F,
    -0.85667261735F, -0.55217812606F, -0.21950585694F,  0.14873188018F,
     0.56742023359F,  1.02405803618F,  1.50045447077F,  1.94463991109F,
     2.30200704155F,  2.49770412004F,  2.47644153789F,  2.18973732926F,
     1.63551486659F,  0.83863567400F, -0.12217081499F, -1.14820506256F,
    -2.10606970384F, -2.87160241987F, -3.32370509497F, -3.39175561153F,
    -3.04327107832F, -2.31683950188F, -1.29292343441F, -0.11082613198F,
     1.07613690055F,  2.09781269081F,  2.82008120976F,  3.14043369568F,
     3.02994023338F,  2.51115474784F,  1.67913757805F,  0.65961564771F,
    -0.38506897891F, -1.30743563139F, -1.97257753397F, -2.30477446144F,
    -2.27085286137F, -1.90744534142F, -1.28618445342F, -0.52588449108F,
     0.25482630593F,  0.93270077662F,  1.42501417762F,  1.67337698250F,
     1.67392046310F,  1.44701926829F,  1.05515387862F,  0.56297046008F,
     0.05049419347F, -0.42534005963F, -0.81115192495F, -1.08651622872F,
    -1.23530297405F, -1.26916753535F, -1.19714949240F, -1.04864358617F,
    -0.84125976260F, -0.60304727401F, -0.34357421682F, -0.08121328695F,
     0.18343553921F,  0.44064124127F,  0.69730503102F,  0.94695582178F,
     1.19310434493F,  1.41933311312F,  1.61399991621F,  1.74213481723F,
     1.77348026463F,  1.65971685362F,  1.36881929554F,  0.86845759799F,
     0.16461568443F, -0.71586784156F, -1.69444388872F, -2.67073486964F,
    -3.50555273319F, -4.06548134374F, -4.22026195255F, -3.89555953424F,
    -3.06599798502F, -1.79167137177F, -0.18948054336F,  1.55278441173F,
     3.22704801279F,  4.60960888792F,  5.52171520656F,  5.83161410036F,
};


/*
const float test_golden[1024] = {
     0.00422265522F,  0.00017362429F, -0.01390355038F, -0.03593308162F, 
    -0.06217749387F, -0.08941783964F, -0.11120399364F, -0.11562718617F, 
    -0.09024010859F, -0.02886274638F,  0.06586309967F,  0.18239032010F, 
     0.29970039760F,  0.38844574201F,  0.41584630330F,  0.35254246025F, 
     0.18085700270F, -0.09459092141F, -0.43844127125F, -0.78591652393F, 
    -1.05319410643F, -1.14892654289F, -0.98572096136F, -0.49795543609F, 
     0.33545991817F,  1.47615082882F,  2.82669929692F,  4.24113660294F, 
     5.53976353441F,  6.53112625428F,  7.04279425148F,  6.95318594782F, 
     6.21505250907F,  4.86736599505F,  3.03450453137F,  0.91094888575F, 
    -1.26588306634F, -3.24744717893F, -4.80724500707F, -5.77485307032F, 
    -6.06355467146F, -5.67997917547F, -4.71458037464F, -3.32323434358F, 
    -1.70560582902F, -0.07642216583F,  1.36927870071F,  2.48474469049F, 
     3.18480997869F,  3.44728572308F,  3.30842689650F,  2.85182276927F, 
     2.18980228548F,  1.44226268763F,  0.71803640489F,  0.10020007787F, 
    -0.36231282714F, -0.65370326852F, -0.77679470619F, -0.76867202000F, 
    -0.67071600524F, -0.52666652705F, -0.37569637870F, -0.25036087206F, 
    -0.17541380911F, -0.16553035596F, -0.22344060995F, -0.34066207531F, 
    -0.50030614205F, -0.67961860593F, -0.85131678287F, -0.98500297706F, 
    -1.05015336236F, -1.02063824096F, -0.87933278483F, -0.62137961331F, 
    -0.25591064603F,  0.19304483361F,  0.68668839955F,  1.17452037092F, 
     1.60152096146F,  1.91727169994F,  2.08310110298F,  2.07581116111F, 
     1.88972055724F,  1.53816649743F,  1.05278291923F,  0.47874966821F, 
    -0.13271856453F, -0.73138680783F, -1.27263300945F, -1.72014893199F, 
    -2.04769589333F, -2.23990484686F, -2.29139731434F, -2.20411922200F, 
    -1.98432442775F, -1.64139506072F, -1.18911959179F, -0.64748627641F, 
    -0.04288597246F,  0.59271430545F,  1.22164668667F,  1.79986748603F, 
     2.27880317752F,  2.61132352805F,  2.75953684228F,  2.70093089184F, 
     2.43176797349F,  1.96856446617F,  1.34774002390F,  0.62231391521F, 
    -0.14477506989F, -0.88921108872F, -1.55307510372F, -2.08995382175F, 
    -2.46747721289F, -2.66868070977F, -2.69257775204F, -2.55234817879F, 
    -2.27003337147F, -1.86992534392F, -1.37461501617F, -0.80507672292F, 
    -0.18259211374F,  0.46962609300F,  1.12475682615F,  1.74996565175F, 
     2.30489806508F,  2.74363227132F,  3.02042700306F,  3.09697642011F, 
     2.94865919123F,  2.56890585650F,  1.97211485393F,  1.19527045465F, 
     0.29701787218F, -0.64762153923F, -1.55678852601F, -2.35246248030F, 
    -2.96875436047F, -3.35718864161F, -3.49047602543F, -3.36478499534F, 
    -2.99846208758F, -2.42663929644F, -1.69454350205F, -0.85274686729F, 
     0.04532171658F,  0.94529379664F,  1.79386993949F,  2.53999777439F, 
     3.13475278631F,  3.53128744810F,  3.68716761348F,  3.57026364972F, 
     3.16673024005F,  2.48773699482F,  1.57253551471F,  0.48805798202F, 
    -0.67402576259F, -1.80380211936F, -2.78495322705F, -3.51080985879F, 
    -3.90016810001F, -3.90903580075F, -3.53754955755F, -2.83215526684F, 
    -1.88145109297F, -0.80396823776F,  0.27066251712F,  1.22232626311F, 
     1.95759344335F,  2.42048318054F,  2.59643863124F,  2.51029089856F, 
     2.21865471990F,  1.79702136254F,  1.32352278444F,  0.86383055435F, 
     0.46153146191F,  0.13497719224F, -0.12101274126F, -0.32938641938F, 
    -0.52055215970F, -0.71914488079F, -0.93291427989F, -1.14851811233F, 
    -1.33482365802F, -1.45095211364F, -1.45638935476F, -1.32182582466F, 
    -1.03928838260F, -0.62891716949F, -0.13936325659F,  0.35962780687F, 
     0.79081918115F,  1.08519341631F,  1.19590623102F,  1.10865760679F, 
     0.84730823809F,  0.47222932568F,  0.06919658188F, -0.26926462002F, 
    -0.46411004813F, -0.46613103222F, -0.26648690693F,  0.09907453703F, 
     0.55403676990F,  0.99459536185F,  1.31108486390F,  1.41195673251F, 
     1.24407180130F,  0.80497438391F,  0.14544292898F, -0.63759244898F, 
    -1.41694003764F, -2.05812955691F, -2.44545909429F, -2.50465430484F, 
    -2.21640749864F, -1.61917466704F, -0.80284272092F,  0.10555730725F, 
     0.96385732293F,  1.64168287331F,  2.04547478719F,  2.13454503726F, 
     1.92527964946F,  1.48464180927F,  0.91526051473F,  0.33439324928F, 
    -0.14986145283F, -0.46013625455F, -0.56395764498F, -0.47728011466F, 
    -0.25761261086F,  0.01024092570F,  0.23251271516F,  0.32674982795F, 
     0.24142773030F, -0.03015034320F, -0.44697856921F, -0.92947672757F, 
    -1.37652831215F, -1.68546234083F, -1.77221682737F, -1.58947210622F, 
    -1.13881141731F, -0.47282478303F,  0.31371964104F,  1.10075681877F, 
     1.76361176033F,  2.19398628300F,  2.31866976814F,  2.11340378160F, 
     1.60878422169F,  0.88564779315F,  0.06039994829F, -0.73542882917F,
    -1.37618182350F, -1.76099399869F, -1.82994278667F, -1.57524763536F,
    -1.04358565146F, -0.32781867181F,  0.44906706828F,  1.15329954165F,
     1.66024947190F,  1.87364102136F,  1.74165180783F,  1.26635397275F,
     0.50415574550F, -0.44230859957F, -1.43830828257F, -2.33448435130F,
    -2.98624330879F, -3.27436822763F, -3.12461996432F, -2.52177149873F,
    -1.51401317949F, -0.20754967438F,  1.24577585327F,  2.66587649068F,
     3.86667786935F,  4.68162961650F,  4.98769168197F,  4.72281651286F,
     3.89506190039F,  2.58371596926F,  0.93222527152F, -0.86809030133F,
    -2.60296131563F, -4.06499521857F, -5.08250678354F, -5.54187163512F,
    -5.40055903191F, -4.69068357747F, -3.51347265536F, -2.02429797336F,
    -0.40909280702F,  1.14343244570F,  2.46738632896F,  3.43904870064F,
     3.98934168827F,  4.10901206821F,  3.84461830962F,  3.28472195790F,
     2.53980562169F,  1.72180118182F,  0.92748090316F,  0.22702620781F,
    -0.34210522927F, -0.77549985881F, -1.09570791123F, -1.34056455682F,
    -1.54909201237F, -1.74951046058F, -1.95234987457F, -2.14843396557F,
    -2.31022448897F, -2.39656616924F, -2.36159967730F, -2.16607418950F,
    -1.78677299643F, -1.22132945183F, -0.48948363272F,  0.36721931742F,
     1.28759943697F,  2.19645990355F,  3.01333652613F,  3.66167384840F,
     4.07578100364F,  4.20505000508F,  4.01686174731F,  3.49976264039F,
     2.66691265099F,  1.55826713421F,  0.24042083810F, -1.19525123222F,
    -2.63560908285F, -3.95167065462F, -5.00987467719F, -5.68791299080F,
    -5.89092135240F, -5.56454027785F, -4.70453378258F, -3.36325404217F,
    -1.65079667442F,  0.27242393048F,  2.21245333734F,  3.96515387781F,
     5.34182259688F,  6.19327846064F,  6.42957566334F,  6.03279103488F,
     5.06014876875F,  3.63551658988F,  1.93029035810F,  0.13836160000F,
    -1.54930314907F, -2.96748265189F, -3.99580850577F, -4.57163991124F,
    -4.69280267523F, -4.40905590526F, -3.80608593007F, -2.98757688026F,
    -2.05865342208F, -1.11186942485F, -0.21725876488F,  0.58137569628F,
     1.26315219898F,  1.82174597157F,  2.25583503338F,  2.56196970122F,
     2.73142173725F,  2.75057337677F,  2.60431184813F,  2.28279560038F,
     1.79087142537F,  1.15623245989F,  0.43133930323F, -0.31233204117F,
    -0.99477304256F, -1.53932154669F, -1.88495303837F, -1.99829879794F,
    -1.88237503477F, -1.57809879329F, -1.15719427312F, -0.70847817974F,
    -0.32090850298F, -0.06638110417F,  0.01524160089F, -0.07387706694F,
    -0.28778065562F, -0.54551387920F, -0.74933704215F, -0.80704614422F,
    -0.65213421572F, -0.25847333299F,  0.35119933439F,  1.10445155066F,
     1.88853501768F,  2.56987631010F,  3.01893979789F,  3.13415960194F,
     2.86087553825F,  2.20328294004F,  1.22728784971F,  0.05216924463F,
    -1.16866754503F, -2.27388949999F, -3.12108191166F, -3.60837525745F,
    -3.68770201714F, -3.36928969363F, -2.71765824087F, -1.83882584879F,
    -0.85982383116F,  0.09467310458F,  0.92197958601F,  1.55349539145F,
     1.95954292012F,  2.14799098112F,  2.15633246494F,  2.03779007273F,
     1.84535614565F,  1.61914244777F,  1.38007465497F,  1.12963775513F,
     0.85408755018F,  0.53203637052F,  0.14464020162F, -0.31334441102F,
    -0.82505752866F, -1.34944729356F, -1.82697044552F, -2.19055029301F,
    -2.37782009896F, -2.34267544569F, -2.06576614853F, -1.56202905827F,
    -0.88169527353F, -0.10333821926F,  0.67836633379F,  1.36634664340F,
     1.87469087062F,  2.14177233006F,  2.14114345653F,  1.88674396766F,
     1.43000766136F,  0.84955599876F,  0.23681705192F, -0.31861777554F,
    -0.74168754715F, -0.98212599792F, -1.02159946313F, -0.87564383259F,
    -0.58896061919F, -0.22582193247F,  0.14078250376F,  0.43954844223F,
     0.61154100577F,  0.62073712804F,  0.46107770276F,  0.15691630195F,
    -0.24258203850F, -0.67199099711F, -1.05898208315F, -1.33434935987F,
    -1.44269370979F, -1.35227919403F, -1.06147073453F, -0.59975816153F,
    -0.02344709691F,  0.59213901601F,  1.16247242262F,  1.60573064385F,
     1.85646249328F,  1.87787777197F,  1.66844533617F,  1.26139218700F,
     0.71921585963F,  0.12509394830F, -0.42928906378F, -0.85962658878F,
    -1.10536400401F, -1.14148117524F, -0.98257261951F, -0.67937518834F,
    -0.30941914321F,  0.03671600177F,  0.27226359614F,  0.33211315282F,
     0.18790572101F, -0.14394447445F, -0.60360163085F, -1.09738667933F,
    -1.51298207021F, -1.73886349997F, -1.68673020623F, -1.31268721544F,
    -0.63044301909F,  0.28781416399F,  1.32187012464F,  2.32153631479F,
     3.12948540207F,  3.60676146353F,  3.65724655523F,  3.24578739660F,
     2.40568681061F,  1.23429452819F, -0.12164941148F, -1.48783514558F,
    -2.68827919406F, -3.57221156765F, -4.03756206736F, -4.04537081197F,
    -3.62135294194F, -2.84591200830F, -1.83746377569F, -0.73292528256F,
     0.33315094035F,  1.24862922667F,  1.93849880972F,  2.37038374021F,
     2.55105548183F,  2.51721743165F,  2.32356273589F,  2.02973989124F,
     1.68790355531F,  1.33348790127F,  0.98183084302F,  0.63144333732F,
     0.27184566409F, -0.10787883892F, -0.51255432193F, -0.93647204917F,
    -1.36043521772F, -1.75168035037F, -2.06878888530F, -2.27063848246F,
    -2.32511831627F, -2.21425521555F, -1.93605223019F, -1.50471337761F,
    -0.94923444698F, -0.30941674381F,  0.36992893346F,  1.04401686176F,
     1.67041541069F,  2.20902558404F,  2.62171678151F,  2.87334140490F,
     2.93409626218F,  2.78195344564F,  2.40487600955F,  1.80447814400F,
     1.00221526321F,  0.04568462595F, -0.98947494843F, -2.00427055833F,
    -2.88621196262F, -3.52247676683F, -3.81608153475F, -3.70398621421F,
    -3.17292021109F, -2.26753580558F, -1.08817971981F,  0.22076966275F,
     1.48949753817F,  2.54606455164F,  3.24234037749F,  3.47928640178F,
     3.22643029814F,  2.52990965618F,  1.50625484795F,  0.32374403154F,
    -0.82423341021F, -1.75002329807F, -2.30342727273F, -2.39881266054F,
    -2.03060309572F, -1.27317982487F, -0.26676858443F,  0.80634722257F,
     1.75172483815F,  2.39556864778F,  2.61530235979F,  2.36123707466F,
     1.66429672652F,  0.62882755729F, -0.58660605075F, -1.79571569376F,
    -2.81388645137F, -3.48690451008F, -3.71490309291F, -3.46645155451F,
    -2.77944287306F, -1.75013080044F, -0.51505584011F,  0.77023818039F,
     1.95383111931F,  2.90869262343F,  3.54778874879F,  3.82948465260F,
     3.75424081741F,  3.35666651035F,  2.69614094850F,  1.84698580377F,
     0.88859710495F, -0.10309727701F, -1.06089796893F, -1.92736754505F,
    -2.65298062670F, -3.19438170793F, -3.51482706091F, -3.58636746962F,
    -3.39252920147F, -2.93189679605F, -2.22350678479F, -1.31217114163F,
    -0.26935478001F,  0.81277589331F,  1.83206199627F,  2.68820750947F,
     3.29430132298F,  3.58811586743F,  3.54203590995F,  3.16872460225F,
     2.52011292296F,  1.67964206986F,  0.74985632825F, -0.16193154335F,
    -0.95802690684F, -1.56370508466F, -1.93762932770F, -2.07612113873F,
    -2.00932792100F, -1.79128865515F, -1.48824401074F, -1.16758062186F,
    -0.88698343891F, -0.68413042076F, -0.57003679955F, -0.52901711505F,
    -0.52471887718F, -0.50913347182F, -0.43225890092F, -0.25186686240F,
     0.05705402364F,  0.49480822012F,  1.03373533605F,  1.62050338837F,
     2.18313517701F,  2.64043722676F,  2.91225250464F,  2.93079435072F,
     2.65298242015F,  2.07087383712F,  1.21610669081F,  0.15729635334F,
    -1.00704105713F, -2.15778734946F, -3.16727327451F, -3.91486655174F,
    -4.30397235760F, -4.27638844020F, -3.82123888208F, -2.97794019730F,
    -1.83330770198F, -0.51243456815F,  0.83682184072F,  2.06423687887F,
     3.03861343351F,  3.66523438147F,  3.89629044923F,  3.73448502490F,
     3.23106239676F,  2.47769344542F,  1.59104601426F,  0.69210534412F,
    -0.11409859886F, -0.75336433037F, -1.19039900518F, -1.43041786580F,
    -1.51367001573F, -1.50249836904F, -1.46342978916F, -1.44948500168F,
    -1.48759954407F, -1.57340983422F, -1.67308473901F, -1.73088607421F,
    -1.68151794103F, -1.46633467227F, -1.05056318966F, -0.43626658187F,
     0.33359688323F,  1.18048698590F,  2.00202612752F,  2.68770367102F,
     3.13755714758F,  3.28135391533F,  3.09297990776F,  2.59533398237F,
     1.85522443050F,  0.97107709211F,  0.05632620701F, -0.77954406582F,
    -1.44972088496F, -1.90449423355F, -2.13720269868F, -2.17973805962F,
    -2.08944738499F, -1.93216490106F, -1.76589176275F, -1.62732307482F,
    -1.52253840442F, -1.42457467119F, -1.28059466888F, -1.02755388014F,
    -0.61114356226F, -0.00281607721F,  0.78750243529F,  1.70449001424F,
     2.64910072512F,  3.49117275079F,  4.09146416077F,  4.32758123165F,
     4.11737727601F,  3.43540935540F,  2.32063039737F,  0.87492524320F,
    -0.74775115997F, -2.36144012544F, -3.77490163516F, -4.81937159023F,
    -5.37141945052F, -5.36809324523F, -4.81418093765F, -3.78086315316F,
    -2.39425997661F, -0.81514364983F,  0.78471432253F,  2.24359307755F,
     3.42581733756F,  4.23314860198F,  4.61174883399F,  4.55369706538F,
     4.09262239310F,  3.29501105678F,  2.25012087512F,  1.06092956101F,
    -0.16350824179F, -1.31559437925F, -2.29728898284F, -3.02747574294F,
    -3.44717218527F, -3.52347443946F, -3.25406567927F, -2.67138644530F,
    -1.84274496991F, -0.86407901114F,  0.15099985697F,  1.08396769486F,
     1.82503128630F,  2.28783070380F,  2.42351064608F,  2.23026185109F,
     1.75480721096F,  1.08494755984F,  0.33503058767F, -0.37252095912F,
    -0.92682521227F, -1.24704750087F, -1.29675436293F, -1.08998558479F,
    -0.68652998635F, -0.17828975383F,  0.32852004798F,  0.73198834048F,
     0.95290184110F,  0.95082492320F,  0.73310628530F,  0.35279385043F,
    -0.10401049875F, -0.53705562047F, -0.85253763449F, -0.98210572581F,
    -0.89736781397F, -0.61674948742F, -0.20231845950F,  0.25339597827F,
     0.64729842567F,  0.88699997015F,  0.90901006573F,  0.69193911849F,
     0.26359829712F, -0.30058637278F, -0.89021362094F, -1.38172554188F,
    -1.66307820828F, -1.65507805575F, -1.32660464464F, -0.70264804993F,
     0.13674547205F,  1.06818944384F,  1.94570756963F,  2.62560984894F,
     2.99004344916F,  2.96551041763F,  2.53455553257F,  1.73961073344F,
     0.67787608498F, -0.51327963515F, -1.67824749122F, -2.66709072831F,
    -3.35595624491F, -3.66186751352F, -3.55236567936F, -3.04934131261F,
    -2.22470998302F, -1.18756623920F, -0.06646823080F,  1.00841270492F,
     1.92100585516F,  2.58256878669F,  2.94134424028F,  2.98651206550F,
     2.74536292343F,  2.27504236840F,  1.65177716018F,  0.96005387286F,
     0.28238349985F, -0.31088449437F, -0.77100947836F, -1.07663039339F,
    -1.23326951305F, -1.26749882272F, -1.21917273196F, -1.13359145836F,
    -1.05296601603F, -1.00735708628F, -1.00792590521F, -1.04524734574F,
    -1.09231739325F, -1.10994829433F, -1.05344282427F, -0.88109625371F,
    -0.56447459202F, -0.09825265643F,  0.49391571617F,  1.16004330989F,
     1.82672056559F,  2.40965344898F,  2.82536957786F,  3.00347098555F,
     2.89892221719F,  2.50154974470F,  1.83901289728F,  0.97250094889F,
    -0.01203775681F, -1.01452726582F, -1.93307815697F, -2.67777710217F,
    -3.18302554301F, -3.41442073798F, -3.36872983119F, -3.06884729401F,
    -2.55662259107F, -1.88529184951F, -1.11199675746F, -0.29089739865F,
     0.53152837585F,  1.31701934532F,  2.03189371475F,  2.64227910132F,
     3.11176713898F,  3.40229025416F,  3.47687371092F,  3.30410447632F,
     2.86566910198F,  2.16628962118F,  1.24180634114F,  0.16103013906F,
    -0.97938961000F, -2.06500542001F, -2.97681907869F, -3.60722760521F,
    -3.87745478009F, -3.75315093087F, -3.25359536364F, -2.45101520412F,
    -1.45967134486F, -0.41749315336F,  0.53607372075F,  1.28317434789F,
     1.74842522400F,  1.91246286149F,  1.81328905706F,  1.53501326058F,
     1.18841181181F,  0.88823867835F,  0.72993023548F,  0.76814302180F,
     1.00187959417F,  1.37148588184F,  1.76940595712F,  2.06201042804F,
     2.11734020355F,  1.83345968770F,  1.16276298714F,  0.12797581624F,
    -1.17412114367F, -2.58373682538F, -3.90258336283F, -4.92614887339F,
    -5.47666525885F, -5.43203489953F, -4.74798466885F, -3.46974715804F,
    -1.72885205471F,  0.27584862885F,  2.30811034840F,  4.12801236335F,
     5.52417466497F,  6.34122732507F,  6.49915183389F,  6.00092333330F,
     4.92693594196F,  3.41820433764F,  1.65299726831F, -0.17801725907F,
    -1.89459055566F, -3.34587608574F, -4.42386734725F, -5.06894824778F,
    -5.26681438713F, -5.03991926916F, -4.43848620551F, -3.53341785570F,
    -2.40979519704F, -1.15980605272F,  0.12344686881F,  1.34958407056F,
     2.43310077029F,  3.29670936839F,  3.87691965840F,  4.13047692814F,
     4.03974953285F,  3.61618762458F,  2.90175464876F,  1.96784276698F,
     0.91016168595F, -0.16183323562F, -1.13992629989F, -1.93229892983F,
    -2.47444295227F, -2.73477432634F, -2.71646450021F, -2.45577703159F,
    -2.01524627025F, -1.47161431410F, -0.90201003289F, -0.37255344795F,
     0.06913828294F,  0.39810594143F,  0.61275344069F,  0.73101700848F,
     0.78205728433F,  0.79595082928F,  0.79517917719F,  0.79068661824F,
     0.78254094718F,  0.76304198520F,  0.72052511274F,  0.64404064343F,
     0.52911238282F,  0.38227298831F,  0.22100737916F,  0.06870796768F,
    -0.05215688406F, -0.12571818827F, -0.14703446012F, -0.12543897077F,
    -0.08521457217F, -0.06127669550F, -0.09013790696F, -0.19937354310F,
    -0.39916116043F, -0.67766565170F, -1.00029973665F, -1.31258507701F,
    -1.54716918968F, -1.63568979407F, -1.52400657542F, -1.18627476170F,
    -0.63313166299F,  0.08736292599F,  0.89453501110F,  1.68479356006F,
     2.34665243819F,  2.77958843509F,  2.91188251934F,  2.71256783649F,
     2.19611357809F,  1.42104713681F,  0.48314959196F, -0.49728511144F,
    -1.39360831504F, -2.09328001234F, -2.51577953198F, -2.62376087483F,
    -2.42600425652F, -1.97343128334F, -1.34989513069F, -0.65832407203F,
    -0.00324298651F,  0.52668160784F,  0.87138297247F,  1.00562141492F,
}; 
*/

// End of File
