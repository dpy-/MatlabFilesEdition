" Vim syntax file
" Language:	Matlab
" Maintainer:	Fabrice Guy <fabrice.guy at gmail dot com>
"		Original authors: Mario Eusebio and Preben Guldberg
" Last Change:	2008 Oct 16 : added try/catch/rethrow and class statements
" 		2008 Oct 28 : added highlighting for most of Matlab functions
" 		2009 Nov 23 : added 'todo' keyword in the matlabTodo keywords 
" 		(for doxygen support)

" For version 5.x: Clear all syntax items
" For version 6.x: Quit when a syntax file was already loaded
if version < 600
  syntax clear
elseif exists("b:current_syntax")
  finish
endif

syn keyword matlabStatement		return function
syn keyword matlabConditional		switch case else elseif end if otherwise break continue
syn keyword matlabRepeat		do for while
syn keyword matlabStorageClass		classdef methods properties events persistent global
syn keyword matlabExceptions		try catch rethrow throw

syn keyword matlabTodo			contained  TODO NOTE FIXME XXX
syn keyword matlabImport		import
" If you do not want these operators lit, uncommment them and the "hi link" below
syn match  matlabRelationalOperator	"\(==\|\~=\|>=\|<=\|=\~\|>\|<\|=\)"
syn match matlabArithmeticOperator	"[-+]"
syn match matlabArithmeticOperator	"\.\=[*/\\^]"
syn match matlabLogicalOperator		"[&|~]"
syn keyword matlabBoolean		true false

syn match matlabLineContinuation	"\.\{3}"

" String
syn region matlabString			start=+'+ end=+'+	oneline

" If you don't like tabs
syn match matlabTab			"\t"

" Standard numbers
syn match matlabNumber		"\<\d\+[ij]\=\>"
" floating point number, with dot, optional exponent
syn match matlabFloat		"\<\d\+\(\.\d*\)\=\([edED][-+]\=\d\+\)\=[ij]\=\>"
" floating point number, starting with a dot, optional exponent
syn match matlabFloat		"\.\d\+\([edED][-+]\=\d\+\)\=[ij]\=\>"
syn keyword matlabConstant	eps Inf NaN pi


" Transpose character and delimiters: Either use just [...] or (...) aswell
syn match matlabDelimiter		"[][]"
"syn match matlabDelimiter		"[][()]"
syn match matlabTransposeOperator	"[])a-zA-Z0-9.]'"lc=1

syn match matlabSemicolon		";"

syn match matlabComment			"%.*$"	contains=matlabTodo,matlabTab
syn region matlabBlockComment        start=+%{+    end=+%}+ contains=matlabBlockComment


" Auto-generated function list (see load_function_list.py)
syn keyword matlabFunc abs accumarray acos acosd acosh acot acotd acoth acsc
syn keyword matlabFunc acscd acsch actxcontrol actxcontrollist
syn keyword matlabFunc actxcontrolselect actxGetRunningServer actxserver add
syn keyword matlabFunc addboundary addcats addCause addClass addCondition
syn keyword matlabFunc addConditionsFrom addConstructor addCorrection addedge
syn keyword matlabFunc addevent addFields addFile addFolderIncludingChildFiles
syn keyword matlabFunc addFunction addLabel addlistener addMethod addmulti
syn keyword matlabFunc addnode addOptional addParameter addParamValue addPath
syn keyword matlabFunc addpath addPlugin addpoints addpref addprop addProperty
syn keyword matlabFunc addproperty addReference addRequired addsample
syn keyword matlabFunc addsampletocollection addShortcut addShutdownFile
syn keyword matlabFunc addStartupFile addTeardown addtodate
syn keyword matlabFunc addToolbarExplorationButtons addts addvars adjacency
syn keyword matlabFunc airy align alim all allchild
syn keyword matlabFunc allowModelReferenceDiscreteSampleTimeInheritanceImpl
syn keyword matlabFunc alpha alphamap alphaShape alphaSpectrum
syn keyword matlabFunc alphaTriangulation amd analyzeCodeCompatibility
syn keyword matlabFunc ancestor and angle animatedline annotation ans any
syn keyword matlabFunc appdesigner append applyFixture area array2table
syn keyword matlabFunc array2timetable arrayfun ascii asec asecd asech asin
syn keyword matlabFunc asind asinh assert assertAccessed assertCalled
syn keyword matlabFunc assertClass assertEmpty assertEqual assertError
syn keyword matlabFunc assertFail assertFalse assertGreaterThan
syn keyword matlabFunc assertGreaterThanOrEqual assertInstanceOf assertLength
syn keyword matlabFunc assertLessThan assertLessThanOrEqual assertMatches
syn keyword matlabFunc assertNotAccessed assertNotCalled assertNotEmpty
syn keyword matlabFunc assertNotEqual assertNotSameHandle assertNotSet
syn keyword matlabFunc assertNumElements assertReturnsTrue assertSameHandle
syn keyword matlabFunc assertSet assertSize assertSubstring assertThat
syn keyword matlabFunc assertTrue assertUsing assertWarning assertWarningFree
syn keyword matlabFunc assignin assignOutputsWhen assumeAccessed assumeCalled
syn keyword matlabFunc assumeClass assumeEmpty assumeEqual assumeError
syn keyword matlabFunc assumeFail assumeFalse assumeGreaterThan
syn keyword matlabFunc assumeGreaterThanOrEqual assumeInstanceOf assumeLength
syn keyword matlabFunc assumeLessThan assumeLessThanOrEqual assumeMatches
syn keyword matlabFunc assumeNotAccessed assumeNotCalled assumeNotEmpty
syn keyword matlabFunc assumeNotEqual assumeNotSameHandle assumeNotSet
syn keyword matlabFunc assumeNumElements assumeReturnsTrue assumeSameHandle
syn keyword matlabFunc assumeSet assumeSize assumeSubstring assumeThat
syn keyword matlabFunc assumeTrue assumeUsing assumeWarning assumeWarningFree
syn keyword matlabFunc atan atan2 atan2d atand atanh audiodevinfo audioinfo
syn keyword matlabFunc audioread audiowrite autumn aviinfo axes axis axtoolbar
syn keyword matlabFunc axtoolbarbtn balance bandwidth bar bar3 bar3h barh
syn keyword matlabFunc barycentricToCartesian baryToCart base2dec
syn keyword matlabFunc batchStartupOptionUsed bctree beep BeginInvoke bench
syn keyword matlabFunc besselh besseli besselj besselk bessely beta betainc
syn keyword matlabFunc betaincinv betaln between bfsearch bicg bicgstab
syn keyword matlabFunc bicgstabl biconncomp bin2dec binary binscatter bitand
syn keyword matlabFunc bitcmp bitget bitnot bitor bitset bitshift bitxor
syn keyword matlabFunc blanks blkdiag bone boundary boundaryFacets
syn keyword matlabFunc boundaryshape boundingbox bounds box brighten brush
syn keyword matlabFunc bsxfun build builddocsearchdb builtin bvp4c bvp5c
syn keyword matlabFunc bvpget bvpinit bvpset bvpxtend caldays caldiff calendar
syn keyword matlabFunc calendarDuration calllib callSoapService calmonths
syn keyword matlabFunc calquarters calweeks calyears camdolly cameratoolbar
syn keyword matlabFunc camlight camlookat camorbit campan campos camproj
syn keyword matlabFunc camroll camtarget camup camva camzoom cancel cancelled
syn keyword matlabFunc cart2pol cart2sph cartesianToBarycentric cartToBary
syn keyword matlabFunc cast cat categorical categories caxis cd cdf2rdf
syn keyword matlabFunc cdfepoch cdfinfo cdflib cdfread cdfwrite ceil cell
syn keyword matlabFunc cell2mat cell2struct cell2table celldisp cellfun
syn keyword matlabFunc cellplot cellstr centrality centroid cgs changeFields
syn keyword matlabFunc char checkcode checkin checkout chol cholupdate choose
syn keyword matlabFunc circshift circumcenter circumcenters cla clabel class
syn keyword matlabFunc classUnderlying clc clear clearAllMemoizedCaches
syn keyword matlabFunc clearCache clearMockHistory clearPersonalValue
syn keyword matlabFunc clearpoints clearTemporaryValue clearvars clf
syn keyword matlabFunc clibRelease clipboard clock clone close closeFile
syn keyword matlabFunc closereq cmopts cmpermute cmunique
syn keyword matlabFunc codeCompatibilityReport colamd collapse colon colorbar
syn keyword matlabFunc colorcube colordef colormap ColorSpec colperm Combine
syn keyword matlabFunc combine comet comet3 compan compass complete complex
syn keyword matlabFunc compose computer cond condeig condensation condest
syn keyword matlabFunc coneplot conj conncomp contour contour3 contourc
syn keyword matlabFunc contourf contourslice contrast conv conv2 convert
syn keyword matlabFunc convertCharsToStrings convertContainedStringsToChars
syn keyword matlabFunc convertLike convertStringsToChars convertvars
syn keyword matlabFunc convexHull convhull convhulln convn cool copper copy
syn keyword matlabFunc copyElement copyfile copyHDU copyobj copyTo corrcoef
syn keyword matlabFunc cos cosd cosh cospi cot cotd coth count countcats
syn keyword matlabFunc countEachLabel cov cplxpair cputime createCategory
syn keyword matlabFunc createClassFromWsdl createFile createImg createLabel
syn keyword matlabFunc createMock createSampleTime createSharedTestFixture
syn keyword matlabFunc createSoapMessage createTbl createTestClassInstance
syn keyword matlabFunc createTestMethodInstance criticalAlpha cross csc cscd
syn keyword matlabFunc csch csvread csvwrite ctranspose cummax cummin cumprod
syn keyword matlabFunc cumsum cumtrapz curl currentProject customverctrl
syn keyword matlabFunc cylinder daqread daspect datacursormode datastore
syn keyword matlabFunc dataTipInteraction dataTipTextRow date datenum
syn keyword matlabFunc dateshift datestr datetick datetime datevec day days
syn keyword matlabFunc dbclear dbcont dbdown dblquad dbmex dbquit dbstack
syn keyword matlabFunc dbstatus dbstep dbstop dbtype dbup dde23 ddeget ddensd
syn keyword matlabFunc ddesd ddeset deal deblank dec2base dec2bin dec2hex
syn keyword matlabFunc decic decomposition deconv defineArgument defineOutput
syn keyword matlabFunc deg2rad degree del2 delaunay delaunayn DelaunayTri
syn keyword matlabFunc delaunayTriangulation delegateTo delete deleteCol
syn keyword matlabFunc deleteFile deleteHDU deleteKey deleteproperty
syn keyword matlabFunc deleteRecord deleteRows delevent delsample
syn keyword matlabFunc delsamplefromcollection demo det details
syn keyword matlabFunc detectImportOptions detrend deval dfsearch diag
syn keyword matlabFunc diagnose dialog diary diff diffuse digraph dir
syn keyword matlabFunc disableDefaultInteractivity discretize disp display
syn keyword matlabFunc displayEmptyObject displayNonScalarObject
syn keyword matlabFunc displayScalarHandleToDeletedObject displayScalarObject
syn keyword matlabFunc dissect distances dither divergence dlmread dlmwrite
syn keyword matlabFunc dmperm doc docsearch done dos dot double drag dragrect
syn keyword matlabFunc drawnow dsearchn duration echo echodemo edgeAttachments
syn keyword matlabFunc edgecount edges edit eig eigs ellipj ellipke ellipsoid
syn keyword matlabFunc empty enableDefaultInteractivity
syn keyword matlabFunc enableNETfromNetworkDrive enableservice EndInvoke
syn keyword matlabFunc endsWith enumeration eomday eps eq equilibrate erase
syn keyword matlabFunc eraseBetween erf erfc erfcinv erfcx erfinv error
syn keyword matlabFunc errorbar errordlg etime etree etreeplot eval evalc
syn keyword matlabFunc evalin eventlisteners exceltime Execute exist exit exp
syn keyword matlabFunc expand expectedContentLength expint expm expm1 export
syn keyword matlabFunc export2wsdlg exportsetupdlg extractAfter extractBefore
syn keyword matlabFunc extractBetween eye ezcontour ezcontourf ezmesh ezmeshc
syn keyword matlabFunc ezplot ezplot3 ezpolar ezsurf ezsurfc faceNormal
syn keyword matlabFunc faceNormals factor factorial false fatalAssertAccessed
syn keyword matlabFunc fatalAssertCalled fatalAssertClass fatalAssertEmpty
syn keyword matlabFunc fatalAssertEqual fatalAssertError fatalAssertFail
syn keyword matlabFunc fatalAssertFalse fatalAssertGreaterThan
syn keyword matlabFunc fatalAssertGreaterThanOrEqual fatalAssertInstanceOf
syn keyword matlabFunc fatalAssertLength fatalAssertLessThan
syn keyword matlabFunc fatalAssertLessThanOrEqual fatalAssertMatches
syn keyword matlabFunc fatalAssertNotAccessed fatalAssertNotCalled
syn keyword matlabFunc fatalAssertNotEmpty fatalAssertNotEqual
syn keyword matlabFunc fatalAssertNotSameHandle fatalAssertNotSet
syn keyword matlabFunc fatalAssertNumElements fatalAssertReturnsTrue
syn keyword matlabFunc fatalAssertSameHandle fatalAssertSet fatalAssertSize
syn keyword matlabFunc fatalAssertSubstring fatalAssertThat fatalAssertTrue
syn keyword matlabFunc fatalAssertUsing fatalAssertWarning
syn keyword matlabFunc fatalAssertWarningFree fclose fcontour feather
syn keyword matlabFunc featureEdges feof ferror Feval feval fewerbins fft fft2
syn keyword matlabFunc fftn fftshift fftw fgetl fgets fieldnames figure
syn keyword matlabFunc figurepalette fileattrib fileDatastore filemarker
syn keyword matlabFunc fileMode fileName fileparts fileread filesep fill fill3
syn keyword matlabFunc fillmissing filloutliers filter filter2 fimplicit
syn keyword matlabFunc fimplicit3 find findall findCategory findedge findEvent
syn keyword matlabFunc findfigs findFile findgroups findLabel findnode findobj
syn keyword matlabFunc findprop findstr finish fitsdisp fitsinfo fitsread
syn keyword matlabFunc fitswrite fix flag flintmax flip flipdim flipedge
syn keyword matlabFunc fliplr flipud floor flow fmesh fminbnd fminsearch fopen
syn keyword matlabFunc format fplot fplot3 fprintf frame2im fread freeBoundary
syn keyword matlabFunc freqspace frewind fscanf fseek fsurf ftell ftp full
syn keyword matlabFunc fullfile func2str functions functiontests funm fwrite
syn keyword matlabFunc fzero gallery gamma gammainc gammaincinv gammaln gather
syn keyword matlabFunc gca gcbf gcbo gcd gcf gcmr gco ge genpath genvarname
syn keyword matlabFunc geoaxes geobasemap geobubble geodensityplot geolimits
syn keyword matlabFunc geoplot geoscatter geotickformat get getabstime
syn keyword matlabFunc getAColParms getappdata getaudiodata getBColParms
syn keyword matlabFunc GetCharArray getClass getColName getColType
syn keyword matlabFunc getConstantValue getCurrentTime getData getdatasamples
syn keyword matlabFunc getdatasamplesize getDiagnosticFor getDiscreteStateImpl
syn keyword matlabFunc getDiscreteStateSpecificationImpl getdisp getenv
syn keyword matlabFunc getEqColType getfield getFields getFileFormats
syn keyword matlabFunc getFooter getframe GetFullMatrix getGlobalNamesImpl
syn keyword matlabFunc getHdrSpace getHDUnum getHDUtype getHeader
syn keyword matlabFunc getHeaderImpl getIconImpl getImgSize getImgType
syn keyword matlabFunc getImpulseResponseLengthImpl
syn keyword matlabFunc getInputDimensionConstraintImpl getinterpmethod
syn keyword matlabFunc getLocation getMockHistory getNegativeDiagnosticFor
syn keyword matlabFunc getnext getNumCols getNumHDUs getNumInputs
syn keyword matlabFunc getNumInputsImpl getNumOutputs getNumOutputsImpl
syn keyword matlabFunc getNumRows getOpenFiles getOutputDataTypeImpl
syn keyword matlabFunc getOutputDimensionConstraintImpl getOutputSizeImpl
syn keyword matlabFunc getParameter getpixelposition getplayer getpoints
syn keyword matlabFunc getPostActValString getPostConditionString
syn keyword matlabFunc getPostDescriptionString getPostExpValString
syn keyword matlabFunc getPreDescriptionString getpref getProfiles
syn keyword matlabFunc getPropertyGroups getPropertyGroupsImpl getqualitydesc
syn keyword matlabFunc getReasonPhrase getReport getsamples getSampleTime
syn keyword matlabFunc getSampleTimeImpl getsampleusingtime
syn keyword matlabFunc getSharedTestFixtures getSimulateUsingImpl
syn keyword matlabFunc getSimulinkFunctionNamesImpl gettimeseriesnames
syn keyword matlabFunc getTimeStr gettsafteratevent gettsafterevent
syn keyword matlabFunc gettsatevent gettsbeforeatevent gettsbeforeevent
syn keyword matlabFunc gettsbetweenevents GetVariable getvaropts getVersion
syn keyword matlabFunc GetWorkspaceData ginput gmres gobjects gplot grabcode
syn keyword matlabFunc gradient graph gray graymon grid griddata griddatan
syn keyword matlabFunc griddedInterpolant groot groupcounts groupsummary
syn keyword matlabFunc grouptransform gsvd gt gtext guidata guide guihandles
syn keyword matlabFunc gunzip gzip h5create h5disp h5info h5read h5readatt
syn keyword matlabFunc h5write h5writeatt hadamard hankel hasdata
syn keyword matlabFunc hasFactoryValue hasfile hasFrame hasnext
syn keyword matlabFunc hasPersonalValue hasTemporaryValue hdf5info hdf5read
syn keyword matlabFunc hdf5write hdfan hdfdf24 hdfdfr8 hdfh hdfhd hdfhe hdfhx
syn keyword matlabFunc hdfinfo hdfml hdfpt hdfread hdftool hdfv hdfvf hdfvh
syn keyword matlabFunc hdfvs head heatmap height help helpbrowser helpdesk
syn keyword matlabFunc helpdlg helpwin hess hex2dec hex2num hgexport hggroup
syn keyword matlabFunc hgload hgsave hgtransform hidden highlight hilb hist
syn keyword matlabFunc histc histcounts histcounts2 histogram histogram2 hms
syn keyword matlabFunc hold holes home horzcat hot hour hours hover hsv
syn keyword matlabFunc hsv2rgb hypot i ichol idealfilter idivide if, elseif,
syn keyword matlabFunc else ifft ifft2 ifftn ifftshift ilu im2double im2frame
syn keyword matlabFunc im2java imag image imageDatastore imagesc imapprox
syn keyword matlabFunc imfinfo imformats imgCompress import importdata imread
syn keyword matlabFunc imresize imshow imtile imwrite incenter incenters
syn keyword matlabFunc incidence ind2rgb ind2sub indegree inedges Inf info
syn keyword matlabFunc infoImpl initialize initializeDatastore inline inmem
syn keyword matlabFunc inner2outer innerjoin inOutStatus inpolygon input
syn keyword matlabFunc inputdlg inputname insertAfter insertATbl insertBefore
syn keyword matlabFunc insertBTbl insertCol insertImg insertRows inShape
syn keyword matlabFunc inspect instrcallback instrfind instrfindall int16
syn keyword matlabFunc int2str int32 int64 int8 integral integral2 integral3
syn keyword matlabFunc interp1 interp1q interp2 interp3 interpft interpn
syn keyword matlabFunc interpstreamspeed intersect intmax intmin inv invhilb
syn keyword matlabFunc invoke ipermute iqr is* isa isappdata isaUnderlying
syn keyword matlabFunc isbanded isbetween iscalendarduration iscategorical
syn keyword matlabFunc iscategory iscell iscellstr ischange ischar iscolumn
syn keyword matlabFunc iscom isCompatible isCompressedImg isConnected isdag
syn keyword matlabFunc isdatetime isdiag isdir
syn keyword matlabFunc isDiscreteStateSpecificationMutableImpl isDone
syn keyword matlabFunc isDoneImpl isdst isduration isEdge isempty isenum
syn keyword matlabFunc isequal isequaln isequalwithequalnans isevent isfield
syn keyword matlabFunc isfile isfinite isfloat isfolder isfullfile isgraphics
syn keyword matlabFunc ishandle ishermitian ishghandle ishold ishole
syn keyword matlabFunc isIllConditioned isInactivePropertyImpl isinf
syn keyword matlabFunc isInputComplexityMutableImpl isInputDataTypeMutableImpl
syn keyword matlabFunc isInputDirectFeedthroughImpl isInputSizeLockedImpl
syn keyword matlabFunc isInputSizeMutableImpl isinteger isinterface isInterior
syn keyword matlabFunc isinterior isisomorphic isjava isKey iskeyword isletter
syn keyword matlabFunc isLoaded islocalmax islocalmin isLocked islogical ismac
syn keyword matlabFunc ismatrix ismember ismembertol ismethod ismissing
syn keyword matlabFunc ismultigraph isnan isnat isNull isnumeric isobject
syn keyword matlabFunc isocaps isocolors isomorphism isonormals isordinal
syn keyword matlabFunc isosurface isoutlier isOutputComplexImpl
syn keyword matlabFunc isOutputFixedSizeImpl ispc isplaying ispref isprime
syn keyword matlabFunc isprop isprotected isreal isrecording isregular isrow
syn keyword matlabFunc isscalar issimplified issorted issortedrows isspace
syn keyword matlabFunc issparse isstr isstring isStringScalar isstrprop
syn keyword matlabFunc isstruct isstudent issymmetric istable istall
syn keyword matlabFunc istimetable istril istriu
syn keyword matlabFunc isTunablePropertyDataTypeMutableImpl isundefined isunix
syn keyword matlabFunc isvalid isvarname isvector isweekend j javaaddpath
syn keyword matlabFunc javaArray javachk javaclasspath javaMethod
syn keyword matlabFunc javaMethodEDT javaObject javaObjectEDT javarmpath jet
syn keyword matlabFunc join jsondecode jsonencode juliandate keepMeasuring
syn keyword matlabFunc keyboard keys kron labeledge labelnode lag laplacian
syn keyword matlabFunc lasterr lasterror lastwarn layout lcm ldivide ldl le
syn keyword matlabFunc legend legendre length libfunctions libfunctionsview
syn keyword matlabFunc libisloaded libpointer libstruct license light
syn keyword matlabFunc lightangle lighting lin2mu line lines LineSpec linkaxes
syn keyword matlabFunc linkdata linkprop linsolve linspace listdlg listener
syn keyword matlabFunc listfonts listModifiedFiles listRequiredFiles load
syn keyword matlabFunc loadlibrary loadobj loadObjectImpl localfunctions log
syn keyword matlabFunc log10 log1p log2 logical Logical Operators: Short-
syn keyword matlabFunc circuit loglog logm logspace lookfor lower ls lscov
syn keyword matlabFunc lsqminnorm lsqnonneg lsqr lt lu magic makehgtform
syn keyword matlabFunc mapreduce mapreducer mat2cell mat2str matchpairs
syn keyword matlabFunc material matfile matlabrc matlabroot max maxflow
syn keyword matlabFunc MaximizeCommandWindow maxk maxNumCompThreads
syn keyword matlabFunc maxpartitions mean median memmapfile memoize memory
syn keyword matlabFunc menu mergecats mergevars mesh meshc meshgrid meshz
syn keyword matlabFunc metaclass methodsview mex mexext mexhost mfilename mget
syn keyword matlabFunc milliseconds min MinimizeCommandWindow mink minres
syn keyword matlabFunc minspantree minus minute minutes mislocked missing
syn keyword matlabFunc mkdir mkpp mldivide mlint mlintrpt mlock mmfileinfo mod
syn keyword matlabFunc mode month more morebins movAbsHDU move movefile
syn keyword matlabFunc movegui movevars movie movmad movmax movmean movmedian
syn keyword matlabFunc movmin movNamHDU movprod movRelHDU movstd movsum movvar
syn keyword matlabFunc mpower mput mrdivide msgbox mtimes mu2lin multibandread
syn keyword matlabFunc multibandwrite munlock mustBeFinite mustBeGreaterThan
syn keyword matlabFunc mustBeGreaterThanOrEqual mustBeInteger mustBeLessThan
syn keyword matlabFunc mustBeLessThanOrEqual mustBeMember mustBeNegative
syn keyword matlabFunc mustBeNonempty mustBeNonNan mustBeNonnegative
syn keyword matlabFunc mustBeNonpositive mustBeNonsparse mustBeNonzero
syn keyword matlabFunc mustBeNumeric mustBeNumericOrLogical mustBePositive
syn keyword matlabFunc mustBeReal namelengthmax NaN nargchk nargin narginchk
syn keyword matlabFunc nargout nargoutchk NaT native2unicode nccreate ncdisp
syn keyword matlabFunc nchoosek ncinfo ncread ncreadatt ncwrite ncwriteatt
syn keyword matlabFunc ncwriteschema ndgrid ndims ne nearest nearestNeighbor
syn keyword matlabFunc nearestvertex neighbors NET newline newplot nextfile
syn keyword matlabFunc nextpow2 nnz nonzeros norm normalize normest not
syn keyword matlabFunc notebook notify now nsidedpoly nthroot null num2cell
syn keyword matlabFunc num2hex num2ruler num2str numArgumentsFromSubscript
syn keyword matlabFunc numboundaries numedges numel numnodes numpartitions
syn keyword matlabFunc numRegions numsides nzmax ode113 ode15i ode15s ode23
syn keyword matlabFunc ode23s ode23t ode23tb ode45 odeget odeset odextend
syn keyword matlabFunc onCleanup ones onFailure open openDiskFile openfig
syn keyword matlabFunc openFile opengl openProject openvar optimget optimset
syn keyword matlabFunc or ordeig orderfields ordqz ordschur orient orth
syn keyword matlabFunc outdegree outedges outerjoin outputImpl overlaps pack
syn keyword matlabFunc pad padecoef pagesetupdlg pan panInteraction
syn keyword matlabFunc parallelplot pareto parfor parquetDatastore parquetinfo
syn keyword matlabFunc parquetread parquetwrite parse parseSoapResponse
syn keyword matlabFunc partition parula pascal patch path path2rc pathsep
syn keyword matlabFunc pathtool pause pbaspect pcg pchip pcode pcolor pdepe
syn keyword matlabFunc pdeval peaks perimeter perl perms permute pi pie pie3
syn keyword matlabFunc pink pinv planerot play playblocking plot plot3
syn keyword matlabFunc plotbrowser plotedit plotmatrix plottools plotyy plus
syn keyword matlabFunc pointLocation pol2cart polar polaraxes polarhistogram
syn keyword matlabFunc polarplot polarscatter poly polyarea polybuffer polyder
syn keyword matlabFunc polyeig polyfit polyint polyshape polyval polyvalm
syn keyword matlabFunc posixtime pow2 power ppval predecessors prefdir
syn keyword matlabFunc preferences preferredBufferSize press preview primes
syn keyword matlabFunc print printdlg printopt printpreview prism
syn keyword matlabFunc processInputSpecificationChangeImpl
syn keyword matlabFunc processTunedPropertiesImpl prod profile profsave
syn keyword matlabFunc progress propagatedInputComplexity
syn keyword matlabFunc propagatedInputDataType propagatedInputFixedSize
syn keyword matlabFunc propagatedInputSize propedit propertyeditor psi publish
syn keyword matlabFunc PutCharArray putData PutFullMatrix PutWorkspaceData pwd
syn keyword matlabFunc pyargs pyversion qmr qr qrdelete qrinsert qrupdate quad
syn keyword matlabFunc quad2d quadgk quadl quadv quarter questdlg quit Quit
syn keyword matlabFunc quiver quiver3 qz rad2deg rand randi randn randperm
syn keyword matlabFunc RandStream rank rat rats rbbox rcond rdivide read
syn keyword matlabFunc readall readasync readATblHdr readBTblHdr readCard
syn keyword matlabFunc readcell readCol readFrame readimage readImg readKey
syn keyword matlabFunc readKeyCmplx readKeyDbl readKeyLongLong readKeyLongStr
syn keyword matlabFunc readKeyUnit readmatrix readRecord readtable
syn keyword matlabFunc readtimetable readvars real reallog realmax realmin
syn keyword matlabFunc realpow realsqrt record recordblocking rectangle
syn keyword matlabFunc rectint recycle reducepatch reducevolume refresh
syn keyword matlabFunc refreshdata refreshSourceControl regexp regexpi
syn keyword matlabFunc regexprep regexptranslate regions regionZoomInteraction
syn keyword matlabFunc registerevent regmatlabserver rehash
syn keyword matlabFunc relationaloperators release releaseImpl reload rem
syn keyword matlabFunc remove Remove RemoveAll removeCategory removecats
syn keyword matlabFunc removeFields removeFile removeLabel removeParameter
syn keyword matlabFunc removePath removeReference removeShortcut
syn keyword matlabFunc removeShutdownFile removeStartupFile
syn keyword matlabFunc removeToolbarExplorationButtons removets removevars
syn keyword matlabFunc rename renamecats rendererinfo reordercats reordernodes
syn keyword matlabFunc repeat repelem replace replaceBetween replaceFields
syn keyword matlabFunc repmat reportFinalizedResult resample rescale reset
syn keyword matlabFunc resetImpl reshape residue resolve restartable
syn keyword matlabFunc restoredefaultpath result resume retime
syn keyword matlabFunc returnStoredValueWhen reusable reverse rgb2gray rgb2hsv
syn keyword matlabFunc rgb2ind rgbplot ribbon rlim rmappdata rmboundary rmdir
syn keyword matlabFunc rmedge rmfield rmholes rmmissing rmnode rmoutliers
syn keyword matlabFunc rmpath rmpref rmprop rmslivers rng roots rose rosser
syn keyword matlabFunc rot90 rotate rotate3d rotateInteraction round rowfun
syn keyword matlabFunc rows2vars rref rsf2csf rtickangle rtickformat
syn keyword matlabFunc rticklabels rticks ruler2num rulerPanInteraction run
syn keyword matlabFunc runInParallel runperf runTest runTestClass
syn keyword matlabFunc runTestMethod runtests runTestSuite samplefun
syn keyword matlabFunc sampleSummary satisfiedBy save saveas savefig saveobj
syn keyword matlabFunc saveObjectImpl savepath scale scatter scatter3
syn keyword matlabFunc scatteredInterpolant scatterhistogram schur scroll sec
syn keyword matlabFunc secd sech second seconds seek selectFailed selectIf
syn keyword matlabFunc selectIncomplete selectLogged selectmoveresize
syn keyword matlabFunc selectPassed semilogx semilogy send sendmail serial
syn keyword matlabFunc serialbreak seriallist set setabstime setappdata
syn keyword matlabFunc setBscale setcats setCompressionType setdatatype
syn keyword matlabFunc setdiff setdisp setenv setfield setHCompScale
syn keyword matlabFunc setHCompSmooth setinterpmethod setParameter
syn keyword matlabFunc setpixelposition setpref setProperties setstr
syn keyword matlabFunc setTileDim settimeseriesnames settings setToValue
syn keyword matlabFunc setTscale setuniformtime setup setupImpl
syn keyword matlabFunc setupSharedTestFixture setupTestClass setupTestMethod
syn keyword matlabFunc setvaropts setvartype setxor sgtitle shading sheetnames
syn keyword matlabFunc shg shiftdim shortestpath shortestpathtree show
syn keyword matlabFunc showFiSettingsImpl showplottool showSimulateUsingImpl
syn keyword matlabFunc shrinkfaces shuffle sign simplify sin sind single sinh
syn keyword matlabFunc sinpi size slice smooth3 smoothdata snapnow sort
syn keyword matlabFunc sortboundaries sortByFixtures sortregions sortrows
syn keyword matlabFunc sortx sorty sound soundsc spalloc sparse spaugment
syn keyword matlabFunc spconvert spdiags specular speye spfun sph2cart sphere
syn keyword matlabFunc spinmap spline split splitapply splitEachLabel
syn keyword matlabFunc splitlines splitvars spones spparms sprand sprandn
syn keyword matlabFunc sprandsym sprank spreadsheetDatastore spring sprintf
syn keyword matlabFunc spy sqrt sqrtm squeeze ss2tf sscanf stack stackedplot
syn keyword matlabFunc stairs standardizeMissing start startat startMeasuring
syn keyword matlabFunc startsWith startup stats std stem stem3 step stepImpl
syn keyword matlabFunc stlread stlwrite stop stopasync stopMeasuring
syn keyword matlabFunc storeValueWhen str2double str2func str2mat str2num
syn keyword matlabFunc strcat strcmp strcmpi stream2 stream3 streamline
syn keyword matlabFunc streamparticles streamribbon streamslice streamtube
syn keyword matlabFunc strfind string strings strip strjoin strjust strlength
syn keyword matlabFunc strmatch strncmp strncmpi strread strrep strsplit
syn keyword matlabFunc strtok strtrim struct struct2cell struct2table
syn keyword matlabFunc structfun strvcat sub2ind subgraph subplot subsasgn
syn keyword matlabFunc subset subsindex subspace subsref substruct subtract
syn keyword matlabFunc subvolume successors sum summary summer superclasses
syn keyword matlabFunc support supportPackageInstaller supports
syn keyword matlabFunc supportsMultipleInstanceImpl surf surf2patch surface
syn keyword matlabFunc surfaceArea surfc surfl surfnorm svd svds swapbytes
syn keyword matlabFunc switch, case, otherwise sylvester symamd symbfact
syn keyword matlabFunc symmlq symrcm symvar synchronize syntax system table
syn keyword matlabFunc table2array table2cell table2struct table2timetable
syn keyword matlabFunc tabularTextDatastore tail tall tallrng tan tand tanh
syn keyword matlabFunc tar tcpclient teardown teardownSharedTestFixture
syn keyword matlabFunc teardownTestClass teardownTestMethod tempdir tempname
syn keyword matlabFunc testsuite tetramesh texlabel text textread textscan
syn keyword matlabFunc textwrap tfqmr then thetalim thetatickformat
syn keyword matlabFunc thetaticklabels thetaticks thingSpeakRead
syn keyword matlabFunc thingSpeakWrite throw throwAsCaller throwExceptionWhen
syn keyword matlabFunc tic time timeit timeofday timerange timerfind
syn keyword matlabFunc timerfindall times timetable timetable2table timezones
syn keyword matlabFunc title toc todatenum toeplitz toolboxdir topkrows
syn keyword matlabFunc toposort trace transclosure transform translate
syn keyword matlabFunc transpose transreduction trapz treelayout treeplot
syn keyword matlabFunc triangulation tril trimesh triplequad triplot TriRep
syn keyword matlabFunc TriScatteredInterp trisurf triu true try, catch
syn keyword matlabFunc tsearchn turningdist type typecast tzoffset uialert
syn keyword matlabFunc uiaxes uibutton uibuttongroup uicheckbox uiconfirm
syn keyword matlabFunc uicontextmenu uicontrol uidatepicker uidropdown
syn keyword matlabFunc uieditfield uifigure uigauge uigetdir uigetfile
syn keyword matlabFunc uigetpref uigridlayout uiimage uiimport uiknob uilabel
syn keyword matlabFunc uilamp uilistbox uimenu uint16 uint32 uint64 uint8
syn keyword matlabFunc uiopen uipanel uiprogressdlg uipushtool uiputfile
syn keyword matlabFunc uiradiobutton uiresume uisave uisetcolor uisetfont
syn keyword matlabFunc uisetpref uislider uispinner uistack uiswitch uitab
syn keyword matlabFunc uitabgroup uitable uitextarea uitogglebutton
syn keyword matlabFunc uitoggletool uitoolbar uitree uitreenode uiwait uminus
syn keyword matlabFunc underlyingValue undocheckout unicode2native union
syn keyword matlabFunc unique uniquetol unix unloadlibrary unmesh unmkpp
syn keyword matlabFunc unregisterallevents unregisterevent unstack untar
syn keyword matlabFunc unwrap unzip updateDependencies updateImpl
syn keyword matlabFunc upgradePreviouslyInstalledSupportPackages uplus upper
syn keyword matlabFunc urlread urlwrite usejava userpath validate
syn keyword matlabFunc validateattributes validateFunctionSignaturesJSON
syn keyword matlabFunc validateInputsImpl validatePropertiesImpl
syn keyword matlabFunc validatestring values vander var varargin varargout
syn keyword matlabFunc varfun vartype vecnorm vectorize ver verctrl
syn keyword matlabFunc verifyAccessed verifyCalled verifyClass verifyEmpty
syn keyword matlabFunc verifyEqual verifyError verifyFail verifyFalse
syn keyword matlabFunc verifyGreaterThan verifyGreaterThanOrEqual
syn keyword matlabFunc verifyInstanceOf verifyLength verifyLessThan
syn keyword matlabFunc verifyLessThanOrEqual verifyMatches verifyNotAccessed
syn keyword matlabFunc verifyNotCalled verifyNotEmpty verifyNotEqual
syn keyword matlabFunc verifyNotSameHandle verifyNotSet verifyNumElements
syn keyword matlabFunc verifyReturnsTrue verifySameHandle verifySet verifySize
syn keyword matlabFunc verifySubstring verifyThat verifyTrue verifyUsing
syn keyword matlabFunc verifyWarning verifyWarningFree verLessThan version
syn keyword matlabFunc vertcat vertexAttachments vertexNormal VideoReader
syn keyword matlabFunc VideoWriter view viewmtx visdiff volume volumebounds
syn keyword matlabFunc voronoi voronoiDiagram voronoin wait waitbar waitfor
syn keyword matlabFunc waitforbuttonpress warndlg warning waterfall web
syn keyword matlabFunc weboptions webread websave webwrite week weekday what
syn keyword matlabFunc whatsnew when which whitebg who whos width wilkinson
syn keyword matlabFunc winopen winqueryreg winter withAnyInputs
syn keyword matlabFunc withExactInputs withNargout withtol wordcloud write
syn keyword matlabFunc writecell writeChecksum writeCol writeComment writeDate
syn keyword matlabFunc writeHistory writeImg writeKey writeKeyUnit writematrix
syn keyword matlabFunc writetable writetimetable writeVideo xcorr xcov xlabel
syn keyword matlabFunc xlim xline xlsfinfo xlsread xlswrite xmlread xmlwrite
syn keyword matlabFunc xor xslt xtickangle xtickformat xticklabels xticks year
syn keyword matlabFunc years ylabel ylim yline ymd ytickangle ytickformat
syn keyword matlabFunc yticklabels yticks yyaxis yyyymmdd zeros zip zlabel
syn keyword matlabFunc zlim zoom zoomInteraction ztickangle ztickformat
syn keyword matlabFunc zticklabels zticks
syn keyword matlabFunc contains

" Define the default highlighting.
" For version 5.7 and earlier: only when not done already
" For version 5.8 and later: only when an item doesn't have highlighting yet
if version >= 508 || !exists("did_matlab_syntax_inits")
  if version < 508
    let did_matlab_syntax_inits = 1
    command -nargs=+ HiLink hi link <args>
  else
    command -nargs=+ HiLink hi def link <args>
  endif

  HiLink matlabTransposeOperator	matlabOperator
  HiLink matlabLineContinuation		Special
  HiLink matlabLabel			Label
  HiLink matlabConditional		Conditional
  HiLink matlabRepeat			Repeat
  HiLink matlabTodo			Todo
  HiLink matlabString			String
  HiLink matlabDelimiter		Identifier
  HiLink matlabTransposeOther		Identifier
  HiLink matlabNumber			Number
  HiLink matlabFloat			Float
  HiLink matlabConstant			Constant
  HiLink matlabImplicit			matlabStatement
  HiLink matlabStatement		Statement
  HiLink matlabSemicolon		SpecialChar
  HiLink matlabComment			Comment
  HiLink matlabBlockComment		Comment
  HiLink matlabImport			Include
  HiLink matlabBoolean			Boolean
  HiLink matlabStorageClass		StorageClass

  HiLink matlabArithmeticOperator	matlabOperator
  HiLink matlabRelationalOperator	matlabOperator
  HiLink matlabLogicalOperator		matlabOperator
  HiLink matlabOperator			Operator
  HiLink matlabExceptions		Exception
  HiLink matlabFunc			Function

"optional highlighting
  "HiLink matlabIdentifier		Identifier
  "HiLink matlabTab			Error
  delcommand HiLink
endif

let b:current_syntax = "matlab"

"EOF	vim: ts=8 noet tw=100 sw=8 sts=0
