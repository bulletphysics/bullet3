//Maya ASCII 8.5 scene
//Name: onepeanut.ma
//Last modified: Tue, Jul 22, 2008 03:48:08 PM
//Codeset: ANSI_X3.4-1968
requires maya "8.5";
currentUnit -l centimeter -a degree -t film;
fileInfo "application" "maya";
createNode transform -s -n "persp";
	setAttr ".v" no;
	setAttr ".t" -type "double3" -14.800230385627952 -10.730054280029755 1.8172397891678551 ;
	setAttr ".r" -type "double3" -215.73835272830993 -262.99999999998937 -2.5444437451708134e-14 ;
createNode camera -s -n "perspShape" -p "persp";
	setAttr -k off ".v" no;
	setAttr ".fl" 34.999999999999993;
	setAttr ".coi" 18.370717045650842;
	setAttr ".imn" -type "string" "persp";
	setAttr ".den" -type "string" "persp_depth";
	setAttr ".man" -type "string" "persp_mask";
	setAttr ".hc" -type "string" "viewSet -p %camera";
createNode transform -s -n "top";
	setAttr ".v" no;
	setAttr ".t" -type "double3" 0 100.1 0 ;
	setAttr ".r" -type "double3" -89.999999999999986 0 0 ;
createNode camera -s -n "topShape" -p "top";
	setAttr -k off ".v" no;
	setAttr ".rnd" no;
	setAttr ".coi" 100.1;
	setAttr ".ow" 30;
	setAttr ".imn" -type "string" "top";
	setAttr ".den" -type "string" "top_depth";
	setAttr ".man" -type "string" "top_mask";
	setAttr ".hc" -type "string" "viewSet -t %camera";
	setAttr ".o" yes;
createNode transform -s -n "front";
	setAttr ".v" no;
	setAttr ".t" -type "double3" 0 0 100.1 ;
createNode camera -s -n "frontShape" -p "front";
	setAttr -k off ".v" no;
	setAttr ".rnd" no;
	setAttr ".coi" 100.1;
	setAttr ".ow" 30;
	setAttr ".imn" -type "string" "front";
	setAttr ".den" -type "string" "front_depth";
	setAttr ".man" -type "string" "front_mask";
	setAttr ".hc" -type "string" "viewSet -f %camera";
	setAttr ".o" yes;
createNode transform -s -n "side";
	setAttr ".v" no;
	setAttr ".t" -type "double3" 100.1 0 0 ;
	setAttr ".r" -type "double3" 0 89.999999999999986 0 ;
createNode camera -s -n "sideShape" -p "side";
	setAttr -k off ".v" no;
	setAttr ".rnd" no;
	setAttr ".coi" 100.1;
	setAttr ".ow" 30;
	setAttr ".imn" -type "string" "side";
	setAttr ".den" -type "string" "side_depth";
	setAttr ".man" -type "string" "side_mask";
	setAttr ".hc" -type "string" "viewSet -s %camera";
	setAttr ".o" yes;
createNode transform -n "pSphere1";
createNode mesh -n "pSphereShape1" -p "pSphere1";
	setAttr -k off ".v";
	setAttr ".vir" yes;
	setAttr ".vif" yes;
	setAttr ".uvst[0].uvsn" -type "string" "map1";
	setAttr ".cuvs" -type "string" "map1";
	setAttr ".dcc" -type "string" "Ambient+Diffuse";
	setAttr ".covm[0]"  0 1 1;
	setAttr ".cdvm[0]"  0 1 1;
	setAttr -s 92 ".pt[0:91]" -type "float3"  -0.2574994 1.1365615 0.12808901 -0.082004264 
		1.1365615 0.24063207 0.17356716 1.1365615 0.24063204 0.25063691 1.1365615 0.12808898 
		0.30980462 1.1365615 -0.054009609 0.25063688 1.1365615 -0.23610812 0.095734812 0.95348024 
		0.027852129 -0.095734924 0.95348024 0.027852133 -0.22844759 1.1365615 -0.23610808 
		-0.25130823 1.1365615 -0.054009598 -0.58973986 0.96681732 0.29236239 -0.18209852 
		0.81107831 0.11910005 0.18209857 0.81107831 0.11910014 0.56631935 0.9230758 0.29236227 
		0.66480297 0.96681732 -0.054009628 0.47674012 0.81107831 -0.0082690716 0.18209849 
		0.81107831 -0.1577463 -0.18209858 0.81107831 -0.15774626 -0.47674012 0.81107831 -0.0082688872 
		-0.62435281 0.96681732 -0.054009598 -0.65617657 0.58928299 0.085604019 -0.25063688 
		0.58928299 0.20351477 0.25063699 0.58928299 0.20351465 0.65617663 0.63782948 0.42273057 
		0.92526954 0.70243376 -0.054009631 0.65617651 0.58928299 -0.099300683 0.25063682 
		0.58928299 -0.30503851 -0.25063699 0.58928299 -0.30503842 -0.65617651 0.58928299 
		-0.099300563 -0.92040229 0.70243376 -0.054009598 -0.7713815 0.30980459 0.11910006 
		-0.29464149 0.30980459 0.25771224 0.29464158 0.30980459 0.25771236 0.7713815 0.30980459 
		0.11910014 1.1083192 0.36929098 -0.054009635 0.77138138 0.30980459 -0.15774632 0.29464146 
		0.30980459 -0.39960572 -0.29464155 0.30980459 -0.3996056 -0.77138138 0.30980459 -0.1577463 
		-1.1104779 0.36929098 -0.054009598 -0.81107849 0 0.13064188 -0.30980459 0 0.27638754 
		0.3098048 0 0.27638742 0.81107849 0 0.1306421 1.1713946 0 -0.054009639 0.81107837 
		0 -0.17788558 0.30980453 0 -0.43219143 -0.30980474 0 -0.43219143 -0.81107837 0 -0.17788543 
		-1.1759737 0 -0.054009598 -0.7713815 -0.30980459 0.11910006 -0.29464149 -0.30980459 
		0.25771224 0.29464158 -0.30980459 0.25771236 0.7713815 -0.30980459 0.11910014 1.1083192 
		-0.36929098 -0.054009635 0.77138138 -0.30980459 -0.15774632 0.29464146 -0.30980459 
		-0.39960572 -0.29464155 -0.30980459 -0.3996056 -0.77138138 -0.30980459 -0.1577463 
		-1.1104779 -0.36929098 -0.054009598 -0.65617657 -0.58928299 0.085604019 -0.25063688 
		-0.58928299 0.20351477 0.25063699 -0.58928299 0.20351465 0.67061043 -0.68686998 0.42273057 
		0.92526954 -0.70243376 -0.054009631 0.65617651 -0.58928299 -0.099300683 0.25063682 
		-0.58928299 -0.30503851 -0.25063699 -0.58928299 -0.30503842 -0.65617651 -0.58928299 
		-0.099300563 -1.0368359 -0.70243376 -0.054009598 -0.58973986 -0.96681732 0.29236239 
		-0.18209852 -0.81107831 0.11910005 0.18209857 -0.81107831 0.11910014 0.49982405 -0.96681732 
		0.29236227 0.64016199 -0.96681732 -0.054009628 0.47674012 -0.81107831 -0.0082690716 
		0.18209849 -0.81107831 -0.1577463 -0.18209858 -0.81107831 -0.15774626 -0.47674012 
		-0.81107831 -0.0082688872 -0.74078602 -0.96681732 -0.054009598 -0.28876579 -1.1365615 
		0.12808901 -0.082004264 -1.1365615 0.24063207 0.095734872 -1.1365615 0.24063204 0.25063691 
		-1.1365615 0.12808898 0.30980462 -1.1365615 -0.054009609 0.25063688 -1.1365615 -0.23610812 
		0.095734812 -0.95348024 0.027852129 -0.095734924 -0.95348024 0.027852133 -0.25063694 
		-1.1365615 -0.23610808 -0.36774182 -1.1365615 -0.054009598 0.079984799 1.195052 -0.054009598 
		0.045781415 -1.195052 -0.054009598;
	setAttr ".dr" 1;
createNode lightLinker -n "lightLinker1";
	setAttr -s 2 ".lnk";
	setAttr -s 2 ".slnk";
createNode displayLayerManager -n "layerManager";
createNode displayLayer -n "defaultLayer";
createNode renderLayerManager -n "renderLayerManager";
createNode renderLayer -n "defaultRenderLayer";
	setAttr ".g" yes;
createNode polySphere -n "polySphere1";
	setAttr ".r" 1.4460482431866666;
	setAttr ".sa" 10;
	setAttr ".sh" 10;
createNode script -n "sceneConfigurationScriptNode";
	setAttr ".b" -type "string" "playbackOptions -min 1 -max 24 -ast 1 -aet 48 ";
	setAttr ".st" 6;
select -ne :time1;
	setAttr ".o" 1;
select -ne :renderPartition;
	setAttr -s 2 ".st";
select -ne :renderGlobalsList1;
select -ne :defaultShaderList1;
	setAttr -s 2 ".s";
select -ne :postProcessList1;
	setAttr -s 2 ".p";
select -ne :lightList1;
select -ne :initialShadingGroup;
	setAttr ".ro" yes;
select -ne :initialParticleSE;
	setAttr ".ro" yes;
select -ne :defaultRenderGlobals;
	setAttr ".npu" 1;
select -ne :hardwareRenderGlobals;
	setAttr ".ctrs" 256;
	setAttr ".btrs" 512;
select -ne :defaultHardwareRenderGlobals;
	setAttr ".fn" -type "string" "im";
	setAttr ".res" -type "string" "ntsc_4d 646 485 1.333";
connectAttr "polySphere1.out" "pSphereShape1.i";
connectAttr ":defaultLightSet.msg" "lightLinker1.lnk[0].llnk";
connectAttr ":initialShadingGroup.msg" "lightLinker1.lnk[0].olnk";
connectAttr ":defaultLightSet.msg" "lightLinker1.lnk[1].llnk";
connectAttr ":initialParticleSE.msg" "lightLinker1.lnk[1].olnk";
connectAttr ":defaultLightSet.msg" "lightLinker1.slnk[0].sllk";
connectAttr ":initialShadingGroup.msg" "lightLinker1.slnk[0].solk";
connectAttr ":defaultLightSet.msg" "lightLinker1.slnk[1].sllk";
connectAttr ":initialParticleSE.msg" "lightLinker1.slnk[1].solk";
connectAttr "layerManager.dli[0]" "defaultLayer.id";
connectAttr "renderLayerManager.rlmi[0]" "defaultRenderLayer.rlid";
connectAttr "lightLinker1.msg" ":lightList1.ln" -na;
connectAttr "pSphereShape1.iog" ":initialShadingGroup.dsm" -na;
// End of onepeanut.ma
