//Maya ASCII 8.5 scene
//Name: donuts.ma
//Last modified: Tue, Jul 22, 2008 03:46:23 PM
//Codeset: ANSI_X3.4-1968
requires maya "8.5";
requires "dynamica" "1.0";
currentUnit -l centimeter -a degree -t film;
fileInfo "application" "maya";
createNode transform -s -n "persp";
	setAttr ".v" no;
	setAttr ".t" -type "double3" 41.083860079555834 34.218971533823932 87.868595563990425 ;
	setAttr ".r" -type "double3" -22.538352729605073 23.400000000000961 -8.663957098263457e-16 ;
createNode camera -s -n "perspShape" -p "persp";
	setAttr -k off ".v" no;
	setAttr ".fl" 34.999999999999986;
	setAttr ".coi" 102.00909559719346;
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
createNode transform -n "pTorus1";
	setAttr ".v" no;
	setAttr ".t" -type "double3" 8.9212312542979149 0 3.5083436775822037 ;
createNode mesh -n "pTorusShape1" -p "pTorus1";
	setAttr -k off ".v";
	setAttr ".vir" yes;
	setAttr ".vif" yes;
	setAttr ".uvst[0].uvsn" -type "string" "map1";
	setAttr ".cuvs" -type "string" "map1";
	setAttr ".dcc" -type "string" "Ambient+Diffuse";
	setAttr ".covm[0]"  0 1 1;
	setAttr ".cdvm[0]"  0 1 1;
createNode transform -n "dRigidBodyArray1";
createNode dRigidBodyArray -n "dRigidBodyArrayShape1" -p "dRigidBodyArray1";
	setAttr -k off ".v";
	setAttr ".nbds" 1000;
	setAttr -s 1000 ".inpo";
	setAttr ".inpo[0:165]" -type "float3"  0 10 0 2 10 0 4 10 0 6 10 0 8 10 0 10 10 
		0 12 10 0 14 10 0 16 10 0 18 10 0 0 10 2 2 10 2 4 10 2 6 10 2 8 10 2 10 10 2 12 10 
		2 14 10 2 16 10 2 18 10 2 0 10 4 2 10 4 4 10 4 6 10 4 8 10 4 10 10 4 12 10 4 14 10 
		4 16 10 4 18 10 4 0 10 6 2 10 6 4 10 6 6 10 6 8 10 6 10 10 6 12 10 6 14 10 6 16 10 
		6 18 10 6 0 10 8 2 10 8 4 10 8 6 10 8 8 10 8 10 10 8 12 10 8 14 10 8 16 10 8 18 10 
		8 0 10 10 2 10 10 4 10 10 6 10 10 8 10 10 10 10 10 12 10 10 14 10 10 16 10 10 18 
		10 10 0 10 12 2 10 12 4 10 12 6 10 12 8 10 12 10 10 12 12 10 12 14 10 12 16 10 12 
		18 10 12 0 10 14 2 10 14 4 10 14 6 10 14 8 10 14 10 10 14 12 10 14 14 10 14 16 10 
		14 18 10 14 0 10 16 2 10 16 4 10 16 6 10 16 8 10 16 10 10 16 12 10 16 14 10 16 16 
		10 16 18 10 16 0 10 18 2 10 18 4 10 18 6 10 18 8 10 18 10 10 18 12 10 18 14 10 18 
		16 10 18 18 10 18 0 12 0 2 12 0 4 12 0 6 12 0 8 12 0 10 12 0 12 12 0 14 12 0 16 12 
		0 18 12 0 0 12 2 2 12 2 4 12 2 6 12 2 8 12 2 10 12 2 12 12 2 14 12 2 16 12 2 18 12 
		2 0 12 4 2 12 4 4 12 4 6 12 4 8 12 4 10 12 4 12 12 4 14 12 4 16 12 4 18 12 4 0 12 
		6 2 12 6 4 12 6 6 12 6 8 12 6 10 12 6 12 12 6 14 12 6 16 12 6 18 12 6 0 12 8 2 12 
		8 4 12 8 6 12 8 8 12 8 10 12 8 12 12 8 14 12 8 16 12 8 18 12 8 0 12 10 2 12 10 4 
		12 10 6 12 10 8 12 10 10 12 10 12 12 10 14 12 10 16 12 10 18 12 10 0 12 12 2 12 12 
		4 12 12 6 12 12 8 12 12 10 12 12;
	setAttr ".inpo[166:331]" 12 12 12 14 12 12 16 12 12 18 12 12 0 12 14 2 12 14 4 12 
		14 6 12 14 8 12 14 10 12 14 12 12 14 14 12 14 16 12 14 18 12 14 0 12 16 2 12 16 4 
		12 16 6 12 16 8 12 16 10 12 16 12 12 16 14 12 16 16 12 16 18 12 16 0 12 18 2 12 18 
		4 12 18 6 12 18 8 12 18 10 12 18 12 12 18 14 12 18 16 12 18 18 12 18 0 14 0 2 14 
		0 4 14 0 6 14 0 8 14 0 10 14 0 12 14 0 14 14 0 16 14 0 18 14 0 0 14 2 2 14 2 4 14 
		2 6 14 2 8 14 2 10 14 2 12 14 2 14 14 2 16 14 2 18 14 2 0 14 4 2 14 4 4 14 4 6 14 
		4 8 14 4 10 14 4 12 14 4 14 14 4 16 14 4 18 14 4 0 14 6 2 14 6 4 14 6 6 14 6 8 14 
		6 10 14 6 12 14 6 14 14 6 16 14 6 18 14 6 0 14 8 2 14 8 4 14 8 6 14 8 8 14 8 10 14 
		8 12 14 8 14 14 8 16 14 8 18 14 8 0 14 10 2 14 10 4 14 10 6 14 10 8 14 10 10 14 10 
		12 14 10 14 14 10 16 14 10 18 14 10 0 14 12 2 14 12 4 14 12 6 14 12 8 14 12 10 14 
		12 12 14 12 14 14 12 16 14 12 18 14 12 0 14 14 2 14 14 4 14 14 6 14 14 8 14 14 10 
		14 14 12 14 14 14 14 14 16 14 14 18 14 14 0 14 16 2 14 16 4 14 16 6 14 16 8 14 16 
		10 14 16 12 14 16 14 14 16 16 14 16 18 14 16 0 14 18 2 14 18 4 14 18 6 14 18 8 14 
		18 10 14 18 12 14 18 14 14 18 16 14 18 18 14 18 0 16 0 2 16 0 4 16 0 6 16 0 8 16 
		0 10 16 0 12 16 0 14 16 0 16 16 0 18 16 0 0 16 2 2 16 2 4 16 2 6 16 2 8 16 2 10 16 
		2 12 16 2 14 16 2 16 16 2 18 16 2 0 16 4 2 16 4 4 16 4 6 16 4 8 16 4 10 16 4 12 16 
		4 14 16 4 16 16 4 18 16 4 0 16 6 2 16 6;
	setAttr ".inpo[332:497]" 4 16 6 6 16 6 8 16 6 10 16 6 12 16 6 14 16 6 16 16 6 18 
		16 6 0 16 8 2 16 8 4 16 8 6 16 8 8 16 8 10 16 8 12 16 8 14 16 8 16 16 8 18 16 8 0 
		16 10 2 16 10 4 16 10 6 16 10 8 16 10 10 16 10 12 16 10 14 16 10 16 16 10 18 16 10 
		0 16 12 2 16 12 4 16 12 6 16 12 8 16 12 10 16 12 12 16 12 14 16 12 16 16 12 18 16 
		12 0 16 14 2 16 14 4 16 14 6 16 14 8 16 14 10 16 14 12 16 14 14 16 14 16 16 14 18 
		16 14 0 16 16 2 16 16 4 16 16 6 16 16 8 16 16 10 16 16 12 16 16 14 16 16 16 16 16 
		18 16 16 0 16 18 2 16 18 4 16 18 6 16 18 8 16 18 10 16 18 12 16 18 14 16 18 16 16 
		18 18 16 18 0 18 0 2 18 0 4 18 0 6 18 0 8 18 0 10 18 0 12 18 0 14 18 0 16 18 0 18 
		18 0 0 18 2 2 18 2 4 18 2 6 18 2 8 18 2 10 18 2 12 18 2 14 18 2 16 18 2 18 18 2 0 
		18 4 2 18 4 4 18 4 6 18 4 8 18 4 10 18 4 12 18 4 14 18 4 16 18 4 18 18 4 0 18 6 2 
		18 6 4 18 6 6 18 6 8 18 6 10 18 6 12 18 6 14 18 6 16 18 6 18 18 6 0 18 8 2 18 8 4 
		18 8 6 18 8 8 18 8 10 18 8 12 18 8 14 18 8 16 18 8 18 18 8 0 18 10 2 18 10 4 18 10 
		6 18 10 8 18 10 10 18 10 12 18 10 14 18 10 16 18 10 18 18 10 0 18 12 2 18 12 4 18 
		12 6 18 12 8 18 12 10 18 12 12 18 12 14 18 12 16 18 12 18 18 12 0 18 14 2 18 14 4 
		18 14 6 18 14 8 18 14 10 18 14 12 18 14 14 18 14 16 18 14 18 18 14 0 18 16 2 18 16 
		4 18 16 6 18 16 8 18 16 10 18 16 12 18 16 14 18 16 16 18 16 18 18 16 0 18 18 2 18 
		18 4 18 18 6 18 18 8 18 18 10 18 18 12 18 18 14 18 18;
	setAttr ".inpo[498:663]" 16 18 18 18 18 18 0 20 0 2 20 0 4 20 0 6 20 0 8 20 0 10 
		20 0 12 20 0 14 20 0 16 20 0 18 20 0 0 20 2 2 20 2 4 20 2 6 20 2 8 20 2 10 20 2 12 
		20 2 14 20 2 16 20 2 18 20 2 0 20 4 2 20 4 4 20 4 6 20 4 8 20 4 10 20 4 12 20 4 14 
		20 4 16 20 4 18 20 4 0 20 6 2 20 6 4 20 6 6 20 6 8 20 6 10 20 6 12 20 6 14 20 6 16 
		20 6 18 20 6 0 20 8 2 20 8 4 20 8 6 20 8 8 20 8 10 20 8 12 20 8 14 20 8 16 20 8 18 
		20 8 0 20 10 2 20 10 4 20 10 6 20 10 8 20 10 10 20 10 12 20 10 14 20 10 16 20 10 
		18 20 10 0 20 12 2 20 12 4 20 12 6 20 12 8 20 12 10 20 12 12 20 12 14 20 12 16 20 
		12 18 20 12 0 20 14 2 20 14 4 20 14 6 20 14 8 20 14 10 20 14 12 20 14 14 20 14 16 
		20 14 18 20 14 0 20 16 2 20 16 4 20 16 6 20 16 8 20 16 10 20 16 12 20 16 14 20 16 
		16 20 16 18 20 16 0 20 18 2 20 18 4 20 18 6 20 18 8 20 18 10 20 18 12 20 18 14 20 
		18 16 20 18 18 20 18 0 22 0 2 22 0 4 22 0 6 22 0 8 22 0 10 22 0 12 22 0 14 22 0 16 
		22 0 18 22 0 0 22 2 2 22 2 4 22 2 6 22 2 8 22 2 10 22 2 12 22 2 14 22 2 16 22 2 18 
		22 2 0 22 4 2 22 4 4 22 4 6 22 4 8 22 4 10 22 4 12 22 4 14 22 4 16 22 4 18 22 4 0 
		22 6 2 22 6 4 22 6 6 22 6 8 22 6 10 22 6 12 22 6 14 22 6 16 22 6 18 22 6 0 22 8 2 
		22 8 4 22 8 6 22 8 8 22 8 10 22 8 12 22 8 14 22 8 16 22 8 18 22 8 0 22 10 2 22 10 
		4 22 10 6 22 10 8 22 10 10 22 10 12 22 10 14 22 10 16 22 10 18 22 10 0 22 12 2 22 
		12 4 22 12 6 22 12;
	setAttr ".inpo[664:829]" 8 22 12 10 22 12 12 22 12 14 22 12 16 22 12 18 22 12 0 
		22 14 2 22 14 4 22 14 6 22 14 8 22 14 10 22 14 12 22 14 14 22 14 16 22 14 18 22 14 
		0 22 16 2 22 16 4 22 16 6 22 16 8 22 16 10 22 16 12 22 16 14 22 16 16 22 16 18 22 
		16 0 22 18 2 22 18 4 22 18 6 22 18 8 22 18 10 22 18 12 22 18 14 22 18 16 22 18 18 
		22 18 0 24 0 2 24 0 4 24 0 6 24 0 8 24 0 10 24 0 12 24 0 14 24 0 16 24 0 18 24 0 
		0 24 2 2 24 2 4 24 2 6 24 2 8 24 2 10 24 2 12 24 2 14 24 2 16 24 2 18 24 2 0 24 4 
		2 24 4 4 24 4 6 24 4 8 24 4 10 24 4 12 24 4 14 24 4 16 24 4 18 24 4 0 24 6 2 24 6 
		4 24 6 6 24 6 8 24 6 10 24 6 12 24 6 14 24 6 16 24 6 18 24 6 0 24 8 2 24 8 4 24 8 
		6 24 8 8 24 8 10 24 8 12 24 8 14 24 8 16 24 8 18 24 8 0 24 10 2 24 10 4 24 10 6 24 
		10 8 24 10 10 24 10 12 24 10 14 24 10 16 24 10 18 24 10 0 24 12 2 24 12 4 24 12 6 
		24 12 8 24 12 10 24 12 12 24 12 14 24 12 16 24 12 18 24 12 0 24 14 2 24 14 4 24 14 
		6 24 14 8 24 14 10 24 14 12 24 14 14 24 14 16 24 14 18 24 14 0 24 16 2 24 16 4 24 
		16 6 24 16 8 24 16 10 24 16 12 24 16 14 24 16 16 24 16 18 24 16 0 24 18 2 24 18 4 
		24 18 6 24 18 8 24 18 10 24 18 12 24 18 14 24 18 16 24 18 18 24 18 0 26 0 2 26 0 
		4 26 0 6 26 0 8 26 0 10 26 0 12 26 0 14 26 0 16 26 0 18 26 0 0 26 2 2 26 2 4 26 2 
		6 26 2 8 26 2 10 26 2 12 26 2 14 26 2 16 26 2 18 26 2 0 26 4 2 26 4 4 26 4 6 26 4 
		8 26 4 10 26 4 12 26 4 14 26 4 16 26 4 18 26 4;
	setAttr ".inpo[830:995]" 0 26 6 2 26 6 4 26 6 6 26 6 8 26 6 10 26 6 12 26 6 14 26 
		6 16 26 6 18 26 6 0 26 8 2 26 8 4 26 8 6 26 8 8 26 8 10 26 8 12 26 8 14 26 8 16 26 
		8 18 26 8 0 26 10 2 26 10 4 26 10 6 26 10 8 26 10 10 26 10 12 26 10 14 26 10 16 26 
		10 18 26 10 0 26 12 2 26 12 4 26 12 6 26 12 8 26 12 10 26 12 12 26 12 14 26 12 16 
		26 12 18 26 12 0 26 14 2 26 14 4 26 14 6 26 14 8 26 14 10 26 14 12 26 14 14 26 14 
		16 26 14 18 26 14 0 26 16 2 26 16 4 26 16 6 26 16 8 26 16 10 26 16 12 26 16 14 26 
		16 16 26 16 18 26 16 0 26 18 2 26 18 4 26 18 6 26 18 8 26 18 10 26 18 12 26 18 14 
		26 18 16 26 18 18 26 18 0 28 0 2 28 0 4 28 0 6 28 0 8 28 0 10 28 0 12 28 0 14 28 
		0 16 28 0 18 28 0 0 28 2 2 28 2 4 28 2 6 28 2 8 28 2 10 28 2 12 28 2 14 28 2 16 28 
		2 18 28 2 0 28 4 2 28 4 4 28 4 6 28 4 8 28 4 10 28 4 12 28 4 14 28 4 16 28 4 18 28 
		4 0 28 6 2 28 6 4 28 6 6 28 6 8 28 6 10 28 6 12 28 6 14 28 6 16 28 6 18 28 6 0 28 
		8 2 28 8 4 28 8 6 28 8 8 28 8 10 28 8 12 28 8 14 28 8 16 28 8 18 28 8 0 28 10 2 28 
		10 4 28 10 6 28 10 8 28 10 10 28 10 12 28 10 14 28 10 16 28 10 18 28 10 0 28 12 2 
		28 12 4 28 12 6 28 12 8 28 12 10 28 12 12 28 12 14 28 12 16 28 12 18 28 12 0 28 14 
		2 28 14 4 28 14 6 28 14 8 28 14 10 28 14 12 28 14 14 28 14 16 28 14 18 28 14 0 28 
		16 2 28 16 4 28 16 6 28 16 8 28 16 10 28 16 12 28 16 14 28 16 16 28 16 18 28 16 0 
		28 18 2 28 18 4 28 18 6 28 18 8 28 18 10 28 18;
	setAttr ".inpo[996:999]" 12 28 18 14 28 18 16 28 18 18 28 18;
createNode transform -n "dRigidBody1";
	setAttr ".r" -type "double3" 0 0 -20.866318216103668 ;
createNode dRigidBody -n "dRigidBodyShape1" -p "dRigidBody1";
	setAttr -k off ".v";
	setAttr ".ac" no;
	setAttr ".inro" -type "float3" 0 0 -20.866302 ;
createNode transform -n "dRigidBody2";
createNode dRigidBody -n "dRigidBodyShape2" -p "dRigidBody2";
	setAttr -k off ".v";
	setAttr ".ac" no;
	setAttr ".inpo" -type "float3" -53.036907 90.318428 0 ;
createNode lightLinker -n "lightLinker1";
	setAttr -s 2 ".lnk";
	setAttr -s 2 ".slnk";
createNode displayLayerManager -n "layerManager";
createNode displayLayer -n "defaultLayer";
createNode renderLayerManager -n "renderLayerManager";
createNode renderLayer -n "defaultRenderLayer";
	setAttr ".g" yes;
createNode dSolver -n "dSolver1";
	setAttr -k off ".grvt" -type "float3" 1.4012985e-45 -9.8100004 0 ;
	setAttr -k off ".grvt";
createNode polyTorus -n "polyTorus1";
	setAttr ".r" 0.80618598613471848;
	setAttr ".sr" 0.19999999999999996;
	setAttr ".sa" 6;
	setAttr ".sh" 6;
createNode dCollisionShape -n "dCollisionShape1";
	setAttr ".tp" 0;
createNode dCollisionShape -n "dCollisionShape2";
	setAttr ".tp" 6;
createNode script -n "sceneConfigurationScriptNode";
	setAttr ".b" -type "string" "playbackOptions -min 1 -max 1000 -ast 1 -aet 1000 ";
	setAttr ".st" 6;
createNode dCollisionShape -n "dCollisionShape3";
	setAttr ".tp" 4;
	setAttr ".sc" -type "float3" 1 1 40 ;
createNode animCurveTL -n "dRigidBody2_translateX";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 -22.268104292112419 69 -2.5917654377150257 101 13.767409521567387;
createNode animCurveTL -n "dRigidBody2_translateY";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 11.787800135659548 69 3.4651682088185822 101 -3.5233609163150508;
createNode animCurveTL -n "dRigidBody2_translateZ";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 0 69 0 101 0;
createNode animCurveTU -n "dRigidBody2_visibility";
	setAttr ".tan" 9;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 1 69 1 101 1;
	setAttr -s 3 ".kot[0:2]"  5 5 5;
createNode animCurveTA -n "dRigidBody2_rotateX";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 0 69 0 101 0;
createNode animCurveTA -n "dRigidBody2_rotateY";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 0 69 0 101 0;
createNode animCurveTA -n "dRigidBody2_rotateZ";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 0 69 0 101 0;
createNode animCurveTU -n "dRigidBody2_scaleX";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 1 69 1 101 1;
createNode animCurveTU -n "dRigidBody2_scaleY";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 1 69 1 101 1;
createNode animCurveTU -n "dRigidBody2_scaleZ";
	setAttr ".tan" 10;
	setAttr ".wgt" no;
	setAttr -s 3 ".ktv[0:2]"  1 1 69 1 101 1;
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
connectAttr "polyTorus1.out" "pTorusShape1.i";
connectAttr "dSolver1.rbds" "dRigidBodyArrayShape1.solv";
connectAttr "dCollisionShape1.oucs" "dRigidBodyArrayShape1.incs";
connectAttr "dSolver1.rbds" "dRigidBodyShape1.solv";
connectAttr "dCollisionShape2.oucs" "dRigidBodyShape1.incs";
connectAttr "dRigidBody2_translateX.o" "dRigidBody2.tx";
connectAttr "dRigidBody2_translateY.o" "dRigidBody2.ty";
connectAttr "dRigidBody2_translateZ.o" "dRigidBody2.tz";
connectAttr "dRigidBody2_visibility.o" "dRigidBody2.v";
connectAttr "dRigidBody2_rotateX.o" "dRigidBody2.rx";
connectAttr "dRigidBody2_rotateY.o" "dRigidBody2.ry";
connectAttr "dRigidBody2_rotateZ.o" "dRigidBody2.rz";
connectAttr "dRigidBody2_scaleX.o" "dRigidBody2.sx";
connectAttr "dRigidBody2_scaleY.o" "dRigidBody2.sy";
connectAttr "dRigidBody2_scaleZ.o" "dRigidBody2.sz";
connectAttr "dSolver1.rbds" "dRigidBodyShape2.solv";
connectAttr "dCollisionShape3.oucs" "dRigidBodyShape2.incs";
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
connectAttr ":time1.o" "dSolver1.it";
connectAttr "pTorusShape1.msg" "dCollisionShape1.insh";
connectAttr "lightLinker1.msg" ":lightList1.ln" -na;
connectAttr "pTorusShape1.iog" ":initialShadingGroup.dsm" -na;
// End of donuts.ma
