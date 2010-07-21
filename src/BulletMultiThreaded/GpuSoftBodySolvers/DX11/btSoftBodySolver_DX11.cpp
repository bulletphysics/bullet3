/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "vectormath/vmInclude.h"

#include "btSoftBodySolver_DX11.h"
#include "btSoftBodySolverVertexBuffer_DX11.h"
#include "BulletSoftBody/btSoftBody.h"

#define MSTRINGIFY(A) #A
static char* PrepareLinksHLSLString = 
#include "HLSL/PrepareLinks.hlsl"
static char* UpdatePositionsFromVelocitiesHLSLString = 
#include "HLSL/UpdatePositionsFromVelocities.hlsl"
static char* SolvePositionsHLSLString = 
#include "HLSL/SolvePositions.hlsl"
static char* UpdateNodesHLSLString = 
#include "HLSL/UpdateNodes.hlsl"
static char* UpdatePositionsHLSLString = 
#include "HLSL/UpdatePositions.hlsl"
static char* UpdateConstantsHLSLString = 
#include "HLSL/UpdateConstants.hlsl"
static char* IntegrateHLSLString = 
#include "HLSL/Integrate.hlsl"
static char* ApplyForcesHLSLString = 
#include "HLSL/ApplyForces.hlsl"
static char* UpdateNormalsHLSLString = 
#include "HLSL/UpdateNormals.hlsl"
static char* OutputToVertexArrayHLSLString = 
#include "HLSL/OutputToVertexArray.hlsl"
static char* VSolveLinksHLSLString = 
#include "HLSL/VSolveLinks.hlsl"


btSoftBodyLinkDataDX11::btSoftBodyLinkDataDX11( ID3D11Device *d3dDevice, ID3D11DeviceContext *d3dDeviceContext ) : 
		m_dx11Links( d3dDevice, d3dDeviceContext, &m_links, false ),
		m_dx11LinkStrength( d3dDevice, d3dDeviceContext, &m_linkStrength, false ),
		m_dx11LinksMassLSC( d3dDevice, d3dDeviceContext, &m_linksMassLSC, false ),
		m_dx11LinksRestLengthSquared( d3dDevice, d3dDeviceContext, &m_linksRestLengthSquared, false ),
		m_dx11LinksCLength( d3dDevice, d3dDeviceContext, &m_linksCLength, false ),
		m_dx11LinksLengthRatio( d3dDevice, d3dDeviceContext, &m_linksLengthRatio, false ),
		m_dx11LinksRestLength( d3dDevice, d3dDeviceContext, &m_linksRestLength, false ),
		m_dx11LinksMaterialLinearStiffnessCoefficient( d3dDevice, d3dDeviceContext, &m_linksMaterialLinearStiffnessCoefficient, false )
{
	m_d3dDevice = d3dDevice;
	m_d3dDeviceContext = d3dDeviceContext;
}

btSoftBodyLinkDataDX11::~btSoftBodyLinkDataDX11()
{
}

static Vectormath::Aos::Vector3 toVector3( const btVector3 &vec )
{
	Vectormath::Aos::Vector3 outVec( vec.getX(), vec.getY(), vec.getZ() );
	return outVec;
}

void btSoftBodyLinkDataDX11::createLinks( int numLinks )
{
	int previousSize = m_links.size();
	int newSize = previousSize + numLinks;

	btSoftBodyLinkData::createLinks( numLinks );

	// Resize the link addresses array as well
	m_linkAddresses.resize( newSize );
}

void btSoftBodyLinkDataDX11::setLinkAt( const btSoftBodyLinkData::LinkDescription &link, int linkIndex )
{
	btSoftBodyLinkData::setLinkAt( link, linkIndex );

	// Set the link index correctly for initialisation
	m_linkAddresses[linkIndex] = linkIndex;
}

bool btSoftBodyLinkDataDX11::onAccelerator()
{
	return m_onGPU;
}

bool btSoftBodyLinkDataDX11::moveToAccelerator()
{
	bool success = true;
	success = success && m_dx11Links.moveToGPU();
	success = success && m_dx11LinkStrength.moveToGPU();
	success = success && m_dx11LinksMassLSC.moveToGPU();
	success = success && m_dx11LinksRestLengthSquared.moveToGPU();
	success = success && m_dx11LinksCLength.moveToGPU();
	success = success && m_dx11LinksLengthRatio.moveToGPU();
	success = success && m_dx11LinksRestLength.moveToGPU();
	success = success && m_dx11LinksMaterialLinearStiffnessCoefficient.moveToGPU();

	if( success )
		m_onGPU = true;

	return success;
}

bool btSoftBodyLinkDataDX11::moveFromAccelerator()
{
	bool success = true;
	success = success && m_dx11Links.moveFromGPU();
	success = success && m_dx11LinkStrength.moveFromGPU();
	success = success && m_dx11LinksMassLSC.moveFromGPU();
	success = success && m_dx11LinksRestLengthSquared.moveFromGPU();
	success = success && m_dx11LinksCLength.moveFromGPU();
	success = success && m_dx11LinksLengthRatio.moveFromGPU();
	success = success && m_dx11LinksRestLength.moveFromGPU();
	success = success && m_dx11LinksMaterialLinearStiffnessCoefficient.moveFromGPU();

	if( success )
		m_onGPU = false;

	return success;
}

void btSoftBodyLinkDataDX11::generateBatches()
{
	int numLinks = getNumLinks();

	// Do the graph colouring here temporarily
	btAlignedObjectArray< int > batchValues;
	batchValues.resize( numLinks, 0 );

	// Find the maximum vertex value internally for now
	int maxVertex = 0;
	for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
	{
		int vertex0 = getVertexPair(linkIndex).vertex0;
		int vertex1 = getVertexPair(linkIndex).vertex1;
		if( vertex0 > maxVertex )
			maxVertex = vertex0;
		if( vertex1 > maxVertex )
			maxVertex = vertex1;
	}
	int numVertices = maxVertex + 1;

	// Set of lists, one for each node, specifying which colours are connected
	// to that node.
	// No two edges into a node can share a colour.
	btAlignedObjectArray< btAlignedObjectArray< int > > vertexConnectedColourLists;
	vertexConnectedColourLists.resize(numVertices);



	// Simple algorithm that chooses the lowest batch number
	// that none of the links attached to either of the connected 
	// nodes is in
	for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
	{				
		int linkLocation = m_linkAddresses[linkIndex];

		int vertex0 = getVertexPair(linkLocation).vertex0;
		int vertex1 = getVertexPair(linkLocation).vertex1;

		// Get the two node colour lists
		btAlignedObjectArray< int > &colourListVertex0( vertexConnectedColourLists[vertex0] );
		btAlignedObjectArray< int > &colourListVertex1( vertexConnectedColourLists[vertex1] );

		// Choose the minimum colour that is in neither list
		int colour = 0;
		while( colourListVertex0.findLinearSearch(colour) != colourListVertex0.size() || colourListVertex1.findLinearSearch(colour) != colourListVertex1.size()  )
			++colour;
		// i should now be the minimum colour in neither list
		// Add to the two lists so that future edges don't share
		// And store the colour against this edge

		colourListVertex0.push_back(colour);
		colourListVertex1.push_back(colour);
		batchValues[linkIndex] = colour;
	}

	// Check the colour counts
	btAlignedObjectArray< int > batchCounts;
	for( int i = 0; i < numLinks; ++i )
	{
		int batch = batchValues[i];
		if( batch >= batchCounts.size() )
			batchCounts.push_back(1);
		else
			++(batchCounts[batch]);
	}

	m_batchStartLengths.resize(batchCounts.size());
	if( m_batchStartLengths.size() > 0 )
	{
		m_batchStartLengths[0] = BatchPair( 0, 0 );

		int sum = 0;
		for( int batchIndex = 0; batchIndex < batchCounts.size(); ++batchIndex )
		{
			m_batchStartLengths[batchIndex].start = sum;
			m_batchStartLengths[batchIndex].length = batchCounts[batchIndex];
			sum += batchCounts[batchIndex];
		}
	}

	/////////////////////////////
	// Sort data based on batches

	// Create source arrays by copying originals
	btAlignedObjectArray<btSoftBodyLinkData::LinkNodePair>				m_links_Backup(m_links);
	btAlignedObjectArray<float>											m_linkStrength_Backup(m_linkStrength);
	btAlignedObjectArray<float>											m_linksMassLSC_Backup(m_linksMassLSC);
	btAlignedObjectArray<float>											m_linksRestLengthSquared_Backup(m_linksRestLengthSquared);
	btAlignedObjectArray<Vectormath::Aos::Vector3>						m_linksCLength_Backup(m_linksCLength);
	btAlignedObjectArray<float>											m_linksLengthRatio_Backup(m_linksLengthRatio);
	btAlignedObjectArray<float>											m_linksRestLength_Backup(m_linksRestLength);
	btAlignedObjectArray<float>											m_linksMaterialLinearStiffnessCoefficient_Backup(m_linksMaterialLinearStiffnessCoefficient);


	for( int batch = 0; batch < batchCounts.size(); ++batch )
		batchCounts[batch] = 0;

	// Do sort as single pass into destination arrays	
	for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
	{
		// To maintain locations run off the original link locations rather than the current position.
		// It's not cache efficient, but as we run this rarely that should not matter.
		// It's faster than searching the link location array for the current location and then updating it.
		// The other alternative would be to unsort before resorting, but this is equivalent to doing that.
		int linkLocation = m_linkAddresses[linkIndex];

		// Obtain batch and calculate target location for the
		// next element in that batch, incrementing the batch counter
		// afterwards
		int batch = batchValues[linkIndex];
		int newLocation = m_batchStartLengths[batch].start + batchCounts[batch];

		batchCounts[batch] = batchCounts[batch] + 1;
		m_links[newLocation] = m_links_Backup[linkLocation];
#if 1
		m_linkStrength[newLocation] = m_linkStrength_Backup[linkLocation];
		m_linksMassLSC[newLocation] = m_linksMassLSC_Backup[linkLocation];
		m_linksRestLengthSquared[newLocation] = m_linksRestLengthSquared_Backup[linkLocation];
		m_linksLengthRatio[newLocation] = m_linksLengthRatio_Backup[linkLocation];
		m_linksRestLength[newLocation] = m_linksRestLength_Backup[linkLocation];
		m_linksMaterialLinearStiffnessCoefficient[newLocation] = m_linksMaterialLinearStiffnessCoefficient_Backup[linkLocation];
#endif
		// Update the locations array to account for the moved entry
		m_linkAddresses[linkIndex] = newLocation;
	}
} // void btSoftBodyLinkDataDX11::generateBatches()



btSoftBodyVertexDataDX11::btSoftBodyVertexDataDX11( ID3D11Device *d3dDevice, ID3D11DeviceContext *d3dDeviceContext ) : 
	m_dx11ClothIdentifier( d3dDevice, d3dDeviceContext, &m_clothIdentifier, false ),
	m_dx11VertexPosition( d3dDevice, d3dDeviceContext, &m_vertexPosition, false ),
	m_dx11VertexPreviousPosition( d3dDevice, d3dDeviceContext, &m_vertexPreviousPosition, false ),
	m_dx11VertexVelocity( d3dDevice, d3dDeviceContext, &m_vertexVelocity, false ),
	m_dx11VertexForceAccumulator( d3dDevice, d3dDeviceContext, &m_vertexForceAccumulator, false ),
	m_dx11VertexNormal( d3dDevice, d3dDeviceContext, &m_vertexNormal, false ),
	m_dx11VertexInverseMass( d3dDevice, d3dDeviceContext, &m_vertexInverseMass, false ),
	m_dx11VertexArea( d3dDevice, d3dDeviceContext, &m_vertexArea, false ),
	m_dx11VertexTriangleCount( d3dDevice, d3dDeviceContext, &m_vertexTriangleCount, false )
{
	m_d3dDevice = d3dDevice;
	m_d3dDeviceContext = d3dDeviceContext;
}

btSoftBodyVertexDataDX11::~btSoftBodyVertexDataDX11()
{

}

bool btSoftBodyVertexDataDX11::onAccelerator()
{
	return m_onGPU;
}

bool btSoftBodyVertexDataDX11::moveToAccelerator()
{
	bool success = true;
	success = success && m_dx11ClothIdentifier.moveToGPU();
	success = success && m_dx11VertexPosition.moveToGPU();
	success = success && m_dx11VertexPreviousPosition.moveToGPU();
	success = success && m_dx11VertexVelocity.moveToGPU();
	success = success && m_dx11VertexForceAccumulator.moveToGPU();
	success = success && m_dx11VertexNormal.moveToGPU();
	success = success && m_dx11VertexInverseMass.moveToGPU();
	success = success && m_dx11VertexArea.moveToGPU();
	success = success && m_dx11VertexTriangleCount.moveToGPU();

	if( success )
		m_onGPU = true;

	return success;
}

bool btSoftBodyVertexDataDX11::moveFromAccelerator()
{
	bool success = true;
	success = success && m_dx11ClothIdentifier.moveFromGPU();
	success = success && m_dx11VertexPosition.moveFromGPU();
	success = success && m_dx11VertexPreviousPosition.moveFromGPU();
	success = success && m_dx11VertexVelocity.moveFromGPU();
	success = success && m_dx11VertexForceAccumulator.moveFromGPU();
	success = success && m_dx11VertexNormal.moveFromGPU();
	success = success && m_dx11VertexInverseMass.moveFromGPU();
	success = success && m_dx11VertexArea.moveFromGPU();
	success = success && m_dx11VertexTriangleCount.moveFromGPU();

	if( success )
		m_onGPU = true;

	return success;
}


btSoftBodyTriangleDataDX11::btSoftBodyTriangleDataDX11( ID3D11Device *d3dDevice, ID3D11DeviceContext *d3dDeviceContext ) : 
	m_dx11VertexIndices( d3dDevice, d3dDeviceContext, &m_vertexIndices, false ),
	m_dx11Area( d3dDevice, d3dDeviceContext, &m_area, false ),
	m_dx11Normal( d3dDevice, d3dDeviceContext, &m_normal, false )
{
	m_d3dDevice = d3dDevice;
	m_d3dDeviceContext = d3dDeviceContext;
}

btSoftBodyTriangleDataDX11::~btSoftBodyTriangleDataDX11()
{

}


/** Allocate enough space in all link-related arrays to fit numLinks links */
void btSoftBodyTriangleDataDX11::createTriangles( int numTriangles )
{
	int previousSize = getNumTriangles();
	int newSize = previousSize + numTriangles;

	btSoftBodyTriangleData::createTriangles( numTriangles );

	// Resize the link addresses array as well
	m_triangleAddresses.resize( newSize );
}

/** Insert the link described into the correct data structures assuming space has already been allocated by a call to createLinks */
void btSoftBodyTriangleDataDX11::setTriangleAt( const btSoftBodyTriangleData::TriangleDescription &triangle, int triangleIndex )
{
	btSoftBodyTriangleData::setTriangleAt( triangle, triangleIndex );

	m_triangleAddresses[triangleIndex] = triangleIndex;
}

bool btSoftBodyTriangleDataDX11::onAccelerator()
{
	return m_onGPU;
}

bool btSoftBodyTriangleDataDX11::moveToAccelerator()
{
	bool success = true;
	success = success && m_dx11VertexIndices.moveToGPU();
	success = success && m_dx11Area.moveToGPU();
	success = success && m_dx11Normal.moveToGPU();

	if( success )
		m_onGPU = true;

	return success;
}

bool btSoftBodyTriangleDataDX11::moveFromAccelerator()
{
	bool success = true;
	success = success && m_dx11VertexIndices.moveFromGPU();
	success = success && m_dx11Area.moveFromGPU();
	success = success && m_dx11Normal.moveFromGPU();

	if( success )
		m_onGPU = true;

	return success;
}

/**
 * Generate (and later update) the batching for the entire triangle set.
 * This redoes a lot of work because it batches the entire set when each cloth is inserted.
 * In theory we could delay it until just before we need the cloth.
 * It's a one-off overhead, though, so that is a later optimisation.
 */
void btSoftBodyTriangleDataDX11::generateBatches()
{
	int numTriangles = getNumTriangles();
	if( numTriangles == 0 )
		return;

	// Do the graph colouring here temporarily
	btAlignedObjectArray< int > batchValues;
	batchValues.resize( numTriangles );

	// Find the maximum vertex value internally for now
	int maxVertex = 0;
	for( int triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
	{
		int vertex0 = getVertexSet(triangleIndex).vertex0;
		int vertex1 = getVertexSet(triangleIndex).vertex1;
		int vertex2 = getVertexSet(triangleIndex).vertex2;
		
		if( vertex0 > maxVertex )
			maxVertex = vertex0;
		if( vertex1 > maxVertex )
			maxVertex = vertex1;
		if( vertex2 > maxVertex )
			maxVertex = vertex2;
	}
	int numVertices = maxVertex + 1;

	// Set of lists, one for each node, specifying which colours are connected
	// to that node.
	// No two edges into a node can share a colour.
	btAlignedObjectArray< btAlignedObjectArray< int > > vertexConnectedColourLists;
	vertexConnectedColourLists.resize(numVertices);


	//std::cout << "\n";
	// Simple algorithm that chooses the lowest batch number
	// that none of the faces attached to either of the connected 
	// nodes is in
	for( int triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
	{
		// To maintain locations run off the original link locations rather than the current position.
		// It's not cache efficient, but as we run this rarely that should not matter.
		// It's faster than searching the link location array for the current location and then updating it.
		// The other alternative would be to unsort before resorting, but this is equivalent to doing that.
		int triangleLocation = m_triangleAddresses[triangleIndex];

		int vertex0 = getVertexSet(triangleLocation).vertex0;
		int vertex1 = getVertexSet(triangleLocation).vertex1;
		int vertex2 = getVertexSet(triangleLocation).vertex2;

		// Get the three node colour lists
		btAlignedObjectArray< int > &colourListVertex0( vertexConnectedColourLists[vertex0] );
		btAlignedObjectArray< int > &colourListVertex1( vertexConnectedColourLists[vertex1] );
		btAlignedObjectArray< int > &colourListVertex2( vertexConnectedColourLists[vertex2] );

		// Choose the minimum colour that is in none of the lists
		int colour = 0;
		while( 
			colourListVertex0.findLinearSearch(colour) != colourListVertex0.size() || 
			colourListVertex1.findLinearSearch(colour) != colourListVertex1.size() ||
			colourListVertex2.findLinearSearch(colour) != colourListVertex2.size() )
		{
			++colour;
		}
		// i should now be the minimum colour in neither list
		// Add to the three lists so that future edges don't share
		// And store the colour against this face
		colourListVertex0.push_back(colour);
		colourListVertex1.push_back(colour);
		colourListVertex2.push_back(colour);

		batchValues[triangleIndex] = colour;
	}


	// Check the colour counts
	btAlignedObjectArray< int > batchCounts;
	for( int i = 0; i < numTriangles; ++i )
	{
		int batch = batchValues[i];
		if( batch >= batchCounts.size() )
			batchCounts.push_back(1);
		else
			++(batchCounts[batch]);
	}


	m_batchStartLengths.resize(batchCounts.size());
	m_batchStartLengths[0] = BatchPair( 0, 0 );


	int sum = 0;
	for( int batchIndex = 0; batchIndex < batchCounts.size(); ++batchIndex )
	{
		m_batchStartLengths[batchIndex].start = sum;
		m_batchStartLengths[batchIndex].length = batchCounts[batchIndex];
		sum += batchCounts[batchIndex];
	}
	
	/////////////////////////////
	// Sort data based on batches
	
	// Create source arrays by copying originals
	btAlignedObjectArray<btSoftBodyTriangleData::TriangleNodeSet>							m_vertexIndices_Backup(m_vertexIndices);
	btAlignedObjectArray<float>										m_area_Backup(m_area);
	btAlignedObjectArray<Vectormath::Aos::Vector3>					m_normal_Backup(m_normal);


	for( int batch = 0; batch < batchCounts.size(); ++batch )
		batchCounts[batch] = 0;

	// Do sort as single pass into destination arrays	
	for( int triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
	{
		// To maintain locations run off the original link locations rather than the current position.
		// It's not cache efficient, but as we run this rarely that should not matter.
		// It's faster than searching the link location array for the current location and then updating it.
		// The other alternative would be to unsort before resorting, but this is equivalent to doing that.
		int triangleLocation = m_triangleAddresses[triangleIndex];

		// Obtain batch and calculate target location for the
		// next element in that batch, incrementing the batch counter
		// afterwards
		int batch = batchValues[triangleIndex];
		int newLocation = m_batchStartLengths[batch].start + batchCounts[batch];

		batchCounts[batch] = batchCounts[batch] + 1;
		m_vertexIndices[newLocation] = m_vertexIndices_Backup[triangleLocation];
		m_area[newLocation] = m_area_Backup[triangleLocation];
		m_normal[newLocation] = m_normal_Backup[triangleLocation];

		// Update the locations array to account for the moved entry
		m_triangleAddresses[triangleIndex] = newLocation;
	}
} // btSoftBodyTriangleDataDX11::generateBatches












btDX11SoftBodySolver::btDX11SoftBodySolver(ID3D11Device * dx11Device, ID3D11DeviceContext* dx11Context) :
	m_dx11Device( dx11Device ),
	m_dx11Context( dx11Context ),
	m_linkData(m_dx11Device, m_dx11Context),
	m_vertexData(m_dx11Device, m_dx11Context),
	m_triangleData(m_dx11Device, m_dx11Context),
	m_dx11PerClothAcceleration( m_dx11Device, m_dx11Context, &m_perClothAcceleration, true ),
	m_dx11PerClothWindVelocity( m_dx11Device, m_dx11Context, &m_perClothWindVelocity, true ),
	m_dx11PerClothDampingFactor( m_dx11Device, m_dx11Context, &m_perClothDampingFactor, true ),
	m_dx11PerClothVelocityCorrectionCoefficient( m_dx11Device, m_dx11Context, &m_perClothVelocityCorrectionCoefficient, true ),
	m_dx11PerClothLiftFactor( m_dx11Device, m_dx11Context, &m_perClothLiftFactor, true ),
	m_dx11PerClothDragFactor( m_dx11Device, m_dx11Context, &m_perClothDragFactor, true ),
	m_dx11PerClothMediumDensity( m_dx11Device, m_dx11Context, &m_perClothMediumDensity, true )
{
	// Initial we will clearly need to update solver constants
	// For now this is global for the cloths linked with this solver - we should probably make this body specific 
	// for performance in future once we understand more clearly when constants need to be updated
	m_updateSolverConstants = true;

	m_shadersInitialized = false;
}

btDX11SoftBodySolver::~btDX11SoftBodySolver()
{
	SAFE_RELEASE( integrateKernel.constBuffer );
	SAFE_RELEASE( integrateKernel.kernel );
	SAFE_RELEASE( prepareLinksKernel.constBuffer );
	SAFE_RELEASE( prepareLinksKernel.kernel );
	SAFE_RELEASE( solvePositionsFromLinksKernel.constBuffer );
	SAFE_RELEASE( solvePositionsFromLinksKernel.kernel );
	SAFE_RELEASE( vSolveLinksKernel.constBuffer );
	SAFE_RELEASE( vSolveLinksKernel.kernel );
	SAFE_RELEASE( updatePositionsFromVelocitiesKernel.constBuffer );
	SAFE_RELEASE( updatePositionsFromVelocitiesKernel.kernel );
	SAFE_RELEASE( updateVelocitiesFromPositionsWithoutVelocitiesKernel.constBuffer );
	SAFE_RELEASE( updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel );
	SAFE_RELEASE( updateVelocitiesFromPositionsWithVelocitiesKernel.constBuffer );
	SAFE_RELEASE( updateVelocitiesFromPositionsWithVelocitiesKernel.kernel );
	SAFE_RELEASE( resetNormalsAndAreasKernel.constBuffer );
	SAFE_RELEASE( resetNormalsAndAreasKernel.kernel );
	SAFE_RELEASE( normalizeNormalsAndAreasKernel.constBuffer );
	SAFE_RELEASE( normalizeNormalsAndAreasKernel.kernel );
	SAFE_RELEASE( updateSoftBodiesKernel.constBuffer );
	SAFE_RELEASE( updateSoftBodiesKernel.kernel );
	SAFE_RELEASE( outputToVertexArrayWithNormalsKernel.constBuffer );
	SAFE_RELEASE( outputToVertexArrayWithNormalsKernel.kernel );
	SAFE_RELEASE( outputToVertexArrayWithoutNormalsKernel.constBuffer );
	SAFE_RELEASE( outputToVertexArrayWithoutNormalsKernel.kernel );


	SAFE_RELEASE( addVelocityKernel.constBuffer );
	SAFE_RELEASE( addVelocityKernel.kernel );
	SAFE_RELEASE( applyForcesKernel.constBuffer );
	SAFE_RELEASE( applyForcesKernel.kernel );
	SAFE_RELEASE( outputToVertexArrayKernel.constBuffer );
	SAFE_RELEASE( outputToVertexArrayKernel.kernel );
	SAFE_RELEASE( collideCylinderKernel.constBuffer );
	SAFE_RELEASE( collideCylinderKernel.kernel );	
}



void btDX11SoftBodySolver::optimize( btAlignedObjectArray< btSoftBody * > &softBodies )
{
	if( m_softBodySet.size() != softBodies.size() )
	{
		// Have a change in the soft body set so update, reloading all the data
		getVertexData().clear();
		getTriangleData().clear();
		getLinkData().clear();
		m_softBodySet.resize(0);


		for( int softBodyIndex = 0; softBodyIndex < softBodies.size(); ++softBodyIndex )
		{
			btSoftBody *softBody = softBodies[ softBodyIndex ];
			using Vectormath::Aos::Matrix3;
			using Vectormath::Aos::Point3;

			// Create SoftBody that will store the information within the solver
			btAcceleratedSoftBodyInterface *newSoftBody = new btAcceleratedSoftBodyInterface( softBody );
			m_softBodySet.push_back( newSoftBody );

			m_perClothAcceleration.push_back( toVector3(softBody->getWorldInfo()->m_gravity) );
			m_perClothDampingFactor.push_back(softBody->m_cfg.kDP);
			m_perClothVelocityCorrectionCoefficient.push_back( softBody->m_cfg.kVCF );
			m_perClothLiftFactor.push_back( softBody->m_cfg.kLF );
			m_perClothDragFactor.push_back( softBody->m_cfg.kDG );
			m_perClothMediumDensity.push_back(softBody->getWorldInfo()->air_density);

			// Add space for new vertices and triangles in the default solver for now
			// TODO: Include space here for tearing too later
			int firstVertex = getVertexData().getNumVertices();
			int numVertices = softBody->m_nodes.size();
			int maxVertices = numVertices;
			// Allocate space for new vertices in all the vertex arrays
			getVertexData().createVertices( maxVertices, softBodyIndex );

			int firstTriangle = getTriangleData().getNumTriangles();
			int numTriangles = softBody->m_faces.size();
			int maxTriangles = numTriangles;
			getTriangleData().createTriangles( maxTriangles );

			// Copy vertices from softbody into the solver
			for( int vertex = 0; vertex < numVertices; ++vertex )
			{
				Point3 multPoint(softBody->m_nodes[vertex].m_x.getX(), softBody->m_nodes[vertex].m_x.getY(), softBody->m_nodes[vertex].m_x.getZ());
				btSoftBodyVertexData::VertexDescription desc;

				// TODO: Position in the softbody might be pre-transformed
				// or we may need to adapt for the pose.
				//desc.setPosition( cloth.getMeshTransform()*multPoint );
				desc.setPosition( multPoint );

				float vertexInverseMass = softBody->m_nodes[vertex].m_im;
				desc.setInverseMass(vertexInverseMass);
				getVertexData().setVertexAt( desc, firstVertex + vertex );
			}

			// Copy triangles similarly
			// We're assuming here that vertex indices are based on the firstVertex rather than the entire scene
			for( int triangle = 0; triangle < numTriangles; ++triangle )
			{
				// Note that large array storage is relative to the array not to the cloth
				// So we need to add firstVertex to each value
				int vertexIndex0 = (softBody->m_faces[triangle].m_n[0] - &(softBody->m_nodes[0]));
				int vertexIndex1 = (softBody->m_faces[triangle].m_n[1] - &(softBody->m_nodes[0]));
				int vertexIndex2 = (softBody->m_faces[triangle].m_n[2] - &(softBody->m_nodes[0]));
				btSoftBodyTriangleData::TriangleDescription newTriangle(vertexIndex0 + firstVertex, vertexIndex1 + firstVertex, vertexIndex2 + firstVertex);
				getTriangleData().setTriangleAt( newTriangle, firstTriangle + triangle );
				
				// Increase vertex triangle counts for this triangle		
				getVertexData().getTriangleCount(newTriangle.getVertexSet().vertex0)++;
				getVertexData().getTriangleCount(newTriangle.getVertexSet().vertex1)++;
				getVertexData().getTriangleCount(newTriangle.getVertexSet().vertex2)++;
			}

			int firstLink = getLinkData().getNumLinks();
			int numLinks = softBody->m_links.size();
			int maxLinks = numLinks;
			
			// Allocate space for the links
			getLinkData().createLinks( numLinks );

			// Add the links
			for( int link = 0; link < numLinks; ++link )
			{
				int vertexIndex0 = softBody->m_links[link].m_n[0] - &(softBody->m_nodes[0]);
				int vertexIndex1 = softBody->m_links[link].m_n[1] - &(softBody->m_nodes[0]);

				btSoftBodyLinkData::LinkDescription newLink(vertexIndex0 + firstVertex, vertexIndex1 + firstVertex, softBody->m_links[link].m_material->m_kLST);
				newLink.setLinkStrength(1.f);
				getLinkData().setLinkAt(newLink, firstLink + link);
			}
			
			newSoftBody->setFirstVertex( firstVertex );
			newSoftBody->setFirstTriangle( firstTriangle );
			newSoftBody->setNumVertices( numVertices );
			newSoftBody->setMaxVertices( maxVertices );
			newSoftBody->setNumTriangles( numTriangles );
			newSoftBody->setMaxTriangles( maxTriangles );
			newSoftBody->setFirstLink( firstLink );
			newSoftBody->setNumLinks( numLinks );
		}



		updateConstants(0.f);


		m_linkData.generateBatches();		
		m_triangleData.generateBatches();
	}
}


btSoftBodyLinkData &btDX11SoftBodySolver::getLinkData()
{
	// TODO: Consider setting link data to "changed" here
	return m_linkData;
}

btSoftBodyVertexData &btDX11SoftBodySolver::getVertexData()
{
	// TODO: Consider setting vertex data to "changed" here
	return m_vertexData;
}

btSoftBodyTriangleData &btDX11SoftBodySolver::getTriangleData()
{
	// TODO: Consider setting triangle data to "changed" here
	return m_triangleData;
}

bool btDX11SoftBodySolver::checkInitialized()
{
	return buildShaders();
}

void btDX11SoftBodySolver::resetNormalsAndAreas( int numVertices )
{
	// No need to batch link solver, it is entirely parallel
	// Copy kernel parameters to GPU
	UpdateSoftBodiesCB constBuffer;
	
	constBuffer.numNodes = numVertices;
	constBuffer.epsilon = FLT_EPSILON;
	
	// Todo: factor this out. Number of nodes is static and sdt might be, too, we can update this just once on setup
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( integrateKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(UpdateSoftBodiesCB) );	
	m_dx11Context->Unmap( integrateKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &integrateKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexNormal.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexArea.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( resetNormalsAndAreasKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks, 1, 1 );

	{
		// Tidy up 
		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SoftBodySolver::resetNormalsAndAreas

void btDX11SoftBodySolver::normalizeNormalsAndAreas( int numVertices )
{
	// No need to batch link solver, it is entirely parallel
	// Copy kernel parameters to GPU
	UpdateSoftBodiesCB constBuffer;
	
	constBuffer.numNodes = numVertices;
	constBuffer.epsilon = FLT_EPSILON;
	
	// Todo: factor this out. Number of nodes is static and sdt might be, too, we can update this just once on setup
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( integrateKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(UpdateSoftBodiesCB) );	
	m_dx11Context->Unmap( integrateKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &integrateKernel.constBuffer );

	// Set resources and dispatch	
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_vertexData.m_dx11VertexTriangleCount.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexNormal.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexArea.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( normalizeNormalsAndAreasKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks, 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SoftBodySolver::normalizeNormalsAndAreas

void btDX11SoftBodySolver::executeUpdateSoftBodies( int firstTriangle, int numTriangles )
{
	// No need to batch link solver, it is entirely parallel
	// Copy kernel parameters to GPU
	UpdateSoftBodiesCB constBuffer;
	
	constBuffer.startFace = firstTriangle;
	constBuffer.numFaces = numTriangles;
	
	// Todo: factor this out. Number of nodes is static and sdt might be, too, we can update this just once on setup
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( updateSoftBodiesKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(UpdateSoftBodiesCB) );	
	m_dx11Context->Unmap( updateSoftBodiesKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &updateSoftBodiesKernel.constBuffer );

	// Set resources and dispatch	
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_triangleData.m_dx11VertexIndices.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_vertexData.m_dx11VertexPosition.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexNormal.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexArea.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 2, 1, &(m_triangleData.m_dx11Normal.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 3, 1, &(m_triangleData.m_dx11Area.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( updateSoftBodiesKernel.kernel, NULL, 0 );

	int	numBlocks = (numTriangles + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks, 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 4, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SoftBodySolver::executeUpdateSoftBodies

void btDX11SoftBodySolver::updateSoftBodies()
{
	using namespace Vectormath::Aos;


	int numVertices = m_vertexData.getNumVertices();
	int numTriangles = m_triangleData.getNumTriangles();

	// Ensure data is on accelerator
	m_vertexData.moveToAccelerator();
	m_triangleData.moveToAccelerator();

	resetNormalsAndAreas( numVertices );


	// Go through triangle batches so updates occur correctly
	for( int batchIndex = 0; batchIndex < m_triangleData.m_batchStartLengths.size(); ++batchIndex )
	{

		int startTriangle = m_triangleData.m_batchStartLengths[batchIndex].start;
		int numTriangles = m_triangleData.m_batchStartLengths[batchIndex].length;

		executeUpdateSoftBodies( startTriangle, numTriangles );
	}


	normalizeNormalsAndAreas( numVertices );

} // btDX11SoftBodySolver::updateSoftBodies


Vectormath::Aos::Vector3 btDX11SoftBodySolver::ProjectOnAxis( const Vectormath::Aos::Vector3 &v, const Vectormath::Aos::Vector3 &a )
{
	return a*Vectormath::Aos::dot(v, a);
}

void btDX11SoftBodySolver::ApplyClampedForce( float solverdt, const Vectormath::Aos::Vector3 &force, const Vectormath::Aos::Vector3 &vertexVelocity, float inverseMass, Vectormath::Aos::Vector3 &vertexForce )
{
	float dtInverseMass = solverdt*inverseMass;
	if( Vectormath::Aos::lengthSqr(force * dtInverseMass) > Vectormath::Aos::lengthSqr(vertexVelocity) )
	{
		vertexForce -= ProjectOnAxis( vertexVelocity, normalize( force ) )/dtInverseMass;
	} else {
		vertexForce += force;
	}
}

void btDX11SoftBodySolver::applyForces( float solverdt )
{		
	using namespace Vectormath::Aos;


	// Ensure data is on accelerator
	m_vertexData.moveToAccelerator();
	m_dx11PerClothAcceleration.moveToGPU();
	m_dx11PerClothLiftFactor.moveToGPU();
	m_dx11PerClothDragFactor.moveToGPU();
	m_dx11PerClothMediumDensity.moveToGPU();
	m_dx11PerClothWindVelocity.moveToGPU();

	// No need to batch link solver, it is entirely parallel
	// Copy kernel parameters to GPU
	ApplyForcesCB constBuffer;
	
	constBuffer.numNodes = m_vertexData.getNumVertices();
	constBuffer.solverdt = solverdt;
	constBuffer.epsilon = FLT_EPSILON;
	
	// Todo: factor this out. Number of nodes is static and sdt might be, too, we can update this just once on setup
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( integrateKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(ApplyForcesCB) );	
	m_dx11Context->Unmap( integrateKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &integrateKernel.constBuffer );

	// Set resources and dispatch	
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_vertexData.m_dx11ClothIdentifier.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_vertexData.m_dx11VertexNormal.getSRV()) );
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_vertexData.m_dx11VertexArea.getSRV()) );
	m_dx11Context->CSSetShaderResources( 3, 1, &(m_vertexData.m_dx11VertexInverseMass.getSRV()) );
	m_dx11Context->CSSetShaderResources( 4, 1, &(m_dx11PerClothLiftFactor.getSRV()) );
	m_dx11Context->CSSetShaderResources( 5, 1, &(m_dx11PerClothDragFactor.getSRV()) );
	m_dx11Context->CSSetShaderResources( 6, 1, &(m_dx11PerClothWindVelocity.getSRV()) );
	m_dx11Context->CSSetShaderResources( 7, 1, &(m_dx11PerClothAcceleration.getSRV()) );
	m_dx11Context->CSSetShaderResources( 8, 1, &(m_dx11PerClothMediumDensity.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexForceAccumulator.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexVelocity.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( applyForcesKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks, 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 3, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 4, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 5, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 6, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 7, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 8, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SoftBodySolver::applyForces

/**
 * Integrate motion on the solver.
 */
void btDX11SoftBodySolver::integrate( float solverdt )
{
	// TEMPORARY COPIES
	m_vertexData.moveToAccelerator();

	// No need to batch link solver, it is entirely parallel
	// Copy kernel parameters to GPU
	IntegrateCB constBuffer;
	
	constBuffer.numNodes = m_vertexData.getNumVertices();
	constBuffer.solverdt = solverdt;
	
	// Todo: factor this out. Number of nodes is static and sdt might be, too, we can update this just once on setup
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( integrateKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(IntegrateCB) );	
	m_dx11Context->Unmap( integrateKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &integrateKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_vertexData.m_dx11VertexInverseMass.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexPosition.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexVelocity.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 2, 1, &(m_vertexData.m_dx11VertexPreviousPosition.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 3, 1, &(m_vertexData.m_dx11VertexForceAccumulator.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( integrateKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks, 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 2, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 3, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SoftBodySolver::integrate

float btDX11SoftBodySolver::computeTriangleArea( 
	const Vectormath::Aos::Point3 &vertex0,
	const Vectormath::Aos::Point3 &vertex1,
	const Vectormath::Aos::Point3 &vertex2 )
{
	Vectormath::Aos::Vector3 a = vertex1 - vertex0;
	Vectormath::Aos::Vector3 b = vertex2 - vertex0;
	Vectormath::Aos::Vector3 crossProduct = cross(a, b);
	float area = length( crossProduct );
	return area;
} // btDX11SoftBodySolver::computeTriangleArea

void btDX11SoftBodySolver::updateConstants( float timeStep )
{
	using namespace Vectormath::Aos;

	if( m_updateSolverConstants )
	{
		m_updateSolverConstants = false;

		// Will have to redo this if we change the structure (tear, maybe) or various other possible changes

		// Initialise link constants
		const int numLinks = m_linkData.getNumLinks();
		for( int linkIndex = 0; linkIndex < numLinks; ++linkIndex )
		{
			btSoftBodyLinkData::LinkNodePair &vertices( m_linkData.getVertexPair(linkIndex) );
			m_linkData.getRestLength(linkIndex) = length((m_vertexData.getPosition( vertices.vertex0 ) - m_vertexData.getPosition( vertices.vertex1 )));
			float invMass0 = m_vertexData.getInverseMass(vertices.vertex0);
			float invMass1 = m_vertexData.getInverseMass(vertices.vertex1);
			float linearStiffness = m_linkData.getLinearStiffnessCoefficient(linkIndex);
			float massLSC = (invMass0 + invMass1)/linearStiffness;
			m_linkData.getMassLSC(linkIndex) = massLSC;
			float restLength = m_linkData.getRestLength(linkIndex);
			float restLengthSquared = restLength*restLength;
			m_linkData.getRestLengthSquared(linkIndex) = restLengthSquared;
		}
	}
} // btDX11SoftBodySolver::updateConstants



void btDX11SoftBodySolver::solveConstraints( float solverdt )
{

	//std::cerr << "'GPU' solve constraints\n";
	using Vectormath::Aos::Vector3;
	using Vectormath::Aos::Point3;
	using Vectormath::Aos::lengthSqr;
	using Vectormath::Aos::dot;

	// Prepare links
	int numLinks = m_linkData.getNumLinks();
	int numVertices = m_vertexData.getNumVertices();

	float kst = 1.f;
	float ti = 0.f;


	m_dx11PerClothDampingFactor.moveToGPU();
	m_dx11PerClothVelocityCorrectionCoefficient.moveToGPU();


	// Ensure data is on accelerator
	m_linkData.moveToAccelerator();
	m_vertexData.moveToAccelerator();


	prepareLinks();	

	for( int iteration = 0; iteration < m_numberOfVelocityIterations ; ++iteration )
	{
		for( int i = 0; i < m_linkData.m_batchStartLengths.size(); ++i )
		{
			int startLink = m_linkData.m_batchStartLengths[i].start;
			int numLinks = m_linkData.m_batchStartLengths[i].length;

			solveLinksForVelocity( startLink, numLinks, kst );
		}
	}

	// Compute new positions from velocity
	// Also update the previous position so that our position computation is now based on the new position from the velocity solution
	// rather than based directly on the original positions
	if( m_numberOfVelocityIterations > 0 )
	{
		updateVelocitiesFromPositionsWithVelocities( 1.f/solverdt );
	} else {
		updateVelocitiesFromPositionsWithoutVelocities( 1.f/solverdt );
	}


	// Solve drift
	for( int iteration = 0; iteration < m_numberOfPositionIterations ; ++iteration )
	{
		for( int i = 0; i < m_linkData.m_batchStartLengths.size(); ++i )
		{
			int startLink = m_linkData.m_batchStartLengths[i].start;
			int numLinks = m_linkData.m_batchStartLengths[i].length;

			solveLinksForPosition( startLink, numLinks, kst, ti );
		}
		
	} // for( int iteration = 0; iteration < m_numberOfPositionIterations ; ++iteration )

	updateVelocitiesFromPositionsWithoutVelocities( 1.f/solverdt );
} // btDX11SoftBodySolver::solveConstraints




//////////////////////////////////////
// Kernel dispatches
void btDX11SoftBodySolver::prepareLinks()
{
	// No need to batch link solver, it is entirely parallel
	// Copy kernel parameters to GPU
	PrepareLinksCB constBuffer;
	
	constBuffer.numLinks = m_linkData.getNumLinks();
	
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( prepareLinksKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(PrepareLinksCB) );	
	m_dx11Context->Unmap( prepareLinksKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &prepareLinksKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_linkData.m_dx11Links.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_linkData.m_dx11LinksMassLSC.getSRV()) );
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_vertexData.m_dx11VertexPreviousPosition.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_linkData.m_dx11LinksLengthRatio.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_linkData.m_dx11LinksCLength.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( prepareLinksKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numLinks + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks , 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );
		
		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}
} // btDX11SoftBodySolver::prepareLinks


void btDX11SoftBodySolver::updatePositionsFromVelocities( float solverdt )
{
	// No need to batch link solver, it is entirely parallel
	// Copy kernel parameters to GPU
	UpdatePositionsFromVelocitiesCB constBuffer;
	
	constBuffer.numNodes = m_vertexData.getNumVertices();
	constBuffer.solverSDT = solverdt;
	
	// Todo: factor this out. Number of nodes is static and sdt might be, too, we can update this just once on setup
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( updatePositionsFromVelocitiesKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(UpdatePositionsFromVelocitiesCB) );	
	m_dx11Context->Unmap( updatePositionsFromVelocitiesKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &updatePositionsFromVelocitiesKernel.constBuffer );

	// Set resources and dispatch			
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_vertexData.m_dx11VertexVelocity.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexPreviousPosition.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexPosition.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( updatePositionsFromVelocitiesKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks, 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SoftBodySolver::updatePositionsFromVelocities

void btDX11SoftBodySolver::solveLinksForPosition( int startLink, int numLinks, float kst, float ti )
{
	// Copy kernel parameters to GPU
	SolvePositionsFromLinksKernelCB constBuffer;

	// Set the first link of the batch
	// and the batch size
	constBuffer.startLink = startLink;
	constBuffer.numLinks = numLinks;

	constBuffer.kst = kst;
	constBuffer.ti = ti;
	
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( solvePositionsFromLinksKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(SolvePositionsFromLinksKernelCB) );	
	m_dx11Context->Unmap( solvePositionsFromLinksKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &solvePositionsFromLinksKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_linkData.m_dx11Links.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_linkData.m_dx11LinksMassLSC.getSRV()) );
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_linkData.m_dx11LinksRestLengthSquared.getSRV()) );
	m_dx11Context->CSSetShaderResources( 3, 1, &(m_vertexData.m_dx11VertexInverseMass.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexPosition.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( solvePositionsFromLinksKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numLinks + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks , 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 3, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
	
} // btDX11SoftBodySolver::solveLinksForPosition

void btDX11SoftBodySolver::solveLinksForVelocity( int startLink, int numLinks, float kst )
{
	// Copy kernel parameters to GPU
	VSolveLinksCB constBuffer;

	// Set the first link of the batch
	// and the batch size

	constBuffer.startLink = startLink;
	constBuffer.numLinks = numLinks;
	constBuffer.kst = kst;
	
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( vSolveLinksKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(VSolveLinksCB) );	
	m_dx11Context->Unmap( vSolveLinksKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &vSolveLinksKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_linkData.m_dx11Links.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_linkData.m_dx11LinksLengthRatio.getSRV()) );
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_linkData.m_dx11LinksCLength.getSRV()) );
	m_dx11Context->CSSetShaderResources( 3, 1, &(m_vertexData.m_dx11VertexInverseMass.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexVelocity.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( vSolveLinksKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numLinks + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks , 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 3, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SoftBodySolver::solveLinksForVelocity


void btDX11SoftBodySolver::updateVelocitiesFromPositionsWithVelocities( float isolverdt )
{
	// Copy kernel parameters to GPU
	UpdateVelocitiesFromPositionsWithVelocitiesCB constBuffer;

	// Set the first link of the batch
	// and the batch size
	constBuffer.numNodes = m_vertexData.getNumVertices();
	constBuffer.isolverdt = isolverdt;

	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( updateVelocitiesFromPositionsWithVelocitiesKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(UpdateVelocitiesFromPositionsWithVelocitiesCB) );	
	m_dx11Context->Unmap( updateVelocitiesFromPositionsWithVelocitiesKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &updateVelocitiesFromPositionsWithVelocitiesKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_vertexData.m_dx11VertexPosition.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_vertexData.m_dx11VertexPreviousPosition.getSRV()) );
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_vertexData.m_dx11ClothIdentifier.getSRV()) );
	m_dx11Context->CSSetShaderResources( 3, 1, &(m_dx11PerClothVelocityCorrectionCoefficient.getSRV()) );
	m_dx11Context->CSSetShaderResources( 4, 1, &(m_dx11PerClothDampingFactor.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexVelocity.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexForceAccumulator.getUAV()), NULL );


	// Execute the kernel
	m_dx11Context->CSSetShader( updateVelocitiesFromPositionsWithVelocitiesKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks , 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 3, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 4, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	

} // btDX11SoftBodySolver::updateVelocitiesFromPositionsWithVelocities

void btDX11SoftBodySolver::updateVelocitiesFromPositionsWithoutVelocities( float isolverdt )
{
	// Copy kernel parameters to GPU
	UpdateVelocitiesFromPositionsWithoutVelocitiesCB constBuffer;

	// Set the first link of the batch
	// and the batch size
	constBuffer.numNodes = m_vertexData.getNumVertices();
	constBuffer.isolverdt = isolverdt;

	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( updateVelocitiesFromPositionsWithoutVelocitiesKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(UpdateVelocitiesFromPositionsWithoutVelocitiesCB) );	
	m_dx11Context->Unmap( updateVelocitiesFromPositionsWithoutVelocitiesKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &updateVelocitiesFromPositionsWithoutVelocitiesKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_vertexData.m_dx11VertexPosition.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_vertexData.m_dx11VertexPreviousPosition.getSRV()) );
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_vertexData.m_dx11ClothIdentifier.getSRV()) );
	m_dx11Context->CSSetShaderResources( 3, 1, &(m_dx11PerClothDampingFactor.getSRV()) );

	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexVelocity.getUAV()), NULL );
	m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &(m_vertexData.m_dx11VertexForceAccumulator.getUAV()), NULL );


	// Execute the kernel
	m_dx11Context->CSSetShader( updateVelocitiesFromPositionsWithoutVelocitiesKernel.kernel, NULL, 0 );

	int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
	m_dx11Context->Dispatch(numBlocks , 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 3, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );
		m_dx11Context->CSSetUnorderedAccessViews( 1, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	

} // btDX11SoftBodySolver::updateVelocitiesFromPositionsWithoutVelocities

// End kernel dispatches
/////////////////////////////////////














btDX11SoftBodySolver::btAcceleratedSoftBodyInterface *btDX11SoftBodySolver::findSoftBodyInterface( const btSoftBody* const softBody )
{
	for( int softBodyIndex = 0; softBodyIndex < m_softBodySet.size(); ++softBodyIndex )
	{
		btAcceleratedSoftBodyInterface *softBodyInterface = m_softBodySet[softBodyIndex];
		if( softBodyInterface->getSoftBody() == softBody )
			return softBodyInterface;
	}
	return 0;
}

void btDX11SoftBodySolver::copySoftBodyToVertexBuffer( const btSoftBody * const softBody, btVertexBufferDescriptor *vertexBuffer )
{
	checkInitialized();
	
	btAcceleratedSoftBodyInterface *currentCloth = findSoftBodyInterface( softBody );


	const int firstVertex = currentCloth->getFirstVertex();
	const int lastVertex = firstVertex + currentCloth->getNumVertices();

	if( vertexBuffer->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER )
	{		
		// If we're doing a CPU-buffer copy must copy the data back to the host first
		m_vertexData.m_dx11VertexPosition.copyFromGPU();
		m_vertexData.m_dx11VertexNormal.copyFromGPU();

		const int firstVertex = currentCloth->getFirstVertex();
		const int lastVertex = firstVertex + currentCloth->getNumVertices();
		const btCPUVertexBufferDescriptor *cpuVertexBuffer = static_cast< btCPUVertexBufferDescriptor* >(vertexBuffer);						
		float *basePointer = cpuVertexBuffer->getBasePointer();						

		if( vertexBuffer->hasVertexPositions() )
		{
			const int vertexOffset = cpuVertexBuffer->getVertexOffset();
			const int vertexStride = cpuVertexBuffer->getVertexStride();
			float *vertexPointer = basePointer + vertexOffset;

			for( int vertexIndex = firstVertex; vertexIndex < lastVertex; ++vertexIndex )
			{
				Vectormath::Aos::Point3 position = m_vertexData.getPosition(vertexIndex);
				*(vertexPointer + 0) = position.getX();
				*(vertexPointer + 1) = position.getY();
				*(vertexPointer + 2) = position.getZ();
				vertexPointer += vertexStride;
			}
		}
		if( vertexBuffer->hasNormals() )
		{
			const int normalOffset = cpuVertexBuffer->getNormalOffset();
			const int normalStride = cpuVertexBuffer->getNormalStride();
			float *normalPointer = basePointer + normalOffset;

			for( int vertexIndex = firstVertex; vertexIndex < lastVertex; ++vertexIndex )
			{
				Vectormath::Aos::Vector3 normal = m_vertexData.getNormal(vertexIndex);
				*(normalPointer + 0) = normal.getX();
				*(normalPointer + 1) = normal.getY();
				*(normalPointer + 2) = normal.getZ();
				normalPointer += normalStride;
			}
		}
	} else 	if( vertexBuffer->getBufferType() == btVertexBufferDescriptor::DX11_BUFFER )
	{
		// Do a DX11 copy shader DX to DX copy

		const btDX11VertexBufferDescriptor *dx11VertexBuffer = static_cast< btDX11VertexBufferDescriptor* >(vertexBuffer);	

		// No need to batch link solver, it is entirely parallel
		// Copy kernel parameters to GPU
		OutputToVertexArrayCB constBuffer;
		ID3D11ComputeShader* outputToVertexArrayShader = outputToVertexArrayWithoutNormalsKernel.kernel;
		ID3D11Buffer* outputToVertexArrayConstBuffer = outputToVertexArrayWithoutNormalsKernel.constBuffer;
		
		constBuffer.startNode = firstVertex;
		constBuffer.numNodes = currentCloth->getNumVertices();
		constBuffer.positionOffset = vertexBuffer->getVertexOffset();
		constBuffer.positionStride = vertexBuffer->getVertexStride();
		if( vertexBuffer->hasNormals() )
		{
			constBuffer.normalOffset = vertexBuffer->getNormalOffset();
			constBuffer.normalStride = vertexBuffer->getNormalStride();
			outputToVertexArrayShader = outputToVertexArrayWithNormalsKernel.kernel;
			outputToVertexArrayConstBuffer = outputToVertexArrayWithNormalsKernel.constBuffer;
		}	
		
		// TODO: factor this out. Number of nodes is static and sdt might be, too, we can update this just once on setup
		D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
		m_dx11Context->Map( outputToVertexArrayConstBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
		memcpy( MappedResource.pData, &constBuffer, sizeof(OutputToVertexArrayCB) );	
		m_dx11Context->Unmap( outputToVertexArrayConstBuffer, 0 );
		m_dx11Context->CSSetConstantBuffers( 0, 1, &outputToVertexArrayConstBuffer );

		// Set resources and dispatch
		m_dx11Context->CSSetShaderResources( 0, 1, &(m_vertexData.m_dx11VertexPosition.getSRV()) );
		m_dx11Context->CSSetShaderResources( 1, 1, &(m_vertexData.m_dx11VertexNormal.getSRV()) );

		ID3D11UnorderedAccessView* dx11UAV = dx11VertexBuffer->getDX11UAV();
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(dx11UAV), NULL );

		// Execute the kernel
		m_dx11Context->CSSetShader( outputToVertexArrayShader, NULL, 0 );

		int	numBlocks = (constBuffer.numNodes + (128-1)) / 128;
		m_dx11Context->Dispatch(numBlocks, 1, 1 );

		{
			// Tidy up 
			ID3D11ShaderResourceView* pViewNULL = NULL;
			m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
			m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );

			ID3D11UnorderedAccessView* pUAViewNULL = NULL;
			m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );

			ID3D11Buffer *pBufferNull = NULL;
			m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
		}	
	}





	if( vertexBuffer->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER )
	{		
		const int firstVertex = currentCloth->getFirstVertex();
		const int lastVertex = firstVertex + currentCloth->getNumVertices();
		const btCPUVertexBufferDescriptor *cpuVertexBuffer = static_cast< btCPUVertexBufferDescriptor* >(vertexBuffer);						
		float *basePointer = cpuVertexBuffer->getBasePointer();						

		if( vertexBuffer->hasVertexPositions() )
		{
			const int vertexOffset = cpuVertexBuffer->getVertexOffset();
			const int vertexStride = cpuVertexBuffer->getVertexStride();
			float *vertexPointer = basePointer + vertexOffset;

			for( int vertexIndex = firstVertex; vertexIndex < lastVertex; ++vertexIndex )
			{
				Vectormath::Aos::Point3 position = m_vertexData.getPosition(vertexIndex);
				*(vertexPointer + 0) = position.getX();
				*(vertexPointer + 1) = position.getY();
				*(vertexPointer + 2) = position.getZ();
				vertexPointer += vertexStride;
			}
		}
		if( vertexBuffer->hasNormals() )
		{
			const int normalOffset = cpuVertexBuffer->getNormalOffset();
			const int normalStride = cpuVertexBuffer->getNormalStride();
			float *normalPointer = basePointer + normalOffset;

			for( int vertexIndex = firstVertex; vertexIndex < lastVertex; ++vertexIndex )
			{
				Vectormath::Aos::Vector3 normal = m_vertexData.getNormal(vertexIndex);
				*(normalPointer + 0) = normal.getX();
				*(normalPointer + 1) = normal.getY();
				*(normalPointer + 2) = normal.getZ();
				normalPointer += normalStride;
			}
		}
	}
} // btDX11SoftBodySolver::outputToVertexBuffers




btDX11SoftBodySolver::KernelDesc btDX11SoftBodySolver::compileComputeShaderFromString( const char* shaderString, const char* shaderName, int constBufferSize )
{
	const char *cs5String = "cs_5_0";

	HRESULT hr = S_OK;
	ID3DBlob* pErrorBlob = NULL;
	ID3DBlob* pBlob = NULL;
	ID3D11ComputeShader*		kernelPointer = 0;
	
	hr = D3DX11CompileFromMemory( 
		shaderString,
		strlen(shaderString),
		shaderName, // file name
		NULL,
		NULL,
		shaderName,
		cs5String,
		D3D10_SHADER_ENABLE_STRICTNESS,
		NULL,
		NULL,
		&pBlob,
		&pErrorBlob,
		NULL
		);

	if( FAILED(hr) )
	{
		if( pErrorBlob ) {
			btAssert( "Compilation of compute shader failed\n" );
			//OutputDebugStringA( (char*)pErrorBlob->GetBufferPointer() );
		}
	
		SAFE_RELEASE( pErrorBlob );
		SAFE_RELEASE( pBlob );    

		btDX11SoftBodySolver::KernelDesc descriptor;
		descriptor.kernel = 0;
		descriptor.constBuffer = 0;
		return descriptor;
	}    

	// Create the Compute Shader
	hr = m_dx11Device->CreateComputeShader( pBlob->GetBufferPointer(), pBlob->GetBufferSize(), NULL, &kernelPointer );
	if( FAILED( hr ) )
	{
		btDX11SoftBodySolver::KernelDesc descriptor;
		descriptor.kernel = 0;
		descriptor.constBuffer = 0;
		return descriptor;
	}

	ID3D11Buffer* constBuffer = 0;
	if( constBufferSize > 0 )
	{
		// Create the constant buffer
		D3D11_BUFFER_DESC constant_buffer_desc;
		ZeroMemory(&constant_buffer_desc, sizeof(constant_buffer_desc));
		constant_buffer_desc.ByteWidth = constBufferSize;
		constant_buffer_desc.Usage = D3D11_USAGE_DYNAMIC;
		constant_buffer_desc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
		constant_buffer_desc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
		m_dx11Device->CreateBuffer(&constant_buffer_desc, NULL, &constBuffer);
		if( FAILED( hr ) )
		{
			KernelDesc descriptor;
			descriptor.kernel = 0;
			descriptor.constBuffer = 0;
			return descriptor;
		}
	}

	SAFE_RELEASE( pErrorBlob );
	SAFE_RELEASE( pBlob );

	btDX11SoftBodySolver::KernelDesc descriptor;
	descriptor.kernel = kernelPointer;
	descriptor.constBuffer = constBuffer;
	return descriptor;
} // compileComputeShader


bool btDX11SoftBodySolver::buildShaders()
{
	bool returnVal = true;

	if( m_shadersInitialized )
		return true;

	prepareLinksKernel = compileComputeShaderFromString( PrepareLinksHLSLString, "PrepareLinksKernel", sizeof(PrepareLinksCB) );
	if( !prepareLinksKernel.constBuffer )
		returnVal = false;
	updatePositionsFromVelocitiesKernel = compileComputeShaderFromString( UpdatePositionsFromVelocitiesHLSLString, "UpdatePositionsFromVelocitiesKernel", sizeof(UpdatePositionsFromVelocitiesCB) );
	if( !updatePositionsFromVelocitiesKernel.constBuffer )
		returnVal = false;
	solvePositionsFromLinksKernel = compileComputeShaderFromString( SolvePositionsHLSLString, "SolvePositionsFromLinksKernel", sizeof(SolvePositionsFromLinksKernelCB) );
	if( !updatePositionsFromVelocitiesKernel.constBuffer )
		returnVal = false;
	vSolveLinksKernel = compileComputeShaderFromString( VSolveLinksHLSLString, "VSolveLinksKernel", sizeof(VSolveLinksCB) );
	if( !vSolveLinksKernel.constBuffer )
		returnVal = false;
	updateVelocitiesFromPositionsWithVelocitiesKernel = compileComputeShaderFromString( UpdateNodesHLSLString, "updateVelocitiesFromPositionsWithVelocitiesKernel", sizeof(UpdateVelocitiesFromPositionsWithVelocitiesCB) );
	if( !updateVelocitiesFromPositionsWithVelocitiesKernel.constBuffer )
		returnVal = false;
	updateVelocitiesFromPositionsWithoutVelocitiesKernel = compileComputeShaderFromString( UpdatePositionsHLSLString, "updateVelocitiesFromPositionsWithoutVelocitiesKernel", sizeof(UpdateVelocitiesFromPositionsWithoutVelocitiesCB) );
	if( !updateVelocitiesFromPositionsWithoutVelocitiesKernel.constBuffer )
		returnVal = false;
	integrateKernel = compileComputeShaderFromString( IntegrateHLSLString, "IntegrateKernel", sizeof(IntegrateCB) );
	if( !integrateKernel.constBuffer )
		returnVal = false;
	applyForcesKernel = compileComputeShaderFromString( ApplyForcesHLSLString, "ApplyForcesKernel", sizeof(ApplyForcesCB) );
	if( !applyForcesKernel.constBuffer )
		returnVal = false;

	// TODO: Rename to UpdateSoftBodies
	resetNormalsAndAreasKernel = compileComputeShaderFromString( UpdateNormalsHLSLString, "ResetNormalsAndAreasKernel", sizeof(UpdateSoftBodiesCB) );
	if( !resetNormalsAndAreasKernel.constBuffer )
		returnVal = false;
	normalizeNormalsAndAreasKernel = compileComputeShaderFromString( UpdateNormalsHLSLString, "NormalizeNormalsAndAreasKernel", sizeof(UpdateSoftBodiesCB) );
	if( !normalizeNormalsAndAreasKernel.constBuffer )
		returnVal = false;
	updateSoftBodiesKernel = compileComputeShaderFromString( UpdateNormalsHLSLString, "UpdateSoftBodiesKernel", sizeof(UpdateSoftBodiesCB) );
	if( !updateSoftBodiesKernel.constBuffer )
		returnVal = false;
	outputToVertexArrayWithNormalsKernel = compileComputeShaderFromString( OutputToVertexArrayHLSLString, "OutputToVertexArrayWithNormalsKernel", sizeof(OutputToVertexArrayCB) );
	if( !outputToVertexArrayWithNormalsKernel.constBuffer )
		returnVal = false;
	outputToVertexArrayWithoutNormalsKernel = compileComputeShaderFromString( OutputToVertexArrayHLSLString, "OutputToVertexArrayWithoutNormalsKernel", sizeof(OutputToVertexArrayCB) );
	if( !outputToVertexArrayWithoutNormalsKernel.constBuffer )
		returnVal = false;



	if( returnVal )
		m_shadersInitialized = true;

	return returnVal;
}


void btDX11SoftBodySolver::predictMotion( float timeStep )
{
	// Fill the force arrays with current acceleration data etc
	m_perClothWindVelocity.resize( m_softBodySet.size() );
	for( int softBodyIndex = 0; softBodyIndex < m_softBodySet.size(); ++softBodyIndex )
	{
		btSoftBody *softBody = m_softBodySet[softBodyIndex]->getSoftBody();
		
		m_perClothWindVelocity[softBodyIndex] = toVector3(softBody->getWindVelocity());
	}
	m_dx11PerClothWindVelocity.changedOnCPU();

	// Apply forces that we know about to the cloths
	applyForces(  timeStep * getTimeScale() );

	// Itegrate motion for all soft bodies dealt with by the solver
	integrate( timeStep * getTimeScale() );
	// End prediction work for solvers
}

