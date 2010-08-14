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

#include <cstdio>


#define WAVEFRONT_SIZE 32
#define WAVEFRONT_BLOCK_MULTIPLIER 2
#define LINKS_PER_SIMD_LANE 16

#define STRINGIFY( S ) STRINGIFY2( S )
#define STRINGIFY2( S ) #S

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "vectormath/vmInclude.h"

#include "btSoftBodySolverLinkData_DX11SIMDAware.h"
#include "btSoftBodySolver_DX11SIMDAware.h"
#include "btSoftBodySolverVertexBuffer_DX11.h"
#include "BulletSoftBody/btSoftBody.h"

#define MSTRINGIFY(A) #A
static char* PrepareLinksHLSLString = 
#include "HLSL/PrepareLinks.hlsl"
static char* UpdatePositionsFromVelocitiesHLSLString = 
#include "HLSL/UpdatePositionsFromVelocities.hlsl"
static char* SolvePositionsSIMDBatchedHLSLString = 
#include "HLSL/SolvePositionsSIMDBatched.hlsl"
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



btSoftBodyLinkDataDX11SIMDAware::btSoftBodyLinkDataDX11SIMDAware( ID3D11Device *d3dDevice, ID3D11DeviceContext *d3dDeviceContext ) : 
		m_d3dDevice( d3dDevice ),
		m_d3dDeviceContext( d3dDeviceContext ),
		m_wavefrontSize( WAVEFRONT_SIZE ),
		m_linksPerWorkItem( LINKS_PER_SIMD_LANE ),
		m_maxBatchesWithinWave( 0 ),
		m_maxLinksPerWavefront( m_wavefrontSize * m_linksPerWorkItem ),
		m_numWavefronts( 0 ),
		m_maxVertex( 0 ),
		m_dx11NumBatchesAndVerticesWithinWaves( d3dDevice, d3dDeviceContext, &m_numBatchesAndVerticesWithinWaves, true ),
		m_dx11WavefrontVerticesGlobalAddresses( d3dDevice, d3dDeviceContext, &m_wavefrontVerticesGlobalAddresses, true ),
		m_dx11LinkVerticesLocalAddresses( d3dDevice, d3dDeviceContext, &m_linkVerticesLocalAddresses, true ),
		m_dx11LinkStrength( d3dDevice, d3dDeviceContext, &m_linkStrength, true ),
		m_dx11LinksMassLSC( d3dDevice, d3dDeviceContext, &m_linksMassLSC, true ),
		m_dx11LinksRestLengthSquared( d3dDevice, d3dDeviceContext, &m_linksRestLengthSquared, true ),
		m_dx11LinksRestLength( d3dDevice, d3dDeviceContext, &m_linksRestLength, true ),
		m_dx11LinksMaterialLinearStiffnessCoefficient( d3dDevice, d3dDeviceContext, &m_linksMaterialLinearStiffnessCoefficient, true )
{
	m_d3dDevice = d3dDevice;
	m_d3dDeviceContext = d3dDeviceContext;
}

btSoftBodyLinkDataDX11SIMDAware::~btSoftBodyLinkDataDX11SIMDAware()
{
}

static Vectormath::Aos::Vector3 toVector3( const btVector3 &vec )
{
	Vectormath::Aos::Vector3 outVec( vec.getX(), vec.getY(), vec.getZ() );
	return outVec;
}

void btSoftBodyLinkDataDX11SIMDAware::createLinks( int numLinks )
{
	int previousSize = m_links.size();
	int newSize = previousSize + numLinks;

	btSoftBodyLinkData::createLinks( numLinks );

	// Resize the link addresses array as well
	m_linkAddresses.resize( newSize );
}

void btSoftBodyLinkDataDX11SIMDAware::setLinkAt( const btSoftBodyLinkData::LinkDescription &link, int linkIndex )
{
	btSoftBodyLinkData::setLinkAt( link, linkIndex );

	if( link.getVertex0() > m_maxVertex )
		m_maxVertex = link.getVertex0();
	if( link.getVertex1() > m_maxVertex )
		m_maxVertex = link.getVertex1();

	// Set the link index correctly for initialisation
	m_linkAddresses[linkIndex] = linkIndex;
}

bool btSoftBodyLinkDataDX11SIMDAware::onAccelerator()
{
	return m_onGPU;
}

bool btSoftBodyLinkDataDX11SIMDAware::moveToAccelerator()
{
	bool success = true;

	success = success && m_dx11NumBatchesAndVerticesWithinWaves.moveToGPU();
	success = success && m_dx11WavefrontVerticesGlobalAddresses.moveToGPU();
	success = success && m_dx11LinkVerticesLocalAddresses.moveToGPU();
	success = success && m_dx11LinkStrength.moveToGPU();
	success = success && m_dx11LinksMassLSC.moveToGPU();
	success = success && m_dx11LinksRestLengthSquared.moveToGPU();
	success = success && m_dx11LinksRestLength.moveToGPU();
	success = success && m_dx11LinksMaterialLinearStiffnessCoefficient.moveToGPU();

	if( success )
		m_onGPU = true;

	return success;
}

bool btSoftBodyLinkDataDX11SIMDAware::moveFromAccelerator()
{
	bool success = true;
	success = success && m_dx11NumBatchesAndVerticesWithinWaves.moveFromGPU();
	success = success && m_dx11WavefrontVerticesGlobalAddresses.moveFromGPU();
	success = success && m_dx11LinkVerticesLocalAddresses.moveFromGPU();
	success = success && m_dx11LinkStrength.moveFromGPU();
	success = success && m_dx11LinksMassLSC.moveFromGPU();
	success = success && m_dx11LinksRestLengthSquared.moveFromGPU();
	success = success && m_dx11LinksRestLength.moveFromGPU();
	success = success && m_dx11LinksMaterialLinearStiffnessCoefficient.moveFromGPU();

	if( success )
		m_onGPU = false;

	return success;
}















btDX11SIMDAwareSoftBodySolver::btDX11SIMDAwareSoftBodySolver(ID3D11Device * dx11Device, ID3D11DeviceContext* dx11Context) :
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

void btDX11SIMDAwareSoftBodySolver::releaseKernels()
{
	SAFE_RELEASE( integrateKernel.constBuffer );
	SAFE_RELEASE( integrateKernel.kernel );
	SAFE_RELEASE( solvePositionsFromLinksKernel.constBuffer );
	SAFE_RELEASE( solvePositionsFromLinksKernel.kernel );
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

	m_shadersInitialized = false;
}

btDX11SIMDAwareSoftBodySolver::~btDX11SIMDAwareSoftBodySolver()
{
	releaseKernels();
}


void btDX11SIMDAwareSoftBodySolver::optimize( btAlignedObjectArray< btSoftBody * > &softBodies )
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

		
		// Build the shaders to match the batching parameters
		buildShaders();
	}

}


btSoftBodyLinkData &btDX11SIMDAwareSoftBodySolver::getLinkData()
{
	// TODO: Consider setting link data to "changed" here
	return m_linkData;
}

btSoftBodyVertexData &btDX11SIMDAwareSoftBodySolver::getVertexData()
{
	// TODO: Consider setting vertex data to "changed" here
	return m_vertexData;
}

btSoftBodyTriangleData &btDX11SIMDAwareSoftBodySolver::getTriangleData()
{
	// TODO: Consider setting triangle data to "changed" here
	return m_triangleData;
}


bool btDX11SIMDAwareSoftBodySolver::checkInitialized()
{
	if( !m_shadersInitialized )
		if( buildShaders() )
			m_shadersInitialized = true;

	return m_shadersInitialized;
}

void btDX11SIMDAwareSoftBodySolver::resetNormalsAndAreas( int numVertices )
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
} // btDX11SIMDAwareSoftBodySolver::resetNormalsAndAreas

void btDX11SIMDAwareSoftBodySolver::normalizeNormalsAndAreas( int numVertices )
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
} // btDX11SIMDAwareSoftBodySolver::normalizeNormalsAndAreas

void btDX11SIMDAwareSoftBodySolver::executeUpdateSoftBodies( int firstTriangle, int numTriangles )
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
} // btDX11SIMDAwareSoftBodySolver::executeUpdateSoftBodies

void btDX11SIMDAwareSoftBodySolver::updateSoftBodies()
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

} // btDX11SIMDAwareSoftBodySolver::updateSoftBodies


Vectormath::Aos::Vector3 btDX11SIMDAwareSoftBodySolver::ProjectOnAxis( const Vectormath::Aos::Vector3 &v, const Vectormath::Aos::Vector3 &a )
{
	return a*Vectormath::Aos::dot(v, a);
}

void btDX11SIMDAwareSoftBodySolver::ApplyClampedForce( float solverdt, const Vectormath::Aos::Vector3 &force, const Vectormath::Aos::Vector3 &vertexVelocity, float inverseMass, Vectormath::Aos::Vector3 &vertexForce )
{
	float dtInverseMass = solverdt*inverseMass;
	if( Vectormath::Aos::lengthSqr(force * dtInverseMass) > Vectormath::Aos::lengthSqr(vertexVelocity) )
	{
		vertexForce -= ProjectOnAxis( vertexVelocity, normalize( force ) )/dtInverseMass;
	} else {
		vertexForce += force;
	}
}

void btDX11SIMDAwareSoftBodySolver::applyForces( float solverdt )
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


} // btDX11SIMDAwareSoftBodySolver::applyForces

/**
 * Integrate motion on the solver.
 */
void btDX11SIMDAwareSoftBodySolver::integrate( float solverdt )
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
} // btDX11SIMDAwareSoftBodySolver::integrate

float btDX11SIMDAwareSoftBodySolver::computeTriangleArea( 
	const Vectormath::Aos::Point3 &vertex0,
	const Vectormath::Aos::Point3 &vertex1,
	const Vectormath::Aos::Point3 &vertex2 )
{
	Vectormath::Aos::Vector3 a = vertex1 - vertex0;
	Vectormath::Aos::Vector3 b = vertex2 - vertex0;
	Vectormath::Aos::Vector3 crossProduct = cross(a, b);
	float area = length( crossProduct );
	return area;
} // btDX11SIMDAwareSoftBodySolver::computeTriangleArea

// Update constants here is a simple CPU version that is run on optimize
void btDX11SIMDAwareSoftBodySolver::updateConstants( float timeStep )
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
} // btDX11SIMDAwareSoftBodySolver::updateConstants



void btDX11SIMDAwareSoftBodySolver::solveConstraints( float solverdt )
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

	// Solve drift
  	for( int iteration = 0; iteration < m_numberOfPositionIterations ; ++iteration )
	{
 		int it = iteration; 

		for( int i = 0; i < m_linkData.m_wavefrontBatchStartLengths.size(); ++i )
		{
			int startWave = m_linkData.m_wavefrontBatchStartLengths[i].start;
			int numWaves = m_linkData.m_wavefrontBatchStartLengths[i].length;

			solveLinksForPosition( startWave, numWaves, kst, ti );
		}	

	} // for( int iteration = 0; iteration < m_numberOfPositionIterations ; ++iteration )




	updateVelocitiesFromPositionsWithoutVelocities( 1.f/solverdt );

} // btDX11SIMDAwareSoftBodySolver::solveConstraints




//////////////////////////////////////
// Kernel dispatches


void btDX11SIMDAwareSoftBodySolver::updatePositionsFromVelocities( float solverdt )
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
} // btDX11SIMDAwareSoftBodySolver::updatePositionsFromVelocities


void btDX11SIMDAwareSoftBodySolver::solveLinksForPosition( int startWave, int numWaves, float kst, float ti )
{


	m_vertexData.moveToAccelerator();
	m_linkData.moveToAccelerator();

	// Copy kernel parameters to GPU
	SolvePositionsFromLinksKernelCB constBuffer;

	// Set the first wave of the batch and the number of waves
	constBuffer.startWave = startWave;
	constBuffer.numWaves = numWaves;

	constBuffer.kst = kst;
	constBuffer.ti = ti;
	
	D3D11_MAPPED_SUBRESOURCE MappedResource = {0};
	m_dx11Context->Map( solvePositionsFromLinksKernel.constBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &MappedResource );
	memcpy( MappedResource.pData, &constBuffer, sizeof(SolvePositionsFromLinksKernelCB) );	
	m_dx11Context->Unmap( solvePositionsFromLinksKernel.constBuffer, 0 );
	m_dx11Context->CSSetConstantBuffers( 0, 1, &solvePositionsFromLinksKernel.constBuffer );

	// Set resources and dispatch
	m_dx11Context->CSSetShaderResources( 0, 1, &(m_linkData.m_dx11NumBatchesAndVerticesWithinWaves.getSRV()) );
	m_dx11Context->CSSetShaderResources( 1, 1, &(m_linkData.m_dx11WavefrontVerticesGlobalAddresses.getSRV()) );
	m_dx11Context->CSSetShaderResources( 2, 1, &(m_vertexData.m_dx11VertexInverseMass.getSRV()) );
	m_dx11Context->CSSetShaderResources( 3, 1, &(m_linkData.m_dx11LinkVerticesLocalAddresses.getSRV()) );
	m_dx11Context->CSSetShaderResources( 4, 1, &(m_linkData.m_dx11LinksMassLSC.getSRV()) );
	m_dx11Context->CSSetShaderResources( 5, 1, &(m_linkData.m_dx11LinksRestLengthSquared.getSRV()) );
	
	m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &(m_vertexData.m_dx11VertexPosition.getUAV()), NULL );

	// Execute the kernel
	m_dx11Context->CSSetShader( solvePositionsFromLinksKernel.kernel, NULL, 0 );

	int	numBlocks = ((constBuffer.numWaves + WAVEFRONT_BLOCK_MULTIPLIER - 1) / WAVEFRONT_BLOCK_MULTIPLIER );
	m_dx11Context->Dispatch(numBlocks , 1, 1 );

	{
		// Tidy up 
		ID3D11ShaderResourceView* pViewNULL = NULL;
		m_dx11Context->CSSetShaderResources( 0, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 1, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 2, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 3, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 4, 1, &pViewNULL );
		m_dx11Context->CSSetShaderResources( 5, 1, &pViewNULL );

		ID3D11UnorderedAccessView* pUAViewNULL = NULL;
		m_dx11Context->CSSetUnorderedAccessViews( 0, 1, &pUAViewNULL, NULL );

		ID3D11Buffer *pBufferNull = NULL;
		m_dx11Context->CSSetConstantBuffers( 0, 1, &pBufferNull );
	}	
} // btDX11SIMDAwareSoftBodySolver::solveLinksForPosition


void btDX11SIMDAwareSoftBodySolver::updateVelocitiesFromPositionsWithVelocities( float isolverdt )
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

} // btDX11SIMDAwareSoftBodySolver::updateVelocitiesFromPositionsWithVelocities

void btDX11SIMDAwareSoftBodySolver::updateVelocitiesFromPositionsWithoutVelocities( float isolverdt )
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

} // btDX11SIMDAwareSoftBodySolver::updateVelocitiesFromPositionsWithoutVelocities

// End kernel dispatches
/////////////////////////////////////









btDX11SIMDAwareSoftBodySolver::btAcceleratedSoftBodyInterface *btDX11SIMDAwareSoftBodySolver::findSoftBodyInterface( const btSoftBody* const softBody )
{
	for( int softBodyIndex = 0; softBodyIndex < m_softBodySet.size(); ++softBodyIndex )
	{
		btAcceleratedSoftBodyInterface *softBodyInterface = m_softBodySet[softBodyIndex];
		if( softBodyInterface->getSoftBody() == softBody )
			return softBodyInterface;
	}
	return 0;
}

void btDX11SIMDAwareSoftBodySolver::copySoftBodyToVertexBuffer( const btSoftBody * const softBody, btVertexBufferDescriptor *vertexBuffer )
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

} // btDX11SoftBodySolver::outputToVertexBuffers





btDX11SIMDAwareSoftBodySolver::KernelDesc btDX11SIMDAwareSoftBodySolver::compileComputeShaderFromString( const char* shaderString, const char* shaderName, int constBufferSize, D3D10_SHADER_MACRO *compileMacros )
{
	const char *cs5String = "cs_5_0";

	HRESULT hr = S_OK;
	ID3DBlob* pErrorBlob = NULL;
	ID3DBlob* pBlob = NULL;
	ID3D11ComputeShader*		kernelPointer = 0;

	hr = D3DX11CompileFromMemory( 
		shaderString,
		strlen(shaderString),
		shaderName,
		compileMacros,
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
			char *debugString = (char*)pErrorBlob->GetBufferPointer();
			OutputDebugStringA( debugString );
		}
	
		SAFE_RELEASE( pErrorBlob );
		SAFE_RELEASE( pBlob );    

		btDX11SIMDAwareSoftBodySolver::KernelDesc descriptor;
		descriptor.kernel = 0;
		descriptor.constBuffer = 0;
		return descriptor;
	}    

	// Create the Compute Shader
	hr = m_dx11Device->CreateComputeShader( pBlob->GetBufferPointer(), pBlob->GetBufferSize(), NULL, &kernelPointer );
	if( FAILED( hr ) )
	{
		btDX11SIMDAwareSoftBodySolver::KernelDesc descriptor;
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

	btDX11SIMDAwareSoftBodySolver::KernelDesc descriptor;
	descriptor.kernel = kernelPointer;
	descriptor.constBuffer = constBuffer;
	return descriptor;
} // compileComputeShader


bool btDX11SIMDAwareSoftBodySolver::buildShaders()
{
	// Ensure current kernels are released first
	releaseKernels();

	bool returnVal = true;


	if( m_shadersInitialized )
		return true;

	
	updatePositionsFromVelocitiesKernel = compileComputeShaderFromString( UpdatePositionsFromVelocitiesHLSLString, "UpdatePositionsFromVelocitiesKernel", sizeof(UpdatePositionsFromVelocitiesCB) );
	if( !updatePositionsFromVelocitiesKernel.constBuffer )
		returnVal = false;
	
	char maxVerticesPerWavefront[20];
	char maxBatchesPerWavefront[20];
	char waveFrontSize[20];
	char waveFrontBlockMultiplier[20];
	char blockSize[20];

	sprintf(maxVerticesPerWavefront, "%d", m_linkData.getMaxVerticesPerWavefront());
	sprintf(maxBatchesPerWavefront, "%d", m_linkData.getMaxBatchesPerWavefront());
	sprintf(waveFrontSize, "%d", m_linkData.getWavefrontSize());	
	sprintf(waveFrontBlockMultiplier, "%d", WAVEFRONT_BLOCK_MULTIPLIER);
	sprintf(blockSize, "%d", WAVEFRONT_BLOCK_MULTIPLIER*m_linkData.getWavefrontSize());
	
	D3D10_SHADER_MACRO solvePositionsMacros[6] = { "MAX_NUM_VERTICES_PER_WAVE", maxVerticesPerWavefront, "MAX_BATCHES_PER_WAVE", maxBatchesPerWavefront, "WAVEFRONT_SIZE", waveFrontSize, "WAVEFRONT_BLOCK_MULTIPLIER", waveFrontBlockMultiplier, "BLOCK_SIZE", blockSize, 0, 0 };

	solvePositionsFromLinksKernel = compileComputeShaderFromString( SolvePositionsSIMDBatchedHLSLString, "SolvePositionsFromLinksKernel", sizeof(SolvePositionsFromLinksKernelCB), solvePositionsMacros );
	if( !solvePositionsFromLinksKernel.constBuffer )
		returnVal = false;

	updateVelocitiesFromPositionsWithVelocitiesKernel = compileComputeShaderFromString( UpdateNodesHLSLString, "updateVelocitiesFromPositionsWithVelocitiesKernel", sizeof(UpdateVelocitiesFromPositionsWithVelocitiesCB) );
	if( !updateVelocitiesFromPositionsWithVelocitiesKernel.constBuffer )
		returnVal = false;
	updateVelocitiesFromPositionsWithoutVelocitiesKernel = compileComputeShaderFromString( UpdatePositionsHLSLString, "updateVelocitiesFromPositionsWithoutVelocitiesKernel", sizeof(UpdateVelocitiesFromPositionsWithoutVelocitiesCB));
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



void btDX11SIMDAwareSoftBodySolver::predictMotion( float timeStep )
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



















static void generateBatchesOfWavefronts( btAlignedObjectArray < btAlignedObjectArray <int> > &linksForWavefronts, btSoftBodyLinkData &linkData, int numVertices, btAlignedObjectArray < btAlignedObjectArray <int> > &wavefrontBatches )
{
	// A per-batch map of truth values stating whether a given vertex is in that batch
	// This allows us to significantly optimize the batching
	btAlignedObjectArray <btAlignedObjectArray<bool> > mapOfVerticesInBatches;

	for( int waveIndex = 0; waveIndex < linksForWavefronts.size(); ++waveIndex )
	{
		btAlignedObjectArray <int> &wavefront( linksForWavefronts[waveIndex] );

		int batch = 0;
		bool placed = false;
		while( batch < wavefrontBatches.size() && !placed )
		{
			// Test the current batch, see if this wave shares any vertex with the waves in the batch
			bool foundSharedVertex = false;
			for( int link = 0; link < wavefront.size(); ++link )
			{
				btSoftBodyLinkData::LinkNodePair vertices = linkData.getVertexPair( wavefront[link] );
				if( (mapOfVerticesInBatches[batch])[vertices.vertex0] || (mapOfVerticesInBatches[batch])[vertices.vertex1] )
				{
					foundSharedVertex = true;
				}
			}

			if( !foundSharedVertex )
			{
				wavefrontBatches[batch].push_back( waveIndex );	
				// Insert vertices into this batch too
				for( int link = 0; link < wavefront.size(); ++link )
				{
					btSoftBodyLinkData::LinkNodePair vertices = linkData.getVertexPair( wavefront[link] );
					(mapOfVerticesInBatches[batch])[vertices.vertex0] = true;
					(mapOfVerticesInBatches[batch])[vertices.vertex1] = true;
				}
				placed = true;
			}
			batch++;
		}
		if( batch == wavefrontBatches.size() && !placed )
		{
			wavefrontBatches.resize( batch + 1 );
			wavefrontBatches[batch].push_back( waveIndex );

			// And resize map as well
			mapOfVerticesInBatches.resize( batch + 1 );
			
			// Resize maps with total number of vertices
			mapOfVerticesInBatches[batch].resize( numVertices, false );

			// Insert vertices into this batch too
			for( int link = 0; link < wavefront.size(); ++link )
			{
				btSoftBodyLinkData::LinkNodePair vertices = linkData.getVertexPair( wavefront[link] );
				(mapOfVerticesInBatches[batch])[vertices.vertex0] = true;
				(mapOfVerticesInBatches[batch])[vertices.vertex1] = true;
			}
		}
	}
	mapOfVerticesInBatches.clear();
}

// Function to remove an object from a vector maintaining correct ordering of the vector
template< typename T > static void removeFromVector( btAlignedObjectArray< T > &vectorToUpdate, int indexToRemove )
{
	int currentSize = vectorToUpdate.size();
	for( int i = indexToRemove; i < (currentSize-1); ++i )
	{
		vectorToUpdate[i] = vectorToUpdate[i+1];
	}
	if( currentSize > 0 )
		vectorToUpdate.resize( currentSize - 1 );
}

/**
 * Insert element into vectorToUpdate at index index.
 */
template< typename T > static void insertAtIndex( btAlignedObjectArray< T > &vectorToUpdate, int index, T element )
{
	vectorToUpdate.resize( vectorToUpdate.size() + 1 );
	for( int i = (vectorToUpdate.size() - 1); i > index; --i )
	{
		vectorToUpdate[i] = vectorToUpdate[i-1];
	}
	vectorToUpdate[index] = element;
}

/** 
 * Insert into btAlignedObjectArray assuming the array is ordered and maintaining both ordering and uniqueness.
 * ie it treats vectorToUpdate as an ordered set.
 */
template< typename T > static void insertUniqueAndOrderedIntoVector( btAlignedObjectArray<T> &vectorToUpdate, T element )
{
	int index = 0;
	while( index < vectorToUpdate.size() && vectorToUpdate[index] < element )
	{
		index++;
	}
	if( index == vectorToUpdate.size() || vectorToUpdate[index] != element )
		insertAtIndex( vectorToUpdate, index, element );
}

// Experimental batch generation that we could use in the simulations
// Attempts to generate larger batches that work on a per-wavefront basis
void generateLinksPerVertex( int numVertices, btSoftBodyLinkData &linkData, btAlignedObjectArray< int > &listOfLinksPerVertex, btAlignedObjectArray <int> &numLinksPerVertex, int &maxLinks )
{
	for( int linkIndex = 0; linkIndex < linkData.getNumLinks(); ++linkIndex )
	{
		btSoftBodyLinkData::LinkNodePair nodes( linkData.getVertexPair(linkIndex) );
		numLinksPerVertex[nodes.vertex0]++;
		numLinksPerVertex[nodes.vertex1]++;
	}
	int maxLinksPerVertex = 0;
	for( int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex )
	{
		maxLinksPerVertex = btMax(numLinksPerVertex[vertexIndex], maxLinksPerVertex);
	}
	maxLinks = maxLinksPerVertex;

	btAlignedObjectArray< int > linksFoundPerVertex;
	linksFoundPerVertex.resize( numVertices, 0 );

	listOfLinksPerVertex.resize( maxLinksPerVertex * numVertices );

	for( int linkIndex = 0; linkIndex < linkData.getNumLinks(); ++linkIndex )
	{
		btSoftBodyLinkData::LinkNodePair nodes( linkData.getVertexPair(linkIndex) );
		{
			// Do vertex 0
			int vertexIndex = nodes.vertex0;
			int linkForVertex = linksFoundPerVertex[nodes.vertex0];
			int linkAddress = vertexIndex * maxLinksPerVertex + linkForVertex;

			listOfLinksPerVertex[linkAddress] = linkIndex;

			linksFoundPerVertex[nodes.vertex0] = linkForVertex + 1;
		}
		{
			// Do vertex 1
			int vertexIndex = nodes.vertex1;
			int linkForVertex = linksFoundPerVertex[nodes.vertex1];
			int linkAddress = vertexIndex * maxLinksPerVertex + linkForVertex;

			listOfLinksPerVertex[linkAddress] = linkIndex;

			linksFoundPerVertex[nodes.vertex1] = linkForVertex + 1;
		}
	}
}

static void computeBatchingIntoWavefronts( 
	btSoftBodyLinkData &linkData, 
	int wavefrontSize, 
	int linksPerWorkItem, 
	int maxLinksPerWavefront, 
	btAlignedObjectArray < btAlignedObjectArray <int> > &linksForWavefronts, 
	btAlignedObjectArray< btAlignedObjectArray < btAlignedObjectArray <int> > > &batchesWithinWaves, /* wave, batch, links in batch */
	btAlignedObjectArray< btAlignedObjectArray< int > > &verticesForWavefronts /* wavefront, vertex */
	)
{
	

	// Attempt generation of larger batches of links.
	btAlignedObjectArray< bool > processedLink;
	processedLink.resize( linkData.getNumLinks() );
	btAlignedObjectArray< int > listOfLinksPerVertex;
	int maxLinksPerVertex = 0;

	// Count num vertices
	int numVertices = 0;
	for( int linkIndex = 0; linkIndex < linkData.getNumLinks(); ++linkIndex )
	{
		btSoftBodyLinkData::LinkNodePair nodes( linkData.getVertexPair(linkIndex) );
		numVertices = btMax( numVertices, nodes.vertex0 + 1 );
		numVertices = btMax( numVertices, nodes.vertex1 + 1 );
	}

	// Need list of links per vertex
	// Compute valence of each vertex
	btAlignedObjectArray <int> numLinksPerVertex;
	numLinksPerVertex.resize(0);
	numLinksPerVertex.resize( numVertices, 0 );

	generateLinksPerVertex( numVertices, linkData, listOfLinksPerVertex, numLinksPerVertex, maxLinksPerVertex );

	for( int vertex = 0; vertex < 10; ++vertex )
	{
		for( int link = 0; link < numLinksPerVertex[vertex]; ++link )
		{
			int linkAddress = vertex * maxLinksPerVertex + link;
		}
	}


	// At this point we know what links we have for each vertex so we can start batching
	
	// We want a vertex to start with, let's go with 0
	int currentVertex = 0;
	int linksProcessed = 0;

	btAlignedObjectArray <int> verticesToProcess;

	while( linksProcessed < linkData.getNumLinks() )
	{
		// Next wavefront
		int nextWavefront = linksForWavefronts.size();
		linksForWavefronts.resize( nextWavefront + 1 );
		btAlignedObjectArray <int> &linksForWavefront(linksForWavefronts[nextWavefront]);
		verticesForWavefronts.resize( nextWavefront + 1 );
		btAlignedObjectArray<int> &vertexSet( verticesForWavefronts[nextWavefront] );

		linksForWavefront.resize(0);

		// Loop to find enough links to fill the wavefront
		// Stopping if we either run out of links, or fill it
		while( linksProcessed < linkData.getNumLinks() && linksForWavefront.size() < maxLinksPerWavefront )
		{
			// Go through the links for the current vertex
			for( int link = 0; link < numLinksPerVertex[currentVertex] && linksForWavefront.size() < maxLinksPerWavefront; ++link )
			{
				int linkAddress = currentVertex * maxLinksPerVertex + link;
				int linkIndex = listOfLinksPerVertex[linkAddress];
				
				// If we have not already processed this link, add it to the wavefront
				// Claim it as another processed link
				// Add the vertex at the far end to the list of vertices to process.
				if( !processedLink[linkIndex] )
				{
					linksForWavefront.push_back( linkIndex );
					linksProcessed++;
					processedLink[linkIndex] = true;
					int v0 = linkData.getVertexPair(linkIndex).vertex0;
					int v1 = linkData.getVertexPair(linkIndex).vertex1;
					if( v0 == currentVertex )
						verticesToProcess.push_back( v1 );
					else
						verticesToProcess.push_back( v0 );
				}
			}
			if( verticesToProcess.size() > 0 )
			{
				// Get the element on the front of the queue and remove it
				currentVertex = verticesToProcess[0];
				removeFromVector( verticesToProcess, 0 );
			} else {		
				// If we've not yet processed all the links, find the first unprocessed one
				// and select one of its vertices as the current vertex
				if( linksProcessed < linkData.getNumLinks() )
				{
					int searchLink = 0;
					while( processedLink[searchLink] )
						searchLink++;
					currentVertex = linkData.getVertexPair(searchLink).vertex0;
				}	
			}
		}

		// We have either finished or filled a wavefront
		for( int link = 0; link < linksForWavefront.size(); ++link )
		{
			int v0 = linkData.getVertexPair( linksForWavefront[link] ).vertex0;
			int v1 = linkData.getVertexPair( linksForWavefront[link] ).vertex1;
			insertUniqueAndOrderedIntoVector( vertexSet, v0 );
			insertUniqueAndOrderedIntoVector( vertexSet, v1 );
		}
		// Iterate over links mapped to the wave and batch those
		// We can run a batch on each cycle trivially
		
		batchesWithinWaves.resize( batchesWithinWaves.size() + 1 );
		btAlignedObjectArray < btAlignedObjectArray <int> > &batchesWithinWave( batchesWithinWaves[batchesWithinWaves.size()-1] );
		

		for( int link = 0; link < linksForWavefront.size(); ++link )
		{
			int linkIndex = linksForWavefront[link];
			btSoftBodyLinkData::LinkNodePair vertices = linkData.getVertexPair( linkIndex );
			
			int batch = 0;
			bool placed = false;
			while( batch < batchesWithinWave.size() && !placed )
			{
				bool foundSharedVertex = false;
				if( batchesWithinWave[batch].size() >= wavefrontSize )
				{
					// If we have already filled this batch, move on to another
					foundSharedVertex = true;
				} else {
					for( int link2 = 0; link2 < batchesWithinWave[batch].size(); ++link2 )
					{
						btSoftBodyLinkData::LinkNodePair vertices2 = linkData.getVertexPair( (batchesWithinWave[batch])[link2] );

						if( vertices.vertex0 == vertices2.vertex0 ||
							vertices.vertex1 == vertices2.vertex0 ||
							vertices.vertex0 == vertices2.vertex1 ||
							vertices.vertex1 == vertices2.vertex1 )
						{
							foundSharedVertex = true;
							break;
						}
					}
				}
				if( !foundSharedVertex )
				{
					batchesWithinWave[batch].push_back( linkIndex );
					placed = true;
				} else {
					++batch;
				}
			}
			if( batch == batchesWithinWave.size() && !placed )
			{
				batchesWithinWave.resize( batch + 1 );
				batchesWithinWave[batch].push_back( linkIndex );
			}
		}
		
	}

}

void btSoftBodyLinkDataDX11SIMDAware::generateBatches()
{
	btAlignedObjectArray < btAlignedObjectArray <int> > linksForWavefronts;
	btAlignedObjectArray < btAlignedObjectArray <int> > wavefrontBatches;
	btAlignedObjectArray< btAlignedObjectArray < btAlignedObjectArray <int> > > batchesWithinWaves;
	btAlignedObjectArray< btAlignedObjectArray< int > > verticesForWavefronts; // wavefronts, vertices in wavefront as an ordered set

	// Group the links into wavefronts
	computeBatchingIntoWavefronts( *this, m_wavefrontSize, m_linksPerWorkItem, m_maxLinksPerWavefront, linksForWavefronts, batchesWithinWaves, verticesForWavefronts );


	// Batch the wavefronts
	generateBatchesOfWavefronts( linksForWavefronts, *this, m_maxVertex, wavefrontBatches );

	m_numWavefronts = linksForWavefronts.size();

	// At this point we have a description of which links we need to process in each wavefront

	// First correctly fill the batch ranges vector
	int numBatches = wavefrontBatches.size();
	m_wavefrontBatchStartLengths.resize(0);
	int prefixSum = 0;
	for( int batchIndex = 0; batchIndex < numBatches; ++batchIndex )
	{
		int wavesInBatch = wavefrontBatches[batchIndex].size();
		int nextPrefixSum = prefixSum + wavesInBatch;
		m_wavefrontBatchStartLengths.push_back( BatchPair( prefixSum, nextPrefixSum - prefixSum ) );

		prefixSum += wavesInBatch;
	}
	
	// Also find max number of batches within a wave
	m_maxBatchesWithinWave = 0;
	m_maxVerticesWithinWave = 0;
	m_numBatchesAndVerticesWithinWaves.resize( m_numWavefronts );
	for( int waveIndex = 0; waveIndex < m_numWavefronts; ++waveIndex )
	{
		// See if the number of batches in this wave is greater than the current maxium
		int batchesInCurrentWave = batchesWithinWaves[waveIndex].size();
		int verticesInCurrentWave = verticesForWavefronts[waveIndex].size();
		m_maxBatchesWithinWave = btMax( batchesInCurrentWave, m_maxBatchesWithinWave );
		m_maxVerticesWithinWave = btMax( verticesInCurrentWave, m_maxVerticesWithinWave );
	}
	
	// Add padding values both for alignment and as dudd addresses within LDS to compute junk rather than branch around
	m_maxVerticesWithinWave = 16*((m_maxVerticesWithinWave/16)+2);

	// Now we know the maximum number of vertices per-wave we can resize the global vertices array
	m_wavefrontVerticesGlobalAddresses.resize( m_maxVerticesWithinWave * m_numWavefronts );

	// Grab backup copies of all the link data arrays for the sorting process
	btAlignedObjectArray<btSoftBodyLinkData::LinkNodePair>				m_links_Backup(m_links);
	btAlignedObjectArray<float>											m_linkStrength_Backup(m_linkStrength);
	btAlignedObjectArray<float>											m_linksMassLSC_Backup(m_linksMassLSC);
	btAlignedObjectArray<float>											m_linksRestLengthSquared_Backup(m_linksRestLengthSquared);
	//btAlignedObjectArray<Vectormath::Aos::Vector3>						m_linksCLength_Backup(m_linksCLength);
	//btAlignedObjectArray<float>											m_linksLengthRatio_Backup(m_linksLengthRatio);
	btAlignedObjectArray<float>											m_linksRestLength_Backup(m_linksRestLength);
	btAlignedObjectArray<float>											m_linksMaterialLinearStiffnessCoefficient_Backup(m_linksMaterialLinearStiffnessCoefficient);

	// Resize to a wavefront sized batch per batch per wave so we get perfectly coherent memory accesses.
	m_links.resize( m_maxBatchesWithinWave * m_wavefrontSize * m_numWavefronts );
	m_linkVerticesLocalAddresses.resize( m_maxBatchesWithinWave * m_wavefrontSize * m_numWavefronts );
	m_linkStrength.resize( m_maxBatchesWithinWave * m_wavefrontSize * m_numWavefronts );
	m_linksMassLSC.resize( m_maxBatchesWithinWave * m_wavefrontSize * m_numWavefronts );
	m_linksRestLengthSquared.resize( m_maxBatchesWithinWave * m_wavefrontSize * m_numWavefronts );
	m_linksRestLength.resize( m_maxBatchesWithinWave * m_wavefrontSize * m_numWavefronts );
	m_linksMaterialLinearStiffnessCoefficient.resize( m_maxBatchesWithinWave * m_wavefrontSize * m_numWavefronts );	
		
	// Then re-order links into wavefront blocks

	// Total number of wavefronts moved. This will decide the ordering of sorted wavefronts.
	int wavefrontCount = 0;

	// Iterate over batches of wavefronts, then wavefronts in the batch
	for( int batchIndex = 0; batchIndex < numBatches; ++batchIndex )
	{
		btAlignedObjectArray <int> &batch( wavefrontBatches[batchIndex] );
		int wavefrontsInBatch = batch.size();

		
		for( int wavefrontIndex = 0; wavefrontIndex < wavefrontsInBatch; ++wavefrontIndex )
		{	

			int originalWavefrontIndex = batch[wavefrontIndex];
			btAlignedObjectArray< int > &wavefrontVertices( verticesForWavefronts[originalWavefrontIndex] );
			int verticesUsedByWavefront = wavefrontVertices.size();

			// Copy the set of vertices into the correctly structured array for use on the device
			// Fill the non-vertices with -1s
			// so we can mask out those reads
			for( int vertex = 0; vertex < verticesUsedByWavefront; ++vertex )
			{
				m_wavefrontVerticesGlobalAddresses[m_maxVerticesWithinWave * wavefrontCount + vertex] = wavefrontVertices[vertex];
			}
			for( int vertex = verticesUsedByWavefront; vertex < m_maxVerticesWithinWave; ++vertex )
			{
				m_wavefrontVerticesGlobalAddresses[m_maxVerticesWithinWave * wavefrontCount + vertex] = -1;
			}

			// Obtain the set of batches within the current wavefront
			btAlignedObjectArray < btAlignedObjectArray <int> > &batchesWithinWavefront( batchesWithinWaves[originalWavefrontIndex] );
			// Set the size of the batches for use in the solver, correctly ordered
			NumBatchesVerticesPair batchesAndVertices;
			batchesAndVertices.numBatches = batchesWithinWavefront.size();
			batchesAndVertices.numVertices = verticesUsedByWavefront;
			m_numBatchesAndVerticesWithinWaves[wavefrontCount] = batchesAndVertices;
			

			// Now iterate over batches within the wavefront to structure the links correctly
			for( int wavefrontBatch = 0; wavefrontBatch < batchesWithinWavefront.size(); ++wavefrontBatch )
			{
				btAlignedObjectArray <int> &linksInBatch( batchesWithinWavefront[wavefrontBatch] );
				int wavefrontBatchSize = linksInBatch.size();

				int batchAddressInTarget = m_maxBatchesWithinWave * m_wavefrontSize * wavefrontCount + m_wavefrontSize * wavefrontBatch;

				for( int linkIndex = 0; linkIndex < wavefrontBatchSize; ++linkIndex )
				{
					int originalLinkAddress = linksInBatch[linkIndex];
					// Reorder simple arrays trivially
					m_links[batchAddressInTarget + linkIndex] = m_links_Backup[originalLinkAddress];
					m_linkStrength[batchAddressInTarget + linkIndex] = m_linkStrength_Backup[originalLinkAddress];
					m_linksMassLSC[batchAddressInTarget + linkIndex] = m_linksMassLSC_Backup[originalLinkAddress];
					m_linksRestLengthSquared[batchAddressInTarget + linkIndex] = m_linksRestLengthSquared_Backup[originalLinkAddress];
					m_linksRestLength[batchAddressInTarget + linkIndex] = m_linksRestLength_Backup[originalLinkAddress];
					m_linksMaterialLinearStiffnessCoefficient[batchAddressInTarget + linkIndex] = m_linksMaterialLinearStiffnessCoefficient_Backup[originalLinkAddress];

					// The local address is more complicated. We need to work out where a given vertex will end up
					// by searching the set of vertices for this link and using the index as the local address
					btSoftBodyLinkData::LinkNodePair localPair;
					btSoftBodyLinkData::LinkNodePair globalPair = m_links[batchAddressInTarget + linkIndex];
					localPair.vertex0 = wavefrontVertices.findLinearSearch( globalPair.vertex0 );
					localPair.vertex1 = wavefrontVertices.findLinearSearch( globalPair.vertex1 );
					m_linkVerticesLocalAddresses[batchAddressInTarget + linkIndex] = localPair;
				}
				for( int linkIndex = wavefrontBatchSize; linkIndex < m_wavefrontSize; ++linkIndex )
				{
					// Put 0s into these arrays for padding for cleanliness
					m_links[batchAddressInTarget + linkIndex] = btSoftBodyLinkData::LinkNodePair(0, 0);
					m_linkStrength[batchAddressInTarget + linkIndex] = 0.f;
					m_linksMassLSC[batchAddressInTarget + linkIndex] = 0.f;
					m_linksRestLengthSquared[batchAddressInTarget + linkIndex] = 0.f;
					m_linksRestLength[batchAddressInTarget + linkIndex] = 0.f;
					m_linksMaterialLinearStiffnessCoefficient[batchAddressInTarget + linkIndex] = 0.f;


					// For local addresses of junk data choose a set of addresses just above the range of valid ones 
					// and cycling tyhrough % 16 so that we don't have bank conficts between all dud addresses
					// The valid addresses will do scatter and gather in the valid range, the junk ones should happily work
					// off the end of that range so we need no control
					btSoftBodyLinkData::LinkNodePair localPair;
					localPair.vertex0 = verticesUsedByWavefront + (linkIndex % 16);
					localPair.vertex1 = verticesUsedByWavefront + (linkIndex % 16);
					m_linkVerticesLocalAddresses[batchAddressInTarget + linkIndex] = localPair;
				}

			}

			
			wavefrontCount++;
		}

	
	}

} // void btSoftBodyLinkDataDX11SIMDAware::generateBatches()
