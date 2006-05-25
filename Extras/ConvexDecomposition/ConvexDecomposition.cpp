#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <algorithm>
#include <vector>

/*----------------------------------------------------------------------
		Copyright (c) 2004 Open Dynamics Framework Group
					www.physicstools.org
		All rights reserved.

		Redistribution and use in source and binary forms, with or without modification, are permitted provided
		that the following conditions are met:

		Redistributions of source code must retain the above copyright notice, this list of conditions
		and the following disclaimer.

		Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

		Neither the name of the Open Dynamics Framework Group nor the names of its contributors may
		be used to endorse or promote products derived from this software without specific prior written permission.

		THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES,
		INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
		DISCLAIMED. IN NO EVENT SHALL THE INTEL OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
		EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
		LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
		IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
		THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------*/

// http://codesuppository.blogspot.com
//
// mailto: jratcliff@infiniplex.net
//
// http://www.amillionpixels.us
//

#include "ConvexDecomposition.h"
#include "cd_vector.h"
#include "cd_hull.h"
#include "bestfit.h"
#include "PlaneTri.h"
#include "vlookup.h"
#include "splitplane.h"
#include "meshvolume.h"
#include "concavity.h"
#include "bestfitobb.h"
#include "float_math.h"
#include "fitsphere.h"

#define SHOW_MESH 0
#define MAKE_MESH 1

static unsigned int MAXDEPTH=8;
static float        CONCAVE_PERCENT=1.0f;
static float        MERGE_PERCENT=2.0f;


using namespace ConvexDecomposition;

typedef std::vector< unsigned int > UintVector;

namespace ConvexDecomposition
{

class FaceTri
{
public:
	FaceTri(void) { };
  FaceTri(const float *vertices,unsigned int i1,unsigned int i2,unsigned int i3)
  {
  	mP1.Set( &vertices[i1*3] );
  	mP2.Set( &vertices[i2*3] );
  	mP3.Set( &vertices[i3*3] );
  }

  Vector3d	mP1;
  Vector3d	mP2;
  Vector3d	mP3;
  Vector3d mNormal;

};


void addTri(VertexLookup vl,UintVector &list,const Vector3d &p1,const Vector3d &p2,const Vector3d &p3)
{
  unsigned int i1 = Vl_getIndex(vl, p1.Ptr() );
  unsigned int i2 = Vl_getIndex(vl, p2.Ptr() );
  unsigned int i3 = Vl_getIndex(vl, p3.Ptr() );

  // do *not* process degenerate triangles!

  if ( i1 != i2 && i1 != i3 && i2 != i3 )
  {
    list.push_back(i1);
    list.push_back(i2);
    list.push_back(i3);
  }
}


void doConvexDecomposition(unsigned int           vcount,
                                const float           *vertices,
                                unsigned int           tcount,
                                const unsigned int    *indices,
                                ConvexDecompInterface *callback,
                                float                  masterVolume,
                                unsigned int           depth)

{

  float plane[4];

  bool split = false;


  if ( depth < MAXDEPTH )
  {

		float volume;
		float c = computeConcavity( vcount, vertices, tcount, indices, callback, plane, volume );

    if ( depth == 0 )
    {
      masterVolume = volume;
    }

		float percent = (c*100.0f)/masterVolume;

		if ( percent > CONCAVE_PERCENT ) // if great than 5% of the total volume is concave, go ahead and keep splitting.
		{
      split = true;
    }

  }

  if ( depth >= MAXDEPTH || !split )
  {

#if 1

    HullResult result;
    HullLibrary hl;
    HullDesc   desc;

  	desc.SetHullFlag(QF_TRIANGLES);

    desc.mVcount       = vcount;
    desc.mVertices     = vertices;
    desc.mVertexStride = sizeof(float)*3;

    HullError ret = hl.CreateConvexHull(desc,result);

    if ( ret == QE_OK )
    {

			ConvexResult r(result.mNumOutputVertices, result.mOutputVertices, result.mNumFaces, result.mIndices);


			callback->ConvexDecompResult(r);
    }


#else

		static unsigned int colors[8] =
		{
			0xFF0000,
		  0x00FF00,
			0x0000FF,
			0xFFFF00,
			0x00FFFF,
			0xFF00FF,
			0xFFFFFF,
			0xFF8040
		};

		static int count = 0;

		count++;

		if ( count == 8 ) count = 0;

		assert( count >= 0 && count < 8 );

		unsigned int color = colors[count];

    const unsigned int *source = indices;

    for (unsigned int i=0; i<tcount; i++)
    {

      unsigned int i1 = *source++;
      unsigned int i2 = *source++;
      unsigned int i3 = *source++;

			FaceTri t(vertices, i1, i2, i3 );

      callback->ConvexDebugTri( t.mP1.Ptr(), t.mP2.Ptr(), t.mP3.Ptr(), color );

    }
#endif

    return;

  }

  UintVector ifront;
  UintVector iback;

  VertexLookup vfront = Vl_createVertexLookup();
  VertexLookup vback  = Vl_createVertexLookup();


	bool showmesh = false;
  #if SHOW_MESH
  showmesh = true;
  #endif

	if ( 0 )
	{
		showmesh = true;
	  for (float x=-1; x<1; x+=0.10f)
		{
		  for (float y=0; y<1; y+=0.10f)
			{
			  for (float z=-1; z<1; z+=0.04f)
				{
				  float d = x*plane[0] + y*plane[1] + z*plane[2] + plane[3];
					Vector3d p(x,y,z);
				  if ( d >= 0 )
					  callback->ConvexDebugPoint(p.Ptr(), 0.02f, 0x00FF00);
				  else
					  callback->ConvexDebugPoint(p.Ptr(), 0.02f, 0xFF0000);
				}
			}
		}
	}

	if ( 1 )
	{
		// ok..now we are going to 'split' all of the input triangles against this plane!
		const unsigned int *source = indices;
		for (unsigned int i=0; i<tcount; i++)
		{
			unsigned int i1 = *source++;
			unsigned int i2 = *source++;
			unsigned int i3 = *source++;

			FaceTri t(vertices, i1, i2, i3 );

			Vector3d front[4];
			Vector3d back[4];

			unsigned int fcount=0;
			unsigned int bcount=0;

			PlaneTriResult result;

		  result = planeTriIntersection(plane,t.mP1.Ptr(),sizeof(Vector3d),0.00001f,front[0].Ptr(),fcount,back[0].Ptr(),bcount );

			if( fcount > 4 || bcount > 4 )
			{
		    result = planeTriIntersection(plane,t.mP1.Ptr(),sizeof(Vector3d),0.00001f,front[0].Ptr(),fcount,back[0].Ptr(),bcount );
			}

			switch ( result )
			{
				case PTR_FRONT:

					assert( fcount == 3 );

          if ( showmesh )
            callback->ConvexDebugTri( front[0].Ptr(), front[1].Ptr(), front[2].Ptr(), 0x00FF00 );

          #if MAKE_MESH

          addTri( vfront, ifront, front[0], front[1], front[2] );


          #endif

					break;
				case PTR_BACK:
					assert( bcount == 3 );

          if ( showmesh )
  					callback->ConvexDebugTri( back[0].Ptr(), back[1].Ptr(), back[2].Ptr(), 0xFFFF00 );

          #if MAKE_MESH

          addTri( vback, iback, back[0], back[1], back[2] );

          #endif

					break;
				case PTR_SPLIT:

					assert( fcount >= 3 && fcount <= 4);
					assert( bcount >= 3 && bcount <= 4);

          #if MAKE_MESH

          addTri( vfront, ifront, front[0], front[1], front[2] );
          addTri( vback, iback, back[0], back[1], back[2] );


          if ( fcount == 4 )
          {
            addTri( vfront, ifront, front[0], front[2], front[3] );
          }

          if ( bcount == 4  )
          {
            addTri( vback, iback, back[0], back[2], back[3] );
          }

          #endif

          if ( showmesh )
          {
  					callback->ConvexDebugTri( front[0].Ptr(), front[1].Ptr(), front[2].Ptr(), 0x00D000 );
  					callback->ConvexDebugTri( back[0].Ptr(), back[1].Ptr(), back[2].Ptr(), 0xD0D000 );

  					if ( fcount == 4 )
  					{
  						callback->ConvexDebugTri( front[0].Ptr(), front[2].Ptr(), front[3].Ptr(), 0x00D000 );
  					}
  					if ( bcount == 4 )
  					{
  						callback->ConvexDebugTri( back[0].Ptr(), back[2].Ptr(), back[3].Ptr(), 0xD0D000 );
  					}
  				}

					break;
			}
		}

		unsigned int fsize = ifront.size()/3;
		unsigned int bsize = iback.size()/3;

    // ok... here we recursively call
    if ( ifront.size() )
    {
      unsigned int vcount   = Vl_getVcount(vfront);
      const float *vertices = Vl_getVertices(vfront);
      unsigned int tcount   = ifront.size()/3;

      doConvexDecomposition(vcount, vertices, tcount, &ifront[0], callback, masterVolume, depth+1);

    }

    ifront.clear();

    Vl_releaseVertexLookup(vfront);

    if ( iback.size() )
    {
      unsigned int vcount   = Vl_getVcount(vback);
      const float *vertices = Vl_getVertices(vback);
      unsigned int tcount   = iback.size()/3;

      doConvexDecomposition(vcount, vertices, tcount, &iback[0], callback, masterVolume, depth+1);

    }

    iback.clear();
    Vl_releaseVertexLookup(vback);

	}
}

class CHull
{
public:
  CHull(const ConvexResult &result)
  {
    mResult = new ConvexResult(result);
    mVolume = computeMeshVolume( result.mHullVertices, result.mHullTcount, result.mHullIndices );

    mDiagonal = getBoundingRegion( result.mHullVcount, result.mHullVertices, sizeof(float)*3, mMin, mMax );

    float dx = mMax[0] - mMin[0];
    float dy = mMax[1] - mMin[1];
    float dz = mMax[2] - mMin[2];

    dx*=0.1f; // inflate 1/10th on each edge
    dy*=0.1f; // inflate 1/10th on each edge
    dz*=0.1f; // inflate 1/10th on each edge

    mMin[0]-=dx;
    mMin[1]-=dy;
    mMin[2]-=dz;

    mMax[0]+=dx;
    mMax[1]+=dy;
    mMax[2]+=dz;


  }

  ~CHull(void)
  {
    delete mResult;
  }

  bool overlap(const CHull &h) const
  {
    return overlapAABB(mMin,mMax, h.mMin, h.mMax );
  }

  float          mMin[3];
  float          mMax[3];
	float          mVolume;
  float          mDiagonal; // long edge..
  ConvexResult  *mResult;
};

// Usage: std::sort( list.begin(), list.end(), StringSortRef() );
class CHullSort
{
	public:

	 bool operator()(const CHull *a,const CHull *b) const
	 {
		 return a->mVolume < b->mVolume;
	 }
};


typedef std::vector< CHull * > CHullVector;


class ConvexBuilder : public ConvexDecompInterface
{
public:
  ConvexBuilder(ConvexDecompInterface *callback)
  {
    mCallback = callback;
  };

  ~ConvexBuilder(void)
  {
    CHullVector::iterator i;
    for (i=mChulls.begin(); i!=mChulls.end(); ++i)
    {
      CHull *cr = (*i);
      delete cr;
    }
  }

	bool isDuplicate(unsigned int i1,unsigned int i2,unsigned int i3,
		               unsigned int ci1,unsigned int ci2,unsigned int ci3)
	{
		unsigned int dcount = 0;

		assert( i1 != i2 && i1 != i3 && i2 != i3 );
		assert( ci1 != ci2 && ci1 != ci3 && ci2 != ci3 );

		if ( i1 == ci1 || i1 == ci2 || i1 == ci3 ) dcount++;
		if ( i2 == ci1 || i2 == ci2 || i2 == ci3 ) dcount++;
		if ( i3 == ci1 || i3 == ci2 || i3 == ci3 ) dcount++;

		return dcount == 3;
	}

	void getMesh(const ConvexResult &cr,VertexLookup vc,UintVector &indices)
	{
		unsigned int *src = cr.mHullIndices;

		for (unsigned int i=0; i<cr.mHullTcount; i++)
		{
			unsigned int i1 = *src++;
			unsigned int i2 = *src++;
			unsigned int i3 = *src++;

			const float *p1 = &cr.mHullVertices[i1*3];
			const float *p2 = &cr.mHullVertices[i2*3];
			const float *p3 = &cr.mHullVertices[i3*3];

			i1 = Vl_getIndex(vc,p1);
			i2 = Vl_getIndex(vc,p2);
			i3 = Vl_getIndex(vc,p3);

#if 0
			bool duplicate = false;

			unsigned int tcount = indices.size()/3;
			for (unsigned int j=0; j<tcount; j++)
			{
				unsigned int ci1 = indices[j*3+0];
				unsigned int ci2 = indices[j*3+1];
				unsigned int ci3 = indices[j*3+2];
				if ( isDuplicate(i1,i2,i3, ci1, ci2, ci3 ) )
				{
					duplicate = true;
					break;
				}
			}

			if ( !duplicate )
			{
			  indices.push_back(i1);
			  indices.push_back(i2);
			  indices.push_back(i3);
			}
#endif

		}
	}

	CHull * canMerge(CHull *a,CHull *b)
	{

    if ( !a->overlap(*b) ) return 0; // if their AABB's (with a little slop) don't overlap, then return.

		CHull *ret = 0;

		// ok..we are going to combine both meshes into a single mesh
		// and then we are going to compute the concavity...

    VertexLookup vc = Vl_createVertexLookup();

		UintVector indices;

    getMesh( *a->mResult, vc, indices );
    getMesh( *b->mResult, vc, indices );

		unsigned int vcount = Vl_getVcount(vc);
		const float *vertices = Vl_getVertices(vc);
		unsigned int tcount = indices.size()/3;
		unsigned int *idx   = &indices[0];

    HullResult hresult;
    HullLibrary hl;
    HullDesc   desc;

  	desc.SetHullFlag(QF_TRIANGLES);

    desc.mVcount       = vcount;
    desc.mVertices     = vertices;
    desc.mVertexStride = sizeof(float)*3;

    HullError hret = hl.CreateConvexHull(desc,hresult);

    if ( hret == QE_OK )
    {

      float combineVolume  = computeMeshVolume( hresult.mOutputVertices, hresult.mNumFaces, hresult.mIndices );
			float sumVolume      = a->mVolume + b->mVolume;

      float percent = (sumVolume*100) / combineVolume;
      if ( percent >= (100.0f-MERGE_PERCENT) )
      {
  			ConvexResult cr(hresult.mNumOutputVertices, hresult.mOutputVertices, hresult.mNumFaces, hresult.mIndices);
    		ret = new CHull(cr);
    	}
		}


		Vl_releaseVertexLookup(vc);

		return ret;
	}

  bool combineHulls(void)
  {

  	bool combine = false;

		sortChulls(mChulls); // sort the convex hulls, largest volume to least...

		CHullVector output; // the output hulls...


    CHullVector::iterator i;

    for (i=mChulls.begin(); i!=mChulls.end() && !combine; ++i)
    {
      CHull *cr = (*i);

      CHullVector::iterator j;
      for (j=mChulls.begin(); j!=mChulls.end(); ++j)
      {
        CHull *match = (*j);

        if ( cr != match ) // don't try to merge a hull with itself, that be stoopid
        {

					CHull *merge = canMerge(cr,match); // if we can merge these two....

					if ( merge )
					{

						output.push_back(merge);


						++i;
						while ( i != mChulls.end() )
						{
							CHull *cr = (*i);
							if ( cr != match )
							{
  							output.push_back(cr);
  						}
							i++;
						}

						delete cr;
						delete match;
						combine = true;
						break;
					}
        }
      }

      if ( combine )
      {
      	break;
      }
      else
      {
      	output.push_back(cr);
      }

    }

		if ( combine )
		{
			mChulls.clear();
			mChulls = output;
			output.clear();
		}


    return combine;
  }

  unsigned int process(const DecompDesc &desc)
  {

  	unsigned int ret = 0;

		MAXDEPTH        = desc.mDepth;
		CONCAVE_PERCENT = desc.mCpercent;
		MERGE_PERCENT   = desc.mPpercent;


    doConvexDecomposition(desc.mVcount, desc.mVertices, desc.mTcount, desc.mIndices,this,0,0);


		while ( combineHulls() ); // keep combinging hulls until I can't combine any more...

    CHullVector::iterator i;
    for (i=mChulls.begin(); i!=mChulls.end(); ++i)
    {
      CHull *cr = (*i);

			// before we hand it back to the application, we need to regenerate the hull based on the
			// limits given by the user.

			const ConvexResult &c = *cr->mResult; // the high resolution hull...

      HullResult result;
      HullLibrary hl;
      HullDesc   hdesc;

    	hdesc.SetHullFlag(QF_TRIANGLES);

      hdesc.mVcount       = c.mHullVcount;
      hdesc.mVertices     = c.mHullVertices;
      hdesc.mVertexStride = sizeof(float)*3;
      hdesc.mMaxVertices  = desc.mMaxVertices; // maximum number of vertices allowed in the output

      if ( desc.mSkinWidth > 0 )
      {
      	hdesc.mSkinWidth = desc.mSkinWidth;
      	hdesc.SetHullFlag(QF_SKIN_WIDTH); // do skin width computation.
      }

      HullError ret = hl.CreateConvexHull(hdesc,result);

      if ( ret == QE_OK )
      {
  			ConvexResult r(result.mNumOutputVertices, result.mOutputVertices, result.mNumFaces, result.mIndices);

				r.mHullVolume = computeMeshVolume( result.mOutputVertices, result.mNumFaces, result.mIndices ); // the volume of the hull.

				// compute the best fit OBB
				computeBestFitOBB( result.mNumOutputVertices, result.mOutputVertices, sizeof(float)*3, r.mOBBSides, r.mOBBTransform );

				r.mOBBVolume = r.mOBBSides[0] * r.mOBBSides[1] *r.mOBBSides[2]; // compute the OBB volume.

				fm_getTranslation( r.mOBBTransform, r.mOBBCenter );      // get the translation component of the 4x4 matrix.

				fm_matrixToQuat( r.mOBBTransform, r.mOBBOrientation );   // extract the orientation as a quaternion.

				r.mSphereRadius = computeBoundingSphere( result.mNumOutputVertices, result.mOutputVertices, r.mSphereCenter );
				r.mSphereVolume = fm_sphereVolume( r.mSphereRadius );


        mCallback->ConvexDecompResult(r);
      }


      delete cr;
    }

		ret = mChulls.size();

    mChulls.clear();

    return ret;
  }


	virtual void ConvexDebugTri(const float *p1,const float *p2,const float *p3,unsigned int color)
  {
    mCallback->ConvexDebugTri(p1,p2,p3,color);
  }

  virtual void ConvexDebugOBB(const float *sides, const float *matrix,unsigned int color)
  {
    mCallback->ConvexDebugOBB(sides,matrix,color);
  }
	virtual void ConvexDebugPoint(const float *p,float dist,unsigned int color)
  {
    mCallback->ConvexDebugPoint(p,dist,color);
  }

  virtual void ConvexDebugBound(const float *bmin,const float *bmax,unsigned int color)
  {
    mCallback->ConvexDebugBound(bmin,bmax,color);
  }

  virtual void ConvexDecompResult(ConvexResult &result)
  {
    CHull *ch = new CHull(result);
		mChulls.push_back(ch);
  }

	void sortChulls(CHullVector &hulls)
	{
		std::sort( hulls.begin(), hulls.end(), CHullSort() );
	}

CHullVector     mChulls;
ConvexDecompInterface *mCallback;

};

unsigned int performConvexDecomposition(const DecompDesc &desc)
{
	unsigned int ret = 0;

  if ( desc.mCallback )
  {
    ConvexBuilder cb(desc.mCallback);

    ret = cb.process(desc);
  }

  return ret;
}



};
