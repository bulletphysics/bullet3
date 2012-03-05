/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "../../../include/physics_effects/base_level/sort/pfx_sort.h"

namespace sce {
namespace PhysicsEffects {


#define Key(a) pfxGetKey(a)

///////////////////////////////////////////////////////////////////////////////
// Merge Sort



template <class SortData>
SCE_PFX_FORCE_INLINE
void pfxCompareAndSwap(SortData &d1,SortData &d2)
{
if(Key(d1) > Key(d2)) {
	SortData tmp = d1;
	d1 = d2;
	d2 = tmp;
}
}

template <class SortData>
SCE_PFX_FORCE_INLINE
void pfxMergeTwoBuffers11(SortData* d1,int n1,SortData* d2,int n2)
{
(void)n1,(void)n2,(void)d2;
pfxCompareAndSwap(d1[0],d1[1]);
}

template <class SortData>
SCE_PFX_FORCE_INLINE
void pfxMergeTwoBuffers12(SortData* d1,int n1,SortData* d2,int n2)
{
(void)n1,(void)d2,(void)n2;
pfxCompareAndSwap(d1[0],d1[1]);
pfxCompareAndSwap(d1[1],d1[2]);
}

template <class SortData>
SCE_PFX_FORCE_INLINE
void pfxMergeTwoBuffers22(SortData* d1,int n1,SortData* d2,int n2)
{
(void)n1,(void)d2,(void)n2;
pfxCompareAndSwap(d1[0],d1[2]);
pfxCompareAndSwap(d1[1],d1[3]);
pfxCompareAndSwap(d1[1],d1[2]);
}

template <class SortData>
SCE_PFX_FORCE_INLINE
void pfxMergeTwoBuffers23(SortData* d1,int n1,SortData* d2,int n2)
{
(void)n1,(void)d2,(void)n2;
pfxCompareAndSwap(d1[0],d1[2]);
pfxCompareAndSwap(d1[1],d1[4]);
pfxCompareAndSwap(d1[2],d1[3]);
pfxCompareAndSwap(d1[1],d1[2]);
pfxCompareAndSwap(d1[2],d1[3]);
}

template <class SortData>
SCE_PFX_FORCE_INLINE
void pfxMergeTwoBuffers33(SortData* d1,int n1,SortData* d2,int n2)
{
(void)n1,(void)d2,(void)n2;
pfxCompareAndSwap(d1[0],d1[3]);
pfxCompareAndSwap(d1[2],d1[5]);
pfxCompareAndSwap(d1[1],d1[2]);
pfxCompareAndSwap(d1[3],d1[4]);
pfxCompareAndSwap(d1[1],d1[3]);
pfxCompareAndSwap(d1[2],d1[4]);
pfxCompareAndSwap(d1[2],d1[3]);
}

template <class SortData>
void pfxMergeTwoBuffers(SortData* d1,unsigned int n1,SortData* d2,unsigned int n2,SortData *buff)
{
unsigned int i=0,j=0;

while(i<n1&&j<n2) {
	if(Key(d1[i]) < Key(d2[j])) {
		buff[i+j] = d1[i++];
	}
	else {
		buff[i+j] = d2[j++];
	}
}

if(i<n1) {
	while(i<n1) {
		buff[i+j] = d1[i++];
	}
}
else if(j<n2) {
	while(j<n2) {
		buff[i+j] = d2[j++];
	}
}

for(unsigned int k=0;k<(n1+n2);k++) {
	d1[k] = buff[k];
}
}

template <class SortData>
void pfxMergeSort(SortData *d,SortData *buff,int n)
{
int n1 = n>>1;
int n2 = n-n1;
if(n1>1) pfxMergeSort(d,buff,n1);
if(n2>1) pfxMergeSort(d+n1,buff,n2);

int nadd = n1+n2;

if(nadd==2) {
	pfxMergeTwoBuffers11(d,n1,d+n1,n2);
}
else if(nadd==3) {
	pfxMergeTwoBuffers12(d,n1,d+n1,n2);
}
else if(nadd==4) {
	pfxMergeTwoBuffers22(d,n1,d+n1,n2);
}
else if(nadd==5) {
	pfxMergeTwoBuffers23(d,n1,d+n1,n2);
}
else if(nadd==6) {
	pfxMergeTwoBuffers33(d,n1,d+n1,n2);
}
else {
	pfxMergeTwoBuffers(d,n1,d+n1,n2,buff);
}
}


///////////////////////////////////////////////////////////////////////////////
// Bitonic Sort


template <class SortData>
void pfxBitonicMerge(SortData *d,unsigned int n,int dir)
{
if(n > 1) {
	unsigned int k = n>>1;
	for(unsigned int i=0;i<k;i++) {
		if(dir==0 && Key(d[i]) > Key(d[i+k])) {
			SortData t = d[i+k];
			d[i+k] = d[i];
			d[i] = t;
		}
		if(dir==1 && Key(d[i]) < Key(d[i+k])) {
			SortData t = d[i+k];
			d[i+k] = d[i];
			d[i] = t;
		}
	}
	pfxBitonicMerge(d,k,dir);
	pfxBitonicMerge(d+k,k,dir);
}
}

template <class SortData>
void pfxBitonicSortInternal(SortData *d,unsigned int n,unsigned int dir=0)
{
if(n > 1) {
	unsigned int k = n>>1;
	pfxBitonicSortInternal(d,k,0);
	pfxBitonicSortInternal(d+k,k,1);
	pfxBitonicMerge(d,n,dir);
}
}

template <class SortData>
void pfxBitonicSort(SortData *d,SortData *buff,unsigned int n)
{
pfxBitonicSortInternal(d,n,0);
memcpy(buff,d,sizeof(SortData)*n);
}


///////////////////////////////////////////////////////////////////////////////
// Hybrid Sort (Bitonic + Merge)

struct PfxDiv2n {
	unsigned int id;
	unsigned int num;

	PfxDiv2n() {}

	PfxDiv2n(unsigned int i,unsigned int n)
	{
		id = i;
		num = n;
	}
};

template <class SortData>
void pfxHybridSort(SortData *data,SortData *buff,unsigned int n)
{
	unsigned int numDiv = 0;
	PfxDiv2n divData[32];

	unsigned int id = 0;
	unsigned int rest = n;
	unsigned int mask = 0x01;

	while(rest>0) {
		if((mask&n)>0) {
			divData[numDiv++] = PfxDiv2n(id,mask);
			pfxBitonicSort(data+id,buff+id,mask);
			rest ^= mask;
			id += mask;
		}
		mask <<= 1;
	}

	if(numDiv==1) {
		for(unsigned int i=0;i<n;i++) {
			data[i] = buff[i];
		}
	}
	else {
		for(unsigned int i=1;i<numDiv;i++) {
			pfxMergeTwoBuffers(
				buff+divData[i-1].id,divData[i-1].num,
				buff+divData[i].id,divData[i].num,
				data);
			divData[i].id = divData[i-1].id;
			divData[i].num += divData[i-1].num;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Single Sort

void pfxSort(PfxSortData16 *data,PfxSortData16 *buff,unsigned int n)
{
pfxMergeSort(data,buff,n);
}

void pfxSort(PfxSortData32 *data,PfxSortData32 *buff,unsigned int n)
{
pfxMergeSort(data,buff,n);
}

} //namespace PhysicsEffects
} //namespace sce
