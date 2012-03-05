@
@ Applied Research Associates Inc. (c)2011
@
@ Redistribution and use in source and binary forms,
@   with or without modification, are permitted provided that the
@   following conditions are met:
@    * Redistributions of source code must retain the above copyright
@      notice, this list of conditions and the following disclaimer.
@    * Redistributions in binary form must reproduce the above copyright
@      notice, this list of conditions and the following disclaimer in the
@      documentation and/or other materials provided with the distribution.
@    * Neither the name of the Applied Research Associates Inc nor the names
@      of its contributors may be used to endorse or promote products derived
@      from this software without specific prior written permission.
@
@   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
@   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
@   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
@   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
@   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
@   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
@   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
@   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
@   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
@   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
@   POSSIBILITY OF SUCH DAMAGE.
@
@----------------------------------------------------------------
@
@ This file contains ARM/NEON assembly versions of some vectormath
@ atomic functions. We have implemented here instead of inline assembly
@ because we have found gcc 4.4.3 to be too inconsistent and inadequate
@ to properly support either NEON intrinsics or inline assembly. (See
@ the inline assembly version of the vector3 dot product, which is
@ contained in vec_aos.h but compiled out, for an example of a simple
@ inline assembly function that wreaks havok if used.)
@
@ Note that the certain NEON registers must be preserved across
@ function calls according to the following document:
@
@ "Procedure Call Standard for the ARM? Architecture," ARM document
@ number ARM IHI 0042D, current through ABI release 2.08,
@ 16th October, 2009, section 5.1.2.1
@
@ The registers are: q4-q7 (and their double-word and single word
@ counterparts)
@
@ These functions preserve all non-scratch general purpose registers
@ as well as the ones listed, and so we do not need to have extra
@ code to do the register preservation
@
	.syntax unified
	.arch armv7-a
	.fpu neon
	.thumb
	.text
	.align 2

@----------------------------------------------------------------
@ pfxVector3DotProductNEON
@
@ Vector3 dot product, result stored directly to memory
@
@ Result stored in memory pointed to by r2. r2 must point to
@ memory sufficient to store two 32 bit floats, though only the
@ first value is of interest
@----------------------------------------------------------------
	.global	pfxVector3DotProductNEON
	.thumb_func
pfxVector3DotProductNEON:
	.fnstart
	vld1.32 {d0,d1}, [r1]		@ input <x2,y2,z2,?> = d0,d1\n\t"
	vld1.32 {d2,d3}, [r0]		@ input <x1,y1,z1,?> = d2,d3\n\t"
	vmul.f32 d4, d0, d2			@ d4 = <x1*x2,y1*y2>\n\t"
	vpadd.f32 d4, d4, d4		@ d4 = <x1*x2 + y1*y2, x1*x2 + y1*y2>\n\t"
	vmla.f32 s8, s2, s6			@ s8 = <x1*x2 + y1*y2 + z1*z2\n\t"
	vst1.32 {d4}, [r2]			@ save result to memory. supports double/quad word only. We only care about first word\n\t"
	bx	lr
	.fnend

@----------------------------------------------------------------
@ pfxVector4DotProductNEON
@
@ Vector4 dot product, result stored directly to memory
@
@ Result stored in memory pointed to by r2. r2 must point to
@ memory sufficient to store two 32 bit floats, though only the
@ first value is of interest
@----------------------------------------------------------------
	.global	pfxVector4DotProductNEON
	.thumb_func
pfxVector4DotProductNEON:
	.fnstart
	vld1.32	{d16,d17}, [r0]		@ input <x1,y1,z1,w1>
	vld1.32	{d18,d19}, [r1]		@ input <x2,y2,z2,w2>
	vmul.f32 d14, d16, d18		@ d14=<x1*x2,y1*y2>
	@ non-fused multiple accumulate
	vmla.f32 d14, d17, d19		@ d14=d14+<z1*z2,w1*w2>=<x1*x2+z1*z2,y1*y2+w1*w2>
	@ fused multiple accumulate - listed here for reference but we use vmla above
	@ instead since the fused version is not recognized by GNU assembler (as part
	@ of the gcc 4.4.3 Android distribution)
	@	vfma.f32 {d14}, d17, d19	@ d14=d14+<z1*z2,w1*w2>=<x1*x2+z1*z2,y1*y2+w1*w2>
	vpadd.f32 d14, d14, d14		@ add the two halves of d14 together, same result in each lane
	vst1.32	{d14}, [r2]
	bx	lr
	.fnend

@----------------------------------------------------------------
@ pfxVector3CrossProductNEON
@
@ Vector3 cross product, result stored directly to memory
@
@ Result stored in memory pointed to by r2. r2 must point to
@ memory sufficient to store four 32 bit floats, though only the
@ first 3 values are of interest
@----------------------------------------------------------------
	.global	pfxVector3CrossProductNEON
	.thumb_func
pfxVector3CrossProductNEON:
	.fnstart
	vld1.32	{d18,d19}, [r1]	@ input <x2,y2,z2,w2> = d18,d19
	vld1.32	{d16,d17}, [r0]	@ input <x1,y1,z1,w1> = d16,d17
	@ rearrange inputs into convenient order
	vtrn.32 d18,d19			@  q9 = <x2,z2,y2,w2> = d18,d19
	vrev64.32 d16,d16		@  q8 = <y1,x1,z1,w1> = d16,d17
	vrev64.32 d18,d18		@  q9 = <z2,x2,y2,w2> = d18,d19
	vtrn.32 d16,d17			@  q8 = <y1,z1,x1,w1> = d16,d17
	@ perform first half of cross product using rearranged inputs
	vmul.f32 q10, q8, q9	@ q10 = <y1*z2,z1*x2,x1*y2,w1*w2>
	@ rearrange inputs again
	vtrn.32 d18,d19			@  q9 = <z2,y2,x2,w2> = d18,d19
	vrev64.32 d16,d16		@  q8 = <z1,y1,x1,w1> = d16,d17
	vrev64.32 d18,d18		@  q9 = <y2,z2,x2,w2> = d18,d19
	vtrn.32 d16,d17			@  q8 = <z1,x1,y1,w1> = d16,d17
	@ perform last half of cross product using rearranged inputs
	vmls.f32 q10, q8, q9	@ q10 = <y1*z2-y2*z1,z1*x2-z2*x1,x1*y2-x2*y1,w1*w2-w2*w1>
	@ and store the result
	vst1.32	{q10}, [r2]
	bx	lr
	.fnend

@----------------------------------------------------------------
@ pfxMatrix3Matrix3ProductNEON
@
@ Matrix3 * Matrix3 product, result stored directly to memory
@
@ Result stored in memory pointed to by r2. r2 must point to
@ memory sufficient to store 12 32-bit floats. The reason for
@ 12 rather than 9 is that each column vector actually has
@ 4 32-bit floats rather than just 3....since NEON works with
@ double and quad vectors.
@
@ Note that, since the inputs are loaded into registers then
@ never used again, r2 can point to one of the inputs, e.g.,
@ result can be stored back out to one of the input memory
@ locations.
@----------------------------------------------------------------
	.global	pfxMatrix3Matrix3ProductNEON
	.thumb_func
pfxMatrix3Matrix3ProductNEON:
	.fnstart
	vld1.32 {d16-d19}, [r0]!	@ load first eight elements of matrix 0
	vld1.32 {d20-d21}, [r0]		@ load second eight elements of matrix 0
	vld1.32 {d0-d3}, [r1]!		@ load first eight elements of matrix 1
	vld1.32 {d4-d5}, [r1]		@ load second eight elements of matrix 1
	vmul.f32 q12, q8, d0[0]		@ rslt col0  = (mat0 col0) * (mat1 col0 elt0)
	vmul.f32 q13, q8, d2[0]		@ rslt col1  = (mat0 col0) * (mat1 col1 elt0)
	vmul.f32 q14, q8, d4[0]		@ rslt col2  = (mat0 col0) * (mat1 col2 elt0)
	vmla.f32 q12, q9, d0[1]		@ rslt col0 += (mat0 col1) * (mat1 col0 elt1)
	vmla.f32 q13, q9, d2[1]		@ rslt col1 += (mat0 col1) * (mat1 col1 elt1)
	vmla.f32 q14, q9, d4[1]		@ rslt col2 += (mat0 col1) * (mat1 col2 elt1)
	vmla.f32 q12, q10, d1[0]	@ rslt col0 += (mat0 col2) * (mat1 col0 elt2)
	vmla.f32 q13, q10, d3[0]	@ rslt col1 += (mat0 col2) * (mat1 col1 elt2)
	vmla.f32 q14, q10, d5[0]	@ rslt col2 += (mat0 col2) * (mat1 col2 elt2)
	vst1.32 {d24-d27}, [r2]!	@ store first eight elements of result (each column has an extra float as stored)
	vst1.32 {d28-d29}, [r2]		@ store last four elements of result (each column has an extra float as stored)
	bx	lr
	.fnend

@----------------------------------------------------------------
@ pfxMatrix4Matrix4ProductNEON
@
@ Matrix4 * Matrix4 product, result stored directly to memory
@
@ Result stored in memory pointed to by r2. r2 must point to
@ memory sufficient to store 16 32 bit floats.
@
@ Note that, since the inputs are loaded into registers then
@ never used again, r2 can point to one of the inputs, e.g.,
@ result can be stored back out to one of the input memory
@ locations.
@----------------------------------------------------------------
	.global	pfxMatrix4Matrix4ProductNEON
	.thumb_func
pfxMatrix4Matrix4ProductNEON:
	.fnstart
	vld1.32 {d16-d19}, [r0]!	@ load first eight elements of matrix 0
	vld1.32 {d20-d23}, [r0]		@ load second eight elements of matrix 0
	vld1.32 {d0-d3}, [r1]!		@ load first eight elements of matrix 1
	vld1.32 {d4-d7}, [r1]		@ load second eight elements of matrix 1
	vmul.f32 q12, q8, d0[0]		@ rslt col0  = (mat0 col0) * (mat1 col0 elt0)
	vmul.f32 q13, q8, d2[0]		@ rslt col1  = (mat0 col0) * (mat1 col1 elt0)
	vmul.f32 q14, q8, d4[0]		@ rslt col2  = (mat0 col0) * (mat1 col2 elt0)
	vmul.f32 q15, q8, d6[0]		@ rslt col3  = (mat0 col0) * (mat1 col3 elt0)
	vmla.f32 q12, q9, d0[1]		@ rslt col0 += (mat0 col1) * (mat1 col0 elt1)
	vmla.f32 q13, q9, d2[1]		@ rslt col1 += (mat0 col1) * (mat1 col1 elt1)
	vmla.f32 q14, q9, d4[1]		@ rslt col2 += (mat0 col1) * (mat1 col2 elt1)
	vmla.f32 q15, q9, d6[1]		@ rslt col3 += (mat0 col1) * (mat1 col3 elt1)
	vmla.f32 q12, q10, d1[0]	@ rslt col0 += (mat0 col2) * (mat1 col0 elt2)
	vmla.f32 q13, q10, d3[0]	@ rslt col1 += (mat0 col2) * (mat1 col1 elt2)
	vmla.f32 q14, q10, d5[0]	@ rslt col2 += (mat0 col2) * (mat1 col2 elt2)
	vmla.f32 q15, q10, d7[0]	@ rslt col3 += (mat0 col2) * (mat1 col2 elt2)
	vmla.f32 q12, q11, d1[1]	@ rslt col0 += (mat0 col3) * (mat1 col0 elt3)
	vmla.f32 q13, q11, d3[1]	@ rslt col1 += (mat0 col3) * (mat1 col1 elt3)
	vmla.f32 q14, q11, d5[1]	@ rslt col2 += (mat0 col3) * (mat1 col2 elt3)
	vmla.f32 q15, q11, d7[1]	@ rslt col3 += (mat0 col3) * (mat1 col3 elt3)
	vst1.32 {d24-d27}, [r2]!	@ store first eight elements of result
	vst1.32 {d28-d31}, [r2]		@ store second eight elements of result
	bx	lr
	.fnend

@----------------------------------------------------------------
@ pfxTransform3OrthoInverseNEON
@
@ Computes the ortho inverse of a Transform 3.
@
@ Result stored in memory pointed to by r1. r1 must point to
@ memory sufficient to store 16 32 bit floats.
@
@ Note that, since the inputs are loaded into registers then
@ never used again, r1 can point to one of the inputs, e.g.,
@ result can be stored back out to one of the input memory
@ locations.
@
@ Note that this expects the inputs to have 4 floats per row,
@ (to be consistent with NEON quad vector), and the last float
@ in each row should be 0.0 for the math to work out.
@----------------------------------------------------------------
	.global	pfxTransform3OrthoInverseNEON
	.thumb_func
pfxTransform3OrthoInverseNEON:
	.fnstart
	@ direct load the first column of the ortho inverse result
	vld1.f32 d0[0], [r0]!
	vld1.f32 d2[0], [r0]!
	vld1.f32 d4[0], [r0]!
	vld1.f32 d1[1], [r0]!

	@ direct load the second column of the ortho inverse result
	vld1.f32 d0[1], [r0]!
	vld1.f32 d2[1], [r0]!
	vld1.f32 d4[1], [r0]!
	vld1.f32 d3[1], [r0]!

	@ direct load the third column of the ortho inverse result
	vld1.f32 d1[0], [r0]!
	vld1.f32 d3[0], [r0]!

	vst1.f32 {d0-d3}, [r1]!		@ store first eight elements of result (1st two columns)

	vld1.f32 d5[0], [r0]!
	vld1.f32 d5[1], [r0]!

	vst1.f32 {d4-d5}, [r1]!		@ store next four elements of result (3rd column)

	@ move q0 into q8 so we can reuse q0 to load fourth column
	@ of input. We do this to avoid using q4-q7 (which have to
	@ be preserved during the function call)....needed to work
	@ around some limitation rules that prevent us from accessing
	@ single s registers associated with q8 and above.
	vmov.f32 q8, q0

	@ direct load the last column of the input
	vld1.f32 {q0}, [r0]

	@ prepare the last column of the output
	vmul.f32 q3, q8, d0[0]		@ multiply result column 1 by x coord of input column 3
	vmla.f32 q3, q1, d0[1]		@ multiply result column 2 by y coord of input column 3 and add
	vmla.f32 q3, q2, d1[0]		@ multiply result column 3 by z coord of input column 3 and add
	vneg.f32 q3, q3				@ negate final column

	vst1.f32 {q3}, [r1]			@ store last four elements of result (4th column)

	bx	lr
	.fnend

@----------------------------------------------------------------
@ pfxTransform3Vector3MultiplyNEON
@
@ Computes the product of a Transform3 and a Vector3, e.g., it
@ applies the transform to the vector.
@
@ Result stored in memory pointed to by r2. r2 must point to
@ memory sufficient to store 4 32-bit floats.
@----------------------------------------------------------------
	.global	pfxTransform3Vector3MultiplyNEON
	.thumb_func
pfxTransform3Vector3MultiplyNEON:
	.fnstart

	vld1.32 {d16-d19}, [r0]!	@ load first eight elements of transform matrix
	vld1.32 {d20-d21}, [r0]		@ load second eight elements of transform matrix
	vld1.32 {d0-d1}, [r1]		@ load the four elements of vector3 (last one is just padding)
	vmul.f32 q12, q8, d0[0]		@ rslt col0  = (mat0 col0) * (mat1 col0 elt0)
	vmla.f32 q12, q9, d0[1]		@ rslt col0 += (mat0 col1) * (mat1 col0 elt1)
	vmla.f32 q12, q10, d1[0]	@ rslt col0 += (mat0 col2) * (mat1 col0 elt2)
	vst1.32 {d24-d25}, [r2]		@ store four elements of result (last one is padding)

	bx	lr
	.fnend
