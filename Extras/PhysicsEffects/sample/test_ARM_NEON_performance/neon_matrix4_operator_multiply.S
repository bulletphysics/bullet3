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
	.syntax unified
	.arch armv7-a
	.fpu neon
	.thumb
	.text
	.align 2

@ matrix4 operator *, result stored directly to memory

	.global	Matrix4OperatorMultiplyNeon
	.thumb_func
Matrix4OperatorMultiplyNeon:
	.fnstart

	vld1.32     {d16-d19}, [r0]!            @ load first eight elements of matrix 0
    vld1.32     {d20-d23}, [r0]             @ load second eight elements of matrix 0
    vld1.32     {d0-d3}, [r1]!              @ load first eight elements of matrix 1
    vld1.32     {d4-d7}, [r1]               @ load second eight elements of matrix 1

    vmul.f32    q12, q8, d0[0]              @ rslt col0  = (mat0 col0) * (mat1 col0 elt0)
    vmul.f32    q13, q8, d2[0]              @ rslt col1  = (mat0 col0) * (mat1 col1 elt0)
    vmul.f32    q14, q8, d4[0]              @ rslt col2  = (mat0 col0) * (mat1 col2 elt0)
    vmul.f32    q15, q8, d6[0]              @ rslt col3  = (mat0 col0) * (mat1 col3 elt0)

    vmla.f32    q12, q9, d0[1]              @ rslt col0 += (mat0 col1) * (mat1 col0 elt1)
    vmla.f32    q13, q9, d2[1]              @ rslt col1 += (mat0 col1) * (mat1 col1 elt1)
    vmla.f32    q14, q9, d4[1]              @ rslt col2 += (mat0 col1) * (mat1 col2 elt1)
    vmla.f32    q15, q9, d6[1]              @ rslt col3 += (mat0 col1) * (mat1 col3 elt1)

    vmla.f32    q12, q10, d1[0]             @ rslt col0 += (mat0 col2) * (mat1 col0 elt2)
    vmla.f32    q13, q10, d3[0]             @ rslt col1 += (mat0 col2) * (mat1 col1 elt2)
    vmla.f32    q14, q10, d5[0]             @ rslt col2 += (mat0 col2) * (mat1 col2 elt2)
    vmla.f32    q15, q10, d7[0]             @ rslt col3 += (mat0 col2) * (mat1 col2 elt2)

    vmla.f32    q12, q11, d1[1]             @ rslt col0 += (mat0 col3) * (mat1 col0 elt3)
    vmla.f32    q13, q11, d3[1]             @ rslt col1 += (mat0 col3) * (mat1 col1 elt3)
    vmla.f32    q14, q11, d5[1]             @ rslt col2 += (mat0 col3) * (mat1 col2 elt3)
    vmla.f32    q15, q11, d7[1]             @ rslt col3 += (mat0 col3) * (mat1 col3 elt3)

    vst1.32     {d24-d27}, [r2]!            @ store first eight elements of result
    vst1.32     {d28-d31}, [r2]             @ store second eight elements of result

	bx	lr
	.fnend
