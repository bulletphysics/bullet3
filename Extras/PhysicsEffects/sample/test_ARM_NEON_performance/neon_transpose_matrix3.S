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

@ transpose a matrix3, result stored directly to memory

	.global	TransposeMatrix3Neon
	.thumb_func
TransposeMatrix3Neon:
	.fnstart

@way1
@	vld1.f32 d0[0], [r0]!
@	vld1.f32 d2[0], [r0]!
@	vld1.f32 d4[0], [r0]!
@	vld1.f32 d1[1], [r0]!

@	vld1.f32 d0[1], [r0]!
@	vld1.f32 d2[1], [r0]!
@	vld1.f32 d4[1], [r0]!
@	vld1.f32 d3[1], [r0]!

@	vld1.f32 d1[0], [r0]!
@	vld1.f32 d3[0], [r0]!
@	vld1.f32 d5[0], [r0]!
@	vld1.f32 d5[1], [r0]

@way2
@	vld2.f32 {d0-d1}, [r0]!
@	vld2.f32 {d2-d3}, [r0]!
@	vld2.f32 {d4-d5}, [r0]

@	vzip.f32 q0, q1
@	vswp.f32 d1, d4
@	vswp.f32 d3, d5
@	vzip.f32 d1, d5

@way3 (Fastest)
	vld1.f32 {d0-d3}, [r0]!   @ store first eight elements of result
    vld1.f32 {d4-d5}, [r0]    @ store second eight elements of result

    vzip.f32 d0, d2
	vzip.f32 d1, d3
	vswp.f32 d1, d4
	vzip.f32 d1, d3

	vst1.f32 {d0-d3}, [r1]!   @ store first eight elements of result
    vst1.f32 {d4-d5}, [r1]    @ store second eight elements of result

	bx	lr
	.fnend
