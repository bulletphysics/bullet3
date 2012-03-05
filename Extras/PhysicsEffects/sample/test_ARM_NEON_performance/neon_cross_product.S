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

@ cross product, result stored directly to memory

	.global	CrossProductNeonResultInMemoryAssembly
	.thumb_func
CrossProductNeonResultInMemoryAssembly:
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

@ cross product, result stored directly to memory, version 2

	.global	CrossProductNeonResultInMemoryAssembly2
	.thumb_func
CrossProductNeonResultInMemoryAssembly2:
	.fnstart
	vld1.32 {d18[1]}, [r1]!
	vld1.32 {d19[0]}, [r1]!
	vld1.32 {d18[0]}, [r1]!
	vld1.32 {d19[1]}, [r1]	@  q9 = <z2,x2,y2,w2> = d18,d19
	vld1.32 {d17[0]}, [r0]!
	vld1.32 {d16}, [r0]!
	vld1.32 {d17[1]}, [r0]	@  q8 = <y1,z1,x1,w1> = d16,d17
@ perform first half of cross product using rearranged inputs
	vmul.f32 q10, q8, q9	@ q10 = <y1*z2,z1*x2,x1*y2,w1*w2>
@ rearrange inputs for second half
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

	.global	CrossProductNeonResultInMemoryAssembly3
	.thumb_func
CrossProductNeonResultInMemoryAssembly3:
	.fnstart
	vld1.32 {d18[1]}, [r1]!
	vld1.32 {d19[0]}, [r1]!
	vld1.32 {d18[0]}, [r1]!
	vld1.32 {d19[1]}, [r1]!	@  q9 = <z2,x2,y2,w2> = d18,d19
	vld1.32 {d17[0]}, [r0]!
	vld1.32 {d16}, [r0]!
	vld1.32 {d17[1]}, [r0]!	@  q8 = <y1,z1,x1,w1> = d16,d17
@ perform first half of cross product using rearranged inputs
	vmul.f32 q10, q8, q9	@ q10 = <y1*z2,z1*x2,x1*y2,w1*w2>
	vld1.32 {d18[1]}, [r0]!
	vld1.32 {d19[0]}, [r0]!
	vld1.32 {d18[0]}, [r0]!
	vld1.32 {d19[1]}, [r0]!	@  q9 = <z2,x2,y2,w2> = d18,d19
	vld1.32 {d17[0]}, [r1]!
	vld1.32 {d16}, [r1]!
	vld1.32 {d17[1]}, [r1]!	@  q8 = <y1,z1,x1,w1> = d16,d17
@ perform last half of cross product using rearranged inputs
	vmls.f32 q10, q8, q9	@ q10 = <y1*z2-y2*z1,z1*x2-z2*x1,x1*y2-x2*y1,w1*w2-w2*w1>
@ and store the result
	vst1.32	{q10}, [r2]
	bx	lr
	.fnend

@ cross product, result stored directly to memory, version 3

@	.global	CrossProductNeonResultInMemoryAssembly3
@	.thumb_func
@CrossProductNeonResultInMemoryAssembly3:
@	.fnstart
@	ldr r3, table
@	vld1.32	{d18,d19}, [r1]	@ input <x2,y2,z2,w2> = d18,d19
@	vld1.32	{d16,d17}, [r0]	@ input <x1,y1,z1,w1> = d16,d17
@ rearrange inputs into convenient order
@	vld1.32 {q10}, [r3]!
@	vld1.32 {q11}, [r3]
@	vtbl.8 q12, {d18,d19}, q10
@	vtbl.8 q13, {d16,d17}, q11
@ perform first half of cross product using rearranged inputs
@	vmul.f32 q14, q12, q13
@ rearrange inputs again
@	vtbl.8 q12, {d18,d19}, q11
@	vtbl.8 q13, {d16,d17}, q10
@ perform last half of cross product using rearranged inputs
@	vmls.f32 q14, q12, q13
@ and store the result
@	vst1.32	{q14}, [r2]
@	bx	lr
@	.fnend

@table:
@z2,x2,y2,w2
@	dcb 08,09,10,11,00,01,02,03,04,05,06,07,00,00,00,00
@y1,z1,x1,w1
@	dcb 04,05,06,07,08,09,10,11,00,01,02,03,00,00,00,00
