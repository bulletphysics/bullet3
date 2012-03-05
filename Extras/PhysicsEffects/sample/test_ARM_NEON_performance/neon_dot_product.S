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

@ dot product, result returned in register

	.global	DotProductNeonResultInRegisterAssembly
	.thumb_func
DotProductNeonResultInRegisterAssembly:
	.fnstart
	vld1.32	{d16,d17}, [r0]	@ input <x1,y1,z1,w1>
	vld1.32	{d18,d19}, [r1]	@ input <x2,y2,z2,w2>
	vmul.f32 d14, d16, d18	@ d14=<x1*x2,y1*y2>
	vmla.f32 d14, d17, d19	@ d14=d14+<z1*z2,w1*w2>=<x1*x2+z1*z2,y1*y2+w1*w2>
	vpadd.f32 d14, d14, d14	@ add the two halves of d14 together, result in each lane
	vmov.f32 r0, s28		@ s28 aliases the first lane of d14
	bx	lr
	.fnend

@ dot product, result stored directly to memory

	.global	DotProductNeonResultInMemoryAssembly
	.thumb_func
DotProductNeonResultInMemoryAssembly:
	.fnstart
	vld1.32	{d16,d17}, [r0]	@ input <x1,y1,z1,w1>
	vld1.32	{d18,d19}, [r1]	@ input <x2,y2,z2,w2>
	vmul.f32 d14, d16, d18	@ d14=<x1*x2,y1*y2>
@ non-fused multiple accumulate
	vmla.f32 d14, d17, d19	@ d14=d14+<z1*z2,w1*w2>=<x1*x2+z1*z2,y1*y2+w1*w2>
@ fused multiple accumulate - not recognized by GNU assembler
@@	vfma.f32 {d14}, d17, d19	@ d14=d14+<z1*z2,w1*w2>=<x1*x2+z1*z2,y1*y2+w1*w2>
	vpadd.f32 d14, d14, d14	@ add the two halves of d14 together, same result in each lane
	vst1.32	{d14}, [r2]
	bx	lr
	.fnend

@ dot product, result stored directly to memory, all inputs are contiguous in memory

	.global	DotProductNeonResultInMemoryAssembly2
	.thumb_func
DotProductNeonResultInMemoryAssembly2:
	.fnstart
	vld1.32	{d16-d19}, [r0]	@ input q8=<x1,y1,z1,w1>, q9=<x2,y2,z2,w2>
	vmul.f32 d14, d16, d18	@ d14=<x1*x2,y1*y2>
@ non-fused multiple accumulate
	vmla.f32 d14, d17, d19	@ d14=d14+<z1*z2,w1*w2>=<x1*x2+z1*z2,y1*y2+w1*w2>
@ fused multiple accumulate - not recognized by GNU assembler
@@	vfma.f32 d14, d17, d19	@ d14=d14+<z1*z2,w1*w2>=<x1*x2+z1*z2,y1*y2+w1*w2>
	vpadd.f32 d14, d14, d14	@ add the two halves of d14 together, same result in each lane
	vst1.32	{d14}, [r1]
	bx	lr
	.fnend

@ same as DotProductNeonResultInMemoryAssembly2, but alternate methodology to compute result

	.global	DotProductNeonResultInMemoryAssembly2b
	.thumb_func
DotProductNeonResultInMemoryAssembly2b:
	.fnstart
	vld1.32	{d14-d17}, [r0]	@ input q7=<x1,y1,z1,w1>, q8=<x2,y2,z2,w2>
	vmul.f32 q7, q7, q8		@ q7 = <x1*x2,y1*y2,z1*z2,w1*w2> = d14,d15 = s28,s29,s30,s31
	vpadd.f32 d14,d14,d14	@ add elements of d14 together=<x1*x2+y1*y2,x1*x2+y1*y2>
	vpadd.f32 d15,d15,d15	@ add elements of d14 together=<z1*z2+w1*w2,z1*z2+w1*w2>
	vadd.f32 d14,d14,d15	@ add d14 and d15 together, final result in both lanes
	vst1.32	{d14}, [r1]
	bx	lr
	.fnend
