/****************************************************************************
 * configs/cxd56evb/src/up_cpu_context.S
 *
 *   Copyright (C) 2016 Sony Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.global	up_cpuctxsavewithwfi
	.global	up_cpuctxload
	.global	up_fpuctxsave
	.global	up_fpuctxload

	.syntax		unified
	.thumb
	.file		"up_cpu_context.S"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

	.section .text,"ax"
	.thumb
	.align 1

	.thumb_func
	.type	up_cpuctxsavewithwfi, function
up_cpuctxsavewithwfi:    
	stmia	r0!,{r4-r7}         /* save r4-r7 */
	mov		r1,r8
	mov		r2,r9
	mov		r3,r10
	stmia	r0!,{r1-r3}         /* save r8-r10 */
	mov		r1,r11
	mov		r2,r12
	mov		r3,r14
	stmia	r0!,{r1-r3}         /* save r11-r12,r14 */
	mrs		r1,msp
	mrs		r2,psp
	stmia	r0!,{r1-r2}         /* save MSP, PSP */
		
	mrs		r1,xpsr
	mrs		r2,primask
	mrs		r3,control
	stmia	r0!,{r1-r3}         /* save xPSR, PRIMASK, CONTROL */
		
	mrs		r2,basepri
	mrs		r3,faultmask
	stmia	r0!,{r2-r3}         /* save BASEPRI, FAULTMASK */
	wfi
	bx		lr

	.size	up_cpuctxsavewithwfi, .-up_cpuctxsavewithwfi

	.thumb_func
	.type	up_cpuctxload, function
up_cpuctxload:   
	.extern	g_V7mCtx
	ldr	r0, =g_V7mCtx
	ldmia	r0!,{r4-r7}         /* restore r4-r7 */
	ldmia	r0!,{r1-r3}
	mov		r8,r1
	mov		r9,r2
	mov		r10,r3              /* restore r8-r10 */
	ldmia	r0!,{r1-r3}
	mov		r11,r1
	mov		r12,r2
	mov		r14,r3              /* restore r11-r12,r14 */
		
	ldmia	r0!,{r1-r2}
	msr		msp,r1              /* restore MSP */
	msr		psp,r2              /* restore PSP */
		
	ldmia	r0!,{r1-r3}
	msr		psr_nzcvq,r1        /* restore xPSR */
	msr		primask,r2          /* restore PRIMASK */
	msr		control,r3          /* restore CONTROL */
	ldmia	r0!,{r2-r3}
	msr		basepri,r2          /* restore BASEPRI */
	msr		faultmask,r3        /* restore FAULTMASK */
	isb
	bx		lr

	.size	up_cpuctxload, .-up_cpuctxload

	.type	up_fpuctxsave, function
up_fpuctxsave:   
	add		r0,r0,#17*4
	vmov	r1,s0
	vmov	r2,s1
	vmov	r3,s2
	stmia	r0!,{r1-r3}         /* save s0-s2 */
	vmov	r1,s3
	vmov	r2,s4
	vmov	r3,s5
	stmia	r0!,{r1-r3}         /* save s3-s5 */
	vmov	r1,s6
	vmov	r2,s7
	vmov	r3,s8
	stmia	r0!,{r1-r3}         /* save s6-s8 */
	vmov	r1,s9
	vmov	r2,s10
	vmov	r3,s11
	stmia	r0!,{r1-r3}         /* save s9-s11 */
	vmov	r1,s12
	vmov	r2,s13
	vmov	r3,s14
	stmia	r0!,{r1-r3}         /* save s12-s14 */
	vmov	r1,s15
	vmov	r2,s16
	vmov	r3,s17
	stmia	r0!,{r1-r3}         /* save s15-s17 */
	vmov	r1,s18
	vmov	r2,s19
	vmov	r3,s20
	stmia	r0!,{r1-r3}         /* save s18-s20 */
	vmov	r1,s21
	vmov	r2,s22
	vmov	r3,s23
	stmia	r0!,{r1-r3}         /* save s21-s23 */
	vmov	r1,s24
	vmov	r2,s25
	vmov	r3,s26
	stmia	r0!,{r1-r3}         /* save s24-s26 */
	vmov	r1,s27
	vmov	r2,s28
	vmov	r3,s29
	stmia	r0!,{r1-r3}         /* save s27-s29 */
	vmov	r1,s30
	vmov	r2,s31
	vmrs	r3,fpscr
	stmia	r0!,{r1-r3}         /* save s30-s31,fpscr */
	bx		lr

	.size	up_fpuctxsave, .-up_fpuctxsave

	.type	up_fpuctxload, function
up_fpuctxload:
	add		r0,r0,#17*4
	ldmia	r0!,{r1-r3}
	vmov	s0,r1
	vmov	s1,r2
	vmov	s2,r3               /* restore s0-s2 */
	ldmia	r0!,{r1-r3}
	vmov	s3,r1
	vmov	s4,r2
	vmov	s5,r3               /* restore s3-s5 */
	ldmia	r0!,{r1-r3}
	vmov	s6,r1
	vmov	s7,r2
	vmov	s8,r3               /* restore s6-s8 */
	ldmia	r0!,{r1-r3}
	vmov	s9,r1
	vmov	s10,r2
	vmov	s11,r3              /* restore s9-s11 */
	ldmia	r0!,{r1-r3}
	vmov	s12,r1
	vmov	s13,r2
	vmov	s14,r3              /* restore s12-s14 */
	ldmia	r0!,{r1-r3}
	vmov	s15,r1
	vmov	s16,r2
	vmov	s17,r3              /* restore s15-s17 */
	ldmia	r0!,{r1-r3}
	vmov	s18,r1
	vmov	s19,r2
	vmov	s20,r3              /* restore s18-s20 */
	ldmia	r0!,{r1-r3}
	vmov	s21,r1
	vmov	s22,r2
	vmov	s23,r3              /* restore s21-s23 */
	ldmia	r0!,{r1-r3}
	vmov	s24,r1
	vmov	s25,r2
	vmov	s26,r3              /* restore s24-s26 */
	ldmia	r0!,{r1-r3}
	vmov	s27,r1
	vmov	s28,r2
	vmov	s29,r3              /* restore s27-s29 */
	ldmia	r0!,{r1-r3}
	vmov	s30,r1
	vmov	s31,r2
	vmsr	fpscr,r3            /* restore s30-s31,fpscr */
	bx		lr

	.size	up_fpuctxload, .-up_fpuctxload

	.end
