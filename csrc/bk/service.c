/*
 * Used to be
 * Copyright 1994, Eric A. Edwards
 * After substantial changes
 * Copyright 2003, Leonid A. Broukhis
 */

/*
 * service.c - Event handler system and interrupt service.
 */


#include "defines.h"

/* 1 if interrupt requested while WAIT was in effect */

unsigned char in_wait_instr = 0;

/* Allows up to 32 different HW interrupts,
 * there can be no more than 1 pending interrupt of each kind.
 */
unsigned long pending_interrupts = 0;
static uint32_t earliest;
event events[NUM_PRI];

/* Deadlines must be less than 2^31 PDP cycles into the future. */
static int cycle_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static int cycle_before(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) < 0;
}

/*
 * ev_init() - Initialize the event system.
 */

int ev_init()
{
	pending_interrupts = 0;
	earliest = 0;
	return OK;
}

/*
 * ev_register() - Register an event.
 */

int
ev_register( priority, handler, delay, info )
unsigned priority;
int (*handler)(d_word info);
unsigned long delay;	/* in PDP cycle ticks */
d_word info;
{
	uint32_t when;

	if (pending_interrupts & (1 << priority)) {
		/* There is one pending already */
		return OK;
	}
	/*
	 * Fill in the info.
	 */

	when = bk_timing_cycles() + (uint32_t)delay;
	events[priority].handler = handler;
	events[priority].info = info;
	events[priority].when = when;
	if (!pending_interrupts || cycle_before(when, earliest)) {
		earliest = when;
	}
	pending_interrupts |= 1 << priority;
	/* fprintf(stderr, "Registering pri %d @ %g\n", priority, earliest); */
	return OK;
}


/*
 * ev_fire() - Fire off any pending event by the mask
 * priority list.
 */

int ev_fire( int priority )
{
	int x;
	uint32_t now = bk_timing_cycles();
	int have_earliest = 0;
	unsigned long mask = 0;
	switch (priority) {
		case 0: mask = ~0; break;
		case 1: mask = ~0; break;
		case 2: mask = ~0; break;
		case 3: mask = ~0; break;
		/* BK-0010 uses MTPS #200 to block keyboard interrupts */
		case 4: mask = 1; break;
		case 5: mask = 0; break;
		case 6: mask = 0; break;
		case 7: mask = 0; break;
	}
	if (!(mask & pending_interrupts) || !cycle_reached(now, earliest)) {
		return OK;
	}

	for( x = 0; x < NUM_PRI && (1UL<<x) <= pending_interrupts; ++x ) {
		if ((mask & pending_interrupts & (1UL<<x)) &&
			cycle_reached(now, events[x].when)) {
			events[x].handler(events[x].info);
			/* fprintf(stderr, "Firing pri %d\\n", x); */
			pending_interrupts &= ~(1UL << x);
			mask = 0;
		} else if (pending_interrupts & (1UL<<x)) {
			if (!have_earliest || cycle_before(events[x].when, earliest)) {
				earliest = events[x].when;
			}
			have_earliest = 1;
		}
	}
	if (!have_earliest) earliest = 0;
	return OK;
}


/*
 * service() - Handle a Trap.
 */

int
service( vector )
d_word vector;
{
	register pdp_regs *p = &pdp;
	int result;
	d_word oldpsw;
	d_word oldpc;
	d_word newpsw;
	d_word newpc;

	last_branch = p->regs[PC];

	oldpsw = p->psw;
	oldpc = p->regs[PC];
	
	/* If we're servicing an interrupt while a WAIT instruction
	 * was executing, we need to return after the WAIT.
	 */
	if (in_wait_instr) {
		oldpc += 2;
		in_wait_instr = 0;
	}

	if (( result = ll_word(p, vector, &newpc)) != OK)
		return result;
	if (( result = ll_word(p, vector + 2, &newpsw)) != OK)
		return result;

	// fprintf(stderr, "Int %o to %06o\n", vector, p->regs[PC]);
	//addtocybuf(-vector);

	if (( result = push( p, oldpsw )) != OK )
		return result;
	if (( result = push( p, oldpc )) != OK )
		return result;

	p->psw = newpsw;
	p->regs[PC] = newpc;
	return OK;
}


/*
 * rti() - Return from Interrupt Instruction.
 */

int rti( p )
register pdp_regs *p;
{
	d_word newpsw;
	d_word newpc;
	int result;

	last_branch = p->regs[PC];
	if (( result = pop( p, &newpc )) != OK )
		return result;
	if (( result = pop( p, &newpsw )) != OK )
		return result;

	p->psw = newpsw;
	p->regs[PC] = newpc;

	return OK;
}


/*
 * rtt() - Return from Interrupt Instruction.
 */

int rtt( p )
register pdp_regs *p;
{
	d_word newpsw;
	d_word newpc;
	int result;

	last_branch = p->regs[PC];
	if (( result = pop( p, &newpc )) != OK )
		return result;
	if (( result = pop( p, &newpsw )) != OK )
		return result;

	p->psw = newpsw;
	p->regs[PC] = newpc;

	return CPU_RTT;
}
