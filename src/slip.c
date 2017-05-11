#include "slip.h"

#define SLIP_END             0xc0    /* indicates end of packet */
#define SLIP_ESC             0xdb    /* indicates byte stuffing */
#define SLIP_ESC_END         0xdc    /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC         0xdd    /* ESC ESC_ESC means ESC data byte */

void slip_send_packet(uint8_t *p, int len, int (*put)(void *ctx, uint8_t c), void *ctx)
{

	/* send an initial END character to flush out any data that may
	 * have accumulated in the receiver due to line noise
	 */
	put(ctx, SLIP_END);

	/* for each byte in the packet, send the appropriate character
	 * sequence
	 */
	while (len--) {
		switch (*p) {
		/* if it's the same code as an END character, we send a
		 * special two character code so as not to make the
		 * receiver think we sent an END
		 */
		case SLIP_END:
			put(ctx, SLIP_ESC);
			put(ctx, SLIP_ESC_END);
			break;

			/* if it's the same code as an ESC character,
			 * we send a special two character code so as not
			 * to make the receiver think we sent an ESC
			 */
		case SLIP_ESC:
			put(ctx, SLIP_ESC);
			put(ctx, SLIP_ESC_ESC);
			break;
			/* otherwise, we just send the character
			 */
		default:
			put(ctx, *p);
		}
		p++;
	}

	/* tell the receiver that we're done sending the packet
	 */
	put(ctx, SLIP_END);
}

int slip_read_packet(uint8_t *p, const int len, int (*get)(void *ctx, uint8_t *c), void *ctx) 
{
    uint8_t c;
    int received = 0;

    /* sit in a loop reading bytes until we put together
     * a whole packet.
     * Make sure not to copy them into the packet if we
     * run out of room.
     */
    while(0 == get(ctx, &c)) {
        switch (c) {

            /* if it's an END character then we're done with
             * the packet
             */
        case SLIP_END:
            /* a minor optimization: if there is no
             * data in the packet, ignore it. This is
             * meant to avoid bothering IP with all
             * the empty packets generated by the
             * duplicate END characters which are in
             * turn sent to try to detect line noise.
             */
            if(received)
                return received;
        break;

        /* if it's the same code as an ESC character, wait
         * and get another character and then figure out
         * what to store in the packet based on that.
         */
        case SLIP_ESC:
            if(0 != get(ctx, &c))
                return -1;

            /* if "c" is not one of these two, then we
             * have a protocol violation.  The best bet
             * seems to be to leave the byte alone and
             * just stuff it into the packet
             */
            switch (c) {
            case SLIP_ESC_END: c = SLIP_END; break;
            case SLIP_ESC_ESC: c = SLIP_ESC; break;
            }
        default:
            if(received >= len)
                goto __err;
            p[received++] = c;
        break;
        }
    }

__err:
    return -1;
}
